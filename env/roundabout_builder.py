"""env/roundabout_config.py 를 읽어 회전교차로 SUMO 도로 파일들을 생성하는 모듈.

기존 직선 고속도로 빌더(road_builder.py)와 별개로 동작하며,
결과물은 env/sumo_roundabout/ 에 만들어진다.

생성 파일:
    roundabout.nod.xml   : 노드 — 원 위의 진입/진출 지점, 실선/점선 경계점, 도로 끝점
    roundabout.edg.xml   : 엣지 — 원호 구간(정원 좌표로 shape 지정)과 진입/진출로
    roundabout.con.xml   : 연결 — 어느 차로에서 어느 차로로 갈 수 있는지 명시
    roundabout.net.xml   : netconvert로 컴파일한 완성 도로망
    roundabout.rou.xml   : 확인용 배경 교통 (모든 진입로 → 모든 진출로)
    roundabout.deco.xml  : 중앙섬, 분리섬, 노란 빗금 안전지대 (순수 시각 요소)
    roundabout.gui.xml   : GUI 시작 화면 설정
    roundabout.sumocfg   : 위 파일들을 묶는 설정 파일

도형 계산 방식:
    회전부는 반지름 RING["radius"]의 정원이다. 원호를 arc_step_deg 간격의
    점으로 샘플링해 엣지 shape로 넣기 때문에 netconvert가 임의로 휘지 않는다.
    진입/진출로(leg)는 각 방향의 축(axis)과 나란히 뻗고, 축에서 옆으로
    떨어진 거리(lateral, 왼쪽 = 반시계 방향이 +)는 섬 간격으로부터 계산된다.
    원 위의 접속 각도는 이 lateral 값에서 자동으로 결정된다.
"""
import math
import os
import subprocess

from sumolib import checkBinary

from env.mdp_config import SIMULATION

PREFIX = "roundabout"


# ──────────────────────────────────────────────────────────────
# 기하 유틸
# ──────────────────────────────────────────────────────────────
def _unit(deg):
    a = math.radians(deg)
    return math.cos(a), math.sin(a)


def _fmt(pts):
    return " ".join(f"{x:.3f},{y:.3f}" for x, y in pts)


class Leg:
    """진입로 또는 진출로 하나.

    위치는 축 방향 거리 a(원 중심에서 축을 따라 잰 거리)와
    축에서의 옆 거리 lateral(a)로 표현한다.
    """

    def __init__(self, approach, idx, spec, axis, length, speed,
                 lat_ring, lat_far, a_ring, a_far, ring_r):
        self.approach = approach
        self.idx = idx
        self.kind = spec["kind"]
        self.lanes = int(spec["lanes"])
        self.axis = axis
        self.length = length
        self.speed = speed
        self.lat_ring = lat_ring
        self.lat_far = lat_far
        self.a_ring = a_ring        # 섬 시작(원 바깥 가장자리) 축 거리
        self.a_far = a_far          # 섬 끝 축 거리 (이후 lateral 일정)
        self.ring_r = ring_r
        self.width = self.lanes * _lane_width()
        suffix = "out" if self.kind == "exit" else "in"
        self.edge_id = f"{approach}_{suffix}{idx}"
        self.far_node = f"{approach}_end{idx}"
        self.ring_node = None       # 원 위 접속 노드 id (나중에 지정)
        self._update_attach()

    def _update_attach(self):
        # 원 위 접속점: lateral 만큼 옆으로 떨어진 원 위의 점
        self.a0 = math.sqrt(max(self.ring_r ** 2 - self.lat_ring ** 2, 0.0))
        x, y = self.point(self.a0, 0.0)
        self.ring_angle = math.degrees(math.atan2(y, x)) % 360.0

    def shift(self, d):
        """leg 전체를 축에서 옆으로 d 만큼 평행이동 (왼쪽 = 반시계 쪽이 +)."""
        self.lat_ring += d
        self.lat_far += d
        self._update_attach()

    def boundary_angle(self, r, side):
        """leg의 왼쪽(side=+1) 또는 오른쪽(side=-1) 경계가 반지름 r 원과 만나는 각도."""
        lat = self.lat_ring + side * self.width / 2.0
        return self.axis + math.degrees(math.asin(max(-1.0, min(1.0, lat / r))))

    def lateral(self, a):
        if a <= self.a_ring:
            return self.lat_ring
        if a >= self.a_far:
            return self.lat_far
        t = (a - self.a_ring) / (self.a_far - self.a_ring)
        return self.lat_ring + t * (self.lat_far - self.lat_ring)

    def point(self, a, offset):
        """축 거리 a에서, 차로 중심선 기준 왼쪽으로 offset 만큼 떨어진 점."""
        ux, uy = _unit(self.axis)
        nx, ny = -uy, ux
        lat = self.lateral(a) + offset
        return ux * a + nx * lat, uy * a + ny * lat

    def left_edge(self, a):
        return self.point(a, self.width / 2.0)

    def right_edge(self, a):
        return self.point(a, -self.width / 2.0)

    def outward_shape(self, a_from=None, a_to=None):
        """원 → 바깥 방향의 중심선 점들 (축 거리 a_from ~ a_to 구간)."""
        a_from = self.a0 if a_from is None else a_from
        a_to = self.a_ring + self.length if a_to is None else a_to
        stops = [a for a in (self.a_ring, self.a_far) if a_from < a < a_to]
        return [self.point(a, 0.0) for a in [a_from] + stops + [a_to]]

    # 엣지 분할: split_a(축 거리)가 정해지면 leg를 원 쪽(near, edge_id)과
    # 바깥쪽(far, edge_id + "_far") 두 엣지로 나눈다. 중간 노드에서 다른 길이 합류할 때 쓴다.
    split_a = None
    far_node_type = "dead_end"

    @property
    def mid_node(self):
        return f"{self.edge_id}_mid"

    @property
    def far_edge(self):
        return f"{self.edge_id}_far"

    @property
    def outer_edge(self):
        """도로망 가장 바깥쪽 엣지 — 진입로면 차량 출발, 진출로면 도착 지점."""
        return self.far_edge if self.split_a is not None else self.edge_id

    def far_point(self):
        return self.point(self.a_ring + self.length, 0.0)


def _lane_width():
    from env.roundabout_config import LANE_WIDTH
    return float(LANE_WIDTH)


def _make_legs(approaches, ring):
    """config의 APPROACHES로부터 Leg 객체들을 만든다."""
    lw = _lane_width()
    rc = float(ring["radius"])
    r_out = rc + ring["num_lanes"] * lw / 2.0
    legs = []
    by_key = {}
    for name, ap in approaches.items():
        specs = ap["legs"]
        islands = ap.get("islands", [])
        if len(islands) != max(len(specs) - 1, 0):
            raise ValueError(f"{name}: islands 개수는 legs 개수 - 1 이어야 합니다")
        widths = [int(s["lanes"]) * lw for s in specs]
        gaps_ring = [float(i["gap_ring"]) for i in islands]
        gaps_far = [float(i["gap_far"]) for i in islands]
        island_len = max([float(i["island_len"]) for i in islands] or [0.0])

        def stack(gaps):
            # legs를 반시계(왼쪽 = +) 순서로 쌓고, 전체 묶음을 축 중심에 맞춘다
            total = sum(widths) + sum(gaps)
            pos, lat = -total / 2.0, []
            for k, w in enumerate(widths):
                lat.append(pos + w / 2.0)
                pos += w + (gaps[k] if k < len(gaps) else 0.0)
            return lat

        lat_r, lat_f = stack(gaps_ring), stack(gaps_far)
        a_ring = r_out + 1.0
        a_far = a_ring + max(island_len, 1.0)
        for k, spec in enumerate(specs):
            leg = Leg(name, k, spec, float(ap["axis"]), float(ap["length"]),
                      float(ap["speed"]), lat_r[k], lat_f[k], a_ring, a_far, rc)
            legs.append(leg)
            by_key[(name, k)] = leg
    return legs, by_key


def _snap_extra_islands(extra_islands, by_key, ring):
    """서로 다른 방향의 두 leg를 원 위에서 gap_ring 간격으로 붙인다.

    예: 단봉로 출구를 검단로 출구 바로 옆(반시계 반대쪽)에 붙여서,
    두 출구가 원에서 거의 붙어 나온 뒤 축 각도 차이만큼 벌어지게 한다.
    첫 번째 leg(between[0])를 옆으로 평행이동해서 맞춘다.
    """
    r_out = float(ring["radius"]) + ring["num_lanes"] * _lane_width() / 2.0
    for isl in extra_islands:
        if "gap_ring" not in isl:
            continue
        (n1, i1), (n2, i2) = isl["between"]
        mover, fixed = by_key[(n1, i1)], by_key[(n2, i2)]
        gap_deg = math.degrees(float(isl["gap_ring"]) / r_out)
        if mover.ring_angle < fixed.ring_angle:
            # mover가 시계방향 쪽: mover의 왼쪽 경계를 fixed의 오른쪽 경계에 붙임
            target = fixed.boundary_angle(r_out, -1) - gap_deg
            lat = r_out * math.sin(math.radians(target - mover.axis)) - mover.width / 2.0
        else:
            target = fixed.boundary_angle(r_out, +1) + gap_deg
            lat = r_out * math.sin(math.radians(target - mover.axis)) + mover.width / 2.0
        mover.shift(lat - mover.lat_ring)


# ──────────────────────────────────────────────────────────────
# 도로망 (nod / edg / con)
# ──────────────────────────────────────────────────────────────
def _ring_segments(legs, ring):
    """원 위 접속점 사이를 원호 엣지로 나눈다.

    각 접속점 앞뒤 solid_len 미터는 실선(차로 변경 금지) 구간으로 따로 자른다.
    반환: (nodes, segments)
        nodes    : [(node_id, angle_deg)]  반시계 순서
        segments : [(edge_id, from_node, to_node, a_from, a_to, solid)]
    """
    rc = float(ring["radius"])
    solid = float(ring.get("solid_len", 0.0))
    ordered = sorted(legs, key=lambda l: l.ring_angle)
    for k, leg in enumerate(ordered):
        leg.ring_node = f"ring{k}_{leg.edge_id}"

    nodes, segs = [], []
    n = len(ordered)
    for k, leg in enumerate(ordered):
        nxt = ordered[(k + 1) % n]
        a0 = leg.ring_angle
        a1 = nxt.ring_angle if k + 1 < n else nxt.ring_angle + 360.0
        arc = math.radians(a1 - a0) * rc
        nodes.append((leg.ring_node, a0))
        if solid > 0 and arc > 2 * solid + 3.0:
            d = math.degrees(solid / rc)
            na, nb = f"ring{k}_a", f"ring{k}_b"
            nodes.append((na, a0 + d))
            nodes.append((nb, a1 - d))
            segs.append((f"ring{k}_s0", leg.ring_node, na, a0, a0 + d, True))
            segs.append((f"ring{k}_d", na, nb, a0 + d, a1 - d, False))
            segs.append((f"ring{k}_s1", nb, nxt.ring_node, a1 - d, a1, True))
        else:
            # 구간이 짧으면 통째로 실선 (solid_len=0 이면 통째로 점선)
            segs.append((f"ring{k}", leg.ring_node, nxt.ring_node, a0, a1, solid > 0))
    return nodes, segs


def _arc_points(rc, a0, a1, step):
    n = max(int(math.ceil((a1 - a0) / step)), 1)
    pts = []
    for i in range(n + 1):
        x, y = _unit(a0 + (a1 - a0) * i / n)
        pts.append((rc * x, rc * y))
    return pts


def _plan_split_link(cfg, by_key):
    """출구 하나가 원 밖에서 양방향 도로가 되고, 반대 방향 차로가 대각선 길로
    다른 진입로에 합류하는 구조 (단봉로 ↔ 검단로).

        ←← 나가는 2차로  ═══╗
        ━━ 노란 중앙선 ━━━━ ╠═ split ── ←← 단봉로 출구 (원에서 나감)
        →→ 들어오는 2차로 ═══╝   ╲
                                  ╲ →→ 대각선 길 → 검단로 입구에 합류 (join)

    대각선 길은 검단로 출구 차로를 가로지르므로, 검단로 출구/입구의 분할 노드를
    교차점 하나로 합쳐(join) 서로 양보하게 한다.
    """
    ex = by_key[tuple(cfg["exit"])]
    je = by_key[tuple(cfg["join_exit"])]
    ji = by_key[tuple(cfg["join_entry"])]
    a_s, a_j, L = float(cfg["split_dist"]), float(cfg["join_dist"]), float(cfg["far_length"])
    lanes_in = int(cfg.get("lanes_in", ex.lanes))
    w_in = lanes_in * _lane_width()

    # 단봉로 출구는 split 지점에서 끝나고, 그 노드가 갈라지는 교차점이 된다
    ex.length = a_s - ex.a_ring
    ex.far_node = f"{ex.edge_id}_split"
    ex.far_node_type = "priority"
    # 들어오는 차로 중심 = 나가는 차로 중심에서 왼쪽(중앙선 건너편)으로
    off_in = ex.width / 2.0 + w_in / 2.0
    s_in = ex.point(a_s, off_in)
    d_out, d_in = ex.point(a_s + L, 0.0), ex.point(a_s + L, off_in)

    je.split_a = a_j
    ji.split_a = a_j
    diag_end = ji.point(a_j, 0.0)

    # 대각선 길: 들어오는 차로 방향(원 쪽)으로 그대로 이어지다가 큰 곡선으로
    # 부드럽게 꺾여 내려간다 (2차 베지어 — 시작 접선 = 들어오는 차로 방향)
    bend = float(cfg.get("diag_curve", 15.0))
    dx, dy = s_in[0] - d_in[0], s_in[1] - d_in[1]
    Ld = math.hypot(dx, dy) or 1.0
    ctrl = (s_in[0] + dx / Ld * bend, s_in[1] + dy / Ld * bend)
    diag_pts = []
    for k in range(17):
        t = k / 16
        diag_pts.append(((1 - t) ** 2 * s_in[0] + 2 * (1 - t) * t * ctrl[0] + t ** 2 * diag_end[0],
                         (1 - t) ** 2 * s_in[1] + 2 * (1 - t) * t * ctrl[1] + t ** 2 * diag_end[1]))

    straight = float(cfg.get("terminal_straight", 0.0))
    if straight > 0:
        fx, fy = _unit(float(cfg.get("terminal_heading", -80.0)))
        entry = (diag_end[0] - fx * straight, diag_end[1] - fy * straight)
        c1 = (s_in[0] + dx / Ld * bend, s_in[1] + dy / Ld * bend)
        c2 = (entry[0] - fx * 10.0, entry[1] - fy * 10.0)
        diag_pts = []
        for k in range(65):
            t = k / 64.0
            diag_pts.append(tuple((1-t)**3*s_in[j] + 3*(1-t)**2*t*c1[j]
                                  + 3*(1-t)*t*t*c2[j] + t**3*entry[j] for j in (0, 1)))
        diag_pts.extend((entry[0] + fx * straight*k/16,
                         entry[1] + fy * straight*k/16) for k in range(1, 17))

    out_far, in_far, diag = f"{ex.approach}_outfar", f"{ex.approach}_infar", f"{ex.approach}_diag"
    nodes = {f"{ex.approach}_split_in": s_in,
             f"{ex.approach}_end_out": d_out,
             f"{ex.approach}_end_in": d_in}
    edges = [
        (out_far, ex.far_node, f"{ex.approach}_end_out", ex.lanes, ex.speed,
         [ex.point(a_s, 0.0), d_out], 1),
        (in_far, f"{ex.approach}_end_in", f"{ex.approach}_split_in", lanes_in, ex.speed,
         [d_in, s_in], 1),
    ]
    cons = [(ex.edge_id, out_far, ex.lanes),
            (in_far, diag, lanes_in),
            (je.edge_id, je.far_edge, je.lanes),
            (ji.far_edge, ji.edge_id, ji.lanes)]

    right_len = float(cfg.get("right_lane_len", 0.0))
    right_from_upstream = right_len > 0 and cfg.get("right_lane_from_upstream", False)
    diag_end_id = None
    lane_shapes = {}
    if right_len > 0:
        # 대각선 길 끝 right_len 구간: 오른쪽에 차로가 하나 더 생긴다 (우회전 전용).
        #   0번(맨 오른쪽, 새 차로) → 검단로 나가는 방향으로 우회전
        #   1번 이상(기존 차로)    → 검단로 입구(원 쪽)로 좌회전
        lw = _lane_width()
        if right_from_upstream:
            # 기존 두 차로의 위치를 유지하면서 오른쪽 차로를 직선 구간까지 연장한다.
            # 곡선 전체를 먼저 오프셋해야 분할점 양쪽의 차로 중심선도 일치한다.
            original_diag = diag_pts
            diag_pts = []
            for k, (x, y) in enumerate(original_diag):
                (ax, ay) = original_diag[max(k - 1, 0)]
                (bx, by) = original_diag[min(k + 1, len(original_diag) - 1)]
                if k == 0:
                    # 단봉로 직선과 같은 접선으로 시작한다.
                    bx, by = x + dx / Ld, y + dy / Ld
                    ax, ay = x, y
                segment_len = math.hypot(bx - ax, by - ay) or 1.0
                diag_pts.append((x + (by - ay) / segment_len * lw / 2.0,
                                 y - (bx - ax) / segment_len * lw / 2.0))
            shift = (diag_pts[0][0] - s_in[0], diag_pts[0][1] - s_in[1])
            s_in = diag_pts[0]
            d_in = (d_in[0] + shift[0], d_in[1] + shift[1])
            nodes[f"{ex.approach}_split_in"] = s_in
            nodes[f"{ex.approach}_end_in"] = d_in
            edges[1] = (in_far, f"{ex.approach}_end_in", f"{ex.approach}_split_in",
                        lanes_in + 1, ex.speed, [d_in, s_in], 1)
            cons[1] = (in_far, diag, lanes_in + 1)
        cum = [0.0]
        for p, q in zip(diag_pts, diag_pts[1:]):
            cum.append(cum[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
        s_cut = max(cum[-1] - right_len, 1.0)
        i = next(k for k in range(1, len(cum)) if cum[k] >= s_cut)
        t = (s_cut - cum[i - 1]) / ((cum[i] - cum[i - 1]) or 1.0)
        (x0, y0), (x1, y1) = diag_pts[i - 1], diag_pts[i]
        cut = (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
        part_a = diag_pts[:i] + [cut]
        part_b = [cut] + diag_pts[i:]
        # 분리 차로가 벌어지는 곡선을 촘촘하게 샘플링한다.
        dense = [part_b[0]]
        for a, b in zip(part_b, part_b[1:]):
            count = max(1, math.ceil(math.dist(a, b) / 0.3))
            dense.extend((a[0] + (b[0]-a[0])*j/count,
                          a[1] + (b[1]-a[1])*j/count) for j in range(1, count+1))
        part_b = dense
        # 차로가 오른쪽에 붙으므로 중심선을 오른쪽으로 반 차로 밀어야 기존 차로가 이어진다
        shifted = []
        for k, (x, y) in enumerate(part_b):
            (ax, ay), (bx, by) = part_b[max(k - 1, 0)], part_b[min(k + 1, len(part_b) - 1)]
            segment_len = math.hypot(bx - ax, by - ay) or 1.0
            nx, ny = -(by - ay) / segment_len, (bx - ax) / segment_len
            shifted.append((x - nx * lw / 2.0, y - ny * lw / 2.0))
        if right_from_upstream:
            shifted = part_b
        diag_end_id = f"{diag}_end"
        node_cut = f"{diag}_widen"
        nodes[node_cut] = shifted[0]
        edges.append((diag, f"{ex.approach}_split_in", node_cut,
                      lanes_in + int(right_from_upstream), ex.speed, part_a, 0))
        edges.append((diag_end_id, node_cut, ji.mid_node, lanes_in + 1, ex.speed, shifted, 0))
        if right_from_upstream and cfg.get("right_lane_island"):
            # 우회전 차로를 실제로 바깥으로 벌려 안전지대가 차로를 침범하지 않게 한다.
            gap = float(cfg["right_lane_island"].get("lane_gap", 3.2))
            run = [0.0]
            for p, q in zip(shifted, shifted[1:]):
                run.append(run[-1] + math.dist(p, q))
            lane0 = []
            for k, (x, y) in enumerate(shifted):
                a, b = shifted[max(0, k-1)], shifted[min(len(shifted)-1, k+1)]
                seglen = math.dist(a, b) or 1.0
                # 교차로 전에 분리 폭을 확보하고 끝에서는 직선을 유지한다.
                t = min(run[k] / 12.0, 1.0)
                separation = gap * (6*t**5 - 15*t**4 + 10*t**3)
                offset = lanes_in * lw / 2.0 + separation
                lane0.append((x + (b[1]-a[1])/seglen*offset,
                              y - (b[0]-a[0])/seglen*offset))
            lane_shapes[(diag_end_id, 0)] = lane0
        # 기존 차로 i → 새 엣지 i+1, 맨 오른쪽 차로는 새 차로(0번)로도 갈 수 있다
        cons.append((diag, diag_end_id,
                     lanes_in + 1 if right_from_upstream else
                     [(0, 0)] + [(k, k + 1) for k in range(lanes_in)]))
        cons.append((diag_end_id, ji.edge_id,
                     [(k + 1, k) for k in range(min(lanes_in, ji.lanes))]))
        cons.append((diag_end_id, je.far_edge, [(0, 0)]))
    else:
        # 대각선 길은 우선순위를 낮춰 검단로(직진) 차량에 양보하게 한다
        edges.append((diag, f"{ex.approach}_split_in", ji.mid_node, lanes_in, ex.speed,
                      diag_pts, 0))
        cons.append((diag, ji.edge_id, min(lanes_in, ji.lanes)))
    return {
        "diag_end": diag_end_id,
        "right_turn_target": je.far_edge,
        "widen_taper": right_len > 0 and not right_from_upstream,
        "right_island": cfg.get("right_lane_island"),
        "lane_shapes": lane_shapes,
        "nodes": nodes,
        "joins": [[je.mid_node, ji.mid_node]],
        "edges": edges,
        "cons": cons,
        "sources": [(in_far, ex.approach)],
        "sinks": {ex.edge_id: out_far},
        # 노란 중앙선: 나가는 차로의 왼쪽 경계
        "centerline": [ex.point(a_s, ex.width / 2.0), ex.point(a_s + L, ex.width / 2.0)],
        # 세 길(단봉로 출구 / 대각선 길 / 검단로 출구) 사이 공간을 채울 때 쓰는 정보
        "split_a": a_s,
        "diag_center": original_diag if right_from_upstream else diag_pts,
        "diag_width": w_in,
    }


def _split_fill_region(cw, ccw, link, a_start, min_sep=0.0):
    """단봉로 출구(cw) · 대각선 길 · 검단로 출구(ccw)로 둘러싸인 공간의 경계 다각형.

    원 쪽 a_start에서 시작해 단봉로 출구 아래 경계 → 대각선 길 동쪽 경계 →
    검단로 출구 위 경계를 따라 다시 원 쪽으로 돌아온다. (곡선 때문에 오목할 수 있음)

    대각선 길은 갈라진 직후 단봉로 출구에 딱 붙어 가다가 서서히 떨어지므로
    갈라지는 곳이 아주 뾰족하다. min_sep > 0 이면 두 경계 사이가 min_sep 이상
    벌어진 곳부터 시작한다 (화단처럼 안쪽으로 줄일 영역에 사용).
    """
    cux, cuy = _unit(cw.axis)
    cnx, cny = -cuy, cux

    def sep(p):       # 단봉로 출구 아래 경계에서 공간 쪽으로 떨어진 거리
        a = p[0] * cux + p[1] * cuy
        return (p[0] * cnx + p[1] * cny) - (cw.lateral(a) + cw.width / 2.0)

    # 대각선 길의 왼쪽(공간 쪽) 경계 = 중심선을 진행방향 왼쪽으로 반 폭만큼
    c = link["diag_center"]
    hw = link["diag_width"] / 2.0
    left = []
    for i, (x, y) in enumerate(c):
        (x0, y0), (x1, y1) = c[max(i - 1, 0)], c[min(i + 1, len(c) - 1)]
        L = math.hypot(x1 - x0, y1 - y0) or 1.0
        left.append((x - (y1 - y0) / L * hw, y + (x1 - x0) / L * hw))

    # 대각선 경계가 검단로 출구 차로에 닿는 지점까지만
    ux, uy = _unit(ccw.axis)
    nx, ny = -uy, ux

    def depth(p):     # > 0 이면 검단로 출구 차로 안쪽으로 들어간 것
        a = p[0] * ux + p[1] * uy
        lat = p[0] * nx + p[1] * ny
        return lat - (ccw.lateral(a) - ccw.width / 2.0)

    # 뾰족한 시작부 건너뛰기
    k0 = 0
    if min_sep > 0:
        while k0 < len(left) - 1 and sep(left[k0]) < min_sep:
            k0 += 1
    left = left[k0:]
    a_top = left[0][0] * cux + left[0][1] * cuy       # 윗변이 끝나는 축 거리
    top = [cw.point(a_start, cw.width / 2.0), cw.point(a_top, cw.width / 2.0)]

    side = [left[0]]
    a_hit = ccw.a_far
    for p, q in zip(left, left[1:]):
        dp, dq = depth(p), depth(q)
        if dq >= 0 > dp:
            t = -dp / (dq - dp)
            hit = (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
            side.append(hit)
            a_hit = hit[0] * ux + hit[1] * uy
            break
        side.append(q)
    bottom = [ccw.point(a, -ccw.width / 2.0)
              for a in sorted({a_hit, a_start} | ({ccw.a_far} if a_start < ccw.a_far < a_hit else set()),
                              reverse=True)]
    # min_sep으로 잘랐으면 윗변 끝과 대각선 경계 시작 사이를 잇는다
    return (top if k0 > 0 else top[:-1]) + side + bottom[1:]


def _plan_uturns(config, approaches, by_key, link):
    """Split only the straight legs and connect their inner lanes below an island."""
    for item in config:
        name, index = item["approach"], int(item.get("island", 0))
        outward, inward = by_key[(name, index)], by_key[(name, index+1)]
        island = approaches[name]["islands"][index]
        gap = abs(inward.lat_far-outward.lat_far)-(inward.width+outward.width)/2
        radius = (gap+_lane_width())/2
        island_end = outward.a_ring+float(island["island_len"])-float(island.get("toward_ring", 0))
        station = island_end+radius+_lane_width()/2+float(item.get("clearance", 0.25))
        outward.split_a = inward.split_a = station
        link["joins"].append([outward.mid_node, inward.mid_node])
        link["cons"].extend([(outward.edge_id, outward.far_edge, outward.lanes),
                             (inward.far_edge, inward.edge_id, inward.lanes),
                             (inward.far_edge, outward.far_edge,
                              [(inward.lanes-1, outward.lanes-1)])])
        # 유턴 차는 반대편(나가는 방향) 차에 양보한다. SUMO 기본 대기 지점은 유턴 차의
        # 차체가 아직 들어오는 차로에 걸친 곳이라, 같은 차로 직진 뒤차가 지나가며 겹친다.
        #   - 정지선에서 기다리게 하면: 출발 후 나가는 두 차로를 천천히 한 번에 막아
        #     나가는 차가 원 안까지 밀려 원 전체가 막힌다.
        #   - 그래서 대기 지점을 "들어오는 차로를 완전히 벗어난 뒤, 나가는 차로 직전"으로
        #     옮긴다. 위치는 build()에서 실제 경로를 재서 정한다(contPos).
        link.setdefault("uturn_wait", {})[(inward.far_edge, outward.far_edge)] = {
            "lane": str(inward.lanes - 1), "straight": inward.edge_id,
            "out_from": outward.edge_id, "out_lanes": outward.lanes, "pos": None,
            "at_entry": bool(item.get("wait_at_entry", False))}


def _movement_path(net_path, fr, to, lane):
    """fr의 lane 차로에서 to로 가는 교차 영역 안 경로 (내부 차로들을 이은 점 목록)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(net_path).getroot()
    shapes = {l.get("id"): [tuple(map(float, p.split(","))) for p in l.get("shape").split()]
              for e in root.iter("edge") for l in e.iter("lane")}
    follow = {(c.get("from"), c.get("fromLane")): c for c in root.iter("connection")
              if c.get("from").startswith(":")}
    c = next(c for c in root.iter("connection")
             if c.get("from") == fr and c.get("to") == to and c.get("fromLane") == str(lane))
    pts, cur = [], c.get("via")
    while cur:
        pts += shapes[cur][1 if pts else 0:]
        e, i = cur.rsplit("_", 1)
        cur = follow[(e, i)].get("via")
    return pts


def _path_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _uturn_wait_point(net_path, key, w, veh_width=1.8, margin=0.5, veh_len=5.0, min_gap=2.5):
    """유턴 경로에서 기다릴 위치 (경로 시작부터 앞범퍼까지 거리, m).

    조건 세 가지:
      1) 기다리는 차의 꼬리까지 들어오는 직진 경로를 벗어나 있다 → 직진 차와 안 겹침
      2) 나가는 차로에 닿기 전이다 → 나가는 차를 막지 않음
      3) 그 뒤에 유턴 차 한 대가 통째로 들어갈 자리가 없다 → 두 번째 유턴 차는
         꼬리가 들어오는 차로에 걸친 채 멈추고, SUMO가 그 차를 같은 차로 직진 뒤차의
         앞차로 인식해 뒤차가 기다린다 (유턴이 끝날 때까지 뒤차 대기)
    """
    if w.get("at_entry"):
        # Narrow medians cannot hold a complete vehicle between directions.
        # Wait before entering the U-turn instead of blocking the outgoing road.
        return 0.0
    from shapely.geometry import LineString
    shapes = _read_lane_shapes(net_path)
    u = LineString(_movement_path(net_path, key[0], key[1], w["lane"]))
    # 직진 차는 교차 영역을 지나 계속 간다: 교차 영역 안 경로 + 이어지는 차로 전체
    straight = LineString(_movement_path(net_path, key[0], w["straight"], w["lane"])
                          + shapes[(w["straight"], int(w["lane"]))])
    # 나가는 차도 교차 영역 앞뒤 차로까지 포함
    outs = [LineString(shapes[(w["out_from"], k)]
                       + _movement_path(net_path, w["out_from"], key[1], str(k))
                       + shapes[(key[1], k)])
            for k in range(int(w["out_lanes"]))]
    reach = _lane_width() / 2 + veh_width / 2 + 0.2
    clear_in = enter_out = None
    for k in range(int(u.length * 10)):
        s = k / 10
        p = u.interpolate(s)
        if clear_in is None and p.distance(straight) > reach:
            clear_in = s
        if enter_out is None and min(p.distance(o) for o in outs) < reach:
            enter_out = s
    if clear_in is None or enter_out is None:
        raise ValueError(f"유턴 {key}: 유턴 경로가 직진 경로나 나가는 차로와 떨어지지 않습니다")
    # 꼬리(앞범퍼에서 veh_len 뒤)가 clear_in을 막 지난 위치 (여유 0.3 m)
    wait = clear_in + veh_len + 0.3
    if wait > enter_out - margin:
        raise ValueError(f"유턴 {key}: 들어오는 차로와 나가는 차로 사이에 기다릴 공간이 없습니다")
    if wait - veh_len - min_gap >= veh_len:
        raise ValueError(f"유턴 {key}: 대기 차 뒤에 유턴 차 한 대가 더 들어갈 수 있습니다")
    return wait



def _write_network(out_dir, ring, legs, link=None):
    lw = _lane_width()
    rc = float(ring["radius"])
    nl = int(ring["num_lanes"])
    nodes, segs = _ring_segments(legs, ring)

    # ── 노드 ──
    lines = ["<nodes>"]
    for nid, ang in nodes:
        x, y = _unit(ang)
        lines.append(f'    <node id="{nid}" x="{rc * x:.3f}" y="{rc * y:.3f}" type="priority"/>')
    for leg in legs:
        x, y = leg.far_point()
        lines.append(f'    <node id="{leg.far_node}" x="{x:.3f}" y="{y:.3f}" type="{leg.far_node_type}"/>')
        if leg.split_a is not None:
            x, y = leg.point(leg.split_a, 0.0)
            lines.append(f'    <node id="{leg.mid_node}" x="{x:.3f}" y="{y:.3f}" type="priority"/>')
    if link:
        for nid, (x, y) in link["nodes"].items():
            lines.append(f'    <node id="{nid}" x="{x:.3f}" y="{y:.3f}" type="priority"/>')
        # 대각선 길이 검단로 출구를 가로질러 입구에 합류하는 곳: 출구/입구의 분할 노드를
        # 교차점 하나로 합쳐야 SUMO가 서로 엇갈리는 차량끼리 양보를 계산한다.
        for group in link["joins"]:
            lines.append(f'    <join nodes="{" ".join(group)}"/>')
    lines.append("</nodes>")
    with open(os.path.join(out_dir, f"{PREFIX}.nod.xml"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # ── 엣지 ──
    # spreadType="center": shape가 엣지의 "가운데 선"이 되도록 한다.
    # (기본값 right는 shape를 왼쪽 경계로 쓰기 때문에 원이 바깥으로 밀린다)
    # 실선 구간: 0번(바깥) 차로는 왼쪽 변경, 1번(안쪽) 차로는 오른쪽 변경 금지.
    #   changeLeft/changeRight = 변경을 허용할 차종 목록. "emergency authority"(긴급차량만)로
    #   두면 일반 차량은 변경 불가.
    lines = ["<edges>"]
    step = float(ring.get("arc_step_deg", 3.0))
    for eid, fr, to, a0, a1, solid in segs:
        shape = _fmt(_arc_points(rc, a0, a1, step))
        attrs = (f'id="{eid}" from="{fr}" to="{to}" numLanes="{nl}" '
                 f'speed="{ring["speed_limit"]}" priority="10" width="{lw}" '
                 f'spreadType="center" shape="{shape}"')
        if solid and nl > 1:
            lines.append(f"    <edge {attrs}>")
            for i in range(nl):
                ch = []
                if i < nl - 1:
                    ch.append('changeLeft="emergency authority"')
                if i > 0:
                    ch.append('changeRight="emergency authority"')
                lines.append(f'        <lane index="{i}" {" ".join(ch)}/>')
            lines.append("    </edge>")
        else:
            lines.append(f"    <edge {attrs}/>")
    def leg_edge(eid, fr, to, lanes, speed, shape, priority=1):
        lines.append(f'    <edge id="{eid}" from="{fr}" to="{to}" '
                     f'numLanes="{lanes}" speed="{speed}" priority="{priority}" '
                     f'width="{lw}" spreadType="center" shape="{_fmt(shape)}"/>')

    for leg in legs:
        if leg.split_a is None:
            pieces = [(leg.edge_id, leg.ring_node, leg.far_node, leg.outward_shape())]
        else:
            pieces = [(leg.edge_id, leg.ring_node, leg.mid_node,
                       leg.outward_shape(None, leg.split_a)),
                      (leg.far_edge, leg.mid_node, leg.far_node,
                       leg.outward_shape(leg.split_a, None))]
        for eid, fr, to, shape in pieces:
            if leg.kind == "entry":       # 진입로는 바깥 → 원 방향
                fr, to, shape = to, fr, shape[::-1]
            leg_edge(eid, fr, to, leg.lanes, leg.speed, shape)
    if link:
        for eid, fr, to, lanes, speed, shape, prio in link["edges"]:
            leg_edge(eid, fr, to, lanes, speed, shape, prio)
            custom = [(idx, pts) for (edge_id, idx), pts in link.get("lane_shapes", {}).items()
                      if edge_id == eid]
            if custom:
                lines[-1] = lines[-1][:-2] + ">"
                for idx, pts in custom:
                    lines.append(f'        <lane index="{idx}" shape="{_fmt(pts)}"/>')
                lines.append('    </edge>')
    ring_edges = " ".join(s[0] for s in segs)
    ring_nodes = " ".join(n[0] for n in nodes)
    lines.append(f'    <roundabout nodes="{ring_nodes}" edges="{ring_edges}"/>')
    lines.append("</edges>")
    with open(os.path.join(out_dir, f"{PREFIX}.edg.xml"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # ── 연결 ──
    # 회전부 통과: 차로 i → 차로 i
    # 진출: 회전부 차로 i → 진출로 차로 i (진출로가 1차로면 바깥 차로만)
    # 진입: 진입로 차로 i → 회전부 차로 i (1차로 진입로는 바깥 차로로 합류)
    into = {s[2]: s[0] for s in segs}     # 노드로 들어오는 원호 엣지
    outof = {s[1]: s[0] for s in segs}    # 노드에서 나가는 원호 엣지
    # 안쪽→바깥쪽 연결(inner_to_outer): 원 위 각 노드에서 안쪽 차로가 다음 구간의
    #   바로 바깥 차로로도 이어지게 한다. 회전부 일반 구간이 아주 짧아(0.2~0.5 m)
    #   차로 변경할 자리가 거의 없어서, 출구로 나가려는 안쪽 차량과 안쪽으로 들어가려는
    #   바깥 차량이 서로 비켜주길 기다리며 멈추는 교착이 생긴다 (SUMO FAQ 권장 해결책).
    inner_to_outer = bool(ring.get("inner_to_outer", False))
    lines = ["<connections>"]
    for nid, _ in nodes:
        e_in, e_out = into[nid], outof[nid]
        for i in range(nl):
            lines.append(f'    <connection from="{e_in}" to="{e_out}" fromLane="{i}" toLane="{i}"/>')
        if inner_to_outer:
            for i in range(1, nl):
                lines.append(f'    <connection from="{e_in}" to="{e_out}" fromLane="{i}" toLane="{i - 1}"/>')
    for leg in legs:
        e_in, e_out = into[leg.ring_node], outof[leg.ring_node]
        if leg.kind == "exit":
            for i in range(min(nl, leg.lanes)):
                lines.append(f'    <connection from="{e_in}" to="{leg.edge_id}" fromLane="{i}" toLane="{i}"/>')
        else:
            for i in range(min(nl, leg.lanes)):
                lines.append(f'    <connection from="{leg.edge_id}" to="{e_out}" fromLane="{i}" toLane="{i}"/>')
    # 분할된 leg와 합류로: 숫자면 차로 i → 차로 i, 목록이면 (from차로, to차로) 쌍
    for fr, to, lanes in (link["cons"] if link else []):
        pairs = [(i, i) for i in range(lanes)] if isinstance(lanes, int) else lanes
        for fl, tl in pairs:
            # contPos: 교차 영역 안 대기 지점 위치 (netconvert가 만든 경로 기준 거리)
            pos = (link.get("uturn_wait", {}).get((fr, to)) or {}).get("pos") if link else None
            wait = f' contPos="{pos:.2f}"' if pos is not None else ""
            lines.append(f'    <connection from="{fr}" to="{to}" fromLane="{fl}" toLane="{tl}"{wait}/>')
    lines.append("</connections>")
    with open(os.path.join(out_dir, f"{PREFIX}.con.xml"), "w") as f:
        f.write("\n".join(lines) + "\n")


# ──────────────────────────────────────────────────────────────
# 확인용 교통
# ──────────────────────────────────────────────────────────────
def _write_routes(out_dir, legs, traffic, link=None, uturn_approaches=()):
    # (출발 엣지, 방향) — 진입로 + 합류로(예: 단봉로 들어오는 방향)
    sources = [(l.outer_edge, l.approach) for l in legs if l.kind == "entry"]
    sources += list(link["sources"]) if link else []
    exits = [l for l in legs if l.kind == "exit"]
    vph = float(traffic["vehs_per_hour_per_entry"])
    # 유턴: 섬 뒤 유턴 연결(UTURNS)이 있는 방향에서만, 원에 들어가기 전에
    #   그 연결로 돌아 들어온 방향으로 되돌아 나가는 차량 비율.
    #   (원을 한 바퀴 돌아 나가는 차량은 만들지 않는다)
    uturn_share = float(traffic.get("uturn_share", 0.0))
    # 출발 방식: "uniform" = 일정 간격(모든 흐름이 같은 박자로 동시에 출발),
    #            "random"  = 매초 확률로 출발(실제 도로처럼 들쭉날쭉한 도착)
    random_dep = traffic.get("departure", "uniform") == "random"
    lc = ""
    if "lc_cooperative" in traffic:
        # 협조 차로 변경(남에게 자리 비켜주기) 강도. 낮추면 회전부에서 서로 비켜주려다
        # 둘 다 멈추는 상호 정체가 줄어든다.
        lc = f' lcCooperative="{traffic["lc_cooperative"]}"'
    lines = ["<routes>",
             f'    <vType id="car" carFollowModel="IDM" accel="2.0" decel="4.0" '
             f'maxSpeed="{traffic["max_speed"]}" speedDev="0.1"{lc} '
             f'vClass="passenger" guiShape="passenger/sedan" color="0,0.8,1"/>',
             # 유턴 차량은 눈에 띄게 주황색. 차량 모델은 Krauss: IDM은 멈출 자리(유턴 경로 안
             # 대기 지점)에 아주 천천히 다가가서, 들어오는 차로에 차체를 걸친 채 기어가는 동안
             # 직진 뒤차와 겹친다. Krauss는 대기 지점까지 바로 들어가 선다.
             f'    <vType id="car_uturn" carFollowModel="Krauss" sigma="0" accel="2.0" decel="4.0" '
             f'maxSpeed="{traffic["max_speed"]}" speedDev="0.1"{lc} '
             f'vClass="passenger" guiShape="passenger/sedan" color="1,0.55,0"/>']
    for src, approach in sources:
        dests = [ex for ex in exits if ex.approach != approach]
        uturns = ([ex for ex in exits if ex.approach == approach]
                  if uturn_share > 0 and approach in uturn_approaches else [])
        plan = [(ex, vph * (1.0 - uturn_share) / len(dests), "car") for ex in dests]
        plan += [(ex, vph * uturn_share / len(uturns), "car_uturn") for ex in uturns]
        if not uturns:      # 되돌아갈 출구가 없는 방향이면 유턴 몫을 나머지에 나눈다
            plan = [(ex, vph / len(dests), "car") for ex in dests]
        for ex, rate, vtype in plan:
            sink = link["sinks"].get(ex.edge_id, ex.outer_edge) if link else ex.outer_edge
            amount = (f'probability="{rate / 3600.0:.5f}"' if random_dep
                      else f'vehsPerHour="{rate:.1f}"')
            lines.append(f'    <flow id="{src}_to_{sink}" type="{vtype}" '
                         f'from="{src}" to="{sink}" begin="0" end="100000" '
                         f'{amount} departLane="best" departSpeed="max"/>')
    lines.append("</routes>")
    with open(os.path.join(out_dir, f"{PREFIX}.rou.xml"), "w") as f:
        f.write("\n".join(lines) + "\n")


# ──────────────────────────────────────────────────────────────
# 시각 요소 (중앙섬 / 분리섬 / 노란 빗금)
# ──────────────────────────────────────────────────────────────
YELLOW = "235,190,40"
CURB = "200,200,195"
GRASS = "80,135,62"
SIDEWALK = "190,186,178"
APRON = "165,150,135"
FLOWER = "214,92,128"
GROUND = "112,140,96"


def _circle(r, n=96):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def _centroid(pts):
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def _inset(pts, d):
    """볼록 다각형을 각 변에서 d 만큼 안쪽으로 줄인다."""
    n = len(pts)
    cx, cy = _centroid(pts)
    lines_ = []
    for i in range(n):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        L = math.hypot(ex, ey) or 1.0
        nx, ny = -ey / L, ex / L
        if (cx - x1) * nx + (cy - y1) * ny < 0:   # 법선을 안쪽으로
            nx, ny = -nx, -ny
        lines_.append(((x1 + nx * d, y1 + ny * d), (ex, ey)))
    out = []
    for i in range(n):
        (p, r), (q, s) = lines_[i - 1], lines_[i]
        den = r[0] * s[1] - r[1] * s[0]
        if abs(den) < 1e-9:
            out.append(q)
            continue
        t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den
        out.append((p[0] + t * r[0], p[1] + t * r[1]))
    return out


def _clip_segment(p, q, poly):
    """선분 p-q를 볼록 다각형 poly 안쪽으로 자른다 (Cyrus-Beck)."""
    cx, cy = _centroid(poly)
    t0, t1 = 0.0, 1.0
    dx, dy = q[0] - p[0], q[1] - p[1]
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        nx, ny = -(y2 - y1), x2 - x1
        if (cx - x1) * nx + (cy - y1) * ny < 0:
            nx, ny = -nx, -ny
        num = (p[0] - x1) * nx + (p[1] - y1) * ny
        den = dx * nx + dy * ny
        if abs(den) < 1e-12:
            if num < 0:
                return None
            continue
        t = -num / den
        if den > 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return None
    return ((p[0] + t0 * dx, p[1] + t0 * dy), (p[0] + t1 * dx, p[1] + t1 * dy))


HATCH_PATTERN = "chevron"


def _hatch(lines, pid, poly, axis_deg, spacing=1.6, layer=3, pattern=None):
    """노란 빗금 안전지대: 노란 테두리 + 줄무늬.

    chevron  : 섬의 축(axis_deg) 위에 꼭짓점을 둔 V자 무늬 (갈매기 무늬)
    diagonal : 축에 45° 기운 사선 줄무늬
    pattern을 주지 않으면 MARKINGS["hatch_pattern"] 기본값을 쓴다.
    """
    pattern = pattern or HATCH_PATTERN
    closed = poly + [poly[0]]
    lines.append(f'    <poly id="{pid}_edge" color="{YELLOW}" fill="0" lineWidth="0.25" '
                 f'layer="{layer}" shape="{_fmt(closed)}"/>')
    cx, cy = _centroid(poly)
    span = max(math.hypot(x - cx, y - cy) for x, y in poly) + 1.0
    segs = []
    if pattern == "chevron":
        ux, uy = _unit(axis_deg)
        nx, ny = -uy, ux
        c = s = math.sqrt(0.5)
        k = -span
        while k <= span:
            ax_, ay_ = cx + ux * k, cy + uy * k            # V의 꼭짓점 (축 위)
            for sign in (1.0, -1.0):
                dx, dy = -ux * c + sign * nx * s, -uy * c + sign * ny * s
                segs.append(((ax_, ay_), (ax_ + dx * 2 * span, ay_ + dy * 2 * span)))
            k += spacing
    else:
        ux, uy = _unit(axis_deg + 45.0)
        nx, ny = -uy, ux
        k = -span
        while k <= span:
            ox, oy = cx + nx * k, cy + ny * k
            segs.append(((ox - ux * span, oy - uy * span), (ox + ux * span, oy + uy * span)))
            k += spacing
    i = 0
    for p, q in segs:
        for seg in _clip_segment_any(p, q, poly):
            if math.hypot(seg[1][0] - seg[0][0], seg[1][1] - seg[0][1]) > 0.3:
                lines.append(f'    <poly id="{pid}_h{i}" color="{YELLOW}" fill="0" lineWidth="0.25" '
                             f'layer="{layer}" shape="{_fmt(list(seg))}"/>')
                i += 1


def _clip_segment_any(p, q, poly):
    """선분 p-q 중 다각형 poly(오목해도 됨) 안에 들어가는 조각들."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    ts = []
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - p[0]) * ey - (y1 - p[1]) * ex) / den
        s = ((x1 - p[0]) * dy - (y1 - p[1]) * dx) / den
        if 0.0 <= s < 1.0 and 0.0 <= t <= 1.0:
            ts.append(t)
    ts.sort()
    # V자 무늬의 팔은 꼭짓점(다각형 안쪽)에서 출발하므로, 시작점이 안쪽이면 t=0부터 안
    inside = False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > p[1]) != (y2 > p[1]) and \
                p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    if inside:
        ts.insert(0, 0.0)
    if len(ts) % 2:
        ts.append(1.0)
    out = []
    for a, b in zip(ts[0::2], ts[1::2]):     # 경계를 넘을 때마다 안/밖이 바뀐다
        out.append(((p[0] + a * dx, p[1] + a * dy), (p[0] + b * dx, p[1] + b * dy)))
    return out


def _clean_polygon(pts, min_len=0.3):
    """너무 가까운 점과, 되돌아가는(스파이크) 꼭짓점을 제거한다."""
    out = list(pts)
    changed = True
    while changed and len(out) > 3:
        changed = False
        for i in range(len(out)):
            p0, p1, p2 = out[i - 1], out[i], out[(i + 1) % len(out)]
            a = (p1[0] - p0[0], p1[1] - p0[1])
            b = (p2[0] - p1[0], p2[1] - p1[1])
            la, lb = math.hypot(*a), math.hypot(*b)
            if la < min_len or (la > 0 and lb > 0 and
                                (a[0] * b[0] + a[1] * b[1]) / (la * lb) < -0.9):
                del out[i]
                changed = True
                break
    return out


def _offset_polygon(pts, d):
    """다각형(오목해도 됨)을 각 변에서 d 만큼 안쪽으로 줄인다 (꼭짓점은 마이터 결합)."""
    n = len(pts)
    area = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
               for i in range(n))
    sgn = 1.0 if area > 0 else -1.0          # 반시계 다각형이면 안쪽 = 진행방향 왼쪽
    offs = []
    for i in range(n):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % n]
        L = math.hypot(x2 - x1, y2 - y1) or 1.0
        nx, ny = -(y2 - y1) / L * sgn, (x2 - x1) / L * sgn
        offs.append(((x1 + nx * d, y1 + ny * d), (x2 - x1, y2 - y1)))
    out = []
    for i in range(n):
        (p, r), (q, s) = offs[i - 1], offs[i]
        den = r[0] * s[1] - r[1] * s[0]
        if abs(den) < 1e-9:
            out.append(q)
            continue
        t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den
        m = (p[0] + t * r[0], p[1] + t * r[1])
        # 아주 뾰족한 꼭짓점은 마이터가 멀리 튀어나가므로 잘라낸다(bevel)
        if math.hypot(m[0] - pts[i][0], m[1] - pts[i][1]) > 3.0 * abs(d):
            Lp = math.hypot(*r) or 1.0
            out.append((p[0] + r[0], p[1] + r[1]))     # 앞 변의 끝
            out.append(q)                               # 다음 변의 시작
        else:
            out.append(m)
    return out


# ──────────────────────────────────────────────────────────────
# 노면 표시 (원 안 점선 / 양보 표시 / 화살표)
# 위치는 netconvert가 만든 실제 차로 shape(net.xml)에서 읽는다.
# ──────────────────────────────────────────────────────────────
WHITE = "255,255,255"
ASPHALT = "84,84,90"


def _read_lane_shapes(net_path):
    import xml.etree.ElementTree as ET
    shapes = {}
    for e in ET.parse(net_path).getroot().iter("edge"):
        if e.get("function") == "internal":
            continue
        for l in e.iter("lane"):
            pts = [tuple(map(float, p.split(","))) for p in l.get("shape").split()]
            shapes[(e.get("id"), int(l.get("index")))] = pts
    return shapes


def _point_from_end(shape, d):
    """차로 끝에서 거꾸로 d 미터 지점의 좌표와 진행 방향 단위벡터."""
    rem = d
    for i in range(len(shape) - 1, 0, -1):
        (x0, y0), (x1, y1) = shape[i - 1], shape[i]
        L = math.hypot(x1 - x0, y1 - y0)
        if L <= 1e-9:
            continue
        fx, fy = (x1 - x0) / L, (y1 - y0) / L
        if rem <= L or i == 1:
            t = min(rem, L)
            return (x1 - fx * t, y1 - fy * t), (fx, fy)
        rem -= L
    return shape[0], (1.0, 0.0)


def _local(origin, f, pts):
    """차로 기준 지역좌표(x=진행방향, y=왼쪽) → 전역좌표."""
    (ox, oy), (fx, fy) = origin, f
    lx, ly = -fy, fx
    return [(ox + fx * x + lx * y, oy + fy * x + ly * y) for x, y in pts]


def _ring_pavement(lines, r_in, r_out):
    """회전부 아래에 둥근 아스팔트를 한 겹 깐다 (layer -1 = 도로망 아래).

    원 위 교차점(junction)들이 맞닿는 곳에 생기는 미세한 틈을 메워서
    회전부 바깥 가장자리가 매끈한 원으로 보이게 한다.
    """
    # SUMO polygons cannot represent a hole reliably. Separate annular sectors
    # preserve a complete circle without a bridged/self-touching fill polygon.
    for k in range(360):
        a, b = math.radians(k), math.radians(k+1)
        sector = [(r_out*math.cos(a), r_out*math.sin(a)),
                  (r_out*math.cos(b), r_out*math.sin(b)),
                  (r_in*math.cos(b), r_in*math.sin(b)),
                  (r_in*math.cos(a), r_in*math.sin(a))]
        lines.append(f'    <poly id="ring_pavement{k}" color="{ASPHALT}" fill="1" layer="1" '
                     f'shape="{_fmt(sector)}"/>')


def _lane_underlay(lines, shapes, extra=0.0, skip_edges=()):
    """모든 진입/진출로 차로 아래에 조금 넓은 아스팔트를 깐다 (layer -1 = 도로망 아래).

    SUMO는 차로를 차로별로 따로 그려서, 곡선 구간 차로 사이나 차로 끝과
    교차점 사이에 가는 틈이 생겨 바닥(초록)이 비쳐 보인다. 이를 메운다.
    """
    k = 0
    for (eid, idx), sh in shapes.items():
        if eid.startswith("ring") or eid in skip_edges or len(sh) < 2:
            continue
        hw = _lane_width() / 2.0 + extra
        left, right = [], []
        for i, (x, y) in enumerate(sh):
            (x0, y0), (x1, y1) = sh[max(i - 1, 0)], sh[min(i + 1, len(sh) - 1)]
            L = math.hypot(x1 - x0, y1 - y0) or 1.0
            nx, ny = -(y1 - y0) / L, (x1 - x0) / L
            left.append((x + nx * hw, y + ny * hw))
            right.append((x - nx * hw, y - ny * hw))
        # 차로 끝도 조금 늘려 교차점과의 틈을 덮는다
        (ax, ay), (bx, by) = sh[0], sh[1]
        La = math.hypot(bx - ax, by - ay) or 1.0
        (cx, cy), (dx, dy) = sh[-2], sh[-1]
        Lb = math.hypot(dx - cx, dy - cy) or 1.0
        ext = 0.0
        s0 = (-(bx - ax) / La * ext, -(by - ay) / La * ext)
        s1 = ((dx - cx) / Lb * ext, (dy - cy) / Lb * ext)
        left[0] = (left[0][0] + s0[0], left[0][1] + s0[1])
        right[0] = (right[0][0] + s0[0], right[0][1] + s0[1])
        left[-1] = (left[-1][0] + s1[0], left[-1][1] + s1[1])
        right[-1] = (right[-1][0] + s1[0], right[-1][1] + s1[1])
        lines.append(f'    <poly id="under{k}" color="{ASPHALT}" fill="1" layer="-1" '
                     f'shape="{_fmt(left + right[::-1])}"/>')
        k += 1


def _approach_pavement(lines, shapes, edge_ids):
    """연속된 진입로 외곽을 한 면으로 이어 분할 노드의 틈과 계단을 덮는다."""
    right, left = [], []
    hw = _lane_width() / 2.0
    for eid in edge_ids:
        indexes = sorted(idx for edge, idx in shapes if edge == eid)
        for target, idx, offset in ((right, indexes[0], -hw), (left, indexes[-1], hw)):
            sh = shapes[(eid, idx)]
            for i, p in enumerate(sh):
                a, b = sh[max(0, i-1)], sh[min(len(sh)-1, i+1)]
                length = math.dist(a, b) or 1.0
                f = ((b[0]-a[0])/length, (b[1]-a[1])/length)
                target.append(_local(p, f, [(0.0, offset)])[0])
    lines.append(f'    <poly id="approach_pavement" color="{ASPHALT}" fill="1" '
                 f'layer="1" shape="{_fmt(right + left[::-1])}"/>')


def _widen_taper(lines, shapes, before, after, taper=0.0):
    """오른쪽에 차로가 하나 늘어나는 곳을 계단 대신 비스듬히 넓어지게 메운다.

    늘어나기 전 엣지(before) 0번 차로의 오른쪽 경계(끝에서 taper m 전)에서
    늘어난 엣지(after) 0번 차로의 오른쪽 경계 시작점까지 아스팔트 쐐기를 깐다.
    """
    hw = _lane_width() / 2.0
    before_sh = shapes[(before, 0)]
    blen = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(before_sh, before_sh[1:]))
    # 꼭짓점: 늘어나기 전 도로의 오른쪽 가장자리 끝 (taper m 상류).
    #   곡선 구간이라 더 상류로 잡으면 곡선 안쪽을 가로질러 엉뚱한 쐐기가 생긴다.
    p, f = _point_from_end(before_sh, min(taper, blen))
    apex = _local(p, f, [(0.0, -hw)])[0]
    # 밑변: 늘어난 뒤 새 차로(0번)의 양쪽 경계, 시작점에서 2 m 하류
    sh = shapes[(after, 0)]
    r, h = _point_from_end(sh[::-1], 2.0)
    h = (-h[0], -h[1])
    outer = _local(r, h, [(0.0, -hw)])[0]
    inner = _local(r, h, [(0.0, hw)])[0]
    lines.append(f'    <poly id="widen_taper" color="{ASPHALT}" fill="1" layer="-1" '
                 f'shape="{_fmt([apex, outer, inner])}"/>')


def _stop_line(lines, pid, shapes, eid, width=0.45, lanes=None, dist=0.4):
    """차로 끝(교차점 직전)에 그 엣지의 모든 차로를 가로지르는 흰색 정지선(실선).

    곡선 도로는 차로마다 끝나는 위치가 조금씩 달라서, 차로별로 그리면 선이
    토막으로 어긋난다. 맨 오른쪽 차로 끝의 오른쪽 경계와 맨 왼쪽 차로 끝의
    왼쪽 경계를 이어 한 줄로 그린다.
    """
    n = 0
    while (eid, n) in shapes:
        n += 1
    if n == 0:
        raise ValueError(f"정지선을 그릴 엣지가 없습니다: {eid}")
    lo, hi = (min(lanes), max(lanes)) if lanes else (0, n - 1)   # 일부 차로만 가능
    hw = _lane_width() / 2.0
    e0, f0 = _point_from_end(shapes[(eid, lo)], dist)
    e1, f1 = _point_from_end(shapes[(eid, hi)], dist)
    seg = [_local(e0, f0, [(0.0, -hw)])[0], _local(e1, f1, [(0.0, hw)])[0]]
    lines.append(f'    <poly id="{pid}" color="{WHITE}" fill="0" lineWidth="{width}" '
                 f'layer="6" shape="{_fmt(seg)}"/>')


def _yield_marks(lines, leg, r_out):
    """진입로: 원 바깥 가장자리를 따라 휘어진 양보선(점선) + 차로마다 양보 표시 ▽.

    양보선은 원을 감싸듯 r_out 원호 위에 진입로 폭만큼만 그린다.
    """
    lw = _lane_width()
    r_line = r_out + 0.3
    ux, uy = _unit(leg.axis)
    nx, ny = -uy, ux

    def on_circle(r, lat):
        a = math.sqrt(max(r ** 2 - lat ** 2, 0.0))
        return ux * a + nx * lat, uy * a + ny * lat

    # 진입로 양쪽 경계가 원호와 만나는 각도
    angs = []
    for off in (-leg.width / 2.0, leg.width / 2.0):
        x, y = on_circle(r_line, leg.lat_ring + off)
        angs.append(math.degrees(math.atan2(y, x)))
    a0, a1 = sorted(angs)
    if a1 - a0 > 180.0:              # 0°/360° 경계를 넘는 경우
        a0, a1 = a1, a0 + 360.0
    arc_len = math.radians(a1 - a0) * r_line
    n = max(int(arc_len // 0.9), 1)
    for j in range(n):
        s0 = a0 + (a1 - a0) * j / n
        s1 = s0 + (a1 - a0) / n * 0.55
        lines.append(f'    <poly id="yl_{leg.edge_id}_{j}" color="{WHITE}" fill="0" '
                     f'lineWidth="0.3" layer="6" shape="{_fmt(_arc_points(r_line, s0, s1, 1.0))}"/>')

    # 양보 표시 ▽: 차로 가운데, 원에서 3 m 바깥. 밑변이 원 쪽, 꼭짓점이 운전자 쪽.
    # 진입 방향(안쪽, -u)의 오른쪽 = +n 쪽이 0번 차로.
    for i in range(leg.lanes):
        lat = leg.lat_ring + leg.width / 2.0 - (i + 0.5) * lw
        c = on_circle(r_out + 3.0, lat)
        f = (-ux, -uy)
        tri = _local(c, f, [(1.5, 0.9), (1.5, -0.9), (-1.5, 0.0), (1.5, 0.9)])
        lines.append(f'    <poly id="yt_{leg.edge_id}_{i}" color="{WHITE}" fill="0" lineWidth="0.2" '
                     f'layer="6" shape="{_fmt(tri)}"/>')


def _arrow(lines, aid, shapes, eid, lane, kind, dist, from_start=False):
    """노면 화살표 — 몸통·꺾임·머리가 하나로 이어진 채운 도형.

    굵은 선(lineWidth)으로 곡선을 그리면 SUMO가 토막마다 사각형으로 그려
    꺾이는 부분이 톱니처럼 보인다. 그래서 몸통 중심선을 양옆으로 반 폭씩
    평행이동한 윤곽선에 머리 삼각형을 붙여 다각형 하나로 채운다.

    from_start=False: 차로 끝에서 거꾸로 dist 지점 (진입로 — 원 쪽 끝 기준)
    from_start=True : 차로 시작에서 dist 지점 (진출로 — 원에서 나온 직후 기준)
    """
    if kind in ("straight_right", "straight_left", "straight_uturn"):
        # Both heads share the same stem and lane position.
        _arrow(lines, aid + "_straight", shapes, eid, lane, "straight", dist, from_start)
        turn = {"straight_right":"right", "straight_left":"left", "straight_uturn":"uturn"}[kind]
        _arrow(lines, aid + "_" + turn, shapes, eid, lane, turn, dist, from_start)
        return
    shape = shapes[(eid, lane)]
    if from_start:
        origin, (bx_, by_) = _point_from_end(shape[::-1], dist)
        f = (-bx_, -by_)
    else:
        origin, f = _point_from_end(shape, dist)

    W = 0.40            # 몸통 폭 (m)
    HL, HW = 1.1, 1.0   # 머리 길이 / 폭 (m)
    if kind == "straight":
        body = [(-3.2, 0.0), (1.0, 0.0)]
    elif kind in ("right", "left", "ccw", "uturn"):
        # 직선 몸통 후 반지름 R로 꺾임 (왼쪽 = y 양수 방향)
        sign = 1.0 if kind in ("left", "uturn") else -1.0
        turn = 180.0 if kind == "uturn" else (90.0 if kind in ("right", "left") else 50.0)
        R = 0.8
        phi = math.radians(turn)
        y0 = 0.0     # 화살표 몸통은 차로 중심선 위에 놓는다.
        body = [(-3.0, y0), (0.0, y0)]
        for k in range(1, 13):
            p = math.radians(turn * k / 12)
            body.append((R * math.sin(p), y0 + sign * (R - R * math.cos(p))))
        # 꺾인 뒤 머리 전까지 짧은 직선
        hx, hy = math.cos(phi), sign * math.sin(phi)
        bx, by = body[-1]
        body.append((bx + hx * 0.2, by + hy * 0.2))
    else:
        raise ValueError(f"알 수 없는 화살표 type: {kind}")

    # 몸통 윤곽: 중심선의 왼쪽/오른쪽 평행선
    left, right = [], []
    for i, (x, y) in enumerate(body):
        (x0, y0_), (x1, y1) = body[max(i - 1, 0)], body[min(i + 1, len(body) - 1)]
        L = math.hypot(x1 - x0, y1 - y0_) or 1.0
        nx, ny = -(y1 - y0_) / L, (x1 - x0) / L
        left.append((x + nx * W / 2, y + ny * W / 2))
        right.append((x - nx * W / 2, y - ny * W / 2))
    # 머리: 몸통 끝 방향으로
    (ex, ey), (fx, fy) = body[-2], body[-1]
    L = math.hypot(fx - ex, fy - ey) or 1.0
    dx, dy = (fx - ex) / L, (fy - ey) / L
    px, py = -dy, dx
    tip = (fx + dx * HL, fy + dy * HL)
    head = [(fx + px * HW / 2, fy + py * HW / 2), tip, (fx - px * HW / 2, fy - py * HW / 2)]
    outline = left + head + right[::-1]
    if kind in ("right", "left", "ccw", "uturn"):
        # 회전 머리를 차로 폭 안에 유지하면서 몸통을 중앙에 정렬한다.
        outline = [(x, y * 0.60) for x, y in outline]
    # 곡선 차로에서도 각 단면의 중심선을 따라 표시해 안전지대 침범을 막는다.
    world = []
    for x, y in outline:
        if from_start:
            p, backward = _point_from_end(shape[::-1], dist + x)
            direction = (-backward[0], -backward[1])
        else:
            p, direction = _point_from_end(shape, dist - x)
        world.append(_local(p, direction, [(0.0, y)])[0])
    lines.append(f'    <poly id="{aid}" color="{WHITE}" fill="1" layer="6" '
                 f'shape="{_fmt(world)}"/>')


def _island_poly(cw_leg, ccw_leg, a_start, a_end, margin=0.3):
    """반시계 순서로 이웃한 두 leg 사이의 섬 영역 (볼록 사각형)."""
    p1 = cw_leg.point(a_start, cw_leg.width / 2.0 + margin)
    p2 = cw_leg.point(a_end, cw_leg.width / 2.0 + margin)
    p3 = ccw_leg.point(a_end, -ccw_leg.width / 2.0 - margin)
    p4 = ccw_leg.point(a_start, -ccw_leg.width / 2.0 - margin)
    return [p1, p2, p3, p4]


def _rounded(pts, r=1.5, n=6):
    """다각형 꼭짓점을 반지름 약 r로 둥글린다 (꼭짓점마다 2차 베지어 곡선)."""
    out = []
    m = len(pts)
    for i in range(m):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % m]
        d01 = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) or 1.0
        d12 = math.hypot(p2[0] - p1[0], p2[1] - p1[1]) or 1.0
        t0, t1 = min(r, d01 / 2) / d01, min(r, d12 / 2) / d12
        a = (p1[0] + (p0[0] - p1[0]) * t0, p1[1] + (p0[1] - p1[1]) * t0)
        b = (p1[0] + (p2[0] - p1[0]) * t1, p1[1] + (p2[1] - p1[1]) * t1)
        for k in range(n + 1):
            t = k / n
            out.append(((1 - t) ** 2 * a[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * b[0],
                        (1 - t) ** 2 * a[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * b[1]))
    return out


def _draw_island(lines, pid, kind, poly, axis, inner=None, pattern=None):
    """섬/안전지대 그리기.

    sidewalk     : 인도형 분리섬 (회색)
    island_hatch : 빗금 영역 + 그 안의 화단 섬 (inner를 주면 그 위치에만)
    island_nose  : 앞코(poly)에만 빗금, 그 뒤에 둥근 화단 섬(inner) — 섬 둘레는 연석만
    hatch        : 빗금만
    """
    if kind == "sidewalk":
        lines.append(f'    <poly id="{pid}" color="{SIDEWALK}" fill="1" layer="2" '
                     f'shape="{_fmt(poly)}"/>')
        lines.append(f'    <poly id="{pid}_curb" color="{CURB}" fill="0" lineWidth="0.3" '
                     f'layer="3" shape="{_fmt(poly + [poly[0]])}"/>')
    elif kind in ("island_hatch", "island_nose"):
        _hatch(lines, pid, poly, axis, pattern=pattern)
        if inner is None:
            inner = _inset(poly, 1.2)
        if kind == "island_nose":
            inner = _rounded(inner, 2.5)
        lines.append(f'    <poly id="{pid}_island" color="{GRASS}" fill="1" layer="4" '
                     f'shape="{_fmt(inner)}"/>')
        lines.append(f'    <poly id="{pid}_curb" color="{CURB}" fill="0" lineWidth="0.3" '
                     f'layer="5" shape="{_fmt(inner + [inner[0]])}"/>')
    elif kind == "hatch":
        _hatch(lines, pid, poly, axis, pattern=pattern)
    else:
        raise ValueError(f"알 수 없는 island type: {kind}")


def _center_line(lines, pid, pts, gap=0.3, clearance=None, near_extension=0.0):
    """노란 중앙선 (복선: 두 줄 사이 gap m)."""
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    L = math.hypot(x1 - x0, y1 - y0) or 1.0
    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
    for k, s in enumerate((-gap / 2.0, gap / 2.0)):
        seg = [(x + nx * s, y + ny * s) for x, y in pts]
        pieces = [seg]
        if clearance is not None:
            from shapely.geometry import LineString
            fitted = LineString(seg).difference(clearance.buffer(0.075))
            pieces = ([list(fitted.coords)] if fitted.geom_type == 'LineString'
                      else [list(g.coords) for g in fitted.geoms if g.geom_type == 'LineString'])
        for j, piece in enumerate(pieces):
            if clearance is not None and j == 0 and len(pieces) > 1 and near_extension:
                x, y = piece[-1]
                piece[-1] = (x+(x1-x0)/L*near_extension, y+(y1-y0)/L*near_extension)
            suffix = '' if j == 0 else f'_part{j}'
            lines.append(f'    <poly id="{pid}_{k}{suffix}" color="{YELLOW}" fill="0" lineWidth="0.15" '
                         f'layer="6" shape="{_fmt(piece)}"/>')


def _uturn_markings(lines, net_path):
    """Mark the actual turning path, leaving the through lanes unobstructed."""
    import xml.etree.ElementTree as ET
    from env.roundabout_geometry import _read, _length, _slice
    root = ET.parse(net_path).getroot()
    lanes = {l.get('id'): _read(l.get('shape')) for e in root.findall('edge')
             for l in e.findall('lane')}
    follow = {(c.get('from'), c.get('fromLane')): c for c in root.findall('connection')
              if c.get('from').startswith(':')}
    for connection in root.findall('connection'):
        if connection.get('dir') != 't' or connection.get('from').startswith(':'):
            continue
        points, current = [], connection.get('via')
        while current:
            points.extend(lanes[current][1 if points else 0:])
            edge, index = current.rsplit('_', 1)
            current = follow[(edge, index)].get('via')
        name = 'uturn_'+connection.get('from')
        total = _length(points)
        north_turn = connection.get('from') == 'N_in1_far'
        triangle_station = total*0.75 if north_turn else 3.0
        arrow_start, arrow_end = ((0.18, 0.45) if north_turn else (0.45, 0.72))
        origin, tangent = _point_from_end(points, total-triangle_station)
        triangle = _local(origin, tangent, [(-1.0, 0.7), (1.0, 0), (-1.0, -0.7), (-1.0, 0.7)])
        lines.append(f'    <poly id="{name}_yield" color="{WHITE}" fill="0" '
                     f'lineWidth="0.15" layer="6" shape="{_fmt(triangle)}"/>')
        stroke = _slice(points, total*arrow_start, total*arrow_end)
        lines.append(f'    <poly id="{name}_arrow_stem" color="{WHITE}" fill="0" '
                     f'lineWidth="0.25" layer="6" shape="{_fmt(stroke)}"/>')
        origin, tangent = _point_from_end(points, total*(1.0-arrow_end))
        head = _local(origin, tangent, [(0.7, 0), (-0.5, 0.6), (-0.5, -0.6)])
        lines.append(f'    <poly id="{name}_arrow_head" color="{WHITE}" fill="1" '
                     f'layer="6" shape="{_fmt(head)}"/>')


def _fit_island_to_uturn(poly, net_path, outward, inward):
    """Extend the safety paint into the median up to the real U-turn lane edge."""
    import xml.etree.ElementTree as ET
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    from env.roundabout_geometry import _read
    root = ET.parse(net_path).getroot()
    turn = next((c for c in root.findall('connection')
                 if c.get('from') == inward.far_edge and c.get('to') == outward.far_edge
                 and c.get('dir') == 't'), None)
    if turn is None:
        return poly
    paths = {l.get('id'): _read(l.get('shape')) for e in root.findall('edge')
             for l in e.findall('lane')}
    follow = {(c.get('from'), c.get('fromLane')): c for c in root.findall('connection')
              if c.get('from').startswith(':')}
    turn_points, current = [], turn.get('via')
    while current:
        turn_points.extend(paths[current])
        edge, index = current.rsplit('_', 1)
        current = follow[(edge, index)].get('via')
    ux, uy = _unit(outward.axis)
    station = max(x*ux+y*uy for x, y in turn_points)+_lane_width()
    extended = list(poly)
    for i in (1, 2):
        x, y = extended[i]
        delta = station-(x*ux+y*uy)
        extended[i] = (x+delta*ux, y+delta*uy)
    road = unary_union([LineString(p).buffer(_lane_width()/2+0.15, cap_style=2,
                                            quad_segs=24) for p in paths.values()])
    fitted = Polygon(extended).difference(road)
    pieces = [fitted] if fitted.geom_type == 'Polygon' else list(fitted.geoms)
    fitted = max(pieces, key=lambda p: p.intersection(Polygon(poly)).area)
    return list(fitted.exterior.coords)[:-1]



def _write_decoration(out_dir, ring, approaches, extra_islands, legs, by_key, scenery,
                      markings, arrows, net_path, link=None, stop_lines=()):
    global HATCH_PATTERN
    HATCH_PATTERN = markings.get("hatch_pattern", "chevron")
    lw = _lane_width()
    rc = float(ring["radius"])
    r_in = rc - ring["num_lanes"] * lw / 2.0
    lines = ["<additional>"]

    if scenery.get("ground", True):
        extent = max(l.a_ring + l.length for l in legs) + 20.0
        sq = [(-extent, -extent), (extent, -extent), (extent, extent), (-extent, extent)]
        lines.append(f'    <poly id="ground" color="{GROUND}" fill="1" layer="-10" '
                     f'shape="{_fmt(sq)}"/>')

    # 중앙섬: 바깥쪽 돌 포장(트럭 에이프런) → 화단 → 꽃
    lines.append(f'    <poly id="center_apron" color="{APRON}" fill="1" layer="2" '
                 f'shape="{_fmt(_circle(r_in - 0.2))}"/>')
    lines.append(f'    <poly id="center_curb" color="{YELLOW}" fill="0" lineWidth="0.25" '
                 f'layer="3" shape="{_fmt(_circle(r_in - 0.2) + [_circle(r_in - 0.2)[0]])}"/>')
    lines.append(f'    <poly id="center_grass" color="{GRASS}" fill="1" layer="3" '
                 f'shape="{_fmt(_circle(r_in - 2.6))}"/>')
    lines.append(f'    <poly id="center_flower" color="{FLOWER}" fill="1" layer="4" '
                 f'shape="{_fmt(_circle(r_in - 4.0))}"/>')
    lines.append(f'    <poly id="center_bush" color="{GRASS}" fill="1" layer="5" '
                 f'shape="{_fmt(_circle(r_in - 7.0))}"/>')

    # 같은 방향 안의 leg 사이 섬
    for name, ap in approaches.items():
        for k, isl in enumerate(ap.get("islands", [])):
            cw, ccw = by_key[(name, k)], by_key[(name, k + 1)]
            a0 = cw.a_ring
            a1 = a0 + float(isl["island_len"])
            poly = _island_poly(cw, ccw, a0, a1)
            # Move only the painted island; final clipping fits it to unchanged roads.
            shift = float(isl.get("toward_ring", 0.0))
            ux, uy = _unit(float(ap["axis"]))
            poly = [(x-shift*ux, y-shift*uy) for x, y in poly]
            inner = None
            if shift and isl["type"] == "island_hatch":
                inner = _inset(poly, 1.2)
                # Keep a painted band between the planting and the circular road.
                for i in (0, len(inner)-1):
                    x, y = inner[i]
                    inner[i] = (x+1.2*ux, y+1.2*uy)
            if cw.split_a is not None and ccw.split_a is not None:
                poly = _fit_island_to_uturn(poly, net_path, cw, ccw)
            _draw_island(lines, f"isl_{name}_{k}", isl["type"], poly, float(ap["axis"]),
                         inner=inner, pattern=isl.get("pattern"))
        eh = ap.get("edge_hatch")
        if eh:
            leg = by_key[(name, int(eh["leg"]))]
            a0 = leg.a_ring
            a1 = a0 + float(eh["len"])
            w = float(eh["width"])
            # 진입로 바깥쪽(반시계 쪽이 바깥인 leg면 왼쪽) 가장자리
            side = 1.0 if leg.idx == len(ap["legs"]) - 1 else -1.0
            e0 = leg.point(a0, side * (leg.width / 2.0 + 0.2))
            e1 = leg.point(a1, side * (leg.width / 2.0 + 0.2))
            o0 = leg.point(a0, side * (leg.width / 2.0 + 0.2 + w))
            _hatch(lines, f"edgehatch_{name}", [e0, e1, o0], float(ap["axis"]))

    small_island_baseline = None
    split_centerline_nose = None
    # 서로 다른 방향의 leg 사이 섬 (예: 단봉로 출구 ↔ 검단로 출구)
    for k, isl in enumerate(extra_islands):
        (n1, i1), (n2, i2) = isl["between"]
        cw, ccw = by_key[(n1, i1)], by_key[(n2, i2)]
        if cw.ring_angle > ccw.ring_angle:
            cw, ccw = ccw, cw
        # 두 leg는 원에서 거의 붙어 나와 축 각도 차이만큼 벌어진다.
        # 경계 사이가 1 m 이상 벌어지는 지점이 빗금 삼각형의 뾰족한 앞코.
        a0 = cw.a_ring
        while a0 < cw.a_ring + 30.0:
            p = cw.point(a0, cw.width / 2.0)
            q = ccw.point(a0, -ccw.width / 2.0)
            if math.hypot(p[0] - q[0], p[1] - q[1]) > 1.0:
                break
            a0 += 0.25
        a1 = a0 + float(isl.get("island_len", 0.0))
        inner = None
        if isl["type"] == "split_fill":
            # 세 길 사이 공간 전체를 빗금으로 채우고, 그 위에 테두리만 남기고 화단(땅).
            # 원 쪽 앞코 nose_len 구간은 빗금만.
            if not link:
                raise ValueError("split_fill 섬은 SPLIT_LINK 설정이 있어야 합니다")
            # 빗금 영역도 갈라지는 곳의 뾰족한 끝(폭 0.8 m 미만)은 잘라야
            # 안쪽으로 줄일 때 테두리가 꼬이지 않는다
            region = _split_fill_region(cw, ccw, link, a0, min_sep=0.8)
            poly = _clean_polygon(_offset_polygon(region, 0.3))
            # Continue the incoming road's median up to the safety-zone nose.
            ux, uy = _unit(cw.axis)
            split_centerline_nose = max(poly, key=lambda p: p[0]*ux + p[1]*uy)
            # The user's guide starts at the large island's bottom tip and
            # follows its lower painted edge southwest across the junction.
            lower_tip = min(range(len(poly)), key=lambda i: poly[i][1])
            small_island_baseline = (poly[lower_tip], poly[(lower_tip+1) % len(poly)])
            margin = float(isl.get("land_margin", 1.6))
            land = _clean_polygon(_offset_polygon(
                _split_fill_region(cw, ccw, link, a0 + float(isl["nose_len"]),
                                   min_sep=2.0 * margin + 1.5),
                margin))
            mid_axis = (cw.axis + ccw.axis) / 2.0
            _hatch(lines, f"isl_extra_{k}", poly, mid_axis, pattern=isl.get("pattern"))
            land = _rounded(land, 2.0)
            lines.append(f'    <poly id="isl_extra_{k}_land" color="{GRASS}" fill="1" layer="4" '
                         f'shape="{_fmt(land)}"/>')
            lines.append(f'    <poly id="isl_extra_{k}_curb" color="{CURB}" fill="0" lineWidth="0.3" '
                         f'layer="5" shape="{_fmt(land + [land[0]])}"/>')
            continue
        if isl["type"] == "island_nose":
            # 앞코 nose_len 구간만 빗금 삼각형, 그 뒤 전부가 둥근 화단 섬
            nose = a0 + float(isl["nose_len"])
            poly = _island_poly(cw, ccw, a0, nose)
            inner = _island_poly(cw, ccw, nose, a1, margin=0.8)
        else:
            poly = _island_poly(cw, ccw, a0, a1)
            if "island_start" in isl:
                # 화단 섬: 빗금 삼각형의 넓은 쪽에만 (앞코에서 island_start 이후)
                inner = _island_poly(cw, ccw, a0 + float(isl["island_start"]), a1 - 1.0,
                                     margin=1.3)
        mid_axis = (cw.axis + ccw.axis) / 2.0
        _draw_island(lines, f"isl_extra_{k}", isl["type"], poly, mid_axis, inner,
                     pattern=isl.get("pattern"))

    # 노면 표시
    shapes = _read_lane_shapes(net_path)
    from env.roundabout_markings import crossing_clearance
    crossing = (crossing_clearance(net_path, **markings['clear_crossing'])
                if markings.get('clear_crossing') else None)
    r_out = rc + ring["num_lanes"] * lw / 2.0
    _ring_pavement(lines, r_in, r_out)
    dash, gap = float(markings.get("dash", 3.0)), float(markings.get("gap", 3.0))
    continuous_edges = []
    if link and link.get("lane_shapes"):
        diag = link["diag_end"].rsplit("_end", 1)[0]
        approach = diag.rsplit("_diag", 1)[0]
        continuous_edges = [f"{approach}_infar", diag, link["diag_end"]]
        _approach_pavement(lines, shapes, continuous_edges)
    # 이 직선 출구는 교차로 외곽과 정확히 맞닿는다. 확장 바닥을 덧대면
    # 매끈한 곡선 끝에 작은 삼각형 돌기가 다시 생기므로 확장하지 않는다.
    joined_exit = [link["right_turn_target"]] if link and link.get("lane_shapes") else []
    _lane_underlay(lines, shapes, skip_edges=continuous_edges + joined_exit)
    if link:
        center = list(link["centerline"])
        if split_centerline_nose is not None:
            center.insert(0, split_centerline_nose)
            lines.append(f'    <poly id="split_median_pavement" color="{ASPHALT}" '
                         f'fill="0" lineWidth="0.65" layer="5" shape="{_fmt(center)}"/>')
        _center_line(lines, "centerline_split", center)
    if markings.get("west_centerline"):
        west_out, west_in = by_key[("W", 0)], by_key[("W", 1)]
        # The triangular safety zone already marks the wide near-ring median.
        # Continue from its narrow end through the junction to the road's far end.
        start = max(west_out.a_far, west_in.a_far)
        end = min(west_out.a_ring+west_out.length, west_in.a_ring+west_in.length)
        a, b = west_out.left_edge(start), west_out.left_edge(end)
        c, d = west_in.right_edge(start), west_in.right_edge(end)
        median = [a, b, d, c]
        lines.append(f'    <poly id="west_median_pavement" color="{ASPHALT}" fill="1" '
                     f'layer="5" shape="{_fmt(median)}"/>')
        center = [((a[0]+c[0])/2, (a[1]+c[1])/2),
                  ((b[0]+d[0])/2, (b[1]+d[1])/2)]
        _center_line(lines, "west_centerline", center, gap=0.22, clearance=crossing,
                     near_extension=float(markings.get('crossing_centerline_extension', 0.0)))
    if markings.get("east_centerline"):
        east_out, east_in = by_key[("E", 0)], by_key[("E", 1)]
        start = max(east_out.a_far, east_in.a_far)
        end = min(east_out.a_ring+east_out.length, east_in.a_ring+east_in.length)
        a, b = east_out.left_edge(start), east_out.left_edge(end)
        c, d = east_in.right_edge(start), east_in.right_edge(end)
        lines.append(f'    <poly id="east_median_pavement" color="{ASPHALT}" fill="1" '
                     f'layer="5" shape="{_fmt([a, b, d, c])}"/>')
        from env import roundabout_config as road_cfg
        t_cfg = getattr(road_cfg, 'T_JUNCTION', None)
        if t_cfg:
            # Keep the median surface continuous, but leave turning paths unmarked.
            line_end = min(end, float(t_cfg['distance'])-25.0)
            b, d = east_out.left_edge(line_end), east_in.right_edge(line_end)
        center = [((a[0]+c[0])/2, (a[1]+c[1])/2),
                  ((b[0]+d[0])/2, (b[1]+d[1])/2)]
        _center_line(lines, "east_centerline", center, gap=0.3)
    if markings.get("yield", True):
        for leg in legs:
            if leg.kind == "entry":
                _yield_marks(lines, leg, r_out)
    for k, a in enumerate(arrows):
        if "edge" in a:
            # leg가 아닌 엣지(예: 대각선 길)는 엣지 id로 직접 지정, 기본은 차로 끝 기준
            eid, from_start = a["edge"], a.get("from") == "start"
        else:
            leg = by_key[(a["approach"], int(a["leg"]))]
            # 진출로는 원에서 나온 지점(차로 시작)부터, 진입로는 원 쪽 끝부터 잰다
            eid, from_start = leg.edge_id, leg.kind == "exit"
        _arrow(lines, f"arrow{k}", shapes, eid, int(a["lane"]),
               a["type"], float(a["dist"]), from_start=from_start)
    for k, s in enumerate(stop_lines):
        if isinstance(s, dict):
            _stop_line(lines, f"stop{k}", shapes, s["edge"], lanes=s.get("lanes"),
                       dist=float(s.get("dist", 0.4)))
        else:
            _stop_line(lines, f"stop{k}", shapes, s)
    if link and link.get("widen_taper"):
        _widen_taper(lines, shapes, link["diag_end"].rsplit("_end", 1)[0], link["diag_end"])
    layout = None
    if link and link.get("diag_end") and link.get("right_island"):
        ri = link["right_island"]
        from env.roundabout_layout import channel_layout
        layout = channel_layout(net_path, link["diag_end"], small_island_baseline,
                                _lane_width(), float(ri.get("clearance", 0.15)))
        p, q = shapes[(link["diag_end"], 1)][-2:]
        heading = math.degrees(math.atan2(q[1]-p[1], q[0]-p[0]))
        _hatch(lines, "right_island", layout["island"],
               heading, spacing=0.9, layer=6, pattern="diagonal")
        lines.append(f'    <poly id="left_lanes_outer_border" color="{WHITE}" fill="0" '
                     f'lineWidth="0.12" layer="6" shape="{_fmt(layout["left_outer"])}"/>')
    from env.roundabout_markings import write_dashes
    _uturn_markings(lines, net_path)
    dash_report = write_dashes(lines, net_path, rc, dash, gap,
                               separated_edge=link["diag_end"] if layout else None,
                               special_divider=layout["divider"] if layout else None,
                               ring=markings.get("ring_divider", True) and ring["num_lanes"] > 1,
                               width=lw, junction_guides=markings.get("junction_guides", ()),
                               omit_dashes=markings.get("omit_dashes", ()), clear_crossing=crossing)

    lines.append("</additional>")
    with open(os.path.join(out_dir, f"{PREFIX}.deco.xml"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return dash_report


# ──────────────────────────────────────────────────────────────
# 진입점
# ──────────────────────────────────────────────────────────────
def build(out_dir: str, force: bool = False) -> str:
    """회전교차로 SUMO 파일들을 out_dir에 생성하고 sumocfg 경로를 반환한다.

    env/sumo_roundabout 의 맵은 이 결과 위에 패치들을 덧붙인 것이라, 덮어쓰면
    손질한 도로가 전부 사라진다. 그래서 그 폴더에 이미 맵이 있으면 force=True 없이는
    막는다. 맵을 다시 만들 땐 python view_roundabout.py --rebuild 를 쓴다.
    """
    protected = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sumo_roundabout")
    if (not force and os.path.abspath(out_dir) == protected
            and os.path.exists(os.path.join(protected, f"{PREFIX}.net.xml"))):
        raise RuntimeError(
            f"{protected} 의 맵은 패치가 적용된 상태라 덮어쓰지 않습니다.\n"
            f"  다시 만들려면: python view_roundabout.py --rebuild")
    from env import roundabout_config as cfg
    from env.roundabout_config import (RING, APPROACHES, EXTRA_ISLANDS,
                                       TRAFFIC, SCENERY, MARKINGS, ARROWS)
    split_cfg = getattr(cfg, "SPLIT_LINK", None)
    os.makedirs(out_dir, exist_ok=True)
    legs, by_key = _make_legs(APPROACHES, RING)
    _snap_extra_islands(EXTRA_ISLANDS, by_key, RING)
    link = _plan_split_link(split_cfg, by_key) if split_cfg else None
    _plan_uturns(getattr(cfg, "UTURNS", []), APPROACHES, by_key, link)
    _write_network(out_dir, RING, legs, link)
    _write_routes(out_dir, legs, TRAFFIC, link,
                  uturn_approaches={u["approach"] for u in getattr(cfg, "UTURNS", [])})
    t_junction = getattr(cfg, "T_JUNCTION", None)
    if t_junction:
        from env.roundabout_tjunction import add_routes
        add_routes(out_dir)

    # GUI 시작 화면: 원 중심, 회전교차로 주변 140 m 정도가 보이게.
    # SUMO가 자동으로 그리는 회전 화살표(showLinkDecals)와 정지선 막대
    # (showLinkRules)는 실제 노면과 달라서 끄고, 노면 표시는 deco 파일에서 그린다.
    # (주행 규칙 — 양보, 차로 연결 — 은 그대로이고 화면 표시만 꺼진다)
    extent = 2.0 * max(l.a_ring + l.length for l in legs)
    view = 140.0
    zoom = extent / view * 100.0
    # 차로 경계선(laneShowBorders)도 끈다: 원 위의 아주 짧은 구간에까지 짧은 흰 선이
    # 생겨 도로가 깨져 보이므로, 차선은 deco 파일에서 직접 그린다.
    asphalt = ASPHALT
    with open(os.path.join(out_dir, f"{PREFIX}.gui.xml"), "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="geomdan">
        <background backgroundColor="{GROUND}" showGrid="0"/>
        <edges laneEdgeMode="0" laneShowBorders="0" showLinkDecals="0" showLinkRules="0" showRails="1">
            <colorScheme name="uniform">
                <entry color="{asphalt}" name="road"/>
                <entry color="grey" name="sidewalk"/>
                <entry color="192,66,44" name="bike lane"/>
                <entry color="0,0,0,0" name="green verge"/>
                <entry color="150,200,200" name="waterway"/>
                <entry color="black" name="railway"/>
                <entry color="64,0,64" name="rails on road"/>
                <entry color="92,92,92" name="no passenger"/>
                <entry color="red" name="closed"/>
                <entry color="green" name="connector"/>
                <entry color="orange" name="forbidden"/>
            </colorScheme>
        </edges>
        <junctions junctionMode="0" drawShape="1">
            <colorScheme name="uniform">
                <entry color="{asphalt}"/>
                <entry color="150,200,200" name="waterway"/>
                <entry color="0,0,0,0" name="railway"/>
            </colorScheme>
        </junctions>
        <vehicles vehicleMode="0" vehicleQuality="2"/>
    </scheme>
    <viewport zoom="{zoom:.0f}" x="0" y="0" angle="0"/>
    <delay value="{float(SIMULATION.get('gui_delay', 100.0))}"/>
</viewsettings>
""")

    cfg_path = os.path.join(out_dir, f"{PREFIX}.sumocfg")
    with open(cfg_path, "w") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="{PREFIX}.net.xml"/>
        <route-files value="{PREFIX}.rou.xml"/>
        <additional-files value="{PREFIX}.deco.xml"/>
    </input>
    <gui_only>
        <gui-settings-file value="{PREFIX}.gui.xml"/>
    </gui_only>
</configuration>
""")

    netconvert = checkBinary("netconvert")
    net_path = os.path.join(out_dir, f"{PREFIX}.net.xml")
    from env.roundabout_geometry import enforce_lane_spacing

    def compile_net():
        if t_junction:
            from env.roundabout_tjunction import add_network
            add_network(out_dir, t_junction)
        result = subprocess.run(
            [netconvert,
             "--node-files", os.path.join(out_dir, f"{PREFIX}.nod.xml"),
             "--edge-files", os.path.join(out_dir, f"{PREFIX}.edg.xml"),
             "--connection-files", os.path.join(out_dir, f"{PREFIX}.con.xml"),
             "--no-turnarounds", "true",
             # 좌표 원점 이동 금지: 원 중심이 (0,0)으로 남아야 장식(deco) 도형과 맞는다
             "--offset.disable-normalization", "true",
             "--junctions.corner-detail", "24",
             "--precision", "6",
             "-o", net_path],
            capture_output=True, text=True, encoding="utf-8", errors="replace")
        if result.returncode != 0:
            raise RuntimeError("netconvert 실패:\n" + result.stderr)
        # 형상 보정 전(netconvert 원본) 유턴 경로 길이 — contPos는 이 길이 기준이다
        raw = {key: _movement_path(net_path, key[0], key[1], w["lane"])
               for key, w in (link.get("uturn_wait", {}) if link else {}).items()}
        checks = enforce_lane_spacing(net_path, _lane_width(),
                             link.get("diag_end") if link and link.get("lane_shapes") else None,
                             float(RING["radius"]), int(RING["num_lanes"]),
                             float(split_cfg.get("right_turn_lead", 3.5)) if split_cfg else 3.5,
                             float(split_cfg.get("right_turn_merge", 8.0)) if split_cfg else 8.0)
        if t_junction:
            from env.roundabout_tjunction import finish_geometry
            finish_geometry(net_path, t_junction)
        return raw, checks

    raw_uturn, geometry_checks = compile_net()
    if link and link.get("uturn_wait"):
        # 2단계: 보정된 실제 경로에서 대기 지점을 재고, 원본 길이 비율로 환산해 다시 만든다
        for key, w in link["uturn_wait"].items():
            final = _uturn_wait_point(net_path, key, w)
            w["pos"] = final * _path_length(raw_uturn[key]) / _path_length(
                _movement_path(net_path, key[0], key[1], w["lane"]))
        _write_network(out_dir, RING, legs, link)
        _, geometry_checks = compile_net()
    from env.roundabout_surfaces import fit_junction_pavement, fit_terrain
    geometry_checks["junctions_fitted"] = fit_junction_pavement(os.path.join(out_dir, f"{PREFIX}.net.xml"))

    # 장식/노면 표시는 컴파일된 차로 위치를 읽어야 하므로 netconvert 다음에 만든다
    dash_report = _write_decoration(out_dir, RING, APPROACHES, EXTRA_ISLANDS, legs, by_key, SCENERY,
                      MARKINGS, ARROWS, os.path.join(out_dir, f"{PREFIX}.net.xml"), link,
                      getattr(cfg, "STOP_LINES", []))
    geometry_checks.update(dash_report)
    fit_terrain(os.path.join(out_dir, f"{PREFIX}.net.xml"),
                os.path.join(out_dir, f"{PREFIX}.deco.xml"),
                float(RING["radius"]), _lane_width(), int(RING["num_lanes"]), geometry_checks)
    if t_junction:
        from env.roundabout_tjunction import add_markings
        add_markings(out_dir)
    return cfg_path
