"""도심 경로 주행(회전교차로 맵)용 관측 추가 · 안전 마스킹 · 보상.

직선도로용 기본 관측(31차원)은 차로 기준 앞뒤 차만 봐서, 교차로에서 옆으로 오는 차나
신호를 모른다. 여기서 다음을 추가한다 (설정: env/roundabout_env_config.py URBAN).

관측 추가 (모두 [-1, 1]):
  신호 (5)     : 앞 신호 있음, 정지선까지 거리, 적/황/녹 (눈에 보이는 것만. 2026-11-15: 수업 수준에 맞춰
                 '다음 신호 변경까지 남은 시간'은 뺐다 — 실제 운전자는 모르는 정보)
  경로 (4)     : 다음 회전(좌·우·유턴)까지 거리, 그 종류(좌/우/유턴 one-hot)
  ego 추가 (4) : 차로 제한속도, 교차로 안인지, 경로에 맞는 차로까지 차로 수, 경로 진행률
  상충 차량 (10): 교차로에서 경로가 겹치는 차 2대 × (있음, 내 거리, 그 차 거리, 그 차 속도, 내가 양보해야 하는지)
  주변 차량 (30): 반경 안 가까운 차 6대 × (있음, 앞뒤 거리, 좌우 거리, 상대속도 앞뒤, 상대속도 좌우)  ← ego 기준 좌표

안전 마스킹 (가속 명령을 "확실히 위험한" 경우에만 줄인다, 개입하면 감점):
  적신호      : 정지선 앞에서 멈출 수 있는 속도로 제한 (황색은 편하게 멈출 수 있을 때만)
  양보 위반   : 내가 양보해야 하는 차와 교차점 도착 시간이 겹치면 교차점 앞에서 멈출 속도로 제한
  추돌        : 앞차가 급정거해도 최소 간격이 남는 속도로 제한 (앞차 감속·반응 지연 1스텝 반영)
  교차로 안   : 멈추거나 머뭇거리지 않음. 앞이 비어 있으면 최소 속도(junction_min_speed)로 빠져나감
  RL 은 언제 들어갈지·얼마나 빨리 갈지·차로를 직접 정하고, 마스킹은 마지막 안전장치다.

보상 (매 스텝):
  + 진행 거리 (경로를 따라 간 거리, 제한속도로 달리면 스텝당 progress_weight)
  - 마스킹 개입, - 이유 없이 멈춤, - 제한속도 초과, - 앞차에 너무 붙음
  종료: + 완주, - 충돌, - 경로 이탈/순간이동
"""
import math
import os

import numpy as np

SIG_RANGE = 200.0      # 신호 관측 거리 (m)
MAN_RANGE = 300.0      # 다음 회전 관측 거리 (m)
FOE_RANGE = 80.0       # 교차로 상충 차량 관측 거리 (m)
NEAR_RADIUS = 50.0     # 주변 차량 반경 (m)
TURN_DEG = 30.0        # 연결 전후 방향 변화가 이 각도(°) 이상이면 회전
SLIP_DEG = 60.0        # 짧은 도로가 그 안에서 이만큼 휘면 회전 (우회전 전용 도로 등)
SLIP_MAX_LEN = 300.0
OVERLAP_BACK = 8.0    # 겹침 시작점은 교차점에서 최대 이만큼(m) 앞까지
OVERLAP_D = 2.3       # 교차 차량: 두 경로 중심선이 이 거리(m) 안이면 차체가 겹침 (차 폭 1.8 + 여유 0.5), 2026-11-20
FOE_WALK = os.environ.get("FOE_WALK", "1") == "1"   # 2026-11-21 공통 수정 2 켜기 (진단용 스위치)
FOE_LOOKAHEAD = 120.0  # 상충 차량: 교차로로 다가오는 차를 이 거리(m)까지 본다 (시속 60 km 면 약 7초 앞)
N_FOES, N_NEAR = 2, 6
LEAD_RANGE = 50.0      # 현재 차로 앞차 관측 거리 (m) = 수업 원본 OBSERVATION visibility
STOP_NEAR = 40.0       # 정지선 근접 거리 관측의 범위 (m)
# 2026-11-17 컨설팅 반영: 차선 단위 경로 정보(맞는 차로까지 차로 수)·양보 판단 칸·정지선 중복 칸 제거,
#   긴급차는 별도 칸 대신 주변 차량 칸의 "긴급차 여부" 1칸으로 합침
#   신호 5 + 다음 회전 4 + ego 추가 3 + 상충 차량 2×4 + 주변 차량 6×6
#   2026-11-17: 차로별 노면 화살표 5차로×(좌·직·우) 15칸 추가 (정지선까지 ARROW_RANGE m 안에서만 보임)
N_FOE_F, N_NEAR_F = 4, 6
N_ARROW_LANES, ARROW_RANGE = 5, 100.0
# 2026-11-19 (사용자 승인): 주변 차량을 "8방향 물체 인식"으로 바꿈 (실제 자율주행 인식 결과 = 물체 상자 목록을 흉내).
#   반경 50 m, 3차로(왼·내·오) × 앞·옆·뒤 = 8칸(앞·앞왼·왼·뒤왼·뒤·뒤오·오·앞오), 칸마다 보이는 가장 가까운 차 1대.
#   칸 배정은 차로를 따라감(같은 도로: 차로 번호·차로 위 위치, 도로 너머: 수업 원본 getLeader/getNeighbors). 교차로 안만 내 차 기준 좌표.
#   가림: 다른 차를 실제 크기 직사각형으로, 건물은 지도 다각형으로 — 내 차 중심에서 상대 차 네 모서리·중심으로 시선 5개,
#   하나라도 안 막히면 보임. 칸마다 7값: 있음, 몸체 사이 가장 가까운 점의 앞뒤·좌우 위치(÷50 m), 앞뒤·좌우 상대속도(÷vmax),
#   차 길이(÷15 m), 긴급차 여부. 앞차 2칸(ego_leader)과 주변 6대(_near)를 이것 하나로 합침 → 관측 107차원
N_SECT, N_SECT_F, SECT_RANGE, SECT_LEN_NORM = 8, 7, 50.0, 15.0
SIDE_MARGIN, LANE_W = 2.5, 3.2   # 옆 칸 = 내 몸체 ± 2.5 m, 차로 폭 (교차로 안 근사용)
# 2026-11-21 C: 8칸·교차 목록 대신 물체 목록 (차 1대 = 카드 1장, 한 차는 한 장에만 → 겹침 없음)
OBS_KIND = "objects"
N_OBJ, N_OBJ_F = 10, 12
OBJ_LEN_NORM = 15.0
EXTRA_DIM = 5 + 4 + 3 + N_ARROW_LANES * 3 + N_OBJ * N_OBJ_F

_NET_CACHE = {}


def _net(path):
    if path not in _NET_CACHE:
        import sumolib
        _NET_CACHE[path] = sumolib.net.readNet(path, withInternal=True)
    return _NET_CACHE[path]


_OCC_CACHE = {}
OCC_TYPES = ("building", "shop", "school")   # 시야를 가리는 다각형 종류 (GUI 그림 파일의 건물·상가·학교)
VEH_OCC_R = 1.0                               # 다른 차가 시선을 가리는 반경 (m, 차 폭 절반 정도)


def _occluders(map_dir):
    """맵의 건물 다각형 STRtree (2026-11-17: 가려짐 반영 — 건물 너머 차는 안 보인다)."""
    if map_dir not in _OCC_CACHE:
        import glob
        import xml.etree.ElementTree as ET
        from shapely.geometry import Polygon
        from shapely.strtree import STRtree
        polys = []
        for f in glob.glob(os.path.join(map_dir, "*.add.xml")):
            try:
                root = ET.parse(f).getroot()
            except ET.ParseError:
                continue
            for p in root.iter("poly"):
                if p.get("type") not in OCC_TYPES:
                    continue
                if p.get("id", "").endswith(("_site", "_fence")):
                    continue      # 2026-11-19: 학교·어린이집 부지 경계·울타리는 시야를 막지 않음 (부지 안을 도로가 지나가 그 도로에서 전부 가려지던 문제)
                pts = [tuple(map(float, xy.split(","))) for xy in p.get("shape", "").split()]
                if len(pts) >= 3:
                    pg = Polygon(pts)
                    if pg.is_valid and pg.area > 4.0:
                        polys.append(pg)
        _OCC_CACHE[map_dir] = (STRtree(polys), polys)
    return _OCC_CACHE[map_dir]


def _line_of_sight(tree, polys, a, b):
    """a → b 시선이 건물에 막히지 않는지."""
    from shapely.geometry import LineString
    seg = LineString([a, b])
    for i in tree.query(seg):
        if polys[i].intersects(seg):
            return False
    return True


def _blocked_by_vehicle(a, b, centers):
    """a → b 시선이 다른 차(중심 centers, 반경 VEH_OCC_R)에 막히는지 (b 보다 가까운 차만)."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-6:
        return False
    for cx, cy in centers:
        t = ((cx - ax) * dx + (cy - ay) * dy) / L2
        if 0.05 < t < 0.95:
            px, py = ax + t * dx - cx, ay + t * dy - cy
            if px * px + py * py < VEH_OCC_R * VEH_OCC_R:
                return True
    return False


def _sig_class(c):
    if c in "rRs":
        return "red"
    if c in "yYu":
        return "yellow"
    if c in "Gg":
        return "green"
    return None


def _heading(shape, end):
    """도로 모양의 시작(end=False) / 끝(end=True) 진행 방향 (라디안). 끝부분 10 m 정도로 잰다."""
    pts = list(shape) if not end else list(reversed(shape))
    p0, acc, p1 = pts[0], 0.0, pts[1]
    for q in pts[1:]:
        p1 = q
        acc = math.dist(p0, q)
        if acc >= 10.0:
            break
    dx, dy = (p1[0] - p0[0], p1[1] - p0[1]) if not end else (p0[0] - p1[0], p0[1] - p1[1])
    return math.atan2(dy, dx)


def _turn(h0, h1):
    """방향 변화 (°, 왼쪽 + / 오른쪽 -)."""
    return (math.degrees(h1 - h0) + 180.0) % 360.0 - 180.0


class UrbanDriving:
    def __init__(self, env, cfg, net_path):
        self.env = env
        self.cfg = dict(cfg)
        self.net = _net(net_path)
        self.comfort_decel = float(self.cfg.get("comfort_decel", 3.0))
        self.brake = float(self.cfg.get("mask_decel", env.max_decel))
        self.stop_margin = float(self.cfg.get("stop_margin", 1.0))
        self.min_gap = float(self.cfg.get("min_gap", 2.0))
        self.yield_window = float(self.cfg.get("yield_window", 2.0))
        self.junction_min_speed = float(self.cfg.get("junction_min_speed", 5.0))
        self.rw = dict(self.cfg.get("reward", {}))
        self.cmap = ConflictMap(self.net)
        self.occ_tree, self.occ_polys = _occluders(os.path.dirname(net_path))
        self._dir_cache = {}
        self._ego_link_major = True
        # 안전 마스킹 켜기/끄기. 끄면 적용은 안 하고 "켜져 있었다면 개입했을" 이유만 기록한다
        # (평가: 학습된 정책만의 실력을 보기 위해 test.py --no-mask)
        self.mask_enabled = bool(self.cfg.get("mask_enabled", True))
        self.MASK_TYPES = ("red_light", "yield", "rear_end", "junction_clear")

    # ------------------------------------------------------------------
    def reset(self):
        """경로의 회전 지점(경로 시작부터 m)과 경로 길이를 미리 계산한다."""
        import traci
        import traci.constants as tc
        self._prev_obj_gap = None              # 11-24: junction_close 보상용 (지난 스텝 최소 몸체 거리)
        edges = self.env.ego_route_edges
        pos, man = 0.0, []
        # 회전 판정은 SUMO 의 연결 방향 표시가 아니라 실제 도로 각도로 한다:
        #   - 연결 전후 진행 방향이 TURN_DEG 이상 바뀌면 좌/우회전 (차로가 늘어나는 지점처럼
        #     살짝 틀어진 연결을 SUMO 가 "우회전"으로 표시하는 경우를 걸러냄)
        #   - 유턴은 SUMO 표시(t) 그대로
        #   - 짧은 도로(SLIP_MAX_LEN 이하)가 그 안에서 SLIP_DEG 이상 휘면 그 도로 시작을 회전으로
        #     (우회전 전용 도로처럼 "직진"으로 갈라진 뒤 길 자체가 휘는 경우)
        for i, (a, b) in enumerate(zip(edges, edges[1:])):
            ea, eb = self.net.getEdge(a), self.net.getEdge(b)
            pos += ea.getLength()
            cons = ea.getConnections(eb)
            d = cons[0].getDirection().lower() if cons else "s"
            both_ring = a.startswith("ring") and b.startswith("ring")
            via = cons[0].getViaLaneID() if cons else ""
            if not both_ring:
                if d == "t":
                    man.append((pos, 2))
                else:
                    delta = _turn(_heading(ea.getShape(), end=True), _heading(eb.getShape(), end=False))
                    if abs(delta) >= TURN_DEG:
                        man.append((pos, 0 if delta > 0 else 1))
                    else:
                        sh = eb.getShape()
                        inner = _turn(_heading(sh, end=False), _heading(sh, end=True))
                        if eb.getLength() <= SLIP_MAX_LEN and abs(inner) >= SLIP_DEG:
                            man.append((pos + (self.net.getLane(via).getLength() if via else 0.0),
                                        0 if inner > 0 else 1))
            if via:
                pos += self.net.getLane(via).getLength()
        pos += self.net.getEdge(edges[-1]).getLength()
        self.maneuvers = man
        self.route_len = pos
        self.prev_odo = 0.0
        self.last = {}
        self.prev_v, self.prev_a = None, 0.0       # 승차감 (저크) 계산용
        self.comfort = {"n": 0, "jerk_sum": 0.0, "jerk_max": 0.0, "jerk_over": 0, "hard_acc": 0, "hard_brake": 0}
        self._ego_link_major = True
        self._prev_tls = None
        self.stop_steps = 0
        self.ep = {"steps": 0, "intervention_steps": 0, "would_steps": 0, "red_violations": 0,
                   **{f"int_{k}": 0 for k in self.MASK_TYPES}, **{f"would_{k}": 0 for k in self.MASK_TYPES}}
        traci.vehicle.subscribeContext(self.env.ego_id, tc.CMD_GET_VEHICLE_VARIABLE, NEAR_RADIUS,
                                       [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ANGLE, tc.VAR_LENGTH, tc.VAR_WIDTH])

    # ------------------------------------------------------------------
    def _next_tls(self):
        """앞 신호 (tl, 정지선까지 거리, ego 에게 유효한 상태, 링크 번호).

        - 지금 지나고 있는 교차로의 신호는 건너뛴다 (교차로 안에서 신호가 바뀌어도 빠져나가야 함)
        - 좌회전·유턴은 보호 신호(G, 화살표)가 있는 교차로면 그 신호에서만 가능: 비보호 녹색(g)은 ego 에게 "적색"
          (한국 도로 규칙: 비보호 좌회전 표지가 없으면 좌회전은 좌회전 신호에서만).
          보호 현시가 아예 없는 연결(비보호 좌회전 교차로)은 녹색(g)에 양보하며 좌회전한다.
        """
        import traci
        ego = self.env.ego_id
        road = traci.vehicle.getRoadID(ego)
        inside = road[1:].rsplit("_", 1)[0] if road.startswith(":") else None
        for tl, idx, dist, state in traci.vehicle.getNextTLS(ego):
            if dist > SIG_RANGE:
                break
            if tl == inside:
                continue
            # 2026-11-17: SUMO 신호 상태 그대로 (보호 좌회전 교차로의 비보호 녹색을 적색으로 바꾸던 가공 제거)
            return tl, dist, state, idx
        return None

    def _has_protected(self, tl, idx):
        """이 연결에 보호 신호(G, 좌회전 화살표)가 있는 현시가 하나라도 있는지. 없으면 비보호 좌회전 교차로
        → 녹색(g)에 양보하며 좌회전 가능 (2026-11-15: SE_triangle_corner 좌회전은 보호 현시가 없어 ego 가 영원히 못 감)."""
        key = ("prot", tl, idx)
        if key not in self._dir_cache:
            import traci
            prot = False
            for prog in traci.trafficlight.getAllProgramLogics(tl):
                for ph in prog.phases:
                    if idx < len(ph.state) and ph.state[idx] == "G":
                        prot = True
            self._dir_cache[key] = prot
        return self._dir_cache[key]

    def _link_dir(self, tl, idx):
        key = (tl, idx)
        if key not in self._dir_cache:
            import traci
            links = traci.trafficlight.getControlledLinks(tl)
            d = "s"
            if idx < len(links) and links[idx]:
                d = self.cmap.direction(links[idx][0][2])
            self._dir_cache[key] = d
        return self._dir_cache[key]

    def _lane_arrows(self, lane_id):
        """차로 하나의 화살표 (좌, 직, 우)와 그 화살표가 있는 정지선까지 더 가야 하는 거리 — 지도에서 계산해 캐시.
        도로가 중간에 잘려 다음 조각으로 직진 하나만 이어지는 곳은 따라가서, 방향이 실제로 갈리는 정지선의 화살표를 쓴다
        (예: 좌회전 전용 차로가 여러 조각으로 나뉘어 있어도 처음 조각부터 좌회전 화살표)."""
        key = ("arrow", lane_id)
        if key not in self._dir_cache:
            out, extra = [0.0, 0.0, 0.0], 0.0
            try:
                ln = self.net.getLane(lane_id)
                for _ in range(8):
                    conns = []
                    for c in ln.getOutgoing():
                        via = c.getViaLaneID()
                        if via:
                            try:
                                if not self.net.getLane(via).allows("passenger"):
                                    continue      # 일반 차는 못 쓰는 연결 (예: 좌회전 전용 차로의 직진 연결)
                            except Exception:
                                pass
                        conns.append(c)
                    # 직진 하나로만 이어지고 그 끝이 신호 교차로(정지선)가 아니면 같은 차로가 이어지는 조각 → 따라감
                    #   (옆에서 다른 길이 합류해도 내 차로는 그대로 이어지는 경우 포함)
                    if (len(conns) == 1 and conns[0].getDirection().lower() == "s"
                            and ln.getEdge().getToNode().getType() != "traffic_light"):
                        nxt = conns[0].getToLane()
                        extra += nxt.getLength()
                        ln = nxt
                        continue
                    for c in conns:
                        d = c.getDirection().lower()
                        if d in ("l", "t"):
                            out[0] = 1.0
                        elif d == "s":
                            out[1] = 1.0
                        elif d == "r":
                            out[2] = 1.0
                    break
            except Exception:
                pass
            self._dir_cache[key] = (out, extra)
        return self._dir_cache[key]

    def _arrows(self, lane, internal):
        import traci
        zeros = [0.0] * (N_ARROW_LANES * 3)
        if internal or not lane:
            return zeros
        try:
            rest = traci.lane.getLength(lane) - traci.vehicle.getLanePosition(self.env.ego_id)
            edge, k = lane.rsplit("_", 1)
            k = int(k)
            n = traci.edge.getLaneNumber(edge)
        except (traci.TraCIException, ValueError):
            return zeros
        out = []
        for off in self.env.obs_offsets:              # (+2, +1, 0, -1, -2) = 좌2 … 우2
            j = k + off
            if 0 <= j < n:
                arr, extra = self._lane_arrows(f"{edge}_{j}")
                out += arr if rest + extra <= ARROW_RANGE else [0.0, 0.0, 0.0]   # 정지선까지 100 m 안에서만 보임
            else:
                out += [0.0, 0.0, 0.0]
        return out

    def _visible(self, vid):
        """ego 앞범퍼에서 그 차까지 건물에 가려지지 않았는지 (차 앞·뒤 끝 중 하나라도 보이면 보임)."""
        import traci
        try:
            a = traci.vehicle.getPosition(self.env.ego_id)
            b = traci.vehicle.getPosition(vid)
            ang = math.radians(traci.vehicle.getAngle(vid))
            L = traci.vehicle.getLength(vid)
        except traci.TraCIException:
            return False
        rear = (b[0] - math.sin(ang) * L, b[1] - math.cos(ang) * L)
        return (_line_of_sight(self.occ_tree, self.occ_polys, a, b)
                or _line_of_sight(self.occ_tree, self.occ_polys, a, rear))

    def _foes(self, v_ego=None, observable=False):
        """교차로 상충 차량 (ego 경로와 교차·합류하는 교차로 내부 경로 위/앞의 차).

        SUMO getJunctionFoes 는 교차로에 들어가기 전에만 동작하고, 좌회전처럼 교차로 중간에
        대기 지점이 있는 경로의 두 번째 구간 상충을 놓친다. 그래서 맵의 내부 경로 모양으로
        교차점을 직접 계산한다 (ConflictMap). 반환: ego 교차점 거리 순 dict 목록
          ego_d: ego 앞범퍼 → 교차점 (m), foe_d: 상대 앞범퍼 → 교차점, foe_v, foe_len,
          ego_yield: ego 가 양보해야 하는지 (ego 링크가 보호/우선 신호가 아니면 True),
          committed: 상대가 이미 교차로 안이거나 교차점 전에 멈출 수 없는지
        observable=True (정책 관측용, 2026-11-17): 상대의 진로(경로)를 보지 않는다 — 상충 경로로 이어지는 진입 차로에
          있는 차는 어느 방향으로 갈지 모르므로 모두 후보. 건물에 가려진 차는 뺀다.
          False (강사·마스킹·보상용): 상대 진로까지 아는 시뮬레이터 정보
        """
        import traci
        ego = self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        pos = traci.vehicle.getLanePosition(ego)
        cm = self.cmap
        if lane.startswith(":"):
            cur, off = lane, -pos
        else:
            # getNextLinks: (다음 차로, 우선권, 열림, 상충, 내부 경로, 신호 상태, 방향, 길이)
            links = traci.vehicle.getNextLinks(ego)
            if not links or not links[0][4]:
                return []
            off = traci.lane.getLength(lane) - pos
            if off > FOE_RANGE:
                return []
            cur = links[0][4]
            self._ego_link_major = links[0][5] in "GM"
        ego_yield = not self._ego_link_major
        found = {}
        hop = 0
        while cur and cur.startswith(":") and hop < 6:
            for foe_lane, s_e, s_f, s_ec, s_fc in cm.conflicts(cur):
                d_e = off + s_e                                # ego 앞범퍼 → 겹침 시작점
                if off + s_ec < -3.0:                          # 중심선 교차점을 이미 3 m 넘게 지남
                    continue
                for vid, d_f in cm.vehicles_toward(foe_lane, s_f, use_intent=not observable):
                    if vid == ego:
                        continue
                    if observable and not self._visible(vid):
                        continue
                    try:
                        fv = traci.vehicle.getSpeed(vid)
                        fl = traci.vehicle.getLength(vid)
                    except traci.TraCIException:
                        continue
                    if d_f + (s_fc - s_f) < -(fl + 2.0):
                        continue                       # 이미 교차점을 지나감
                    committed = foe_lane == traci.vehicle.getLaneID(vid) or d_f <= fv * fv / 9.0 + 1.0
                    try:
                        fa = traci.vehicle.getAcceleration(vid) if not observable else 0.0   # 가속도: 강사용만
                    except traci.TraCIException:
                        fa = 0.0
                    rec = dict(id=vid, ego_d=d_e, foe_d=d_f, foe_v=fv, foe_len=fl, foe_a=fa,
                               ego_zone=2.0 * max(s_ec - s_e, 0.0), foe_zone=2.0 * max(s_fc - s_f, 0.0),
                               ego_yield=ego_yield, committed=committed)
                    if vid not in found or d_e < found[vid]["ego_d"]:
                        found[vid] = rec
            off += cm.length(cur)
            nxt_int = cm.next_internal(cur)
            if nxt_int is None and FOE_WALK:
                # 2026-11-21 공통 수정 2: 교차로를 빠져나간 뒤 짧은 토막 너머 다음 교차로까지 (내 경로 기준, FOE_RANGE m 안)
                #   (교차로 두 개가 붙어 있는 주차장 통로에서 다음 교차로의 엇갈리는 차를 들어가서야 보던 문제)
                out_lane = cm._dest.get(cur)
                nxt_int = None
                if out_lane and off + cm._llen.get(out_lane, 1e9) <= FOE_RANGE:
                    try:
                        rt = traci.vehicle.getRoute(ego); ri = traci.vehicle.getRouteIndex(ego)
                        oe = out_lane.rsplit("_", 1)[0]
                        k = list(rt).index(oe, max(ri, 0)) if oe in rt[max(ri, 0):] else -1
                        nxt_edge = rt[k + 1] if 0 <= k < len(rt) - 1 else None
                        if nxt_edge:
                            for lk in traci.lane.getLinks(out_lane):
                                if lk[0].rsplit("_", 1)[0] == nxt_edge and lk[4]:
                                    off += cm._llen.get(out_lane, 0.0)
                                    nxt_int = lk[4]
                                    ego_yield = not (lk[5] in "GM")
                                    break
                    except (traci.TraCIException, ValueError):
                        nxt_int = None
            cur = nxt_int
            hop += 1
        return sorted(found.values(), key=lambda f: f["ego_d"])

    # ------------------------------------------------------------------
    def leader_obs(self):
        """현재 차로 앞차 2칸 [범퍼 사이 간격/50 m, (앞차 속도-내 속도)/vmax] — 수업 원본 _neighbor(0, leader=True) 와 같은 정의.
        traci getLeader(50 m): 같은 차로(다음 도로로 이어지는 차로 포함)의 가장 가까운 앞차. 건물에 가려 안 보이면 없음.
        없으면 [1, 1] (원본과 같음). 고른 앞차 id 는 self.leader_id 에 두고 _near 에서 뺀다."""
        import traci
        env, ego = self.env, self.env.ego_id
        self.leader_id = None
        try:
            res = traci.vehicle.getLeader(ego, LEAD_RANGE)
        except traci.TraCIException:
            res = None
        if not res or not res[0] or res[1] < 0 or res[1] > LEAD_RANGE:
            return [1.0, 1.0]
        vid, gap = res
        try:
            ex, ey = traci.vehicle.getPosition(ego)
            x, y = traci.vehicle.getPosition(vid)
            b = math.radians(traci.vehicle.getAngle(vid))
            Lv = traci.vehicle.getLength(vid)
            ends = ((x, y), (x - math.sin(b) * Lv, y - math.cos(b) * Lv))
            if not any(_line_of_sight(self.occ_tree, self.occ_polys, (ex, ey), e) for e in ends):
                return [1.0, 1.0]                      # 굽은 길에서 건물에 가림
            lv, v = traci.vehicle.getSpeed(vid), traci.vehicle.getSpeed(ego)
        except traci.TraCIException:
            return [1.0, 1.0]
        self.leader_id = vid
        return [float(np.clip(gap / LEAD_RANGE, 0.0, 1.0)), float(np.clip((lv - v) / env.vmax, -1.0, 1.0))]

    def features(self):
        import traci
        env, ego = self.env, self.env.ego_id
        vmax = env.vmax
        f = []
        sect_f = self._sectors()           # 8방향 물체 인식 (가시성 판정 → self.visible, 고른 차 → self.sector_ids)
        # 신호
        nt = self._next_tls()
        if nt:
            tl, dist, st = nt[:3]
            cls = _sig_class(st)
            f += [1.0, dist / SIG_RANGE, float(cls == "red"), float(cls == "yellow"), float(cls == "green")]
        else:
            f += [0.0, 1.0, 0.0, 0.0, 0.0]
        self.last["tls"] = nt
        # 경로: 다음 회전
        odo = traci.vehicle.getDistance(ego)
        nxt = next(((p - odo, k) for p, k in self.maneuvers if p - odo > -5.0), None)
        if nxt and nxt[0] <= MAN_RANGE:
            oh = [0.0, 0.0, 0.0]
            oh[nxt[1]] = 1.0
            f += [max(nxt[0], 0.0) / MAN_RANGE] + oh
        else:
            f += [1.0, 0.0, 0.0, 0.0]
        # ego 추가
        lane = traci.vehicle.getLaneID(ego)
        vlim = traci.lane.getMaxSpeed(lane) if lane else vmax
        internal = lane.startswith(":")
        # 2026-11-17: "경로에 맞는 차로까지 차로 수"(차선 단위 경로 정보)는 넣지 않는다 (컨설팅: 큰 틀 라우팅만 허용)
        f += [min(vlim / vmax, 1.0), float(internal), float(np.clip(odo / max(self.route_len, 1.0), 0, 1))]
        self.last["vlim"] = vlim
        # 차로별 노면 화살표 (2026-11-17): 내 차로와 양옆 2차로씩 (좌2·좌1·현재·우1·우2) 각각 좌회전·직진·우회전 가능 여부.
        #   정지선에 그려진 화살표 = 지도의 차로 연결 방향 (일반 승용차가 못 쓰는 연결은 뺌, 유턴은 좌회전 화살표로).
        #   내 경로와 무관한 도로 표시 정보 (어느 차로로 가라는 정보가 아님). 정지선까지 ARROW_RANGE m 안에서만 보이고,
        #   멀거나 교차로 안이면 0
        f += self._arrows(lane, internal)
        # 2026-11-21 C: 물체 목록 (보이는 차만; 상대 진로·"양보해야 하나" 같은 판단은 넣지 않음)
        f += self._objects()
        return f

    def _sectors(self):
        """8칸 물체 인식 56칸 (3차로 × 앞·옆·뒤). 칸 순서: 0 앞, 1 앞왼, 2 왼, 3 뒤왼, 4 뒤, 5 뒤오, 6 오, 7 앞오 (왼쪽 +)."""
        import traci
        import traci.constants as tc
        from shapely.geometry import Polygon, LineString, Point
        from shapely.ops import nearest_points
        from shapely.strtree import STRtree
        env, ego = self.env, self.env.ego_id
        self.visible, self.sector_ids, self._ctx_ids = set(), set(), set()
        self._geo = None
        out = [0.0] * (N_SECT * N_SECT_F)
        res = traci.vehicle.getContextSubscriptionResults(ego) or {}
        me = res.get(ego)
        if not me:
            return out

        def box(d):
            x, y = d[tc.VAR_POSITION]
            b = math.radians(d[tc.VAR_ANGLE])
            hx, hy = math.sin(b), math.cos(b)
            L, w = d.get(tc.VAR_LENGTH, 5.0), d.get(tc.VAR_WIDTH, 1.8) / 2
            rx, ry = x - hx * L, y - hy * L
            pts = [(x - hy * w, y + hx * w), (x + hy * w, y - hx * w), (rx + hy * w, ry - hx * w), (rx - hy * w, ry + hx * w)]
            return pts, ((x + rx) / 2, (y + ry) / 2)

        ego_pts, eye = box(me)
        ego_poly = Polygon(ego_pts)
        a = math.radians(me[tc.VAR_ANGLE])
        hx, hy = math.sin(a), math.cos(a)        # 진행 방향
        lx, ly = -hy, hx                         # 왼쪽
        ev = me[tc.VAR_SPEED]
        ids, polys, cens, pts_of = [], [], {}, {}
        for vid, d in res.items():
            if vid == ego:
                continue
            pts, c = box(d)
            ids.append(vid); polys.append(Polygon(pts)); cens[vid] = c; pts_of[vid] = pts
        self._ctx_ids = set(ids)
        if not ids:                     # 주변 50 m 에 차가 없어도 멀리서 엇갈려 오는 차는 물체 목록에 들어갈 수 있게
            self._geo = dict(res=res, ego_poly=ego_poly, polys=[], idx={}, hx=hx, hy=hy, lx=lx, ly=ly, ev=ev,
                             vis=set(), path_lead=None)
            return out
        tree = STRtree(polys)
        idx = {vid: i for i, vid in enumerate(ids)}
        for vid in ids:
            i = idx[vid]
            seen = False
            for e in pts_of[vid] + [cens[vid]]:
                if not _line_of_sight(self.occ_tree, self.occ_polys, eye, e):
                    continue
                ray = LineString([eye, e])
                if any(j != i and polys[j].intersects(ray) for j in tree.query(ray)):
                    continue
                seen = True
                break
            if seen:
                self.visible.add(vid)
        # ---- 칸 배정: 차로를 따라가는 좌표 (실제 자율주행의 정밀 지도 차로 배정과 같은 방식) ----
        #   칸 = 3차로(왼·내·오) × 앞·옆·뒤. 옆 = 내 차 몸체 ± SIDE_MARGIN m 와 겹치는 차. 앞·뒤 = 그 밖 SECT_RANGE m 까지.
        #   같은 도로: 차로 번호 차이 + 차로 위 위치 차이. 다음·이전 도로: 수업 원본과 같은 SUMO getLeader/getFollower/getNeighbors
        #   (차로를 따라 도로 경계를 넘어 찾음). 교차로 안(내부 차로)에 있을 때만 내 차 기준 좌표로 나눔.
        #   칸마다 "보이는 차 중 가장 가까운 1대". 가려진 차는 후보에서 이미 빠져 있음.
        def is_road_obj(vid):
            try:
                return not traci.vehicle.isStoppedParking(vid)       # 주차 칸 안의 차 = 도로 밖
            except traci.TraCIException:
                return False
        vis = {vid for vid in self.visible if is_road_obj(vid)}
        lane_id = traci.vehicle.getLaneID(ego)
        Le = me.get(tc.VAR_LENGTH, 5.0)
        cand = {}          # vid -> (cell, 거리)
        def put(vid, cell, dist):
            if vid in vis and (vid not in cand or dist < cand[vid][1]):
                cand[vid] = (cell, dist)
        FRONT = {0: 0, 1: 1, -1: 7}; SIDE = {1: 2, -1: 6}; REAR = {0: 4, 1: 3, -1: 5}
        if lane_id and not lane_id.startswith(":"):
            edge, k = traci.vehicle.getRoadID(ego), traci.vehicle.getLaneIndex(ego)
            p = traci.vehicle.getLanePosition(ego)
            n = traci.edge.getLaneNumber(edge)
            mg = traci.vehicle.getMinGap(ego)
            for col in (-1, 0, 1):                                   # SUMO: +1 = 왼쪽 차로
                li = k + col
                if not 0 <= li < n:
                    continue
                for vid in traci.lane.getLastStepVehicleIDs(f"{edge}_{li}"):
                    if vid == ego:
                        continue
                    po = traci.vehicle.getLanePosition(vid)
                    lo = traci.vehicle.getLength(vid)
                    front_gap = (po - lo) - p                        # 상대 뒤범퍼 - 내 앞범퍼
                    rear_gap = (p - Le) - po                         # 내 뒤범퍼 - 상대 앞범퍼
                    if col != 0 and front_gap < SIDE_MARGIN and rear_gap < SIDE_MARGIN:
                        put(vid, SIDE[col], max(front_gap, rear_gap, 0.0))
                    elif front_gap >= -0.1 and front_gap <= SECT_RANGE:
                        put(vid, FRONT[col], max(front_gap, 0.0))
                    elif rear_gap >= -0.1 and rear_gap <= SECT_RANGE:
                        put(vid, REAR[col], max(rear_gap, 0.0))
            # 도로 경계 너머 (수업 원본 _neighbor 와 같은 SUMO 함수)
            try:
                r = traci.vehicle.getLeader(ego, SECT_RANGE)
                if r and r[0]:
                    put(r[0], 0, r[1] + mg)
                r = traci.vehicle.getFollower(ego, SECT_RANGE)
                if r and r[0]:
                    put(r[0], 4, r[1])
                for col, (m_lead, m_foll) in ((1, (0b10, 0b00)), (-1, (0b11, 0b01))):
                    for vid, g in traci.vehicle.getNeighbors(ego, m_lead):
                        if 0 <= g + mg <= SECT_RANGE and vid not in cand:
                            put(vid, FRONT[col] if g + mg >= SIDE_MARGIN else SIDE[col], g + mg)
                    for vid, g in traci.vehicle.getNeighbors(ego, m_foll):
                        if 0 <= g <= SECT_RANGE and vid not in cand:
                            put(vid, REAR[col] if g >= SIDE_MARGIN else SIDE[col], g)
            except traci.TraCIException:
                pass
            # 보완 (2026-11-20 사용자 승인): 위에서 못 찾은 보이는 차 중 나와 같은 방향(±45°)으로 가는 차는 내 차 기준
            #   좌우 띠로 칸에 넣는다. 교차로를 막 빠져나왔을 때 옆·뒤 차가 아직 교차로 안쪽 구간에 있으면 차로 훑기와
            #   getNeighbors 가 놓쳐 "옆 차로 빔"으로 보이던 문제 (그 순간 차로를 바꾸다 옆 차와 부딪힘). 맞은편 차로 차는 방향으로 걸러냄.
            for vid in vis:
                if vid in cand:
                    continue
                dang = (res[vid][tc.VAR_ANGLE] - me[tc.VAR_ANGLE] + 180.0) % 360.0 - 180.0
                if abs(dang) > 45.0:
                    continue
                p_e, p_o = nearest_points(ego_poly, polys[idx[vid]])
                gx, gy = p_o.x - p_e.x, p_o.y - p_e.y
                lon = gx * hx + gy * hy
                cx, cy = cens[vid][0] - eye[0], cens[vid][1] - eye[1]
                clat = cx * lx + cy * ly
                col = 0 if abs(clat) < LANE_W / 2 else (1 if clat > 0 else -1) if abs(clat) < LANE_W * 1.5 else None
                if col is None or abs(lon) > SECT_RANGE:
                    continue
                if col != 0 and abs(lon) < SIDE_MARGIN:
                    put(vid, SIDE[col], abs(lon))
                elif lon > 0:
                    put(vid, FRONT[col], lon)
                else:
                    put(vid, REAR[col], -lon)
        else:                                                         # 교차로 안: 내 차 기준 좌표
            for vid in vis:
                p_e, p_o = nearest_points(ego_poly, polys[idx[vid]])
                gx, gy = p_o.x - p_e.x, p_o.y - p_e.y
                lon, lat = gx * hx + gy * hy, gx * lx + gy * ly
                cx, cy = cens[vid][0] - eye[0], cens[vid][1] - eye[1]
                clat = cx * lx + cy * ly
                col = 0 if abs(clat) < LANE_W / 2 else (1 if clat > 0 else -1) if abs(clat) < LANE_W * 1.5 else None
                if col is None:
                    continue
                if col != 0 and abs(lon) < SIDE_MARGIN:
                    put(vid, SIDE[col], abs(lon))
                elif lon > 0:
                    put(vid, FRONT[col], lon)
                else:
                    put(vid, REAR[col], -lon)
        # 앞 칸 = 내 경로상 앞차 (모든 구간, 2026-11-20 사용자 승인). 수업 원본 _neighbor 와 같은 getLeader:
        #   내 차로를 따라가다 교차로에서는 내 경로가 이어지는 쪽으로 SECT_RANGE m 까지 찾는다 (좌·우회전 중에도 갈 길 앞의 차).
        #   보이는 차일 때만 넣고, 가려졌거나 없으면 위의 칸 배정 결과를 그대로 쓴다.
        path_lead = None
        try:
            r = traci.vehicle.getLeader(ego, SECT_RANGE)
            if r and r[0] and r[0] in vis:
                g = max(r[1] + traci.vehicle.getMinGap(ego), 0.0)
                if g <= SECT_RANGE:
                    path_lead = (r[0], g)
        except traci.TraCIException:
            pass
        chosen = {}
        if path_lead:
            cand.pop(path_lead[0], None)
            chosen[0] = path_lead
        for vid, (cell, dist) in sorted(cand.items(), key=lambda kv: kv[1][1]):
            if cell not in chosen:
                chosen[cell] = (vid, dist)
        for cell, (vid, dist) in chosen.items():
            d = res[vid]
            p_e, p_o = nearest_points(ego_poly, polys[idx[vid]])
            gx, gy = p_o.x - p_e.x, p_o.y - p_e.y
            lat = gx * lx + gy * ly
            sign = -1.0 if cell in (3, 4, 5) else 1.0
            lon = 0.0 if cell in (2, 6) else sign * dist                # 앞·뒤 = 차로를 따라 잰 몸체 사이 거리, 옆 = 0
            b = math.radians(d[tc.VAR_ANGLE])
            vx, vy = d[tc.VAR_SPEED] * math.sin(b) - ev * hx, d[tc.VAR_SPEED] * math.cos(b) - ev * hy
            try:
                is_ev = traci.vehicle.getVehicleClass(vid) == "emergency"
            except traci.TraCIException:
                is_ev = False
            f = [1.0, lon / SECT_RANGE, lat / SECT_RANGE,
                 (vx * hx + vy * hy) / env.vmax, (vx * lx + vy * ly) / env.vmax,
                 min(d.get(tc.VAR_LENGTH, 5.0) / SECT_LEN_NORM, 1.0), 1.0 if is_ev else 0.0]
            out[cell * N_SECT_F:(cell + 1) * N_SECT_F] = [float(np.clip(x, -1, 1)) for x in f]
            self.sector_ids.add(vid)
        chosen = {c: v for c, (v, _) in chosen.items()}
        self.last["sectors"] = {sec: vid for sec, vid in chosen.items()}
        self._geo = dict(res=res, ego_poly=ego_poly, polys=polys, idx=idx, hx=hx, hy=hy, lx=lx, ly=ly, ev=ev,
                         vis=vis, path_lead=path_lead[0] if path_lead else None)
        return out

    def _objects(self):
        """C 관측: 보이는 차 목록 N_OBJ 대 × N_OBJ_F 값 (빈 자리는 "없음").
        대상: 50 m(SECT_RANGE) 안의 보이는 도로 위 차 + 120 m 안에서 내 경로와 엇갈리는 보이는 차.
        값: 있음, 앞뒤·좌우 몸체 간격/50, 앞뒤·좌우 상대속도/vmax, 상대 방향 sin·cos, 길이, 긴급차,
            내 경로상 앞차(getLeader), 엇갈림까지 내 거리·상대 거리/FOE_RANGE (엇갈리지 않으면 1, 1).
        순서: 경로상 앞차 → 엇갈리는 차(내 엇갈림 거리 순) → 나머지(거리 순). 신경망이 순서와 무관하게 읽으므로
            순서는 10대를 넘을 때 누구를 남길지만 정한다."""
        import traci
        import traci.constants as tc
        from shapely.geometry import Polygon
        from shapely.ops import nearest_points
        out = [0.0] * (N_OBJ * N_OBJ_F)
        g = getattr(self, "_geo", None)
        self.obj_ids = []
        self.sector_ids = set()
        self.last["obj_min_gap"] = None          # 11-24: 관측에 들어간 차와 몸체 사이 최소 거리 (보상 junction_close 용)
        if not g:
            self.last["foes"] = []
            return out
        env, ego = self.env, self.env.ego_id
        res, ego_poly, polys, idx = g["res"], g["ego_poly"], g["polys"], g["idx"]
        hx, hy, lx, ly, ev = g["hx"], g["hy"], g["lx"], g["ly"], g["ev"]
        me = res.get(ego)
        ea = math.radians(me[tc.VAR_ANGLE]) if me else 0.0
        try:
            foes = self._foes(observable=True)
        except traci.TraCIException:
            foes = []
        # 50 m 안인데 차에 가려진 차는 뺌 (먼 차는 건물 가림만 검사 — 기존 교차 목록과 같은 기준)
        foes = [fo for fo in foes if not (fo["id"] in self._ctx_ids and fo["id"] not in self.visible)]
        self.last["foes"] = foes
        conf = {}
        for fo in foes:
            conf.setdefault(fo["id"], fo)
        cand = {}
        for vid in g["vis"]:
            p_e, p_o = nearest_points(ego_poly, polys[idx[vid]])
            if p_e.distance(p_o) <= SECT_RANGE:
                cand[vid] = p_e.distance(p_o)
        for vid in conf:
            if vid not in cand:
                cand[vid] = 1e3
        def prio(vid):
            if vid == g["path_lead"]:
                return (0, 0.0)
            if vid in conf:
                return (1, conf[vid]["ego_d"])
            return (2, cand[vid])
        order = sorted(cand, key=prio)[:N_OBJ]
        for k, vid in enumerate(order):
            try:
                if vid in res:
                    d = res[vid]
                    x, y = d[tc.VAR_POSITION]; ang = d[tc.VAR_ANGLE]; sp = d[tc.VAR_SPEED]
                    L = d.get(tc.VAR_LENGTH, 5.0); W = d.get(tc.VAR_WIDTH, 1.8)
                else:
                    x, y = traci.vehicle.getPosition(vid); ang = traci.vehicle.getAngle(vid)
                    sp = traci.vehicle.getSpeed(vid); L = traci.vehicle.getLength(vid); W = traci.vehicle.getWidth(vid)
                is_ev = traci.vehicle.getVehicleClass(vid) == "emergency"
            except traci.TraCIException:
                continue
            if vid in idx:
                poly = polys[idx[vid]]
            else:
                b = math.radians(ang); bx, by = math.sin(b), math.cos(b); w = W / 2
                rx, ry = x - bx * L, y - by * L
                poly = Polygon([(x - by * w, y + bx * w), (x + by * w, y - bx * w), (rx + by * w, ry - bx * w), (rx - by * w, ry + bx * w)])
            p_e, p_o = nearest_points(ego_poly, poly)
            dmin = p_e.distance(p_o)
            if self.last["obj_min_gap"] is None or dmin < self.last["obj_min_gap"]:
                self.last["obj_min_gap"] = dmin
            gx, gy = p_o.x - p_e.x, p_o.y - p_e.y
            b = math.radians(ang)
            vx, vy = sp * math.sin(b) - ev * hx, sp * math.cos(b) - ev * hy
            rel = b - ea
            cf = conf.get(vid)
            f = [1.0, (gx * hx + gy * hy) / SECT_RANGE, (gx * lx + gy * ly) / SECT_RANGE,
                 (vx * hx + vy * hy) / env.vmax, (vx * lx + vy * ly) / env.vmax,
                 math.sin(rel), math.cos(rel), min(L / OBJ_LEN_NORM, 1.0), 1.0 if is_ev else 0.0,
                 1.0 if vid == g["path_lead"] else 0.0,
                 (cf["ego_d"] / FOE_RANGE) if cf else 1.0, (cf["foe_d"] / FOE_RANGE) if cf else 1.0]
            out[k * N_OBJ_F:(k + 1) * N_OBJ_F] = [float(np.clip(v_, -1, 1)) for v_ in f]
            self.obj_ids.append(vid)
        self.sector_ids = set(self.obj_ids)
        return out

    def _near(self):
        import traci
        import traci.constants as tc
        env, ego = self.env, self.env.ego_id
        res = traci.vehicle.getContextSubscriptionResults(ego) or {}
        me = res.get(ego)
        out = []
        self.visible = set()           # 이번 스텝에 보이는 차 (가려짐 통과) — 기본 관측의 차로 밀도도 이것만 센다
        if me:
            ex, ey = me[tc.VAR_POSITION]
            a = math.radians(me[tc.VAR_ANGLE])
            hx, hy = math.sin(a), math.cos(a)          # SUMO 각도: 북쪽 0°, 시계 방향
            lx, ly = -hy, hx                            # 왼쪽
            ev = me[tc.VAR_SPEED]
            # 가려짐 (2026-11-17): 건물에 가리거나, 더 가까운 다른 차가 시선을 막으면 안 보인다
            #   (차 앞·뒤 끝 중 하나라도 보이면 보임). 다른 차의 중심 = 앞범퍼에서 진행 반대로 길이 절반
            cen = {}
            for vid, d in res.items():
                if vid == ego:
                    continue
                b = math.radians(d[tc.VAR_ANGLE])
                x, y = d[tc.VAR_POSITION]
                h = d.get(tc.VAR_LENGTH, 5.0) / 2
                cen[vid] = (x - math.sin(b) * h, y - math.cos(b) * h)
            others = []
            for vid, d in res.items():
                if vid == ego:
                    continue
                x, y = d[tc.VAR_POSITION]
                b = math.radians(d[tc.VAR_ANGLE])
                Lv = d.get(tc.VAR_LENGTH, 5.0)
                ends = ((x, y), (x - math.sin(b) * Lv, y - math.cos(b) * Lv))
                block = [c for k, c in cen.items() if k != vid]
                if not any(_line_of_sight(self.occ_tree, self.occ_polys, (ex, ey), e)
                           and not _blocked_by_vehicle((ex, ey), e, block) for e in ends):
                    continue
                self.visible.add(vid)
                try:
                    if traci.vehicle.isStoppedParking(vid):
                        continue                        # 주차 구역 칸에 들어가 있는 차 (도로 밖) — 장애물 아님
                    is_ev = traci.vehicle.getVehicleClass(vid) == "emergency"
                except traci.TraCIException:
                    continue
                if vid == getattr(self, "leader_id", None) and not is_ev:
                    continue                            # 현재 차로 앞차는 앞차 2칸(leader_obs)에서 봄 → 주변 6대에서 제외 (겹침 없음)
                x, y = d[tc.VAR_POSITION]
                dx, dy = x - ex, y - ey
                b = math.radians(d[tc.VAR_ANGLE])
                vx, vy = d[tc.VAR_SPEED] * math.sin(b) - ev * hx, d[tc.VAR_SPEED] * math.cos(b) - ev * hy
                others.append((not is_ev, dx * dx + dy * dy, dx * hx + dy * hy, dx * lx + dy * ly,
                               vx * hx + vy * hy, vx * lx + vy * ly))
            others.sort()      # 반경 안 긴급차는 먼저, 나머지는 가까운 순 (긴급차가 7번째로 밀려 안 보이는 일 방지)
            for o in others[:N_NEAR]:
                out += [1.0, o[2] / NEAR_RADIUS, o[3] / NEAR_RADIUS,
                        float(np.clip(o[4] / env.vmax, -1, 1)), float(np.clip(o[5] / env.vmax, -1, 1)),
                        0.0 if o[0] else 1.0]
        out += [0.0] * N_NEAR_F * (N_NEAR - len(out) // N_NEAR_F)
        return out

    # ------------------------------------------------------------------
    def mask(self, v, v_cmd, brake=None, margin=None, skip_signal=False, start_extra=0.0):
        """요청 속도 v_cmd 를 확실히 위험한 경우에만 줄인다. (적용 속도, 개입 이유 목록)

        brake / margin: 정지 계산에 쓸 제동 감속도(m/s²)와 정지선 앞 여유(m). 생략하면 설정값.
        규칙 기반 강사(env/expert_driver)는 가속 명령으로 운전해 반응이 한 스텝 늦으므로
        더 편안한 제동·큰 여유로 부른다.
        """
        import traci
        ego, dt = self.env.ego_id, self.env.step_length
        b_stop = float(brake) if brake else self.brake
        m_stop = float(margin) if margin is not None else self.stop_margin
        caps = []
        why_floor = []
        # 교차로 안에서는 멈추거나 머뭇거리지 않는다: 앞이 비어 있으면 최소 속도로 빠져나간다.
        # (교차로 안에서 기어가다 신호가 바뀌면 다른 방향 차와 부딪힌다. 아래 추돌 제한이 우선)
        if traci.vehicle.getRoadID(ego).startswith(":"):
            lead = traci.vehicle.getLeader(ego, 15.0)
            floor = min(self.junction_min_speed, v + self.env.max_accel * dt)
            if (not lead or not lead[0]) and v_cmd < floor:
                v_cmd = floor
                why_floor.append("junction_clear")

        def stop_cap(d):          # 거리 d 앞에서 멈출 수 있는 최대 속도 (반응 지연 1스텝 포함, Gipps 식)
            d = max(d, 0.0)
            return max(0.0, -b_stop * dt + math.sqrt((b_stop * dt) ** 2 + 2.0 * b_stop * d))

        nt = None if skip_signal else self._next_tls()
        if nt:
            tl, dist, st = nt[:3]
            cls = _sig_class(st)
            d = dist - m_stop
            if cls == "red" or (cls == "yellow" and v * v / (2 * self.comfort_decel) < d):
                caps.append((stop_cap(d), "red_light"))
        ego_len = traci.vehicle.getLength(ego)
        # 양보해야 할 때는 상충점이 아니라 교차로 진입선(지금 차로 끝) 앞에서 선다. 상충점이 교차로 안쪽 깊이
        # 있으면 "상충점 3 m 앞"은 이미 교차로 안이라 큰길 차와 부딪힘 (마전 T자 합류부, 2026-11-16)
        lane_now = traci.vehicle.getLaneID(ego)
        d_entry = (traci.lane.getLength(lane_now) - traci.vehicle.getLanePosition(ego)
                   if lane_now and not lane_now.startswith(":") else None)
        for fo in self._foes():
            d_e = fo["ego_d"]
            if d_e < 0.5:
                continue                 # 이미 겹침 구역에 들어섬: 멈추면 오히려 막고 서므로 그대로 빠져나감
            if d_e < 1.5 and fo["foe_v"] < 0.5 and fo["foe_d"] > 1.5:
                continue                 # 겹침 구역 바로 앞에 섰는데 상대도 구역 밖(앞)에 서 있음: 서로 기다리는 교착 → 천천히 지나감.
                                         # 상대가 구역 안이나 바로 앞에 서 있으면(줄이 밀려 교차로 안에 선 차) 비켜날 때까지 기다림 (2026-11-20)
            fv = max(fo["foe_v"], 0.5)
            fa = fo.get("foe_a", 0.0)
            zf = fo.get("foe_zone", 0.0)

            def t_reach(d):              # 상대가 거리 d 를 가는 시각. 가속 중이면 가속을 반영 (막 출발한 차를 늦게 온다고 착각하지 않게, 2026-11-20)
                d = max(d, 0.0)
                if fa > 0.1:
                    v_top = max(self.env.vmax, fv)
                    t1 = (v_top - fv) / fa
                    d1 = fv * t1 + 0.5 * fa * t1 * t1
                    if d <= d1:
                        return (-fv + math.sqrt(fv * fv + 2.0 * fa * d)) / fa
                    return t1 + (d - d1) / v_top
                return d / fv
            t_f = t_reach(fo["foe_d"])                                 # 상대가 겹침 구역에 닿는 시각
            t_f_out = (fo["foe_d"] + zf + fo["foe_len"] + 2.0) / fv     # 상대가 겹침 구역을 벗어나는 시각
            # 내가 교차점에 닿는 시각은 요청 속도가 아니라 실제로 낼 수 있는 속도로 (요청 속도로 계산하면
            # 먼저 지나갈 수 있다고 착각해 늦게 서서 부딪힘 — 마전 T자 합류부, 2026-11-15)
            # 2026-11-20: 서 있거나 느릴 때 고정 속도로 계산하면 지나가는 데 한참 걸린다고 착각해 계속 양보하며 갇힘
            #   → 지금 속도에서 최대 가속으로 요청 속도까지 올리는 시간으로 계산
            v_top_e = max(min(v_cmd, self.env.vmax), 0.5)
            a_e = self.env.max_accel

            def t_ego(d):
                d = max(d, 0.0)
                v0 = min(v, v_top_e)
                t1 = max(v_top_e - v0, 0.0) / a_e
                d1 = v0 * t1 + 0.5 * a_e * t1 * t1
                if d <= d1:
                    return (-v0 + math.sqrt(v0 * v0 + 2.0 * a_e * d)) / a_e
                return t1 + (d - d1) / v_top_e
            t_e, t_e_out = t_ego(d_e), t_ego(d_e + fo.get("ego_zone", 0.0) + ego_len + 2.0)
            w = self.yield_window if fo["ego_yield"] else 0.5
            if v < 2.0:
                w += start_extra         # 2026-11-21 공통 수정 7 (강사만): 멈춰 있다 출발할 때 틈을 더 넉넉히
            overlap = t_e < t_f_out + w and t_f < t_e_out + w
            if overlap and (fo["ego_yield"] or fo["committed"]):
                d_stop = d_e - 1.0       # 겹침 시작점 1 m 앞 (예전: 중심선 교차점 3 m 앞)
                if fo["ego_yield"] and d_entry is not None and d_e - d_entry < 25.0:   # 바로 앞 교차로의 상충점일 때만
                    d_stop = min(d_stop, d_entry - 0.5)
                caps.append((stop_cap(d_stop), "yield"))
        lead = traci.vehicle.getLeader(ego, 100.0)
        if lead and lead[0]:
            # 앞차의 다음 스텝 속도(지금 감속 중이면 반영)와 마스킹 반응 지연(1스텝)을 고려해,
            # 앞차가 최대 제동해도 최소 간격이 남는 속도 (Gipps 식):
            #   v·dt + v²/(2b) ≤ gap − min_gap + lv²/(2b)
            lv = traci.vehicle.getSpeed(lead[0])
            la = min(traci.vehicle.getAcceleration(lead[0]), 0.0)
            lv = max(lv + la * dt, 0.0)
            gap, b = lead[1], self.brake
            room = gap - self.min_gap + lv * lv / (2 * b)
            cap = -b * dt + math.sqrt(max((b * dt) ** 2 + 2 * b * room, 0.0)) if room > 0 else 0.0
            caps.append((max(cap, 0.0), "rear_end"))
        v_apply, why = v_cmd, list(why_floor)
        for cap, name in caps:
            if v_apply > cap + 0.05:
                v_apply = cap
                why.append(name)
        return max(v_apply, 0.0), why

    # ------------------------------------------------------------------
    def after_step(self, applied, would):
        """시뮬레이션 1스텝 뒤: 신호 위반 감지 + 에피소드 개입 집계. 신호 위반이면 True.

        신호 통과 판정: 직전 스텝의 "다음 신호"가 20 m 안에 있었는데 지금은 사라졌거나 다른
        신호로 바뀌었으면 그 신호를 지난 것. 직전 스텝과 지금 모두 내 차로 신호가 (ego 기준)
        적색일 때만 위반 — 정지선에서 기다리다 녹색으로 바뀌는 바로 그 스텝에 출발해 넘는 것,
        좌회전 화살표가 황색 없이 비보호 녹색으로 바뀌는 순간에 지나는 것은 위반이 아니다.
        """
        import traci
        violation = False
        nt = self._next_tls() if self.env.ego_id in traci.vehicle.getIDList() else None
        p = self._prev_tls
        if p and p[1] < 20.0 and (not nt or nt[0] != p[0] or nt[1] > p[1] + 1.0) and p[2] in "rRs":
            now = traci.trafficlight.getRedYellowGreenState(p[0])[p[3]]
            violation = now in "rRs"
        self._prev_tls = nt
        e = self.ep
        e["steps"] += 1
        e["intervention_steps"] += bool(applied)
        e["would_steps"] += bool(would)
        e["red_violations"] += violation
        for k in applied:
            e[f"int_{k}"] += 1
        for k in would:
            e[f"would_{k}"] += 1
        return violation

    def wrong_lane_dead_end(self):
        """경로에 맞지 않는 차로로 차로 끝(3 m 안)에 도착했는지.

        SUMO 가 경로 유지 차선변경을 대신 하지 않으므로, 틀린 차로로 교차로에 다가가면
        다음 경로 도로로 이어지는 연결이 없어 그 자리에 갇힌다 → 경로 이탈 실패로 끝낸다.
        """
        import traci
        ego = self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        if not lane or lane.startswith(":"):
            return False
        if traci.lane.getLength(lane) - traci.vehicle.getLanePosition(ego) > 3.0:
            return False
        idx = traci.vehicle.getRouteIndex(ego)
        route = self.env.ego_route_edges
        if idx < 0 or idx + 1 >= len(route):
            return False
        nxt = route[idx + 1]
        return not any(l[0].rsplit("_", 1)[0] == nxt for l in traci.lane.getLinks(lane))

    def episode_summary(self):
        n = max(self.ep["steps"], 1)
        c = getattr(self, "comfort", None) or {"n": 0}
        cn = max(c.get("n", 0), 1)
        out = {"steps": self.ep["steps"], "red_violations": self.ep["red_violations"],
               "jerk_mean": c.get("jerk_sum", 0.0) / cn, "jerk_max": c.get("jerk_max", 0.0),
               "jerk_over_rate": c.get("jerk_over", 0) / cn, "hard_acc": c.get("hard_acc", 0),
               "hard_brake": c.get("hard_brake", 0),
               "intervention_rate": self.ep["intervention_steps"] / n,
               "would_intervene_rate": self.ep["would_steps"] / n}
        for k in self.MASK_TYPES:
            out[f"int_{k}_rate"] = self.ep[f"int_{k}"] / n
            out[f"would_{k}_rate"] = self.ep[f"would_{k}"] / n
        return out

    # ------------------------------------------------------------------
    def step_reward(self, v, why, gap, lane_change, lane_change_applied):
        import traci
        env, rw = self.env, self.rw
        odo = traci.vehicle.getDistance(env.ego_id)
        prog = max(odo - self.prev_odo, 0.0)
        self.prev_odo = odo
        r = rw.get("progress_weight", 0.1) * prog / (env.vmax * env.step_length)
        terms = {"progress": r}
        # 시간 감점: 기어가며 버티면(실패를 미루거나 시간 제한까지 버티기) 손해가 되게
        if rw.get("time_penalty", 0.0):
            r -= rw["time_penalty"]
            terms["time"] = -rw["time_penalty"]
        vlim = self.last.get("vlim", env.vmax)
        if v > 1.1 * vlim:
            p = rw.get("speeding_penalty", 0.1) * (v - 1.1 * vlim) / vlim
            r -= p
            terms["speeding"] = -p
        # 멈춤 (2026-11-17: '오래 서 있기'와 '이유 없이 멈춤' 두 항을 하나로 합침).
        #   거의 서 있는데 (가까운 적·황색 신호, 양보해야 할 상충 차량, 15 m 안 앞차) 중 아무 이유도 없거나,
        #   적·황색 신호 대기가 아닌데 stall_after_s 초 넘게 서 있으면 스텝마다 stop_penalty
        self.stop_steps = self.stop_steps + 1 if v < 0.5 else 0
        if v < 0.5 and rw.get("stop_penalty", 0.0):
            nt = self.last.get("tls")
            red_near = bool(nt) and _sig_class(nt[2]) in ("red", "yellow") and nt[1] < 40
            foe_near = any(f["ego_yield"] and f["ego_d"] < 30 for f in self.last.get("foes", []))
            lead_near = gap is not None and gap < 15
            long_stop = self.stop_steps * env.step_length > rw.get("stall_after_s", 30.0) and not red_near
            if long_stop or not (red_near or foe_near or lead_near):
                r -= rw["stop_penalty"]
                terms["stop"] = -rw["stop_penalty"]
        # 앞차 간격: 시간 간격(내 속도 × headway_s) 미만이면 감점, 최소 close_gap_threshold (m)
        # (멈춰 있을 때는 제외: 신호 대기 줄에서 앞차와 2~3 m 간격으로 서 있는 건 정상)
        if gap is not None and (v or 0.0) >= 1.0 and gap < max(rw.get("close_gap_threshold", 3.0),
                                                               rw.get("headway_s", 0.0) * (v or 0.0)):
            r -= rw.get("close_gap_penalty", 0.05)
            terms["close_gap"] = -rw.get("close_gap_penalty", 0.05)
        # 2026-11-24 (충돌 원인 분석 → 사용자 승인 A안): 교차로 안에서 움직이면서(0.5 m/s 이상) 관측에 보이는 차와
        #   몸체 사이가 junction_close_m 안이고 그 거리가 줄어드는 중이면(스텝당 0.1 m 이상) 스텝마다 감점.
        #   강건성 5000판 충돌의 72%가 "교차로 안에서 보이는 차를 천천히 밀고 들어가다 닿음"이었고, 앞차 간격 감점
        #   (같은 차로 앞차만)은 이걸 못 잡았다. 나란히 가는 옆 차로 차(거리 일정)는 감점 안 됨. 관측에 들어간 차만 쓴다.
        dmin = self.last.get("obj_min_gap")
        d_prev, self._prev_obj_gap = getattr(self, "_prev_obj_gap", None), dmin
        if rw.get("junction_close_penalty", 0.0) and (v or 0.0) >= 0.5 and dmin is not None and d_prev is not None \
                and dmin < rw.get("junction_close_m", 3.0) and d_prev - dmin >= 0.1 \
                and traci.vehicle.getLaneID(env.ego_id).startswith(":"):
            r -= rw["junction_close_penalty"]
            terms["junction_close"] = -rw["junction_close_penalty"]
        # 승차감 (2026-11-17): 실제 가감속 a = 속도 변화 / 0.5 s, 저크 = |a − 지난 a| / 0.5 s.
        #   저크가 jerk_threshold(4 m/s³)를 넘은 만큼만 감점 — 평소 운전·꾸준한 가감속은 감점 없음,
        #   급조작만 손해 (최대 약 0.09 / 스텝, 충돌 50·신호위반 30 보다 훨씬 작아 필요한 급제동은 막지 않음)
        dt = env.step_length
        if self.prev_v is not None:
            a_now = (v - self.prev_v) / dt
            jerk = abs(a_now - self.prev_a) / dt
            c = self.comfort
            c["n"] += 1; c["jerk_sum"] += jerk; c["jerk_max"] = max(c["jerk_max"], jerk)
            c["jerk_over"] += jerk > rw.get("jerk_threshold", 4.0)
            c["hard_acc"] += a_now > 3.0; c["hard_brake"] += a_now < -4.0
            if rw.get("jerk_penalty", 0.0) and jerk > rw.get("jerk_threshold", 4.0):
                p = rw["jerk_penalty"] * (jerk - rw.get("jerk_threshold", 4.0)) / rw.get("jerk_threshold", 4.0)
                r -= p
                terms["jerk"] = -p
            self.prev_a = a_now
        self.prev_v = v
        # 2026-11-17: 실제로 차로를 바꿀 때만 감점 (불가능한 명령 감점·틀린 차로 감점·마스킹 개입 감점은 뺌)
        if lane_change != 0 and lane_change_applied:
            r -= rw.get("lane_change_penalty", 0.0)
            terms["lane_change"] = -rw.get("lane_change_penalty", 0.0)
        return r, terms


class ConflictMap:
    """교차로 내부 경로(internal lane)끼리 교차·합류하는 지점을 맵에서 계산해 둔다 (필요할 때 계산 후 캐시)."""

    def __init__(self, net):
        from shapely.geometry import LineString
        self.net = net
        self._shape, self._len, self._junction, self._next, self._feeder, self._dir = {}, {}, {}, {}, {}, {}
        self._by_junction = {}
        for e in net.getEdges(withInternal=True):
            for ln in e.getLanes():
                lid = ln.getID()
                for c in ln.getOutgoing():
                    via = c.getViaLaneID()
                    if via:
                        self._feeder.setdefault(via, []).append(lid)
                        self._dir[via] = c.getDirection().lower()
                        if lid.startswith(":"):
                            self._next[lid] = via
                if lid.startswith(":"):
                    self._shape[lid] = LineString(ln.getShape()) if len(ln.getShape()) > 1 else None
                    self._len[lid] = ln.getLength()
                    j = lid[1:].rsplit("_", 2)[0]
                    self._junction[lid] = j
                    self._by_junction.setdefault(j, []).append(lid)
        # 교차로 중간 대기 지점 뒤의 두 번째 구간도 첫 구간과 같은 방향
        for a, b in self._next.items():
            if b not in self._dir and a in self._dir:
                self._dir[b] = self._dir[a]
        self._conf = {}
        self._nl = {}
        # 2026-11-21 공통 수정 2: 짧은 진입 토막 너머 윗길까지 거슬러 찾기 위한 지도 연결 (차로 → [(윗 차로, 사이 내부 경로)])
        #   와 내부 경로가 이어지는 일반 차로 (내부 경로는 출구가 하나뿐 → 지도 정보)
        self._pred, self._dest, self._llen = {}, {}, {}
        for e in net.getEdges(withInternal=True):
            for ln in e.getLanes():
                self._llen[ln.getID()] = ln.getLength()
                for c in ln.getOutgoing():
                    to = c.getToLane().getID()
                    via = c.getViaLaneID()
                    if not ln.getID().startswith(":") and not to.startswith(":"):
                        self._pred.setdefault(to, []).append((ln.getID(), via or None))
                    if via:
                        self._dest[via] = to
        for a, b in list(self._next.items()):
            if a in self._dest:
                self._dest[b] = self._dest[a]

    def length(self, lane):
        return self._len.get(lane, 0.0)

    def next_internal(self, lane):
        return self._next.get(lane)

    def direction(self, via):
        return self._dir.get(via, "s")

    def _chain(self, lane):
        out, cur = {lane}, lane
        while cur in self._next:
            cur = self._next[cur]
            out.add(cur)
        for prev, nxt in self._next.items():       # 앞쪽 구간
            if nxt == lane:
                out.add(prev)
        return out

    def conflicts(self, lane):
        """[(상대 내부 경로, ego 경로 위 교차점 거리, 상대 경로 위 교차점 거리)]"""
        if lane in self._conf:
            return self._conf[lane]
        from shapely.ops import nearest_points
        res = []
        me = self._shape.get(lane)
        if me is not None:
            own = self._chain(lane)
            my_feed = set(self._feeder.get(lane, []))
            for other in self._by_junction.get(self._junction[lane], []):
                sh = self._shape.get(other)
                if other in own or sh is None or my_feed & set(self._feeder.get(other, [])):
                    continue
                if me.distance(sh) > 1.2:          # 차 폭을 고려해 1.2 m 안으로 스치면 상충
                    continue
                inter = me.intersection(sh)
                pt = (inter.geoms[0] if hasattr(inter, "geoms") else inter) if not inter.is_empty else \
                    nearest_points(me, sh)[0]
                if pt.geom_type != "Point":
                    pt = pt.interpolate(0) if hasattr(pt, "interpolate") else pt.centroid
                s_ec, s_fc = me.project(pt), sh.project(pt)
                # 2026-11-20: 중심선 교차점이 아니라 두 차체가 겹치기 시작하는 지점 (중심선 사이 거리 < 차 폭 + 여유).
                #   비스듬히 만나는 곳(SE 삼각 모퉁이)은 겹침이 교차점보다 훨씬 앞에서 시작해 "교차점 3 m 앞"에 서도 부딪혔음
                #   교차점에서 뒤로 걸어가며 차체 겹침이 끝나는 곳까지 (최대 OVERLAP_BACK m)
                def back(line, other_line, s_c):
                    x = s_c
                    while x > 0.0 and s_c - x < OVERLAP_BACK and line.interpolate(max(x - 0.25, 0.0)).distance(other_line) < OVERLAP_D:
                        x -= 0.25
                    return max(x, 0.0)
                s_es, s_fs = back(me, sh, s_ec), back(sh, me, s_fc)
                res.append((other, float(s_es), float(s_fs), float(s_ec), float(s_fc)))
        self._conf[lane] = res
        return res

    def _n_lanes(self, edge):
        if edge not in self._nl:
            self._nl[edge] = self.net.getEdge(edge).getLaneNumber() if self.net.hasEdge(edge) else 0
        return self._nl[edge]

    def vehicles_toward(self, foe_lane, s_f, use_intent=True):
        """foe_lane 위의 차, 또는 곧 foe_lane 으로 들어갈 수 있는 차: [(차, 앞범퍼 → 교차점 거리)]
        use_intent=False: 진입 차로의 차가 실제로 foe_lane 으로 갈지(그 차의 경로)는 보지 않는다 (관측용)."""
        import traci
        out = []
        for v in traci.lane.getLastStepVehicleIDs(foe_lane):
            out.append((v, s_f - traci.vehicle.getLanePosition(v)))
        for feed in self._feeder.get(foe_lane, []):
            flen = traci.lane.getLength(feed) if not feed.startswith(":") else self._len.get(feed, 0.0)
            for v in traci.lane.getLastStepVehicleIDs(feed):
                rem = flen - traci.vehicle.getLanePosition(v)
                if rem > FOE_LOOKAHEAD:
                    continue
                if use_intent and not feed.startswith(":"):
                    nl = traci.vehicle.getNextLinks(v)
                    if not nl or nl[0][4] != foe_lane:
                        continue
                out.append((v, rem + s_f))
            # 2026-11-21 공통 수정 2: 진입 토막이 짧으면 그 윗길(지도로 이어진 차로)까지 FOE_LOOKAHEAD m 안의 차도 찾는다.
            #   (주차장 입구처럼 13 m 토막 뒤에서 달려오는 차를 토막에 들어와서야 발견하던 문제)
            #   관측용(use_intent=False)은 그 차가 어디로 갈지 보지 않고 이어진 차로의 차를 모두 후보로 (보임 여부는 호출하는 쪽에서 검사).
            #   강사용(use_intent=True)은 그 차의 경로가 이 진입 토막 → 이 교차로 출구로 이어질 때만.
            if not feed.startswith(":") and FOE_WALK:
                dest_edge = self._dest.get(foe_lane, "").rsplit("_", 1)[0] if self._dest.get(foe_lane) else None
                feed_edge = feed.rsplit("_", 1)[0]
                stack = [(feed, flen, 0)]
                seen_l = {feed}
                while stack:
                    lane_d, acc, depth = stack.pop()       # acc: lane_d 시작점 → 교차점 전 진입 토막 끝까지
                    if depth >= 3:
                        continue
                    for up, via in self._pred.get(lane_d, []):
                        if up in seen_l:
                            continue
                        seen_l.add(up)
                        vlen = self._len.get(via, 0.0) if via else 0.0
                        ulen = self._llen.get(up, 0.0)
                        for ln_, base in ((via, acc), (up, acc + vlen)):
                            if not ln_:
                                continue
                            L_ = vlen if ln_ == via else ulen
                            try:
                                vids = traci.lane.getLastStepVehicleIDs(ln_)
                            except traci.TraCIException:
                                continue
                            for v in vids:
                                d_back = (L_ - traci.vehicle.getLanePosition(v)) + base
                                if d_back > FOE_LOOKAHEAD:
                                    continue
                                if use_intent:
                                    try:
                                        rt = traci.vehicle.getRoute(v); ri = traci.vehicle.getRouteIndex(v)
                                    except traci.TraCIException:
                                        continue
                                    rest = list(rt[max(ri, 0):])
                                    if feed_edge not in rest:
                                        continue
                                    j = rest.index(feed_edge)
                                    if dest_edge and (j + 1 >= len(rest) or rest[j + 1] != dest_edge):
                                        continue
                                out.append((v, d_back + s_f))
                        if acc + vlen + ulen < FOE_LOOKAHEAD:
                            stack.append((up, acc + vlen + ulen, depth + 1))
            # 옆 차로에서 이 차로로 끼어들려는 차 (방향지시등을 이 차로 쪽으로 켬)
            if not feed.startswith(":"):
                edge, k = feed.rsplit("_", 1)
                k = int(k)
                n_lanes = self._n_lanes(edge)
                for adj, blink in ((k + 1, 1), (k - 1, 2)):   # 왼쪽 차로 → 오른쪽 깜빡이(1), 오른쪽 → 왼쪽(2)
                    if adj < 0 or adj >= n_lanes:
                        continue
                    alane = f"{edge}_{adj}"
                    try:
                        vids = traci.lane.getLastStepVehicleIDs(alane)
                        alen = traci.lane.getLength(alane)
                    except traci.TraCIException:
                        continue
                    for v in vids:
                        rem = alen - traci.vehicle.getLanePosition(v)
                        if rem <= FOE_LOOKAHEAD and traci.vehicle.getSignals(v) & blink:
                            out.append((v, rem + s_f))
        return out
