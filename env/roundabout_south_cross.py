"""빨간 도로(남쪽 연장) 끝과 파란 도로(SW_end_T_right) 끝을 직선 도로로 잇고, 양쪽 끝을 십자 교차로로 만든다.

  - 교차로 A (S_cross): 빨간 도로 끝 바로 너머. 가지 = 빨간 도로(북), 새 직선 도로(동),
    빨간 도로 연장 방향 짧은 가지(남), 새 직선 도로 반대 방향 짧은 가지(서).
  - 교차로 B (SW_end_T_right): 파란 도로 끝. 가지 = 새 직선 도로(서), 파란 도로(동),
    파란 도로 축에 직각인 짧은 가지(북/남).
  - 새 도로: A→B 직선이 파란 도로 방향과 어긋나면 직선 → 원호(LINK_RADIUS) → 직선(LINK_END_STRAIGHT)으로
    교차로 B 에 파란 도로와 일직선으로 들어간다. 단면은 파란 도로와 같음(방향별 5차로, 중앙 포장 1.0m, 황색 단선).
  - 짧은 가지: 방향별 3차로, 황색 복선, 끝은 막다른 길.
  - 두 교차로 모두 신호 교차로(netconvert 기본 신호).
  - (선택) 새 직선 도로 중간(U_T): 서행 중앙선 쪽 1차로에서만 유턴. 현재 U_T=None(사용 안 함).
  모든 차로 폭은 3.2m.
roundabout_south_extend 가 남쪽 6차선/안전지대를 적용한 뒤 호출한다.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

LANE_W = 3.2
SPEED = '16.670000'
A_OFF = 10.0            # 빨간 도로 끝에서 교차로 중심까지
STUB = 70.0             # 짧은 가지 길이 (교차로 중심부터)
STUB_LANES = 3
# 가지별 (나가는 차로 수, 들어오는 차로 수). 없으면 STUB_LANES 양방향.
STUB_LANES_BY = {'S_cross_s': (4, 5),     # 교차로 A 남쪽 가지: 남행 4, 북행(교차로 진입) 5
                 'S_cross_w': (5, 6)}     # 교차로 A 서쪽 가지: 서행(나감) 5, 동행(교차로 진입) 6
LINK_LANES = 5
MEDIAN_HALF = 0.5
YELLOW, WHITE, ASPHALT = '235,190,40', '255,255,255', '84,84,90'

A_NODE, B_NODE = 'S_cross', 'SW_end_T_right'
RED_OUT, RED_IN = 'S_out0_ext5', 'S_in1_ext'
BLUE_IN, BLUE_OUT = 'SW_end_T_right_in', 'SW_end_T_right_out'
LINK_E, LINK_W = 'S_link_east', 'S_link_west'
# 교차로 A 로 들어오는 서행: 정지선 앞에서 바깥(북쪽)에 차로 하나 추가 → 6차로
W6_NODE, LINK_W6 = 'S_link_w6', 'S_link_west6'
W6_LEN = 80.0           # 교차로 A 중심에서 6차로 시작점까지
W6_TAPER = 40.0         # 그 앞 바깥 가장자리 테이퍼
# 서행 6차로의 차로별 진행 방향 (0 = 가장 바깥). 노면 화살표와 실제 연결을 함께 정한다.
W6_ARROWS = {5: 'left', 4: 'straight_left', 3: 'straight', 2: 'straight', 1: 'straight', 0: 'right'}
# 정지선 바로 뒤 화살표 (엣지: {차로: 종류}, 0 = 가장 바깥)
RED_ARROWS = {4: 'left', 3: 'straight', 2: 'straight', 1: 'straight', 0: 'right'}
# 교차로 A 남쪽 가지에서 올라오는 5차로: 좌 → 서쪽 가지, 직 → 빨간 도로 북행, 우 → 새 직선 도로
S_IN_ARROWS = {4: 'left', 3: 'straight', 2: 'straight', 1: 'straight', 0: 'right'}
# 교차로 A 서쪽 가지에서 들어오는 6차로: 좌 → 빨간 도로 북행, 직 → 새 직선 도로, 우 → 남쪽 가지
W_IN_ARROWS = {5: 'left', 4: 'straight', 3: 'straight', 2: 'straight', 1: 'straight', 0: 'right'}
# 교차로 B 로 들어오는 서행 5차로(T자 쪽에서): 직좌, 직, 직, 직, 직우
B_IN_ARROWS = {4: 'straight_left', 3: 'straight', 2: 'straight', 1: 'straight', 0: 'straight_right'}
# 교차로 B 나머지 세 진입로: (화살표, 직진 대상, 좌회전 대상, 우회전 대상)
B_OTHER = {
    'SW_cross_n_in': ({2: 'straight_left', 1: 'straight', 0: 'straight'},
                      'SW_cross_s_out', 'SW_end_T_right_in', 'S_link_west'),
    'LINK_E_END':    ({4: 'straight_left', 3: 'straight', 2: 'straight', 1: 'straight', 0: 'straight_right'},
                      'SW_end_T_right_in', 'SW_cross_n_out', 'SW_cross_s_out'),
    'SW_cross_s_in': ({2: 'straight_left', 1: 'straight', 0: 'straight_right'},
                      'SW_cross_n_out', 'S_link_west', 'SW_end_T_right_in'),
}


def _b_other():
    le_end = LINK_E2 if U_T is not None else LINK_E
    return {(le_end if k == 'LINK_E_END' else k): v for k, v in B_OTHER.items()}
OLD_DEAD = ('S_ext_end0', 'S_ext_end1')
# 새 직선 도로 중간 유턴: 서행(B→A)의 중앙선 쪽 1차로에서만 동행으로 유턴
U_NODE = 'S_link_uturn'
U_T = None              # 교차로 A 중심에서 유턴 지점까지 (m). None 이면 유턴 없음
                        # (2026-11-07: 사용자 요청으로 유턴은 T자-동남쪽 긴 도로로 옮김 → roundabout_long_uturn)
U_ZONE = (3.0, 17.0)    # 유턴 구역: 유턴 지점 기준 A 쪽 3m ~ B 쪽 17m (중앙선 대신 흰 점선)
LINK_E2, LINK_W2 = 'S_link_east2', 'S_link_west2'   # 유턴 지점에서 나뉜 뒷부분
# 새 도로 선형: A→B 직선이 파란 도로 방향과 LINK_CURVE_MIN_DEG 이상 어긋나면
#   A 에서 직선 → 반지름 LINK_RADIUS 원호 → B 앞 LINK_END_STRAIGHT m 직선(파란 도로와 같은 방향)으로 잇는다.
#   (도로를 펴서 B 가 옮겨져도 교차로 B 에는 예전처럼 파란 도로와 일직선으로 들어간다)
LINK_RADIUS = 250.0
LINK_END_STRAIGHT = 70.0
LINK_CURVE_MIN_DEG = 1.0


def H(t):
    t = min(1.0, max(0.0, t))
    return t*t*t*(10 - 15*t + 6*t*t)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _unit(a, b):
    d = math.dist(a, b)
    return ((b[0]-a[0])/d, (b[1]-a[1])/d)


def _add(p, v, k=1.0):
    return (p[0]+v[0]*k, p[1]+v[1]*k)


class LinkAxis:
    """A 에서 B 까지 새 도로 중심축: 직선 + 원호 + 직선. s = A 에서 거리, l = 왼쪽(+)."""

    def __init__(self, A, B, b):
        chord = _unit(A, B)
        tb = math.atan2(b[1], b[0])
        tc = math.atan2(chord[1], chord[0])
        dev = math.degrees(math.atan2(math.sin(tc - tb), math.cos(tc - tb)))
        if abs(dev) < LINK_CURVE_MIN_DEG:
            self.pts = [A, B]
        else:
            R, Lf = LINK_RADIUS, LINK_END_STRAIGHT
            Q = _add(B, b, -Lf)                      # 원호 끝
            sgn = 1.0 if dev > 0 else -1.0           # A 에서 왼쪽(+)으로 출발해 오른쪽으로 돌거나 그 반대

            def arc_disp(t0):
                if sgn > 0:      # 시계 방향(오른쪽)으로 t0 → tb
                    return (R*(math.sin(t0) - math.sin(tb)), R*(math.cos(tb) - math.cos(t0)))
                return (R*(math.sin(tb) - math.sin(t0)), R*(math.cos(t0) - math.cos(tb)))

            def f(t0):
                D = arc_disp(t0)
                v = (Q[0]-A[0]-D[0], Q[1]-A[1]-D[1])
                return v[0]*math.sin(t0) - v[1]*math.cos(t0), v[0]*math.cos(t0) + v[1]*math.sin(t0)

            lo, hi = tb, tb + sgn*math.radians(89.0)
            flo = f(lo)[0]
            for _ in range(200):
                mid = (lo + hi)/2
                fm = f(mid)[0]
                if (fm > 0) == (flo > 0):
                    lo, flo = mid, fm
                else:
                    hi = mid
            t0 = (lo + hi)/2
            L1 = f(t0)[1]
            assert L1 > W6_LEN + W6_TAPER + 20.0, f'새 도로 첫 직선이 너무 짧다: {L1:.1f} m'
            P0 = _add(A, (math.cos(t0), math.sin(t0)), L1)
            n = max(8, int(abs(t0 - tb)*R/2.0))
            arc = []
            for i in range(n+1):
                th = t0 + (tb - t0)*i/n
                # 원호 위 점 = 원호 시작점 + (t0→tb 변위) - (th→tb 변위)
                d0, d1 = arc_disp(t0), arc_disp(th)
                arc.append((P0[0] + d0[0] - d1[0], P0[1] + d0[1] - d1[1]))
            self.pts = [A] + arc + [B]
            self.start_deg, self.L1 = math.degrees(t0), L1
        self.cum = [0.0]
        for p, q in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.dist(p, q))
        self.L = self.cum[-1]

    def at(self, s):
        import bisect
        i = min(max(bisect.bisect_right(self.cum, s), 1), len(self.cum) - 1)
        p, q = self.pts[i-1], self.pts[i]
        seg = self.cum[i] - self.cum[i-1] or 1e-9
        h = ((q[0]-p[0])/seg, (q[1]-p[1])/seg)
        f = (s - self.cum[i-1])/seg
        return (p[0] + (q[0]-p[0])*f, p[1] + (q[1]-p[1])*f), h

    def P(self, s, l=0.0):
        p, h = self.at(s)
        return (p[0] - h[1]*l, p[1] + h[0]*l)

    def proj(self, x):
        best = None
        for i, (p, q) in enumerate(zip(self.pts, self.pts[1:])):
            seg = self.cum[i+1] - self.cum[i] or 1e-9
            h = ((q[0]-p[0])/seg, (q[1]-p[1])/seg)
            t = (x[0]-p[0])*h[0] + (x[1]-p[1])*h[1]
            lo = -1e9 if i == 0 else 0.0                       # A 바깥쪽으로 연장
            hi = 1e9 if i == len(self.pts) - 2 else seg        # B 바깥쪽으로 연장
            tc = min(max(t, lo), hi)
            c = (p[0] + h[0]*tc, p[1] + h[1]*tc)
            d = math.dist(c, x)
            if best is None or d < best[0]:
                best = (d, self.cum[i] + tc)
        return best[1]

    def ts(self, a, b, step=2.0):
        """a~b 사이 표본 위치 (원호 꼭짓점 포함)."""
        lo, hi = min(a, b), max(a, b)
        out = sorted({lo, hi} | {c for c in self.cum if lo < c < hi} |
                     {lo + (hi-lo)*i/max(1, int((hi-lo)/step)) for i in range(max(1, int((hi-lo)/step)))})
        return out if a <= b else out[::-1]


class Plan:
    def __init__(self, root, frame, length):
        edges = {e.get('id'): e for e in root.findall('edge')}
        junctions = {j.get('id'): j for j in root.findall('junction')}
        self.frame = frame
        self.s_end = frame.s0 + length
        self.sA = self.s_end + A_OFF
        self.u = frame.u
        self.nu = frame.n                       # 동쪽(+) = 북행 쪽
        self.A = frame.P(self.sA, 0.0)
        self.B = tuple(float(junctions[B_NODE].get(k)) for k in 'xy')
        sa, sb = _read(edges[BLUE_IN].get('shape'))
        self.b = _unit(sa, sb)
        self.nb = (-self.b[1], self.b[0])       # 북쪽(+)
        self.axis = LinkAxis(self.A, self.B, self.b)
        self.d = self.axis.at(0.0)[1]           # 새 도로의 A 쪽 방향 (A→B)
        self.nL = (-self.d[1], self.d[0])       # 왼쪽(+) = 서행 쪽

    def pt(self, t, l=0.0):
        """새 도로 축 좌표: A 에서 B 쪽으로 축을 따라 t, 왼쪽(서행 쪽)으로 l."""
        return self.axis.P(t, l)

    def t_of(self, p):
        return self.axis.proj(p)

    def line(self, t0, t1, l=0.0, step=2.0):
        """축을 따라 t0 → t1 의 오프셋 선."""
        return [self.pt(t, l) for t in self.axis.ts(t0, t1, step)]

    def strip(self, t0, t1, l0, l1):
        return self.line(t0, t1, l0) + self.line(t1, t0, l1)

    # 짧은 가지들: (out_id, in_id, end_node, 방향벡터, 바깥쪽 진행차로 쪽 오른쪽 벡터, 중심)
    def stubs(self):
        f = self.frame
        A, B = self.A, self.B
        neg = lambda v: (-v[0], -v[1])
        return [
            ('S_cross_s', A, self.u, neg(self.nu)),       # 남쪽 가지: 나가는 차로는 서쪽
            ('S_cross_w', A, neg(self.d), self.nL),       # 서쪽 가지: 나가는 차로는 nL 쪽
            ('SW_cross_n', B, self.nb, self.b),           # 북쪽 가지: 나가는 차로는 동쪽
            ('SW_cross_s', B, neg(self.nb), neg(self.b)), # 남쪽 가지: 나가는 차로는 서쪽
        ]


def _subnet(plan, root, work):
    edges = {e.get('id'): e for e in root.findall('edge')}
    f = plan.frame
    c5 = -(LINK_LANES*LANE_W/2)                 # 남행 5차로 중심 (안쪽 경계 0)
    c3 = STUB_LANES*LANE_W/2
    nodes = [(A_NODE, plan.A, 'traffic_light'), (B_NODE, plan.B, 'traffic_light'),
             ('R0', f.P(plan.s_end-30, c5), 'dead_end'), ('R1', f.P(plan.s_end-30, c3), 'dead_end')]
    spec = [
        (RED_OUT, 'R0', A_NODE, LINK_LANES, [f.P(plan.s_end-30, c5), f.P(plan.sA, c5)]),
        (RED_IN, A_NODE, 'R1', STUB_LANES, [f.P(plan.sA, c3), f.P(plan.s_end-30, c3)]),
    ]
    # 파란 도로 (B → T 방향 끝은 가짜 노드)
    sin, sout = _read(edges[BLUE_IN].get('shape')), _read(edges[BLUE_OUT].get('shape'))
    nodes += [('T0', sin[-1], 'dead_end'), ('T1', sout[0], 'dead_end')]
    spec += [(BLUE_IN, B_NODE, 'T0', LINK_LANES, sin), (BLUE_OUT, 'T1', B_NODE, LINK_LANES, sout)]
    # 새 직선 도로
    off = MEDIAN_HALF + LINK_LANES*LANE_W/2
    if U_T is None:
        L = plan.axis.L
        spec += [(LINK_E, A_NODE, B_NODE, LINK_LANES, plan.line(0.0, L, -off)),
                 (LINK_W, B_NODE, W6_NODE, LINK_LANES, plan.line(L, W6_LEN, off))]
    else:
        nodes.append((U_NODE, plan.pt(U_T), 'priority'))
        L = plan.axis.L
        spec += [(LINK_E, A_NODE, U_NODE, LINK_LANES, plan.line(0.0, U_T, -off)),
                 (LINK_E2, U_NODE, B_NODE, LINK_LANES, plan.line(U_T, L, -off)),
                 (LINK_W, B_NODE, U_NODE, LINK_LANES, plan.line(L, U_T, off)),
                 (LINK_W2, U_NODE, W6_NODE, LINK_LANES, plan.line(U_T, W6_LEN, off))]
    off6 = MEDIAN_HALF + (LINK_LANES+1)*LANE_W/2
    nodes.append((W6_NODE, _add(plan.pt(W6_LEN), plan.nL, off6), 'priority'))
    spec += [(LINK_W6, W6_NODE, A_NODE, LINK_LANES+1,
              [_add(plan.pt(W6_LEN), plan.nL, off6), _add(plan.A, plan.nL, off6)])]
    # 짧은 가지
    for name, center, dirv, right in plan.stubs():
        end = _add(center, dirv, STUB)
        nodes.append((f'{name}_end', end, 'dead_end'))
        n_out, n_in = STUB_LANES_BY.get(name, (STUB_LANES, STUB_LANES))
        co, ci = n_out*LANE_W/2, n_in*LANE_W/2      # 중앙선(가지 축)에서 각 방향 차로 중심까지
        out_pts = [_add(center, right, co), _add(end, right, co)]
        in_pts = [_add(end, right, -ci), _add(center, right, -ci)]
        node = A_NODE if center == plan.A else B_NODE
        spec += [(f'{name}_out', node, f'{name}_end', n_out, out_pts),
                 (f'{name}_in', f'{name}_end', node, n_in, in_pts)]
    with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
        fh.write('<nodes>\n')
        for nid, (x, y), t in nodes:
            fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"/>\n')
        fh.write('</nodes>\n')
    with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
        fh.write('<edges>\n')
        for eid, fr, to, n, pts in spec:
            fh.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{SPEED}" '
                     f'priority="2" width="{LANE_W}" spreadType="center" shape="{_write(pts)}"/>\n')
        fh.write('</edges>\n')
    with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
        fh.write('<connections>\n')
        w_last = LINK_W2 if U_T is not None else LINK_W
        for k in range(LINK_LANES):     # 기존 5개 차로는 번호 +1, 새 차로는 가장 바깥(0번)
            fh.write(f'  <connection from="{w_last}" to="{LINK_W6}" fromLane="{k}" toLane="{k+1}"/>\n')
        if U_T is not None:
            # 유턴 지점: 직진은 그대로, 서행 1차로(가장 안쪽)에서만 동행 1차로로 유턴
            for k in range(LINK_LANES):
                fh.write(f'  <connection from="{LINK_W}" to="{LINK_W2}" fromLane="{k}" toLane="{k}"/>\n')
                fh.write(f'  <connection from="{LINK_E}" to="{LINK_E2}" fromLane="{k}" toLane="{k}"/>\n')
            fh.write(f'  <connection from="{LINK_W}" to="{LINK_E2}" fromLane="{LINK_LANES-1}" toLane="{LINK_LANES-1}"/>\n')
        # 교차로 A: 화살표와 같은 연결 (우 → 빨간 도로 북행, 직 → 서쪽 가지, 좌 → 남쪽 가지)
        n_s_out = STUB_LANES_BY.get('S_cross_s', (STUB_LANES,))[0]
        left_lanes = sorted(k for k, a in W6_ARROWS.items() if 'left' in a)
        for k, kind in W6_ARROWS.items():
            if kind == 'right':
                fh.write(f'  <connection from="{LINK_W6}" to="{RED_IN}" fromLane="{k}" toLane="0"/>\n')
            if 'straight' in kind:
                fh.write(f'  <connection from="{LINK_W6}" to="S_cross_w_out" fromLane="{k}" toLane="{k}"/>\n')
            if 'left' in kind:
                to = n_s_out - len(left_lanes) + left_lanes.index(k)
                fh.write(f'  <connection from="{LINK_W6}" to="S_cross_s_out" fromLane="{k}" toLane="{to}"/>\n')
        # 빨간 도로 남행: 화살표와 같은 연결 (우 → 서쪽 가지, 직 → 남쪽 가지, 좌 → 새 직선 도로)
        for k, kind in RED_ARROWS.items():
            if kind == 'right':
                fh.write(f'  <connection from="{RED_OUT}" to="S_cross_w_out" fromLane="{k}" toLane="0"/>\n')
            if 'straight' in kind:
                fh.write(f'  <connection from="{RED_OUT}" to="S_cross_s_out" fromLane="{k}" toLane="{min(k, n_s_out-1)}"/>\n')
            if 'left' in kind:
                for to in (LINK_LANES-2, LINK_LANES-1):
                    fh.write(f'  <connection from="{RED_OUT}" to="{LINK_E}" fromLane="{k}" toLane="{to}"/>\n')
        # 남쪽 가지 진입: 화살표와 같은 연결
        n_w_out = STUB_LANES_BY.get('S_cross_w', (STUB_LANES,))[0]
        straight = sorted(k for k, a in S_IN_ARROWS.items() if a == 'straight')
        for k, kind in S_IN_ARROWS.items():
            if kind == 'right':
                fh.write(f'  <connection from="S_cross_s_in" to="{LINK_E}" fromLane="{k}" toLane="0"/>\n')
            if kind == 'straight':
                fh.write(f'  <connection from="S_cross_s_in" to="{RED_IN}" fromLane="{k}" toLane="{straight.index(k)}"/>\n')
            if kind == 'left':
                fh.write(f'  <connection from="S_cross_s_in" to="S_cross_w_out" fromLane="{k}" toLane="{n_w_out-1}"/>\n')
        # 교차로 B 나머지 진입로: 화살표와 같은 연결
        lanes_of = {e: n for e, _, _, n, _ in spec}
        for eid, (arrows, s_to, l_to, r_to) in _b_other().items():
            for k, kind in arrows.items():
                if 'straight' in kind:
                    fh.write(f'  <connection from="{eid}" to="{s_to}" fromLane="{k}" toLane="{min(k, lanes_of[s_to]-1)}"/>\n')
                if 'left' in kind:
                    fh.write(f'  <connection from="{eid}" to="{l_to}" fromLane="{k}" toLane="{lanes_of[l_to]-1}"/>\n')
                if 'right' in kind:
                    fh.write(f'  <connection from="{eid}" to="{r_to}" fromLane="{k}" toLane="0"/>\n')
        # 교차로 B 서행 진입: 화살표와 같은 연결 (직 → 새 직선 도로, 좌 → 남쪽 가지, 우 → 북쪽 가지)
        nb_s = STUB_LANES_BY.get('SW_cross_s', (STUB_LANES,))[0]
        for k, kind in B_IN_ARROWS.items():
            if 'straight' in kind:
                fh.write(f'  <connection from="{BLUE_OUT}" to="{LINK_W}" fromLane="{k}" toLane="{k}"/>\n')
            if 'left' in kind:
                fh.write(f'  <connection from="{BLUE_OUT}" to="SW_cross_s_out" fromLane="{k}" toLane="{nb_s-1}"/>\n')
            if 'right' in kind:
                fh.write(f'  <connection from="{BLUE_OUT}" to="SW_cross_n_out" fromLane="{k}" toLane="0"/>\n')
        # 서쪽 가지 진입: 화살표와 같은 연결
        straight = sorted(k for k, a in W_IN_ARROWS.items() if a == 'straight')
        for k, kind in W_IN_ARROWS.items():
            if kind == 'right':
                fh.write(f'  <connection from="S_cross_w_in" to="S_cross_s_out" fromLane="{k}" toLane="0"/>\n')
            if kind == 'straight':
                fh.write(f'  <connection from="S_cross_w_in" to="{LINK_E}" fromLane="{k}" toLane="{straight.index(k)}"/>\n')
            if kind == 'left':
                fh.write(f'  <connection from="S_cross_w_in" to="{RED_IN}" fromLane="{k}" toLane="{STUB_LANES-1}"/>\n')
        fh.write('</connections>\n')
    from sumolib import checkBinary
    out = os.path.join(work, 'p.net.xml')
    r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                        '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                        '--junctions.corner-detail', '24', '--junctions.internal-link-detail', '20',
                        # 신호가 바뀔 때 교차로 안에 남은 좌회전 차량이 빠져나가도록 전적색 3초
                        '--tls.allred.time', '3',
                        '--precision', '6', '-o', out],
                       cwd=work, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr)
    return ET.parse(out).getroot(), [s[0] for s in spec]


def apply_net(net_path, frame, length):
    tree = ET.parse(net_path)
    root = tree.getroot()
    plan = Plan(root, frame, length)
    with tempfile.TemporaryDirectory() as work:
        sub, spec_ids = _subnet(plan, root, work)
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}

    # 기존 엣지 4개: 교차로 쪽 끝점만 subnet 결과로
    for e in root.findall('edge'):
        eid = e.get('id')
        if eid not in (RED_OUT, RED_IN, BLUE_IN, BLUE_OUT):
            continue
        at_end = eid in (RED_OUT, BLUE_OUT)
        if eid == RED_OUT:
            e.set('to', A_NODE)
        if eid == RED_IN:
            e.set('from', A_NODE)
        for lane in e.findall('lane'):
            new = _read(next(l for l in sub_edges[eid].findall('lane')
                             if l.get('index') == lane.get('index')).get('shape'))
            pts = _read(lane.get('shape'))
            if at_end:
                pts[-1] = new[-1]
            else:
                pts[0] = new[0]
            lane.set('shape', _write(pts))
            lane.set('length', f'{_length(pts):.6f}')

    keep_nodes = {A_NODE, B_NODE, W6_NODE, U_NODE} | {f'{n}_end' for n, *_ in plan.stubs()}
    prefixes = (f':{A_NODE}_', f':{B_NODE}_', f':{W6_NODE}_', f':{U_NODE}_')
    new_edge_ids = {LINK_E, LINK_W, LINK_W6, LINK_E2, LINK_W2} | \
        {f'{n}_{k}' for n, *_ in plan.stubs() for k in ('out', 'in')}

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in new_edge_ids or e.get('id', '').startswith(prefixes):
            root.insert(pos, e)
            pos += 1
    pos = last_index('tlLogic') + 1
    for t in sub.findall('tlLogic'):
        root.insert(pos, t)
        pos += 1
    for j in root.findall('junction'):
        if j.get('id') in OLD_DEAD + (B_NODE,):
            root.remove(j)
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in keep_nodes or j.get('id', '').startswith(prefixes):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        if c.get('from', '').startswith(prefixes) or c.get('via', '').startswith(prefixes) or \
                c.get('tl') in (A_NODE, B_NODE) or c.get('from') in (LINK_W, LINK_W2, LINK_E):
            root.insert(pos, c)
            pos += 1
    _free_right_turns(root)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return plan


def _free_right_turns(root):
    """우회전은 신호와 관계없이 항상 허용 (양보 후 진행: 'g'). 한국 도로의 우회전 방식."""
    right = {}
    for c in root.findall('connection'):
        if c.get('tl') in (A_NODE, B_NODE) and c.get('dir') == 'r':
            right.setdefault(c.get('tl'), set()).add(int(c.get('linkIndex')))
    for t in root.findall('tlLogic'):
        idx = right.get(t.get('id'))
        if not idx:
            continue
        for ph in t.findall('phase'):
            st = list(ph.get('state'))
            for i in idx:
                if st[i] in 'ry':
                    st[i] = 'g'
            ph.set('state', ''.join(st))


# ───────────────────────── 노면 표시 ─────────────────────────

def _junction_polys(net_root):
    from shapely.geometry import Polygon
    out = []
    for j in net_root.findall('junction'):
        if j.get('id') in (A_NODE, B_NODE):
            out.append(Polygon(_read(j.get('shape'))).buffer(0))
    return out


def apply_deco(deco_path, net_path, plan):
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    from env.roundabout_markings import dash_style
    from env.roundabout_builder import _read_lane_shapes, _stop_line
    tree = ET.parse(deco_path)
    root = tree.getroot()
    net = ET.parse(net_path).getroot()
    area = unary_union(_junction_polys(net)).buffer(0.3)

    # 1) 교차로 안으로 들어간 기존 선(흰/노란 선) 잘라내기
    clipped = 0
    for p in list(root.findall('poly')):
        if p.get('fill') != '0':
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2:
            continue
        g = LineString(pts)
        if not g.intersects(area):
            continue
        rest = g.difference(area)
        # 흰 점선은 교차로 경계에서 잘려 남는 짧은 조각(1.5m 미만)까지 지운다
        min_len = 1.5 if (p.get('color') == WHITE and p.get('lineWidth') == '0.15') else 0.3
        pieces = [x for x in getattr(rest, 'geoms', [rest]) if x.geom_type == 'LineString' and x.length > min_len]
        idx = list(root).index(p)
        root.remove(p)
        for k, x in enumerate(pieces):
            el = ET.Element('poly', dict(p.attrib))
            el.set('id', p.get('id') if k == 0 else f'{p.get("id")}_c{k}')
            el.set('shape', _write(list(x.coords)))
            root.insert(idx+k, el)
        clipped += 1

    # 2) 새 도로 표시
    lanes = {}
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            lanes[(e.get('id'), int(l.get('index')))] = _read(l.get('shape'))
    dash, gap, _ = dash_style(20)
    period = dash + gap
    polys = []

    def poly(pid, color, pts, fill=False, layer='6', width='0.15'):
        a = dict(id=pid, color=color, fill='1' if fill else '0', layer=layer, shape=_write(pts))
        if not fill:
            a['lineWidth'] = width
        polys.append(a)

    def dashed(pid, a, b):
        L = math.dist(a, b)
        v = ((b[0]-a[0])/L, (b[1]-a[1])/L)
        t, j = gap, 0
        while t + 0.5 < L:
            poly(f'{pid}_{j}', WHITE, [_add(a, v, t), _add(a, v, min(t+dash, L))])
            t += period
            j += 1

    def adashed(pid, t0, t1, l):
        """축을 따라 휘는 점선 (직선 축이면 dashed 와 같다)."""
        pts = plan.line(t0, t1, l, step=1.0)
        cum = [0.0]
        for p, q in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.dist(p, q))

        def at(x):
            import bisect
            i = min(max(bisect.bisect_right(cum, x), 1), len(cum) - 1)
            p, q = pts[i-1], pts[i]
            f = (x - cum[i-1])/((cum[i] - cum[i-1]) or 1e-9)
            return (p[0] + (q[0]-p[0])*f, p[1] + (q[1]-p[1])*f)
        t, j, L = gap, 0, cum[-1]
        while t + 0.5 < L:
            e = min(t + dash, L)
            mids = [pts[i] for i, c in enumerate(cum) if t < c < e]
            poly(f'{pid}_{j}', WHITE, [at(t)] + mids + [at(e)])
            t += period
            j += 1

    def boundary(eid, k, side):
        """차로 k 의 왼쪽(+1)/오른쪽(-1) 경계 직선."""
        pts = lanes[(eid, k)]
        a, b = pts[0], pts[-1]
        v = _unit(a, b)
        n = (-v[1], v[0])
        return _add(a, n, side*LANE_W/2), _add(b, n, side*LANE_W/2)

    new_edges = [f'{n}_{k}' for n, *_ in plan.stubs() for k in ('out', 'in')]
    for eid in new_edges:
        n = sum(1 for (e, _) in lanes if e == eid)
        for k in range(n-1):
            dashed(f'scross_dash_{eid}_{k}', *boundary(eid, k, +1))
        # 차로 사이 미세한 틈을 메우는 하부 포장
        oa, ob = boundary(eid, 0, -1)
        ia, ib = boundary(eid, n-1, +1)
        poly(f'scross_under_{eid}', ASPHALT, [oa, ob, ib, ia], fill=True, layer='-1')
    # 서행(B→A): 5차로/6차로 구간 경계에서 끊기지 않도록 축 좌표로 이어 그린다
    tB = plan.t_of(lanes[(LINK_W, 0)][0])
    tA = plan.t_of(lanes[(LINK_W6, 0)][-1])
    for j in range(1, LINK_LANES):
        l = MEDIAN_HALF + LANE_W*j
        adashed(f'scross_dash_{LINK_W}_{j}', tB, tA, l)
    # 새 바깥 차로 경계: 6차로가 시작되는 지점부터 (테이퍼 안으로 15m 앞당김)
    l = MEDIAN_HALF + LANE_W*LINK_LANES
    adashed(f'scross_dash_{LINK_W6}_new', W6_LEN + 15.0, tA, l)
    tW6 = plan.t_of(lanes[(LINK_W6, 0)][0])
    poly('scross_under_west5', ASPHALT, plan.strip(tB, tW6, l, MEDIAN_HALF), fill=True, layer='-1')
    l6 = l + LANE_W
    poly('scross_under_west6', ASPHALT, [plan.pt(tW6+0.5, MEDIAN_HALF), plan.pt(tW6+0.5, l6),
                                        plan.pt(tA, l6), plan.pt(tA, MEDIAN_HALF)], fill=True, layer='-1')
    # 6차로 앞 바깥 가장자리 테이퍼 포장
    ts = [W6_LEN + W6_TAPER*i/80 for i in range(81)]
    curve = [plan.pt(t, l + LANE_W*H((W6_LEN + W6_TAPER - t)/W6_TAPER)) for t in ts]
    poly('scross_west6_taper', ASPHALT,
         curve + [plan.pt(W6_LEN + W6_TAPER, l - 0.05), plan.pt(tW6 - 1.0, l - 0.05),
                  plan.pt(tW6 - 1.0, l6)], fill=True, layer='4.9')   # 작은 교차점 모서리까지 덮는다
    # 동행(A→B): 유턴 지점에서 나뉘어도 끊기지 않도록 축 좌표로 이어 그린다
    tE0 = plan.t_of(lanes[(LINK_E, 0)][0])
    LE_END = LINK_E2 if U_T is not None else LINK_E
    tE1 = plan.t_of(lanes[(LE_END, 0)][-1])
    for j in range(1, LINK_LANES):
        adashed(f'scross_dash_{LINK_E}_{j}', tE0, tE1, -(MEDIAN_HALF + LANE_W*j))
    poly('scross_under_east', ASPHALT, plan.strip(tE0, tE1, -MEDIAN_HALF, -l), fill=True, layer='-1')
    # 새 직선 도로: 중앙 포장 + 황색 단선 (파란 도로와 같은 모양). 유턴 구역은 흰 점선
    poly('scross_link_median', ASPHALT, plan.strip(tE0, tE1, -MEDIAN_HALF, MEDIAN_HALF),
         fill=True, layer='-1')
    if U_T is None:
        poly('scross_link_center', YELLOW, plan.line(tE0, tE1), width='0.2')
    else:
        z0, z1 = U_T - U_ZONE[0], U_T + U_ZONE[1]
        poly('scross_link_center', YELLOW, plan.line(tE0, z0), width='0.2')
        poly('scross_link_center_b', YELLOW, plan.line(z1, tE1), width='0.2')
        t, j = z0 + gap/2, 0
        while t + dash <= z1:
            poly(f'scross_uturn_zone_{j}', WHITE, [plan.pt(t), plan.pt(t + dash)], width='0.2')
            t += period
            j += 1
    # 짧은 가지: 황색 복선
    for name, *_ in plan.stubs():
        oa, ob = boundary(f'{name}_out', STUB_LANES_BY.get(name, (STUB_LANES,))[0]-1, +1)
        v = _unit(oa, ob)
        n = (-v[1], v[0])
        for k, s in enumerate((-0.15, 0.15)):
            poly(f'scross_{name}_center_{k}', YELLOW, [_add(oa, n, s), _add(ob, n, s)])
    # 정지선: 교차로로 들어오는 모든 엣지
    shapes = _read_lane_shapes(net_path)
    marks = []
    for eid in [RED_OUT, LINK_W6, 'S_cross_s_in', 'S_cross_w_in',
                LE_END, BLUE_OUT, 'SW_cross_n_in', 'SW_cross_s_in']:
        _stop_line(marks, f'scross_stop_{eid}', shapes, eid, dist=1.0)
    # 서행 6차로 정지선 바로 뒤 화살표
    from env.roundabout_builder import _arrow
    for eid, arrows in ((LINK_W6, W6_ARROWS), (RED_OUT, RED_ARROWS), ('S_cross_s_in', S_IN_ARROWS),
                         ('S_cross_w_in', W_IN_ARROWS), (BLUE_OUT, B_IN_ARROWS)) + \
            tuple((e, v[0]) for e, v in _b_other().items()):
        for k, kind in arrows.items():
            _arrow(marks, f'scross_arrow_{eid}_{k}', shapes, eid, k, kind, 5.0)
    # 유턴 차로(서행 1차로) 유턴 화살표: 유턴 구역 안
    if U_T is not None:
        _arrow(marks, f'scross_arrow_uturn', shapes, LINK_W, LINK_LANES-1, 'uturn', 8.0)
    _junction_guides(net, plan, poly, dash, gap, A_NODE,
                     {RED_OUT: RED_ARROWS, LINK_W6: W6_ARROWS, 'S_cross_s_in': S_IN_ARROWS,
                      'S_cross_w_in': W_IN_ARROWS})
    _junction_guides(net, plan, poly, dash, gap, B_NODE,
                     dict([(BLUE_OUT, B_IN_ARROWS)] + [(e, v[0]) for e, v in _b_other().items()]))
    insert_at = next((i for i, el in enumerate(list(root)) if el.get('id', '').startswith('lane_dash_')),
                     len(root))
    for a in reversed(polys):
        root.insert(insert_at, ET.Element('poly', a))
    for m in marks:
        root.append(ET.fromstring(m))
    # 정지선과 겹치는 흰 점선은 지운다
    stops = unary_union([LineString(_read(ET.fromstring(m).get('shape'))).buffer(0.45/2 + 0.1)
                         for m in marks if 'scross_stop_' in m])
    removed = 0
    for p in list(root.findall('poly')):
        if p.get('fill') == '0' and p.get('color') == WHITE and p.get('lineWidth') == '0.15' \
                and not p.get('id', '').startswith('scross_stop_'):
            pts = _read(p.get('shape'))
            if len(pts) >= 2 and LineString(pts).buffer(0.075).intersects(stops):
                root.remove(p)
                removed += 1
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'clipped_lines': clipped, 'new_marks': len(polys) + len(marks), 'dashes_on_stop_removed': removed}


def _junction_guides(net, plan, poly, dash, gap, node, approaches):
    """교차로 안의 좌회전·직진 점선 유도선 (실제 교차로 내부 경로 기준).

    - 직진: 이웃한 직진 차로 두 경로의 가운데(차로 사이 경계)만. 좌회전 차로·우회전 차로와의
      경계와 중앙선 쪽 경계에는 만들지 않는다.
    - 좌회전: 가장 안쪽 좌회전 경로를 차로 폭 절반만큼 안쪽(왼쪽)으로 띄운 경계.
      좌회전 차로가 둘이면 두 경로 사이 경계도 그린다.
    """
    from shapely.geometry import LineString
    internal = {}
    for e in net.findall('edge'):
        if e.get('function') == 'internal':
            for l in e.findall('lane'):
                internal[l.get('id')] = _read(l.get('shape'))
    conns = net.findall('connection')
    nxt = {}
    for c in conns:
        if c.get('from', '').startswith(':') and c.get('via'):
            nxt[f"{c.get('from')}_{c.get('fromLane')}"] = c.get('via')

    def path(c):
        pts, via = [], c.get('via')
        while via:
            seg = internal[via]
            pts += seg if not pts else seg[1:]
            via = nxt.get(via)
        return pts

    def resample(pts, n=200):
        g = LineString(pts)
        return [g.interpolate(g.length*i/n).coords[0] for i in range(n+1)]

    def mid(a, b):
        a, b = resample(a), resample(b)
        return [((p[0]+q[0])/2, (p[1]+q[1])/2) for p, q in zip(a, b)]

    def dashed(pid, pts):
        g = LineString(pts)
        t, j = 0.0, 0
        while t + 0.5 < g.length:
            seg = [g.interpolate(x).coords[0] for x in (t, min(t + dash, g.length))]
            poly(f'{pid}_{j}', WHITE, seg)
            t += dash + gap
            j += 1

    for eid, arrows in approaches.items():
        by_lane = {}
        for c in conns:
            if c.get('from') == eid and c.get('via', '').startswith(f':{node}_'):
                by_lane.setdefault(int(c.get('fromLane')), []).append(c)
        straight = sorted(k for k, a in arrows.items() if 'straight' in a)
        lefts = sorted(k for k, a in arrows.items() if 'left' in a)

        def pick(k, kind):
            cs = [c for c in by_lane.get(k, []) if c.get('dir') == kind]
            return max(cs, key=lambda c: int(c.get('toLane'))) if cs else None
        # 직진: 직진 차로끼리 사이 경계
        for k0, k1 in zip(straight, straight[1:]):
            c0, c1 = pick(k0, 's'), pick(k1, 's')
            if c0 is not None and c1 is not None:
                dashed(f'scross_guide_straight_{eid}_{k0}', mid(path(c0), path(c1)))
        # 좌회전: 가장 안쪽 경로의 안쪽 경계 + (둘이면) 두 경로 사이 경계
        lpaths = [path(pick(k, 'l')) for k in lefts if pick(k, 'l') is not None]
        if lpaths:
            inner = LineString(lpaths[-1]).offset_curve(LANE_W/2)
            dashed(f'scross_guide_left_{eid}_inner', list(inner.coords))
            for j, (pa, pb) in enumerate(zip(lpaths, lpaths[1:])):
                dashed(f'scross_guide_left_{eid}_between{j}', mid(pa, pb))

