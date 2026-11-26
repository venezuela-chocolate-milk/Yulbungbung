"""남쪽 6차선 도로(S 방향)를 동쪽 T자 교차로의 서쪽 가지(SW_end_T_right)까지 잇는다.

구성
  1) 검은 도로(SW_final_forward_in/out)를 자기 방향으로 D 만큼 늘린다.
     그 아래 묶음(T자 교차로, 서쪽 가지, 동남쪽 긴 도로와 끝 교차로 전체)은 모양 그대로
     같은 방향으로 평행이동한다. 검은 도로 방향은 서쪽 가지 축과 직각이라
     이동하면 서쪽 가지 축이 남쪽으로 평행하게 내려온다.
  2) 남쪽 도로: 기존 2차로 끝에서 안쪽 차로가 하나 더 생겨 방향별 3차로가 되고(기존과 동일),
     직선 → 완만한 곡선(25° 회전) → 긴 직선 → 완만한 곡선(나머지 25°) → 직선으로
     서쪽 가지 축에 접선 방향으로 들어간다. 곡률은 sin² 분포라 곡률까지 연속이다.
  3) 연결점보다 조금 앞에서 바깥쪽에 차로 2개가 테이퍼로 생겨 방향별 5차로, 총 10차로가 되고,
     같은 구간에서 중앙 포장이 0 → 1.0m 로 넓어져 서쪽 가지 단면과 정확히 일치한다.
  모든 차로 폭은 3.2m 로 동일하다. 테이퍼는 차로가 아닌 포장 장식이다.

    python -m env.roundabout_south_link
원본은 reports/south-road-before/ 에서 매번 다시 읽는다.
"""
import bisect
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
BEFORE_DIR = os.path.join(BASE, 'reports', 'south-road-before')

LANE_W = 3.2
SPEED = '16.670000'
YELLOW, WHITE, ASPHALT = '235,190,40', '255,255,255', '84,84,90'
CENTER_GAP = 0.3

# 남쪽 시작부 (기존 6차선 연장과 동일)
START_TAPER = 40.0       # 추가 안쪽 차로 앞 포장 테이퍼
SHIFT = 60.0             # 두 방향이 맞닿을 때까지
# 연결 도로 선형
STRAIGHT0 = 100.0        # 시작 직선 (합류 구간 포함)
TURN1_DEG = 25.0         # 첫 회전 (빨간 도로 '약간 회전')
CURVE1 = 300.0
CURVE2 = 350.0
FINAL_STRAIGHT = 250.0   # 서쪽 가지와 같은 방향의 마지막 직선 (목표값)
# 10차로 확장
WIDE_LEN = 100.0         # 연결점 앞 5차로 구간
WIDE_TAPER = 80.0        # 차로 2개 추가 테이퍼 (6.4m / 80m)
MEDIAN_HALF = 0.5        # 서쪽 가지 중앙 포장 반폭

# 이름
OUT_EDGE, IN_EDGE = 'S_out0_far', 'S_in1_far'
OUT_NODE, IN_NODE = 'S_end0', 'S_end1'
OUT_EXT, IN_EXT = 'S_out0_ext', 'S_in1_ext'
OUT_WIDE, IN_WIDE = 'S_out0_wide', 'S_in1_wide'
OUT_ADD, IN_ADD = 'S_add0', 'S_add1'
STUB_NODE = 'SW_end_T_right'
STUB_IN, STUB_OUT = 'SW_end_T_right_in', 'SW_end_T_right_out'   # in: T 방향(동), out: T 에서 서쪽
BLACK = ('SW_final_forward_in', 'SW_final_forward_out')
TRIANGLE, T_NODE = 'SE_triangle_SW', 'SW_final_forward'


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


# ───────────────────────── 1. 검은 도로 연장 + 평행이동 ─────────────────────────

def _cluster(root):
    adj = {}
    for e in root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        adj.setdefault(e.get('from'), set()).add(e.get('to'))
        adj.setdefault(e.get('to'), set()).add(e.get('from'))
    seen, stack = set(), [T_NODE]
    while stack:
        n = stack.pop()
        if n in seen or n == TRIANGLE:
            continue
        seen.add(n)
        stack += list(adj.get(n, ()))
    for e in root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        ends = {e.get('from'), e.get('to')}
        if ends & seen and not ends <= seen:
            assert e.get('id') in BLACK, e.get('id')
    return seen


class Stretch:
    def __init__(self, root):
        edges = {e.get('id'): e for e in root.findall('edge')}
        a, b = _read(edges['SW_final_forward_out'].get('shape'))
        self.a = a
        self.w = _unit(a, b)
        self.black_len = math.dist(a, b)
        self.cluster = _cluster(root)
        pat = '|'.join(re.escape(j) for j in self.cluster)
        self.internal = re.compile(rf'^:({pat})_\d+(_\d+)?$')

    def station(self, p):
        return (p[0]-self.a[0])*self.w[0] + (p[1]-self.a[1])*self.w[1]

    def move(self, p, D):
        return (p[0]+D*self.w[0], p[1]+D*self.w[1])


def _black_dash_info(deco_root, st):
    """검은 도로 점선 그룹: 주기와 '빈 구간'에 걸리지 않는 절단 위치."""
    groups = {}
    for p in deco_root.findall('poly'):
        pid = p.get('id', '')
        if pid.startswith(('lane_dash_SW_final_forward_in_', 'SW_forward_aligned_dash_')):
            key = pid.rsplit('_', 1)[0]
            pts = _read(p.get('shape'))
            groups.setdefault(key, []).append((min(st.station(q) for q in pts), p))
    period = None
    spans = []
    for key, items in groups.items():
        items.sort(key=lambda t: t[0])
        diffs = [b[0]-a[0] for a, b in zip(items, items[1:])]
        diffs = [d for d in diffs if d < 10]
        period = sum(diffs)/len(diffs) if period is None else period
        for s0, p in items:
            pts = _read(p.get('shape'))
            ss = [st.station(q) for q in pts]
            spans.append((min(ss), max(ss)))
    # 검은 도로 가운데쯤, 어느 점선에도 걸리지 않는 위치
    mid = st.black_len/2
    cand = [mid + d*0.1 for d in range(-200, 200)]
    cut = min((c for c in cand if all(not (lo-0.3 <= c <= hi+0.3) for lo, hi in spans)),
              key=lambda c: abs(c-mid))
    return groups, period, cut


def stretch_black(net_root, deco_root, D, pre=None):
    """pre: 이동 전 net 루트(노면 표시 분류용). 없으면 원본 스냅샷(BEFORE_DIR)을 읽는다."""
    st = Stretch(net_root)
    groups, period, cut = _black_dash_info(deco_root, st)
    D = round(D/period)*period
    moved_ids = set()

    # --- net ---
    def tr_pts(v, fn):
        return _write([fn(p) for p in _read(v)])

    move_all = lambda p: st.move(p, D)
    move_cut = lambda p: st.move(p, D) if st.station(p) > cut else p
    for e in net_root.findall('edge'):
        eid = e.get('id')
        if e.get('function') == 'internal':
            fn = move_all if st.internal.match(eid) else None
        elif eid in BLACK:
            fn = move_cut
        elif e.get('from') in st.cluster and e.get('to') in st.cluster:
            fn = move_all
        else:
            fn = None
        if fn is None:
            continue
        moved_ids.add(eid)
        if e.get('shape'):
            e.set('shape', tr_pts(e.get('shape'), fn))
        for lane in e.findall('lane'):
            lane.set('shape', tr_pts(lane.get('shape'), fn))
            if eid in BLACK:
                lane.set('length', f'{_length(_read(lane.get("shape"))):.6f}')
    for j in net_root.findall('junction'):
        jid = j.get('id')
        if jid in st.cluster or (jid.startswith(':') and st.internal.match(jid)):
            x, y = st.move((float(j.get('x')), float(j.get('y'))), D)
            j.set('x', f'{x:.6f}')
            j.set('y', f'{y:.6f}')
            if j.get('shape'):
                j.set('shape', tr_pts(j.get('shape'), move_all))

    # --- deco: 점마다 '움직이는 도로'에 더 가까우면 이동 ---
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree
    moving, static = [], []
    if pre is None:
        pre = ET.parse(os.path.join(BEFORE_DIR, 'roundabout.net.xml')).getroot()
    pre_internal = st.internal
    for e in pre.findall('edge'):
        eid = e.get('id')
        is_mov = (e.get('function') == 'internal' and pre_internal.match(eid)) or (
            e.get('function') != 'internal' and e.get('from') in st.cluster and e.get('to') in st.cluster)
        for lane in e.findall('lane'):
            pts = _read(lane.get('shape'))
            if len(pts) < 2:
                continue
            g = LineString(pts)
            if eid in BLACK:
                a = [p for p in pts]
                # 검은 도로: 절단 앞/뒤로 나눠 담는다
                s0, s1 = st.station(a[0]), st.station(a[-1])
                f = (cut - s0)/(s1 - s0)
                m = (a[0][0]+(a[-1][0]-a[0][0])*f, a[0][1]+(a[-1][1]-a[0][1])*f)
                near, far = (LineString([a[0], m]), LineString([m, a[-1]])) if s0 < s1 else \
                            (LineString([m, a[-1]]), LineString([a[0], m]))
                static.append(near)
                moving.append(far)
            else:
                (moving if is_mov else static).append(g)
    for j in pre.findall('junction'):
        if j.get('shape') and not j.get('id').startswith(':'):
            pts = _read(j.get('shape'))
            if len(pts) >= 3:
                g = Polygon(pts).exterior
                (moving if j.get('id') in st.cluster else static).append(g)
    t_mov, t_sta = STRtree(moving), STRtree(static)

    def dist(tree, geoms, p):
        i = tree.nearest(p)
        return geoms[i].distance(p)

    n_moved = 0
    cache = {}
    # 검은 도로는 절단 앞/뒤로 나눠 담았으므로, 점이 '움직이는 쪽' 도로에 더 가까우면 이동한다.
    # (먼 쪽 교차로의 북쪽 가지처럼 절단선 뒤로 넘어가는 부분도 묶음과 함께 움직여야 한다)
    for poly in deco_root.findall('poly'):
        if poly.get('id') == 'ground':      # 배경과 같은 색의 바닥 사각형 — 그대로 둔다
            continue
        pts = _read(poly.get('shape'))
        new = []
        changed = False
        for p in pts:
            if True:
                key = (round(p[0], 3), round(p[1], 3))
                if key not in cache:
                    P = Point(p)
                    cache[key] = dist(t_mov, moving, P) < dist(t_sta, static, P)
                if cache[key]:
                    p = st.move(p, D)
                    changed = True
            new.append(p)
        if changed:
            poly.set('shape', _write(new))
            n_moved += 1

    # --- 늘어난 구간 점선 채우기 (주기 그대로) ---
    filled = 0
    for key, items in groups.items():
        before = [(s0, p) for s0, p in items if s0 < cut]
        s_last, tmpl = max(before, key=lambda t: t[0])
        pts = _read(tmpl.get('shape'))   # 절단 앞이라 이동되지 않은 원형
        k = 1
        while s_last + k*period < cut + D - 1e-6:
            if True:
                el = ET.Element('poly', dict(tmpl.attrib))
                el.set('id', f'{tmpl.get("id")}_ext{k}')
                el.set('shape', _write([st.move(p, k*period) for p in pts]))
                parent_index = list(deco_root).index(tmpl)
                deco_root.insert(parent_index+1, el)
                filled += 1
            k += 1
    return {'D': D, 'cut': cut, 'period': period, 'deco_moved': n_moved, 'dash_filled': filled,
            'cluster_nodes': len(st.cluster)}


# ───────────────────────── 2. 연결 도로 선형 ─────────────────────────

class Axis:
    """도로 중심축. sigma = 축 거리(남쪽 끝 접속점 = 0), lateral + = 북행/서행(in) 쪽."""

    def __init__(self, net_root, D_target):
        edges = {e.get('id'): e for e in net_root.findall('edge')}
        junctions = {j.get('id'): j for j in net_root.findall('junction')}
        a, b = _read(edges[OUT_EDGE].get('shape'))[0], _read(edges[OUT_EDGE].get('shape'))[-1]
        self.u = _unit(a, b)
        p0 = tuple(float(junctions[OUT_NODE].get(k)) for k in 'xy')
        p1 = tuple(float(junctions[IN_NODE].get(k)) for k in 'xy')
        self.o = ((p0[0]+p1[0])/2, (p0[1]+p1[1])/2)
        n = (-self.u[1], self.u[0])
        rel = lambda p: ((p[0]-self.o[0])*self.u[0]+(p[1]-self.o[1])*self.u[1],
                         (p[0]-self.o[0])*n[0]+(p[1]-self.o[1])*n[1])
        self.s0, l0 = rel(p0)
        self.s1, l1 = rel(p1)
        self.c0, self.c1 = -l0, l1
        sa, sb = _read(edges[STUB_IN].get('shape'))
        self.b = _unit(sa, sb)
        self.h0 = math.atan2(self.u[1], self.u[0])
        self.h1 = math.atan2(self.b[1], self.b[0])
        self.turn = self.h1 - self.h0
        self.th1 = math.radians(TURN1_DEG)
        self.th2 = self.turn - self.th1
        self.D_target = D_target

    def integrate(self, S, L3, ds=0.25, back=START_TAPER+10):
        segs = [(STRAIGHT0, 0.0), (CURVE1, self.th1), (S, 0.0), (CURVE2, self.th2), (L3, 0.0)]
        h = self.h0
        # 뒤쪽 직선
        n = int(round(back/ds))
        sig, xs, ys, hs = [], [], [], []
        for i in range(n):
            s = -back + i*ds
            sig.append(s)
            xs.append(self.o[0]+s*self.u[0])
            ys.append(self.o[1]+s*self.u[1])
            hs.append(h)
        x, y, s = self.o[0], self.o[1], 0.0
        sig.append(s); xs.append(x); ys.append(y); hs.append(h)
        for Ls, th in segs:
            if Ls <= 0:
                continue
            n = max(1, int(math.ceil(Ls/ds)))
            d = Ls/n
            for i in range(n):
                t0, t1 = i/n, (i+1)/n
                # sin² 곡률 분포의 정확한 각도 적분
                if th:
                    F = lambda t: th*(t - math.sin(2*math.pi*t)/(2*math.pi))
                    hm = h + F((t0+t1)/2) - F(t0)
                    h_next = h + F(t1) - F(t0)
                else:
                    hm = h_next = h
                x += math.cos(hm)*d
                y += math.sin(hm)*d
                h = h_next
                s += d
                sig.append(s); xs.append(x); ys.append(y); hs.append(h)
        return sig, xs, ys, hs

    def solve(self, B0, stretch_w):
        """끝점이 이동된 서쪽 가지 축 위, 서쪽 가지 끝에 오도록 S, L3 를 정한다.
        D 는 직선 길이 S 로 맞추고(점선 주기에 맞춘 D_target), L3 는 축 방향 위치로 맞춘다."""
        b, nb = self.b, (-self.b[1], self.b[0])
        target_n = (B0[0]*nb[0]+B0[1]*nb[1]) - self.D_target*(stretch_w[0]*(-nb[0])+stretch_w[1]*(-nb[1]))

        def end(S, L3):
            _, xs, ys, _ = self.integrate(S, L3, ds=0.5)
            return xs[-1], ys[-1]

        lo, hi = 0.0, 3000.0
        for _ in range(60):
            mid = (lo+hi)/2
            x, y = end(mid, 0.0)
            if x*nb[0]+y*nb[1] > target_n:
                lo = mid
            else:
                hi = mid
        S = (lo+hi)/2
        Bm = (B0[0]+self.D_target*stretch_w[0], B0[1]+self.D_target*stretch_w[1])
        x, y = end(S, 0.0)
        L3 = (Bm[0]-x)*b[0]+(Bm[1]-y)*b[1]
        self.S, self.L3 = S, L3
        self.sig, self.xs, self.ys, self.hs = self.integrate(S, L3)
        self.end = self.sig[-1]
        self.node = self.end - WIDE_LEN
        self.taper0 = self.node - WIDE_TAPER
        err = math.dist((self.xs[-1], self.ys[-1]), Bm)
        return {'S': S, 'L3': L3, 'end_error_m': err, 'heading_error_deg': math.degrees(self.hs[-1]-self.h1),
                'Rmin1': CURVE1/(2*self.th1), 'Rmin2': CURVE2/(2*self.th2), 'total': self.end}

    def at(self, s):
        i = min(max(bisect.bisect_left(self.sig, s), 1), len(self.sig)-1)
        s0, s1 = self.sig[i-1], self.sig[i]
        f = (s-s0)/(s1-s0) if s1 > s0 else 0
        x = self.xs[i-1]+(self.xs[i]-self.xs[i-1])*f
        y = self.ys[i-1]+(self.ys[i]-self.ys[i-1])*f
        h = self.hs[i-1]+(self.hs[i]-self.hs[i-1])*f
        return x, y, h

    def P(self, s, l):
        x, y, h = self.at(s)
        return (x - l*math.sin(h), y + l*math.cos(h))

    def local_start(self, p):
        dx, dy = p[0]-self.o[0], p[1]-self.o[1]
        return dx*self.u[0]+dy*self.u[1]

    # 각 방향 안쪽 경계 |l|
    def inner(self, side, s):
        c, s_start = (self.c1, self.s1) if side > 0 else (self.c0, self.s0)
        if s < s_start:
            return (c - LANE_W) - LANE_W*H((s - (s_start - START_TAPER))/START_TAPER)
        if s < self.taper0:
            return (c + LANE_W - 3*LANE_W)*(1 - H((s - s_start)/SHIFT))
        return MEDIAN_HALF*H((s - self.taper0)/WIDE_TAPER)

    def lanes(self, s):
        return 3 if s < self.node else 5

    def center(self, side, s, lanes):
        return side*(self.inner(side, s) + lanes*LANE_W/2)

    def stations(self, a, b, step=1.0):
        n = max(1, int(math.ceil((b-a)/step)))
        return [a+(b-a)*i/n for i in range(n+1)]

    def shape(self, side, a, b, lanes, step=1.0):
        pts = [self.P(s, self.center(side, s, lanes)) for s in self.stations(a, b, step)]
        return pts if side < 0 else list(reversed(pts))


# ───────────────────────── 3. netconvert 부분망 + 접합 ─────────────────────────

def _subnet(ax, root, work):
    edges = {e.get('id'): e for e in root.findall('edge')}
    junctions = {j.get('id'): j for j in root.findall('junction')}
    out_pts = _read(edges[OUT_EDGE].get('shape'))
    in_pts = _read(edges[IN_EDGE].get('shape'))
    sin_pts = _read(edges[STUB_IN].get('shape'))
    sout_pts = _read(edges[STUB_OUT].get('shape'))
    stub = tuple(float(junctions[STUB_NODE].get(k)) for k in 'xy')
    add0 = ax.P(ax.node, -(MEDIAN_HALF+3*LANE_W/2))
    add1 = ax.P(ax.node, (MEDIAN_HALF+3*LANE_W/2))
    nodes = [('U0', out_pts[0], 'dead_end'), (OUT_NODE, out_pts[-1], 'priority'),
             ('U1', in_pts[-1], 'dead_end'), (IN_NODE, in_pts[0], 'priority'),
             (OUT_ADD, add0, 'priority'), (IN_ADD, add1, 'priority'),
             (STUB_NODE, stub, 'priority'), ('T0', sin_pts[-1], 'dead_end'), ('T1', sout_pts[0], 'dead_end')]
    with open(os.path.join(work, 'p.nod.xml'), 'w') as f:
        f.write('<nodes>\n')
        for nid, (x, y), t in nodes:
            f.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}" radius="0"/>\n')
        f.write('</nodes>\n')
    s_out, s_in = ax.s0, ax.s1
    spec = [
        (OUT_EDGE, 'U0', OUT_NODE, 2, [out_pts[0], out_pts[-1]]),
        (OUT_EXT, OUT_NODE, OUT_ADD, 3, ax.shape(-1, s_out, ax.node, 3)),
        (OUT_WIDE, OUT_ADD, STUB_NODE, 5, ax.shape(-1, ax.node, ax.end, 5, 5.0)),
        (STUB_IN, STUB_NODE, 'T0', 5, sin_pts),
        (STUB_OUT, 'T1', STUB_NODE, 5, sout_pts),
        (IN_WIDE, STUB_NODE, IN_ADD, 5, ax.shape(1, ax.node, ax.end, 5, 5.0)),
        (IN_EXT, IN_ADD, IN_NODE, 3, ax.shape(1, s_in, ax.node, 3)),
        (IN_EDGE, IN_NODE, 'U1', 2, [in_pts[0], in_pts[-1]]),
    ]
    with open(os.path.join(work, 'p.edg.xml'), 'w') as f:
        f.write('<edges>\n')
        for eid, fr, to, n, pts in spec:
            f.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{SPEED}" '
                    f'priority="1" width="{LANE_W}" spreadType="center" shape="{_write(pts)}"/>\n')
        f.write('</edges>\n')
    con = []
    for k in (0, 1):
        con += [(OUT_EDGE, OUT_EXT, k, k), (IN_EXT, IN_EDGE, k, k)]
    for k in range(3):
        con += [(OUT_EXT, OUT_WIDE, k, k+2), (IN_WIDE, IN_EXT, k+2, k)]
    for k in range(5):
        con += [(OUT_WIDE, STUB_IN, k, k), (STUB_OUT, IN_WIDE, k, k)]
    with open(os.path.join(work, 'p.con.xml'), 'w') as f:
        f.write('<connections>\n')
        for a, b, i, j in con:
            f.write(f'  <connection from="{a}" to="{b}" fromLane="{i}" toLane="{j}"/>\n')
        f.write('</connections>\n')
    from sumolib import checkBinary
    out = os.path.join(work, 'p.net.xml')
    r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                        '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                        '--junctions.corner-detail', '24', '--precision', '6', '-o', out],
                       cwd=work, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr)
    return ET.parse(out).getroot()


def splice_link(root, ax):
    with tempfile.TemporaryDirectory() as work:
        sub = _subnet(ax, root, work)
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}
    new_nodes = (OUT_NODE, IN_NODE, OUT_ADD, IN_ADD, STUB_NODE)
    # 기존 엣지: 접합점 쪽 끝점만 subnet 결과로
    for e in root.findall('edge'):
        eid = e.get('id')
        if eid not in (OUT_EDGE, IN_EDGE, STUB_IN, STUB_OUT):
            continue
        at_end = eid in (OUT_EDGE, STUB_OUT)
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

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    prefixes = tuple(f':{n}_' for n in new_nodes)
    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in (OUT_EXT, IN_EXT, OUT_WIDE, IN_WIDE) or e.get('id', '').startswith(prefixes):
            root.insert(pos, e)
            pos += 1
    for j in root.findall('junction'):
        if j.get('id') in new_nodes:
            root.remove(j)
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in new_nodes or j.get('id', '').startswith(prefixes):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        if c.get('from') in (OUT_EDGE, OUT_EXT, OUT_WIDE, STUB_OUT, IN_WIDE, IN_EXT) or \
                c.get('from', '').startswith(prefixes):
            root.insert(pos, c)
            pos += 1


def update_boundary(root):
    xs, ys = [], []
    for e in root.findall('edge'):
        for lane in e.findall('lane'):
            for x, y in _read(lane.get('shape')):
                xs.append(x); ys.append(y)
    loc = root.find('location')
    b = f'{min(xs):.6f},{min(ys):.6f},{max(xs):.6f},{max(ys):.6f}'
    loc.set('convBoundary', b)
    loc.set('origBoundary', b)


# ───────────────────────── 4. 노면 표시 ─────────────────────────

def _resample(pts, step=0.25):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(math.dist(a, b)/step)))
        out += [(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(1, n+1)]
    return out


def _cum(pts):
    c = [0.0]
    for a, b in zip(pts, pts[1:]):
        c.append(c[-1]+math.dist(a, b))
    return c


def _piece(pts, cum, a, b):
    return [p for p, c in zip(pts, cum) if a <= c <= b]


def _separator(l1, l2):
    a, b = _resample(l1), _resample(l2)
    # 같은 비율 위치끼리 평균 (두 차로 길이가 거의 같다)
    ca, cb = _cum(a), _cum(b)
    out = []
    j = 0
    for p, c in zip(a, ca):
        t = c/ca[-1]*cb[-1]
        while j < len(cb)-2 and cb[j+1] < t:
            j += 1
        f = (t-cb[j])/(cb[j+1]-cb[j]) if cb[j+1] > cb[j] else 0
        q = (b[j][0]+(b[j+1][0]-b[j][0])*f, b[j][1]+(b[j+1][1]-b[j][1])*f)
        out.append(((p[0]+q[0])/2, (p[1]+q[1])/2))
    return out


def add_markings(deco_root, net_root, ax):
    from env.roundabout_markings import dash_style
    shapes = {l.get('id'): _read(l.get('shape')) for e in net_root.findall('edge') for l in e.findall('lane')}
    dash, gap, _ = dash_style(20)
    period = dash + gap
    polys = []

    def poly(pid, color, pts, fill=False, layer='6', width='0.15'):
        attrs = dict(id=pid, color=color, fill='1' if fill else '0', layer=layer, shape=_write(pts))
        if not fill:
            attrs['lineWidth'] = width
        polys.append(attrs)

    def phase(prefix):
        starts = [min(ax.local_start(q) for q in _read(p.get('shape')))
                  for p in deco_root.findall('poly') if p.get('id', '').startswith(prefix)]
        return max(starts) % period

    def dashes(pid, sep, first_dash_at):
        """sep 위 거리 first_dash_at + k*period 에서 시작하는 점선."""
        cum = _cum(sep)
        total = cum[-1]
        k = math.floor((0 - first_dash_at)/period)
        j = 0
        while True:
            a0 = first_dash_at + k*period
            k += 1
            if a0 >= total - 0.5:
                break
            a1, b1 = max(a0, 0.0), min(a0+dash, total)
            if b1 - a1 < 0.5:
                continue
            piece = _piece(sep, cum, a1, b1)
            if len(piece) >= 2:
                poly(f'{pid}_{j}', WHITE, piece)
                j += 1

    # (a) 3차로 구간 점선: 남쪽 끝 기존 점선 위상을 이어받는다
    for side, edge, prefix in ((-1, OUT_EXT, 'lane_dash_S_out0_0_'), (1, IN_EXT, f'lane_dash_{IN_EDGE}_0_')):
        ph = phase(prefix)
        lanes = [shapes[f'{edge}_{i}'] for i in range(3)]
        if side > 0:
            lanes = [list(reversed(l)) for l in lanes]
        for k in range(2):
            sep = _separator(lanes[k], lanes[k+1])
            s_first = ax.local_start(sep[0])
            poly_id = f'southlink_dash_{edge}_{k}'
            dashes(poly_id, sep, (ph - s_first) % period - period)
    # (b) 5차로 구간 점선: 서쪽 가지 점선 위상(가지 끝에서 동쪽으로 [0,dash])을 거꾸로 이어받는다
    for side, edge in ((-1, OUT_WIDE), (1, IN_WIDE)):
        lanes = [shapes[f'{edge}_{i}'] for i in range(5)]
        if side > 0:
            lanes = [list(reversed(l)) for l in lanes]   # 서→동 방향으로 통일, 끝이 가지 접속점
        for k in range(4):
            sep = _separator(lanes[k], lanes[k+1])
            total = _cum(sep)[-1]
            # 접속점(끝)에서 거리 d 뒤로 [period-dash, period] 가 점선 → 시작 기준 위치
            first = total - period + 0.0
            first = first - math.floor(first/period)*period
            dashes(f'southlink_dash_{edge}_{k}', sep, first - period)

    # (c) 시작부 안쪽 차로 테이퍼 포장 (기존 6차선 연장과 동일)
    for side, s_start, c in ((-1, ax.s0, ax.c0), (1, ax.s1, ax.c1)):
        ss = ax.stations(s_start-START_TAPER, s_start, 0.5)
        old_inner = c - LANE_W
        curve = [ax.P(s, side*ax.inner(side, s)) for s in ss]
        edge_line = [ax.P(s, side*(old_inner+0.05)) for s in reversed(ss)]
        cap = [ax.P(s_start+0.6, side*ax.inner(side, s_start+0.6)), ax.P(s_start+0.6, side*(old_inner+0.05))]
        poly(f'southlink_start_taper_{"in" if side > 0 else "out"}', ASPHALT, curve+cap+edge_line,
             fill=True, layer='4.9')

    # (d) 10차로 확장 테이퍼 포장 (바깥쪽 2개 차로)
    for side in (-1, 1):
        ss = ax.stations(ax.taper0, ax.node, 0.5)
        base = [ax.P(s, side*(ax.inner(side, s)+3*LANE_W-0.05)) for s in ss]
        curve = [ax.P(s, side*(ax.inner(side, s)+3*LANE_W+2*LANE_W*H((s-ax.taper0)/WIDE_TAPER))) for s in ss]
        cap = [ax.P(ax.node+0.6, side*(MEDIAN_HALF+5*LANE_W)), ax.P(ax.node+0.6, side*(MEDIAN_HALF+3*LANE_W-0.05))]
        poly(f'southlink_wide_taper_{"in" if side > 0 else "out"}', ASPHALT,
             base + [cap[1], cap[0]] + list(reversed(curve)),
             fill=True, layer='4.9')

    # (e) 하부 포장 (곡선 차로 사이 미세한 틈 방지) — 3차로 / 5차로 구간
    for side, s_start in ((-1, ax.s0), (1, ax.s1)):
        for a, b, n in ((s_start-1.0, ax.node+0.5, 3), (ax.node-0.5, ax.end+0.5, 5)):
            ss = ax.stations(a, b, 1.0)
            outer = [ax.P(s, side*(ax.inner(side, max(s, s_start))+n*LANE_W)) for s in ss]
            inner = [ax.P(s, side*ax.inner(side, max(s, s_start))) for s in reversed(ss)]
            poly(f'southlink_under_{"in" if side > 0 else "out"}_{n}', ASPHALT, outer+inner, fill=True, layer='-1')
    # 중앙 포장(넓어지는 구간)
    ss = ax.stations(ax.taper0, ax.end+0.5, 1.0)
    poly('southlink_median_pavement', ASPHALT,
         [ax.P(s, ax.inner(1, s)+0.05) for s in ss] + [ax.P(s, -ax.inner(-1, s)-0.05) for s in reversed(ss)],
         fill=True, layer='-1')

    # (f) 황색 복선 중앙선
    s_lo = max(ax.s0, ax.s1)
    s_meet = next(s for s in ax.stations(s_lo, s_lo+SHIFT, 0.1)
                  if max(ax.inner(1, s), ax.inner(-1, s)) <= 0.075)
    for k, sgn in enumerate((-1, 1)):
        poly(f'southlink_centerline_{k}', YELLOW,
             [ax.P(s, sgn*CENTER_GAP/2) for s in ax.stations(s_meet, ax.end, 1.0)])

    insert_at = next((i for i, el in enumerate(list(deco_root)) if el.get('id', '').startswith('lane_dash_')),
                     len(deco_root))
    for attrs in reversed(polys):
        deco_root.insert(insert_at, ET.Element('poly', attrs))
    return {'link_polys': len(polys), 'dashes': sum('dash' in p['id'] for p in polys)}


# ───────────────────────── main ─────────────────────────

def main():
    sys.path.insert(0, BASE)
    net_path = os.path.join(NET_DIR, 'roundabout.net.xml')
    deco_path = os.path.join(NET_DIR, 'roundabout.deco.xml')
    net_tree = ET.parse(os.path.join(BEFORE_DIR, 'roundabout.net.xml'))
    deco_tree = ET.parse(os.path.join(BEFORE_DIR, 'roundabout.deco.xml'))
    net, deco = net_tree.getroot(), deco_tree.getroot()

    # 1) 필요한 D 를 먼저 구한다 (평행이동 전 좌표 기준)
    st = Stretch(net)
    _, period, _ = _black_dash_info(deco, st)
    junctions = {j.get('id'): j for j in net.findall('junction')}
    B0 = tuple(float(junctions[STUB_NODE].get(k)) for k in 'xy')
    probe = Axis(net, 0.0)
    nb = (-probe.b[1], probe.b[0])
    # 직선 길이 S 를 바꿔가며 L3 가 목표가 되는 D 를 찾는다
    lo, hi = 0.0, 3000.0
    for _ in range(50):
        D = (lo+hi)/2
        probe.D_target = D
        info = probe.solve(B0, st.w)
        if info['L3'] > FINAL_STRAIGHT:
            lo = D
        else:
            hi = D
    D = round(((lo+hi)/2)/period)*period

    # 2) 검은 도로 연장 + 묶음 평행이동
    rep = stretch_black(net, deco, D)

    # 3) 연결 도로
    ax = Axis(net, 0.0)
    ax.D_target = 0.0
    jn = {j.get('id'): j for j in net.findall('junction')}
    B1 = tuple(float(jn[STUB_NODE].get(k)) for k in 'xy')
    geo = ax.solve(B1, st.w)
    splice_link(net, ax)
    update_boundary(net)
    ET.indent(net_tree, space='    ')
    net_tree.write(net_path, encoding='utf-8', xml_declaration=True)

    net2 = ET.parse(net_path).getroot()
    mk = add_markings(deco, net2, ax)
    ET.indent(deco_tree, space='    ')
    deco_tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    print({**rep, **{k: round(v, 3) for k, v in geo.items()}, **mk,
           'node_sigma': round(ax.node, 1), 'taper0_sigma': round(ax.taper0, 1)})


if __name__ == '__main__':
    main()
