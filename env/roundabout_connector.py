"""위쪽 삼각형 교차로의 동쪽 끝 도로(SE_triangle_forward, 방향별 2차로)를
아래쪽 긴 도로(유턴이 있는 SW_end_T_left_*)에 곡선 도로로 잇는다.

  - 연결 지점 J: 유턴 지점에서 동남쪽으로 J_FROM_UTURN m. 위 도로가 긴 도로 북쪽에 붙는 T자 교차로.
    신호 없음. 중앙선이 막혀 있어 우회전으로만 들고 난다. 위 도로는 정지선에서 멈춘 뒤 진입.
  - 선형: 위 도로 끝에서 같은 방향 직선 → 반지름 TURN_RADIUS 원호 → 긴 도로에 직각인 직선으로 들어간다.
  - 단면: 위 도로와 같음(방향별 2차로, 중앙 포장 1.0m, 황색 복선).
    J 앞 REDUCE_AT m 부터 REDUCE_TAPER m 동안 양쪽 바깥 차로가 줄어 방향별 1차로가 된다.
  - 긴 도로는 J 에서 나뉜다: J~유턴 쪽은 기존 *_far 이름 유지, J~먼 쪽은 *_far2.
roundabout_south_extend 가 유턴/연장 다음 단계로 호출한다.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from env import roundabout_long_uturn as uturn

LANE_W = 3.2
MEDIAN_HALF = 0.5
J_FROM_UTURN = 280.0    # 2026-11-10 실측: 합류 지점에서 서쪽 280m 에 유턴
STRAIGHT_IN = 150.0
# 선형 (2026-11-10, 실제 도로처럼): 위 도로 방향 그대로 긴 직선 → 반지름 TURN_RADIUS 원호 → 긴 도로에 직각인
# 짧은 직선으로 합류. 마지막 직선이 MIN_STRAIGHT_IN 보다 짧아지면 원호 반지름을 줄인다.
TURN_RADIUS = 200.0
MIN_STRAIGHT_IN = 60.0
REDUCE_AT = 100.0
REDUCE_TAPER = 40.0
BEZIER_K = 450.0
BEZIER_FRAC = 0.55     # 원호에 가까운 곡선이 되는 3차 베지어 제어점 비율
YELLOW, WHITE, ASPHALT = '235,190,40', '255,255,255', '84,84,90'

ARM_OUT, ARM_IN, ARM_END = 'SE_triangle_forward_out', 'SE_triangle_forward_in', 'SE_triangle_forward'
J_NODE, N1, N2 = 'SW_long_conn', 'S_conn_n1', 'S_conn_n2'
C_DOWN2, C_DOWN1 = 'S_conn_down', 'S_conn_down1'      # 위 → 아래 (2차로 → 1차로)
C_UP1, C_UP2 = 'S_conn_up1', 'S_conn_up'               # 아래 → 위 (1차로 → 2차로)
IN_FAR2, OUT_FAR2 = 'SW_end_T_left_in_far2', 'SW_end_T_left_out_far2'


def H(t):
    t = min(1.0, max(0.0, t))
    return t*t*t*(10 - 15*t + 6*t*t)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class Curve:
    """연결 도로 중심축(중앙 포장 가운데). s = 위 도로 끝에서 거리, l = 왼쪽(+, 올라가는 쪽)."""

    def __init__(self, S, hs, J, hj):
        # 위 도로 연장선과 J 에서 거꾸로 올린 직각선의 교점 V, 그 꼭짓점을 원호로 둥글린다
        den = hs[0]*hj[1] - hs[1]*hj[0]
        a = ((J[0]-S[0])*hj[1] - (J[1]-S[1])*hj[0]) / den       # S → V 거리
        b = -((J[0]-S[0])*hs[1] - (J[1]-S[1])*hs[0]) / den      # V → J 거리
        V = (S[0] + hs[0]*a, S[1] + hs[1]*a)
        delta = math.atan2(den, hs[0]*hj[0] + hs[1]*hj[1])      # 부호 있는 회전각
        tq = math.tan(abs(delta)/2)
        R = min(TURN_RADIUS, (b - MIN_STRAIGHT_IN)/tq, (a - 1.0)/tq)
        assert R > 50.0 and a > 0 and b > 0, f'곡선을 만들 수 없다: a={a:.1f}, b={b:.1f}, R={R:.1f}'
        t = R*tq
        P1 = (V[0] - hs[0]*t, V[1] - hs[1]*t)
        sg = 1.0 if delta > 0 else -1.0
        C = (P1[0] - hs[1]*R*sg, P1[1] + hs[0]*R*sg)
        r0 = (P1[0]-C[0], P1[1]-C[1])
        raw = [(S[0] + hs[0]*(a - t)*i/200, S[1] + hs[1]*(a - t)*i/200) for i in range(200)]
        n = max(8, int(abs(delta)*R/0.5))
        for i in range(n+1):
            f = delta*i/n
            c, si = math.cos(f), math.sin(f)
            raw.append((C[0] + c*r0[0] - si*r0[1], C[1] + si*r0[0] + c*r0[1]))
        P2 = raw[-1]
        m = max(2, int(b - t))
        raw += [(P2[0] + (J[0]-P2[0])*i/m, P2[1] + (J[1]-P2[1])*i/m) for i in range(1, m+1)]
        self.radius = R
        self.pts = raw
        self.cum = [0.0]
        for p, q in zip(raw, raw[1:]):
            self.cum.append(self.cum[-1] + math.dist(p, q))
        self.L = self.cum[-1]

    def at(self, s):
        import bisect
        s = min(max(s, 0.0), self.L)
        i = min(max(bisect.bisect_left(self.cum, s), 1), len(self.cum)-1)
        p, q = self.pts[i-1], self.pts[i]
        seg = self.cum[i] - self.cum[i-1] or 1e-9
        f = (s - self.cum[i-1])/seg
        h = ((q[0]-p[0])/seg, (q[1]-p[1])/seg)
        return (p[0]+(q[0]-p[0])*f, p[1]+(q[1]-p[1])*f), h

    def P(self, s, l):
        p, h = self.at(s)
        return (p[0] - h[1]*l, p[1] + h[0]*l)

    def stations(self, a, b, step=1.0):
        n = max(1, int(math.ceil((b-a)/step)))
        return [a+(b-a)*i/n for i in range(n+1)]

    def min_radius(self):
        best = 1e9
        for s in self.stations(5, self.L-5, 5.0):
            (_, h0), (_, h1) = self.at(s-5), self.at(s+5)
            d = math.atan2(h0[0]*h1[1]-h0[1]*h1[0], h0[0]*h1[0]+h0[1]*h1[1])
            if abs(d) > 1e-9:
                best = min(best, 10.0/abs(d))
        return best

    # 방향별 차로 수 / 바깥 경계
    def lanes(self, s):
        return 2 if s < self.L - REDUCE_AT else 1

    def outer(self, s):
        k = H((s - (self.L - REDUCE_AT))/REDUCE_TAPER)
        return MEDIAN_HALF + LANE_W*(2 - k)


def apply_net(net_path, ax, median):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    junctions = {j.get('id'): j for j in root.findall('junction')}
    lane = lambda eid, k: _read(next(l for l in edges[eid].findall('lane') if l.get('index') == str(k)).get('shape'))

    # 위 도로 끝 / 방향
    a0, a1 = _read(edges[ARM_OUT].get('shape'))[0], _read(edges[ARM_OUT].get('shape'))[-1]
    d = math.dist(a0, a1)
    hs = ((a1[0]-a0[0])/d, (a1[1]-a0[1])/d)
    S = tuple(float(junctions[ARM_END].get(k)) for k in 'xy')
    # 연결 지점 J (긴 도로 중앙 포장 가운데), 들어가는 방향 = 긴 도로 북쪽에서 남쪽으로
    st_j = uturn.U_ST + J_FROM_UTURN
    J = ax.P(st_j, median)
    hj = (-ax.n[0], -ax.n[1])
    cv = Curve(S, hs, J, hj)

    n_in = len(edges[uturn.IN_FAR].findall('lane'))
    n_out = len(edges[uturn.OUT_FAR].findall('lane'))
    in_lat = ax.lat(lane(uturn.IN_FAR, n_in//2)[-1])
    out_lat = (ax.lat(lane(uturn.OUT_FAR, 0)[0]) + ax.lat(lane(uturn.OUT_FAR, n_out-1)[0]))/2
    in_s0, in_s1 = ax.st(lane(uturn.IN_FAR, 0)[0]), ax.st(lane(uturn.IN_FAR, 0)[-1])     # 먼 쪽 → 유턴
    out_s0, out_s1 = ax.st(lane(uturn.OUT_FAR, 0)[0]), ax.st(lane(uturn.OUT_FAR, 0)[-1])  # 유턴 → 먼 쪽
    c2, c1 = MEDIAN_HALF + LANE_W, MEDIAN_HALF + LANE_W/2      # 2차로/1차로 엣지 중심 횡거리
    sN = cv.L - REDUCE_AT

    with tempfile.TemporaryDirectory() as work:
        arm_out, arm_in = _read(edges[ARM_OUT].get('shape')), _read(edges[ARM_IN].get('shape'))
        nodes = [(ARM_END, S, 'priority'), ('A0', arm_out[0], 'dead_end'), ('A1', arm_in[-1], 'dead_end'),
                 (J_NODE, J, 'priority'),          # 위 도로는 신호 없이 양보 후 진입 (완전 정지 아님)
                 (N1, cv.P(sN, -c1), 'priority'), (N2, cv.P(sN, c1), 'priority'),
                 ('Lf_in', ax.P(in_s0, in_lat), 'dead_end'), ('Lu_in', ax.P(in_s1, in_lat), 'dead_end'),
                 ('Lu_out', ax.P(out_s0, out_lat), 'dead_end'), ('Lf_out', ax.P(out_s1, out_lat), 'dead_end')]
        down2 = [cv.P(s, -c2) for s in cv.stations(0, sN, 2.0)]
        down1 = [cv.P(s, -c1) for s in cv.stations(sN, cv.L, 2.0)]
        up1 = [cv.P(s, c1) for s in reversed(cv.stations(sN, cv.L, 2.0))]
        up2 = [cv.P(s, c2) for s in reversed(cv.stations(0, sN, 2.0))]
        speed = edges[uturn.IN_FAR].findall('lane')[0].get('speed')
        spec = [(ARM_OUT, 'A0', ARM_END, 2, arm_out, 1), (ARM_IN, ARM_END, 'A1', 2, arm_in, 1),
                (C_DOWN2, ARM_END, N1, 2, down2, 1), (C_DOWN1, N1, J_NODE, 1, down1, 1),
                (C_UP1, J_NODE, N2, 1, up1, 1), (C_UP2, N2, ARM_END, 2, up2, 1),
                (IN_FAR2, 'Lf_in', J_NODE, n_in, [ax.P(in_s0, in_lat), ax.P(st_j, in_lat)], 3),
                (uturn.IN_FAR, J_NODE, 'Lu_in', n_in, [ax.P(st_j, in_lat), ax.P(in_s1, in_lat)], 3),
                (uturn.OUT_FAR, 'Lu_out', J_NODE, n_out, [ax.P(out_s0, out_lat), ax.P(st_j, out_lat)], 3),
                (OUT_FAR2, J_NODE, 'Lf_out', n_out, [ax.P(st_j, out_lat), ax.P(out_s1, out_lat)], 3)]
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            for nid, (x, y), t in nodes:
                fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            for eid, fr, to, n, pts, pr in spec:
                fh.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{speed}" '
                         f'priority="{pr}" width="{LANE_W}" spreadType="center" shape="{_write(pts)}"/>\n')
            fh.write('</edges>\n')
        con = [(ARM_OUT, C_DOWN2, 0, 0), (ARM_OUT, C_DOWN2, 1, 1), (C_UP2, ARM_IN, 0, 0), (C_UP2, ARM_IN, 1, 1),
               (C_DOWN2, C_DOWN1, 1, 0),              # 바깥(0번) 차로는 줄어든다
               (C_UP1, C_UP2, 0, 1),                  # 바깥(0번) 차로가 새로 생긴다
               (C_DOWN1, uturn.IN_FAR, 0, 0),         # 우회전 → 서행(유턴/T자 쪽)
               (IN_FAR2, C_UP1, 0, 0)]                # 서행 우회전 → 위 도로
        # 중앙선이 막혀 있으므로 좌회전(위 도로 → 동행, 동행 → 위 도로)은 없다
        con += [(IN_FAR2, uturn.IN_FAR, k, k) for k in range(n_in)]
        con += [(uturn.OUT_FAR, OUT_FAR2, k, k) for k in range(n_out)]
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            for a, b, i, j in con:
                fh.write(f'  <connection from="{a}" to="{b}" fromLane="{i}" toLane="{j}"/>\n')
            fh.write('</connections>\n')
        from sumolib import checkBinary
        out = os.path.join(work, 'p.net.xml')
        r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                            '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                            '--junctions.corner-detail', '24', '--precision', '6', '-o', out],
                           cwd=work, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        sub = ET.parse(out).getroot()
    sub_lane = {(e.get('id'), int(l.get('index'))): _read(l.get('shape'))
                for e in sub.findall('edge') for l in e.findall('lane')}

    # 1) 위 도로 두 엣지: 끝점만 subnet 결과로
    for eid, at_end in ((ARM_OUT, True), (ARM_IN, False)):
        for l in edges[eid].findall('lane'):
            pts = _read(l.get('shape'))
            new = sub_lane[(eid, int(l.get('index')))]
            if at_end:
                pts[-1] = new[-1]
            else:
                pts[0] = new[0]
            l.set('shape', _write(pts))
            l.set('length', f'{_length(pts):.6f}')

    # 2) 긴 도로 나누기: 유턴 쪽은 기존 이름, 먼 쪽은 *_far2
    e_in, e_out = edges[uturn.IN_FAR], edges[uturn.OUT_FAR]
    far_in = ET.Element('edge', dict(e_in.attrib)); far_in.set('id', IN_FAR2); far_in.set('to', J_NODE)
    far_out = ET.Element('edge', dict(e_out.attrib)); far_out.set('id', OUT_FAR2); far_out.set('from', J_NODE)
    for e, near, far, far_el in ((e_in, uturn.IN_FAR, IN_FAR2, far_in), (e_out, uturn.OUT_FAR, OUT_FAR2, far_out)):
        if e.get('shape'):
            sh = _read(e.get('shape'))
            far_el.set('shape', _write(uturn._split(sh, ax, st_j, False, ax.P(st_j, ax.lat(sh[0])))))
            e.set('shape', _write(uturn._split(sh, ax, st_j, True, ax.P(st_j, ax.lat(sh[0])))))
        for l in list(e.findall('lane')):
            k = int(l.get('index'))
            pts = _read(l.get('shape'))
            near_pts = uturn._split(pts, ax, st_j, True, sub_lane[(near, k)][0 if near == uturn.IN_FAR else -1])
            far_pts = uturn._split(pts, ax, st_j, False, sub_lane[(far, k)][-1 if far == IN_FAR2 else 0])
            fl = ET.SubElement(far_el, 'lane', dict(l.attrib))
            fl.set('id', f'{far}_{k}')
            fl.set('shape', _write(far_pts))
            fl.set('length', f'{_length(far_pts):.6f}')
            l.set('shape', _write(near_pts))
            l.set('length', f'{_length(near_pts):.6f}')
    e_in.set('from', J_NODE)
    e_out.set('to', J_NODE)
    idx = list(root).index(e_out) + 1
    root.insert(idx, far_out)
    root.insert(idx, far_in)
    # 먼 쪽 교차점의 이름 바꾸기 (유턴 교차점 연결은 그대로)
    for c in root.findall('connection'):
        if c.get('to') == uturn.IN_FAR:
            c.set('to', IN_FAR2)
        if c.get('from') == uturn.OUT_FAR:
            c.set('from', OUT_FAR2)
    for j in root.findall('junction'):
        jid = j.get('id')
        if jid == uturn.FAR_OUT_NODE or jid.startswith(f':{uturn.FAR_OUT_NODE}_'):
            j.set('incLanes', ' '.join(x.replace(f'{uturn.OUT_FAR}_', f'{OUT_FAR2}_')
                                       for x in j.get('incLanes', '').split()))

    # 3) 새 교차점/엣지/연결
    new_nodes = (ARM_END, J_NODE, N1, N2)
    prefixes = tuple(f':{n}_' for n in new_nodes)
    for j in root.findall('junction'):
        if j.get('id') == ARM_END:
            root.remove(j)

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in (C_DOWN2, C_DOWN1, C_UP1, C_UP2) or e.get('id', '').startswith(prefixes):
            root.insert(pos, e)
            pos += 1
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in new_nodes or j.get('id', '').startswith(prefixes):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        if c.get('via', '').startswith(prefixes) or c.get('from', '').startswith(prefixes):
            root.insert(pos, c)
            pos += 1
    _merge_yield_only_to_target_lane(root)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return cv


def _merge_yield_only_to_target_lane(root):
    """J 우회전 합류(위 도로 → 서행 0번 차로)는 실제로 겹치는 서행 0번 차로 직진에만 양보한다.

    netconvert 는 부도로 연결이 주도로의 모든 직진(5개 차로)에 양보하게 만들 수 있다. 하지만 우회전 경로는 0번 차로하고만 겹치고(1~4번과는 3~13 m 떨어짐),
    5개 차로가 동시에 비는 틈은 거의 없어서 곡선 끝에서 차가 계속 서 있게 된다.
    양보 대상만 0번 차로 직진으로 줄인다.
    """
    links = [c for c in root.findall('connection') if c.get('via', '').startswith(f':{J_NODE}_')]
    j = next(x for x in root.findall('junction') if x.get('id') == J_NODE)
    ints = j.get('intLanes').split()
    idx = {c.get('via'): ints.index(c.get('via')) for c in links}
    merge = next(idx[c.get('via')] for c in links if c.get('from') == C_DOWN1)
    target = next(idx[c.get('via')] for c in links
                  if c.get('from') == IN_FAR2 and c.get('to') == uturn.IN_FAR and c.get('fromLane') == '0')
    n = len(ints)

    def bit(s, k, v):                    # 오른쪽 끝이 링크 0
        s = list(s)
        s[n - 1 - k] = v
        return ''.join(s)

    for r in j.findall('request'):
        k = int(r.get('index'))
        if k == merge:
            only = bit('0' * n, target, '1')
            r.set('response', only)
            r.set('foes', only)
        elif k != target:
            r.set('foes', bit(r.get('foes'), merge, '0'))
            r.set('response', bit(r.get('response'), merge, '0'))


def apply_deco(deco_path, net_path, cv):
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    from env.roundabout_markings import dash_style
    tree = ET.parse(deco_path)
    root = tree.getroot()
    net = ET.parse(net_path).getroot()
    jshape = next(j for j in net.findall('junction') if j.get('id') == J_NODE).get('shape')
    area = Polygon(_read(jshape)).buffer(0.3)

    # 1) 긴 도로 표시는 교차로 구간에서도 그대로 둔다 (중앙선 막힘, 흰 점선 이어짐)
    clipped = 0

    # 2) 연결 도로 표시
    lanes = {(e.get('id'), int(l.get('index'))): _read(l.get('shape'))
             for e in net.findall('edge') for l in e.findall('lane')}
    s_end = None
    # J 경계까지 (1차로 엣지 끝)
    end_pt = lanes[(C_DOWN1, 0)][-1]
    s_end = min(cv.stations(cv.L - 60, cv.L, 0.1), key=lambda s: math.dist(cv.P(s, -(MEDIAN_HALF+LANE_W/2)), end_pt))
    dash, gap, _ = dash_style(20)
    period = dash + gap
    sN = cv.L - REDUCE_AT
    polys = []

    def poly(pid, color, pts, fill=False, layer='6', width='0.15'):
        a = dict(id=pid, color=color, fill='1' if fill else '0', layer=layer, shape=_write(pts))
        if not fill:
            a['lineWidth'] = width
        polys.append(a)

    ss = cv.stations(0, s_end, 1.0)
    for side, name in ((-1, 'down'), (1, 'up')):
        # 하부 포장(차로 + 줄어드는 테이퍼)
        poly(f'sconn_under_{name}', ASPHALT,
             [cv.P(s, side*MEDIAN_HALF) for s in ss] + [cv.P(s, side*cv.outer(s)) for s in reversed(ss)],
             fill=True, layer='-1')
        # 차로 사이 흰 점선 (2차로 구간)
        sep = [cv.P(s, side*(MEDIAN_HALF + LANE_W)) for s in cv.stations(0, sN, 0.25)]
        cum = [0.0]
        for p, q in zip(sep, sep[1:]):
            cum.append(cum[-1] + math.dist(p, q))
        t, j = gap, 0
        while t + 0.5 < cum[-1]:
            piece = [p for p, c in zip(sep, cum) if t <= c <= min(t + dash, cum[-1])]
            if len(piece) >= 2:
                poly(f'sconn_dash_{name}_{j}', WHITE, piece)
                j += 1
            t += period
    # 중앙 포장 + 황색 복선
    poly('sconn_median', ASPHALT, [cv.P(s, -MEDIAN_HALF) for s in ss] + [cv.P(s, MEDIAN_HALF) for s in reversed(ss)],
         fill=True, layer='-1')
    for k, l in enumerate((-0.15, 0.15)):
        poly(f'sconn_center_{k}', YELLOW, [cv.P(s, l) for s in ss])
    # 정지선 + 우회전 화살표 (위 도로 → 긴 도로 서행)
    s_y = s_end - 1.0
    poly('sconn_stop', WHITE, [cv.P(s_y, -MEDIAN_HALF), cv.P(s_y, -(MEDIAN_HALF + LANE_W))], width='0.45')
    from env.roundabout_builder import _read_lane_shapes, _arrow
    marks = []
    _arrow(marks, 'sconn_arrow_right', _read_lane_shapes(net_path), C_DOWN1, 0, 'right', 5.0)
    for m in marks:
        polys.append(dict(ET.fromstring(m).attrib))

    insert_at = next((i for i, el in enumerate(list(root)) if el.get('id', '').startswith('lane_dash_')), len(root))
    for a in reversed(polys):
        root.insert(insert_at, ET.Element('poly', a))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'conn_length': round(cv.L, 1), 'conn_min_radius': round(cv.min_radius(), 1),
            'j_clipped': clipped, 'conn_marks': len(polys)}
