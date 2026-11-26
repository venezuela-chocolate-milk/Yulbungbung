"""검은 도로(SW_final_forward_out, 남행 4차로)를 T자 교차로 앞 REDUCE m 구간에서 3차로로 줄이고,
T자 교차로 진입 연결을 중앙선 쪽부터 좌·좌·우로 바꾼다.

  - 엣지를 나눈다: 삼각형 교차로 쪽 4차로는 SW_final_forward_out_up, T자 쪽 3차로가 SW_final_forward_out.
    가장 바깥(0번) 차로가 끝나고, 나머지 1~3번 차로가 3차로 구간의 0~2번 차로로 이어진다.
  - T자 교차로는 회전 경로를 손질해 둔 원래 교차로라 다시 만들지 않고 이 진입로 연결만 바꾼다.
      우(0번)  → SW_end_T_right_out 1번 (원래 1번 차로 우회전 경로 그대로)
      좌(1번)  → SW_end_T_left_out 2번 (원래 좌회전 경로를 차로 폭만큼 오른쪽으로 평행이동)
      좌(2번)  → SW_end_T_left_out 3번 (원래 좌회전 경로 그대로)
    새 좌회전(1번)은 같은 진출 차로(2번)로 합류하는 동행 직진(2번 차로)에 양보한다.
  - 노면: 줄어드는 바깥 차로 자리는 REDUCE_TAPER m 동안 부드럽게 좁아지고, 그 차로 경계 점선은 지운다.
    3차로 구간 정지선 뒤에 좌·좌·우 화살표.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

T = 'SW_final_forward'
EDGE = 'SW_final_forward_out'
UP = 'SW_final_forward_out_up'
NODE = 'SW_forward_reduce3'
RIGHT_OUT, LEFT_OUT = 'SW_end_T_right_out', 'SW_end_T_left_out'
REDUCE = 100.0
REDUCE_TAPER = 40.0
LANE_W = 3.2
ARROWS = {2: 'left', 1: 'left', 0: 'right'}


def H(t):
    t = min(1.0, max(0.0, t))
    return t*t*t*(10 - 15*t + 6*t*t)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _bit(bits, i):
    return bits[len(bits) - 1 - i]


def _dir(pts):
    a, b = pts[0], pts[1]
    d = math.dist(a, b)
    return ((b[0]-a[0])/d, (b[1]-a[1])/d)


def _turn_curve(P0, h0, P3, h3, n=40):
    """P0 에서 h0 방향으로 들어와 P3 에서 h3 방향으로 나가는 원호형 3차 베지어."""
    # 두 접선의 교점까지 거리(접선 길이)와 회전각으로 원호 근사 제어점 길이를 정한다
    den = h0[0]*h3[1] - h0[1]*h3[0]
    t0 = ((P3[0]-P0[0])*h3[1] - (P3[1]-P0[1])*h3[0]) / den
    t3 = (h0[0]*(P3[1]-P0[1]) - h0[1]*(P3[0]-P0[0])) / den
    theta = abs(math.atan2(den, h0[0]*h3[0] + h0[1]*h3[1]))
    f = (4/3)*math.tan(theta/4)/math.tan(theta/2)
    C1 = (P0[0] + h0[0]*t0*f, P0[1] + h0[1]*t0*f)
    C2 = (P3[0] - h3[0]*t3*f, P3[1] - h3[1]*t3*f)
    out = []
    for i in range(n+1):
        t = i/n
        a, b, c, d = (1-t)**3, 3*(1-t)**2*t, 3*(1-t)*t*t, t**3
        out.append((a*P0[0]+b*C1[0]+c*C2[0]+d*P3[0], a*P0[1]+b*C1[1]+c*C2[1]+d*P3[1]))
    return out


def _straight_tail(pts, tol_deg=0.05):
    """polyline 끝의 직선 구간 (대각선 도로 윗부분이 휘어 있어도 T자 앞 직선만 쓴다)."""
    h = math.atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])
    i = len(pts) - 2
    while i > 0:
        g = math.atan2(pts[i][1]-pts[i-1][1], pts[i][0]-pts[i-1][0])
        if abs(math.degrees(math.atan2(math.sin(g-h), math.cos(g-h)))) > tol_deg:
            break
        i -= 1
    return pts[i:]


class Axis:
    def __init__(self, lane0):
        lane0 = _straight_tail(lane0)
        a, b = lane0[0], lane0[-1]
        self.a, self.L = a, math.dist(a, b)
        self.u = ((b[0]-a[0])/self.L, (b[1]-a[1])/self.L)
        self.n = (-self.u[1], self.u[0])           # 왼쪽(+) = 중앙선 쪽

    def st(self, p):
        return (p[0]-self.a[0])*self.u[0] + (p[1]-self.a[1])*self.u[1]

    def lat(self, p):
        return (p[0]-self.a[0])*self.n[0] + (p[1]-self.a[1])*self.n[1]

    def P(self, s, l):
        return (self.a[0]+self.u[0]*s+self.n[0]*l, self.a[1]+self.u[1]*s+self.n[1]*l)


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    if UP in edges:
        return {'black_reduce': 'already'}
    e = edges[EDGE]
    lanes = {int(l.get('index')): _read(l.get('shape')) for l in e.findall('lane')}
    ax = Axis(lanes[0])
    sN = ax.L - REDUCE
    lat = {k: ax.lat(v[-1]) for k, v in lanes.items()}
    c4 = (lat[1] + lat[2]) / 2
    c3 = lat[2]
    speed = e.findall('lane')[0].get('speed')

    # 1) 엣지 나누기 (netconvert 부분망으로 접속점 만들기)
    with tempfile.TemporaryDirectory() as work:
        nodes = [('D0', ax.P(0, c4), 'dead_end'), (NODE, ax.P(sN, c4), 'priority'), ('D1', ax.P(ax.L, c3), 'dead_end')]
        spec = [(UP, 'D0', NODE, 4, [ax.P(0, c4), ax.P(sN, c4)]), (EDGE, NODE, 'D1', 3, [ax.P(sN, c3), ax.P(ax.L, c3)])]
        with open(os.path.join(work, 'n.xml'), 'w') as fh:
            fh.write('<nodes>' + ''.join(f'<node id="{a}" x="{p[0]:.6f}" y="{p[1]:.6f}" type="{t}" radius="0"/>'
                                         for a, p, t in nodes) + '</nodes>')
        with open(os.path.join(work, 'e.xml'), 'w') as fh:
            fh.write('<edges>' + ''.join(f'<edge id="{i}" from="{f}" to="{t}" numLanes="{n}" speed="{speed}" priority="2" '
                                         f'width="{LANE_W}" spreadType="center" shape="{_write(sh)}"/>'
                                         for i, f, t, n, sh in spec) + '</edges>')
        with open(os.path.join(work, 'c.xml'), 'w') as fh:
            fh.write('<connections>' + ''.join(f'<connection from="{UP}" to="{EDGE}" fromLane="{k}" toLane="{k-1}"/>'
                                               for k in (1, 2, 3)) + '</connections>')
        from sumolib import checkBinary
        r = subprocess.run([checkBinary('netconvert'), '-n', 'n.xml', '-e', 'e.xml', '-x', 'c.xml',
                            '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                            '--precision', '6', '-o', 'o.xml'], cwd=work, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        sub = ET.parse(os.path.join(work, 'o.xml')).getroot()
    sl = {(x.get('id'), int(l.get('index'))): _read(l.get('shape')) for x in sub.findall('edge') for l in x.findall('lane')}

    # 원래 엣지 → 위쪽 4차로 엣지
    e.set('id', UP)
    e.set('to', NODE)
    for l in e.findall('lane'):
        k = int(l.get('index'))
        # 휜 윗부분은 그대로 두고 직선 구간만 새 끝점까지 자른다 (직선 도로면 [시작, 새 끝])
        pts = [p for p in lanes[k] if ax.st(p) < sN - 1.0] + [sl[(UP, k)][-1]]
        l.set('id', f'{UP}_{k}')
        l.set('shape', _write(pts))
        l.set('length', f'{_length(pts):.6f}')
    down = ET.Element('edge', {'id': EDGE, 'from': NODE, 'to': T, 'priority': e.get('priority', '2'),
                               'spreadType': 'center', 'shape': _write([ax.P(sN, c3), ax.P(ax.L, c3)])})
    lane_attr = dict(e.findall('lane')[0].attrib)
    for k in range(3):
        pts = [sl[(EDGE, k)][0], lanes[k+1][-1]]
        a = dict(lane_attr, id=f'{EDGE}_{k}', index=str(k), shape=_write(pts), length=f'{_length(pts):.6f}')
        ET.SubElement(down, 'lane', a)
    root.insert(list(root).index(e) + 1, down)
    # 삼각형 교차로 쪽 연결은 위쪽 엣지로
    for c in root.findall('connection'):
        if c.get('to') == EDGE:
            c.set('to', UP)

    # 2) T자 교차로: 이 진입로 링크 0~3 을 [우, 좌(새), 좌] 로 교체
    internal = {x.get('id'): x for x in root.findall('edge') if x.get('function') == 'internal'}
    e0, e3 = internal[f':{T}_0'], internal[f':{T}_3']
    l01 = next(l for l in e0.findall('lane') if l.get('index') == '1')
    old_left = next(l for l in e3.findall('lane') if l.get('index') == '0')
    # 우회전 경로: 바깥 차로 끝 → 서쪽 도로 가장 바깥 차로 시작, 양 끝 접선이 차로 방향인 원호형 3차 곡선
    ro0 = _read(next(l for l in edges[RIGHT_OUT].findall('lane') if l.get('index') == '0').get('shape'))
    rpts = _turn_curve(lanes[1][-1], ax.u, ro0[0], _dir(ro0))
    for l in list(e0.findall('lane')):
        e0.remove(l)
    ET.SubElement(e0, 'lane', dict(l01.attrib, id=f':{T}_0_0', index='0', shape=_write(rpts),
                                   length=f'{_length(rpts):.6f}'))
    # 새 좌회전 경로: 원래 좌회전 경로를 오른쪽으로 차로 폭만큼 평행이동, 양 끝은 실제 차로 끝/시작에 맞춤
    from shapely.geometry import LineString
    old_pts = _read(old_left.get('shape'))
    off = LineString(old_pts).offset_curve(-LANE_W)
    g = off
    new_pts = [g.interpolate(g.length*i/40).coords[0] for i in range(41)]
    start = lanes[2][-1]
    end = _read(next(l for l in edges[LEFT_OUT].findall('lane') if l.get('index') == '2').get('shape'))[0]
    if math.dist(new_pts[0], start) > math.dist(new_pts[-1], start):
        new_pts.reverse()
    new_pts[0], new_pts[-1] = start, end
    for l in list(e3.findall('lane')):
        e3.remove(l)
    ET.SubElement(e3, 'lane', dict(old_left.attrib, id=f':{T}_3_0', index='0', shape=_write(new_pts),
                                   length=f'{_length(new_pts):.6f}'))
    ET.SubElement(e3, 'lane', dict(old_left.attrib, id=f':{T}_3_1', index='1'))

    j = next(x for x in root.findall('junction') if x.get('id') == T)
    old_ints = j.get('intLanes').split()
    reqs = sorted(j.findall('request'), key=lambda r: int(r.get('index')))
    oldF = [r.get('foes') for r in reqs]
    oldR = [r.get('response') for r in reqs]
    n_old = len(reqs)
    # 새 링크 목록: 원래 번호 (None = 새 좌회전, 원형은 3번)
    order = [1, None, 3] + list(range(4, n_old))
    lane2_straight = next(i for i in range(n_old) if old_ints[i] == f':{T}_10_2')
    lane3_straight = next(i for i in range(n_old) if old_ints[i] == f':{T}_10_3')

    def rel(a, b):
        """(foe, response) : a 가 b 와 충돌하는지, a 가 b 에 양보하는지."""
        if a is None and b is None:
            return '0', '0'
        if a is None:
            if b == lane2_straight:
                return '1', '1'
            if b in (3, lane3_straight):
                return '0', '0'
            return _bit(oldF[3], b), _bit(oldR[3], b)
        if b is None:
            if a == lane2_straight:
                return '1', '0'
            if a in (3, lane3_straight):
                return '0', '0'
            return _bit(oldF[a], 3), _bit(oldR[a], 3)
        return _bit(oldF[a], b), _bit(oldR[a], b)

    n = len(order)
    for r in reqs:
        j.remove(r)
    pos = list(j).index(j.findall('param')[0]) if j.findall('param') else len(j)
    for i, a in enumerate(order):
        foes = ''.join(rel(a, b)[0] for b in reversed(order))
        resp = ''.join(rel(a, b)[1] for b in reversed(order))
        j.insert(pos + i, ET.Element('request', {'index': str(i), 'response': resp, 'foes': foes, 'cont': '0'}))
    j.set('intLanes', ' '.join([f':{T}_0_0', f':{T}_3_0', f':{T}_3_1'] + old_ints[4:]))
    inc = [x for x in j.get('incLanes').split() if not x.startswith(f'{EDGE}_')]
    j.set('incLanes', ' '.join([f'{EDGE}_{k}' for k in range(3)] + inc))

    # 연결 교체
    olds = [c for c in root.findall('connection')
            if (c.get('from') == EDGE and c.get('via', '').startswith(f':{T}_'))
            or c.get('from') in (f':{T}_0', f':{T}_3')]
    first = list(root).index(olds[0])
    first_int = list(root).index(next(c for c in olds if c.get('from', '').startswith(':')))
    for c in olds:
        root.remove(c)
    mk = lambda a: ET.Element('connection', a)
    news = [mk({'from': EDGE, 'to': RIGHT_OUT, 'fromLane': '0', 'toLane': '0', 'contPos': '0.000000',
                'via': f':{T}_0_0', 'dir': 'r', 'state': 'm'}),
            mk({'from': EDGE, 'to': LEFT_OUT, 'fromLane': '1', 'toLane': '2', 'contPos': '0.000000',
                'via': f':{T}_3_0', 'dir': 'l', 'state': 'm'}),
            mk({'from': EDGE, 'to': LEFT_OUT, 'fromLane': '2', 'toLane': '3', 'contPos': '0.000000',
                'via': f':{T}_3_1', 'dir': 'l', 'state': 'm'})]
    for k, c in enumerate(news):
        root.insert(first + k, c)
    first_int = [i for i, c in enumerate(list(root)) if c.tag == 'connection' and c.get('from', '').startswith(f':{T}_')][0]
    ints = [mk({'from': f':{T}_0', 'to': RIGHT_OUT, 'fromLane': '0', 'toLane': '0', 'dir': 'r', 'state': 'M'}),
            mk({'from': f':{T}_3', 'to': LEFT_OUT, 'fromLane': '0', 'toLane': '2', 'dir': 'l', 'state': 'M'}),
            mk({'from': f':{T}_3', 'to': LEFT_OUT, 'fromLane': '1', 'toLane': '3', 'dir': 'l', 'state': 'M'})]
    for k, c in enumerate(ints):
        root.insert(first_int + k, c)

    # 3) 차로 줄이는 접속점(부분망)의 교차점/내부 엣지/연결
    pref = f':{NODE}_'
    for x in sub.findall('edge'):
        if x.get('id', '').startswith(pref):
            root.insert([i for i, c in enumerate(list(root)) if c.tag == 'edge'][-1] + 1, x)
    for x in sub.findall('junction'):
        if x.get('id') == NODE or x.get('id', '').startswith(pref):
            root.insert([i for i, c in enumerate(list(root)) if c.tag == 'junction'][-1] + 1, x)
    for x in sub.findall('connection'):
        if x.get('via', '').startswith(pref) or x.get('from', '').startswith(pref):
            root.insert([i for i, c in enumerate(list(root)) if c.tag == 'connection'][-1] + 1, x)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'black_reduce': 'done'}, ax, sN, lat


def apply_deco(deco_path, net_path, ax, sN, lat):
    from shapely.geometry import LineString, Polygon
    from env.roundabout_builder import _read_lane_shapes, _arrow
    tree = ET.parse(deco_path)
    root = tree.getroot()
    b01 = (lat[0] + lat[1]) / 2                # 줄어드는 차로와 1번 차로 사이 경계
    outer = lat[0] - LANE_W/2                 # 바깥 가장자리
    # 1) 줄어드는 차로 경계 점선(접속점 너머) 지우기
    removed = 0
    for p in list(root.findall('poly')):
        if p.get('color') != '255,255,255' or p.get('fill') != '0':
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2:
            continue
        m = ((pts[0][0]+pts[-1][0])/2, (pts[0][1]+pts[-1][1])/2)
        if abs(ax.lat(m) - b01) < 0.4 and ax.st(m) > sN - 0.5 and ax.st(m) < ax.L + 1:
            root.remove(p)
            removed += 1
    # 2) 바깥 차로 하부 포장을 접속점에서 자르고, 테이퍼로 좁힌다
    beyond = Polygon([ax.P(sN, outer - 3), ax.P(ax.L + 40, outer - 3), ax.P(ax.L + 40, b01 - 0.05), ax.P(sN, b01 - 0.05)])
    cut = 0
    for p in list(root.findall('poly')):
        if p.get('fill') != '1' or p.get('color') != '84,84,90':
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 3:
            continue
        g = Polygon(pts).buffer(0)
        if g.area < 1 or not g.intersects(beyond):
            continue
        c = g.centroid
        if abs(ax.lat((c.x, c.y)) - lat[0]) > 1.0:      # 바깥 차로 하부 포장만
            continue
        rest = g.difference(beyond)
        if rest.is_empty:
            root.remove(p)
        else:
            big = max(getattr(rest, 'geoms', [rest]), key=lambda x: x.area)
            p.set('shape', _write(list(big.exterior.coords)))
        cut += 1
    ss = [sN + REDUCE_TAPER*i/60 for i in range(61)]
    curve = [ax.P(s, outer + LANE_W*H((s - sN)/REDUCE_TAPER)) for s in ss]
    ET.SubElement(root, 'poly', {'id': 'black_reduce_taper', 'color': '84,84,90', 'fill': '1', 'layer': '4.9',
                                 'shape': _write(curve + [ax.P(sN + REDUCE_TAPER, b01 + 0.05), ax.P(sN - 0.6, b01 + 0.05),
                                                          ax.P(sN - 0.6, outer)])})
    # 우회전 모서리: 우회전 경로에서 차로 폭 절반 바깥 선을 새 도로 가장자리로 삼고,
    # 4차로 기준으로 크게 둥글던 교차로 모서리 중 그 바깥 부분은 잔디색으로 덮는다
    net = ET.parse(net_path).getroot()
    rpath = _read(next(l for e in net.findall('edge') if e.get('id') == f':{T}_0'
                       for l in e.findall('lane')).get('shape'))
    curb = LineString(rpath).offset_curve(-LANE_W/2)           # 진행 방향 오른쪽 = 바깥
    cp = list(curb.coords)
    if math.dist(cp[0], rpath[0]) > math.dist(cp[-1], rpath[0]):
        cp.reverse()
    h0, h1 = _dir(rpath), _dir(rpath[::-1])
    # 곡선 양 끝을 진행 방향으로 조금 연장하고, 바깥쪽으로 넓게 닫아 '바깥 영역'을 만든다
    a = (cp[0][0] - h0[0]*6, cp[0][1] - h0[1]*6)
    b = (cp[-1][0] - h1[0]*6, cp[-1][1] - h1[1]*6)
    side = lambda p, h, k: (p[0] + h[1]*k, p[1] - h[0]*k)     # 진행 방향 오른쪽으로 k
    outside = Polygon([a] + cp + [b, side(b, (-h1[0], -h1[1]), 30), side(a, h0, 30)]).buffer(0)
    jshape = Polygon(_read(next(j for j in net.findall('junction') if j.get('id') == T).get('shape'))).buffer(0)
    mask = jshape.intersection(outside)
    for k, g in enumerate(getattr(mask, 'geoms', [mask])):
        if g.geom_type == 'Polygon' and g.area > 0.2:
            ET.SubElement(root, 'poly', {'id': f'black_corner_grass_{k}', 'color': '112,140,96', 'fill': '1',
                                         'layer': '5.1', 'shape': _write(list(g.exterior.coords))})
    # 두 좌회전 경로 사이 점선 유도선 (이중 좌회전)
    from env.roundabout_markings import dash_style
    dash, gap, _ = dash_style(20)
    left = {l.get('index'): _read(l.get('shape')) for e in net.findall('edge') if e.get('id') == f':{T}_3'
            for l in e.findall('lane')}
    ga, gb = LineString(left['0']), LineString(left['1'])
    between = LineString([((pa.x+pb.x)/2, (pa.y+pb.y)/2) for pa, pb in
                          ((ga.interpolate(ga.length*i/80), gb.interpolate(gb.length*i/80)) for i in range(81))])
    t, k = 0.0, 0
    while t + 0.5 < between.length:
        seg = [between.interpolate(x).coords[0] for x in (t, min(t + dash, between.length))]
        ET.SubElement(root, 'poly', {'id': f'black_left_guide_between_{k}', 'color': '255,255,255', 'fill': '0',
                                     'lineWidth': '0.15', 'layer': '6', 'shape': _write(seg)})
        t += dash + gap
        k += 1
    # 3) 정지선 뒤 화살표
    shapes = _read_lane_shapes(net_path)
    marks = []
    for k, kind in ARROWS.items():
        _arrow(marks, f'black_reduce_arrow_{k}', shapes, EDGE, k, kind, 5.0)
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'black_dashes_removed': removed, 'black_under_cut': cut}
