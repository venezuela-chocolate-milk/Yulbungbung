"""T자 교차로(SW_final_forward)와 동남쪽 끝 교차로 사이 긴 도로 중간에 유턴을 만든다.

  - 서행(SW_end_T_left_in, 동남쪽 → T자, 5차로)의 중앙선 쪽 1차로(4번)에서만
    동행(SW_end_T_left_out, T자 → 동남쪽, 4차로)의 1차로(3번)로 유턴.
  - 유턴 지점에서 두 엣지를 나눈다. T자 쪽 절반은 원래 이름을 유지하고,
    먼 쪽 절반은 *_far 로 새로 만든다 (먼 쪽 교차점의 연결 이름도 함께 바꾼다).
  - 유턴 구역(약 20m)은 노란 중앙선 대신 흰 점선, 서행 1차로에 유턴 화살표.
roundabout_south_extend 가 마지막 단계로 호출한다.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

LANE_W = 3.2
IN, OUT = 'SW_end_T_left_in', 'SW_end_T_left_out'
IN_FAR, OUT_FAR = 'SW_end_T_left_in_far', 'SW_end_T_left_out_far'
FAR_IN_NODE, FAR_OUT_NODE = 'SW_end_T_left', 'SW_long_right_start'
U_NODE = 'SW_long_uturn'
U_ST = 260.0            # T자 교차로 쪽 시작점에서 유턴 지점까지 (사용자 표시 위치)
U_ZONE = (3.0, 17.0)    # 유턴 지점 기준 T자 쪽 3m ~ 먼 쪽 17m
YELLOW, WHITE = '235,190,40', '255,255,255'


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class Axis:
    def __init__(self, out_lane0):
        a, b = out_lane0[0], out_lane0[-1]
        d = math.dist(a, b)
        self.a = a
        self.u = ((b[0]-a[0])/d, (b[1]-a[1])/d)       # T자 → 먼 쪽
        self.n = (-self.u[1], self.u[0])              # 서행 쪽(+)

    def st(self, p):
        return (p[0]-self.a[0])*self.u[0] + (p[1]-self.a[1])*self.u[1]

    def lat(self, p):
        return (p[0]-self.a[0])*self.n[0] + (p[1]-self.a[1])*self.n[1]

    def P(self, s, l):
        return (self.a[0]+self.u[0]*s+self.n[0]*l, self.a[1]+self.u[1]*s+self.n[1]*l)


def _split(pts, ax, cut, keep_low, new_end):
    """station 기준으로 polyline 을 자르고, 자른 끝을 new_end 로 둔다."""
    # 방향은 원래 polyline 전체로 판단한다 (남는 점이 하나뿐이면 kept 로는 판단할 수 없다)
    ascending = ax.st(pts[0]) <= ax.st(pts[-1])
    if keep_low:
        kept = [p for p in pts if ax.st(p) < cut]
        return kept + [new_end] if ascending else [new_end] + kept
    kept = [p for p in pts if ax.st(p) > cut]
    return [new_end] + kept if ascending else kept + [new_end]


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    lane = lambda eid, k: _read(next(l for l in edges[eid].findall('lane') if l.get('index') == str(k)).get('shape'))
    n_in = len(edges[IN].findall('lane'))
    n_out = len(edges[OUT].findall('lane'))
    ax = Axis(lane(OUT, 0))
    in_lat = ax.lat(lane(IN, n_in//2)[-1])            # 서행 엣지 중심
    out_lat = (ax.lat(lane(OUT, 0)[0]) + ax.lat(lane(OUT, n_out-1)[0]))/2
    median = (ax.lat(lane(IN, n_in-1)[-1]) + ax.lat(lane(OUT, n_out-1)[0]))/2
    in_s0, in_s1 = ax.st(lane(IN, 0)[0]), ax.st(lane(IN, 0)[-1])      # 먼 쪽 → T자
    out_s0, out_s1 = ax.st(lane(OUT, 0)[0]), ax.st(lane(OUT, 0)[-1])  # T자 → 먼 쪽

    with tempfile.TemporaryDirectory() as work:
        nodes = [('Fin', ax.P(in_s0, in_lat), 'dead_end'), ('Tin', ax.P(in_s1, in_lat), 'dead_end'),
                 ('Tout', ax.P(out_s0, out_lat), 'dead_end'), ('Fout', ax.P(out_s1, out_lat), 'dead_end'),
                 (U_NODE, ax.P(U_ST, median), 'priority')]
        spec = [(IN_FAR, 'Fin', U_NODE, n_in, [ax.P(in_s0, in_lat), ax.P(U_ST, in_lat)]),
                (IN, U_NODE, 'Tin', n_in, [ax.P(U_ST, in_lat), ax.P(in_s1, in_lat)]),
                (OUT, 'Tout', U_NODE, n_out, [ax.P(out_s0, out_lat), ax.P(U_ST, out_lat)]),
                (OUT_FAR, U_NODE, 'Fout', n_out, [ax.P(U_ST, out_lat), ax.P(out_s1, out_lat)])]
        speed = edges[IN].findall('lane')[0].get('speed')
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            for nid, (x, y), t in nodes:
                fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            for eid, fr, to, n, pts in spec:
                fh.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{speed}" '
                         f'priority="2" width="{LANE_W}" spreadType="center" shape="{_write(pts)}"/>\n')
            fh.write('</edges>\n')
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            for k in range(n_in):
                fh.write(f'  <connection from="{IN_FAR}" to="{IN}" fromLane="{k}" toLane="{k}"/>\n')
            for k in range(n_out):
                fh.write(f'  <connection from="{OUT}" to="{OUT_FAR}" fromLane="{k}" toLane="{k}"/>\n')
            fh.write(f'  <connection from="{IN_FAR}" to="{OUT_FAR}" fromLane="{n_in-1}" toLane="{n_out-1}"/>\n')
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

    # 1) 엣지 나누기: T자 쪽 절반은 원래 엣지, 먼 쪽 절반은 새 엣지
    e_in, e_out = edges[IN], edges[OUT]
    far_in = ET.Element('edge', dict(e_in.attrib))
    far_in.set('id', IN_FAR)
    far_in.set('to', U_NODE)
    far_out = ET.Element('edge', dict(e_out.attrib))
    far_out.set('id', OUT_FAR)
    far_out.set('from', U_NODE)
    for e, near, far, far_el in ((e_in, IN, IN_FAR, far_in), (e_out, OUT, OUT_FAR, far_out)):
        if e.get('shape'):
            sh = _read(e.get('shape'))
            far_el.set('shape', _write(_split(sh, ax, U_ST, False, ax.P(U_ST, ax.lat(sh[0])))))
            e.set('shape', _write(_split(sh, ax, U_ST, True, ax.P(U_ST, ax.lat(sh[0])))))
        for l in list(e.findall('lane')):
            k = int(l.get('index'))
            pts = _read(l.get('shape'))
            near_pts = _split(pts, ax, U_ST, True, sub_lane[(near, k)][0 if near == IN else -1])
            far_pts = _split(pts, ax, U_ST, False, sub_lane[(far, k)][-1 if far == IN_FAR else 0])
            fl = ET.SubElement(far_el, 'lane', dict(l.attrib))
            fl.set('id', f'{far}_{k}')
            fl.set('shape', _write(far_pts))
            fl.set('length', f'{_length(far_pts):.6f}')
            l.set('shape', _write(near_pts))
            l.set('length', f'{_length(near_pts):.6f}')
    e_in.set('from', U_NODE)
    e_out.set('to', U_NODE)
    idx = list(root).index(e_out) + 1
    root.insert(idx, far_out)
    root.insert(idx, far_in)

    # 2) 먼 쪽 교차점에서 쓰던 이름을 새 엣지로
    for c in root.findall('connection'):
        if c.get('to') == IN:
            c.set('to', IN_FAR)
        if c.get('from') == OUT:
            c.set('from', OUT_FAR)
    for j in root.findall('junction'):
        jid = j.get('id')
        if jid == FAR_OUT_NODE or jid.startswith(f':{FAR_OUT_NODE}_'):
            j.set('incLanes', ' '.join(x.replace(f'{OUT}_', f'{OUT_FAR}_') for x in j.get('incLanes', '').split()))

    # 3) 유턴 교차점, 내부 엣지, 연결 추가
    pref = f':{U_NODE}_'

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id', '').startswith(pref):
            root.insert(pos, e)
            pos += 1
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') == U_NODE or j.get('id', '').startswith(pref):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        if c.get('via', '').startswith(pref) or c.get('from', '').startswith(pref):
            root.insert(pos, c)
            pos += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return ax, median


def apply_deco(deco_path, net_path, ax, median):
    from shapely.geometry import LineString, box
    from shapely import affinity
    from env.roundabout_markings import dash_style
    from env.roundabout_builder import _read_lane_shapes, _arrow
    tree = ET.parse(deco_path)
    root = tree.getroot()
    z0, z1 = U_ST - U_ZONE[0], U_ST + U_ZONE[1]
    # 유턴 구역의 노란 중앙선을 끊는다 (중앙선 = 중앙 포장 가운데 노란 선)
    gap_zone = LineString([ax.P(z0, median), ax.P(z1, median)]).buffer(0.6, cap_style=2)
    cut = 0
    for p in list(root.findall('poly')):
        if p.get('color') != YELLOW or p.get('fill') != '0':
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2:
            continue
        g = LineString(pts)
        if not g.intersects(gap_zone):
            continue
        rest = g.difference(gap_zone)
        pieces = [x for x in getattr(rest, 'geoms', [rest]) if x.geom_type == 'LineString' and x.length > 0.3]
        i = list(root).index(p)
        root.remove(p)
        for k, x in enumerate(pieces):
            el = ET.Element('poly', dict(p.attrib))
            el.set('id', p.get('id') if k == 0 else f'{p.get("id")}_u{k}')
            el.set('shape', _write(list(x.coords)))
            root.insert(i+k, el)
        cut += 1
    # 유턴 구역선(흰 점선)
    dash, gap, _ = dash_style(20)
    t, j = z0 + gap/2, 0
    while t + dash <= z1:
        ET.SubElement(root, 'poly', dict(id=f'long_uturn_zone_{j}', color=WHITE, fill='0', lineWidth='0.2',
                                          layer='6', shape=_write([ax.P(t, median), ax.P(t+dash, median)])))
        t += dash + gap
        j += 1
    # 서행 1차로 유턴 화살표
    shapes = _read_lane_shapes(net_path)
    n_in = max(k for (e, k) in shapes if e == IN_FAR) + 1
    marks = []
    _arrow(marks, 'long_uturn_arrow', shapes, IN_FAR, n_in-1, 'uturn', 8.0)
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'centerline_cut': cut, 'zone_dashes': j}
