"""검단사거리역 남동쪽 상가 골목 (완정로 남쪽, 실제 지도 완정로117번길 일대)을 만든다.

  큰길(X_se_out, 남동행) 남쪽 가장자리를 기준으로 한 국소 좌표 (s: 큰길 따라 검단사거리 중심에서 남동쪽으로,
  t: 큰길 남쪽 가장자리에서 남서쪽으로, m) 위에 격자 골목을 놓는다. 실제 지도 축척(1px ≈ 0.47m)으로 잰
  상가 블록 크기에 맞췄다.

                       큰길(완정로) ────J1──────────J2──────────J3──── (X_se_out → _s1 → _s2 → _s3 → 학교 진입로)
                    ／                  │C1          │C2          │C3
   남서쪽 큰길 ── J0 ──── 117번길 ──── N1 ───────── C2N ──────── C3N        t = D1_T  (D1)
   (T_stop_main)                        │V1          │            │
                                        N2 ───────── C2S ──────── C3S        t = D2_T  (D2)

  - 117 골목: 2026-11-11 네이버 지도대로 완정로117번길(T_stop_side)에서 갈라져 검단로와 나란히(블록 깊이 일정),
    모퉁이에서 꺾여 N1 까지. (예전에는 남서쪽 큰길 중간에서 갈라졌다)
  - 골목은 모두 왕복 2차로(차로당 3.2m, 30 km/h). 큰길과는 우회전 진입·진출만 (중앙분리대).
  - 우선순위: 큰길 2 > 117번길·D1·D2 1 > 짧은 골목(C, V) 0. 신호 없음.
  - 표시: 골목 노란 중앙선, 우선순위 낮은 쪽 정지선, 차로별 진행 방향 화살표,
    네 갈래 교차점(C2N)의 좌회전 유도선 (우회전 유도선은 만들지 않는다).
roundabout_south_extend 가 학교 진입로 다음 단계로 호출한다.  단독 실행: python -m env.roundabout_shops
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

LANE_W = 3.2
SPEED = 8.33                        # 30 km/h
CENTER = 'T_end_main_end'           # 검단사거리 교차점
MAIN = 'X_se_out'
SW_MAIN = 'T_stop_main_out'         # 검단사거리 남서쪽 큰길, 검단사거리 방향 (상가 쪽이 오른쪽)
STUB = 40.0
YELLOW, WHITE = '235,190,40', '255,255,255'
CROSS_S = {'c1': 80.0, 'c2': 160.0, 'c3': 240.0}     # 큰길에서 내려오는 골목 위치 (s)
D1_T, D2_T = 25.0, 70.0                               # 큰길과 나란한 골목 위치 (t)
# 2026-11-11 (네이버 지도대로): 117 골목은 남서쪽 큰길이 아니라 완정로117번길(T_stop_side)에서 갈라져
# 검단로(남서쪽 큰길)와 나란히 가다가 모퉁이에서 꺾여 N1 까지 간다. 위쪽 블록 = 폭이 일정한 ㄱ자 띠.
BLOCK_DEPTH = D1_T - LANE_W         # 큰길 가장자리 ~ 골목 가장자리 (동남쪽 블록과 같은 깊이)
ALLEY_FILLET = 12.0                 # 골목 모퉁이 곡선 (m)
SIDE_OUT, SIDE_IN = 'T_stop_side_out', 'T_stop_side_in'      # 117번길 (검단로 → 남쪽, 남쪽 → 검단로)
SIDE_PIECES = {SIDE_OUT: [SIDE_OUT, f'{SIDE_OUT}_b'], SIDE_IN: [f'{SIDE_IN}_a', SIDE_IN]}   # 검단로 쪽 조각이 원래 이름
MAIN_PIECES = ['X_se_out', 'X_se_out_s1', 'X_se_out_s2', 'X_se_out_s3']
SW_PIECES = [SW_MAIN]
SHOP_NODES = ['shop_J0', 'shop_N1', 'shop_C2N', 'shop_C3N', 'shop_N2', 'shop_C2S', 'shop_C3S',
              'shop_J1', 'shop_J2', 'shop_J3']


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _cum(pts):
    c = [0.0]
    for a, b in zip(pts, pts[1:]):
        c.append(c[-1] + math.dist(a, b))
    return c


def _project(pts, q):
    best, bs = None, 0.0
    c = _cum(pts)
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        L2 = dx*dx + dy*dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0]-a[0])*dx + (q[1]-a[1])*dy) / L2))
        d = math.dist((a[0]+dx*t, a[1]+dy*t), q)
        if best is None or d < best:
            best, bs = d, c[i] + t*math.sqrt(L2)
    return bs


def _at(pts, s):
    c = _cum(pts)
    for i in range(len(pts)-1):
        if c[i+1] >= s:
            t = 0.0 if c[i+1] == c[i] else (s-c[i])/(c[i+1]-c[i])
            a, b = pts[i], pts[i+1]
            return (a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t)
    return pts[-1]


def _slice(pts, s0, s1):
    c = _cum(pts)
    return [_at(pts, s0)] + [p for p, d in zip(pts, c) if s0 < d < s1] + [_at(pts, s1)]


class Frame:
    """큰길 남쪽 가장자리 기준 국소 좌표. P(s, t) → SUMO 좌표."""

    def __init__(self, net):
        lane0 = []                      # 큰길 바깥 차로 (상가 골목 적용 뒤에는 조각들을 이어서)
        es = {e.get('id'): e for e in net.findall('edge')}
        for eid in MAIN_PIECES:
            if eid in es:
                lane0 += _read(next(l for l in es[eid].findall('lane') if l.get('index') == '0').get('shape'))
        J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
        a, b = _at(lane0, 20.0), _at(lane0, 320.0)
        L = math.dist(a, b)
        self.u = ((b[0]-a[0])/L, (b[1]-a[1])/L)
        self.v = (self.u[1], -self.u[0])                      # 진행 방향 오른쪽 = 큰길 남쪽(남서)
        edge = (a[0] + self.v[0]*LANE_W/2, a[1] + self.v[1]*LANE_W/2)   # 바깥 차로 오른쪽 가장자리
        c = J[CENTER]
        s_c = (c[0]-edge[0])*self.u[0] + (c[1]-edge[1])*self.u[1]
        self.o = (edge[0] + self.u[0]*s_c, edge[1] + self.u[1]*s_c)    # 검단사거리 중심의 투영

    def P(self, s, t):
        return (self.o[0] + self.u[0]*s + self.v[0]*t, self.o[1] + self.u[1]*s + self.v[1]*t)

    def st(self, p):
        d = (p[0]-self.o[0], p[1]-self.o[1])
        return d[0]*self.u[0] + d[1]*self.u[1], d[0]*self.v[0] + d[1]*self.v[1]


def plan(net):
    """골목 노드 위치와 골목 목록 (이름, 시작, 끝, 우선순위, 중간 모양점)."""
    F = Frame(net)
    s1, s2, s3 = CROSS_S['c1'], CROSS_S['c2'], CROSS_S['c3']
    nodes = {'shop_N1': F.P(s1, D1_T), 'shop_C2N': F.P(s2, D1_T), 'shop_C3N': F.P(s3, D1_T),
             'shop_N2': F.P(s1, D2_T), 'shop_C2S': F.P(s2, D2_T), 'shop_C3S': F.P(s3, D2_T)}
    streets = [('shop_c1', 'shop_J1', 'shop_N1', 0), ('shop_c2a', 'shop_J2', 'shop_C2N', 0),
               ('shop_c3a', 'shop_J3', 'shop_C3N', 0),
               ('shop_c2b', 'shop_C2N', 'shop_C2S', 0), ('shop_c3b', 'shop_C3N', 'shop_C3S', 0),
               ('shop_d1a', 'shop_N1', 'shop_C2N', 1), ('shop_d1b', 'shop_C2N', 'shop_C3N', 1),
               ('shop_d2a', 'shop_N2', 'shop_C2S', 1), ('shop_d2b', 'shop_C2S', 'shop_C3S', 1),
               ('shop_v1', 'shop_N1', 'shop_N2', 0), ('shop_117', 'shop_J0', 'shop_N1', 1)]
    nodes['shop_J0'], shapes = _alley_117(net, F, nodes['shop_N1'])
    return F, nodes, streets, shapes


def _isect(p, d, q, e):
    """두 직선 p + a*d, q + b*e 의 교점."""
    den = d[0]*e[1] - d[1]*e[0]
    a = ((q[0]-p[0])*e[1] - (q[1]-p[1])*e[0]) / den
    return (p[0] + d[0]*a, p[1] + d[1]*a)


def _alley_117(net, F, n1):
    """117 골목: 117번길 위 J0 → 검단로와 나란히 → 모퉁이 곡선 → N1 (골목 중심선 = 두 방향 차로 사이)."""
    es = {e.get('id'): e for e in net.findall('edge')}
    lane = lambda eid: _read(es[eid].find('lane').get('shape'))
    sw = next(_read(l.get('shape')) for l in es[SW_MAIN].findall('lane') if l.get('index') == '0')
    d1 = (sw[-1][0]-sw[0][0], sw[-1][1]-sw[0][1])
    L = math.hypot(*d1)
    d1 = (d1[0]/L, d1[1]/L)                                   # 검단사거리 방향
    rgt = (d1[1], -d1[0])                                     # 상가 쪽
    off = LANE_W/2 + BLOCK_DEPTH + LANE_W                     # 바깥 차로 중심 → 골목 중심선
    p1 = (sw[0][0] + rgt[0]*off, sw[0][1] + rgt[1]*off)
    # 117번길 중심선 (두 방향 차로 사이)
    so, si = lane(SIDE_OUT), lane(SIDE_IN)[::-1]
    c0 = ((so[0][0]+si[0][0])/2, (so[0][1]+si[0][1])/2)
    c1 = ((so[-1][0]+si[-1][0])/2, (so[-1][1]+si[-1][1])/2)
    ds = (c1[0]-c0[0], c1[1]-c0[1])
    j0 = _isect(p1, d1, c0, ds)
    q = _isect(p1, d1, n1, F.u)                               # 두 방향이 만나는 모퉁이
    r = ALLEY_FILLET
    a = (q[0] - d1[0]*r, q[1] - d1[1]*r)
    b = (q[0] + F.u[0]*r, q[1] + F.u[1]*r)
    curve = [((1-t)**2*a[0] + 2*(1-t)*t*q[0] + t*t*b[0], (1-t)**2*a[1] + 2*(1-t)*t*q[1] + t*t*b[1])
             for t in (i/12 for i in range(13))]
    return j0, {'shop_117': [j0] + curve + [n1]}


def _split_plan(net, F):
    """나눌 기존 엣지: (엣지, [(새 교차점, 위치 m)], 조각 이름들)."""
    es = {e.get('id'): e for e in net.findall('edge')}
    out = []
    shape = _read(es[MAIN].get('shape'))
    cuts = sorted((_project(shape, F.P(CROSS_S[k], -LANE_W)), f'shop_J{i}')
                  for i, k in enumerate(('c1', 'c2', 'c3'), 1))
    out.append((MAIN, [(j, s) for s, j in cuts], MAIN_PIECES))
    return out


def _split_side(net, j0):
    es = {e.get('id'): e for e in net.findall('edge')}
    out = []
    for eid, pieces in SIDE_PIECES.items():
        shape = _read(es[eid].get('shape'))
        out.append((eid, [('shop_J0', _project(shape, j0))], pieces))
    return out


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    if 'X_se_out_s1' in edges:
        raise RuntimeError('상가 골목이 이미 적용됨')
    F, nodes, streets, alley_shapes = plan(root)
    splits = _split_plan(root, F) + _split_side(root, nodes['shop_J0'])
    j0_fixed = nodes['shop_J0']
    for eid, cuts, _ in splits:
        shape = _read(edges[eid].get('shape'))
        for j, s in cuts:
            nodes[j] = _at(shape, s)
    nodes['shop_J0'] = j0_fixed                     # 117번길 두 방향 사이 중심선 위

    with tempfile.TemporaryDirectory() as work:
        nod = [(k, p, 'priority') for k, p in nodes.items()]
        edg = []
        con = []
        for eid, cuts, pieces in splits:
            e = edges[eid]
            shape = _read(e.get('shape'))
            n = len(e.findall('lane'))
            speed = e.find('lane').get('speed')
            u, d = f'U_{eid}', f'D_{eid}'
            nod += [(u, _at(shape, max(0.0, cuts[0][1] - STUB)), 'dead_end'),
                    (d, _at(shape, min(_length(shape), cuts[-1][1] + STUB)), 'dead_end')]
            seq = [u] + [j for j, _ in cuts] + [d]
            bounds = [max(0.0, cuts[0][1] - STUB)] + [s for _, s in cuts] + [min(_length(shape), cuts[-1][1] + STUB)]
            for i, pid in enumerate(pieces):
                edg.append((pid, seq[i], seq[i+1], n, speed, e.get('priority', '2'),
                            _slice(shape, bounds[i], bounds[i+1]), 'center'))
            for a, b in zip(pieces, pieces[1:]):
                con += [(a, b, k, k) for k in range(n)]
            # 큰길 → 골목 우회전 진입 (큰길 연결을 직접 적었으므로 netconvert 가 회전 연결을 따로 만들지 않는다)
            for i, (j, _) in enumerate(cuts):
                street = next(name for name, a, _, _ in streets if a == j)
                con.append((pieces[i], f'{street}_f', 0, 0))
                if j == 'shop_J0':                  # 117 골목 → 117번길 남쪽(SIDE_OUT 뒤 조각) / 검단로 쪽(SIDE_IN 뒤 조각)
                    con.append(('shop_117_r', pieces[-1], 0, 0))
        for name, a, b, prio in streets:
            pts = alley_shapes.get(name, [nodes[a], nodes[b]])
            edg.append((f'{name}_f', a, b, 1, SPEED, prio, pts, 'right'))
            edg.append((f'{name}_r', b, a, 1, SPEED, prio, pts[::-1], 'right'))
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            for nid, (x, y), t in nod:
                fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            for eid, fr, to, n, speed, prio, pts, spread in edg:
                fh.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{speed}" priority="{prio}" '
                         f'width="{LANE_W}" spreadType="{spread}" shape="{_write(pts)}"/>\n')
            fh.write('</edges>\n')
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            for a, b, i, k in con:
                fh.write(f'  <connection from="{a}" to="{b}" fromLane="{i}" toLane="{k}"/>\n')
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
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}
    sub_lane = {(e.get('id'), int(l.get('index'))): _read(l.get('shape'))
                for e in sub.findall('edge') for l in e.findall('lane')}

    # 1) 기존 엣지 나누기: 첫 조각은 원래 차로를 교차점 경계에서 자르고, 가운데 조각은 부분망 그대로,
    #    마지막 조각은 교차점 경계부터 원래 차로 나머지. 원래 이름은 MAIN 은 첫 조각, 남서쪽 큰길은 마지막 조각.
    for eid, cuts, pieces in splits:
        e = edges[eid]
        shape = _read(e.get('shape'))
        lanes = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
        fr, to = e.get('from'), e.get('to')
        bounds = [0.0] + [s for _, s in cuts] + [_length(shape)]
        els = {}
        pos = list(root).index(e)
        root.remove(e)
        for i, pid in enumerate(pieces):
            el = ET.Element('edge', {k: v for k, v in e.attrib.items()})
            el.set('id', pid)
            el.set('from', fr if i == 0 else sub_edges[pid].get('from'))
            el.set('to', to if i == len(pieces) - 1 else sub_edges[pid].get('to'))
            el.set('shape', _write(_slice(shape, bounds[i], bounds[i+1])))
            els[pid] = el
            root.insert(pos + i, el)
        for l in lanes:
            k = int(l.get('index'))
            pts = _read(l.get('shape'))
            a = sub_lane[(pieces[0], k)][-1]
            b = sub_lane[(pieces[-1], k)][0]
            shapes = {pieces[0]: _slice(pts, 0.0, _project(pts, a))[:-1] + [a],
                      pieces[-1]: [b] + _slice(pts, _project(pts, b), _length(pts))[1:]}
            for pid in pieces[1:-1]:
                shapes[pid] = sub_lane[(pid, k)]
            for pid in pieces:
                ln = ET.SubElement(els[pid], 'lane', dict(l.attrib))
                ln.set('id', f'{pid}_{k}')
                ln.set('shape', _write(shapes[pid]))
                ln.set('length', f'{_length(shapes[pid]):.6f}')
        # 양 끝 교차점에서 쓰던 이름 바꾸기
        first, last = pieces[0], pieces[-1]
        for c in root.findall('connection'):
            if last != eid and c.get('from') == eid:
                c.set('from', last)
            if first != eid and c.get('to') == eid:
                c.set('to', first)
        if last != eid:
            for j in root.findall('junction'):
                if j.get('id') == to or j.get('id', '').startswith(f':{to}_'):
                    j.set('incLanes', ' '.join(x.replace(f'{eid}_', f'{last}_', 1) if x.rsplit('_', 1)[0] == eid
                                               else x for x in j.get('incLanes', '').split()))

    # 2) 골목 엣지, 새 교차점, 내부 엣지, 연결
    new_nodes = set(nodes)
    prefs = tuple(f':{x}_' for x in new_nodes)
    street_edges = {f'{s[0]}_{d}' for s in streets for d in ('f', 'r')}
    pieces_all = {p for _, _, ps in splits for p in ps}

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in street_edges or e.get('id', '').startswith(prefs):
            root.insert(pos, e)
            pos += 1
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in new_nodes or j.get('id', '').startswith(prefs):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        f, t = c.get('from'), c.get('to')
        if f.startswith(prefs) or f in street_edges or t in street_edges or (f in pieces_all and t in pieces_all):
            root.insert(pos, c)
            pos += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'shop_streets': len(streets), 'shop_junctions': len(new_nodes)}


# ───────────────────────── 노면 표시 ─────────────────────────

def _turn(a_dir, b_dir):
    cross = a_dir[0]*b_dir[1] - a_dir[1]*b_dir[0]
    dot = a_dir[0]*b_dir[0] + a_dir[1]*b_dir[1]
    ang = math.degrees(math.atan2(cross, dot))
    return 'straight' if abs(ang) < 35 else ('left' if ang > 0 else 'right')


def _dir(pts, at_end):
    a, b = (pts[-2], pts[-1]) if at_end else (pts[0], pts[1])
    L = math.dist(a, b) or 1.0
    return ((b[0]-a[0])/L, (b[1]-a[1])/L)


def _dashes(root, prefix, path, dash, gap, width='0.15'):
    L = _length(path)
    if L < dash:
        return 0
    count = max(2, round((L + gap) / (dash + gap)))
    g = (L - count*dash) / (count - 1)
    for i in range(count):
        a = i*(dash + g)
        ET.SubElement(root, 'poly', dict(id=f'{prefix}_{i}', color=WHITE, fill='0', lineWidth=width,
                                          layer='6', shape=_write(_slice(path, a, a + dash))))
    return count


def movements(net_path):
    """새 교차점의 (교차점, 출발 엣지, 출발 차로, 도착 엣지, 회전)."""
    net = ET.parse(net_path).getroot()
    junctions = set(SHOP_NODES)
    edge_to = {e.get('id'): e.get('to') for e in net.findall('edge') if e.get('function') != 'internal'}
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    out = []
    for c in net.findall('connection'):
        f = c.get('from')
        if f.startswith(':') or edge_to.get(f) not in junctions:
            continue
        a = lane[f'{f}_{c.get("fromLane")}']
        b = lane[f'{c.get("to")}_{c.get("toLane")}']
        out.append((edge_to[f], f, int(c.get('fromLane')), c.get('to'), _turn(_dir(a, True), _dir(b, False))))
    return out


def guide_paths(net_path):
    """네 갈래 교차점(C2N)의 좌회전 유도선 경로: 실제 차량 경로를 차로 폭 절반만큼 왼쪽(안쪽)으로 옮긴 경계,
    출발 차로 끝(정지선 자리)부터 도착 차로 시작까지."""
    from env.roundabout_builder import _movement_path
    from env.roundabout_geometry import _offset
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    out = {}
    for jn, f, k, t, turn in movements(net_path):
        if jn != 'shop_C2N' or turn != 'left':
            continue
        src, dst = lane[f'{f}_{k}'], lane[f'{t}_0']
        center = [src[-2]] + _movement_path(net_path, f, t, k) + [dst[1]]
        boundary = _offset(center, LANE_W / 2)
        a = math.dist(src[-2], src[-1])
        b = _length(boundary) - math.dist(dst[0], dst[1])
        out[f'{f}_to_{t}'] = _slice(boundary, a, b)
    return out


ARROW_KINDS = {frozenset({'straight'}): ['straight'], frozenset({'left'}): ['left'],
               frozenset({'right'}): ['right'], frozenset({'straight', 'left'}): ['straight_left'],
               frozenset({'straight', 'right'}): ['straight_right'], frozenset({'left', 'right'}): ['left', 'right'],
               frozenset({'straight', 'left', 'right'}): ['straight_left', 'right']}


def apply_deco(deco_path, net_path):
    """골목 노란 중앙선, 정지선(우선순위 낮은 쪽), 진행 방향 화살표, 4거리 좌회전 유도선."""
    from env.roundabout_builder import _arrow, _read_lane_shapes, _stop_line
    from env.roundabout_geometry import _offset
    net = ET.parse(net_path).getroot()
    edges = {e.get('id'): e for e in net.findall('edge') if e.get('function') != 'internal'}
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    shapes = _read_lane_shapes(net_path)
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith('shop_'):
            root.remove(p)
    # 117번길에 있던 중앙선·점선은 새 교차점(J0) 안에서 끊는다
    from shapely.geometry import LineString, Polygon
    j0 = next(j for j in net.findall('junction') if j.get('id') == 'shop_J0')
    jpoly = Polygon(_read(j0.get('shape'))).buffer(0.3)
    n_clip = 0
    for p in list(root.findall('poly')):
        if p.get('fill') != '0' or not p.get('shape'):
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2 or not LineString(pts).intersects(jpoly):
            continue
        rest = LineString(pts).difference(jpoly)
        idx = list(root).index(p)
        root.remove(p)
        for k, part in enumerate(x for x in getattr(rest, 'geoms', [rest]) if x.geom_type == 'LineString' and x.length > 0.5):
            el = ET.Element('poly', dict(p.attrib))
            el.set('id', p.get('id') if k == 0 else f"{p.get('id')}_j0c{k}")
            el.set('shape', _write(list(part.coords)))
            root.insert(idx + k, el)
        n_clip += 1
    h = LANE_W / 2
    # 1) 골목 노란 중앙선 (_f 차로 왼쪽 경계)
    n_center = 0
    for eid in edges:
        if eid.startswith('shop_') and eid.endswith('_f'):
            ET.SubElement(root, 'poly', dict(id=f'{eid[:-2]}_center', color=YELLOW, fill='0', lineWidth='0.15',
                                              layer='6', shape=_write(_offset(lane[f'{eid}_0'], h))))
            n_center += 1
    mv = movements(net_path)
    by_j = {}
    for jn, f, k, t, turn in mv:
        by_j.setdefault(jn, {}).setdefault((f, k), set()).add(turn)
    main_like = set(MAIN_PIECES) | set(SW_PIECES)
    marks = []
    n_stop = n_arrow = 0
    for jn, approaches in by_j.items():
        incoming = {f for f, _ in approaches}
        if len(incoming) < 3 and not (incoming & main_like):
            continue                                    # 굽은 길(두 갈래)은 표시 없음
        top = max(int(edges[f].get('priority', '0')) for f in incoming)
        for f in sorted(incoming):
            # 2) 정지선: 우선순위 낮은 쪽
            if int(edges[f].get('priority', '0')) < top:
                _stop_line(marks, f'shop_stop_{f}', shapes, f, dist=1.0)
                n_stop += 1
        for (f, k), turns in sorted(approaches.items()):
            if f in main_like:                          # 큰길 바깥 차로: 직진·우회전 (교차점 25m 앞)
                if turns == {'straight', 'right'} and _length(lane[f'{f}_{k}']) > 32.0:
                    _arrow(marks, f'shop_arrow_{f}_{k}', shapes, f, k, 'straight_right', 25.0)
                    n_arrow += 1
                continue
            if _length(lane[f'{f}_{k}']) < 7.5:
                continue
            for kind in ARROW_KINDS.get(frozenset(turns), []):
                _arrow(marks, f'shop_arrow_{f}_{k}_{kind}', shapes, f, k, kind,
                       min(7.0, _length(lane[f'{f}_{k}']) - 3.5))
                n_arrow += 1
    for m in marks:
        root.append(ET.fromstring(m))
    # 3) 좌회전 유도선 (흰 점선 1m/1m)
    n_guide = 0
    for name, path in guide_paths(net_path).items():
        n_guide += _dashes(root, f'shop_guide_{name}', path, 1.0, 1.0)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'shop_centerlines': n_center, 'shop_stoplines': n_stop, 'shop_arrows': n_arrow,
            'shop_guide_dashes': n_guide, 'shop_j0_lines_clipped': n_clip}


def main():
    import sys
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, base)
    if os.environ.get('SUMO_HOME'):
        sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    net_dir = os.path.join(base, 'env', 'sumo_roundabout')
    net_path = os.path.join(net_dir, 'roundabout.net.xml')
    deco_path = os.path.join(net_dir, 'roundabout.deco.xml')
    info = apply_net(net_path)
    from env import roundabout_south_link as link
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(apply_deco(deco_path, net_path))
    print(info)


if __name__ == '__main__':
    main()
