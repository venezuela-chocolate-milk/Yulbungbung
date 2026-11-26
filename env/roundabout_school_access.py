"""위쪽 긴 도로(X_se_out, 남동행)에서 완정초·마전중 사이로 들어가는 학교 진입로를 만든다.

  - 네이버 지도에서 두 학교 사이 길이 큰길과 만나는 지점(SUMO 좌표 약 1220, 32)에서
    X_se_out 을 둘로 나눈다. 앞쪽은 원래 이름(X_se_out), 뒤쪽은 X_se_out_b.
  - 진입로: 들어가는 쪽(school_in) 2차로 = 0번 우회전(완정초) / 1번 좌회전(마전중),
    나오는 쪽(school_out) 1차로. 차로당 3.2m, 30 km/h, 길이 SPUR_LEN m.
    끝 교차점(school_access_mid)에서 양쪽 학교로 1차로 왕복 길이 하나씩 갈라진다.
      서쪽 → 완정초 (school_wanjeong_in/out), 동쪽 → 마전중 (school_majeon_in/out)
  - 학교 길은 정문을 지나 학교 부지 안까지 들어가 끝난다 (회차 없음). 학교로 가는 차는 길 끝(학교 안)에서
    도착해 사라지고, 학교에서 나오는 차는 school_*_out 에서 출발한다.
  - 우회전 진입(X_se_out 바깥 차로 → school_in), 우회전 진출(school_out → X_se_out_b 바깥 차로)만 둔다.
    반대 방향(X_se_in)은 중앙분리대가 있어 좌회전으로 드나들지 않는다.
  - 표시: 노란 중앙선, 진입로 차로 사이 흰 점선, 정지선(갈림 교차점 세 방향 + 큰길 진출),
    회전 화살표, 회전 경로 흰 점선 유도선(실제 차량 경로에서 차로 폭 절반만큼 왼쪽 경계).
학교 그림은 env.roundabout_school_buildings 가 학교 길 끝에 맞춰 그린다.
roundabout_south_extend 가 마지막 단계로 호출한다.  단독 실행: python -m env.roundabout_school_access
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

LANE_W = 3.2
MAIN, MAIN_B = 'X_se_out', 'X_se_out_b'
FAR_NODE = 'X_se_widen'
J_NODE, MID_NODE = 'school_access_J', 'school_access_mid'
S_IN, S_OUT = 'school_in', 'school_out'
MOUTH = (1220.0, 32.0)              # 지도에서 두 학교 사이 길이 큰길과 만나는 지점 (근사)
SPUR_DIR = (-54.5, -103.2)          # 지도에서 두 학교 사이 길 방향 (큰길 → 학교 쪽)
SPUR_LEN = 45.0                     # 큰길 교차점 ~ 양쪽 학교 갈림 교차점 (m)
GATE_LEN = 22.0                     # 갈림 교차점 ~ 학교 길 끝 (학교 부지 안, m)
GATE_INSET = 8.0                    # 학교 길이 정문을 지나 부지 안으로 들어가는 길이 (m)
# (이름, 학교 길 끝 노드, 방향) — 방향 +1 = 진입로 진행 방향 오른쪽(서쪽), -1 = 왼쪽(동쪽)
GATES = [('school_wanjeong', 'school_wanjeong_gate', +1),
         ('school_majeon', 'school_majeon_gate', -1)]
# 2026-11-14 사용자 요청: 완정초에서 나와 갈림 교차점에서 마전중으로 바로 직진 (큰길로 나가는 회전과 함께)
THROUGH = [('school_wanjeong_out', 'school_majeon_in')]
SPUR_SPEED = 8.33                   # 30 km/h (학교 앞)
STUB = 40.0                         # netconvert 부분망에 넣는 큰길 앞뒤 길이 (m)
YELLOW, WHITE = '235,190,40', '255,255,255'


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
    """polyline 위에서 q 에 가장 가까운 점의 누적 거리."""
    best, bs = None, 0.0
    c = _cum(pts)
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        L2 = dx*dx + dy*dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0]-a[0])*dx + (q[1]-a[1])*dy) / L2))
        p = (a[0]+dx*t, a[1]+dy*t)
        d = math.dist(p, q)
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
    out = [_at(pts, s0)] + [p for p, d in zip(pts, c) if s0 < d < s1] + [_at(pts, s1)]
    ded = [out[0]]                  # 부동소수점 오차로 생기는 길이 0 조각(같은 점 두 번) 빼기 (2026-11-11)
    for q in out[1:]:
        if math.dist(q, ded[-1]) > 1e-3:
            ded.append(q)
    return ded if len(ded) > 1 else [out[0], out[-1]]


def layout(J):
    """큰길 교차점 J 에서 갈림 교차점, 양쪽 정문 위치."""
    L = math.hypot(*SPUR_DIR)
    u = (SPUR_DIR[0]/L, SPUR_DIR[1]/L)
    right = (u[1], -u[0])                       # 진행 방향 오른쪽
    M = (J[0] + u[0]*SPUR_LEN, J[1] + u[1]*SPUR_LEN)
    gates = {name: (node, (M[0] + side*right[0]*GATE_LEN, M[1] + side*right[1]*GATE_LEN), side)
             for name, node, side in GATES}
    return M, u, right, gates


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    if MAIN_B in edges:
        raise RuntimeError(f'{MAIN_B} 가 이미 있다 (학교 진입로가 이미 적용됨)')
    e_main = edges[MAIN]
    shape = _read(e_main.get('shape'))
    lanes = sorted(e_main.findall('lane'), key=lambda l: int(l.get('index')))
    n = len(lanes)
    sc = _project(shape, MOUTH)
    J = _at(shape, sc)
    up = _slice(shape, sc - STUB, sc)
    dn = _slice(shape, sc, sc + STUB)
    M, _, _, gates = layout(J)
    speed = lanes[0].get('speed')
    new_edges = {S_IN, S_OUT} | {f'{g}_{d}' for g in gates for d in ('in', 'out')}
    turn_lane = {g: (0 if side > 0 else 1) for g, (_, _, side) in gates.items()}   # 오른쪽 학교 ← 0번, 왼쪽 ← 1번

    with tempfile.TemporaryDirectory() as work:
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            nodes = [('U', up[0], 'dead_end'), (J_NODE, J, 'priority'), ('D', dn[-1], 'dead_end'),
                     (MID_NODE, M, 'priority')] + [(node, P, 'dead_end') for node, P, _ in gates.values()]
            for nid, (x, y), t in nodes:
                fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            fh.write(f'  <edge id="{MAIN}" from="U" to="{J_NODE}" numLanes="{n}" speed="{speed}" priority="2" '
                     f'width="{LANE_W}" spreadType="center" shape="{_write(up)}"/>\n')
            fh.write(f'  <edge id="{MAIN_B}" from="{J_NODE}" to="D" numLanes="{n}" speed="{speed}" priority="2" '
                     f'width="{LANE_W}" spreadType="center" shape="{_write(dn)}"/>\n')
            pairs = [(S_IN, S_OUT, J_NODE, MID_NODE)] + \
                    [(f'{g}_in', f'{g}_out', MID_NODE, node) for g, (node, _, _) in gates.items()]
            for ein, eout, a, b in pairs:
                n_in = 2 if ein == S_IN else 1
                fh.write(f'  <edge id="{ein}" from="{a}" to="{b}" numLanes="{n_in}" speed="{SPUR_SPEED}" '
                         f'priority="1" width="{LANE_W}"/>\n')
                fh.write(f'  <edge id="{eout}" from="{b}" to="{a}" numLanes="1" speed="{SPUR_SPEED}" '
                         f'priority="1" width="{LANE_W}"/>\n')
            fh.write('</edges>\n')
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            con = [(MAIN, MAIN_B, k, k) for k in range(n)]
            con += [(MAIN, S_IN, 0, 0), (MAIN, S_IN, 0, 1), (S_OUT, MAIN_B, 0, 0)]
            for g in gates:
                con += [(S_IN, f'{g}_in', turn_lane[g], 0), (f'{g}_out', S_OUT, 0, 0)]
            con += [(a, b, 0, 0) for a, b in THROUGH]
            for f, t, a, b in con:
                fh.write(f'  <connection from="{f}" to="{t}" fromLane="{a}" toLane="{b}"/>\n')
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

    # 1) X_se_out 을 교차점에서 나눈다: 앞쪽은 원래 이름, 뒤쪽은 X_se_out_b
    e_b = ET.Element('edge', dict(e_main.attrib))
    e_b.set('id', MAIN_B)
    e_b.set('from', J_NODE)
    e_main.set('shape', _write(_slice(shape, 0.0, sc)))
    e_b.set('shape', _write(_slice(shape, sc, _length(shape))))
    for l in lanes:
        k = int(l.get('index'))
        pts = _read(l.get('shape'))
        a = sub_lane[(MAIN, k)][-1]            # 부분망에서 앞쪽 차로가 끝나는 점 (교차점 경계)
        b = sub_lane[(MAIN_B, k)][0]           # 부분망에서 뒤쪽 차로가 시작하는 점
        sa, sb = _project(pts, a), _project(pts, b)
        near = _slice(pts, 0.0, sa)[:-1] + [a]
        far = [b] + _slice(pts, sb, _length(pts))[1:]
        bl = ET.SubElement(e_b, 'lane', dict(l.attrib))
        bl.set('id', f'{MAIN_B}_{k}')
        bl.set('shape', _write(far))
        bl.set('length', f'{_length(far):.6f}')
        l.set('shape', _write(near))
        l.set('length', f'{_length(near):.6f}')
    e_main.set('to', J_NODE)
    pos = list(root).index(e_main) + 1
    root.insert(pos, e_b)

    # 2) 먼 쪽 교차점(X_se_widen)에서 쓰던 이름을 뒤쪽 엣지로
    for c in root.findall('connection'):
        if c.get('from') == MAIN:
            c.set('from', MAIN_B)
    for j in root.findall('junction'):
        jid = j.get('id')
        if jid == FAR_NODE or jid.startswith(f':{FAR_NODE}_'):
            j.set('incLanes', ' '.join(x.replace(f'{MAIN}_', f'{MAIN_B}_') if x.startswith(f'{MAIN}_') else x
                                       for x in j.get('incLanes', '').split()))

    # 3) 진입로·정문 길 엣지, 새 교차점, 내부 엣지, 연결 추가
    new_nodes = {J_NODE, MID_NODE} | {node for node, _, _ in gates.values()}
    prefs = tuple(f':{x}_' for x in new_nodes)

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in new_edges or e.get('id', '').startswith(prefs):
            root.insert(pos, e)
            pos += 1
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in new_nodes or j.get('id', '').startswith(prefs):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        if c.get('from') in new_edges or c.get('to') in new_edges or \
                c.get('from', '').startswith(prefs) or (c.get('from') == MAIN and c.get('to') == MAIN_B):
            root.insert(pos, c)
            pos += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'school_junction': (round(J[0], 1), round(J[1], 1)), 'school_spur_m': SPUR_LEN,
            'school_gate_roads': len(gates)}


def _center(a, b, trim_start, trim_end):
    """반대 방향 두 차로(a: 정방향, b: 역방향 차로를 뒤집은 것)의 가운데 선."""
    L = min(_length(a), _length(b))
    m = max(2, int(L / 2.0))
    c = [((p[0]+q[0])/2, (p[1]+q[1])/2)
         for p, q in zip((_at(a, L*i/m) for i in range(m+1)), (_at(b, L*i/m) for i in range(m+1)))]
    return _slice(c, trim_start, _length(c) - trim_end)


def _dashes(root, prefix, path, dash, gap, width='0.15'):
    """path 를 따라 흰 점선 (양 끝이 점선 조각으로 시작·끝나게 간격을 맞춘다)."""
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


def _movements():
    """회전 이동: (출발 엣지, 출발 차로, 도착 엣지, 이름) — 큰길 교차점 2개 + 갈림 교차점 4개."""
    mv = [(MAIN, 0, S_IN, 'main_right_in'), (S_OUT, 0, MAIN_B, 'right_out')]
    for g, _, side in GATES:
        mv.append((S_IN, 0 if side > 0 else 1, f'{g}_in', f'to_{g}'))
        mv.append((f'{g}_out', 0, S_OUT, f'from_{g}'))
    return mv


def _turn_sign(path):
    """경로 시작 방향 대비 끝 방향의 회전 (+ 좌회전, - 우회전)."""
    a0, a1, b0, b1 = path[0], path[1], path[-2], path[-1]
    u, v = (a1[0]-a0[0], a1[1]-a0[1]), (b1[0]-b0[0], b1[1]-b0[1])
    return u[0]*v[1] - u[1]*v[0]


def _edge_nodes(net, eid):
    e = next(e for e in net.findall('edge') if e.get('id') == eid)
    return e.get('from'), e.get('to')


def guide_paths(net_path):
    """회전 유도선 경로: 실제 차량 경로(출발 차로 끝 → 내부 차로 → 도착 차로 시작)를
    차로 폭 절반만큼 왼쪽으로 옮긴 경계 중, 출발 차로 끝(정지선 자리)부터 도착 차로 시작까지."""
    from env.roundabout_builder import _movement_path
    from env.roundabout_geometry import _offset
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    out = {}
    for eid, k, target, name in _movements():
        # 큰길 교차점의 우회전은 바깥 차로가 직진·우회전 겸용이고 진입로가 2차로라,
        # 차로 왼쪽 경계 유도선을 직진 차와 진입로 1번 차로로 들어가는 차가 밟는다 → 갈림 교차점만 그린다.
        if J_NODE in (_edge_nodes(net, eid)[1],):
            continue
        # 우회전 유도선은 만들지 않는다 (사용자 지침 2026-11-10) → 좌회전만
        center = [lane[f'{eid}_{k}'][-2]] + _movement_path(net_path, eid, target, k)
        if _turn_sign(center) <= 0:
            continue
        src, dst = lane[f'{eid}_{k}'], lane[f'{target}_0']
        center = [src[-2]] + _movement_path(net_path, eid, target, k) + [dst[1]]
        boundary = _offset(center, LANE_W / 2)
        a = math.dist(src[-2], src[-1])
        b = _length(boundary) - math.dist(dst[0], dst[1])
        out[name] = _slice(boundary, a, b)
    return out


def apply_deco(deco_path, net_path):
    """노란 중앙선, 진입로 차로 사이 점선, 정지선, 회전 화살표, 회전 유도선."""
    from env.roundabout_builder import _arrow, _read_lane_shapes, _stop_line, _movement_path
    from env.roundabout_geometry import _offset
    from env.roundabout_markings import dash_style
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    shapes = _read_lane_shapes(net_path)
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith('school_access_'):
            root.remove(p)
    h = LANE_W / 2
    # 1) 노란 중앙선: 진입로는 school_in 1번 차로 왼쪽 경계와 school_out 왼쪽 경계 사이, 학교 길은 정문 앞까지
    lines = {'school_access_center': _center(_offset(lane[f'{S_IN}_1'], h), _offset(lane[f'{S_OUT}_0'], h)[::-1],
                                             0.0, 0.0)}
    for g, _, _ in GATES:
        lines[f'school_access_center_{g}'] = _center(_offset(lane[f'{g}_in_0'], h),
                                                     _offset(lane[f'{g}_out_0'], h)[::-1], 0.0, GATE_INSET)
    for pid, pts in lines.items():
        if _length(pts) > 1.0:
            ET.SubElement(root, 'poly', dict(id=pid, color=YELLOW, fill='0', lineWidth='0.15',
                                              layer='6', shape=_write(pts)))
    # 2) 진입로 들어가는 두 차로 사이: 흰 점선, 정지선 앞 10m 는 실선 (차로 변경 금지)
    dash, gap, _ = dash_style(20)
    div = _offset(lane[f'{S_IN}_0'], h)
    Ld = _length(div)
    n_dash = _dashes(root, 'school_access_lane_dash', _slice(div, 0.0, Ld - 10.0), dash, gap)
    ET.SubElement(root, 'poly', dict(id='school_access_lane_solid', color=WHITE, fill='0', lineWidth='0.15',
                                      layer='6', shape=_write(_slice(div, Ld - 10.0, Ld - 1.0))))
    # 3) 정지선 (차로 끝 1m 뒤): 갈림 교차점으로 들어오는 세 방향 + 큰길로 나가는 진출로
    marks = []
    stops = [S_IN, S_OUT] + [f'{g}_out' for g, _, _ in GATES]
    for eid in stops:
        _stop_line(marks, f'school_access_stop_{eid}', shapes, eid, dist=1.0)
    # 4) 화살표: 큰길 바깥 차로는 직진·우회전, 나머지는 실제 회전 방향
    for eid, k, target, name in _movements():
        if eid == MAIN:
            _arrow(marks, f'school_access_arrow_{name}', shapes, eid, k, 'straight_right', 25.0)
            continue
        src = lane[f'{eid}_{k}']
        b = _movement_path(net_path, eid, target, k)[-1]
        d0 = (src[-1][0] - src[-2][0], src[-1][1] - src[-2][1])
        cross = d0[0]*(b[1] - src[-1][1]) - d0[1]*(b[0] - src[-1][0])
        turn = 'left' if cross > 0 else 'right'
        if any(a == eid for a, _ in THROUGH):
            turn = f'straight_{turn}'                       # 직진(옆 학교) + 큰길로 나가는 회전
        _arrow(marks, f'school_access_arrow_{name}', shapes, eid, k, turn, 7.0)
    for m in marks:
        root.append(ET.fromstring(m))
    # 5) 회전 유도선 (흰 점선 1m/1m)
    guides = 0
    for name, path in guide_paths(net_path).items():
        guides += _dashes(root, f'school_access_guide_{name}', path, 1.0, 1.0)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'school_centerlines': len(lines), 'school_lane_dashes': n_dash, 'school_stoplines': len(stops),
            'school_arrows': len(_movements()), 'school_guide_dashes': guides}


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
