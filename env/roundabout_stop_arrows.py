"""정지선 뒤에 진행 방향 화살표가 없는 진입 차로에 화살표를 넣는다. (2026-11-12 사용자 요청, 맵 전체)

  - 정지선: 노면 표시 중 id 에 'stop' 이 든 흰 선/도형. 그 정지선 2.5m 안에서 끝나는 진입 차로(연결이 있는 차로)를 찾는다.
  - 그 차로 끝 40m 안에 화살표(가운데가 그 차로 안)가 하나도 없으면, 실제 연결의 회전 방향(직진/좌/우, 둘·셋이면
    직좌/직우/좌·우)으로 화살표를 정지선 뒤에 넣는다. 위치는 차로 끝에서 min(8, 차로 길이 - 3.5)m
    (화살표가 차로 안에 다 들어가게). 차로가 7.5m 보다 짧으면 넣지 않고 보고한다.
  - 이미 화살표가 있는 차로·정지선이 없는 차로는 건드리지 않는다. 다른 단계가 그린 표시도 그대로 둔다.
roundabout_south_extend 가 노면 표시 단계들이 다 끝난 뒤(신호등 그림 앞) 호출한다.
"""
import math
import xml.etree.ElementTree as ET

PREFIX = 'stoparrow_'
SEARCH = 40.0
KINDS = {frozenset({'straight'}): ['straight'], frozenset({'left'}): ['left'], frozenset({'right'}): ['right'],
         frozenset({'straight', 'left'}): ['straight_left'], frozenset({'straight', 'right'}): ['straight_right'],
         frozenset({'left', 'right'}): ['left', 'right'],
         frozenset({'straight', 'left', 'right'}): ['straight_left', 'right']}


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _turn(a, b):
    d1 = (a[-1][0] - a[-2][0], a[-1][1] - a[-2][1])
    d2 = (b[1][0] - b[0][0], b[1][1] - b[0][1])
    ang = math.degrees(math.atan2(d1[0]*d2[1] - d1[1]*d2[0], d1[0]*d2[0] + d1[1]*d2[1]))
    return 'straight' if abs(ang) < 35 else ('left' if ang > 0 else 'right')


def apply(deco_path, net_path):
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import substring
    from env.roundabout_builder import _arrow, _read_lane_shapes
    net = ET.parse(net_path).getroot()
    lanes, width = {}, {}
    for e in net.findall('edge'):
        if e.get('function') == 'internal':
            continue
        for l in e.findall('lane'):
            lanes[(e.get('id'), int(l.get('index')))] = _read(l.get('shape'))
            width[(e.get('id'), int(l.get('index')))] = float(l.get('width', '3.2'))
    turns, pred = {}, {}
    for c in net.findall('connection'):
        f = c.get('from')
        if f.startswith(':'):
            continue
        a = lanes.get((f, int(c.get('fromLane'))))
        b = lanes.get((c.get('to'), int(c.get('toLane'))))
        if a and b and len(a) > 1 and len(b) > 1:
            t = _turn(a, b)
            turns.setdefault((f, int(c.get('fromLane'))), set()).add(t)
            if t == 'straight':
                pred.setdefault((c.get('to'), int(c.get('toLane'))), set()).add((f, int(c.get('fromLane'))))
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith(PREFIX):
            root.remove(p)
    stops, arrows = [], []
    for p in root.findall('poly'):
        pid, s = p.get('id', '').lower(), p.get('shape')
        if not s:
            continue
        pts = _read(s)
        if 'arrow' in pid and len(pts) >= 3:
            arrows.append(Polygon(pts).buffer(0).centroid)
        elif 'stop' in pid and p.get('color', '').startswith('255,255,255'):
            stops.append(Polygon(pts).buffer(0) if len(pts) >= 3 and p.get('fill') == '1'
                         else LineString(pts).buffer(0.3))
    shapes = _read_lane_shapes(net_path)
    marks, added, short, removed = [], [], [], []
    for key, kinds in sorted(turns.items()):
        pts = lanes[key]
        end = Point(pts[-1])
        if not any(g.distance(end) < 2.5 for g in stops):
            continue
        L = LineString(pts).length
        tail = substring(LineString(pts), max(0.0, L - SEARCH), L).buffer(width[key]/2 - 0.4, cap_style=2)
        if L < SEARCH:                                  # 짧은 차로: 바로 앞(직진으로 이어 들어오는) 차로 끝부분도 본다
            for pk in pred.get(key, ()):
                pp = LineString(lanes[pk])
                tail = tail.union(substring(pp, max(0.0, pp.length - (SEARCH - L)), pp.length)
                                  .buffer(width[pk]/2 - 0.4, cap_style=2))
        if any(tail.contains(c) for c in arrows):
            continue
        if L < 7.5:
            short.append(f'{key[0]}_{key[1]}')
            continue
        dist = min(8.0, L - 3.5)
        # 이 차로 근처에서 차로 가운데가 아니라 차선 위에 걸친 옛 화살표(예전 단계에서 차로만 옮겨지고 남은 것)는 지운다
        near = tail.buffer(width[key])
        centers = [LineString(v) for v in lanes.values()]
        for p in list(root.findall('poly')):
            pid = p.get('id', '')
            if 'arrow' not in pid.lower() or pid.startswith(PREFIX) or not p.get('shape'):
                continue
            q = _read(p.get('shape'))
            if len(q) < 3:
                continue
            c = Polygon(q).buffer(0).centroid
            if near.contains(c) and min(cl.distance(c) for cl in centers if cl.distance(c) < 5) > 1.2:
                root.remove(p)
                removed.append(pid)
        for kind in KINDS.get(frozenset(kinds), []):
            _arrow(marks, f'{PREFIX}{key[0]}_{key[1]}_{kind}', shapes, key[0], key[1], kind, dist)
        added.append(f'{key[0]}_{key[1]}:{"+".join(sorted(kinds))}')
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'stop_arrows_added': len(added), 'stop_arrows_lanes': added, 'stop_arrows_too_short': short,
            'stop_arrows_misaligned_removed': removed}
