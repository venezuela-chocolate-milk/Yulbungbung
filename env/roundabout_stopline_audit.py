"""맵 전체 정지선 점검 (2026-11-14 사용자 요청: 정지선·정지선 뒤 화살표 빠진 곳 전부 추가).

  정지선이 있어야 하는 진입 차로 = 교차점(이웃 교차점 3개 이상)으로 들어가며, 신호를 받거나(tl)
  연결이 모두 양보(m/s/w)인 차로. 그 차로 끝 3m 안에 정지선·양보선(id 에 stop/yield/give, 회전교차로 yl_)이
  없으면 정지선을 그린다. 빼는 것:
    - 주차장 안 통로 (주차장 엣지 이름 p?_ / sp_ 로 시작) — 주차장 안에는 정지선을 긋지 않는다
    - 회전교차로 (ring*) — 양보선이 따로 있다
    - 합류 (직진 하나뿐이고 그 차로로 우선 직진이 따로 들어오는 곳) — 정지선 대신 합류
    - 5m 보다 짧은 차로
  화살표는 이 뒤에 env.roundabout_stop_arrows 를 다시 불러 넣는다 (정지선이 있는데 화살표가 없는 차로 전부).
roundabout_south_extend 가 모든 도로 단계가 끝난 뒤 부른다.
"""
import collections
import math
import re
import xml.etree.ElementTree as ET

PREFIX = 'audit_stop_'
PARK = re.compile(r'^(p[a-z]|sp)[a-z0-9]*_')


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _turn(a, b):
    d1 = (a[-1][0] - a[-2][0], a[-1][1] - a[-2][1])
    d2 = (b[1][0] - b[0][0], b[1][1] - b[0][1])
    ang = math.degrees(math.atan2(d1[0]*d2[1] - d1[1]*d2[0], d1[0]*d2[0] + d1[1]*d2[1]))
    return 'straight' if abs(ang) < 35 else ('left' if ang > 0 else 'right')


def apply(deco_path, net_path):
    from shapely.geometry import LineString, Point, Polygon
    from env.roundabout_builder import _read_lane_shapes, _stop_line
    net = ET.parse(net_path).getroot()
    E = {e.get('id'): e for e in net.findall('edge') if e.get('function') != 'internal'}
    lanes = {l.get('id'): _read(l.get('shape')) for e in E.values() for l in e.findall('lane')}
    nb = collections.defaultdict(set)
    for e in E.values():
        nb[e.get('to')].add(e.get('from'))
        nb[e.get('from')].add(e.get('to'))
    states, tl, turns, into = collections.defaultdict(set), set(), collections.defaultdict(set), collections.defaultdict(list)
    for c in net.findall('connection'):
        f = c.get('from')
        if f.startswith(':'):
            continue
        k = f'{f}_{c.get("fromLane")}'
        t = f'{c.get("to")}_{c.get("toLane")}'
        states[k].add(c.get('state'))
        if c.get('tl'):
            tl.add(k)
        if len(lanes[k]) > 1 and len(lanes[t]) > 1:
            kind = _turn(lanes[k], lanes[t])
            turns[k].add(kind)
            into[t].append((k, kind, c.get('state')))
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith(PREFIX):
            root.remove(p)
    marks = []
    for p in root.findall('poly'):
        pid, s = p.get('id', '').lower(), p.get('shape')
        if not s or not any(w in pid for w in ('stop', 'yield', 'give')) and not pid.startswith('yl_'):
            continue
        pts = _read(s)
        marks.append(Polygon(pts).buffer(0) if len(pts) >= 3 and p.get('fill') == '1' else LineString(pts).buffer(0.3))
    missing = collections.defaultdict(list)
    for k, ss in states.items():
        eid, idx = k.rsplit('_', 1)
        J = E[eid].get('to')
        if len(nb[J]) < 3 or J.startswith('ring') or PARK.match(eid):
            continue
        if not (k in tl or ss <= {'m', 's', 'w'}):
            continue
        pts = lanes[k]
        if LineString(pts).length < 5.0:
            continue
        if turns[k] == {'straight'}:
            targets = [t for t, lst in into.items() if any(x[0] == k for x in lst)]
            if any(any(x[0].rsplit('_', 1)[0] != eid and x[1] == 'straight' and x[2] == 'M' for x in into[t])
                   for t in targets):
                continue                                    # 합류
        if any(g.distance(Point(pts[-1])) < 3.0 for g in marks):
            continue
        missing[eid].append(int(idx))
    shapes = _read_lane_shapes(net_path)
    out = []
    for eid, idx in sorted(missing.items()):
        _stop_line(out, f'{PREFIX}{eid}', shapes, eid, dist=1.0)        # 정지선은 진입로 전체 폭
    for m in out:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'stopline_audit_added': {e: sorted(v) for e, v in sorted(missing.items())}}
