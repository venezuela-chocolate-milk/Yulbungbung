"""T자 교차로(SW_final_forward)로 들어오는 동행 5차로(SW_end_T_right_in)의 4번 차로(중앙선 쪽 두 번째)를
좌회전 전용에서 직진·좌회전 겸용으로 바꾼다.

원래 T자 교차로는 회전 경로를 따로 손질해 만든 것이라 netconvert 로 다시 만들면 모양이 바뀐다.
그래서 기존 연결/내부 경로는 그대로 두고 직진 연결 1개만 끼워 넣는다.
  - 내부 경로: 3번 차로 직진 경로(:SW_final_forward_10_2)와 나란히, 양 끝을 4번 차로 끝/진출 3번 차로 시작에 맞춘다.
  - 연결 순서(링크 번호): 같은 진입 차로 안에서 직진이 좌회전보다 앞 → 새 링크는 13번, 기존 13·14번은 14·15번.
  - 규칙표: 새 직진은 검은 도로 좌회전(3번, 같은 진출 차로로 합류)과만 충돌하고, 그 좌회전이 양보한다.
  - 노면 화살표: 4번 차로 좌회전 화살표를 직진·좌회전 화살표로 바꾼다.
"""
import math
import xml.etree.ElementTree as ET

T = 'SW_final_forward'
IN, OUT = 'SW_end_T_right_in', 'SW_end_T_left_out'
LANE = 3
STRAIGHT_VIA_EDGE = f':{T}_10'
REF_LANE = f':{T}_10_2'
NEW_LANE = f':{T}_10_3'
NEW_INDEX = 13
MERGE_FOE = 3            # 검은 도로 좌회전 → SW_end_T_left_out 3번 차로
ARROW_ID = 'SW_end_T_right_arrow_3'


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _ins(bits, index, bit):
    """비트 문자열(오른쪽 끝 = 링크 0)의 index 자리에 bit 를 끼운다."""
    k = len(bits) - index
    return bits[:k] + bit + bits[k:]


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    if root.find(f".//lane[@id='{NEW_LANE}']") is not None:
        return {'t_straight_left': 'already'}
    edges = {e.get('id'): e for e in root.findall('edge')}
    lane = lambda eid, k: next(l for l in edges[eid].findall('lane') if l.get('index') == str(k))

    # 1) 내부 경로: 기준 직진 경로를 시작/끝 오프셋을 길이 비율로 섞어 평행 이동
    ref = _read(lane(STRAIGHT_VIA_EDGE, 2).get('shape'))
    a0 = _read(lane(IN, LANE - 1).get('shape'))[-1]
    a1 = _read(lane(IN, LANE).get('shape'))[-1]
    b0 = _read(lane(OUT, LANE - 1).get('shape'))[0]
    b1 = _read(lane(OUT, LANE).get('shape'))[0]
    ds, de = (a1[0]-a0[0], a1[1]-a0[1]), (b1[0]-b0[0], b1[1]-b0[1])
    L = _length(ref)
    pts, acc = [], 0.0
    for i, p in enumerate(ref):
        if i:
            acc += math.dist(ref[i-1], p)
        f = acc / L
        pts.append((p[0] + ds[0]*(1-f) + de[0]*f, p[1] + ds[1]*(1-f) + de[1]*f))
    pts[0], pts[-1] = a1, b1
    new = ET.SubElement(edges[STRAIGHT_VIA_EDGE], 'lane', dict(lane(STRAIGHT_VIA_EDGE, 2).attrib))
    new.set('id', NEW_LANE)
    new.set('index', '3')
    new.set('shape', _write(pts))
    new.set('length', f'{_length(pts):.6f}')

    # 2) 교차점: 내부 차로 목록과 규칙표
    j = next(x for x in root.findall('junction') if x.get('id') == T)
    ints = j.get('intLanes').split()
    ints.insert(NEW_INDEX, NEW_LANE)
    j.set('intLanes', ' '.join(ints))
    reqs = j.findall('request')
    for r in reqs:
        bit = '1' if int(r.get('index')) == MERGE_FOE else '0'
        r.set('response', _ins(r.get('response'), NEW_INDEX, bit))
        r.set('foes', _ins(r.get('foes'), NEW_INDEX, bit))
        if int(r.get('index')) >= NEW_INDEX:
            r.set('index', str(int(r.get('index')) + 1))
    n = len(ints)
    foes = ['0'] * n
    foes[n - 1 - MERGE_FOE] = '1'
    row = ET.Element('request', {'index': str(NEW_INDEX), 'response': '0' * n, 'foes': ''.join(foes), 'cont': '0'})
    pos = list(j).index(next(r for r in j.findall('request') if r.get('index') == str(NEW_INDEX + 1)))
    j.insert(pos, row)

    # 3) 연결: 같은 차로의 좌회전 연결 바로 앞에 직진 연결을 넣는다
    conns = list(root)
    left = next(c for c in root.findall('connection')
                if c.get('from') == IN and c.get('fromLane') == str(LANE) and c.get('dir') == 'l')
    c_new = ET.Element('connection', {'from': IN, 'to': OUT, 'fromLane': str(LANE), 'toLane': str(LANE),
                                      'contPos': '0.000000', 'via': NEW_LANE, 'dir': 's', 'state': 'M'})
    root.insert(conns.index(left), c_new)
    last_int = [c for c in root.findall('connection') if c.get('from') == STRAIGHT_VIA_EDGE][-1]
    c_int = ET.Element('connection', {'from': STRAIGHT_VIA_EDGE, 'to': OUT, 'fromLane': '3', 'toLane': str(LANE),
                                      'dir': 's', 'state': 'M'})
    root.insert(list(root).index(last_int) + 1, c_int)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'t_straight_left': 'added'}


def apply_deco(deco_path, net_path):
    from env.roundabout_builder import _read_lane_shapes, _arrow
    from shapely.geometry import LineString, Point
    tree = ET.parse(deco_path)
    root = tree.getroot()
    shapes = _read_lane_shapes(net_path)
    old = next((p for p in root.findall('poly') if p.get('id') == ARROW_ID), None)
    ref = next((p for p in root.findall('poly') if p.get('id') == 'SW_end_T_right_arrow_2'), None)
    if old is None or ref is None:
        return {'t_arrow': 'not found'}
    # 옆 차로 직진 화살표와 같은 거리에 둔다 (직진 화살표 머리 끝 = 원점 + 2.1m)
    lane2 = LineString(shapes[(IN, LANE - 1)])
    tip = max(lane2.project(Point(q)) for q in _read(ref.get('shape')))
    dist = lane2.length - tip + 2.1
    idx = list(root).index(old)
    root.remove(old)
    marks = []
    _arrow(marks, ARROW_ID, shapes, IN, LANE, 'straight_left', dist)
    for k, m in enumerate(marks):
        root.insert(idx + k, ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'t_arrow': 'straight_left'}
