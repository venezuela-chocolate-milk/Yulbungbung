"""회전교차로 서쪽 아래 합류부(cluster_W_in1_mid_W_out0_mid) 신호 (2026-11-14 사용자 요청: 학습 전에 정체 해결).

  문제: 단봉로(DANBONG_diag_end) 2·3번 차로 좌회전은 회전교차로에서 나오는 차(W_out0)를 가로지른 뒤
        남서쪽 진입로(W_in1_far)에 합류한다 — 두 흐름 모두에 양보(m)라 틈이 거의 없어, 단봉로 줄이
        유승한내들 정문(env.roundabout_yuseung)까지 늘 밀려 왔다 (합류 직전 정지 차량 있는 시간 92%).
  신호 (고정, 정문 신호와 같은 주기 CYCLE):
    P1  남서쪽 직진 W_in1_far (G) + 회전교차로에서 나오는 차 W_out0 (G) + 단봉로 우회전 (g)  — 단봉로 좌회전 정지
    P2  단봉로 좌·우회전 보호 (G)                                                       — 남서쪽 직진·나오는 차 정지
  시험 (2026-11-14, 3000초, 전체 교통):
    신호 없음                     정문 대기 309초, 단봉로 지체 332초
    P2 에 나오는 차 계속 진행     단봉로 지체 231초 (좌·우회전이 나오는 차 흐름에 계속 막힘)
    단봉로 배경 교통 절반         단봉로 지체 230초 (수요가 아니라 구조 문제)
    지금 (완전 보호)              정문 대기 15초, 단봉로 지체 54초, 남서쪽 지체 28→40초, 회전교차로 안 정체 없음
  env.roundabout_black_t_signal 처럼 교차점 모양·내부 경로는 그대로 두고 신호 계획만 넣는다.
roundabout_south_extend 파이프라인에서 apply_net 을 부른다 (유승한내들 정문 뒤).
"""
import xml.etree.ElementTree as ET

T = 'cluster_W_in1_mid_W_out0_mid'
LEFT_SRC, LEFT_DST = 'DANBONG_diag_end', 'W_in1'
MAIN_SRC = 'W_in1_far'
EXIT_SRC = 'W_out0'
P1, P2 = 30, 25                  # 녹색 (초)
YELLOW, ALLRED = 3, 2
CYCLE = P1 + P2 + 2*(YELLOW + ALLRED)
OFFSET = 0


def _links(root):
    j = next(x for x in root.findall('junction') if x.get('id') == T)
    ints = j.get('intLanes').split()
    chain = {f"{c.get('from')}_{c.get('fromLane')}": c.get('via') for c in root.findall('connection')
             if c.get('from', '').startswith(f':{T}_') and c.get('via')}

    def link(c):
        v = c.get('via', '')
        while v not in ints and v in chain:
            v = chain[v]
        return ints.index(v) if v in ints else None
    return j, ints, link


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    if any(t.get('id') == T for t in root.findall('tlLogic')):
        return {'w_merge_signal': 'already'}
    j, ints, link = _links(root)
    kind = {}
    for c in root.findall('connection'):
        if c.get('from', '').startswith(':') or link(c) is None:
            continue
        f, t = c.get('from'), c.get('to')
        kind[link(c)] = ('left' if (f, t) == (LEFT_SRC, LEFT_DST) else 'right' if f == LEFT_SRC else
                         'main' if f == MAIN_SRC else 'exit' if f == EXIT_SRC else 'other')
    assert len(kind) == len(ints) and 'other' not in kind.values(), kind
    st = lambda G=(), g=(), y=(): ''.join('y' if kind[k] in y else 'G' if kind[k] in G else
                                          'g' if kind[k] in g else 'r' for k in range(len(ints)))
    plan = [(P1, st(G=('main', 'exit'), g=('right',))),
            (YELLOW, st(g=('right',), y=('main', 'exit'))),
            (ALLRED, st(g=('right',))),
            (P2, st(G=('right', 'left'))),
            (YELLOW, st(g=('right',), y=('left',))),
            (ALLRED, st(g=('right',)))]
    j.set('type', 'traffic_light')
    first = plan[0][1]
    for c in root.findall('connection'):
        if not c.get('from', '').startswith(':') and link(c) is not None:
            k = link(c)
            c.set('tl', T)
            c.set('linkIndex', str(k))
            c.set('state', first[k])
    tl = ET.Element('tlLogic', {'id': T, 'type': 'static', 'programID': '0', 'offset': str(OFFSET)})
    for dur, s in plan:
        ET.SubElement(tl, 'phase', {'duration': str(dur), 'state': s})
    others = root.findall('tlLogic')
    root.insert(list(root).index(others[-1]) + 1, tl)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'w_merge_signal': 'added', 'w_merge_cycle_s': CYCLE}


def apply_deco(deco_path, net_path):
    """새로 신호를 받는 남서쪽 진입로·회전교차로 출구에 정지선 (단봉로 쪽은 roundabout_config STOP_LINES 에 있음)."""
    from env.roundabout_builder import _stop_line, _read_lane_shapes
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in [p for p in root.findall('poly') if p.get('id', '').startswith('w_merge_stop_')]:
        root.remove(p)
    marks = []
    shapes = _read_lane_shapes(net_path)
    for eid in (MAIN_SRC, EXIT_SRC):
        _stop_line(marks, f'w_merge_stop_{eid}', shapes, eid, dist=1.0)
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'w_merge_stoplines': len(marks)}
