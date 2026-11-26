"""검은 도로 끝 T자 교차로(SW_final_forward)를 신호 교차로로 바꾼다.

신호가 없을 때는 검은 도로 좌회전 2개 차로가 동서 방향 직진(서행 5 + 동행 4 차로)에 양보하다
틈을 못 찾아 막히고 순간이동까지 됐다. 좌회전 2차로 교차로는 실제로도 신호로 운영한다.

원래 T자는 회전 경로를 손으로 다듬어 만든 것이라 netconvert 로 다시 만들지 않는다.
교차로 모양·내부 경로·규칙표는 그대로 두고, 교차로 유형/연결의 신호 번호/신호 계획만 넣는다.

링크 번호 (교차로 intLanes 순서):
   0      검은 도로 우회전 → 서쪽
   1, 2   검은 도로 좌회전 → 동쪽 (동행 2·3번 차로로 합류)
   3      서행 우회전 → 검은 도로 북쪽
   4~8    서행 직진
   9~12   동행 직진
  13, 14  동행 좌회전 → 검은 도로 북쪽

신호 단계 (G 보호 진행, g 양보하며 진행, y 황색, r 정지):
   A 동서 직진        (검은 도로 우회전은 양보 진행)
   B 동행 좌회전 보호  (동행 직진도 진행, 서행은 정지)
   C 검은 도로 좌·우회전
  단계 사이 황색 3초 + 전적색 2초. 서행 우회전(3)은 항상 진행.
"""
import xml.etree.ElementTree as ET

T = 'SW_final_forward'
N_LINKS = 15
GREEN = dict(A=40, B=15, C=20)
YELLOW, ALLRED = 3, 2


def _state(G=(), g=(), y=()):
    s = ['r'] * N_LINKS
    for k in g:
        s[k] = 'g'
    for k in G:
        s[k] = 'G'
    for k in y:
        s[k] = 'y'
    return ''.join(s)


WB, EB, EBL, BL = range(4, 9), range(9, 13), (13, 14), (1, 2)


def phases():
    A = _state(G=[3, *WB, *EB], g=[0])
    B = _state(G=[0, 3, *EB, *EBL])
    C = _state(G=[0, 3, *BL])
    return [
        (GREEN['A'], A),
        (YELLOW, _state(G=[3, *EB], y=[0, *WB])),     # 서행 직진 끝 (동행 직진은 B 에서도 진행)
        (ALLRED, _state(G=[3, *EB])),
        (GREEN['B'], B),
        (YELLOW, _state(G=[0, 3], y=[*EB, *EBL])),
        (ALLRED, _state(G=[0, 3])),
        (GREEN['C'], C),
        (YELLOW, _state(G=[3], y=[0, *BL])),
        (ALLRED, _state(G=[3])),
    ]


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    if any(t.get('id') == T for t in root.findall('tlLogic')):
        return {'black_t_signal': 'already'}
    j = next(x for x in root.findall('junction') if x.get('id') == T)
    ints = j.get('intLanes').split()
    assert len(ints) == N_LINKS, len(ints)
    j.set('type', 'traffic_light')

    plan = phases()
    first = plan[0][1]
    for c in root.findall('connection'):
        if c.get('via', '') in ints:
            k = ints.index(c.get('via'))
            c.set('tl', T)
            c.set('linkIndex', str(k))
            c.set('state', first[k].upper() if first[k] == 'G' else first[k])

    tl = ET.Element('tlLogic', {'id': T, 'type': 'static', 'programID': '0', 'offset': '0'})
    for dur, st in plan:
        ET.SubElement(tl, 'phase', {'duration': str(dur), 'state': st})
    others = root.findall('tlLogic')
    anchor = others[-1] if others else root.findall('junction')[0]
    idx = list(root).index(anchor) + (1 if others else 0)
    root.insert(idx, tl)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'black_t_signal': 'added', 'black_t_cycle_s': sum(d for d, _ in plan)}
