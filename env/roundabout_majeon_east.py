"""마전동 동쪽 (원당대로 북쪽, 완정로 곡선과 서곶로 사이, 2026-11-12 네이버 위성 지도 기준) 도로.

  가장 중요한 것: 기존 틀(원당대로·서곶로 교차로·완정로 곡선·마전동 블록)은 그대로 두고, 새 길은 끝에 잇거나 T자로만 붙인다.

  - 서곶로 연장: 십자 교차로(SW_new_cross) 북쪽의 막다른 짧은 길(SW_new_upper_in 남행 6차로 / SW_new_upper_out 북행 5차로)을
    같은 단면(차로 수·폭, 노란 중앙선 + 흰 점선) 그대로 이어 위쪽으로. 처음엔 짧은 길 방향, 큰 원호로 북쪽(약 8° 동쪽)으로 튼다.
    짧은 길 이름은 그대로 둔다 (RL 배경 교통이 여기서 출발·도착한다).
  - 위쪽 변 길(me_top): 완정로 곡선(S_conn_up, 북행)에서 북북동으로 올라가 위에서 동쪽으로 꺾여 서곶로 연장까지.
  - 안쪽 골목: 원당대로에서 북쪽으로 올라가 위쪽 변 길까지 가는 가운데 길(me_s1), 가운데 길에서 서곶로로 빠지는 동서 골목 둘.
  큰길과 만나는 곳은 지금까지처럼 우회전 진입·진출만 (안쪽 길 공통 방식, env.roundabout_interior).

roundabout_south_extend 가 마전동 블록 골목 다음 단계로 호출한다.
"""
import math
import xml.etree.ElementTree as ET

PREFIX = 'me_'
STUB = ('SW_new_upper_in', 'SW_new_upper_out')     # 남행(교차로로 들어감), 북행(교차로에서 나감)
STUB_END = 'SW_new_upper'
EXT = ('sg_ext_s', 'sg_ext_n')                     # 서곶로 연장 남행, 북행
EXT_END = 'sg_ext_end'
EXT_SPEED = 13.89
EXT_STRAIGHT = 60.0                                # 짧은 길 방향 그대로 곧게 가는 길이 (m)
EXT_TOP = (2305.0, 190.0)                          # 연장 끝 — 2026-11-12 마전동 북쪽 동네 위 끝까지 (처음엔 (2275, -140))
EXT_RADIUS = 250.0
YELLOW, WHITE = '235,190,40', '255,255,255'

TEES = [
    ('me_T_A', 'S_conn_up', (1945.0, -462.0)),               # 완정로 곡선 (북행, 동쪽 차로) — 위쪽 변 길
    ('me_T_S1', 'SW_end_T_left_in_far2', (2040.0, -675.0)),  # 원당대로 (서행, 북쪽 차로) — 가운데 길
    ('me_T_E1', 'sg_ext_s', (2245.0, -540.0)),               # 서곶로 연장 (남행, 서쪽 차로) — 아래 동서 골목
    ('me_T_E2', 'sg_ext_s', (2262.0, -400.0)),               # 서곶로 연장 — 위 동서 골목
    ('me_T_TOP', 'sg_ext_s', (2272.0, -195.0)),              # 서곶로 연장 — 위쪽 변 길
]
NODES = {'me_J1': (2050.0, -520.0), 'me_J2': (2060.0, -390.0), 'me_J3': (2090.0, -157.0)}   # 위쪽 변 길이 동쪽으로 꺾인 뒤 (가운데 길과 거의 직각)
STREETS = [
    ('me_s1a', 'me_T_S1', 'me_J1', 1, []),
    ('me_s1b', 'me_J1', 'me_J2', 1, []),
    ('me_s1c', 'me_J2', 'me_J3', 1, []),
    ('me_e1', 'me_J1', 'me_T_E1', 0, []),
    ('me_e2', 'me_J2', 'me_T_E2', 0, []),
    ('me_top1', 'me_T_A', 'me_J3', 1, [(1979.0, -330.0), (2023.0, -242.0), (2048.0, -178.0)]),
    ('me_top2', 'me_J3', 'me_T_TOP', 1, [(2165.0, -150.0)]),
]
# 2026-11-14 사용자 요청: 원당대로에서 올라오는 가운데 길 첫 구간(KB국민은행 옆)을 편도 2차로, 원당대로로는 이중 우회전 진출.
#   차로 수는 위쪽 교차점(me_J1)에서 바뀐다.
LANES = {'me_s1a': 2}
CFG = dict(TEES=TEES, REBUILD={}, NODES=NODES, STREETS=STREETS, PREFIX=PREFIX, MEDIAN={},
           DOUBLE_CENTER=set(), APPROACH=30.0, LANES=LANES)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _ext_line(net):
    """서곶로 연장 중앙선 (짧은 길 끝 교차점에서 시작)."""
    from env.roundabout_interior import _fillet
    es = {e.get('id'): e for e in net.findall('edge')}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    lane = _read(es[STUB[1]].find("lane[@index='0']").get('shape'))
    d = (lane[-1][0] - lane[0][0], lane[-1][1] - lane[0][1])
    L = math.hypot(*d)
    d = (d[0]/L, d[1]/L)
    c0 = J[STUB_END]
    cc = (c0[0] + d[0]*EXT_STRAIGHT, c0[1] + d[1]*EXT_STRAIGHT)
    return _fillet([c0, cc, EXT_TOP], radius=EXT_RADIUS, step=4.0)


def apply_extension(net_path):
    """서곶로 연장 (같은 차로 수, 짧은 길 끝 교차점을 이음부로 다시 만든다)."""
    from env.roundabout_graft import graft
    from env.roundabout_geometry import _offset
    # 짧은 길 엣지에는 shape 가 없어 graft 가 0번 차로 선을 길 모양으로 쓰면 차로가 옆으로 밀린다 → 실제 모양(맨 왼쪽 차로의
    # 왼쪽 경계, spreadType right)을 먼저 적어 둔다
    tree = ET.parse(net_path)
    for e in tree.getroot().findall('edge'):
        if e.get('id') in STUB and not e.get('shape'):
            ls = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
            left = _read(ls[-1].get('shape'))
            e.set('shape', ' '.join(f'{x:.6f},{y:.6f}' for x, y in _offset(left, float(ls[-1].get('width', '3.2'))/2)))
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    net = ET.parse(net_path).getroot()
    es = {e.get('id'): e for e in net.findall('edge')}
    n_s, n_n = len(es[STUB[0]].findall('lane')), len(es[STUB[1]].findall('lane'))
    w = float(es[STUB[0]].find('lane').get('width', '3.2'))
    line = _ext_line(net)
    streets = [(EXT[1], STUB_END, EXT_END, n_n, w, EXT_SPEED, 2, line),
               (EXT[0], EXT_END, STUB_END, n_s, w, EXT_SPEED, 2, line[::-1])]
    extra = [(STUB[1], EXT[1], k, k) for k in range(n_n)] + [(EXT[0], STUB[0], k, k) for k in range(n_s)]
    graft(net_path, {EXT_END: line[-1]}, streets, [(STUB[0], [], [STUB[0]]), (STUB[1], [], [STUB[1]])],
          attach={STUB_END}, extra_con=extra, node_types={EXT_END: 'dead_end'})
    return {'sg_ext_m': round(sum(math.dist(a, b) for a, b in zip(line, line[1:])), 1), 'sg_ext_lanes': (n_n, n_s)}


def apply_net(net_path):
    info = apply_extension(net_path)
    from env import roundabout_interior as it
    info.update({f'me_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(CFG, it.apply_net, net_path).items()})
    # 회전 경로 끝 0.5m 미만 조각 점 정리 (env.roundabout_nw_shops 와 같은 처리, 이 단계 교차점만)
    from env.roundabout_nw_shops import _strip_short_internal
    _strip_short_internal(net_path, tuple(NODES) + tuple(t[0] for t in TEES))
    return info


def apply_deco(deco_path, net_path):
    """서곶로 연장: 짧은 길과 같은 노란 중앙선(0.2) + 흰 점선. 안쪽 길: 안쪽 길 공통 표시."""
    from env.roundabout_geometry import _offset
    from env.roundabout_markings import dash_style
    from env.roundabout_nw_shops import _dashes
    net = ET.parse(net_path).getroot()
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith('sg_ext_'):
            root.remove(p)
    lanes = {}
    for e in net.findall('edge'):
        eid = e.get('id', '')
        if e.get('function') == 'internal' or not eid.startswith(EXT):
            continue
        for l in e.findall('lane'):
            lanes.setdefault(eid.split('_me_')[0] if '_me_' in eid else eid, []).append(
                (eid, int(l.get('index')), _read(l.get('shape'))))
    dash, gap, _ = dash_style(20)
    n = 0
    for base, items in lanes.items():
        for eid, k, pts in items:
            nl = max(i for e2, i, _ in items if e2 == eid) + 1
            if k < nl - 1:
                n += _dashes(root, f'sg_ext_div_{eid}_{k}', _offset(pts, 1.6), dash, gap)
            elif base == EXT[1]:                    # 북행 맨 왼쪽 차로 왼쪽 경계 = 중앙선
                ET.SubElement(root, 'poly', dict(id=f'sg_ext_center_{eid}', color=YELLOW, fill='0', lineWidth='0.2',
                                                  layer='6', shape=' '.join(f'{x:.3f},{y:.3f}' for x, y in
                                                                            _offset(pts, 1.6))))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    from env import roundabout_interior as it
    info = {'sg_ext_marks': n}
    info.update({f'me_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(CFG, it.apply_deco, deco_path, net_path).items()})
    return info
