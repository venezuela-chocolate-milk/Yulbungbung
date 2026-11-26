"""마전동 블록 골목 (완정로65번길·완정로·원당대로로 둘러싸인 삼각형, 2026-11-11 네이버 지도 캡처 기준).

  가장 중요한 것: 새 골목이 기존 큰길·막다른 길과 자연스럽게 이어지고 차가 문제없이 지나가는 것.
  SUMO 삼각형은 실제보다 북서쪽이 훨씬 넓어(완정로가 바깥으로 크게 휜다) 네이버 위치를 그대로 옮기지 않고,
  길의 연결 관계와 순서(어느 길이 어느 길과 몇 번째로 만나는지)를 살려 SUMO 모양에 맞게 놓았다.

  길 (차로당 3.2m, 30 km/h). 처음엔 모두 왕복 1차로였고, 2026-11-14 사용자 요청(불법 주정차)으로 전부 편도 2차로
  (LANES, 중앙선 기준 양쪽으로 한 차로씩). 상가 뒤 골목이 시작하는 사거리 토막도 함께 넓힌다 (env.roundabout_majeon_two_lane).
    - mj_b1  상가 뒤 골목: 북서 모서리 막다른 길(SW_final_left, 117번길과 같은 가운데 1m 띠)을 그대로 이어
             완정로와 나란히 동남쪽으로 3번길(mj_X1)까지. 가운데 띠 + 홑 노란선도 그대로.
    - mj_s1  완정로3번길: 완정로(mj_T_N1)에서 완정로에 수직으로 내려와 상가 뒤 골목·45번안길·45번길과 만나고 원당대로(mj_T_S)까지.
    - mj_c   윗 7번길: 3번길(mj_X1)에서 동쪽으로 완정로 곡선부(mj_T_N2)까지.
    - mj_d   아래 7번길: 3번길(mj_X2)에서 동쪽으로 완정로(mj_T_N3)까지.
    - mj_e   완정로45번길: 65번길(mj_T_W2)에서 동쪽으로 3번길(mj_X2)까지 (원당대로와 나란히).
    - mj_w1  45번길 남북: 45번길(mj_J6)에서 북쪽 골목(mj_J8)까지 (65번길과 나란히).
    - mj_g   북쪽 골목: 65번길(mj_T_W1)에서 동쪽으로 상가 뒤 골목(mj_J9)까지.
    - mj_h   45번안길: 45번길(mj_J7)에서 우버니공원 남쪽을 지나 3번길(mj_T_H)까지.
    - mj_p1  완정로(mj_T_N0)에서 상가 뒤 골목(mj_J10)으로 내려오는 골목 (한방병원·보은교회 사이 길).
  큰길과 만나는 곳(T자)은 우회전 진입·진출만 둔다 (다른 안쪽 길과 같다). 원당대로 T자는 유턴(x≈1650)에서 60m 서쪽.
  길을 까는 방식은 env.roundabout_interior 와 같다 (설정만 바꿔 같은 코드로).

roundabout_south_extend 가 안쪽 도로 다음 단계로 호출한다.  단독 실행 불가(파이프라인으로).
"""
import math

PREFIX = 'mj_'


def _on(p, d, s):
    return (p[0] + d[0]*s, p[1] + d[1]*s)


# 상가 뒤 골목 방향 = 막다른 길 SW_final_left 방향 = 완정로 방향
_D = (0.857, -0.515)
_E0 = (1382.0, -271.0)                    # 막다른 길 끝 (근처, 실제 위치는 망에서)
_X1 = (1685.6, -453.2)                    # 3번길 × 상가 뒤 골목
_TS = (1591.4, -610.0)                    # 3번길 원당대로 T자
_S1 = ((_TS[0] - _X1[0]) / math.dist(_X1, _TS), (_TS[1] - _X1[1]) / math.dist(_X1, _TS))
_L1 = math.dist(_X1, _TS)
_TH = _on(_X1, _S1, 0.27*_L1)             # 45번안길이 들어오는 T
_X2 = _on(_X1, _S1, 0.60*_L1)             # 45번길·아래 7번길 네거리
_TW2 = (1277.0, -495.0)
_E = ((_X2[0] - _TW2[0]), (_X2[1] - _TW2[1]))


def _on_e(x):                              # 45번길 위 x 좌표의 점
    return (x, _TW2[1] + _E[1]*(x - _TW2[0])/_E[0])


def _on_b1(x):                             # 상가 뒤 골목 위 x 좌표의 점
    return (x, _X1[1] + _D[1]/_D[0]*(x - _X1[0]))


# 3번길·상가 뒤 골목에 붙는 골목은 끝 ARM m 를 거의 수직으로 들어오게 한다 (예각 교차 → 급한 회전 방지)
ARM = 26.0
_S1E = (-_S1[1], _S1[0])                   # 3번길 동쪽 법선
_S1W = (_S1[1], -_S1[0])                   # 3번길 서쪽 법선
_B1S = (-0.515, -0.857)                    # 상가 뒤 골목 남서쪽 법선
_J9 = _on_b1(1572.0)


TEES = [
    ('mj_T_W1', 'SW_final_forward_in', (1302.0, -380.0)),     # 65번길 (북행, 동쪽 차로) — 북쪽 골목
    ('mj_T_W2', 'SW_final_forward_in', _TW2),                 # 65번길 — 45번길
    ('mj_T_N0', 'S_conn_down', (1600.0, -198.0)),             # 완정로 (남동행, 남서쪽 차로) — 골목 p1
    ('mj_T_N1', 'S_conn_down', (1775.0, -302.0)),             # 완정로 — 3번길
    ('mj_T_N2', 'S_conn_down', (1929.0, -438.0)),             # 완정로 곡선부 — 윗 7번길
    ('mj_T_N3', 'S_conn_down', (1944.0, -505.0)),             # 완정로 곡선부 — 아래 7번길
    ('mj_T_S', 'SW_end_T_left_in', _TS),                      # 원당대로 (서행, 북쪽 차로) — 3번길
]
REBUILD = {'SW_final_left': [(('SW_final_left_out', 'SW_final_left_in'), 15.0, 'mj_B1M')]}
NODES = {
    'mj_X1': _X1, 'mj_T_H': _TH, 'mj_X2': _X2,
    'mj_J6': _on_e(1324.0), 'mj_J7': _on_e(1525.0),
    'mj_J8': (1357.0, -386.0), 'mj_J9': _J9, 'mj_J10': _on_b1(1510.0),
}
STREETS = [
    ('mj_b1', 'SW_final_left', 'mj_J10', 1, []),
    ('mj_b1b', 'mj_J10', 'mj_J9', 1, []),
    ('mj_b1c', 'mj_J9', 'mj_X1', 1, []),
    ('mj_s1', 'mj_T_N1', 'mj_X1', 1, []),
    ('mj_s1b', 'mj_X1', 'mj_T_H', 1, []),
    ('mj_s1c', 'mj_T_H', 'mj_X2', 1, []),
    ('mj_s1d', 'mj_X2', 'mj_T_S', 1, []),
    ('mj_c', 'mj_X1', 'mj_T_N2', 0, [_on(_X1, _S1E, ARM)]),
    ('mj_d', 'mj_X2', 'mj_T_N3', 0, [_on(_X2, _S1E, ARM)]),
    ('mj_e', 'mj_T_W2', 'mj_J6', 1, []),
    ('mj_eb', 'mj_J6', 'mj_J7', 1, []),
    ('mj_ec', 'mj_J7', 'mj_X2', 1, []),
    ('mj_w1', 'mj_J6', 'mj_J8', 0, []),
    ('mj_g', 'mj_T_W1', 'mj_J8', 0, []),
    ('mj_gb', 'mj_J8', 'mj_J9', 0, [(1505.0, -398.0), _on(_J9, _B1S, ARM)]),
    # 45번안길: 45번길에서 거의 수직으로 나와(좁은 쐐기 방지) 휘어 3번길에도 거의 수직으로
    ('mj_h', 'mj_J7', 'mj_T_H', 0, [(_on_e(1525.0)[0] + 22.0, _on_e(1525.0)[1] + 24.0), _on(_TH, _S1W, ARM)]),
    ('mj_p1', 'mj_T_N0', 'mj_J10', 0, []),
]
# 막다른 길 SW_final_left 를 잇는 상가 뒤 골목은 같은 단면 (가운데 1m 띠): 처음 조각만이 아니라 3번길까지 전부
MEDIAN = {'mj_b1': 1.0, 'mj_b1b': 1.0, 'mj_b1c': 1.0}
# 2026-11-14 사용자 요청: 마전동 골목 전부 편도 2차로 (불법 주정차 차를 옆 차로로 피해 가게)
LANES = {s[0]: 2 for s in STREETS}
CFG = dict(TEES=TEES, REBUILD=REBUILD, NODES=NODES, STREETS=STREETS, PREFIX=PREFIX, MEDIAN=MEDIAN,
           DOUBLE_CENTER=set(), APPROACH=30.0, LANES=LANES)


def apply_net(net_path):
    from env import roundabout_interior as it
    return {f'majeon_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(CFG, it.apply_net, net_path).items()}


def apply_deco(deco_path, net_path):
    from env import roundabout_interior as it
    return {f'majeon_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(CFG, it.apply_deco, deco_path, net_path).items()}
