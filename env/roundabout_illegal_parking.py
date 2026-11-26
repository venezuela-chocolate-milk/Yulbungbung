"""불법 주정차 구간 — 중앙선 양쪽(양방향) 바깥 차로에 에피소드마다 무작위로 선다.

  2026-11-15 사용자 요청으로 구간을 엄마 경로가 지나는 빨간 표시 두 곳으로 옮김 (예전 아래 7번길 mj_d 는 없앰):
    ① 큰손왕만두 오른쪽 상가 골목 D1 (shop_d1a·shop_d1b, env.roundabout_shop_d1_two_lane 으로 편도 2차로)
    ② 새마을금고 위 윗 7번길 (mj_c, 편도 2차로)
  - 띄엄띄엄 (차 앞끝 사이 GAP m, 자리마다 SKIP 확률로 비움), 교차로 앞뒤는 엣지별 여유(EDGES 값)만큼 비운다.
  - 차로 위에 그대로 선다 (주차장 칸이 아니라 바깥 차로를 실제로 막는다) → 다른 차·ego 는 안쪽 차로로 피해 간다.
  - 비상등(양쪽 깜빡이)을 켠다.
  - 학습 환경(env.sumo_env, reset 때 준비 상태를 불러온 뒤)과 GUI(view_roundabout.py)가 spawn / tick 을 부른다.
    준비 상태(state pool)에는 들어가지 않는다 (불러올 때마다 새로 뿌린다).
"""
import numpy as np

# 엣지: (들어오는 교차로 뒤 비우는 길이, 나가는 교차로 앞 비우는 길이) m
EDGES = {
    'shop_d1a_f': (10.0, 14.0), 'shop_d1a_r': (10.0, 14.0),     # 상가 골목 서쪽 토막 (약 66 m)
    'shop_d1b_f': (10.0, 14.0), 'shop_d1b_r': (10.0, 14.0),     # 상가 골목 동쪽 토막 (약 66 m)
    'mj_c_f': (25.0, 30.0), 'mj_c_r': (25.0, 30.0),             # 윗 7번길 (약 235 m)
}
LANE = 0                                    # 바깥 차로 (편도 2차로의 오른쪽)
GAP = (16.0, 40.0)                          # 이웃한 두 차 앞끝 사이 (m)
SKIP = 0.3                                  # 자리마다 비울 확률 (대수가 매번 달라진다)
PREFIX = 'illegal_'
VTYPE = 'illegal_parked'
COLORS = [(235, 235, 230), (200, 200, 205), (110, 112, 118), (40, 42, 46), (150, 30, 35), (40, 70, 130)]

_pending = set()
try:
    from traci.constants import VEH_SIGNAL_BLINKER_EMERGENCY as _HAZARD
except ImportError:
    _HAZARD = 4


def spawn(traci, rng=None):
    """바깥 차로에 불법 주정차 차를 뿌린다. 넣은 차 수를 돌려준다."""
    rng = rng or np.random
    if VTYPE not in traci.vehicletype.getIDList():
        traci.vehicletype.copy('DEFAULT_VEHTYPE', VTYPE)
        traci.vehicletype.setLength(VTYPE, 4.6)
        traci.vehicletype.setVehicleClass(VTYPE, 'passenger')
        traci.vehicletype.setShapeClass(VTYPE, 'passenger/sedan')
    routes = set(traci.route.getIDList())
    n = 0
    for edge, (m_start, m_end) in EDGES.items():
        rid = f'{PREFIX}route_{edge}'
        if rid not in routes:
            traci.route.add(rid, [edge])
        L = traci.lane.getLength(f'{edge}_{LANE}')
        pos = m_start + 4.6 + rng.uniform(0.0, GAP[1] - GAP[0])
        k = 0
        while pos < L - m_end:
            if rng.uniform() >= SKIP:
                vid = f'{PREFIX}{edge}_{k}'
                p = round(pos, 1)
                traci.vehicle.add(vid, rid, typeID=VTYPE, departLane=str(LANE), departPos=f'{p:.1f}',
                                  departSpeed='0')
                # 정지 위치는 출발 위치보다 조금 앞 (같거나 뒤면 '경로 아래쪽이 아니다'로 거부된다)
                traci.vehicle.setStop(vid, edge, pos=p + 1.0, laneIndex=LANE, duration=1e7)
                traci.vehicle.setColor(vid, COLORS[int(rng.randint(len(COLORS)))] + (255,))
                _pending.add(vid)
                n += 1
            k += 1
            pos += rng.uniform(*GAP)
    return n


def tick(traci):
    """도로에 올라온 불법 주정차 차의 비상등을 켠다 (삽입이 늦어진 차도 올라오는 대로). 매 스텝 불러도 가볍다."""
    if not _pending:
        return
    on = set(traci.vehicle.getIDList())
    for vid in list(_pending):
        if vid in on:
            traci.vehicle.setSignals(vid, _HAZARD)
            _pending.discard(vid)


def clear():
    _pending.clear()
