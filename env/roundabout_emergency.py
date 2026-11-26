"""왕길119안전센터 긴급차 (2026-11-14 사용자 요청): 소방차·구급차가 주기적으로 마전동 쪽으로 출동.

  - 출발: 센터 안 소방차 전용 출구(nw_119, env.roundabout_nw_shops).
  - 경로 (사용자가 그린 빨간 선): 완정로 남동행(완정초·마전중 스쿨존 통과) → 탑유도관 앞 → 동쪽 곡선 길로 내려가
    → 남쪽 큰길 서행 → 서쪽 유턴(SW_long_uturn) → 남쪽 큰길 동행 → 동쪽 길 북행 → 북쪽 끝(지도 경계)에서 사라진다.
  - 주기: PERIOD_S 초마다 한 대, 소방차·구급차 번갈아 (각 차종은 2 × PERIOD_S 마다). FIRST_S 초에 첫 출동.
  - 사이렌(SUMO bluelight 장치)은 BLUELIGHT=False 로 끔: SUMO 1.27.1 에서 이 경로의 남쪽 큰길 동행
    SW_end_T_taper_out(4차로) → SW_end_T_extension_out(5차로) 에 들어가는 순간 SUMO 가 죽는다 (혼자 달려도,
    reactiondist 0 · speedFactor 없이도 같음, 2026-11-14 시험). 지금은 긴급차 차종·모양·빠른 속도(speedFactor)만.
  - 경로는 VIAS 를 거치는 최단 경로를 미리 계산해 엣지 목록으로 적는다 (출동 때마다 같은 길).
    주차장 통로(LOT_PREFIXES)는 경로 계산에서 뺀다 — 큰길 옆 주차장을 지름길로 들어가지 않게.
출력: env/sumo_roundabout/roundabout.emergency.rou.xml
roundabout_south_extend 파이프라인 끝에서 write 를 부른다.  단독 실행: python -m env.roundabout_emergency
"""
import os

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ROU = os.path.join(NET_DIR, 'roundabout.emergency.rou.xml')
PERIOD_S = 600               # 출동 간격 (초) — 바꾸려면 이 값만
FIRST_S = 60                 # 첫 출동 시각 (초)
END_S = 100000
START, END = 'nw_119', 'sg_ext_n_pd_2'
VIAS = ['X_se_out_sz', 'S_conn_down_mj_4', 'S_conn_down1', 'SW_end_T_left_in_far', 'SW_end_T_left_out_far',
        'sg_ext_n']
BLUELIGHT = False            # True 로 바꾸면 사이렌 장치 (위 SUMO 문제 참고)
TYPES = [
    # (id, 모양, 길이, 색)
    ('em_fire', 'firebrigade', 9.0, '220,20,20'),
    # 2026-11-17 사용자 요청: 구급차 제외, 소방차만
]


# 주차장 통로·진출입로 (최단 경로가 큰길 옆 주차장을 지름길로 쓰지 않게 뺀다, 2026-11-14)
LOT_PREFIXES = ('pm_', 'pn_', 'pk_', 'pw_', 'pd_', 'pl_', 'pf_', 'pj_', 'mz_k', 'skw_', 'skm_')


def _shortest(net, a, b, vclass='emergency'):
    """a → b 최단 경로 (연결 따라, 주차장 엣지 제외)."""
    import heapq
    src, dst = net.getEdge(a), net.getEdge(b)
    dist, prev, heap = {src.getID(): 0.0}, {}, [(0.0, src.getID())]
    while heap:
        d, eid = heapq.heappop(heap)
        if eid == dst.getID():
            break
        if d > dist.get(eid, 1e18):
            continue
        for nxt in net.getEdge(eid).getOutgoing():
            nid = nxt.getID()
            if nid.startswith(LOT_PREFIXES) or not nxt.allows(vclass):
                continue
            nd = d + nxt.getLength()
            if nd < dist.get(nid, 1e18):
                dist[nid], prev[nid] = nd, eid
                heapq.heappush(heap, (nd, nid))
    if dst.getID() not in dist:
        return None
    path = [dst.getID()]
    while path[-1] != src.getID():
        path.append(prev[path[-1]])
    return path[::-1]


def route_edges(net_path=os.path.join(NET_DIR, 'roundabout.net.xml')):
    import sumolib
    net = sumolib.net.readNet(net_path)
    wp = [START] + VIAS + [END]
    edges = [START]
    for a, b in zip(wp, wp[1:]):
        path = _shortest(net, a, b)
        if path is None:
            raise RuntimeError(f'긴급차 경로 없음: {a} -> {b}')
        edges += path[1:]
    bad = [e for e in edges if e.startswith(LOT_PREFIXES)]
    assert not bad, bad
    length = sum(net.getEdge(e).getLength() for e in edges)
    return edges, length


def write(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), path=ROU):
    edges, length = route_edges(net_path)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_emergency.py 가 생성. 왕길119안전센터 소방차·구급차 출동 (마전동 쪽으로, 끝에서 사라짐) -->',
             '<routes>']
    for tid, shape, L, color in TYPES:
        lines += [f'    <vType id="{tid}" vClass="emergency" guiShape="{shape}" length="{L}" width="2.5" color="{color}" '
                  f'accel="1.8" decel="4.5" maxSpeed="22.22" speedFactor="1.3" speedDev="0.05" minGap="2.5">',
                  f'        <param key="has.bluelight.device" value="{str(BLUELIGHT).lower()}"/>',
                  '    </vType>']
    lines.append(f'    <route id="em_route" edges="{" ".join(edges)}"/>')
    n = len(TYPES)
    for k, (tid, *_) in enumerate(TYPES):
        lines.append(f'    <flow id="{tid}_119" type="{tid}" route="em_route" begin="{FIRST_S + k*PERIOD_S}" end="{END_S}" '
                     f'period="{n*PERIOD_S}" departPos="0" departSpeed="0" departLane="best"/>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'emergency_route_m': round(length), 'emergency_edges': len(edges), 'emergency_period_s': PERIOD_S}


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(write())
