"""엄마 심부름 경로 (ego 학습 경로, 2026-11-14 사용자 확정 / 2026-11-15 학습용으로 저장소에 옮김).

  출발: 왕길유승한내들아파트 단지 주차장 → 단봉로 정문에서 바로 좌회전 → 회전교차로 서쪽 아래(W_in1) 진입
  들를 곳 (매번 주차장에 들어가 칸 옆 통로에 멈춘다):
    1. 큰손왕만두 (공영주차장)  2. 완정초등학교  3. 마전어린이집  4. 새마을금고  5. 동아아파트 (외할머니, 도착)
  구간마다 사용자가 그린 길(VIAS)을 거치는 최단 경로를 잇는다 (세션 스크립트 mom_route.py 와 같은 계산).

  build() → env/sumo_roundabout/mom_route.json:
    edges: 경로 엣지 (주차장 통로를 지나며 같은 엣지가 두 번 나올 수 있음 → 경로 순번으로 구분)
    depart: 출발 주차장 (parkingArea, 엣지, 위치)
    stops: 들를 곳마다 {name, pa, lane, start, end, route_index}  (route_index = 그 주차 구역 엣지의 경로 순번)
  맵을 다시 만들면 (roundabout_south_extend 마지막 단계) 다시 계산한다.
"""
import glob
import json
import math
import os
import xml.etree.ElementTree as ET

MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
OUT = os.path.join(MAP_DIR, 'mom_route.json')

# (이름, 주차 구역 접두어, 위치: 좌표 또는 POI 이름) — 위치에서 가장 가까운 그 접두어의 주차 구역에 선다
STOPS = [('출발 왕길유승한내들아파트', 'pu_', (-125, 16)),
         ('큰손왕만두 (공영주차장)', 'pk_', 'shopz_keunson'),
         ('완정초등학교', 'skw_', 'school_wanjeong'),
         ('마전어린이집', 'mz_k', 'mzd_name'),
         ('새마을금고', 'pn_', 'shopz_mj_saemaeul'),
         ('동아아파트 (외할머니)', 'pd_', (2178, -99))]
# 구간 k (STOPS[k] → STOPS[k+1]) 에서 거칠 길 (사용자가 그린 빨간 선)
VIAS = {0: ['pu_drv_r', 'DANBONG_infar_pu_1', 'W_in1', 'E_out0'],      # 정문 좌회전 → 회전교차로 서쪽 아래 진입
        3: ['S_conn_down_mj_4', 'S_conn_down1'],
        4: ['SW_end_T_left_out_far', 'SW_end_T_extension_out', 'sg_ext_n']}


def build(map_dir=MAP_DIR, out=OUT):
    import sumolib
    net = sumolib.net.readNet(os.path.join(map_dir, 'roundabout.net.xml'))
    deco = ET.parse(os.path.join(map_dir, 'roundabout.deco.xml')).getroot()
    pas = {}
    for pa in deco.iter('parkingArea'):
        spaces = [(float(s.get('x')), float(s.get('y'))) for s in pa.findall('space')]
        pas[pa.get('id')] = dict(lane=pa.get('lane'), edge=pa.get('lane').rsplit('_', 1)[0],
                                 start=float(pa.get('startPos', 0)), end=float(pa.get('endPos', 0)), spaces=spaces)
    pois = {}
    for f in glob.glob(os.path.join(map_dir, '*.add.xml')):
        for p in ET.parse(f).getroot().iter('poi'):
            pois[p.get('id')] = (float(p.get('x')), float(p.get('y')))
    chosen = []
    for name, pre, where in STOPS:
        q = pois[where] if isinstance(where, str) else where
        best = min((k for k in pas if k.startswith(pre) and pas[k]['spaces']),
                   key=lambda k: min(math.dist(p, q) for p in pas[k]['spaces']))
        chosen.append((name, best, list(q)))
    route, stop_idx, legs_m = [], [], []
    for k in range(len(chosen) - 1):
        a_edge, b_edge = pas[chosen[k][1]]['edge'], pas[chosen[k + 1][1]]['edge']
        wp = [a_edge] + VIAS.get(k, []) + [b_edge]
        leg = [a_edge]
        for a, b in zip(wp, wp[1:]):
            path, _ = net.getShortestPath(net.getEdge(a), net.getEdge(b), vClass='passenger')
            if path is None:
                raise RuntimeError(f'엄마 경로 없음: {chosen[k][0]} 구간 {a} -> {b}')
            leg += [e.getID() for e in path[1:]]
        route += leg if not route else leg[1:]
        stop_idx.append(len(route) - 1)
        legs_m.append(round(sum(net.getEdge(e).getLength() for e in leg[1:])))
    # 들를 곳마다: 그 주차장(접두어)의 주차 구역 중 이번 구간 경로가 지나는 통로 위에 있고 칸이 가장 많은 곳에 선다
    # (가장 가까운 구역이 칸 1~2개짜리 짧은 통로면 빈 칸이 없을 때가 많아서, 2026-11-15)
    stops_out = []
    prev = 0
    for (name, pa0, _), idx, m, (_, pre, _) in zip(chosen[1:], stop_idx, legs_m, STOPS[1:]):
        best = (len(pas[pa0]['spaces']), idx, pa0)
        for i in range(prev + 1, idx + 1):
            for k, v in pas.items():
                if k.startswith(pre) and v['edge'] == route[i] and len(v['spaces']) > best[0]:
                    best = (len(v['spaces']), i, k)
        _, i, pa = best
        stops_out.append(dict(name=name, pa=pa, lane=pas[pa]['lane'], start=pas[pa]['start'], end=pas[pa]['end'],
                              route_index=i, leg_m=m, spaces=len(pas[pa]['spaces'])))
        prev = idx
    # 2026-11-17: 정차 구역을 칸이 더 많은 통로로 옮긴 뒤, 그 통로를 기준으로 구간 경로를 다시 최단으로 잇는다
    #   (예전: 가장 가까운 구역까지 갔다가 되돌아 나오는 모양이 남아 큰손왕만두 주차장 안을 한 바퀴 돎, 928 m → 792 m)
    sel = [pas[chosen[0][1]]['edge']] + [st['lane'].rsplit('_', 1)[0] for st in stops_out]
    route2, idx2 = [], []
    for k in range(len(sel) - 1):
        wp = [sel[k]] + VIAS.get(k, []) + [sel[k + 1]]
        leg = [sel[k]]
        for a, b in zip(wp, wp[1:]):
            path, _ = net.getShortestPath(net.getEdge(a), net.getEdge(b), vClass='passenger')
            if path is None:
                raise RuntimeError(f'엄마 경로 없음 (다시 잇기): {a} -> {b}')
            leg += [e.getID() for e in path[1:]]
        route2 += leg if not route2 else leg[1:]
        idx2.append(len(route2) - 1)
        stops_out[k]['leg_m'] = round(sum(net.getEdge(e).getLength() for e in leg[1:]))
    for st, i in zip(stops_out, idx2):
        st['route_index'] = i
    route = route2
    p0 = pas[chosen[0][1]]
    data = dict(
        edges=route,
        depart=dict(name=chosen[0][0], pa=chosen[0][1], edge=p0['edge'], pos=round(p0['start'] + 0.5, 2)),
        stops=stops_out,
        length_m=round(sum(net.getEdge(e).getLength() for e in route)),
    )
    with open(out, 'w', encoding='utf-8') as fh:
        json.dump(data, fh, ensure_ascii=False, indent=1)
    return {'mom_route_edges': len(route), 'mom_route_m': data['length_m'], 'mom_stops': len(data['stops'])}


def load(path=OUT):
    if not os.path.exists(path):
        build()
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


if __name__ == '__main__':
    print(build())
