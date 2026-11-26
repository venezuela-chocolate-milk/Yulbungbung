"""구간 안에서 들어오고 나가는 배경 교통 (2026-11-17, 사용자 요청: 구간마다 눈에 보이게 많이, 원형교차로는 그대로).

진입로 배율만 올리면 큰 도로 차들이 원형교차로에서 막혀 그 뒤 골목·직진 구간이 비어 있었다. 그래서 엄마 경로를
교차로 기준 구간으로 나누고, 구간마다 그 구간 시작 도로의 시작점(교차로를 막 지난 곳)에서 나와(옆길·건물 출입) 구간을 지나 다음 교차로에서
빠지는 흐름을 둔다 (양방향). 원형교차로 안·주차장 통로·짧은 조각(60 m 미만)은 만들지 않는다.

  구간 종류      기본 흐름 (파일 값 × 0.8 = 차로당 시간당)   목표 밀도 (차로 1 km 당)
  큰 도로        125 → 100 대/h                              18
  골목(30km/h)   75  → 60 대/h                               9
흐름 id: loc_<구간번호>[r]_<k> — traffic_tune.py 가 구간(loc_<구간번호>[r]) 단위로 배율을 정한다.

  python -m env.roundabout_local_traffic      → env/sumo_roundabout/roundabout.local.rou.xml + local_segments.json
"""
import json
import os

MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
OUT = os.path.join(MAP_DIR, 'roundabout.local.rou.xml')
SEG_OUT = os.path.join(MAP_DIR, 'local_segments.json')
LOTS = ('pu_', 'pk_', 'skw_', 'skm_', 'mz_k', 'pn_', 'pd_', 'pm_', 'pw_', 'pl_', 'pf_', 'pj_', 'school_')
MIN_SEG = 60.0
MAX_DETOUR = 150.0          # 구간 끝에서 출구까지 이 거리(m) 안에 닿는 출구만
BIG_RATE, ALLEY_RATE = 125.0, 75.0          # 차로당 시간당 (파일 값; ×0.8 이 기준)
TARGET = {'big': 18.0, 'alley': 9.0}
# 2026-11-18 사용자 요청: 유턴 앞뒤 직진(5·4차로)과 마지막 아파트 앞 직진(sg_ext_n)이 특히 비어 보임 → 목표 밀도 높게
PRIORITY_EDGES = {'SW_end_T_left_in_far', 'SW_end_T_left_in_far_pn_2', 'SW_end_T_left_out_far',
                  'SW_long_main_continue', 'sg_ext_n'}
PRIORITY_TARGET = 25.0


def _is_junction(node):
    """진짜 교차로(신호 또는 3방향 이상)인지. 그냥 도로가 잘린 곳·합류 조각은 아님."""
    if node.getType() == 'traffic_light':
        return True
    nb = {e.getFromNode().getID() for e in node.getIncoming()} | {e.getToNode().getID() for e in node.getOutgoing()}
    return len(nb) >= 3


def segments(net, route):
    segs, cur = [], []
    for e in route:
        E = net.getEdge(e)
        skip = e.startswith(LOTS) or e.startswith('ring') or E.getFunction() == 'internal'
        if skip:
            if cur:
                segs.append(cur)
            cur = []
            continue
        cur.append(e)
        if _is_junction(E.getToNode()):
            segs.append(cur)
            cur = []
    if cur:
        segs.append(cur)
    out = []
    for s in segs:
        L = sum(net.getEdge(e).getLength() for e in s)
        if L < MIN_SEG:
            continue
        v = max(net.getEdge(e).getSpeed() for e in s)
        out.append(dict(edges=s, length=round(L, 1), lanes=max(net.getEdge(e).getLaneNumber() for e in s),
                        kind='big' if (v >= 13.0 or max(net.getEdge(e).getLaneNumber() for e in s) >= 3) else 'alley'))
    return out


def _reverse(net, edges):
    rev = []
    for e in reversed(edges):
        E = net.getEdge(e)
        r = [x for x in E.getToNode().getOutgoing() if x.getToNode() == E.getFromNode() and not x.getID().startswith(':')]
        if not r:
            return None
        rev.append(r[0].getID())
    return rev


def _exits(net, last_edge):
    """구간 끝 교차로에서 나가는 도로 (주차장·원형교차로 안 제외). 방향마다 하나씩."""
    E = net.getEdge(last_edge)
    out = []
    for tgt in E.getOutgoing():
        tid = tgt.getID()
        if tid.startswith(LOTS) or tid.startswith('ring') or tid.startswith(':'):
            continue
        path, cost = net.getShortestPath(E, tgt, vClass='passenger')
        if path and cost - E.getLength() <= MAX_DETOUR:      # 다음 교차로에서 바로 빠지는 출구만 (멀리 돌지 않게)
            out.append(tid)
    # 2026-11-18: 출구가 주차장 진입로에서 잘린 도로 조각이면(예: sg_ext_n_pd_1 → 아파트 진입로 + sg_ext_n_pd_2)
    #   진입로 앞에서 사라지지 않게 진입로를 지나 이어지는 도로 조각까지 간다.
    return sorted({_past_driveway(net, x) for x in out})


def _side_sources(net, route, first):
    """구간으로 옆에서 들어오는 도로. 첫 도로 바로 앞 교차로에 없으면 AV 경로를 거슬러 올라가며(최대 3개 도로,
    주차장·원형교차로 전까지) 옆길이 있는 교차로를 찾는다."""
    if first not in route:
        return []
    i = route.index(first)
    for j in range(i, max(0, i - 3) - 1, -1):
        e = route[j]
        if j < i and (e.startswith(LOTS) or e.startswith('ring') or e.startswith(':')):
            break
        out = _side_in(net, route, e)
        if out:
            return out
    return []


def _side_in(net, route, first):
    E = net.getEdge(first)
    prev = route[route.index(first) - 1] if first in route and route.index(first) > 0 else None
    out = []
    for inc in E.getIncoming():
        iid = inc.getID()
        if iid == prev or iid.startswith(':') or iid.startswith(LOTS) or iid.startswith('ring'):
            continue
        if inc.getFromNode() == E.getToNode():          # 유턴으로 되돌아오는 맞은편 도로
            continue
        if inc.getLength() < 20.0:
            continue
        out.append(iid)
    return sorted(out)


def _past_driveway(net, edge_id):
    for _ in range(4):
        nxt = [x for x in net.getEdge(edge_id).getOutgoing() if not x.getID().startswith(':')]
        lots = [x for x in nxt if x.getID().startswith(LOTS)]
        road = [x for x in nxt if not x.getID().startswith(LOTS) and not x.getID().startswith('ring')]
        if not lots or len(road) != 1:
            break
        edge_id = road[0].getID()
    return edge_id


def build(map_dir=MAP_DIR):
    import sumolib
    net = sumolib.net.readNet(os.path.join(map_dir, 'roundabout.net.xml'))
    route = json.load(open(os.path.join(map_dir, 'mom_route.json')))['edges']
    segs = segments(net, route)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_local_traffic.py 가 생성. 구간 안에서 들어오고 나가는 배경 교통 (traffic_tune.py 가 구간별 배율) -->',
             '<routes>']
    info = []
    for i, s in enumerate(segs):
        for tag, edges in (('', s['edges']), ('r', _reverse(net, s['edges']))):
            if not edges:
                continue
            exits = _exits(net, edges[-1])
            if not exits:
                continue
            first = edges[0]
            lanes = net.getEdge(first).getLaneNumber()
            rate = (BIG_RATE if s['kind'] == 'big' else ALLEY_RATE) * lanes
            gid = f'loc_{i}{tag}'
            # 2026-11-18: AV 가 지나가는 방향(tag '')은 구간 첫 도로 시작점에서 차를 만들지 않는다 — 그곳이 바로 AV 가
            #   유턴·주차장 출구·교차로에서 들어가는 자리라, 계속 생겨나는 차에 막혀 진입·차로변경을 못 했다
            #   (강사 112판: 유턴 직후 SW_end_T_left_out_far 막힘/이탈 12~14건). 대신 그 교차로로 들어오는 다른 방향 도로에서
            #   출발해 교차로를 지나 구간으로 들어오게 한다 (옆길에서 꺾어 들어오는 차). 그런 도로가 없으면 예전처럼.
            # 골목(alley)은 옆길에서 꺾어 들어오게 하면 좁은 교차점이 엉켜 막힘 (강사 112판 0%, 마전동 mj_X1) → 큰 도로만
            srcs = _side_sources(net, route, first) if tag == '' and s['kind'] == 'big' else []
            pairs = [(src, x) for src in (srcs or [first]) for x in exits]
            for k, (src, x) in enumerate(pairs):
                via = f' via="{first}"' if src != first else ''
                lines.append(f'    <flow id="{gid}_{k}" type="car" from="{src}" to="{x}"{via} begin="0" end="100000" '
                             f'vehsPerHour="{rate / len(pairs):.2f}" departLane="random" departPos="base" '
                             f'departSpeed="max"/>')
            tgt = PRIORITY_TARGET if set(s['edges']) & PRIORITY_EDGES or set(edges) & PRIORITY_EDGES else TARGET[s['kind']]
            info.append(dict(group=gid, kind=s['kind'], edges=edges, length=s['length'], lanes=lanes,
                             target=tgt, exits=exits))
    lines.append('</routes>')
    open(os.path.join(map_dir, 'roundabout.local.rou.xml'), 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    json.dump(info, open(os.path.join(map_dir, 'local_segments.json'), 'w'), ensure_ascii=False, indent=1)
    return {'local_groups': len(info), 'big': sum(x['kind'] == 'big' for x in info),
            'alley': sum(x['kind'] == 'alley' for x in info)}


if __name__ == '__main__':
    print(build())
