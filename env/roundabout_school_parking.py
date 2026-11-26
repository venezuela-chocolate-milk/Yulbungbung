"""완정초·마전중 학교 주차장 (2026-11-12 사용자 요청): 정문으로 들어가자마자 주차장, 그 안쪽에 학교.

  - 정문(학교 길 끝, env.roundabout_school_access 의 school_*_gate)을 입구 겸 출구로 쓴다 (왕복 1차로 그대로).
  - 틀: 원점 = 정문, u = 학교 길 진행 방향(학교 안쪽), v = u 오른쪽.
  - 주차장: 정문에서 STEM m 들어간 곳에 가로 통로(앞), 양옆 세로 통로 두 개, 끝 가로 통로(뒤)로 된 왕복 고리 통로
    (차로 3.2m × 2, 15 km/h). 세로 통로 안쪽 두 줄은 등을 맞대고, 바깥 두 줄은 담장 쪽, 끝 통로 뒤에 한 줄.
  - 칸은 env.roundabout_parking_s 와 같은 규칙: 2.5 × 5.0m 직각, 칸마다 parkingArea <space>
    (SUMO 는 주차한 차 앞머리를 space 점에 둔다 → 칸 안쪽 끝 0.2m 앞 점), 교차점 회전 경로가 지나는 칸은 뺀다,
    SUMO 기본 주차장·rerouter 표시는 바닥 그림(층 200)으로 덮는다.
  - 학교 그림(env.roundabout_school_buildings 의 POI)은 주차장 뒤로 옮기고, 정문~학교 전체를 담장 두른 부지로 그린다.
  - 메인 도로(큰길)는 침범하지 않는다: 부지가 학교·주차장 길이 아닌 차로(+보도 2.5m)와 겹치면 멈춘다.
  - 학교 오가는 차(roundabout.school.rou.xml): 처음부터 서 있는 차 + 주차하러 오는 흐름, 주차 뒤 큰길(X_se_out_b)로.

roundabout_south_extend 파이프라인 끝(블록 S 주차장 뒤)에서 apply 를 부른다.  단독 실행: python -m env.roundabout_school_parking
"""
import math
import os
import random
import re
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, _offset_band, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, TURN_CLEAR, SPEED,
                                      ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE, CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.school.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.school.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'school_img')
STEM = 10.0                  # 정문 ~ 앞 가로 통로 가운데
SPLIT = 5.0                  # 정문 길을 정문에서 이만큼 앞에서 나눠 정문 교차점을 다시 만든다
C = ST_D + AH                # 세로 통로 가운데 |v| (안쪽 두 줄이 등을 맞댄다)
N_ROW = 14                   # 세로 통로 옆 한 줄 칸 수
GAP_SCHOOL = 4.0             # 주차장 끝 ~ 학교 부지(그림)
SITE_MARGIN = 3.0
SITE_FILL, FENCE = '214,210,200', '70,90,70'
L_SITE, L_SCHOOL = 198.5, 198.8
SCHOOLS = [   # (키, 접두사, 정문 노드, 학교 길 진입 엣지, 진출 엣지, 학교 POI)
    ('wanjeong', 'skw_', 'school_wanjeong_gate', 'school_wanjeong_in', 'school_wanjeong_out', 'school_wanjeong'),
    ('majeon', 'skm_', 'school_majeon_gate', 'school_majeon_in', 'school_majeon_out', 'school_majeon'),
]
ORIGINS = ['X_main_in', 'X_nw_in']
DEST = 'X_se_out_b'          # 학교 진출은 큰길 우회전만
INIT_OCC = 0.5
TARGET_OCC = 0.45
DURATIONS = [900, 1800, 3600, 5400]


def _frame(net, gate, in_edge):
    from env.roundabout_shop_zones import LocalFrame
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    lane = next(l for e in net.findall('edge') if e.get('id') == in_edge for l in e.findall('lane'))
    p = _rd(lane.get('shape'))
    return LocalFrame(J[gate], (p[-1][0] - p[-2][0], p[-1][1] - p[-2][1]))


def _school_size(poi_id):
    root = ET.parse(ADD).getroot()
    el = next(e for e in root if e.get('id') == poi_id)
    return float(el.get('width')), float(el.get('height'))


def layout(net, school):
    from shapely.geometry import box
    key, p, gate, in_e, out_e, poi = school
    fr = _frame(net, gate, in_e)
    u0, u1 = STEM, STEM + AH + N_ROW*ST_W + AH            # 앞·끝 가로 통로 가운데 u
    vo = C + AH + ST_D                                      # 바깥 줄 바깥 끝 |v|
    stalls = []                                             # (틀 사각형 u0, u1, v0, v1, 칸 안쪽 방향(틀))
    for side in (-1, 1):
        for vin, vout in ((C - AH - ST_D, C - AH), (C + AH, C + AH + ST_D)):     # 안쪽 줄, 바깥 줄 (|v|)
            into = (0.0, float(side)) if vin > C else (0.0, -float(side))
            for k in range(N_ROW):
                a = u0 + AH + k*ST_W
                v0, v1 = sorted((side*vin, side*vout))
                stalls.append(((a, a + ST_W, v0, v1), into))
    n_far = int((2*vo) // ST_W)
    v = -n_far*ST_W/2
    for k in range(n_far):                                  # 끝 통로 뒤 한 줄 (학교 쪽)
        stalls.append(((u1 + AH, u1 + AH + ST_D, v + k*ST_W, v + (k + 1)*ST_W), (1.0, 0.0)))
    lot_u = (u0 - AH - 1.0, u1 + AH + ST_D + 1.0)
    lot_v = vo + 1.0
    sw, sh = _school_size(poi)                              # 학교 그림 (폭 = v 방향, 깊이 = u 방향)
    school_u = lot_u[1] + GAP_SCHOOL + sh/2
    half = max(lot_v, sw/2) + SITE_MARGIN
    site = box(0.0, -half, school_u + sh/2 + SITE_MARGIN, half)
    nodes = {f'{p}nC': fr.P(u0, 0.0), f'{p}nL0': fr.P(u0, -C), f'{p}nR0': fr.P(u0, C),
             f'{p}nL1': fr.P(u1, -C), f'{p}nR1': fr.P(u1, C)}
    streets = [(f'{p}stem', gate, f'{p}nC', 1, []),
               (f'{p}nearL', f'{p}nC', f'{p}nL0', 0, []), (f'{p}nearR', f'{p}nC', f'{p}nR0', 0, []),
               (f'{p}left', f'{p}nL0', f'{p}nL1', 0, []), (f'{p}right', f'{p}nR0', f'{p}nR1', 0, []),
               (f'{p}far', f'{p}nL1', f'{p}nR1', 0, [])]
    return dict(key=key, p=p, gate=gate, in_e=in_e, out_e=out_e, poi=poi, fr=fr, u0=u0, u1=u1, stalls=stalls,
                lot_u=lot_u, lot_v=lot_v, site=site, school_u=school_u, nodes=nodes, streets=streets)


def check_main_roads(net, L):
    """부지가 메인 도로(학교·주차장 길이 아닌 차로 + 보도)와 겹치면 멈춘다."""
    from shapely.geometry import LineString, Polygon
    fr = L['fr']
    site = Polygon([fr.P(*q) for q in L['site'].exterior.coords])
    for e in net.findall('edge'):
        eid = e.get('id', '')
        if e.get('function') == 'internal' or eid.startswith('school') or eid.startswith(('skw_', 'skm_')):
            continue
        for l in e.findall('lane'):
            pts = _rd(l.get('shape'))
            if len(pts) >= 2 and LineString(pts).buffer(float(l.get('width', 3.2))/2 + 2.5).intersects(site):
                raise RuntimeError(f'{L["key"]} 학교 부지가 도로 {eid} 를 침범함')


def apply_net(net_path, L):
    from env import roundabout_interior as it
    cfg = dict(TEES=[], REBUILD={L['gate']: [((L['out_e'], L['in_e']), SPLIT, f'{L["p"]}cut')]}, NODES=L['nodes'],
               STREETS=L['streets'], PREFIX=L['p'], MEDIAN={}, DOUBLE_CENTER=set(), MIDS=[], ONEWAY=set(),
               EXTRA_CON=[], SPEED=SPEED)
    return it.with_cfg(cfg, it.apply_net, net_path)


def _world_stall(L, rect, into):
    fr = L['fr']
    a, b, c, d = rect
    corners = [fr.P(a, c), fr.P(b, c), fr.P(b, d), fr.P(a, d)]
    cen = fr.P((a + b)/2, (c + d)/2)
    dw = (fr.u[0]*into[0] + fr.v[0]*into[1], fr.u[1]*into[0] + fr.v[1]*into[1])
    return corners, cen, dw


def assign(net_path, L):
    """칸마다 통로 차로 정하기 (칸 입구가 그 차로 오른쪽에 있고 가장 가까운 차로) + 교차점 회전 경로가 지나는 칸 빼기."""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    net = ET.parse(net_path).getroot()
    p = L['p']
    lanes = {l.get('id'): _rd(l.get('shape')) for e in net.findall('edge')
             if e.get('id', '').startswith(p) and e.get('function') != 'internal' for l in e.findall('lane')}
    sweep = unary_union([LineString(_rd(l.get('shape'))).buffer(TURN_CLEAR) for e in net.findall('edge')
                         if e.get('function') == 'internal' and e.get('id', '').startswith((f':{p}', f':{L["gate"]}'))
                         for l in e.findall('lane') if len(_rd(l.get('shape'))) >= 2])
    out, dropped = [], 0
    for rect, into in L['stalls']:
        corners, cen, dw = _world_stall(L, rect, into)
        if Polygon(corners).intersection(sweep).area > 0.05:
            dropped += 1
            continue
        mouth = (cen[0] - dw[0]*ST_D/2, cen[1] - dw[1]*ST_D/2)
        best = None
        for lid, pts in lanes.items():
            g = LineString(pts)
            dist = g.distance(Point(mouth))
            s = g.project(Point(mouth))
            a, b = g.interpolate(max(0.0, s - 0.5)), g.interpolate(min(g.length, s + 0.5))
            t = (b.x - a.x, b.y - a.y)
            right = (cen[0] - a.x)*t[1] - (cen[1] - a.y)*t[0] > 0
            if right and dist < 3.0 and (best is None or dist < best[0]):
                best = (dist, lid[:-2])
        if best is None:
            dropped += 1
            continue
        out.append((rect, into, best[1]))
    L['placed'] = out
    return dropped


def apply_deco(deco_path, net_path, L, vtype='sk_car', name=None):
    from env.roundabout_builder import _stop_line, _read_lane_shapes  # noqa: F401 (같은 도구 모음)
    net = ET.parse(net_path).getroot()
    p = L['p']
    lanes = {l.get('id'): float(l.get('length')) for e in net.findall('edge') for l in e.findall('lane')}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for el in list(root):
        if el.get('id', '').startswith(p):
            root.remove(el)
    groups = {}
    for rect, into, edge in L['placed']:
        groups.setdefault(edge, []).append((rect, into))
    areas = {}
    for edge, items in sorted(groups.items()):
        ln = f'{edge}_0'
        pa = ET.SubElement(root, 'parkingArea', dict(
            id=f'{p}pa_{edge[len(p):]}', lane=ln, startPos='0.10', endPos=f'{lanes[ln] - 0.10:.2f}',
            roadsideCapacity='0', friendlyPos='true', name=name or f'{L["key"]} 학교 주차장'))
        for rect, into in items:
            _, (cx, cy), dw = _world_stall(L, rect, into)
            k = ST_D/2 - FRONT_GAP
            navi = math.degrees(math.atan2(dw[0], dw[1])) % 360
            ET.SubElement(pa, 'space', dict(x=f'{cx + dw[0]*k:.3f}', y=f'{cy + dw[1]*k:.3f}', width=f'{ST_W:.2f}',
                                             length=f'{ST_D:.2f}', angle=f'{(180.0 - navi) % 360:.2f}'))
        areas[pa.get('id')] = (edge, len(items))
    rr_edges = sorted(e.get('id') for e in net.findall('edge')
                      if e.get('id', '').startswith(p) and e.get('function') != 'internal')
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{p}reroute', edges=' '.join(rr_edges), vTypes=vtype))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


def draw(L, add_path=ADD):
    """학교 부지(담장) + 주차장 바닥·칸 선 + P 간판, 학교 그림을 주차장 뒤로. school.add.xml 의 접두사 줄은 다시 그린다."""
    from shapely.geometry import box
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    fr, p = L['fr'], L['p']
    W = lambda pts: [fr.P(*q) for q in pts]
    out = []
    s0, v0, s1, v1 = L['site'].bounds
    out.append(f'    <poly id="{p}site" type="school" color="{SITE_FILL}" fill="1" layer="{L_SITE}" '
               f'shape="{_wr(W(L["site"].exterior.coords))}"/>')
    gw = 4.5                                                  # 정문 열린 폭 절반
    fence = [(0.0, gw), (0.0, v1), (s1, v1), (s1, v0), (0.0, v0), (0.0, -gw)]
    out.append(f'    <poly id="{p}fence" type="school" color="{FENCE}" fill="0" lineWidth="0.5" layer="{L_SITE + 0.1}" '
               f'shape="{_wr(W(fence))}"/>')
    lot = box(L['lot_u'][0], -L['lot_v'], L['lot_u'][1], L['lot_v']).union(box(-0.5, -AH - 1.0, L['lot_u'][0] + 1, AH + 1.0))
    lot = lot.buffer(1.0, join_style=1).buffer(-1.0, join_style=1)
    shp = _wr(W(lot.exterior.coords))
    out.append(f'    <poly id="{p}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    out.append(f'    <poly id="{p}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
               f'layer="{L_LOT + 0.1}" shape="{shp}"/>')
    for n, (rect, into, _) in enumerate(L['placed']):
        a, b, c, d = rect
        if into[0]:                                           # 끝 줄: 통로가 u 작은 쪽
            pts = [(a, c), (b, c), (b, d), (a, d)]
        else:
            near, far = (c, d) if into[1] > 0 else (d, c)
            pts = [(a, near), (a, far), (b, far), (b, near)]
        out.append(f'    <poly id="{p}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr(W(pts))}"/>')
    img = sign_stack(['P 주차장'], (40, 90, 200), font_m=1.8)
    fn = f'{p}sign.png'
    img.save(os.path.join(IMG_DIR, fn))
    x, y = fr.P(-1.5, gw + 3.5)
    out.append(f'    <poi id="{p}sign" type="parking" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
               f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="school_img/{fn}"/>')
    with open(add_path, encoding='utf-8') as fh:
        lines = fh.read().split('\n')
    lines = [ln for ln in lines if not re.search(rf' id="{p}', ln)]
    # 학교 그림: 주차장 뒤로 (아래쪽 가운데 정문이 주차장을 본다), 부지 위 층
    cx, cy = fr.P(L['school_u'], 0.0)
    ang = math.degrees(math.atan2(fr.u[0], fr.u[1]))
    for i, ln in enumerate(lines):
        if f'id="{L["poi"]}"' in ln:
            ln = re.sub(r' x="[^"]+"', f' x="{cx:.3f}"', ln)
            ln = re.sub(r' y="[^"]+"', f' y="{cy:.3f}"', ln)
            ln = re.sub(r' angle="[^"]+"', f' angle="{ang:.2f}"', ln)
            lines[i] = re.sub(r' layer="[^"]+"', f' layer="{L_SCHOOL}"', ln)
    end = max(i for i, ln in enumerate(lines) if ln.strip() == '</additional>')
    lines[end:end] = out
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines))
    return len(out)


def write_routes(all_areas, path=ROU, seed=20261113):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 학교 오가는 차 (env/roundabout_school_parking.py). 정문 안 학교 주차장 칸(parkingArea space)에 주차했다가',
             '     큰길(X_se_out_b)로 나간다. 처음부터 서 있는 차는 departPos="stop". 찬 구역이면 주차장 안에서 빈 구역으로 돌린다. -->',
             '<routes>',
             f'    <vType id="sk_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in all_areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="sk_init_{n}" type="sk_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{DEST}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 5400)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(all_areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="sk_f_{pid}" type="sk_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DEST}" departLane="best" departSpeed="max" '
                     f'color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'school_init_cars': n, 'school_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    rep, all_areas = {}, {}
    for school in SCHOOLS:
        net = ET.parse(net_path).getroot()
        L = layout(net, school)
        check_main_roads(net, L)
        apply_net(net_path, L)
        root = ET.parse(net_path).getroot()
        link.update_boundary(root)
        ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
        rep[f'{L["key"]}_turn_dropped'] = assign(net_path, L)
        areas = apply_deco(deco_path, net_path, L)
        rep[f'{L["key"]}_spaces'] = sum(c for _, c in areas.values())
        draw(L)
        all_areas.update(areas)
    rep.update(write_routes(all_areas))
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
