"""마전동 ② 블록 공영주차장 (2026-11-12 사용자 요청): 오른쪽 두 띠(예전 맘스터치·메가MGC커피)를 통째로 공영주차장.

  - 땅: env.roundabout_majeon_buildings.reserved_region (② 블록 'reserved' 띠 두 개, 완정로 ~ 상가 뒤 골목, 거의 직사각형).
  - 입구·출구: 둘 다 완정로(S_conn_down_mj_1, 남동행) 쪽. 완정로는 중앙선이 있어 다른 골목처럼 우회전 진입·진출만:
    입구는 앞(북서) 쪽, 출구는 뒤(남동) 쪽 — 한 방향 진출입로 (정지선은 출구).
  - 통로: 완정로에 수직인 남북 통로 4개 + 앞(완정로 쪽)·뒤 가로 통로, 왕복 3.2m × 2, 15 km/h.
    칸은 env.roundabout_wanggil_parking.place_stalls 그대로 (통로 양옆 직각 칸, 회전 경로 칸 빼기, 앞머리 점).
  - 그림: roundabout.majeon_parking2.add.xml (바닥·칸 선·P 공영주차장·입구·출구 간판), SUMO 표시는 바닥(층 200)으로 덮는다.

roundabout_south_extend 파이프라인 끝에서 apply 를 부른다.  단독 실행: python -m env.roundabout_majeon_parking2
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, SPEED, ASPHALT, CURB_COLOR,
                                      L_LOT, L_LINE, LINE, CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.majeon_parking2.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.majeon_parking2.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pm_'
NAME = '마전동 공영주차장'
MAIN = 'S_conn_down_mj_1'    # 완정로 남동행 (블록 쪽 차로)
N_AISLE = 4
IN_GAP, OUT_GAP = (0, 1), (2, 3)     # 입구·출구가 붙는 앞 가로 통로 자리 (이 두 통로 사이 가운데)
ORIGINS = ['S_conn_down', 'SE_triangle_forward_out']
DESTS = ['S_conn_down_mj_2', 'mj_s1_f']
INIT_OCC, TARGET_OCC = 0.5, 0.4
DURATIONS = [1200, 2400, 3600, 4800]


def layout(net):
    from shapely.geometry import Polygon
    from shapely.ops import transform
    from env.roundabout_shop_zones import LocalFrame
    from env.roundabout_nw_buildings import _max_rect
    from env.roundabout_majeon_buildings import reserved_region, WJ
    reg, _ = reserved_region(net, 2, skip_prefix=P_)
    # 틀: s = 완정로 남동행 방향, t = 블록 안쪽 (s 오른쪽 = 완정로에서 멀어지는 쪽)
    fr0 = LocalFrame(reg.centroid.coords[0], WJ)
    to_st = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr0.st(p) for p in zip(x, y)]))), g)
    s0, s1, t0, t1 = _max_rect(to_st(reg).buffer(-0.3))
    fr = LocalFrame(fr0.P(s0, t0), WJ)
    W, D = s1 - s0, t1 - t0
    rect = Polygon([fr.P(0, 0), fr.P(W, 0), fr.P(W, D), fr.P(0, D)])
    span = (N_AISLE - 1)*(2*AH + 2*ST_D)
    first = (W - span)/2
    us = [first + k*(2*AH + 2*ST_D) for k in range(N_AISLE)]
    vf, vb = 0.5 + ST_D + AH, D - (0.5 + ST_D + AH)
    N = {}
    for k, u in enumerate(us):
        N[f'A{k}F'], N[f'A{k}B'] = (u, vf), (u, vb)
    N['IN'] = ((us[IN_GAP[0]] + us[IN_GAP[1]])/2, vf)
    N['OUT'] = ((us[OUT_GAP[0]] + us[OUT_GAP[1]])/2, vf)
    streets = [(f'a{k}', f'A{k}F', f'A{k}B') for k in range(N_AISLE)]
    for k in range(N_AISLE - 1):
        if (k, k + 1) == IN_GAP:
            streets += [(f'f{k}a', f'A{k}F', 'IN'), (f'f{k}b', 'IN', f'A{k + 1}F')]
        elif (k, k + 1) == OUT_GAP:
            streets += [(f'f{k}a', f'A{k}F', 'OUT'), (f'f{k}b', 'OUT', f'A{k + 1}F')]
        else:
            streets.append((f'f{k}', f'A{k}F', f'A{k + 1}F'))
        streets.append((f'b{k}', f'A{k}B', f'A{k + 1}B'))
    # 회전 경로 여유: 진입로에서 앞 가로 통로로 꺾는 차가 1.4m 보다 크게 돌아 2.0m (2026-11-12 겹침 시험)
    return dict(fr=fr, R=rect, N=N, streets=streets, size=(round(W, 1), round(D, 1)), turn_clear=2.0)


def cfg(net, L):
    fr, N = L['fr'], L['N']
    nodes = {f'{P_}n{k}': fr.P(*v) for k, v in N.items()}
    streets = [(f'{P_}{nm}', f'{P_}n{a}', f'{P_}n{b}', 1 if nm.startswith('a') else 0, []) for nm, a, b in L['streets']]
    streets += [(f'{P_}drv_in', f'{P_}T_in', f'{P_}nIN', 0, []), (f'{P_}drv_out', f'{P_}nOUT', f'{P_}T_out', 0, [])]
    tees = [(f'{P_}T_in', MAIN, fr.P(N['IN'][0], -6.0)), (f'{P_}T_out', MAIN, fr.P(N['OUT'][0], -6.0))]
    return dict(TEES=tees, REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=P_, MEDIAN={}, DOUBLE_CENTER=set(),
                MIDS=[], ONEWAY={f'{P_}drv_in', f'{P_}drv_out'}, EXTRA_CON=[], SPEED=SPEED, APPROACH=6.0)


def place_stalls(net_path, L):
    """env.roundabout_wanggil_parking.place_stalls 를 이 주차장 접두사로 (같은 규칙)."""
    from env import roundabout_wanggil_parking as wp
    old = wp.P_
    wp.P_ = P_
    try:
        return wp.place_stalls(net_path, L)
    finally:
        wp.P_ = old


def apply_deco(deco_path, net_path, L):
    from env.roundabout_builder import _stop_line, _read_lane_shapes
    net = ET.parse(net_path).getroot()
    lanes = {l.get('id'): float(l.get('length')) for e in net.findall('edge') for l in e.findall('lane')}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for el in list(root):
        if el.get('id', '').startswith(P_) and el.tag != 'poly' or el.get('id', '').startswith(f'{P_}stop'):
            root.remove(el)
    groups = {}
    for corners, cen, n, edge in L['placed']:
        groups.setdefault(edge, []).append((cen, n))
    areas = {}
    for edge, items in sorted(groups.items()):
        ln = f'{edge}_0'
        pa = ET.SubElement(root, 'parkingArea', dict(id=f'{P_}pa_{edge[len(P_):]}', lane=ln, startPos='0.10',
                                                     endPos=f'{lanes[ln] - 0.10:.2f}', roadsideCapacity='0',
                                                     friendlyPos='true', name=NAME))
        for cen, n in items:
            k = ST_D/2 - FRONT_GAP
            navi = math.degrees(math.atan2(n[0], n[1])) % 360
            ET.SubElement(pa, 'space', dict(x=f'{cen[0] + n[0]*k:.3f}', y=f'{cen[1] + n[1]*k:.3f}', width=f'{ST_W:.2f}',
                                             length=f'{ST_D:.2f}', angle=f'{(180.0 - navi) % 360:.2f}'))
        areas[pa.get('id')] = (edge, len(items))
    rr_edges = sorted(e.get('id') for e in net.findall('edge') if e.get('id', '').startswith(P_)
                      and e.get('function') != 'internal' and 'drv' not in e.get('id'))
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{P_}reroute', edges=' '.join(rr_edges), vTypes='pm_car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    marks = []
    _stop_line(marks, f'{P_}stop_out', _read_lane_shapes(net_path), f'{P_}drv_out_f', dist=1.0)
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


def draw(L, add_path=ADD):
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_majeon_parking2.py 가 생성. 마전동 ② 블록 공영주차장 (화면 표시 전용) -->', '<additional>']
    shp = _wr(L['R'].exterior.coords)
    out.append(f'    <poly id="{P_}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    out.append(f'    <poly id="{P_}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" layer="{L_LOT + 0.1}" '
               f'shape="{shp}"/>')
    for n, (corners, cen, nv, _) in enumerate(L['placed']):
        a, b, c, d = corners
        out.append(f'    <poly id="{P_}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
    fr, N = L['fr'], L['N']
    for key, names, color, (u, v) in [('sign', ['P 공영주차장'], (40, 90, 200), ((N['IN'][0] + N['OUT'][0])/2, -1.6)),
                                      ('in', ['입구'], (30, 140, 70), (N['IN'][0] - 7.0, -1.6)),
                                      ('out', ['출구'], (200, 60, 40), (N['OUT'][0] + 7.0, -1.6))]:
        img = sign_stack(names, color, font_m=1.8)
        fn = f'{P_}{key}.png'
        img.save(os.path.join(IMG_DIR, fn))
        x, y = fr.P(u, v)
        out.append(f'    <poi id="{P_}{key}" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
                   f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')
    return len(out)


def write_routes(areas, path=ROU, seed=20261116):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 마전동 ② 블록 공영주차장 차 (env/roundabout_majeon_parking2.py): 완정로에서 우회전으로 들어와 주차했다가 우회전으로 나간다. -->',
             '<routes>',
             f'    <vType id="pm_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="pm_init_{n}" type="pm_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="pm_f_{pid}" type="pm_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[k % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'majeon2_init_cars': n, 'majeon2_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('마전동 ② 블록 주차장이 이미 적용됨')
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'majeon2_lot_{k}': v for k, v in rep.items()}
    rep['majeon2_size'] = L['size']
    rep['majeon2_stalls'] = place_stalls(net_path, L)
    areas = apply_deco(deco_path, net_path, L)
    draw(L)
    rep.update(write_routes(areas))
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
