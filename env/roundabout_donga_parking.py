"""동아아파트 주차장 (2026-11-13 사용자 요청): 동아아파트 아래쪽 동들 + 오른쪽 상가(태화프라자·가대지국밥) 자리를 아파트 주차장으로.

  - 자리: 마전동 북쪽 그림(env.roundabout_majeon_north_scenery)의 'mx_donga' 아래쪽 + 'mx_taehwa' 전체.
    위쪽 끝은 동아아파트 5번째 줄 동 바로 아래 (동 줄과 나란한 선), 아래는 곡선 도로(me_top2) 보도까지 (도로 남쪽은 그대로),
    서쪽은 서쪽 길(mx_s1d) 보도, 동쪽은 서곶로(sg_ext_s_mx) 보도까지.
  - 틀: u = 동아아파트 동 줄 방향(-3°, 동쪽), v = 북쪽(u 왼쪽). 바닥은 도로 따라 생긴 땅 그대로(아래 변은 곡선 도로를 따라 휜다).
  - 통로(왕복 3.2m × 2, 15 km/h): 남북 통로 여러 개 + 앞(곡선 도로 쪽)·뒤 가로 통로. 앞 가로 통로는 곡선 도로를 따라 꺾인다.
    칸은 env.roundabout_wanggil_parking.place_stalls 그대로 (통로 양옆 직각 칸, 회전 경로 칸 빼기, 앞머리 점).
  - 입구·출구: 모두 동쪽 변 서곶로(남행 sg_ext_s_mx · 북행 sg_ext_n)에 (2026-11-13 사용자 요청으로 곡선 도로·서쪽 길에서 옮김).
    맨 동쪽 통로에서 짧은 진출입로. 입구(북쪽)는 남행 바깥 차로에서 우회전 + 북행 중앙선 바로 오른쪽 차로(1차로)에서
    좌회전 진입 (그 자리 노란 중앙선을 끊는다). 출구(남쪽)는 남행 바깥 차로로 우회전 진출만 (정지선·우회전 화살표).
  - 그림: roundabout.donga_parking.add.xml (주차장 바닥·칸 선·간판 — 아파트 주차장). 그 자리의 동아아파트 동·상가 그림은
    env.roundabout_majeon_north_scenery 가 이 파일을 보고 뺀다.

roundabout_south_extend 파이프라인 끝에서 apply 를 부른다.  단독 실행: python -m env.roundabout_donga_parking
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, SPEED,
                                      ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE, CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.donga_parking.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.donga_parking.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pd_'
NAME = '동아아파트 주차장'
SIDEWALK = 3.0
ANGLE = -3.0                  # 동아아파트 동 줄 방향 (env.roundabout_majeon_north_scenery 'mx_donga' angle)
NORTH_CUT = ((2085.0, -37.0), (2310.0, -48.8))   # 5번째 줄 동(아래 끝 y -33 ~ -38) 바로 아래, 동 줄과 나란히
HINT_S = -260.0               # 아래로는 곡선 도로 보도에서 잘린다
SEED_PT = (2170.0, -100.0)    # 주차장 땅 조각 고르기
EDGE_GAP = 0.5 + ST_D + AH    # 바닥 가장자리 ~ 통로 중심 (칸 한 줄 + 여유)
PITCH = 2*AH + 2*ST_D         # 통로 간격 (칸 두 줄 등 맞댐)
IN_FRAC, OUT_FRAC = 0.65, 0.33   # 입구·출구가 붙는 맨 동쪽 통로 위치 (앞 ~ 뒤 비율)
SOUTH, NORTH = 'sg_ext_s_mx', 'sg_ext_n'   # 서곶로 남행(주차장 쪽) · 북행
ORIGINS = ['sg_ext_s', 'sg_ext_n']
DESTS = ['sg_ext_s_me_1', 'me_top2_r']
INIT_OCC, TARGET_OCC = 0.5, 0.4
DURATIONS = [1200, 2400, 3600, 4800]
EXIT_ARROW_DIST = 3.4
LEFT_ARROW_DIST = 6.0        # 좌회전 진입 차로 정지선에서 화살표까지
CENTER_GAP = 6.0              # 진출입 교차점 양쪽으로 노란 중앙선을 비우는 길이
CENTERS = (('sg_ext_center_sg_ext_n', 'mid_in'),)   # 좌회전 진입 자리만


class Frame:
    def __init__(self, o, u):
        L = math.hypot(*u)
        self.o, self.u = o, (u[0]/L, u[1]/L)
        self.v = (-self.u[1], self.u[0])          # u 왼쪽 = 북쪽

    def P(self, s, t):
        return (self.o[0] + self.u[0]*s + self.v[0]*t, self.o[1] + self.u[1]*s + self.v[1]*t)

    def st(self, p):
        dx, dy = p[0] - self.o[0], p[1] - self.o[1]
        return (dx*self.u[0] + dy*self.u[1], dx*self.v[0] + dy*self.v[1])


def _keep(net):
    """도로(차로 + 보도 3m) — 주차장 바닥이 겹치면 안 되는 땅 (pd_ 길은 뺀다)."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    keep = []
    for e in net.findall('edge'):
        if e.get('id', '').startswith((P_, f':{P_}')):
            continue
        for l in e.findall('lane'):
            p = _rd(l.get('shape'))
            if len(p) >= 2:
                keep.append(LineString(p).buffer(float(l.get('width', 3.2))/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and not j.get('id', '').startswith(P_) and len(_rd(s)) >= 3:
            keep.append(Polygon(_rd(s)).buffer(SIDEWALK))
    return unary_union(keep)


def layout(net):
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import transform
    keep = _keep(net)
    (xa, ya), (xb, yb) = NORTH_CUT
    hint = Polygon([(xa, ya), (xb, yb), (xb, HINT_S), (xa, HINT_S)])
    land = hint.difference(keep)
    land = next(g for g in getattr(land, 'geoms', [land]) if g.buffer(0.01).contains(Point(SEED_PT)))
    a = math.radians(ANGLE)
    fr0 = Frame(land.centroid.coords[0], (math.cos(a), math.sin(a)))
    to_st = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr0.st(p) for p in zip(x, y)]))), g)
    L_st = to_st(land)
    u_min, v_min, u_max, v_max = L_st.bounds
    fr = Frame(fr0.P(u_min, v_min), fr0.u)
    L_st = transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.st(p) for p in zip(x, y)]))), land)  # u, v ≥ 0
    _, _, W, D = L_st.bounds

    def v_span(u0, u1):
        """u0 ~ u1 띠에서 땅이 빈틈없이 차 있는 v 범위 (아래 끝 최댓값, 위 끝 최솟값)."""
        lo, hi = -1e9, 1e9
        for k in range(9):
            u = u0 + (u1 - u0)*k/8
            seg = L_st.intersection(LineString([(u, -10.0), (u, D + 10.0)]))
            parts = [g for g in getattr(seg, 'geoms', [seg]) if g.length > 0]
            if not parts:
                return None
            g = max(parts, key=lambda q: q.length)
            ys = [c[1] for c in g.coords]
            lo, hi = max(lo, min(ys)), min(hi, max(ys))
        return lo, hi

    # 통로 수: 가장 좁은 높이에서도 바깥 칸 줄까지 들어가게 (서쪽 끝 ~ 동쪽 끝)
    # 서쪽·동쪽 끝은 아래·위 모두에서 땅이 차 있는 u 범위
    us_ok = [u*0.5 for u in range(int(W*2) + 1) if v_span(u*0.5, u*0.5) is not None
             and (lambda s: s[1] - s[0] > 2*EDGE_GAP + 10)(v_span(u*0.5, u*0.5))]
    ua, ub = min(us_ok), max(us_ok)
    # 동쪽 변이 비스듬하므로 동쪽 끝은 앞(아래) 쪽 기준으로 줄인다
    while ub > ua and not L_st.buffer(0.01).contains(Point(ub, v_span(ub, ub)[0] + EDGE_GAP)):
        ub -= 0.5
    n = int((ub - ua - 2*EDGE_GAP)//PITCH) + 1
    first = ua + EDGE_GAP + ((ub - ua - 2*EDGE_GAP) - (n - 1)*PITCH)/2
    us = [first + k*PITCH for k in range(n)]
    lot_u0, lot_u1 = us[0] - EDGE_GAP, us[-1] + EDGE_GAP
    N = {}
    for k, u in enumerate(us):
        lo, hi = v_span(u - AH - ST_D, u + AH + ST_D)
        N[f'A{k}F'], N[f'A{k}B'] = (u, lo + EDGE_GAP), (u, hi - EDGE_GAP)
    e = n - 1                                   # 맨 동쪽 통로
    vf, vb = N[f'A{e}F'][1], N[f'A{e}B'][1]
    N['IN'], N['OUT'] = (us[e], vf + (vb - vf)*IN_FRAC), (us[e], vf + (vb - vf)*OUT_FRAC)
    streets = [(f'a{k}', f'A{k}F', f'A{k}B') for k in range(e)]
    streets += [(f'a{e}s', f'A{e}F', 'OUT'), (f'a{e}m', 'OUT', 'IN'), (f'a{e}n', 'IN', f'A{e}B')]
    for k in range(n - 1):
        streets.append((f'f{k}', f'A{k}F', f'A{k + 1}F'))
        streets.append((f'b{k}', f'A{k}B', f'A{k + 1}B'))
    # 바닥: 땅 중 칸이 있는 u 범위 (서쪽·동쪽 남는 띠는 아파트 바닥 그대로)
    lot_st = L_st.intersection(box(lot_u0, -10.0, lot_u1, D + 10.0))
    lot_st = max(getattr(lot_st, 'geoms', [lot_st]), key=lambda g: g.area)
    lot = Polygon([fr.P(*c) for c in lot_st.exterior.coords])
    return dict(fr=fr, R=lot, N=N, streets=streets, n_aisle=n, turn_clear=2.0, size=(round(lot_u1 - lot_u0, 1),
                                                                                       round(lot.area)))


def cfg(net, L):
    fr, N = L['fr'], L['N']
    nodes = {f'{P_}n{k}': fr.P(*v) for k, v in N.items()}
    streets = [(f'{P_}{nm}', f'{P_}n{a}', f'{P_}n{b}', 1 if nm.startswith('a') else 0, []) for nm, a, b in L['streets']]
    streets += [(f'{P_}drv_in', f'{P_}mid_in', f'{P_}nIN', 0, []), (f'{P_}drv_out', f'{P_}nOUT', f'{P_}mid_out', 0, [])]
    mids = [(f'{P_}mid_in', (SOUTH, NORTH), fr.P(N['IN'][0] + 30.0, N['IN'][1])),
            (f'{P_}mid_out', (SOUTH, NORTH), fr.P(N['OUT'][0] + 30.0, N['OUT'][1]))]
    n_north = next(len(e.findall('lane')) for e in net.findall('edge') if e.get('id') == NORTH)
    # 남행: 입구가 먼저(북쪽), 출구가 뒤 → 조각 SOUTH, SOUTH_pd1(입구~출구), SOUTH_pd2
    # 북행: 출구 자리가 먼저(남쪽), 입구가 뒤 → 조각 NORTH, NORTH_pd1(출구~입구), NORTH_pd2
    extra = [(SOUTH, f'{P_}drv_in_f', 0, 0),                                   # 우회전 진입 (남행 바깥 차로)
             (f'{NORTH}_{P_}1', f'{P_}drv_in_f', n_north - 1, 0),              # 좌회전 진입 (북행 중앙선 쪽 차로)
             (f'{P_}drv_out_f', f'{SOUTH}_{P_}2', 0, 0)]                       # 우회전 진출 (남행 바깥 차로)
    return dict(TEES=[], REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=P_, MEDIAN={}, DOUBLE_CENTER=set(),
                MIDS=mids, ONEWAY={f'{P_}drv_in', f'{P_}drv_out'}, EXTRA_CON=extra, SPEED=SPEED)


def place_stalls(net_path, L):
    """env.roundabout_wanggil_parking.place_stalls 를 이 주차장 접두사로 (같은 규칙)."""
    from env import roundabout_wanggil_parking as wp
    old = wp.P_
    wp.P_ = P_
    try:
        return wp.place_stalls(net_path, L)
    finally:
        wp.P_ = old


def _gap_centers(root, net):
    from shapely.geometry import LineString, Point
    from shapely.ops import substring
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    polys = {p.get('id'): p for p in root.findall('poly')}
    for cid, mid in CENTERS:
        if cid not in polys or f'{cid}_b' in polys:
            continue
        p = polys[cid]
        g = LineString(_rd(p.get('shape')))
        s = g.project(Point(J[f'{P_}{mid}']))
        a, b = substring(g, 0, s - CENTER_GAP), substring(g, s + CENTER_GAP, g.length)
        p.set('shape', ' '.join(f'{x:.6f},{y:.6f}' for x, y in a.coords))
        q = ET.SubElement(root, 'poly', dict(p.attrib))
        q.set('id', f'{cid}_b')
        q.set('shape', ' '.join(f'{x:.6f},{y:.6f}' for x, y in b.coords))


def apply_deco(deco_path, net_path, L):
    from env.roundabout_builder import _stop_line, _read_lane_shapes, _arrow
    net = ET.parse(net_path).getroot()
    lanes = {l.get('id'): float(l.get('length')) for e in net.findall('edge') for l in e.findall('lane')}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for el in list(root):
        if el.get('id', '').startswith(P_):
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
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{P_}reroute', edges=' '.join(rr_edges), vTypes='pd_car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    marks = []
    shapes = _read_lane_shapes(net_path)
    _stop_line(marks, f'{P_}stop_out', shapes, f'{P_}drv_out_f', dist=1.0)
    _arrow(marks, f'{P_}arrow_out_right', shapes, f'{P_}drv_out_f', 0, 'right', EXIT_ARROW_DIST)
    # 2026-11-13: 북행 좌회전 진입 차로(중앙선 바로 오른쪽 차로)에만 정지선 + 직진·좌회전 화살표
    nl = next(len(e.findall('lane')) for e in net.findall('edge') if e.get('id') == f'{NORTH}_{P_}1')
    _stop_line(marks, f'{P_}stop_in_left', shapes, f'{NORTH}_{P_}1', lanes=[nl - 1], dist=1.0)
    _arrow(marks, f'{P_}arrow_in_left', shapes, f'{NORTH}_{P_}1', nl - 1, 'straight_left', LEFT_ARROW_DIST)
    for m in marks:
        root.append(ET.fromstring(m))
    _gap_centers(root, net)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


def draw(L, add_path=ADD):
    """주차장 바닥(연석) + 칸 선 + 간판 (아파트 주차장)."""
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_donga_parking.py 가 생성. 동아아파트 주차장 (화면 표시 전용) -->', '<additional>']
    shp = _wr(L['R'].exterior.coords)
    out.append(f'    <poly id="{P_}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    out.append(f'    <poly id="{P_}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" layer="{L_LOT + 0.1}" '
               f'shape="{shp}"/>')
    for n, (corners, cen, nv, _) in enumerate(L['placed']):
        a, b, c, d = corners
        out.append(f'    <poly id="{P_}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
    fr, N = L['fr'], L['N']
    iu, iv = N['IN']
    ou, ov = N['OUT']
    su = iu + EDGE_GAP - 2.5                    # 동쪽 가장자리 안쪽
    for key, names, color, (u, v) in [('sign', ['P 동아아파트 주차장'], (40, 90, 200), (su - 8.0, (iv + ov)/2)),
                                      ('in', ['입구'], (30, 140, 70), (su, iv + 6.0)),
                                      ('out', ['출구'], (200, 60, 40), (su, ov - 6.0))]:
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


def write_routes(areas, path=ROU, seed=20261113):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 동아아파트 주차장 차 (env/roundabout_donga_parking.py): 서곶로에서 우회전·좌회전으로 들어와 주차했다가 우회전으로 나간다. -->',
             '<routes>',
             f'    <vType id="pd_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="pd_init_{n}" type="pd_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="pd_f_{pid}" type="pd_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[k % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'donga_init_cars': n, 'donga_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_majeon_north_scenery as mns
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('동아아파트 주차장이 이미 적용됨')
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'donga_lot_{k}': v for k, v in rep.items()}
    rep['donga_size'] = L['size']
    rep['donga_aisles'] = L['n_aisle']
    rep['donga_stalls'] = place_stalls(net_path, L)
    areas = apply_deco(deco_path, net_path, L)
    draw(L)
    rep.update(write_routes(areas))
    rep['donga_redraw'] = {k: v for k, v in mns.build(net_path).items() if k in ('mx_donga', 'mx_taehwa')}
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
