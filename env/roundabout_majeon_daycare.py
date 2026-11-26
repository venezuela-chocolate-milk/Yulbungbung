"""마전동 완정로 북쪽 상가 줄 + 마전어린이집 (2026-11-12 사용자 요청, 네이버 위성 지도 캡처 이식).

  자리: 마전영남탑스빌 블록(env.roundabout_majeon_north_scenery 'mx_yeongnam')의 아래쪽 —
  완정로(S_conn_up_mx)·마전역 북쪽 길·mx_c 로 둘러싸인 땅 중 사용자가 표시한 띠(STRIP).
  틀: 원점 = mx_T_C(완정로 × mx_c), u = 완정로 북서행 방향, v = 블록 안쪽 (북동).
  네이버 픽셀 → 세계: 표시 원과 완정로가 만나는 두 끝을 맞춘 닮음 변환 (NAVER_ANCHORS).

  - 길 (왕복 1차로 3.2m, 30 km/h): 뒷길 S1(최강마트 뒤, v=S1_V)이 mx_c(가운데 교차점)에서 서쪽으로,
    완정로에서 올라오는 골목 둘(X1: 최강마트·미니공방 사이, X2: 설빙·한빛철물마트 사이, 우회전 진입·진출 T자)과 만난다.
  - 마전어린이집: 뒷길에서 들어가자마자 주차장(최강마트 옆 노천 주차장 자리), 그 안쪽에 어린이집 (학교 주차장과 같은 구조,
    입구 = 출구 하나, 고리 통로, 칸마다 parkingArea, 회전 경로 칸 빼기, SUMO 표시 덮기).
  - 상가: 네이버에서 이름이 보이는 가게마다 건물 하나 + 간판, 나머지 땅은 이름 없는 상가 (env.roundabout_wanggil_scenery 방식).
    어린이집 부지·주차장 그림은 roundabout.daycare.add.xml, 상가는 마전동 북쪽 그림(roundabout.majeon_north.add.xml)에 넣는다.

roundabout_south_extend 파이프라인 끝(학교 주차장 뒤)에서 apply 를 부른다.  단독 실행: python -m env.roundabout_majeon_daycare
"""
import math
import os
import random
import re
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, SPEED, ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE,
                                      CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.daycare.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.daycare.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
PREFIX = 'mz_'
LOT = 'mz_k'                 # 어린이집 주차장 엣지·교차점 접두사 (골목과 구분: rerouter·주차 구역은 이것만)
MAIN = 'S_conn_up_mx'        # 완정로 북서행 (블록 쪽 차로)
O = (1742.383858, -274.389797)          # mx_T_C
U = (-0.85678, 0.51569)                 # 완정로 북서행 방향
V = (U[1], -U[0])                       # 블록 안쪽
NAVER_ANCHORS = (((1290, 505), O), ((850, 200), (1440.0, -95.0)))
S1_V = 80.0                  # 뒷길 깊이 (최강마트 뒤)
X1_U, X2_U = 212.0, 100.0    # 완정로 골목 자리
DRV_U = 170.0                # 어린이집 진입로 자리 (뒷길 위)
STEM = 8.0                   # 진입로 교차점 ~ 주차장 앞 통로
C = ST_D + AH                # 세로 통로 가운데 |v|
N_ROW = 5
GAP_DC = 3.0
DC_SIZE = (30.0, 22.0)       # 어린이집 부지 안 건물+마당 (폭, 깊이)
SITE_FILL, FENCE = '214,210,200', '70,90,70'
L_SITE = 198.5
# 표시 띠 (세계 좌표): 마전역 모퉁이 → 마전역 북쪽 길 → 표시 원 위쪽 → mx_c → mx_T_C → 완정로
STRIP = [(1424.0, -86.0), (1492.0, 30.0), (1530.0, 30.0), (1600.0, 0.0), (1680.0, -30.0), (1776.0, -60.0),
         (1749.0, -276.0), (1526.0, -150.0)]
# 네이버 캡처 픽셀 상자 (x0, y0, x1, y1) — 이름이 보이는 가게
NAMED = [
    ('mz_topsville', '탑스빌공인중개사', (893, 12, 975, 58)),
    ('mz_bread', '우리동네식빵', (838, 48, 895, 92)),
    ('mz_seven', '세븐일레븐', (815, 92, 868, 128)),
    ('mz_noah', '노아브런치카페', (965, 52, 1025, 92)),
    ('mz_bob', '밥플릭스', (985, 95, 1045, 128)),
    ('mz_diet', '다이어트랩', (1015, 130, 1070, 168)),
    ('mz_best', '베스트교육학원', (868, 168, 958, 240)),
    ('mz_mart', '최강마트', (962, 192, 1072, 285)),
    ('mz_mini', '미니공방', (1040, 300, 1115, 348)),
    ('mz_sulbing', '설빙', (1110, 350, 1175, 392)),
    ('mz_hanbit', '한빛철물마트', (1168, 395, 1228, 430)),
    ('mz_goodmorning', '굿모닝공인중개사', (1180, 430, 1240, 462)),
    ('mz_hyoja', '효자갈비', (1240, 398, 1298, 432)),
    ('mz_horin', '호린당', (1232, 358, 1290, 395)),
    ('mz_happy', '해피하우스', (1200, 262, 1262, 292)),
]
ROAD_ANGLE = -31.0
ORIGINS = ['S_conn_up_me_1', 'mx_ne_out_gd']
DESTS = ['SE_triangle_forward_in', 'mx_ne_in_gd']
INIT_OCC, TARGET_OCC = 0.5, 0.45
DURATIONS = [600, 1200, 2400, 3600]   # 등·하원 잠깐 + 교사


def P(u, v):
    return (O[0] + U[0]*u + V[0]*v, O[1] + U[1]*u + V[1]*v)


def naver(px):
    (n0, w0), (n1, w1) = NAVER_ANCHORS
    N0, N1, W0, W1 = complex(n0[0], -n0[1]), complex(n1[0], -n1[1]), complex(*w0), complex(*w1)
    a = (W1 - W0)/(N1 - N0)
    w = a*complex(px[0], -px[1]) + W0 - a*N0
    return (w.real, w.imag)


def scenery_zones():
    """마전동 북쪽 그림에 넣을 구역: (영남탑스빌 새 모양, 상가 구역 목록). 앞에 적은 구역이 땅을 먼저 차지한다."""
    from shapely.geometry import Polygon
    strip = Polygon(STRIP)
    zones = []
    # 이름 있는 가게는 draw_shops 가 네이버 자리에 직접 그린다 (roundabout.daycare.add.xml → 아래 채우기에서 빠진다)
    # 나머지 땅: 새 길로 나뉜 면마다 이름 없는 상가 (구역 하나는 가장 큰 조각만 쓰므로 길 따라 나눈다)
    big = 600.0
    cuts = [(-big, X2_U, -big, S1_V), (X2_U, X1_U, -big, S1_V), (X1_U, big, -big, S1_V), (-big, big, S1_V, big)]
    for k, (u0, u1, v0, v1) in enumerate(cuts):
        g = Polygon([P(u0, v0), P(u1, v0), P(u1, v1), P(u0, v1)]).intersection(strip)
        for j, piece in enumerate(getattr(g, 'geoms', [g])):
            if piece.area > 100:
                zones.append((f'mz_fill{k}_{j}', [tuple(c) for c in piece.exterior.coords][:-1], 'shops', [],
                              dict(angle=ROAD_ANGLE, bld=(16, 12))))
    return strip, zones


def draw_shops(net_path, add_path=ADD):
    """이름이 보이는 가게: 네이버 자리(가운데)에 완정로 방향 건물, 도로·보도·어린이집 부지·앞 건물과 겹치지 않게 들어가는 만큼.
    roundabout.daycare.add.xml 끝에 덧붙인다 (draw 뒤에 부른다)."""
    from shapely.geometry import Polygon
    from shapely.ops import transform
    from env.roundabout_shop_zones import _roads, LocalFrame
    from env.roundabout_nw_buildings import _max_rect
    from env import roundabout_interior_scenery as isc
    net = ET.parse(net_path).getroot()
    _, keep = _roads(net)
    taken = []
    for el in ET.parse(add_path).getroot():
        if el.tag == 'poly' and el.get('fill') == '1':
            taken.append(Polygon(_rd(el.get('shape'))).buffer(0))
    from shapely.ops import unary_union
    free = Polygon(STRIP).difference(keep).difference(unary_union(taken).buffer(1.5))
    fr = LocalFrame(O, U)
    to_l = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.st(p) for p in zip(x, y)]))), g)
    out, rep, roofs = [], {}, isc.SHOP_ROOFS
    for n, (key, name, (x0, y0, x1, y1)) in enumerate(NAMED):
        w = math.dist(naver((x0, y0)), naver((x1, y0)))
        h = math.dist(naver((x0, y0)), naver((x0, y1)))
        c = fr.st(naver(((x0 + x1)/2, (y0 + y1)/2)))
        a_, d_ = max(10.0, 0.75*w), max(9.0, 0.75*h)
        want = Polygon([fr.P(c[0] + su*a_/2, c[1] + sv*d_/2) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        r = None
        # 네이버 자리에 안 들어가면 주변 15m 안 빈 땅에서 (크기는 원래 이하)
        from shapely.geometry import Point
        for cand in (want, Point(fr.P(*c)).buffer(15.0)):
            cell = cand.intersection(free)
            if cell.is_empty or cell.area < 40:
                continue
            cell = max(getattr(cell, 'geoms', [cell]), key=lambda g: g.area)
            r = _max_rect(to_l(cell))
            if r and (r[1] - r[0]) >= 6 and (r[3] - r[2]) >= 5:
                s0, s1, t0, t1 = r
                if s1 - s0 > a_:                       # 원래 크기보다 크지 않게 (가운데 쪽으로)
                    m = (s0 + s1)/2
                    s0, s1 = m - a_/2, m + a_/2
                if t1 - t0 > d_:
                    m = (t0 + t1)/2
                    t0, t1 = m - d_/2, m + d_/2
                r = (s0, s1, t0, t1)
                break
            r = None
        if r is None:
            rep[key] = 'no room'
            continue
        s0, s1, t0, t1 = r
        b = Polygon([fr.P(s0, t0), fr.P(s1, t0), fr.P(s1, t1), fr.P(s0, t1)])
        out += isc._bld(key, b, roofs[n % len(roofs)])
        out.append(isc._sign(key, [name], isc.SIGN['shops'], (b.centroid.x, b.centroid.y), font_m=2.0))
        free = free.difference(b.buffer(2.0))
        rep[key] = (round(s1 - s0), round(t1 - t0))
    with open(add_path, encoding='utf-8') as fh:
        lines = fh.read().rstrip('\n').split('\n')
    end = max(i for i, ln in enumerate(lines) if ln.strip() == '</additional>')
    lines[end:end] = out
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return rep


def _frame():
    from env.roundabout_shop_zones import LocalFrame
    o = P(DRV_U, S1_V)
    return LocalFrame(o, V)        # 진입로 틀: u = 블록 안쪽, v = 그 오른쪽 (= 완정로 남동행 쪽)


def layout():
    from shapely.geometry import box
    fr = _frame()
    u0, u1 = STEM, STEM + AH + N_ROW*ST_W + AH
    vo = C + AH + ST_D
    stalls = []
    for side in (-1, 1):
        for vin, vout in ((C - AH - ST_D, C - AH), (C + AH, C + AH + ST_D)):
            into = (0.0, float(side)) if vin > C else (0.0, -float(side))
            for k in range(N_ROW):
                a = u0 + AH + k*ST_W
                v0, v1 = sorted((side*vin, side*vout))
                stalls.append(((a, a + ST_W, v0, v1), into))
    n_far = int((2*vo) // ST_W)
    v = -n_far*ST_W/2
    for k in range(n_far):
        stalls.append(((u1 + AH, u1 + AH + ST_D, v + k*ST_W, v + (k + 1)*ST_W), (1.0, 0.0)))
    lot_u = (u0 - AH - 1.0, u1 + AH + ST_D + 1.0)
    lot_v = vo + 1.0
    dc_u = lot_u[1] + GAP_DC + DC_SIZE[1]/2
    half = max(lot_v, DC_SIZE[0]/2) + 2.0
    site = box(AH + 1.6, -half, dc_u + DC_SIZE[1]/2 + 2.0, half)      # 뒷길 보도 뒤에서 시작
    return dict(key='daycare', p=LOT, gate=f'{LOT}J', fr=fr, stalls=stalls, lot_u=lot_u, lot_v=lot_v, site=site,
                dc_u=dc_u, u0=u0, u1=u1)


def cfg(net, L):
    fr = L['fr']
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    nodes = {'mz_J1': P(X1_U, S1_V), 'mz_J2': P(X2_U, S1_V), f'{LOT}J': P(DRV_U, S1_V),
             f'{LOT}C': fr.P(L['u0'], 0.0), f'{LOT}L0': fr.P(L['u0'], -C), f'{LOT}R0': fr.P(L['u0'], C),
             f'{LOT}L1': fr.P(L['u1'], -C), f'{LOT}R1': fr.P(L['u1'], C)}
    streets = [('mz_s1a', 'mz_mid_c', 'mz_J2', 0, []), ('mz_s1b', 'mz_J2', f'{LOT}J', 0, []),
               ('mz_s1c', f'{LOT}J', 'mz_J1', 0, []),
               ('mz_x1', 'mz_T1', 'mz_J1', 0, []), ('mz_x2', 'mz_T2', 'mz_J2', 0, []),
               (f'{LOT}stem', f'{LOT}J', f'{LOT}C', 0, []),
               (f'{LOT}nearL', f'{LOT}C', f'{LOT}L0', 0, []), (f'{LOT}nearR', f'{LOT}C', f'{LOT}R0', 0, []),
               (f'{LOT}left', f'{LOT}L0', f'{LOT}L1', 0, []), (f'{LOT}right', f'{LOT}R0', f'{LOT}R1', 0, []),
               (f'{LOT}far', f'{LOT}L1', f'{LOT}R1', 0, [])]
    # mx_c 위 가운데 교차점: 뒷길 깊이에서 mx_c 에 수직으로 닿는 점
    mc = next(l for e in net.findall('edge') if e.get('id') == 'mx_c_f' for l in e.findall('lane'))
    from shapely.geometry import LineString, Point
    g = LineString(_rd(mc.get('shape')))
    near = g.interpolate(g.project(Point(P(0.0, S1_V))))
    mids = [('mz_mid_c', ('mx_c_f', 'mx_c_r'), (near.x, near.y))]
    extra = [('mx_c_f', 'mz_s1a_f', 0, 0), ('mx_c_r', 'mz_s1a_f', 0, 0),
             ('mz_s1a_r', 'mx_c_f_mz_1', 0, 0), ('mz_s1a_r', 'mx_c_r_mz_1', 0, 0)]
    tees = [('mz_T1', MAIN, P(X1_U, 0.0)), ('mz_T2', MAIN, P(X2_U, 0.0))]
    return dict(TEES=tees, REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=PREFIX, MEDIAN={}, DOUBLE_CENTER=set(),
                MIDS=mids, ONEWAY=set(), EXTRA_CON=extra, SPEED=8.33, APPROACH=20.0), J


def check_roads(net, L):
    """어린이집 부지가 완정로·mx_c 등 다른 길(차로 + 보도 2.5m)과 겹치면 멈춘다 (새 골목 mz_ 은 정문 쪽이라 뺀다)."""
    from shapely.geometry import LineString, Polygon
    fr = L['fr']
    site = Polygon([fr.P(*q) for q in L['site'].exterior.coords])
    for e in net.findall('edge'):
        eid = e.get('id', '')
        if e.get('function') == 'internal' or eid.startswith(PREFIX):
            continue
        for l in e.findall('lane'):
            pts = _rd(l.get('shape'))
            if len(pts) >= 2 and LineString(pts).buffer(float(l.get('width', 3.2))/2 + 2.5).intersects(site):
                raise RuntimeError(f'어린이집 부지가 도로 {eid} 를 침범함')


def apply_net(net_path, L):
    from env import roundabout_interior as it
    net = ET.parse(net_path).getroot()
    c, _ = cfg(net, L)
    if any(e.get('id', '').startswith(PREFIX) for e in net.findall('edge')):
        raise RuntimeError('마전 상가 골목이 이미 적용됨')
    rep = it.with_cfg(c, it.apply_net, net_path)
    return c, rep


def draw(L, add_path=ADD):
    """어린이집 부지(담장) + 주차장 바닥·칸 선 + 어린이집 건물·놀이터 + 간판 (roundabout.daycare.add.xml 새로 씀)."""
    from shapely.geometry import box
    from env.roundabout_building_style import polys as style_polys, ROOF_PALETTE, PARAPET_INSET
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    fr = L['fr']
    W = lambda pts: [fr.P(*q) for q in pts]
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_majeon_daycare.py 가 생성. 마전어린이집 부지·주차장 (화면 표시 전용) -->', '<additional>']
    s0, v0, s1, v1 = L['site'].bounds
    out.append(f'    <poly id="mzd_site" type="school" color="{SITE_FILL}" fill="1" layer="{L_SITE}" '
               f'shape="{_wr(W(L["site"].exterior.coords))}"/>')
    gw = 4.0
    fence = [(s0, gw), (s0, v1), (s1, v1), (s1, v0), (s0, v0), (s0, -gw)]
    out.append(f'    <poly id="mzd_fence" type="school" color="{FENCE}" fill="0" lineWidth="0.4" layer="{L_SITE + 0.1}" '
               f'shape="{_wr(W(fence))}"/>')
    lot = box(L['lot_u'][0], -L['lot_v'], L['lot_u'][1], L['lot_v']).union(box(0.0, -AH - 1.0, L['lot_u'][0] + 1, AH + 1.0))
    lot = lot.buffer(1.0, join_style=1).buffer(-1.0, join_style=1)
    shp = _wr(W(lot.exterior.coords))
    out.append(f'    <poly id="mzd_lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    out.append(f'    <poly id="mzd_lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
               f'layer="{L_LOT + 0.1}" shape="{shp}"/>')
    for n, (rect, into, _) in enumerate(L['placed']):
        a, b, c, d = rect
        if into[0]:
            pts = [(a, c), (b, c), (b, d), (a, d)]
        else:
            near, far = (c, d) if into[1] > 0 else (d, c)
            pts = [(a, near), (a, far), (b, far), (b, near)]
        out.append(f'    <poly id="mzd_stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr(W(pts))}"/>')
    # 어린이집: 안쪽 절반은 건물(주황 지붕), 주차장 쪽 절반은 놀이터(모래색 + 놀이기구 몇 개) + 나무
    du, (bw, bd) = L['dc_u'], DC_SIZE
    ua, ub = du - bd/2, du + bd/2
    play = box(ua, -bw/2, ua + bd*0.45, bw/2)
    out.append(f'    <poly id="mzd_play" type="playground" color="222,196,150" fill="1" layer="{L_SITE + 0.2}" '
               f'shape="{_wr(W(play.exterior.coords))}"/>')
    rnd = random.Random(7)
    for k, (cu, cv, col) in enumerate([(ua + 3.0, -bw/2 + 4, '220,80,60'), (ua + 5.5, -2.0, '60,140,210'),
                                       (ua + 3.5, bw/2 - 5, '240,190,40')]):
        g = box(cu - 1.2, cv - 1.8, cu + 1.2, cv + 1.8)
        out.append(f'    <poly id="mzd_play_{k}" type="playground" color="{col}" fill="1" layer="{L_SITE + 0.3}" '
                   f'shape="{_wr(W(g.exterior.coords))}"/>')
    for k in range(5):
        cu, cv = ua + bd*0.45 - 1.5, -bw/2 + 2 + k*(bw - 4)/4 + rnd.uniform(-0.5, 0.5)
        from shapely.geometry import Point
        t = Point(fr.P(cu, cv)).buffer(1.3, 8)
        out.append(f'    <poly id="mzd_tree_{k}" type="tree" color="58,104,58" fill="1" layer="{L_SITE + 0.3}" '
                   f'shape="{_wr(t.exterior.coords)}"/>')
    bu0, bu1, bv0, bv1 = ua + bd*0.5, ub - 1.0, -bw/2 + 2.0, bw/2 - 2.0
    k_ = PARAPET_INSET
    corners = [(bu0, bv0), (bu1, bv0), (bu1, bv1), (bu0, bv1)]
    inner = [(bu0 + k_, bv0 + k_), (bu1 - k_, bv0 + k_), (bu1 - k_, bv1 - k_), (bu0 + k_, bv1 - k_)]
    for ln in style_polys('mzd_house', W(corners), W(inner), (226, 150, 110)):
        out.append(re.sub(r' layer="(-?[\d.]+)"', lambda m: f' layer="{float(m.group(1)) + 200.0:.3f}"', ln))
    for key, names, color, (cu, cv), fm in [('mzd_name', ['마전어린이집'], (220, 90, 60), ((bu0 + bu1)/2, 0.0), 2.2),
                                            ('mzd_sign', ['P 어린이집 주차장'], (40, 90, 200), (-1.0, gw + 6.0), 1.6)]:
        img = sign_stack(names, color, font_m=fm)
        fn = f'{key}.png'
        img.save(os.path.join(IMG_DIR, fn))
        x, y = fr.P(cu, cv)
        out.append(f'    <poi id="{key}" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
                   f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')
    return len(out)


def write_routes(areas, path=ROU, seed=20261114):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 마전어린이집 주차장 차 (env/roundabout_majeon_daycare.py): 칸에 주차했다가 나간다. 처음부터 서 있는 차는 departPos="stop". -->',
             '<routes>',
             f'    <vType id="dc_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="dc_init_{n}" type="dc_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="dc_f_{pid}" type="dc_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[(k + 1) % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'daycare_init_cars': n, 'daycare_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_school_parking as sp
    from env import roundabout_majeon_north_scenery as mns
    L = layout()
    net = ET.parse(net_path).getroot()
    check_roads(net, L)
    c, rep = apply_net(net_path, L)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    it.with_cfg(c, it.apply_deco, deco_path, net_path)          # 골목 중앙선·정지선·화살표 (주차장 안 표시는 바닥에 덮인다)
    rep = {f'majeon_strip_{k}': v for k, v in rep.items()}
    rep['daycare_turn_dropped'] = sp.assign(net_path, L)
    areas = sp.apply_deco(deco_path, net_path, L, vtype='dc_car', name='마전어린이집 주차장')
    rep['daycare_spaces'] = sum(n for _, n in areas.values())
    draw(L)
    rep['majeon_shops'] = draw_shops(net_path)
    rep.update(write_routes(areas))
    rep['majeon_north_redraw'] = {k: v for k, v in mns.build(net_path).items() if k != 'out'}
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
