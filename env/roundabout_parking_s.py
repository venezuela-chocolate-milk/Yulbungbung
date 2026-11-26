"""블록 S 큰 공영주차장 (2026-11-12 사용자 요청): 실제로 들어가 주차하고 나오는 주차장.

  블록 S = 117번길(서)·117 골목(북)·V1 골목/등뼈(동)·117번길 연장(남)으로 닫힌 블록. 드림오피스텔은 남쪽 모서리로
  내리고(env.roundabout_shop_zones.BLOCK_S) 나머지 땅을 주차장으로 쓴다.

  틀: env.roundabout_shop_zones.block_s_geometry — 원점 = 보도 뺀 땅의 북서 모서리, s = 117 골목 방향, t = 117번길 방향.
  두 길이 거의 직각이라 주차 줄을 이 틀에 맞춘다.

  - 통로: 왕복 통로(차로 3.2m × 2 = 6.4m, 15 km/h). 117 골목과 나란한 가로 통로 4개(A~D) + 양 끝 세로 통로(서·동).
  - 입구: 117번길(서쪽, 통로 B 높이)에서 들어오는 한 방향 진입로 (양쪽 방향에서 좌·우회전 진입).
    출구: 동쪽 세로 통로 위 끝에서 117 골목(북쪽)으로 나가는 한 방향 진출로 (좌·우회전 진출, 정지선).
  - 주차 칸: 2.5m × 5.0m 직각 주차. 가로 통로 양옆 줄 + 동쪽 세로 통로 동쪽 줄. 칸마다 SUMO parkingArea 의 <space>
    (통로 차로 오른쪽 칸만 그 차로의 주차장) → 차가 실제로 칸 앞까지 가서 칸에 들어가 서 있다가 나온다.
  - 교통: 처음부터 일부 칸에 서 있는 차(departPos="stop") + 주차하러 오는 흐름. 찬 칸이면 입구에서 빈 주차 구역으로 돌린다.

roundabout_south_extend 파이프라인 끝(블록 S 그림 뒤)에서 apply 를 부른다.  단독 실행: python -m env.roundabout_parking_s
"""
import math
import os
import random
import xml.etree.ElementTree as ET

PREFIX = 'pk_'
LANE_W = 3.2
AH = LANE_W                  # 왕복 통로 절반 폭 (차로 하나)
SPEED = 4.17                 # 15 km/h
ST_W, ST_D = 2.5, 5.0        # 주차 칸 폭·길이
CAR_LEN = 4.6                # 주차 차량 길이 (칸보다 짧게: 칸 앞뒤 0.2m 여유)
FRONT_GAP = 0.2              # 칸 안쪽 끝 ~ 차 앞머리
# SUMO 는 주차한 차의 '앞머리'를 <space x y> 에 놓는다 (칸 가운데가 아님, 1.27 에서 확인).
# 칸 가운데를 주면 차 뒤 절반(2.5m)이 통로로 튀어나와 지나가는 차가 밟고 지나간다 → 칸 안쪽 끝에서 FRONT_GAP 앞 점을 준다.
CURB = 0.5                   # 보도 ~ 주차장 가장자리
N_AISLE = 4
S_W = CURB + AH              # 서쪽 세로 통로 가운데 s
ENTRY_AISLE = 1              # 입구가 이어지는 가로 통로 (B)
# 주차 줄 번호: 북 → 남 1~8 (1 = 북쪽 가장자리 한 줄, 2·3 / 4·5 / 6·7 = 등 맞댄 두 줄, 8 = 남쪽 가장자리 한 줄)
# 뺄 줄 (DROP_ROWS, 뺀 자리는 잔디 화단) / 출구 쪽 동쪽 세로 줄 (EAST_COLUMN)
# 2026-11-12: 2~5줄·동쪽 줄을 뺐다가 사용자 요청으로 되돌림 (줄은 전부 둔다)
DROP_ROWS = set()
# 2026-11-12 사용자 표시: 동쪽 세로 줄 전부, 1줄 출구 쪽 끝, 2~7줄 입구 쪽(서쪽) 끝 4칸을 지운다.
#   주차장 바닥 크기는 그대로 (지운 칸 자리도 아스팔트). 칸 범위: (줄 번호들, s 시작, s 끝) 안에 온전히 든 칸.
EAST_COLUMN = False
CLEAR_SLOTS = [({1}, 58.0, 1e9), ({2, 3, 4, 5, 6, 7}, -1e9, S_W + AH + 4*ST_W + 0.01)]
# 출구: 통로 A 위 S_EXIT 에서 북쪽 117 골목으로 (예전: 동쪽 세로 통로 위 끝). 1줄 출구 쪽 끝 칸을 지운 자리를 지난다.
S_EXIT = 68.0
ASPHALT = '84,84,90'         # 도로 바닥과 같은 색 (roundabout.gui.xml)
LINE = '240,240,240'
GRASS = '112,140,96'
CURB_COLOR = '176,172,164'
# 그림 층: SUMO 는 parkingArea·칸·rerouter 표시를 늘 그리고(설정 파일 색 무시, 1.27), 층 번호 = 그리기 순서
# (차로 2, 차량 256). 주차장 바닥·칸 선·잔디를 차로·주차장 표시보다 위, 차량보다 아래에 깔아 기본 표시를 덮는다.
L_GRASS, L_LOT, L_LINE = 199.0, 200.0, 200.2
ORIGINS = ['X_main_in', 'X_nw_in', 'T_stop_side_in_a']        # 주차하러 오는 차 출발 (상가 흐름과 같은 큰길·117번길)
DESTS = ['X_main_out', 'X_nw_out', 'X_se_out_b']
INIT_OCC = 0.55              # 처음 서 있는 차 비율
TARGET_OCC = 0.5             # 흐름으로 유지할 평균 점유율
DURATIONS = [1200, 2400, 3600, 4800]
CAR_COLORS = ['245,245,245', '30,30,32', '170,172,176', '110,112,118', '35,55,95', '150,30,35',
              '235,235,230', '60,62,66', '200,200,205']
NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ROU = os.path.join(NET_DIR, 'roundabout.parking.rou.xml')


def _rd(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _wr(pts):
    return ' '.join(f'{x:.3f},{y:.3f}' for x, y in pts)


def layout(net):
    """틀 좌표로 통로·칸·입출구를 정한다."""
    from shapely.geometry import LineString, box
    from shapely.ops import transform
    from env.roundabout_shop_zones import block_s_geometry, BLOCK_S
    _, build_w, fr, _, _ = block_s_geometry(net)
    to_st = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.st(p) for p in zip(x, y)]))), g)
    R = to_st(build_w)
    inner = R.buffer(-0.2)
    # 가로 통로 가운데 t: 맨 위 줄(칸 하나) 아래 통로, 그 뒤로 등 맞댄 두 줄마다 통로 하나, 맨 아래 줄(칸 하나)
    aisle_t = [CURB + ST_D + AH + k*(2*AH + 2*ST_D) for k in range(N_AISLE)]
    t_top, t_bot = aisle_t[0] - AH, aisle_t[-1] + AH
    lot_bottom = t_bot + ST_D

    def s_right(t):
        g = R.intersection(LineString([(-50, t), (300, t)]))
        return max(c[0] for gg in getattr(g, 'geoms', [g]) for c in gg.coords)
    # 동쪽 세로 통로: 맨 아래 통로까지 블록 안에 들어가는 가장 동쪽
    s_e = math.floor(min(s_right(t_top + k*0.5) for k in range(int((t_bot + 0.5 - t_top)/0.5) + 1)) - AH - CURB)
    stalls = []   # (틀 사각형 s0, s1, t0, t1, 칸 안쪽 방향(틀), 통로 엣지)
    lot_stalls = []   # 바닥 그림용: 지우기 전 칸 전부 (주차장 크기 유지)

    def add(rect, into, edge, row=None):
        s0, s1, t0, t1 = rect
        if inner.contains(box(s0, t0, s1, t1)):
            lot_stalls.append((rect, into, edge))
            if row is None and not EAST_COLUMN:
                return
            if any(row in rows and s0 >= a - 1e-6 and s1 <= b + 1e-6 for rows, a, b in CLEAR_SLOTS):
                return
            stalls.append((rect, into, edge))
    # 가로 줄: 통로 k 북쪽 줄은 _r(서행) 오른쪽, 남쪽 줄은 _f(동행) 오른쪽
    for k, c in enumerate(aisle_t):
        for side in (-1, 1):
            t0, t1 = sorted((c + side*AH, c + side*(AH + ST_D)))
            row = 2*k + (1 if side < 0 else 2)
            if row in DROP_ROWS:
                continue
            outer = (k == 0 and side < 0) or (k == N_AISLE - 1 and side > 0)
            s = CURB if outer else S_W + AH
            while s + ST_W <= s_e - AH + 1e-6:
                seg = f'{PREFIX}a0b_' if k == 0 and s + ST_W/2 > S_EXIT else f'{PREFIX}a{k}_'   # 통로 A 는 출구에서 나뉜다
                add((s, s + ST_W, t0, t1), (0.0, float(side)), seg + ('f' if side > 0 else 'r'), row)
                s += ST_W
    # 동쪽 세로 통로 동쪽 줄: 북행(_r) 오른쪽. 칸이 걸친 통로 조각 (E_k → E_k+1)
    t = t_top
    while t + ST_W <= t_bot + 1e-6:
        k = min(N_AISLE - 2, max(0, sum(1 for c in aisle_t if c <= t + ST_W/2) - 1))
        add((s_e + AH, s_e + AH + ST_D, t, t + ST_W), (1.0, 0.0), f'{PREFIX}e{k}_r')
        t += ST_W
    dream = next(b for b in BLOCK_S if b[0] == 'dream')[4]
    return dict(fr=fr, R=R, aisle_t=aisle_t, s_e=s_e, s_w=S_W, t_top=t_top, t_bot=t_bot, lot_bottom=lot_bottom,
                stalls=stalls, lot_stalls=lot_stalls, dream=dream)


TURN_CLEAR = 1.4             # 교차점 회전 경로 중심 ~ 칸: 차 반폭 0.9 + 여유 0.5


def clear_turns(net_path, L):
    """통로 교차점 안 회전 경로(내부 차로)가 지나가는 칸을 뺀다. 칸이 남아 있으면 모퉁이를 도는 차가 그 칸 차를 밟는다
    (2026-11-12 확인: 서쪽 세로 통로에서 가로 통로로 꺾는 차가 줄 첫 칸을 가로지름). 앞으로 만드는 주차장도 망을 만든 뒤 이걸 거친다."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    net = ET.parse(net_path).getroot()
    paths = [LineString(_rd(l.get('shape'))).buffer(TURN_CLEAR)
             for e in net.findall('edge') if e.get('function') == 'internal' and e.get('id', '').startswith(f':{PREFIX}')
             for l in e.findall('lane') if len(_rd(l.get('shape'))) >= 2]
    sweep = unary_union(paths)
    keep, dropped = [], 0
    for st in L['stalls']:
        g = Polygon(_stall_world(L, st[0], st[1])[0])
        if g.intersection(sweep).area > 0.05:
            dropped += 1
        else:
            keep.append(st)
    L['stalls'] = keep
    return dropped


def cfg(net, L):
    fr = L['fr']
    P = lambda s, t: fr.P(s, t)
    nodes, streets = {}, []
    for k, c in enumerate(L['aisle_t']):
        nodes[f'{PREFIX}nW{k}'] = P(L['s_w'], c)
        nodes[f'{PREFIX}nE{k}'] = P(L['s_e'], c)
        if k == 0:              # 통로 A: 출구 교차점(nX)에서 둘로
            nodes[f'{PREFIX}nX'] = P(S_EXIT, c)
            streets.append((f'{PREFIX}a0', f'{PREFIX}nW0', f'{PREFIX}nX', 0, []))
            streets.append((f'{PREFIX}a0b', f'{PREFIX}nX', f'{PREFIX}nE0', 0, []))
        else:
            streets.append((f'{PREFIX}a{k}', f'{PREFIX}nW{k}', f'{PREFIX}nE{k}', 0, []))
        if k:
            streets.append((f'{PREFIX}w{k-1}', f'{PREFIX}nW{k-1}', f'{PREFIX}nW{k}', 1, []))
            streets.append((f'{PREFIX}e{k-1}', f'{PREFIX}nE{k-1}', f'{PREFIX}nE{k}', 1, []))
    c_in = L['aisle_t'][ENTRY_AISLE]
    streets.append((f'{PREFIX}drv_in', f'{PREFIX}mid_in', f'{PREFIX}nW{ENTRY_AISLE}', 0, []))
    streets.append((f'{PREFIX}drv_out', f'{PREFIX}nX', f'{PREFIX}mid_out', 0, []))
    mids = [(f'{PREFIX}mid_in', ('T_stop_side_in_a', 'T_stop_side_out_b'), P(-6.0, c_in)),
            (f'{PREFIX}mid_out', ('shop_117_f', 'shop_117_r'), P(S_EXIT, -6.0))]
    extra = [('T_stop_side_in_a', f'{PREFIX}drv_in_f', 0, 0), ('T_stop_side_out_b', f'{PREFIX}drv_in_f', 0, 0),
             (f'{PREFIX}drv_out_f', f'shop_117_f_{PREFIX}1', 0, 0), (f'{PREFIX}drv_out_f', f'shop_117_r_{PREFIX}1', 0, 0)]
    return dict(TEES=[], REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=PREFIX, MEDIAN={}, DOUBLE_CENTER=set(),
                MIDS=mids, ONEWAY={f'{PREFIX}drv_in', f'{PREFIX}drv_out'}, EXTRA_CON=extra, SPEED=SPEED)


def apply_net(net_path):
    from env import roundabout_interior as it
    net = ET.parse(net_path).getroot()
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    return L, {f'parking_{k.split("_", 1)[1]}': v for k, v in rep.items()}


def _stall_world(L, rect, into):
    fr = L['fr']
    s0, s1, t0, t1 = rect
    corners = [fr.P(s0, t0), fr.P(s1, t0), fr.P(s1, t1), fr.P(s0, t1)]
    cx, cy = fr.P((s0 + s1)/2, (t0 + t1)/2)
    d = (fr.u[0]*into[0] + fr.v[0]*into[1], fr.u[1]*into[0] + fr.v[1]*into[1])   # 칸 안쪽(앞머리) 방향, 세계
    navi = math.degrees(math.atan2(d[0], d[1])) % 360                        # 북 0°, 시계 방향
    return corners, (cx, cy), navi, d


def apply_deco(deco_path, net_path, L):
    """parkingArea(칸마다 <space>) + 입구 재배정(rerouter) + 출구 정지선. deco.xml 은 시뮬레이션·화면 둘 다 불러온다."""
    from env.roundabout_builder import _stop_line, _read_lane_shapes
    net = ET.parse(net_path).getroot()
    lanes = {l.get('id'): float(l.get('length')) for e in net.findall('edge') for l in e.findall('lane')}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for el in list(root):
        if el.get('id', '').startswith(PREFIX):
            root.remove(el)
    groups = {}
    for rect, into, edge in L['stalls']:
        groups.setdefault(edge, []).append((rect, into))
    areas = {}
    for edge, items in sorted(groups.items()):
        ln = f'{edge}_0'
        if ln not in lanes:
            raise RuntimeError(f'주차 통로 차로 없음: {ln}')
        pa = ET.SubElement(root, 'parkingArea', dict(
            id=f'{PREFIX}pa_{edge[len(PREFIX):]}', lane=ln, startPos='0.10', endPos=f'{lanes[ln] - 0.10:.2f}',
            roadsideCapacity='0', friendlyPos='true', name='공영주차장'))
        for rect, into in items:
            _, (cx, cy), navi, d = _stall_world(L, rect, into)
            k = ST_D/2 - FRONT_GAP                       # 칸 가운데 → 앞머리 자리 (칸 안쪽 방향)
            cx, cy = cx + d[0]*k, cy + d[1]*k
            # SUMO <space angle>: 180 - 진행 방향(북 0° 시계) (roadside 칸과 같은 약속) → 차가 칸 안쪽을 보고 선다
            ET.SubElement(pa, 'space', dict(x=f'{cx:.3f}', y=f'{cy:.3f}', width=f'{ST_W:.2f}', length=f'{ST_D:.2f}',
                                             angle=f'{(180.0 - navi) % 360:.2f}'))
        areas[pa.get('id')] = (edge, len(items))
    # 주차장 안 통로 전부 (찬 구역 앞에서도 빈 구역 찾기를 이어 가게). 진입로에 두면 rerouter 표시가 바닥 그림 밖으로 보인다
    rr_edges = sorted(e.get('id') for e in net.findall('edge')
                      if e.get('id', '').startswith(PREFIX) and e.get('function') != 'internal' and 'drv' not in e.get('id'))
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{PREFIX}reroute_in', edges=' '.join(rr_edges), vTypes='pk_car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    shapes = _read_lane_shapes(net_path)
    marks = []
    _stop_line(marks, f'{PREFIX}stop_out', shapes, f'{PREFIX}drv_out_f', dist=1.0)
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


def draw(net_path, L, add_path=None, img_dir=None):
    """roundabout.shops.add.xml 에 주차장 바닥·칸 선·잔디·간판 (화면 표시 전용, pk_ 로 시작하는 줄은 다시 그린다)."""
    import re
    from shapely.geometry import box, Polygon
    from shapely.ops import unary_union
    from env.roundabout_shop_buildings import OUT, IMG_DIR, PPM
    from env.roundabout_nw_buildings import sign_stack
    add_path = add_path or OUT
    img_dir = img_dir or IMG_DIR
    fr, R = L['fr'], L['R']
    from env.roundabout_shop_zones import block_s_geometry
    from env.roundabout_shop_buildings import PAVING
    blockS = block_s_geometry(net := ET.parse(net_path).getroot())[0]
    W = lambda pts: [fr.P(*p) for p in pts]
    lane = {l.get('id'): _rd(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    out = []
    # 블록 가장자리 보도도 같은 층으로 한 번 더 (주차 구역 표지(P)가 보도 위로 비죽 나오는 것 덮기). 진출입로 자리는 비운다
    drv = unary_union([Polygon(_offset_band(lane[f'{PREFIX}{d}_f_0'], 7.0)) for d in ('drv_in', 'drv_out')])
    ring = blockS.difference(Polygon([fr.P(*q) for q in R.exterior.coords]).buffer(0.05)).difference(drv)
    for k, g in enumerate(getattr(ring, 'geoms', [ring])):
        if g.area > 1:
            out.append(f'    <poly id="{PREFIX}walk_{k}" type="sidewalk" color="{PAVING}" fill="1" layer="{L_GRASS - 0.1}" '
                       f'shape="{_wr(g.exterior.coords)}"/>')
    # 잔디: 블록 땅(보도 뺀) 중 드림오피스텔 구역 밖 (주차장 바닥이 그 위에)
    s0, s1, t0, t1 = L['dream']
    grass = R.difference(box(s0, t0, s1, t1))
    for k, g in enumerate(getattr(grass, 'geoms', [grass])):
        if g.area > 2:
            out.append(f'    <poly id="{PREFIX}grass_{k}" type="grass" color="{GRASS}" fill="1" layer="{L_GRASS}" '
                       f'shape="{_wr(W(g.exterior.coords))}"/>')
    # 주차장 바닥: 통로 띠 + 칸 + 모서리 (둥글게)
    se, sw, at = L['s_e'], L['s_w'], L['aisle_t']
    parts = [box(sw - AH, L['t_top'], sw + AH, L['t_bot']), box(se - AH, L['t_top'], se + AH, L['t_bot'])]
    parts += [box(sw - AH, c - AH, se + AH, c + AH) for c in at]
    parts += [box(r[0], r[2], r[1], r[3]) for r, _, _ in L['lot_stalls']]
    lot = unary_union(parts).buffer(1.2, join_style=2).buffer(-1.2, join_style=1).intersection(R.buffer(-0.05))
    lot = max(getattr(lot, 'geoms', [lot]), key=lambda g: g.area)
    shp = _wr(W(lot.exterior.coords))
    out.append(f'    <poly id="{PREFIX}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    out.append(f'    <poly id="{PREFIX}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
               f'layer="{L_LOT + 0.1}" shape="{shp}"/>')
    # 통로로 둘러싸인 안쪽 빈 자리(칸을 뺀 줄): 잔디 화단 + 연석
    for k, hole in enumerate(h for h in lot.interiors if Polygon(h).area > 30.0):   # 모서리 칸 하나 뺀 자리는 바닥 그대로
        hs = _wr(W(hole.coords))
        out.append(f'    <poly id="{PREFIX}island_{k}" type="grass" color="{GRASS}" fill="1" layer="{L_LOT + 0.05}" shape="{hs}"/>')
        out.append(f'    <poly id="{PREFIX}island_curb_{k}" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
                   f'layer="{L_LOT + 0.1}" shape="{hs}"/>')
    # 진출입로가 보도를 지나는 자리: 도로 바닥. 입구는 rerouter 표시를 덮게 주차장 층, 출구는 정지선이 보이게 아래 층
    for eid, lay in ((f'{PREFIX}drv_in_f', L_LOT), (f'{PREFIX}drv_out_f', -1.69)):
        g = Polygon(_offset_band(lane[f'{eid}_0'], 3.2))
        out.append(f'    <poly id="{PREFIX}apron_{eid}" type="parking" color="{ASPHALT}" fill="1" layer="{lay}" '
                   f'shape="{_wr(g.exterior.coords)}"/>')
    # 칸 선: 통로 쪽이 열린 ㄷ자 (흰 선 0.12m)
    for n, (rect, into, _) in enumerate(L['stalls']):
        a, b, c, d = rect
        if into[1]:                      # 가로 줄: 통로가 t 쪽
            near_t, far_t = (c, d) if into[1] > 0 else (d, c)
            pts = [(a, near_t), (a, far_t), (b, far_t), (b, near_t)]
        else:                            # 동쪽 줄: 통로가 s 작은 쪽
            pts = [(a, c), (b, c), (b, d), (a, d)]
        out.append(f'    <poly id="{PREFIX}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr(W(pts))}"/>')
    # 간판: P 공영주차장(입구 옆), 입구, 출구
    signs = [('sign', ['P 공영주차장'], (40, 90, 200), fr.P(-1.0, at[ENTRY_AISLE] + AH + 6.5)),   # 입구 남쪽 보도
             ('in', ['입구'], (30, 140, 70), fr.P(-1.0, at[ENTRY_AISLE] - AH - 2.4)),
             ('out', ['출구'], (200, 60, 40), fr.P(S_EXIT + AH + 2.6, -1.0))]
    for key, names, color, (x, y) in signs:
        img = sign_stack(names, color, font_m=2.0)
        fn = f'{PREFIX}{key}.png'
        img.save(os.path.join(img_dir, fn))
        out.append(f'    <poi id="{PREFIX}{key}" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
                   f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    with open(add_path, encoding='utf-8') as fh:
        old = fh.read().split('\n')
    keep = [ln for ln in old if not re.search(rf' id="{PREFIX}', ln)]
    end = max(i for i, ln in enumerate(keep) if ln.strip() == '</additional>')
    keep[end:end] = out
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(keep))
    return len(out)


def _offset_band(pts, w):
    from env.roundabout_geometry import _offset
    a, b = _offset(pts, w/2), _offset(pts, -w/2)
    return a + b[::-1]


def write_routes(L, areas, path=ROU, seed=20261112):
    """처음부터 서 있는 차 + 주차하러 오는 흐름 (주차 구역마다 하나, 칸 수에 비례한 확률)."""
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 블록 S 공영주차장(env/roundabout_parking_s.py) 주차 차량. 칸(parkingArea space)에 실제로 들어가 서 있다가 나간다.',
             '     처음부터 서 있는 차는 departPos="stop" (칸에서 출발). 찬 구역이면 입구(pk_drv_in_f)에서 빈 구역으로 돌린다. -->',
             '<routes>',
             '    <vType id="pk_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n_init = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                dur = rnd.randint(120, 3600)
                lines.append(f'    <trip id="{PREFIX}init_{n_init}" type="pk_car" depart="0" departPos="stop" '
                             f'from="{edge}" to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
                lines.append('    </trip>')
                n_init += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        p = TARGET_OCC*cap/dur
        rate += p
        lines.append(f'    <flow id="{PREFIX}f_{pid[len(PREFIX):]}" type="pk_car" begin="0" end="100000" probability="{p:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[(k + 1) % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'parking_init_cars': n_init, 'parking_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    L, rep = apply_net(net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep['parking_turn_cleared'] = clear_turns(net_path, L)
    areas = apply_deco(deco_path, net_path, L)
    rep['parking_areas'] = len(areas)
    rep['parking_spaces'] = sum(c for _, c in areas.values())
    rep['parking_draw_lines'] = draw(net_path, L)
    rep.update(write_routes(L, areas))
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
