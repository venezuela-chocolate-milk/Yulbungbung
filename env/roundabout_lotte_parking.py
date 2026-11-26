"""롯데마트 주차장 (2026-11-13 사용자 요청): 롯데마트를 줄이고, 막다른 3+3 차로 길(SW_cross_n) 끝에 실제로 드나드는 주차장.

  - 입구·출구는 한 차로 (2026-11-13 사용자 요청). 3+3 길은 신호 교차로 쪽만 3차로로 두고, 합류부(쐐기 바닥, TAPER m)에서
    노란 중앙선은 곧게 둔 채 바깥 가장자리만 좁아져 1+1 이 된다: 들어가는 쪽 3 → 1 (지퍼식 합류), 나오는 쪽 1 → 3 (벌어짐).
    막다른 끝(SW_cross_n_end)은 1+1 길이 주차장 앞 가로 통로와 만나는 네거리 (끝에서 TP_DIST m 자리에서 길을 나눠
    그 앞 조각을 1+1 로 바꾼다 — 117번길 연장과 같은 REBUILD 방식, 신호 쪽 3차로 조각은 원래 이름 그대로).
      들어온 한 차로 → 우회전 동쪽 통로 / 직진 가운데 통로 / 좌회전 서쪽 통로, 세 통로 → 나가는 한 차로.
    1+1 길과 가운데 통로가 큰길, 앞 가로 통로는 양보 (정지선).
    합류부 북쪽에 남는 3+3 길 그림(차로 점선·바닥)은 잘라내고, 노란 중앙선은 네거리 바로 앞까지.
  - 땅: 롯데마트 동쪽 ~ 검단금호어울림 서쪽, 3+3 길 끝 ~ 북쪽 길(int_east) 보도. 축은 SUMO 동서·남북 (마트·아파트와 나란히).
    남북 통로 4개(가운데 통로는 3+3 길을 그대로 잇는다) + 앞·뒤 가로 통로, 왕복 3.2m × 2, 15 km/h.
    칸은 env.roundabout_wanggil_parking.place_stalls 그대로. 바닥은 3+3 길 차로만 비우고 길 끝에 바로 붙는다.
  - 그림: roundabout.lotte_parking.add.xml (바닥·칸 선·간판). 롯데마트 건물·예전 주차장 그림은
    env.roundabout_interior_scenery 'lotte' 구역 (줄임) / 'lotte_park' 구역 (뺌).

roundabout_south_extend 파이프라인 끝에서 apply 를 부른다.  단독 실행: python -m env.roundabout_lotte_parking
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, SPEED,
                                      ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE, CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.lotte_parking.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.lotte_parking.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pl_'
NAME = '롯데마트 주차장'
END = 'SW_cross_n_end'                       # 3+3 길 막다른 끝 → 네거리
ROAD_IN, ROAD_OUT = 'SW_cross_n_out', 'SW_cross_n_in'   # 주차장으로 들어오는(북행) · 나가는(남행) 3차로
S_NARROW, TAPER = 12.0, 20.0                # 끝에서 12m 까지 1+1, 그 뒤 20m 합류부(쐐기), 그 뒤 신호 쪽까지 3+3
TP_DIST = S_NARROW + TAPER/2                 # 합류부 교차점(쐐기 가운데) — 끝에서 이만큼 떨어진 자리에서 길을 나눈다
CENTER_END = 5.0                             # 노란 중앙선은 끝(네거리 가운데)에서 이만큼 앞까지
N_ROAD = 3
SIDEWALK = 3.0
EDGE_GAP = 0.5 + ST_D + AH
PITCH = 2*AH + 2*ST_D
AISLES = (-2, -1, 0, 1)                      # 가운데 통로(0) 기준 통로 자리 (서쪽 2개, 동쪽 1개)
X_EAST_MAX = 1200.0                          # 검단금호어울림 구역 서쪽 끝(1202) 안쪽
ORIGINS = ['S_link_east', 'SW_cross_s_in', 'SW_end_T_right_in']
DESTS = ['S_link_west', 'SW_cross_s_out', 'SW_end_T_right_out']
INIT_OCC, TARGET_OCC = 0.5, 0.4
DURATIONS = [1200, 2400, 3600, 4800]
# 2026-11-13: 원래 이 길 끝에서 사라지거나 생겨나던 파란 RL 흐름(env/roundabout_rl_traffic.py crossB_sn·crossB_ns·lowEB_B_n)
# 대신 같은 양을 주차 차로 돌린다. 신호 교차로 남→북 120, 북→남 180 대/시 유지.
#   (이름, 출발, 도착, 대/시, 주차 시간 s, 주차장에 세워진 채로 출발?)
LOTTE_FROM_RL = [
    ('sn_in', 'SW_cross_s_in', 'SW_cross_s_out', 120, 600, False),     # crossB_sn → 들어와 10분 주차 후 남쪽으로 (= crossB_ns)
    ('out_x', None, 'SW_new_cross_out', 60, 30, True),                 # lowEB_B_n → 주차해 있던 차가 나가 동쪽 아래 길로
]


def _axis(net):
    """3+3 길 가운데 선(노란 중앙선): 끝(J) 에서 신호 교차로 쪽으로 s m 인 점 C(s), 북행 오른쪽(동쪽) 단위벡터 e."""
    E = {e.get('id'): _rd(e.get('shape')) for e in net.findall('edge') if e.get('id') in (ROAD_IN, ROAD_OUT)}
    a, b = E[ROAD_OUT], E[ROAD_IN]          # 남행(끝 → 신호), 북행(신호 → 끝): 각 3차로 가운데
    top = ((a[0][0] + b[-1][0])/2, (a[0][1] + b[-1][1])/2)
    bot = ((a[-1][0] + b[0][0])/2, (a[-1][1] + b[0][1])/2)
    L = math.dist(top, bot)
    d = ((bot[0] - top[0])/L, (bot[1] - top[1])/L)      # 신호 쪽(남쪽)
    e = (-d[1], d[0])                                   # 북행 진행 방향(-d)의 오른쪽
    C = lambda s, t=0.0: (top[0] + d[0]*s + e[0]*t, top[1] + d[1]*s + e[1]*t)
    return C, top


def _taper(C):
    full, narrow = N_ROAD*AH, AH
    sa, sb = S_NARROW + TAPER, S_NARROW
    return [C(sa, -full), C(sa, full), C(sb, narrow), C(sb, -narrow)]


def _keep(net):
    """주차장 바닥이 겹치면 안 되는 도로 (차로 + 보도). 3+3 길은 따로 (보도 없이) 뺀다."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    keep, road = [], []
    for e in net.findall('edge'):
        eid = e.get('id', '')
        if eid.startswith((P_, f':{P_}', f':{END}')):
            continue
        if eid.startswith(('SW_cross_n_in', 'SW_cross_n_out')):
            continue                             # 주차장 앞은 1+1 길로 바뀐다 (아래 road)
        for l in e.findall('lane'):
            p = _rd(l.get('shape'))
            if len(p) >= 2:
                w = float(l.get('width', 3.2))/2
                keep.append(LineString(p).buffer(w + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') not in ('internal', 'dead_end') and not j.get('id', '').startswith((P_, END)) \
                and len(_rd(s)) >= 3:
            keep.append(Polygon(_rd(s)).buffer(SIDEWALK))
    C, _ = _axis(net)
    road = LineString([C(S_NARROW + TAPER), C(0.0)]).buffer(AH, cap_style=2)     # 1+1 길 (주차장 바닥은 그 옆까지)
    return unary_union(keep), road


def layout(net):
    from shapely.geometry import box
    J = next((float(j.get('x')), float(j.get('y'))) for j in net.findall('junction') if j.get('id') == END)
    keep, road = _keep(net)
    xs = [J[0] + k*PITCH for k in AISLES]
    xw, xe = xs[0] - EDGE_GAP, xs[-1] + EDGE_GAP
    if xe > X_EAST_MAX:
        raise RuntimeError(f'동쪽 끝 {xe:.1f} 이 검단금호어울림을 넘음')
    y0 = J[1] - AH - 0.5                         # 앞 가로 통로 바로 아래 (1+1 길이 바닥 안으로 파고들지 않게)
    top = J[1] + 2*EDGE_GAP + 10
    while not box(xw, y0, xe, top + 0.5).intersects(keep):
        top += 0.5
    yb = top - EDGE_GAP
    N, streets = {}, []
    name = {k: ('0' if k == 0 else (f'W{-k}' if k < 0 else f'E{k}')) for k in AISLES}
    for k, x in zip(AISLES, xs):
        N[f'A{name[k]}B'] = (x, yb)
        if k != 0:
            N[f'A{name[k]}F'] = (x, J[1])
    node = lambda k, end: END if (k == 0 and end == 'F') else f'A{name[k]}{end}'
    for k in AISLES:
        streets.append((f'a{name[k]}', node(k, 'F'), node(k, 'B')))
    for k0, k1 in zip(AISLES, AISLES[1:]):
        # 앞 가로 통로는 네거리에서 바깥으로 (…_f 가 네거리에서 나가는 쪽)
        if k1 == 0:
            streets.append((f'f{name[k0]}', END, node(k0, 'F')))
        elif k0 == 0:
            streets.append((f'f{name[k1]}', END, node(k1, 'F')))
        else:
            a, b = (k1, k0) if k1 < 0 else (k0, k1)
            streets.append((f'f{name[a]}{name[b]}', node(a, 'F'), node(b, 'F')))
        streets.append((f'b{name[k0]}{name[k1]}', node(k0, 'B'), node(k1, 'B')))
    lot = box(xw, y0, xe, top).difference(road.buffer(0.05))
    lot = max(getattr(lot, 'geoms', [lot]), key=lambda g: g.area)
    # 연석 선: 바닥 둘레 중 1+1 길과 맞닿는 곳(입구)은 비운다
    curb = lot.exterior.difference(road.buffer(0.6))

    class Fr:                                     # place_stalls 가 쓰는 fr.P (SUMO 좌표 그대로)
        @staticmethod
        def P(x, y):
            return (x, y)
    return dict(fr=Fr, R=lot, curb=curb, N={**N, END: J}, streets=streets, J=J, turn_clear=2.0, C=_axis(net)[0],
                size=(round(xe - xw, 1), round(top - y0, 1), round(lot.area)))


def cfg(net, L):
    N = L['N']
    nodes = {f'{P_}n{k}': v for k, v in N.items() if k != END}
    nid = lambda k: END if k == END else f'{P_}n{k}'
    streets = [(f'{P_}{nm}', nid(a), nid(b), 1 if nm == 'a0' else 0, []) for nm, a, b in L['streets']]
    w = next(nm for nm, a, b in L['streets'] if a == END and nm.startswith('f') and N[b][0] < L['J'][0])
    e = next(nm for nm, a, b in L['streets'] if a == END and nm.startswith('f') and N[b][0] > L['J'][0])
    rin, rout = f'{ROAD_IN}_{P_}1', f'{ROAD_OUT}_{P_}1'     # 합류부 ~ 네거리 1차로 조각 (들어가는·나오는)
    tp = f'{P_}TP'
    C, top = _axis(net)
    extra = [(ROAD_IN, rin, 1, 0), (ROAD_IN, rin, 2, 0),        # 3 → 1 합류 (0 → 0 은 자동)
             (rout, ROAD_OUT, 0, 1), (rout, ROAD_OUT, 0, 2),    # 1 → 3 벌어짐
             (rin, f'{P_}{e}_f', 0, 0),       # 우회전 (동쪽 통로)
             (rin, f'{P_}a0_f', 0, 0),        # 직진 (가운데 통로)
             (rin, f'{P_}{w}_f', 0, 0),       # 좌회전 (서쪽 통로)
             (f'{P_}{w}_r', rout, 0, 0), (f'{P_}a0_r', rout, 0, 0), (f'{P_}{e}_r', rout, 0, 0)]
    L['west'], L['east'] = w, e
    one = dict(lanes=1, spread='right')
    override = {rin: dict(one, shape=[C(TP_DIST), C(0.0)]),     # 가운데 선을 따라 오른쪽에 한 차로
                rout: dict(one, shape=[C(0.0), C(TP_DIST)])}
    return dict(TEES=[], REBUILD={END: [((ROAD_OUT, ROAD_IN), TP_DIST, tp)]}, NODES=nodes, STREETS=streets,
                PREFIX=P_, MEDIAN={}, DOUBLE_CENTER=set(), MIDS=[], ONEWAY=set(), EXTRA_CON=extra, SPEED=SPEED,
                PIECE_OVERRIDE=override, NODE_POS={tp: C(TP_DIST)}, NODE_SHAPES={tp: _taper(C)},
                NODE_TYPES={tp: 'zipper'})


def place_stalls(net_path, L):
    """env.roundabout_wanggil_parking.place_stalls 를 이 주차장 접두사로. 네거리(SW_cross_n_end) 회전 경로도 칸에서 뺀다."""
    from env import roundabout_wanggil_parking as wp
    old = wp.P_
    wp.P_ = P_
    try:
        net = ET.parse(net_path).getroot()
        # 네거리 내부 경로를 pl_ 이름으로 보이게 임시 net 을 만든다
        tmp = net_path + '.pl_tmp.xml'
        for e in net.findall('edge'):
            if e.get('id', '').startswith(f':{END}_'):
                e.set('id', f':{P_}{e.get("id")[1:]}')
        ET.ElementTree(net).write(tmp, encoding='utf-8', xml_declaration=True)
        return wp.place_stalls(tmp, L)
    finally:
        wp.P_ = old
        if os.path.exists(net_path + '.pl_tmp.xml'):
            os.remove(net_path + '.pl_tmp.xml')


def _trim_road_paint(root, C):
    """3+3 길 그림(env.roundabout_south_cross): 차로 점선·바닥은 합류부 시작(3차로 끝)까지, 노란 중앙선은 네거리 바로 앞까지."""
    from shapely.geometry import LineString, Polygon
    half = lambda s0: Polygon([C(s0, -60), C(s0, 60), C(s0 + 300, 60), C(s0 + 300, -60)])   # 신호 쪽 반평면
    rules = [('scross_dash_SW_cross_n_', half(S_NARROW + TAPER)), ('scross_under_SW_cross_n_', half(S_NARROW + TAPER)),
             ('scross_SW_cross_n_center', half(CENTER_END))]
    for p in list(root.findall('poly')):
        pid = p.get('id', '')
        hp = next((h for pre, h in rules if pid.startswith(pre)), None)
        if hp is None:
            continue
        pts = _rd(p.get('shape'))
        g = Polygon(pts).buffer(0) if p.get('fill') == '1' and len(pts) >= 3 else LineString(pts)
        g = g.intersection(hp)
        parts = [q for q in getattr(g, 'geoms', [g]) if not q.is_empty and q.geom_type in ('LineString', 'Polygon')]
        if not parts:
            root.remove(p)
            continue
        q = max(parts, key=lambda q: q.length)
        p.set('shape', _wr(q.exterior.coords if q.geom_type == 'Polygon' else q.coords))


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
    # 네거리(SW_cross_n_end)에 붙은 앞 가로 통로는 빼서 리루터 표지가 길 끝에 보이지 않게
    # (주차 칸이 있는 가운데 통로는 넣어야 만차 때 다른 칸을 찾는다)
    rr_edges = sorted(e.get('id') for e in net.findall('edge') if e.get('id', '').startswith(P_)
                      and e.get('function') != 'internal'
                      and (END not in (e.get('from'), e.get('to')) or e.get('id', '').startswith(f'{P_}a0_')))
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{P_}reroute', edges=' '.join(rr_edges), vTypes='pl_car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    marks = []
    shapes = _read_lane_shapes(net_path)
    # 네거리로 들어오는 앞 가로 통로(양보) 정지선
    for nm in (L['west'], L['east']):
        _stop_line(marks, f'{P_}stop_{nm}', shapes, f'{P_}{nm}_r', dist=1.0)
    _trim_road_paint(root, L['C'])
    for m in marks:
        el = ET.fromstring(m)
        if el.get('id', '').startswith(f'{P_}stop_'):
            el.set('layer', f'{L_LINE}')           # 정지선은 주차장 바닥(층 200) 위에
        root.append(el)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


def draw(L, add_path=ADD):
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_lotte_parking.py 가 생성. 롯데마트 주차장 (화면 표시 전용) -->', '<additional>']
    shp = _wr(L['R'].exterior.coords)
    out.append(f'    <poly id="{P_}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    for k, g in enumerate(getattr(L['curb'], 'geoms', [L['curb']])):
        out.append(f'    <poly id="{P_}lot_curb{k}" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
                   f'layer="{L_LOT + 0.1}" shape="{_wr(g.coords)}"/>')
    for n, (corners, cen, nv, _) in enumerate(L['placed']):
        a, b, c, d = corners
        out.append(f'    <poly id="{P_}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
    jx, jy = L['J']
    x0, y0, x1, y1 = L['R'].bounds
    for key, names, color, (x, y) in [('sign', ['P 롯데마트 주차장'], (40, 90, 200), (x0 + 12.0, y1 - 3.0))]:
        img = sign_stack(names, color, font_m=1.8)
        fn = f'{P_}{key}.png'
        img.save(os.path.join(IMG_DIR, fn))
        out.append(f'    <poi id="{P_}{key}" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
                   f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')
    return len(out)


def write_routes(areas, path=ROU, seed=20261114):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 롯데마트 주차장 차 (env/roundabout_lotte_parking.py): 신호 교차로에서 3+3 길로 들어와 주차했다가 다시 나간다. -->',
             '<routes>',
             f'    <vType id="pl_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="pl_init_{n}" type="pl_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="pl_f_{pid}" type="pl_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[(k + 1) % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    tot_cap = sum(cap for _, cap in areas.values())
    rl_vph = 0.0
    for name, frm, to, vph, dur, parked in LOTTE_FROM_RL:
        for k, (pid, (edge, cap)) in enumerate(areas.items()):
            v = vph*cap/tot_cap
            rl_vph += v
            src = f'from="{edge}" departPos="stop"' if parked else f'from="{frm}"'
            lines.append(f'    <flow id="pl_rl_{name}_{pid}" type="pl_car" begin="0" end="100000" vehsPerHour="{v:.3f}" '
                         f'{src} to="{to}" departLane="best" departSpeed="{0 if parked else "max"}" '
                         f'color="{CAR_COLORS[(k + 3) % len(CAR_COLORS)]}">')
            lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
            lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'lotte_init_cars': n, 'lotte_arrivals_per_h': round(rate*3600, 1), 'lotte_rl_vph': round(rl_vph)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_interior_scenery as isc
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('롯데마트 주차장이 이미 적용됨')
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'lotte_lot_{k}': v for k, v in rep.items()}
    rep['lotte_size'] = L['size']
    rep['lotte_stalls'] = place_stalls(net_path, L)
    areas = apply_deco(deco_path, net_path, L)
    draw(L)
    rep.update(write_routes(areas))
    rep['lotte_redraw'] = {k: v for k, v in isc.build(net_path).items() if k in ('lotte', 'lotte_park', 'kumho')}
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
