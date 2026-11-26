"""검단피오레대주1차 아파트 주차장 (2026-11-13 사용자 요청): 지금 붙어 있는 막다른 짧은 골목(SW_final_right) 끝에 주차장.

  - 입구·출구: 골목(왕복 1+1) 그대로. 들어가는 차로(SW_final_right_out)가 입구, 바로 옆 나가는 차로(SW_final_right_in)가 출구.
    골목 끝(SW_final_right, 막다른 끝)을 주차장 앞 가로 통로와 만나는 네거리로 바꾼다 (롯데마트 주차장과 같은 방식,
    골목 끝에서 TP_DIST m 자리에서 나눠 그 앞 조각만 다시 만든다 — 117번길 연장과 같은 REBUILD).
      들어온 차로 → 우회전 / 직진(가운데 통로) / 좌회전,  세 통로 → 나가는 차로.  골목·가운데 통로가 큰길, 앞 가로 통로는 양보(정지선).
  - 틀: u = 골목 방향(주차장 안쪽, 서북서), v = u 왼쪽. 통로 3개(가운데 통로는 골목을 그대로 잇는다) + 앞·뒤 가로 통로,
    왕복 3.2m × 2, 15 km/h. 칸은 env.roundabout_wanggil_parking.place_stalls 그대로.
  - 그림: roundabout.fiore_parking.add.xml (바닥·칸 선·간판). 그 자리의 피오레 아파트 동은
    env.roundabout_interior_scenery 가 이 파일을 보고 뺀다 (나머지 동 배치는 그대로).

roundabout_south_extend 파이프라인 끝에서 apply 를 부른다.  단독 실행: python -m env.roundabout_fiore_parking
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, SPEED,
                                      ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE, CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.fiore_parking.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.fiore_parking.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pf_'
NAME = '검단피오레대주1차 주차장'
END = 'SW_final_right'                       # 골목 막다른 끝 → 네거리
ROAD_IN, ROAD_OUT = 'SW_final_right_out', 'SW_final_right_in'   # 주차장으로 들어오는 · 나가는 차로
TP_DIST = 15.0
ROAD_HALF = 3.8                              # 골목 가운데 선 ~ 바깥 차로 가장자리 (차로 3.2 + 가운데 띠)
AISLES = (-1, 0, 1)
DEPTH = 62.0                                 # 앞 가로 통로 ~ 바닥 뒤 끝 (m)
SIDEWALK = 3.0
EDGE_GAP = 0.5 + ST_D + AH
PITCH = 2*AH + 2*ST_D
JUNCTION = 'SE_triangle_SW'                  # 골목이 붙은 큰 교차점
INIT_OCC, TARGET_OCC = 0.5, 0.4
DURATIONS = [1200, 2400, 3600, 4800]


class Frame:
    def __init__(self, o, u):
        L = math.hypot(*u)
        self.o, self.u = o, (u[0]/L, u[1]/L)
        self.v = (-self.u[1], self.u[0])

    def P(self, s, t):
        return (self.o[0] + self.u[0]*s + self.v[0]*t, self.o[1] + self.u[1]*s + self.v[1]*t)


def _frame(net):
    """골목 가운데 선 끝(J) 기준 틀. u = 큰 교차점 → 골목 끝 (주차장 안쪽)."""
    E = {e.get('id'): _rd(e.find('lane').get('shape'))
         for e in net.findall('edge') if e.get('id') in (ROAD_IN, ROAD_OUT)}
    a, b = E[ROAD_IN], E[ROAD_OUT]           # 들어가는(큰 교차점 → 끝), 나가는(끝 → 큰 교차점)
    top = ((a[-1][0] + b[0][0])/2, (a[-1][1] + b[0][1])/2)
    bot = ((a[0][0] + b[-1][0])/2, (a[0][1] + b[-1][1])/2)
    return Frame(top, (top[0] - bot[0], top[1] - bot[1])), math.dist(top, bot)


def _keep(net):
    """주차장 바닥이 겹치면 안 되는 도로 (차로 + 보도). 골목은 보도 없이 따로."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    keep = []
    for e in net.findall('edge'):
        eid = e.get('id', '')
        if eid.startswith((P_, f':{P_}', f':{END}', ROAD_IN, ROAD_OUT)):
            continue
        for l in e.findall('lane'):
            p = _rd(l.get('shape'))
            if len(p) >= 2:
                keep.append(LineString(p).buffer(float(l.get('width', 3.2))/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') not in ('internal', 'dead_end') and not j.get('id', '').startswith((P_, END)) \
                and len(_rd(s)) >= 3:
            keep.append(Polygon(_rd(s)).buffer(SIDEWALK))
    fr, L = _frame(net)
    road = LineString([fr.P(-L, 0.0), fr.P(0.0, 0.0)]).buffer(ROAD_HALF, cap_style=2)
    return unary_union(keep), road


def layout(net):
    from shapely.geometry import Polygon
    fr, _ = _frame(net)
    keep, road = _keep(net)
    vs = [k*PITCH for k in AISLES]
    v0, v1 = vs[0] - EDGE_GAP, vs[-1] + EDGE_GAP
    u0, u1 = -(AH + 0.5), DEPTH
    box = Polygon([fr.P(u0, v0), fr.P(u1, v0), fr.P(u1, v1), fr.P(u0, v1)])
    if box.intersects(keep):
        raise RuntimeError('피오레 주차장 바닥이 도로·보도와 겹침')
    ub = u1 - EDGE_GAP
    name = {k: ('0' if k == 0 else (f'L{-k}' if k < 0 else f'R{k}')) for k in AISLES}
    N, streets = {END: (0.0, 0.0)}, []
    for k, v in zip(AISLES, vs):
        N[f'A{name[k]}B'] = (ub, v)
        if k != 0:
            N[f'A{name[k]}F'] = (0.0, v)
    node = lambda k, end: END if (k == 0 and end == 'F') else f'A{name[k]}{end}'
    for k in AISLES:
        streets.append((f'a{name[k]}', node(k, 'F'), node(k, 'B')))
    for k0, k1 in zip(AISLES, AISLES[1:]):
        other = k0 if k1 == 0 else k1
        streets.append((f'f{name[other]}', END, node(other, 'F')))       # 앞 가로 통로: 네거리에서 바깥으로
        streets.append((f'b{name[k0]}{name[k1]}', node(k0, 'B'), node(k1, 'B')))
    lot = box.difference(road.buffer(0.05))
    lot = max(getattr(lot, 'geoms', [lot]), key=lambda g: g.area)
    curb = lot.exterior.difference(road.buffer(0.6))
    return dict(fr=fr, R=lot, curb=curb, N=N, streets=streets, turn_clear=2.0,
                size=(round(v1 - v0, 1), round(u1 - u0, 1), round(lot.area)))


def cfg(net, L):
    fr, N = L['fr'], L['N']
    nodes = {f'{P_}n{k}': fr.P(*v) for k, v in N.items() if k != END}
    nid = lambda k: END if k == END else f'{P_}n{k}'
    streets = [(f'{P_}{nm}', nid(a), nid(b), 1 if nm == 'a0' else 0, []) for nm, a, b in L['streets']]
    fronts = [nm for nm, a, b in L['streets'] if a == END and nm.startswith('f')]
    rin, rout = f'{ROAD_IN}_{P_}1', f'{ROAD_OUT}_{P_}1'     # 나눈 자리 ~ 네거리 조각 (들어가는·나오는)
    extra = [(rin, f'{P_}a0_f', 0, 0)] + [(rin, f'{P_}{nm}_f', 0, 0) for nm in fronts]
    extra += [(f'{P_}a0_r', rout, 0, 0)] + [(f'{P_}{nm}_r', rout, 0, 0) for nm in fronts]
    L['fronts'] = fronts
    return dict(TEES=[], REBUILD={END: [((ROAD_OUT, ROAD_IN), TP_DIST, f'{P_}TP')]}, NODES=nodes, STREETS=streets,
                PREFIX=P_, MEDIAN={}, DOUBLE_CENTER=set(), MIDS=[], ONEWAY=set(), EXTRA_CON=extra, SPEED=SPEED)


def place_stalls(net_path, L):
    """env.roundabout_wanggil_parking.place_stalls 를 이 주차장 접두사로. 네거리(골목 끝) 회전 경로도 칸에서 뺀다."""
    from env import roundabout_wanggil_parking as wp
    old = wp.P_
    wp.P_ = P_
    tmp = net_path + '.pf_tmp.xml'
    try:
        net = ET.parse(net_path).getroot()
        for e in net.findall('edge'):
            if e.get('id', '').startswith(f':{END}_'):
                e.set('id', f':{P_}{e.get("id")[1:]}')
        ET.ElementTree(net).write(tmp, encoding='utf-8', xml_declaration=True)
        return wp.place_stalls(tmp, L)
    finally:
        wp.P_ = old
        if os.path.exists(tmp):
            os.remove(tmp)


def apply_deco(deco_path, net_path, L):
    from env.roundabout_builder import _stop_line, _read_lane_shapes
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
    # 네거리에 붙은 앞 가로 통로는 빼서 리루터 표지가 골목 끝에 보이지 않게 (칸이 있는 가운데 통로는 넣는다)
    rr_edges = sorted(e.get('id') for e in net.findall('edge') if e.get('id', '').startswith(P_)
                      and e.get('function') != 'internal'
                      and (END not in (e.get('from'), e.get('to')) or e.get('id', '').startswith(f'{P_}a0_')))
    pa_edges = {edge for edge, _ in areas.values()}
    rr_edges = sorted(set(rr_edges) | pa_edges)      # 칸이 있는 통로는 반드시 리루터에
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{P_}reroute', edges=' '.join(rr_edges), vTypes='pf_car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    marks = []
    shapes = _read_lane_shapes(net_path)
    for nm in L['fronts']:                       # 네거리로 들어오는 앞 가로 통로(양보) 정지선
        _stop_line(marks, f'{P_}stop_{nm}', shapes, f'{P_}{nm}_r', dist=1.0)
    for m in marks:
        el = ET.fromstring(m)
        el.set('layer', f'{L_LINE}')             # 주차장 바닥(층 200) 위에
        root.append(el)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


def draw(L, add_path=ADD):
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_fiore_parking.py 가 생성. 검단피오레대주1차 주차장 (화면 표시 전용) -->', '<additional>']
    shp = _wr(L['R'].exterior.coords)
    out.append(f'    <poly id="{P_}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    for k, g in enumerate(getattr(L['curb'], 'geoms', [L['curb']])):
        out.append(f'    <poly id="{P_}lot_curb{k}" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
                   f'layer="{L_LOT + 0.1}" shape="{_wr(g.coords)}"/>')
    for n, (corners, cen, nv, _) in enumerate(L['placed']):
        a, b, c, d = corners
        out.append(f'    <poly id="{P_}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
    x, y = L['fr'].P(DEPTH - 3.0, 0.0)
    img = sign_stack(['P 피오레대주 주차장'], (40, 90, 200), font_m=1.8)
    fn = f'{P_}sign.png'
    img.save(os.path.join(IMG_DIR, fn))
    out.append(f'    <poi id="{P_}sign" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
               f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')
    return len(out)


def _ods(net):
    """큰 교차점(JUNCTION)으로 들어오는·나가는 큰길 (골목 빼고)."""
    ins = [e.get('id') for e in net.findall('edge') if e.get('to') == JUNCTION and e.get('function') != 'internal'
           and not e.get('id', '').startswith(('SW_final_right', P_))]
    outs = [e.get('id') for e in net.findall('edge') if e.get('from') == JUNCTION and e.get('function') != 'internal'
            and not e.get('id', '').startswith(('SW_final_right', P_))]
    return sorted(ins), sorted(outs)


def write_routes(areas, net, path=ROU, seed=20261115):
    rnd = random.Random(seed)
    origins, dests = _ods(net)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 피오레대주 주차장 차 (env/roundabout_fiore_parking.py): 큰 교차점에서 골목으로 들어와 주차했다가 다시 나간다. -->',
             '<routes>',
             f'    <vType id="pf_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="pf_init_{n}" type="pf_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(dests)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="pf_f_{pid}" type="pf_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{origins[k % len(origins)]}" to="{dests[(k + 1) % len(dests)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'fiore_init_cars': n, 'fiore_arrivals_per_h': round(rate*3600, 1), 'fiore_ods': (origins, dests)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_interior_scenery as isc
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('피오레 주차장이 이미 적용됨')
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'fiore_lot_{k}': v for k, v in rep.items()}
    rep['fiore_size'] = L['size']
    rep['fiore_stalls'] = place_stalls(net_path, L)
    areas = apply_deco(deco_path, net_path, L)
    draw(L)
    rep.update(write_routes(areas, ET.parse(net_path).getroot()))
    rep['fiore_redraw'] = {k: v for k, v in isc.build(net_path).items() if k in ('fiore',)}
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
