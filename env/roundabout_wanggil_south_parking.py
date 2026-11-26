"""왕길동 아래 블록 두 아파트 주차장 (2026-11-14 사용자 요청).

  - 왕길초등학교를 지우고(env.roundabout_wanggil_scenery), 왕길근린공원은 그대로, 나머지 땅은
    서쪽 검단1차풍림아이원아파트 · 학교 자리와 남쪽 검단원흥아파트.
  - 두 단지 모두 아파트 주차장 하나씩: 왕길초 앞길(nw_wg) 남쪽 면에 붙은 작은 직사각형 (통로 N_AISLE 개, 칸은 통로 양옆 직각).
    들어갈 자리가 빠듯해서 작게 만든다 (사용자 허용).
  - 진출입: 앞길에서 주차장 앞쪽 가로 통로로 짧은 왕복 진입로 하나 (우회전·좌회전 진입, 우회전·좌회전 진출, 정지선).
    좌회전이 노란 중앙선을 넘지 않게 진입로 앞 중앙선을 끊는다.
  - 틀: 원점 = 직사각형 북동 모서리(앞길 쪽), u = 앞길 서행 방향, v = u 왼쪽 = 남쪽(단지 안쪽).
  - 그림: roundabout.wanggil_south_parking.add.xml (바닥·칸 선·간판), 차: roundabout.wanggil_south_parking.rou.xml.

roundabout_south_extend 파이프라인에서 apply 를 부른다.  단독 실행: python -m env.roundabout_wanggil_south_parking
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, TURN_CLEAR, SPEED,
                                      ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE, CAR_COLORS)
from env.roundabout_wanggil_parking import Frame, EXIT_ARROW_DIST

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.wanggil_south_parking.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.wanggil_south_parking.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pj_'
SIDEWALK = 3.0
ROAD_F, ROAD_R = 'nw_wg_f_pw_1', 'nw_wg_r'          # 앞길 동행·서행 (마전주공 주차장이 먼저 나눈 뒤 이름)
CENTER = 'nw_wg_center'                              # 앞길 노란 중앙선 (env.roundabout_nw_shops 가 그림)
CENTER_GAP = 6.0
# (접두사, 이름, 북동 모서리 대략(앞길 보도 바로 아래), 통로 수, 깊이 m)
LOTS = [
    ('pj_pr_', '검단1차풍림아이원아파트', (6.0, 284.0), 3, 40.0),
    ('pj_wh_', '검단원흥아파트', (108.0, 294.0), 3, 44.0),
]
IN_AISLES = (1, 2)
ORIGINS = ['nw_hs1_f', 'nw_wg_r', 'nw_hs4_r']
DESTS = ['nw_hs4_f', 'nw_hs1_r']
INIT_OCC, TARGET_OCC = 0.5, 0.4
DURATIONS = [1200, 2400, 3600, 4800]


def _lot_w(n):
    return 2*0.5 + n*2*AH + (n - 1)*2*ST_D + 2*ST_D


def _keep(net):
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    keep = []
    for e in net.findall('edge'):
        if e.get('id', '').startswith(P_):
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
    from shapely.geometry import Polygon
    keep = _keep(net)
    road = next(l for e in net.findall('edge') if e.get('id') == ROAD_R for l in e.findall('lane'))
    p = _rd(road.get('shape'))
    q0 = min(p, key=lambda q: abs(q[0] - 120.0))
    q1 = min(p, key=lambda q: abs(q[0] + 30.0))
    u = (q1[0] - q0[0], q1[1] - q0[1])                 # 서행 방향
    lots = []
    for lp, name, hint, n_aisle, depth in LOTS:
        W = _lot_w(n_aisle)
        fr0 = Frame(hint, u)
        found = None
        for dv in [x*0.25 for x in range(0, 80)]:
            for du in [x*0.5 for x in range(0, 30)]:
                fr = Frame(fr0.P(du, dv), u)
                rect = Polygon([fr.P(0, 0), fr.P(W, 0), fr.P(W, depth), fr.P(0, depth)])
                if not rect.intersects(keep):
                    found = (fr, rect)
                    break
            if found:
                break
        if not found:
            raise RuntimeError(f'{name} 주차장 자리를 못 찾음')
        fr, rect = found
        us = [0.5 + ST_D + AH + k*(2*AH + 2*ST_D) for k in range(n_aisle)]
        vs, vn = 0.5 + ST_D + AH, depth - (0.5 + ST_D + AH)
        N = {}
        for k, uu in enumerate(us):
            N[f'A{k}S'], N[f'A{k}N'] = (uu, vs), (uu, vn)
        N['IN'] = ((us[IN_AISLES[0]] + us[IN_AISLES[1]])/2, vs)
        streets = [(f'a{k}', f'A{k}S', f'A{k}N') for k in range(n_aisle)]
        for k in range(n_aisle - 1):
            if k == IN_AISLES[0]:
                streets += [(f's{k}a', f'A{k}S', 'IN'), (f's{k}b', 'IN', f'A{k + 1}S')]
            else:
                streets.append((f's{k}', f'A{k}S', f'A{k + 1}S'))
            streets.append((f'n{k}', f'A{k}N', f'A{k + 1}N'))
        lots.append(dict(lp=lp, name=name, fr=fr, R=rect, N=N, streets=streets))
    return lots


def _pieces(net, lots):
    """앞길 두 엣지가 진입로 교차점들로 나뉜 뒤 조각 이름 (interior.plan 규칙: 원래 이름 + _pj_1, _pj_2 …, 시작 쪽부터)."""
    from shapely.geometry import LineString, Point
    E = {e.get('id'): e for e in net.findall('edge')}
    out = {}
    for eid in (ROAD_F, ROAD_R):
        g = LineString(_rd(E[eid].get('shape') or E[eid].find('lane').get('shape')))
        order = sorted(lots, key=lambda L: g.project(Point(L['mid_near'])))
        names = [eid] + [f'{eid}_{P_}{k + 1}' for k in range(len(lots))]
        for k, L in enumerate(order):
            out[(eid, L['lp'])] = (names[k], names[k + 1])          # (진입로 앞 조각, 뒤 조각)
    return out


def cfg(net, lots):
    nodes, streets, mids, extra = {}, [], [], []
    for L in lots:
        lp, fr, N = L['lp'], L['fr'], L['N']
        L['mid_near'] = fr.P(N['IN'][0], -10.0)
    pieces = _pieces(net, lots)
    for L in lots:
        lp, fr, N = L['lp'], L['fr'], L['N']
        nodes.update({f'{lp}n{k}': fr.P(*v) for k, v in N.items()})
        streets += [(f'{lp}{nm}', f'{lp}n{a}', f'{lp}n{b}', 1 if nm.startswith('a') else 0, []) for nm, a, b in L['streets']]
        streets.append((f'{lp}drv', f'{lp}mid_in', f'{lp}nIN', 0, []))
        mids.append((f'{lp}mid_in', (ROAD_F, ROAD_R), L['mid_near']))
        fb, fa = pieces[(ROAD_F, lp)]
        rb, ra = pieces[(ROAD_R, lp)]
        extra += [(fb, f'{lp}drv_f', 0, 0), (rb, f'{lp}drv_f', 0, 0),       # 우회전·좌회전 진입
                  (f'{lp}drv_r', fa, 0, 0), (f'{lp}drv_r', ra, 0, 0)]       # 우회전(동행)·좌회전(서행) 진출
    return dict(TEES=[], REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=P_, MEDIAN={}, DOUBLE_CENTER=set(),
                MIDS=mids, ONEWAY=set(), EXTRA_CON=extra, SPEED=SPEED)


def place_stalls(net_path, L):
    """통로마다 양옆에 직각 칸 (env.roundabout_wanggil_parking.place_stalls 와 같은 규칙, 주차장 접두사별)."""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    from shapely.prepared import prep
    lp = L['lp']
    net = ET.parse(net_path).getroot()
    lanes = {l.get('id'): _rd(l.get('shape')) for e in net.findall('edge')
             if e.get('id', '').startswith(lp) and e.get('function') != 'internal' for l in e.findall('lane')}
    bands = unary_union([LineString(p).buffer(1.6, cap_style=2) for p in lanes.values()])
    sweep = unary_union([LineString(_rd(l.get('shape'))).buffer(TURN_CLEAR) for e in net.findall('edge')
                         if e.get('function') == 'internal' and e.get('id', '').startswith(f':{lp}')
                         for l in e.findall('lane') if len(_rd(l.get('shape'))) >= 2])
    drv = unary_union([LineString(p).buffer(4.0) for k, p in lanes.items() if 'drv' in k])
    land = prep(L['R'].buffer(-0.3))
    block = bands.union(sweep).union(drv)
    fr, N = L['fr'], L['N']
    stalls, used = [], []
    for nm, a, b in L['streets']:
        A, B = fr.P(*N[a]), fr.P(*N[b])
        d = (B[0] - A[0], B[1] - A[1])
        Ln = math.hypot(*d)
        d = (d[0]/Ln, d[1]/Ln)
        for side in (-1, 1):
            n = (d[1]*side, -d[0]*side)
            s = 0.0
            while s + ST_W <= Ln:
                c0 = (A[0] + d[0]*s + n[0]*AH, A[1] + d[1]*s + n[1]*AH)
                corners = [c0, (c0[0] + d[0]*ST_W, c0[1] + d[1]*ST_W),
                           (c0[0] + d[0]*ST_W + n[0]*ST_D, c0[1] + d[1]*ST_W + n[1]*ST_D),
                           (c0[0] + n[0]*ST_D, c0[1] + n[1]*ST_D)]
                g = Polygon(corners)
                if land.contains(g) and g.intersection(block).area < 0.05 and \
                        not any(g.intersection(u).area > 0.05 for u in used):
                    stalls.append((corners, n))
                    used.append(g)
                    s += ST_W
                else:
                    s += 0.5
    out = []
    for corners, n in stalls:
        cen = (sum(q[0] for q in corners)/4, sum(q[1] for q in corners)/4)
        mouth = (cen[0] - n[0]*ST_D/2, cen[1] - n[1]*ST_D/2)
        best = None
        for lid, pts in lanes.items():
            if 'drv' in lid:
                continue
            g = LineString(pts)
            dist = g.distance(Point(mouth))
            s = g.project(Point(mouth))
            p0, p1 = g.interpolate(max(0.0, s - 0.5)), g.interpolate(min(g.length, s + 0.5))
            t = (p1.x - p0.x, p1.y - p0.y)
            if (cen[0] - p0.x)*t[1] - (cen[1] - p0.y)*t[0] > 0 and dist < 3.0 and (best is None or dist < best[0]):
                best = (dist, lid[:-2])
        if best:
            out.append((corners, cen, n, best[1]))
    L['placed'] = out
    return len(out)


def apply_deco(deco_path, net_path, lots):
    from env.roundabout_builder import _stop_line, _read_lane_shapes, _arrow
    net = ET.parse(net_path).getroot()
    lanes = {l.get('id'): float(l.get('length')) for e in net.findall('edge') for l in e.findall('lane')}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for el in list(root):
        if el.get('id', '').startswith(P_) and el.tag != 'poly' or el.get('id', '').startswith(f'{P_}stop'):
            root.remove(el)
    shapes = _read_lane_shapes(net_path)
    marks = []
    all_areas = {}
    for L in lots:
        lp = L['lp']
        groups = {}
        for corners, cen, n, edge in L['placed']:
            groups.setdefault(edge, []).append((cen, n))
        areas = {}
        for edge, items in sorted(groups.items()):
            ln = f'{edge}_0'
            pa = ET.SubElement(root, 'parkingArea', dict(id=f'{lp}pa_{edge[len(lp):]}', lane=ln, startPos='0.10',
                                                         endPos=f'{lanes[ln] - 0.10:.2f}', roadsideCapacity='0',
                                                         friendlyPos='true', name=f'{L["name"]} 주차장'))
            for cen, n in items:
                k = ST_D/2 - FRONT_GAP
                navi = math.degrees(math.atan2(n[0], n[1])) % 360
                ET.SubElement(pa, 'space', dict(x=f'{cen[0] + n[0]*k:.3f}', y=f'{cen[1] + n[1]*k:.3f}',
                                                 width=f'{ST_W:.2f}', length=f'{ST_D:.2f}',
                                                 angle=f'{(180.0 - navi) % 360:.2f}'))
            areas[pa.get('id')] = (edge, len(items))
        rr_edges = sorted(e.get('id') for e in net.findall('edge') if e.get('id', '').startswith(lp)
                          and e.get('function') != 'internal' and 'drv' not in e.get('id'))
        rr = ET.SubElement(root, 'rerouter', dict(id=f'{lp}reroute', edges=' '.join(rr_edges), vTypes=f'{lp}car'))
        iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
        for pid in areas:
            ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
        _stop_line(marks, f'{lp}stop_out', shapes, f'{lp}drv_r', dist=1.0)
        for kind in ('left', 'right'):
            _arrow(marks, f'{lp}arrow_out_{kind}', shapes, f'{lp}drv_r', 0, kind, EXIT_ARROW_DIST)
        all_areas[lp] = areas
    for m in marks:
        root.append(ET.fromstring(m))
    _gap_center(root, net, [f'{L["lp"]}mid_in' for L in lots])
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return all_areas


def _gap_center(root, net, mids):
    """앞길 노란 중앙선을 진입로 교차점 양쪽 CENTER_GAP 만큼 끊는다 (조각마다 poly 하나)."""
    from shapely.geometry import LineString, Point
    from shapely.ops import substring
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    polys = [p for p in root.findall('poly') if p.get('id') == CENTER or p.get('id', '').startswith(f'{CENTER}_')]
    base = next((p for p in polys if p.get('id') == CENTER), None)
    if base is None:
        return
    for p in polys:
        if p is not base and p.get('id', '').startswith(f'{CENTER}_{P_}'):
            root.remove(p)                                      # 예전에 끊은 조각 (단독 재실행 대비)
    g = LineString(_rd(base.get('shape')))
    cuts = sorted(g.project(Point(J[m])) for m in mids)
    keep, s0 = [], 0.0
    for s in cuts:
        keep.append((s0, s - CENTER_GAP))
        s0 = s + CENTER_GAP
    keep.append((s0, g.length))
    for k, (a, b) in enumerate(keep):
        seg = substring(g, a, b)
        if k == 0:
            base.set('shape', _wr(seg.coords))
        else:
            q = ET.SubElement(root, 'poly', dict(base.attrib))
            q.set('id', f'{CENTER}_{P_}{k}')
            q.set('shape', _wr(seg.coords))


def draw(lots, add_path=ADD):
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_wanggil_south_parking.py 가 생성. 검단1차풍림아이원·검단원흥아파트 주차장 (화면 표시 전용) -->',
           '<additional>']
    for L in lots:
        lp = L['lp']
        shp = _wr(L['R'].exterior.coords)
        out.append(f'    <poly id="{lp}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
        out.append(f'    <poly id="{lp}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
                   f'layer="{L_LOT + 0.1}" shape="{shp}"/>')
        for n, (corners, cen, nv, _) in enumerate(L['placed']):
            a, b, c, d = corners
            out.append(f'    <poly id="{lp}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                       f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
        fr, N = L['fr'], L['N']
        img = sign_stack([f'P {L["name"]} 주차장'], (40, 90, 200), font_m=1.6)
        fn = f'{lp}sign.png'
        img.save(os.path.join(IMG_DIR, fn))
        x, y = fr.P(N['IN'][0] + 16.0, 2.0)
        out.append(f'    <poi id="{lp}sign" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
                   f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')


def write_routes(all_areas, path=ROU, seed=20261114):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 검단1차풍림아이원·검단원흥아파트 주차장 차 (env/roundabout_wanggil_south_parking.py): 칸에 주차했다가 나간다. -->',
             '<routes>']
    for lp in all_areas:
        lines.append(f'    <vType id="{lp}car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
                     f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>')
    n, rate = 0, 0.0
    for lp, areas in all_areas.items():
        for pid, (edge, cap) in areas.items():
            for _ in range(cap):
                if rnd.random() < INIT_OCC:
                    lines.append(f'    <trip id="{lp}init_{n}" type="{lp}car" depart="0" departPos="stop" from="{edge}" '
                                 f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                    lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                    lines.append('    </trip>')
                    n += 1
        for k, (pid, (edge, cap)) in enumerate(areas.items()):
            dur = DURATIONS[k % len(DURATIONS)]
            pr = TARGET_OCC*cap/dur
            rate += pr
            lines.append(f'    <flow id="{lp}f_{pid}" type="{lp}car" begin="0" end="100000" probability="{pr:.5f}" '
                         f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[k % len(DESTS)]}" departLane="best" '
                         f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
            lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
            lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'wanggil_s_init_cars': n, 'wanggil_s_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_wanggil_scenery as wg
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('왕길동 아래 블록 주차장이 이미 적용됨')
    lots = layout(net)
    rep = it.with_cfg(cfg(net, lots), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'wanggil_s_lot_{k}': v for k, v in rep.items()}
    for L in lots:
        rep[f'{L["lp"]}size'] = (round(_lot_w(len([k for k in L['N'] if k.endswith('S') and k != 'IN'])), 1),
                                 round(L['R'].area))
        rep[f'{L["lp"]}stalls'] = place_stalls(net_path, L)
    all_areas = apply_deco(deco_path, net_path, lots)
    draw(lots)
    rep.update(write_routes(all_areas))
    rep['wanggil_s_redraw'] = {k: v for k, v in wg.build(net_path).items()
                               if k in ('wg_pungrim', 'wg_wonheung', 'wg_wonheung_n', 'wg_park', 'forest_polys')}
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
