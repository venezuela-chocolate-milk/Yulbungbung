"""마전주공아파트 주차장 (2026-11-12 사용자 요청): 아래쪽 동을 없앤 자리에 네모 반듯한 아파트 주차장.

  - 자리: 왕길동 위 블록 서쪽 끝 마전주공아파트(env.roundabout_wanggil_scenery 'wg_majeon') 아래쪽.
    남쪽 왕길초 앞길(nw_wg)에 붙은 직사각형 (남쪽 변이 왕길초 앞길과 나란함). 서쪽 비스듬한 도로(nw_hs2)는 침범하지 않고,
    동쪽 이웃 단지(디아망) 쪽으로는 넘어가도 된다 (사용자 허용). 크기는 예전 사다리꼴 땅(약 11,150 m²)의 2/3.
    남는 땅(도로 쪽 띠·위쪽)은 마전주공아파트 동으로 채운다 (구역은 원래 크기, 주차장 그림 자리만 빠진다).
  - 틀: 원점 = 직사각형 남서 모서리, u = 왕길초 앞길 동행 방향, v = 북쪽(u 왼쪽).
  - 통로(왕복 3.2m × 2, 15 km/h): 남북 통로 4개 + 남·북 가로 통로. 칸은 통로 양옆에 직각으로 빈틈없이
    (env.roundabout_parking_s 규칙: 앞머리 점, 회전 경로 칸 빼기, SUMO 표시 덮기).
  - 입구: 남쪽 왕길초 앞길에서 남쪽 가로 통로로 (한 방향, 양쪽 방향에서 진입).
    출구: 서쪽 통로에서 단지 사이 짧은 진출로로 서쪽 도로(nw_hs2)에 (한 방향, 우회전 북행 + 좌회전 남행, 정지선).
  - 그림: roundabout.wanggil_parking.add.xml (주차장 바닥·칸 선·간판 — 공영주차장이 아니라 아파트 주차장).

roundabout_south_extend 파이프라인 끝에서 apply 를 부른다.  단독 실행: python -m env.roundabout_wanggil_parking
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_W, ST_D, CAR_LEN, FRONT_GAP, TURN_CLEAR, SPEED,
                                      ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE, CAR_COLORS)

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.wanggil_parking.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.wanggil_parking.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pw_'
NAME = '마전주공아파트 주차장'
SIDEWALK = 3.0               # 왕길동 그림과 같은 보도 폭
OLD_AREA = 11150.0           # 예전 사다리꼴 주차장 땅
N_AISLE = 4
LOT_W = 2*0.5 + N_AISLE*2*AH + (N_AISLE - 1)*2*ST_D + 2*ST_D        # 칸 한 줄 + 통로/등 맞댄 두 줄 … + 칸 한 줄
LOT_D = round(OLD_AREA*2/3/LOT_W, 1)
SW_HINT = (-104.0, 287.0)    # 남서 모서리 대략 (서쪽 도로 보도 바로 안쪽, 왕길초 앞길 보도 바로 위)
IN_AISLES = (1, 2)           # 입구는 이 두 통로 사이 남쪽 가로 통로에
EXIT_V = 42.0                # 출구가 붙는 서쪽 통로 위 v
ORIGINS = ['nw_hs1_f', 'nw_wg_r', 'nw_hs4_r']
DESTS = ['nw_hs4_f', 'nw_hs1_r']
INIT_OCC, TARGET_OCC = 0.5, 0.4
DURATIONS = [1200, 2400, 3600, 4800]


class Frame:
    def __init__(self, o, u):
        L = math.hypot(*u)
        self.o, self.u = o, (u[0]/L, u[1]/L)
        self.v = (-self.u[1], self.u[0])          # u 왼쪽 = 북쪽

    def P(self, s, t):
        return (self.o[0] + self.u[0]*s + self.v[0]*t, self.o[1] + self.u[1]*s + self.v[1]*t)


def _keep(net):
    """도로(차로 + 보도 3m) — 주차장 직사각형이 겹치면 안 되는 땅 (pw_ 길은 뺀다)."""
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
    wg = next(l for e in net.findall('edge') if e.get('id') == 'nw_wg_f' for l in e.findall('lane'))
    p = _rd(wg.get('shape'))
    # 왕길초 앞길 전체 방향 (교차점에서 휘어 나오는 첫 조각 말고, x -90 ~ -10 사이 곧은 구간)
    q0 = min(p, key=lambda q: abs(q[0] + 90.0))
    q1 = min(p, key=lambda q: abs(q[0] + 10.0))
    fr0 = Frame(SW_HINT, (q1[0] - q0[0], q1[1] - q0[1]))
    # 도로와 안 겹칠 때까지 북쪽(v)·동쪽(u)으로 조금씩 민다
    found = None
    for dv in [x*0.25 for x in range(0, 80)]:
        for du in [x*0.5 for x in range(0, 30)]:
            fr = Frame(fr0.P(du, dv), fr0.u)
            rect = Polygon([fr.P(0, 0), fr.P(LOT_W, 0), fr.P(LOT_W, LOT_D), fr.P(0, LOT_D)])
            if not rect.intersects(keep):
                found = (fr, rect)
                break
        if found:
            break
    if not found:
        raise RuntimeError('주차장 직사각형 자리를 못 찾음')
    fr, rect = found
    us = [0.5 + ST_D + AH + k*(2*AH + 2*ST_D) for k in range(N_AISLE)]
    vs, vn = 0.5 + ST_D + AH, LOT_D - (0.5 + ST_D + AH)
    N = {}
    for k, u in enumerate(us):
        N[f'A{k}S'], N[f'A{k}N'] = (u, vs), (u, vn)
    N['IN'] = ((us[IN_AISLES[0]] + us[IN_AISLES[1]])/2, vs)
    N['X'] = (us[0], EXIT_V)
    streets = [('a0s', 'A0S', 'X'), ('a0n', 'X', 'A0N')]
    streets += [(f'a{k}', f'A{k}S', f'A{k}N') for k in range(1, N_AISLE)]
    for k in range(N_AISLE - 1):
        if k == IN_AISLES[0]:
            streets += [(f's{k}a', f'A{k}S', 'IN'), (f's{k}b', 'IN', f'A{k + 1}S')]
        else:
            streets.append((f's{k}', f'A{k}S', f'A{k + 1}S'))
        streets.append((f'n{k}', f'A{k}N', f'A{k + 1}N'))
    return dict(fr=fr, R=rect, N=N, streets=streets)


def cfg(net, L):
    fr, N = L['fr'], L['N']
    nodes = {f'{P_}n{k}': fr.P(*v) for k, v in N.items()}
    streets = [(f'{P_}{nm}', f'{P_}n{a}', f'{P_}n{b}', 1 if nm.startswith('a') else 0, []) for nm, a, b in L['streets']]
    streets += [(f'{P_}drv_in', f'{P_}mid_in', f'{P_}nIN', 0, []), (f'{P_}drv_out', f'{P_}nX', f'{P_}mid_out', 0, [])]
    mids = [(f'{P_}mid_in', ('nw_wg_f', 'nw_wg_r'), fr.P(N['IN'][0], -10.0)),
            (f'{P_}mid_out', ('nw_hs2_f', 'nw_hs2_r'), fr.P(-40.0, N['X'][1]))]
    extra = [('nw_wg_f', f'{P_}drv_in_f', 0, 0), ('nw_wg_r', f'{P_}drv_in_f', 0, 0),
             (f'{P_}drv_out_f', f'nw_hs2_f_{P_}1', 0, 0),      # 우회전 (북행 바깥 차로)
             (f'{P_}drv_out_f', f'nw_hs2_r_{P_}1', 0, 1)]      # 좌회전 (남행 안쪽 차로)
    return dict(TEES=[], REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=P_, MEDIAN={}, DOUBLE_CENTER=set(),
                MIDS=mids, ONEWAY={f'{P_}drv_in', f'{P_}drv_out'}, EXTRA_CON=extra, SPEED=SPEED)


def place_stalls(net_path, L):
    """통로마다 양옆에 직각 칸: 직사각형 안에 온전히, 통로 띠·진출입로·회전 경로·다른 칸과 안 겹치게."""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    from shapely.prepared import prep
    net = ET.parse(net_path).getroot()
    lanes = {l.get('id'): _rd(l.get('shape')) for e in net.findall('edge')
             if e.get('id', '').startswith(P_) and e.get('function') != 'internal' for l in e.findall('lane')}
    bands = unary_union([LineString(p).buffer(1.6, cap_style=2) for p in lanes.values()])
    sweep = unary_union([LineString(_rd(l.get('shape'))).buffer(L.get('turn_clear', TURN_CLEAR)) for e in net.findall('edge')
                         if e.get('function') == 'internal' and e.get('id', '').startswith(f':{P_}')
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
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{P_}reroute', edges=' '.join(rr_edges), vTypes='pw_car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    marks = []
    shapes = _read_lane_shapes(net_path)
    _stop_line(marks, f'{P_}stop_out', shapes, f'{P_}drv_out_f', dist=1.0)
    # 출구 좌·우회전 화살표 (두 화살표가 겹쳐 ㅜ자 좌우 화살표)
    from env.roundabout_builder import _arrow
    for kind in ('left', 'right'):
        _arrow(marks, f'{P_}arrow_out_{kind}', shapes, f'{P_}drv_out_f', 0, kind, EXIT_ARROW_DIST)
    for m in marks:
        root.append(ET.fromstring(m))
    # 서쪽 도로 노란 중앙선을 출구 앞에서 끊는다 (좌회전 진출이 중앙선을 넘지 않게, 2026-11-12 사용자 요청)
    _gap_center(root, net)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas


EXIT_ARROW_DIST = 3.4        # 출구 진출로 끝(정지선)에서 화살표까지
CENTER = 'nw_hs2_center'     # 서쪽 도로 노란 중앙선 (env.roundabout_nw_shops 가 그림)
CENTER_GAP = 6.0             # 출구 교차점 양쪽으로 비우는 길이


def _gap_center(root, net):
    from shapely.geometry import LineString, Point
    from shapely.ops import substring
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    polys = {p.get('id'): p for p in root.findall('poly')}
    if CENTER not in polys or f'{CENTER}_b' in polys:
        return
    p = polys[CENTER]
    g = LineString(_rd(p.get('shape')))
    s = g.project(Point(J[f'{P_}mid_out']))
    a, b = substring(g, 0, s - CENTER_GAP), substring(g, s + CENTER_GAP, g.length)
    p.set('shape', ' '.join(f'{x:.6f},{y:.6f}' for x, y in a.coords))
    q = ET.SubElement(root, 'poly', dict(p.attrib))
    q.set('id', f'{CENTER}_b')
    q.set('shape', ' '.join(f'{x:.6f},{y:.6f}' for x, y in b.coords))


def draw(net_path, L, add_path=ADD):
    """직사각형 주차장 바닥(연석) + 칸 선 + 간판 (아파트 주차장)."""
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_wanggil_parking.py 가 생성. 마전주공아파트 주차장 (화면 표시 전용) -->', '<additional>']
    shp = _wr(L['R'].exterior.coords)
    out.append(f'    <poly id="{P_}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" shape="{shp}"/>')
    out.append(f'    <poly id="{P_}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" layer="{L_LOT + 0.1}" '
               f'shape="{shp}"/>')
    for n, (corners, cen, nv, _) in enumerate(L['placed']):
        a, b, c, d = corners                                  # a→b 통로 쪽, d·c 안쪽 끝
        out.append(f'    <poly id="{P_}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
    fr, N = L['fr'], L['N']
    for key, names, color, (u, v) in [('sign', ['P 마전주공아파트 주차장'], (40, 90, 200), (N['IN'][0] + 14.0, -3.5)),
                                      ('in', ['입구'], (30, 140, 70), (N['IN'][0] - 6.0, -3.5)),
                                      ('out', ['출구'], (200, 60, 40), (-6.0, N['X'][1] + 5.0))]:
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


def write_routes(areas, path=ROU, seed=20261115):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- 마전주공아파트 주차장 차 (env/roundabout_wanggil_parking.py): 칸에 주차했다가 나간다 (출구에서 좌·우회전). -->',
             '<routes>',
             f'    <vType id="pw_car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="pw_init_{n}" type="pw_car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(120, 3600)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="pw_f_{pid}" type="pw_car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[k % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'wanggil_init_cars': n, 'wanggil_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_wanggil_scenery as wg
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('마전주공 주차장이 이미 적용됨')
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'wanggil_lot_{k}': v for k, v in rep.items()}
    rep['wanggil_lot_size'] = (round(LOT_W, 1), LOT_D, round(L['R'].area))
    rep['wanggil_stalls'] = place_stalls(net_path, L)
    areas = apply_deco(deco_path, net_path, L)
    draw(net_path, L)
    rep.update(write_routes(areas))
    rep['wanggil_redraw'] = {k: v for k, v in wg.build(net_path).items() if k in ('wg_majeon', 'wg_dimang', 'forest_polys')}
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
