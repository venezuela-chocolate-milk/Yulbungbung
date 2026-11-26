"""왕길유승한내들아파트 (608세대) 이식 + 단지 주차장 (2026-11-14 사용자 요청, 네이버 위성 지도 캡처).

  - 자리: 회전교차로 북서쪽 빈 땅 (동쪽 = 북쪽에서 내려오는 대각선 길 N_in1_far·nw_hs1, 남쪽 = 단봉로 DANBONG).
    실제처럼 동은 대각선 길(실제는 인천2호선 철길)과 나란히: 동쪽 줄 101·102·103동, 서쪽 줄 104·105동.
  - 틀: u = 대각선 길 방향(북북서), v = u 왼쪽(서남서, 단지 안쪽). 원점 = 단지 남동 모서리.
  - 주차장 (단지 안 전체가 주차장): 동 사이·서쪽 둘레 통로 3개(u 방향) + 남·북 가로 통로, 통로 양옆 직각 칸
    (env.roundabout_wanggil_south_parking.place_stalls 같은 규칙, 동 자리에는 칸을 놓지 않음).
    두 줄 동 사이 통로 a0·a1, 그 서쪽 놀이터·테니스장 띠, 맨 서쪽 둘레 통로 a2.
  - 정문 (입구 = 출구): 단지 남서쪽, 단봉로가 회전교차로 쪽으로 갈라지기(DANBONG_split_in) 전.
    단봉로 두 방향(DANBONG_outfar 서행 2차로 · DANBONG_infar 동행 3차로)을 한 교차점으로 묶어
      들어오기: 서행 → 우회전, 동행 → 좌회전 /  나가기: 좌회전 → 동행(→ DANBONG_diag → W_in1, 회전교차로 서쪽 아래 진입),
      우회전 → 서행.  좌회전이 넘도록 정문 앞 노란 이중 중앙선(centerline_split_0/1)을 끊는다 (사용자 요청: 가장 중요).
    정문 교차점은 고정 신호 (위성 사진의 정문 앞 횡단보도 자리): 신호 없이는 나가는 차가 단봉로 5개 차로를 양보하며
    최대 837초 기다리고 순간이동까지 됐다 (2026-11-14 시험). 단계 A 본선 직진 + 서행 우회전 진입,
    단계 B 정문 진출(좌·우회전) + 동행 좌회전 진입 — env.roundabout_black_t_signal 처럼 교차점 모양은 두고 신호 계획만.
  - 그림: roundabout.yuseung.add.xml (단지 바닥·동·놀이터·테니스장·주차장 바닥·칸 선·간판), 차: roundabout.yuseung.rou.xml.

roundabout_south_extend 파이프라인에서 apply 를 부른다.  단독 실행: python -m env.roundabout_yuseung
"""
import math
import os
import random
import xml.etree.ElementTree as ET

from env.roundabout_parking_s import (_rd, _wr, AH, ST_D, CAR_LEN, SPEED, ASPHALT, LINE, CURB_COLOR, L_LOT, L_LINE,
                                      CAR_COLORS)
from env.roundabout_wanggil_parking import Frame, EXIT_ARROW_DIST

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.yuseung.add.xml')
ROU = os.path.join(NET_DIR, 'roundabout.yuseung.rou.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
P_ = 'pu_'
NAME = '왕길유승한내들아파트'
SIDEWALK = 3.0
ROAD_F, ROAD_R = 'DANBONG_infar', 'DANBONG_outfar'     # 단봉로 동행(회전교차로 쪽, 3차로) · 서행(2차로)
CENTERS = ('centerline_split_0', 'centerline_split_1')  # 단봉로 노란 이중 중앙선
CENTER_GAP = 7.0
DIAG = ((-27.0, 53.0), (-105.0, 267.0))                 # 대각선 길 방향 (남 → 북)
HINT = (-24.0, 36.0)                                    # 단지 남동 모서리 대략 (여기서 서·북으로 밀며 자리 찾기)
# 단지 크기·통로 (t = 대각선 길에서 서쪽으로, s = 남에서 북으로)
L_U = 160.0                                             # 남쪽 15m 는 정문 진입로 자리 (DRV_SHIFT)
ROW_A = (3.0, 17.0)                                     # 동쪽 줄 동 (101·102·103)
A0 = ROW_A[1] + 1.5 + ST_D + AH                         # 두 줄 사이 통로
ROW_B = (A0 + AH + ST_D + 1.5, A0 + AH + ST_D + 1.5 + 14.0)   # 서쪽 줄 동 (104·105)
A1 = ROW_B[1] + 1.5 + ST_D + AH                         # 서쪽 줄 서쪽 통로
AMEN = (A1 + AH + ST_D + 1.0, A1 + AH + ST_D + 17.0)   # 놀이터·테니스장 띠
A2 = AMEN[1] + 1.0 + ST_D + AH                          # 맨 서쪽 둘레 통로 (정문은 이 통로 남쪽 끝)
W_V = A2 + AH + ST_D + 0.5
S_S, S_N = 0.5 + ST_D + AH, L_U - (0.5 + ST_D + AH)    # 남·북 가로 통로
# 동: (번호, 줄, s 범위) — 실제 배치 (남쪽이 103·105)
BLDGS = [('103', ROW_A, (16.0, 52.0)), ('102', ROW_A, (62.0, 98.0)), ('101', ROW_A, (108.0, 144.0)),
         ('105', ROW_B, (18.0, 68.0)), ('104', ROW_B, (80.0, 144.0))]
DRV_SHIFT = 15.0             # 진입로 길이를 늘리려고 단지 남쪽 끝을 북쪽으로 미는 거리 (m, 북쪽 끝은 그대로)
DRV_MEDIAN = 2.0             # 진입로 들어오는·나가는 차로 사이 분리대 폭 (m)
ORIGINS = ['DANBONG_infar', 'N_in1_far', 'W_in1_far']
DESTS = ['DANBONG_outfar', 'N_out0_far', 'W_out0_far']
# 정문은 회전교차로 갈림 24m 앞이라 회전교차로 쪽 줄이 밀려오면 한 번에 몇 대만 나간다 → 입주민 차 수요를 낮게 (2026-11-14)
INIT_OCC, TARGET_OCC = 0.35, 0.25
DURATIONS = [3600, 7200, 10800, 14400]


def _unit(a, b):
    d = (b[0] - a[0], b[1] - a[1])
    L = math.hypot(*d)
    return (d[0]/L, d[1]/L)


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
    from shapely.ops import unary_union
    keep = _keep(net)
    u = _unit(*DIAG)
    fr0 = Frame(HINT, u)                     # v = u 왼쪽 = 서남서
    found = None
    for ds in [x*0.5 for x in range(0, 120)]:
        for dt in [x*0.5 for x in range(0, 60)]:
            fr = Frame(fr0.P(ds, dt), u)
            # 진입로 자리(DRV_SHIFT)까지 포함한 길이로 찾는다 (짧은 단지로 찾으면 북쪽이 도로에 더 붙는다)
            rect = Polygon([fr.P(0, 0), fr.P(L_U + DRV_SHIFT, 0), fr.P(L_U + DRV_SHIFT, W_V), fr.P(0, W_V)])
            if not rect.intersects(keep):
                found = (fr, rect)
                break
        if found:
            break
    if not found:
        raise RuntimeError('유승한내들 자리를 못 찾음')
    fr, rect = found
    # 정문 진입로가 보이게(정지선·화살표·분리대) 단지를 대각선 길 방향으로 DRV_SHIFT 만큼 더 북쪽으로 (2026-11-14)
    fr = Frame(fr.P(DRV_SHIFT, 0.0), u)
    rect = Polygon([fr.P(0, 0), fr.P(L_U, 0), fr.P(L_U, W_V), fr.P(0, W_V)])
    if rect.intersects(keep):
        raise RuntimeError('유승한내들: 북쪽으로 옮긴 자리가 도로와 겹침')
    rect_st = lambda s0, s1, t0, t1: Polygon([fr.P(s0, t0), fr.P(s1, t0), fr.P(s1, t1), fr.P(s0, t1)])
    blds = [(no, rect_st(s0, s1, r[0], r[1])) for no, r, (s0, s1) in BLDGS]
    amen = rect_st(S_S + AH + ST_D + 1.0, S_N - AH - ST_D - 1.0, AMEN[0], AMEN[1])
    N = {}
    for k, t in enumerate((A0, A1, A2)):
        N[f'A{k}S'], N[f'A{k}N'] = (S_S, t), (S_N, t)
    streets = [(f'a{k}', f'A{k}S', f'A{k}N') for k in range(3)]
    streets += [('s0', 'A0S', 'A1S'), ('s1', 'A1S', 'A2S'), ('n0', 'A0N', 'A1N'), ('n1', 'A1N', 'A2N')]
    # 칸을 놓을 땅: 단지 직사각형에서 동·놀이터 띠를 뺀 곳
    land = rect.difference(unary_union([b.buffer(1.0, join_style=2) for _, b in blds] + [amen.buffer(0.5, join_style=2)]))
    return dict(lp=P_, name=NAME, fr=fr, R=land, rect=rect, N=N, streets=streets, blds=blds, amen=amen)


def _pieces():
    """정문 교차점으로 나뉜 단봉로 조각 (interior.plan 규칙: 시작 쪽이 원래 이름)."""
    return {ROAD_F: (ROAD_F, f'{ROAD_F}_{P_}1'), ROAD_R: (ROAD_R, f'{ROAD_R}_{P_}1')}


def cfg(net, L):
    fr, N = L['fr'], L['N']
    nodes = {f'{P_}n{k}': fr.P(*v) for k, v in N.items()}
    streets = [(f'{P_}{nm}', f'{P_}n{a}', f'{P_}n{b}', 1 if nm.startswith('a') else 0, []) for nm, a, b in L['streets']]
    gate = fr.P(*N['A2S'])
    near = (gate[0] + 6.0, -6.0)                 # 정문: a2 남쪽 끝에서 단봉로로 거의 수직
    L['gate_near'] = near
    streets.append((f'{P_}drv', f'{P_}mid_in', f'{P_}nA2S', 0, []))
    mids = [(f'{P_}mid_in', (ROAD_F, ROAD_R), near)]
    (fb, fa), (rb, ra) = _pieces()[ROAD_F], _pieces()[ROAD_R]
    nf = len(next(e for e in net.findall('edge') if e.get('id') == ROAD_F).findall('lane'))
    extra = [(rb, f'{P_}drv_f', 0, 0),                 # 서행 → 우회전 진입
             (fb, f'{P_}drv_f', nf - 1, 0),            # 동행 → 좌회전 진입 (맨 왼쪽 차로)
             (f'{P_}drv_r', ra, 0, 0)]                 # 우회전 진출 → 서행
    # 좌회전 진출 → 동행 모든 차로 (→ 회전교차로). 맨 왼쪽 차로 하나만 이으면 그 차로가 회전교차로 쪽 줄로
    # 차 있을 때 나가지 못해 단지 안까지 막혔다 (2026-11-14 시험)
    extra += [(f'{P_}drv_r', fa, 0, k) for k in range(nf)]
    return dict(TEES=[], REBUILD={}, NODES=nodes, STREETS=streets, PREFIX=P_, MEDIAN={f'{P_}drv': DRV_MEDIAN},
                DOUBLE_CENTER=set(), MIDS=mids, ONEWAY=set(), EXTRA_CON=extra, SPEED=SPEED)


# 감응 신호 (초): (최소, 최대) 녹색
SIG_A, SIG_B = (35, 35), (20, 20)       # 고정 신호 (본선, 정문). 합류부 신호(env.roundabout_w_merge_signal)와 같은 65초 주기
YELLOW, ALLRED = 3, 2


def add_signal(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    T = f'{P_}mid_in'
    if any(t.get('id') == T for t in root.findall('tlLogic')):
        return {'yuseung_signal': 'already'}
    j = next(x for x in root.findall('junction') if x.get('id') == T)
    ints = j.get('intLanes').split()
    (fb, fa), (rb, ra) = _pieces()[ROAD_F], _pieces()[ROAD_R]
    # 2단 회전(교차점 안 대기 지점)은 첫 내부 차로 → 다음 내부 차로로 이어지고 intLanes 에는 뒤쪽이 들어 있다
    chain = {f"{c.get('from')}_{c.get('fromLane')}": c.get('via') for c in root.findall('connection')
             if c.get('from', '').startswith(f':{T}_') and c.get('via')}

    def link(c):
        v = c.get('via', '')
        while v not in ints and v in chain:
            v = chain[v]
        return ints.index(v) if v in ints else None

    kind = {}
    for c in root.findall('connection'):
        if not c.get('from', '').startswith(':') and link(c) is not None:
            f, t = c.get('from'), c.get('to')
            k = link(c)
            kind[k] = ('main' if (f, t) in ((fb, fa), (rb, ra)) else
                       'right_in' if (f, t) == (rb, f'{P_}drv_f') else 'left_in' if (f, t) == (fb, f'{P_}drv_f') else
                       'exit' if f == f'{P_}drv_r' else 'other')
    assert len(kind) == len(ints) and 'other' not in kind.values(), kind
    st = lambda G=(), g=(), y=(): ''.join('y' if kind[k] in y else 'G' if kind[k] in G else
                                          'g' if kind[k] in g else 'r' for k in range(len(ints)))
    # 동행 좌회전 진입은 서행이 멈춘 B 에서만: A 에서 양보 진행으로 두면 교차점 한가운데서 서행 틈을 기다리다
    # B 로 바뀐 뒤에도 남아 나가는 차를 막았다 (2026-11-14 시험)
    plan = [(SIG_A, st(G=('main',), g=('right_in',))), ((YELLOW, YELLOW), st(y=('main', 'right_in'))),
            ((ALLRED, ALLRED), st()), (SIG_B, st(G=('exit',), g=('left_in',))),
            ((YELLOW, YELLOW), st(y=('exit', 'left_in'))), ((ALLRED, ALLRED), st())]
    j.set('type', 'traffic_light')
    first = plan[0][1]
    for c in root.findall('connection'):
        if not c.get('from', '').startswith(':') and link(c) is not None:
            k = link(c)
            c.set('tl', T)
            c.set('linkIndex', str(k))
            c.set('state', first[k])                    # env.roundabout_black_t_signal 과 같게 (첫 단계 신호)
    tl = ET.Element('tlLogic', {'id': T, 'type': 'static', 'programID': '0', 'offset': '0'})
    for (mn, mx), state in plan:
        ET.SubElement(tl, 'phase', {'duration': str(mn), 'minDur': str(mn), 'maxDur': str(mx), 'state': state})
    others = root.findall('tlLogic')
    anchor = others[-1] if others else root.findall('junction')[0]
    root.insert(list(root).index(anchor) + (1 if others else 0), tl)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'yuseung_signal': 'added', 'yuseung_links': len(ints)}


def _drv_island(root, net):
    """진입로 들어오는·나가는 차로 사이 분리대 (연석 + 잔디). 양 끝 교차점 쪽은 1.5m 비운다."""
    from shapely.geometry import LineString
    from shapely.ops import substring
    for p in [p for p in root.findall('poly') if p.get('id', '').startswith(f'{P_}island')]:
        root.remove(p)
    lane = {l.get('id'): _rd(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    a, b = lane[f'{P_}drv_f_0'], lane[f'{P_}drv_r_0'][::-1]
    n = min(len(a), len(b))
    if n < 2:
        return
    g = LineString([((p[0] + q[0])/2, (p[1] + q[1])/2) for p, q in zip(a[:n], b[:n])])
    g = substring(g, 1.5, g.length - 1.5)
    curb = g.buffer(DRV_MEDIAN/2 - 0.2, cap_style=2)
    grass = g.buffer(DRV_MEDIAN/2 - 0.5, cap_style=2)
    ET.SubElement(root, 'poly', dict(id=f'{P_}island_curb', type='island', color='204,200,192', fill='1', layer='5',
                                      shape=_wr(curb.exterior.coords)))
    ET.SubElement(root, 'poly', dict(id=f'{P_}island_grass', type='island', color='104,146,84', fill='1', layer='5.1',
                                      shape=_wr(grass.exterior.coords)))


def _gap_centers(root, net):
    """정문 교차점 양쪽 CENTER_GAP 만큼 단봉로 노란 이중 중앙선을 끊는다 (좌회전 진출·진입이 넘는 자리)."""
    from shapely.geometry import LineString, Point
    from shapely.ops import substring
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    polys = {p.get('id'): p for p in root.findall('poly')}
    n = 0
    for cid in CENTERS:
        if cid not in polys or f'{cid}_{P_}b' in polys:
            continue
        p = polys[cid]
        g = LineString(_rd(p.get('shape')))
        s = g.project(Point(J[f'{P_}mid_in']))
        a, b = substring(g, 0, s - CENTER_GAP), substring(g, s + CENTER_GAP, g.length)
        p.set('shape', _wr(a.coords))
        q = ET.SubElement(root, 'poly', dict(p.attrib))
        q.set('id', f'{cid}_{P_}b')
        q.set('shape', _wr(b.coords))
        n += 1
    return n


def apply_deco(deco_path, net_path, L):
    from env.roundabout_builder import _stop_line, _read_lane_shapes, _arrow
    from env.roundabout_parking_s import ST_W, FRONT_GAP
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
                                                     friendlyPos='true', name=f'{NAME} 주차장'))
        for cen, n in items:
            k = ST_D/2 - FRONT_GAP
            navi = math.degrees(math.atan2(n[0], n[1])) % 360
            ET.SubElement(pa, 'space', dict(x=f'{cen[0] + n[0]*k:.3f}', y=f'{cen[1] + n[1]*k:.3f}', width=f'{ST_W:.2f}',
                                             length=f'{ST_D:.2f}', angle=f'{(180.0 - navi) % 360:.2f}'))
        areas[pa.get('id')] = (edge, len(items))
    rr_edges = sorted(e.get('id') for e in net.findall('edge') if e.get('id', '').startswith(P_)
                      and e.get('function') != 'internal' and 'drv' not in e.get('id'))
    rr = ET.SubElement(root, 'rerouter', dict(id=f'{P_}reroute', edges=' '.join(rr_edges), vTypes=f'{P_}car'))
    iv = ET.SubElement(rr, 'interval', dict(begin='0', end='1e9'))
    for pid in areas:
        ET.SubElement(iv, 'parkingAreaReroute', dict(id=pid, visible='true'))
    marks = []
    shapes = _read_lane_shapes(net_path)
    _stop_line(marks, f'{P_}stop_out', shapes, f'{P_}drv_r', dist=1.0)
    for eid in (ROAD_F, ROAD_R):                               # 신호 앞 단봉로 정지선
        _stop_line(marks, f'{P_}stop_{eid}', shapes, eid, dist=1.0)
    for kind in ('left', 'right'):
        _arrow(marks, f'{P_}arrow_out_{kind}', shapes, f'{P_}drv_r', 0, kind, EXIT_ARROW_DIST)
    _arrow(marks, f'{P_}arrow_in', shapes, f'{P_}drv_f', 0, 'straight', 5.0)     # 들어가는 차로: 직진
    _drv_island(root, net)
    for m in marks:
        root.append(ET.fromstring(m))
    gaps = _gap_centers(root, net)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return areas, gaps


def draw(L, add_path=ADD):
    """단지 바닥 + 주차장 바닥(아스팔트) + 칸 선 + 동 5개(동 번호) + 놀이터·테니스장 + 단지 이름·주차장 간판."""
    from shapely.geometry import Point
    from env import roundabout_interior_scenery as isc
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    fr = L['fr']
    _sign0 = isc._sign
    # 간판은 주차장 바닥(층 L_LOT) 위에 보이게 (기본 층 -0.9 는 바닥에 가린다)
    isc_sign = lambda *a, **k: _sign0(*a, **k).replace('layer="-0.9"', f'layer="{L_LINE + 1}"')
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           f'<!-- env/roundabout_yuseung.py 가 생성. {NAME} 단지·주차장 (화면 표시 전용) -->', '<additional>']
    out += isc._ground(f'{P_}site', L['rect'].buffer(2.0, join_style=2), isc.GROUND['apt'], '-1.9')
    out.append(f'    <poly id="{P_}lot" type="parking" color="{ASPHALT}" fill="1" layer="{L_LOT}" '
               f'shape="{_wr(L["R"].exterior.coords)}"/>')
    out.append(f'    <poly id="{P_}lot_curb" type="parking" color="{CURB_COLOR}" fill="0" lineWidth="0.3" '
               f'layer="{L_LOT + 0.1}" shape="{_wr(L["rect"].exterior.coords)}"/>')
    for n, (corners, cen, nv, _) in enumerate(L['placed']):
        a, b, c, d = corners
        out.append(f'    <poly id="{P_}stall_{n}" type="parking_line" color="{LINE}" fill="0" lineWidth="0.12" '
                   f'layer="{L_LINE}" shape="{_wr([a, d, c, b])}"/>')
    # 동 (지붕 + 동 번호)
    for k, (no, b) in enumerate(L['blds']):
        for line in isc._bld(f'{P_}b{no}', b, isc.APT_ROOFS[k % len(isc.APT_ROOFS)], layer=L_LINE + 0.5):
            out.append(line)
        c = b.centroid
        out.append(isc_sign(f'{P_}no{no}', [f'{no}동'], (90, 90, 90), (c.x, c.y), font_m=2.0))
    # 놀이터(북쪽 절반, 모래색) · 테니스장(남쪽 절반, 초록)
    s0, s1 = S_S + AH + ST_D + 1.0, S_N - AH - ST_D - 1.0
    mid = (s0 + s1)/2
    from shapely.geometry import Polygon
    P = lambda s, t: fr.P(s, t)
    play = Polygon([P(mid + 2, AMEN[0]), P(s1, AMEN[0]), P(s1, AMEN[1]), P(mid + 2, AMEN[1])])
    tennis = Polygon([P(s0, AMEN[0]), P(mid - 2, AMEN[0]), P(mid - 2, AMEN[1]), P(s0, AMEN[1])])
    out += isc._ground(f'{P_}amen', L['amen'], isc.GROUND['park'], str(L_LINE + 0.2))
    out += isc._ground(f'{P_}play', play.buffer(-1.0), (222, 206, 160), str(L_LINE + 0.3))
    out += isc._ground(f'{P_}tennis', tennis.buffer(-1.0), (84, 140, 96), str(L_LINE + 0.3))
    out.append(f'    <poly id="{P_}tennis_line" type="marking" color="245,245,240" fill="0" lineWidth="0.2" '
               f'layer="{L_LINE + 0.35}" shape="{_wr(tennis.buffer(-3.0).exterior.coords)}"/>')
    c = play.centroid
    out.append(isc_sign(f'{P_}play_sign', ['어린이놀이터'], (40, 140, 90), (c.x, c.y), font_m=1.6))
    c = tennis.centroid
    out.append(isc_sign(f'{P_}tennis_sign', ['테니스장'], (40, 140, 90), (c.x, c.y), font_m=1.6))
    # 단지 이름 (두 줄 동 사이 가운데) · 정문 주차장 간판
    c = fr.P((S_S + S_N)/2, (A0 + ROW_B[0])/2 + 2.0)
    out.append(isc_sign(f'{P_}name', [NAME], isc.SIGN['apt'], c, font_m=3.0))
    img = sign_stack(['정문 · P 입주민 주차장'], (40, 90, 200), font_m=1.6)
    fn = f'{P_}gate.png'
    img.save(os.path.join(IMG_DIR, fn))
    x, y = fr.P(S_S - 6.0, A2 + 10.0)
    out.append(f'    <poi id="{P_}gate" type="shop" color="255,255,255" layer="{L_LINE + 1}" x="{x:.3f}" y="{y:.3f}" '
               f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="shop_img/{fn}"/>')
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')


def write_routes(areas, path=ROU, seed=20261114):
    rnd = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             f'<!-- {NAME} 주차장 차 (env/roundabout_yuseung.py): 정문으로 들어와 칸에 주차했다가 정문으로 나간다. -->',
             '<routes>',
             f'    <vType id="{P_}car" carFollowModel="IDM" accel="2.0" decel="4.0" maxSpeed="16.67" speedDev="0.1" '
             f'vClass="passenger" guiShape="passenger/sedan" length="{CAR_LEN}" color="200,200,205"/>']
    n = 0
    for pid, (edge, cap) in areas.items():
        for _ in range(cap):
            if rnd.random() < INIT_OCC:
                lines.append(f'    <trip id="{P_}init_{n}" type="{P_}car" depart="0" departPos="stop" from="{edge}" '
                             f'to="{rnd.choice(DESTS)}" color="{rnd.choice(CAR_COLORS)}">')
                lines.append(f'        <stop parkingArea="{pid}" duration="{rnd.randint(600, 14400)}"/>')
                lines.append('    </trip>')
                n += 1
    rate = 0.0
    for k, (pid, (edge, cap)) in enumerate(areas.items()):
        dur = DURATIONS[k % len(DURATIONS)]
        pr = TARGET_OCC*cap/dur
        rate += pr
        lines.append(f'    <flow id="{P_}f_{pid}" type="{P_}car" begin="0" end="100000" probability="{pr:.5f}" '
                     f'from="{ORIGINS[k % len(ORIGINS)]}" to="{DESTS[k % len(DESTS)]}" departLane="best" '
                     f'departSpeed="max" color="{CAR_COLORS[k % len(CAR_COLORS)]}">')
        lines.append(f'        <stop parkingArea="{pid}" duration="{dur}"/>')
        lines.append('    </flow>')
    lines.append('</routes>')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')
    return {'yuseung_init_cars': n, 'yuseung_arrivals_per_h': round(rate*3600, 1)}


def apply(net_path, deco_path):
    from env import roundabout_south_link as link
    from env import roundabout_interior as it
    from env import roundabout_wanggil_south_parking as wsp
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(P_) for e in net.findall('edge')):
        raise RuntimeError('유승한내들 주차장이 이미 적용됨')
    L = layout(net)
    rep = it.with_cfg(cfg(net, L), it.apply_net, net_path)
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    rep = {f'yuseung_{k}': v for k, v in rep.items()}
    rep.update(add_signal(net_path))
    rep['yuseung_site'] = (L_U, round(W_V, 1))
    rep['yuseung_stalls'] = wsp.place_stalls(net_path, L)
    areas, gaps = apply_deco(deco_path, net_path, L)
    rep['yuseung_center_gaps'] = gaps
    draw(L)
    rep.update(write_routes(areas))
    J = {j.get('id'): (round(float(j.get('x')), 1), round(float(j.get('y')), 1))
         for j in ET.parse(net_path).getroot().findall('junction') if j.get('id') == f'{P_}mid_in'}
    rep['yuseung_gate'] = J.get(f'{P_}mid_in')
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml'), os.path.join(NET_DIR, 'roundabout.deco.xml')))
