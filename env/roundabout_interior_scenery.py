"""큰 루프 안쪽을 뭉텅이로 채운다: 아파트 단지, 빌라, 상가, 공장 지대, 묘원, 공원, 롯데마트, 산(숲). (화면 표시 전용)

    python -m env.roundabout_interior_scenery

  - 2026-11-11 네이버 지도 캡처 기준. 세밀하지 않게 블록 단위로, 상가마다 이름 하나만.
  - 큰길을 따라 놓는 구역은 그 큰길 바깥 차로 기준 좌표(s: 길 따라, t: 루프 안쪽으로)로 정하고,
    나머지는 SUMO 좌표 다각형으로 정한다. 모든 구역은 도로(차로 + 보도 폭)와 기존 그림(상가·학교)을 뺀다.
  - 남는 땅은 모두 산(숲)으로 칠하고 나무 점을 흩뿌린다.
  - 건물은 env.roundabout_building_style 스타일 (평지붕 + 파라펫 + 테두리 + 그림자), 간판은 북서쪽 상가와 같은 모양.
출력: env/sumo_roundabout/roundabout.interior.add.xml, env/sumo_roundabout/interior_img/*.png
"""
import math
import os
import random
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
IMG_DIR = os.path.join(NET_DIR, 'interior_img')
OUT = os.path.join(NET_DIR, 'roundabout.interior.add.xml')
SIDEWALK = 3.0
# 루프 대략 모양 (큰길 중심선 근처) — 안쪽 땅을 고를 때만 쓴다
LOOP = [(0, 0), (275, 215), (460, 330), (600, 410), (1219, 31), (1420, -100), (1330, -240), (1262, -560),
        (1000, -556), (244, -850), (55, -165)]

GROUND = {'apt': (214, 210, 202), 'villa': (206, 202, 194), 'shops': (204, 200, 192), 'factory': (182, 180, 174),
          'town': (206, 202, 194), 'cemetery': (160, 176, 128), 'park': (146, 182, 112), 'mart': (204, 200, 192),
          'vacant': (218, 215, 207)}      # 빈 땅 (네이버에서 회색 빈터, 2026-11-12)
FOREST = (78, 116, 66)
TREE = (58, 96, 52)
APT_ROOFS = [(222, 220, 214), (214, 214, 210), (226, 222, 212)]
VILLA_ROOFS = [(204, 190, 178), (196, 186, 176), (212, 205, 188), (190, 198, 208), (184, 120, 96)]
TOWN_ROOFS = [(176, 96, 74), (168, 104, 86), (186, 112, 84)]
FACTORY_ROOFS = [(88, 128, 186), (96, 140, 196), (150, 156, 162), (196, 198, 200), (120, 126, 134)]
SHOP_ROOFS = [(206, 204, 198), (214, 205, 188), (190, 198, 208), (204, 190, 178), (220, 216, 208), (198, 188, 200)]
SIGN = {'apt': (40, 110, 70), 'villa': (120, 70, 160), 'shops': (200, 90, 40), 'factory': (60, 90, 150),
        'town': (150, 80, 60), 'cemetery': (90, 110, 60), 'park': (40, 140, 90), 'mart': (220, 40, 40)}

# 구역: (키, 기준, 모양, 종류, 간판 목록, 선택값)
#   기준 'world': 모양 = SUMO 좌표 꼭짓점, 선택값 angle(건물 방향, 도)
#   기준 '완정로' 등: 모양 = (s0, s1, t0, t1)
ZONES = [
    # ── 검단로 남쪽 (북서) ──
    # 117번길 연장(int_117) 바로 남쪽: 이편한세상 104동·110동 (2026-11-11 네이버, 길과 나란히, 동마다 이름 하나).
    # 이편한세상 구역의 빈 북쪽 끝을 먼저 차지한다 (이편한세상 기존 동 배치는 그대로).
    # 새 길과 이편한세상 사이가 좁아, 길에 가장 가까운 이름 없는 기존 동 하나((531, 187) 자리)를 104동·110동이 대신한다
    ('ipyeon_104', 'world', [(522, 262), (545, 236), (602, 236), (602, 196), (520, 196)],
     'apt', ['104동', '110동'], dict(angle=0, slab=(22, 10), gap=(6, 8))),
    ('ipyeon', 'world', [(300, 152), (468, 262), (560, 222), (585, 150), (585, 40), (470, 25), (330, 70)],
     'apt', ['검단대림이편한세상'], dict(angle=34, slab=(52, 14), gap=(14, 30))),
    ('osk', 'world', [(196, 112), (262, 158), (290, 128), (226, 80)], 'shops', ['OSK마트', 'CU'], dict(angle=34)),
    # ── 봉수대로 동쪽 (서) ──
    ('nae_ri', '봉수대로', (600, 660), 'villa', ['내리요양원'], dict(t=(16, 60))),
    ('kia', '봉수대로', (520, 590), 'shops', ['기아오토큐'], dict(t=(16, 70))),
    ('fac_1394', '봉수대로', (440, 515), 'factory', [], dict(t=(16, 270))),
    ('gonggu', '봉수대로', (250, 405), 'shops', ['왕길공구종합상가', '두리종합상가'], dict(t=(16, 110))),
    # 2026-11-12 봉수대로1394번길 안쪽 골목 사이 공장·작업장 (네이버 위성 지도: 골목 사이가 공장 지붕으로 꽉 차 있다)
    ('fac_1394mid', 'world', [(240, -405), (432, -380), (437, -522), (245, -517)], 'factory', [], dict(angle=-8)),
    ('fac_1394n', 'world', [(262, -386), (425, -366), (425, -282), (290, -285)], 'factory', [], dict(angle=-8)),
    ('fac_1394s', 'world', [(172, -536), (437, -530), (442, -600), (184, -604)], 'factory', [], dict(angle=-2)),
    ('fac_sw', '봉수대로', (40, 240), 'factory', [], dict(t=(16, 190))),
    ('fac_mid', 'world', [(455, -400), (632, -360), (632, -610), (470, -715)], 'factory', [], dict(angle=-8)),
    # ── 원당대로 북쪽 (남) ──
    ('tire', 'world', [(675, -350), (872, -405), (882, -585), (680, -662)], 'factory', ['타이어세상', '상주상회'],
     dict(angle=-12)),
    ('yaksu', 'world', [(888, -450), (1052, -452), (1052, -545), (888, -568)], 'shops',
     ['마전도서관', '약수동마을회관', '북경짜장'], dict(angle=0)),
    # 2026-11-13 사용자 요청: 롯데마트를 줄이고(동쪽 변 1150 → 1126) 동쪽에 실제로 드나드는 주차장
    #   (env/roundabout_lotte_parking.py, 3+3 길 끝 네거리). 예전 그림 주차장 'lotte_park' 은 뺀다.
    ('lotte', 'world', [(1060, -448), (1126, -447), (1126, -542), (1060, -546)], 'mart', ['롯데마트'], dict(angle=0)),
    ('kumho', 'world', [(1202, -400), (1252, -372), (1262, -440), (1256, -540), (1202, -536)], 'apt',
     ['검단금호어울림'], dict(angle=0, slab=(30, 12), gap=(8, 14))),
    # ── 완정로65번길 서쪽 (동) ──
    ('fiore', 'world', [(1112, -82), (1290, -118), (1345, -175), (1300, -322), (1205, -350), (1112, -300)], 'apt',
     ['검단피오레대주1차'], dict(angle=-20, slab=(44, 14), gap=(12, 26))),
    ('cemetery', 'world', [(900, -255), (1100, -300), (1110, -405), (950, -398), (885, -360)], 'cemetery',
     ['마전동묘원'], dict(outside='baesuji_vacant')),   # 2026-11-13: 빈 땅 경계(숲 선)를 넘어 서쪽으로 삐져나오지 않게
    # 왕길배수지공원 ← 2026-11-12 네이버 모양·위치로 옮김 (env/roundabout_baesuji_scenery.py)
    # ── 완정로 남쪽 (북동) ──
    ('tower', '완정로', (252, 345), 'apt', ['타워팰리스6차'], dict(t=(8, 62), slab=(30, 13), gap=(8, 18))),
    ('yangji_town', '완정로', (378, 470), 'town', ['양지마을타운하우스'], dict(t=(10, 52))),
    ('wan_strip', '완정로', (482, 600), 'shops', ['S-OIL', '골프존파크', '육대장', '교동짬뽕', '샤브올데이'],
     dict(t=(6, 32), bld=(17, 15))),
    # 빌라 바둑판 ← 2026-11-12 순환 골목(env/roundabout_baesuji.py)을 따라 네이버 배치로 옮김 (env/roundabout_baesuji_scenery.py)
    # 165번길 동쪽 ~ 배수지공원 ~ 동쪽 골목: 숲이 아니라 빈 땅 (네이버 회색 빈터). 마전동묘원 쪽 녹지(동쪽 꼭짓점들 바깥)만 숲.
    # 다른 구역이 먼저 땅을 차지하고 남는 곳만 깐다 (맨 끝)
    ('baesuji_vacant', 'world', [(608, 190), (905, 250), (1110, 160), (1110, -45), (940, -45), (897, -192), (915, -240),
                                 (939, -290), (940, -330), (880, -400), (662, -322), (654, -230), (640, -130),
                                 (624, -20), (612, 90)], 'vacant', [], {}),
]
# 아파트 단지 안 주차장 (접두사, 바꾼 골목 끝 교차점, 그림 파일): 통로는 땅을 가르지 않고 바닥과 겹치는 동만 뺀다
UNDER_LOTS = [('pf_', 'SW_final_right', 'fiore_parking')]     # 2026-11-13 검단피오레대주1차 (env/roundabout_fiore_parking.py)
PARK_LABELS = [('학굴공원', (470, -110)), ('검단대은교회', (1150, -90))]


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _fmt(pts):
    return ' '.join(f'{x:.3f},{y:.3f}' for x, y in pts)


class Line:
    """큰길 바깥 차로를 따라가는 좌표 (s: 길 따라, t: 진행 방향 오른쪽 = 루프 안쪽)."""

    def __init__(self, pts):
        self.p = pts
        self.c = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.c.append(self.c[-1] + math.dist(a, b))

    def P(self, s, t):
        s = max(0.0, min(self.c[-1], s))
        for i in range(len(self.p) - 1):
            if self.c[i+1] >= s:
                a, b = self.p[i], self.p[i+1]
                L = (self.c[i+1] - self.c[i]) or 1.0
                k = (s - self.c[i]) / L
                d = ((b[0]-a[0])/L, (b[1]-a[1])/L)
                return (a[0] + (b[0]-a[0])*k + d[1]*t, a[1] + (b[1]-a[1])*k - d[0]*t)
        return self.p[-1]

    def dir(self, s):
        a, b = self.P(s - 2, 0), self.P(s + 2, 0)
        return math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))

    def band(self, s0, s1, t0, t1, step=5.0):
        n = max(2, int((s1 - s0) / step))
        ss = [s0 + (s1 - s0)*k/n for k in range(n + 1)]
        return [self.P(s, t0) for s in ss] + [self.P(s, t1) for s in ss[::-1]]


def _lines(net):
    lanes = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}

    def chain(ids):
        pts = []
        for eid in ids:
            if f'{eid}_0' in lanes:
                q = lanes[f'{eid}_0']
                pts += q if not pts else q[1:]
        return Line(pts)
    from env.roundabout_shops import Frame
    F = Frame(net)
    wj = Line([F.P(-50, 0), F.P(1000, 0)])
    return {
        '완정로': wj,
        '봉수대로': chain(['S_in1_ext', 'S_in1_ext_int_1', 'S_in1_ext_int_2']),
        '원당대로': chain(['S_link_west', 'S_link_west_int_1', 'S_link_west_int_2']),
    }


def _rect_fill(poly, angle, size, gap, jitter=0.0, rng=None):
    """다각형 안에 angle 방향 격자로 size 사각형을 채운다 (다 들어가는 것만)."""
    from shapely import affinity
    from shapely.geometry import box
    c = poly.centroid
    rp = affinity.rotate(poly, -angle, origin=c)
    x0, y0, x1, y1 = rp.bounds
    w, h = size
    out = []
    y = y0 + gap[1]*0.3
    while y + h <= y1:
        x = x0 + gap[0]*0.3
        while x + w <= x1:
            ww, hh = w, h
            if jitter and rng:
                ww = w*(1 - jitter*rng.random())
                hh = h*(1 - jitter*rng.random())
            b = box(x, y, x + ww, y + hh)
            if rp.contains(b):
                out.append(affinity.rotate(b, angle, origin=c))
            x += w + gap[0]
        y += h + gap[1]
    return out


def _bld(pid, geom, roof, layer=-1.3):
    from env.roundabout_building_style import polys, PARAPET_INSET
    pts = list(geom.exterior.coords)[:-1]
    inner = geom.buffer(-PARAPET_INSET, join_style=2)
    ipts = list(inner.exterior.coords)[:-1] if not inner.is_empty and inner.geom_type == 'Polygon' else pts
    return polys(pid, pts, ipts, roof, layer)


def _sign(key, names, color, pos, font_m=2.4):
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    img = sign_stack(names, color, font_m=font_m)
    fn = f'int_{key}.png'
    img.save(os.path.join(IMG_DIR, fn))
    return (f'    <poi id="int_sign_{key}" type="sign" color="255,255,255" layer="-0.9" x="{pos[0]:.3f}" y="{pos[1]:.3f}" '
            f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" imgFile="interior_img/{fn}"/>')


def _no_holes(geom):
    """SUMO 다각형은 구멍을 못 그리므로, 구멍 가운데를 지나는 선으로 잘라 구멍 없는 조각들로."""
    from shapely.geometry import LineString
    from shapely.ops import split
    todo = [g for g in getattr(geom, 'geoms', [geom]) if g.geom_type == 'Polygon']
    out = []
    while todo:
        g = todo.pop()
        if not g.interiors:
            out.append(g)
            continue
        c = g.interiors[0].centroid
        x0, y0, x1, y1 = g.bounds
        parts = split(g, LineString([(c.x, y0 - 1), (c.x, y1 + 1)]))
        todo += [p for p in parts.geoms if p.geom_type == 'Polygon' and p.area > 1]
    return out


def _ground(pid, geom, color, layer):
    out = []
    for k, g in enumerate(_no_holes(geom)):
        if g.area < 20:
            continue
        out.append(f'    <poly id="{pid}_{k}" type="ground" color="{",".join(map(str, color))}" fill="1" '
                   f'layer="{layer}" shape="{_fmt(g.exterior.coords)}"/>')
    return out


def build(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), out=OUT):
    from shapely.geometry import LineString, Polygon, Point, box
    from shapely.ops import unary_union
    net = ET.parse(net_path).getroot()
    os.makedirs(IMG_DIR, exist_ok=True)
    rng = random.Random(11)
    # 도로 (차로 + 보도 폭)
    roads, paint = [], []
    # 2026-11-13: 아파트 단지 안 주차장 통로(UNDER_LOTS)는 땅을 가르지 않게 뺀다 (단지 동 배치가 그대로 유지되게)
    skip = tuple(pre for pre, _, _ in UNDER_LOTS) + tuple(f':{pre}' for pre, _, _ in UNDER_LOTS)
    skip_j = tuple(j for _, j, _ in UNDER_LOTS)
    for e in net.findall('edge'):
        if e.get('id', '').startswith(skip) or any(e.get('id', '').startswith(f':{j}_') for j in skip_j):
            continue
        for l in e.findall('lane'):
            pts = _read(l.get('shape'))
            if len(pts) >= 2:
                w = float(l.get('width', '3.2'))
                roads.append(LineString(pts).buffer(w/2 + SIDEWALK, cap_style=2))
                paint.append(LineString(pts).buffer(w/2, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and len(_read(s)) >= 3 \
                and not j.get('id', '').startswith(skip) and j.get('id') not in skip_j:
            roads.append(Polygon(_read(s)).buffer(SIDEWALK))
            paint.append(Polygon(_read(s)).buffer(0))
    road = unary_union(roads)
    paint_u = unary_union(paint)
    # 기존 그림 (상가·북서쪽 상가·학교) 자리
    taken = []
    # lotte_parking: 롯데마트 주차장 (있을 때만, 2026-11-13)
    # school: 학교 부지·주차장 바닥 (2026-11-13 — 예전엔 학교 그림 네모만 빼서 부지가 숲·빈 땅 위로 삐져나왔다)
    for f in ('shops', 'nw', 'lotte_parking', 'school'):
        fp = os.path.join(NET_DIR, f'roundabout.{f}.add.xml')
        if not os.path.exists(fp):
            continue
        r = ET.parse(fp).getroot()
        for p in r.findall('poly'):
            if p.get('fill') == '1' and p.get('shape'):
                pts = _read(p.get('shape'))
                if len(pts) >= 3:
                    taken.append(Polygon(pts).buffer(0))
    # 학교 그림 (POI 이미지: 가운데·크기·각도로 네모)
    from shapely import affinity
    sr = ET.parse(os.path.join(NET_DIR, 'roundabout.school.add.xml')).getroot()
    for poi in sr.findall('poi'):
        x, y = float(poi.get('x')), float(poi.get('y'))
        w, h = float(poi.get('width')), float(poi.get('height'))
        rect = box(x - w/2, y - h/2, x + w/2, y + h/2)
        taken.append(affinity.rotate(rect, -float(poi.get('angle', 0)), origin=(x, y)).buffer(1.0))
    taken = unary_union(taken).buffer(2.0)
    # 단지 안 주차장 바닥: 구역 땅은 그대로 두고, 바닥과 겹치는 동만 뺀다
    under = []
    for pre, _, f in UNDER_LOTS:
        fp = os.path.join(NET_DIR, f'roundabout.{f}.add.xml')
        if os.path.exists(fp):
            under += [Polygon(_read(p.get('shape'))).buffer(2.0) for p in ET.parse(fp).getroot().findall('poly')
                      if p.get('id') == f'{pre}lot']
    under = unary_union(under) if under else None
    # 루프 안쪽 땅 = 큰 상자 - 도로 중 루프 안쪽 조각들
    loop = Polygon(LOOP)
    bigbox = loop.buffer(80).envelope
    faces = bigbox.difference(unary_union(paint))
    land = unary_union([g for g in getattr(faces, 'geoms', [faces])
                        if g.area > 200 and loop.contains(g.representative_point())])
    land = land.difference(road).difference(taken)
    L = _lines(net)

    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_interior_scenery.py 가 생성. 큰 루프 안쪽 아파트·상가·공장·산 (주행에는 영향 없음) -->',
             '<additional>']
    used = []
    report = {}
    for key, ref, shape, kind, names, opt in ZONES:
        # 2026-11-13: 구역마다 따로 무작위 (한 구역을 고쳐도 다른 구역 건물·나무가 줄줄이 바뀌지 않게)
        rng = random.Random(f'11:{key}')
        if ref == 'world':
            zone = Polygon(shape)
            angle = opt.get('angle', 0)
        else:
            s0, s1 = shape
            t0, t1 = opt.get('t', (10, 60))
            zone = Polygon(L[ref].band(s0, s1, t0, t1))
            angle = L[ref].dir((s0 + s1)/2)
        if opt.get('outside'):         # 다른 구역 모양 밖으로만 (경계선을 맞춘다)
            zone = zone.difference(Polygon(next(z[2] for z in ZONES if z[0] == opt['outside'])))
        if kind == 'parking':          # 주차장은 도로 끝(입구)에 바로 붙는다 (보도 폭 안 띄움)
            zone = zone.difference(paint_u.buffer(0.2)).difference(taken)
        else:
            zone = zone.intersection(land)
        for u in used:
            zone = zone.difference(u)
        if zone.is_empty:
            report[key] = 'empty'
            continue
        if kind == 'vacant':           # 빈 땅: 길로 나뉜 조각 모두 (건물·간판 없음)
            # 길 건너편으로 남는 가는 띠(평균 폭 = 넓이 / 둘레 절반 < 8m)는 빼서 옆 숲에 맡긴다 (2026-11-13: 길 옆 쐐기처럼 보였다)
            parts = [g for g in getattr(zone, 'geoms', [zone]) if g.geom_type == 'Polygon' and g.area > 50
                     and g.area/(g.length/2) >= 8.0]
            for k, g in enumerate(parts):
                lines += _ground(f'int_{key}{k}', g, GROUND[kind], '-1.8')
                used.append(g.buffer(1.0))
            report[key] = f'{len(parts)} parts'
            continue
        zone = max(getattr(zone, 'geoms', [zone]), key=lambda g: g.area) if zone.geom_type != 'Polygon' else zone
        used.append(zone.buffer(1.0))
        if kind == 'parking':
            lines += _ground(f'int_{key}', zone, (92, 92, 96), '-1.8')
            rects = _rect_fill(zone.buffer(-1.5), angle, (2.5, 5.0), (0.0, 6.0))
            for k, r in enumerate(rects):
                c = list(r.exterior.coords)
                lines.append(f'    <poly id="int_{key}_stall{k}" type="marking" color="235,235,235" fill="0" '
                             f'lineWidth="0.12" layer="-1.75" shape="{_fmt([c[0], c[1]])}"/>')
            report[key] = f'{len(rects)} stalls'
            continue
        lines += _ground(f'int_{key}', zone, GROUND[kind], '-1.8')
        inner = zone.buffer(-3.0)
        if inner.is_empty:
            report[key] = 'thin'
            continue
        inner = max(getattr(inner, 'geoms', [inner]), key=lambda g: g.area)
        blds = []
        if kind == 'apt':
            size = opt.get('slab', (50, 14))
            gap = opt.get('gap', (14, 28))
            blds = [(b, APT_ROOFS[k % len(APT_ROOFS)]) for k, b in enumerate(_rect_fill(inner, angle, size, gap))]
        elif kind == 'villa':
            blds = [(b, VILLA_ROOFS[k % len(VILLA_ROOFS)])
                    for k, b in enumerate(_rect_fill(inner, angle, (15, 13), (6, 6), 0.25, rng))]
        elif kind == 'town':
            blds = [(b, TOWN_ROOFS[k % len(TOWN_ROOFS)])
                    for k, b in enumerate(_rect_fill(inner, angle, (11, 9), (4, 5), 0.15, rng))]
        elif kind == 'shops':
            blds = [(b, SHOP_ROOFS[k % len(SHOP_ROOFS)])
                    for k, b in enumerate(_rect_fill(inner, angle, opt.get('bld', (20, 15)), (4, 5), 0.2, rng))]
        elif kind == 'factory':
            cand = _rect_fill(inner, angle, (30, 18), (7, 9), 0.35, rng)
            blds = [(b, FACTORY_ROOFS[rng.randrange(len(FACTORY_ROOFS))]) for b in cand if rng.random() < 0.82]
        elif kind == 'mart':
            b = inner.buffer(-4.0, join_style=2)
            if not b.is_empty:
                blds = [(b.minimum_rotated_rectangle, (210, 214, 218))]
        elif kind == 'cemetery':
            rows = _rect_fill(inner.buffer(-2), angle + 15, (1.6, 1.0), (2.4, 3.0))
            for k, r in enumerate(rows):
                lines.append(f'    <poly id="int_{key}_grave{k}" type="ground" color="196,198,186" fill="1" '
                             f'layer="-1.7" shape="{_fmt(r.exterior.coords)}"/>')
        elif kind == 'park':
            c = inner.centroid
            ring = Point(c.x, c.y).buffer(min(25.0, math.sqrt(inner.area)/3)).exterior
            path = LineString(list(ring.coords)).intersection(inner)
            for k, g in enumerate(getattr(path, 'geoms', [path])):
                if g.geom_type == 'LineString':
                    lines.append(f'    <poly id="int_{key}_path{k}" type="path" color="226,214,180" fill="0" '
                                 f'lineWidth="2.0" layer="-1.7" shape="{_fmt(g.coords)}"/>')
        if under is not None:
            blds = [(b, roof) for b, roof in blds if not b.intersects(under)]
        for k, (b, roof) in enumerate(blds):
            lines += _bld(f'int_{key}_b{k}', b, roof)
        # 간판: 단지·구역 이름 하나, 상가 구역은 건물마다 이름 하나 (이름 수만큼 고르게)
        if names:
            if kind in ('shops', 'villa', 'factory', 'apt') and len(names) > 1 and len(blds) > 1:
                # 건물 하나에 이름 하나. 건물이 이름보다 적으면 남는 이름은 뺀다 (한 건물에 쌓지 않음)
                n = min(len(names), len(blds))
                if n < len(names):
                    print(f'[경고] {key}: 건물 {len(blds)}개 < 이름 {len(names)}개 → {names[n:]} 생략')
                picks = [blds[round(i*(len(blds) - 1)/max(1, n - 1))][0] for i in range(n)]
                for i, (nm, b) in enumerate(zip(names[:n], picks)):
                    c = b.centroid
                    lines.append(_sign(f'{key}_{i}', [nm], SIGN[kind], (c.x, c.y), font_m=2.0))
            elif kind in ('shops', 'villa', 'factory', 'apt') and blds and len(names) > 1:
                print(f'[경고] {key}: 건물 {len(blds)}개 < 이름 {len(names)}개 → {names[1:]} 생략')
                c = blds[0][0].centroid
                lines.append(_sign(key, names[:1], SIGN[kind], (c.x, c.y), font_m=2.0))
            elif kind in ('shops', 'villa', 'factory') and blds:
                c = blds[len(blds)//2][0].centroid
                lines.append(_sign(key, names, SIGN[kind], (c.x, c.y), font_m=2.0))
            elif len(names) > 1 and kind == 'apt':
                print(f'[경고] {key}: 건물이 들어가지 않아 간판 생략 ({names})')
            else:
                c = zone.representative_point() if kind != 'apt' else inner.centroid
                if not zone.contains(c):
                    c = zone.representative_point()
                lines.append(_sign(key, names, SIGN[kind], (c.x, c.y), font_m=3.2 if kind == 'apt' else 2.6))
        report[key] = len(blds)
    # 산(숲): 남는 땅 전부
    forest = land
    for u in used:
        forest = forest.difference(u.buffer(1.0))
    n_forest = 0
    for k, g in enumerate(getattr(forest, 'geoms', [forest])):
        if g.geom_type != 'Polygon' or g.area < 20:
            continue
        if g.area < 300:               # 작은 남는 조각도 숲 바닥으로 (빈 바탕이 쐐기처럼 보이지 않게, 나무 점은 없이)
            lines.append(f'    <poly id="int_forest_{n_forest}" type="forest" color="{",".join(map(str, FOREST))}" '
                         f'fill="1" layer="-1.85" shape="{_fmt(g.exterior.coords)}"/>')
            n_forest += 1
            continue
        g2 = g.buffer(-1.0)
        if g2.is_empty:
            continue
        for gg in _no_holes(g2):
            cx, cy = gg.representative_point().coords[0]
            rng = random.Random(f'11:forest:{round(cx, -1)}:{round(cy, -1)}')     # 숲 조각마다 따로 (위와 같은 이유)
            lines.append(f'    <poly id="int_forest_{n_forest}" type="forest" color="{",".join(map(str, FOREST))}" '
                         f'fill="1" layer="-1.85" shape="{_fmt(gg.exterior.coords)}"/>')
            n_forest += 1
            # 나무 점 (어두운 동그라미)
            x0, y0, x1, y1 = gg.bounds
            sp = 14.0
            y = y0 + sp/2
            while y < y1:
                x = x0 + sp/2 + (sp/2 if int((y - y0)/sp) % 2 else 0)
                while x < x1:
                    px, py = x + rng.uniform(-4, 4), y + rng.uniform(-4, 4)
                    r = rng.uniform(2.2, 4.2)
                    if gg.contains(Point(px, py).buffer(r)):
                        t = Point(px, py).buffer(r, 6)
                        lines.append(f'    <poly id="int_tree_{n_forest}_{len(lines)}" type="tree" '
                                     f'color="{",".join(map(str, TREE))}" fill="1" layer="-1.82" '
                                     f'shape="{_fmt(t.exterior.coords)}"/>')
                    x += sp
                y += sp
    for i, (nm, pos) in enumerate(PARK_LABELS):
        lines.append(_sign(f'label{i}', [nm], SIGN['park'], pos, font_m=2.8))
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    report['forest_polys'] = n_forest
    report['out'] = out
    return report


if __name__ == '__main__':
    import sys
    sys.path.insert(0, BASE)
    print(build())
