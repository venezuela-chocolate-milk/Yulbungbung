"""검단사거리 남쪽 상가 블록을 네이버 지도(2026-11-11 캡처) 배치대로 구역 + 건물로 채운다. (화면 표시 전용)

  - 블록 A: 검단로·완정로·117번길·117 골목·C1 으로 둘러싸인 ㄱ자 띠 (네이버 지도의 한 덩어리, 2026-11-11).
            모퉁이에서 두 다리(검단로 쪽, 완정로 쪽)로 나눠 각 다리 방향으로 구역을 나눈다.
    블록 S: 117 골목 아래, V1 골목 서쪽 (다음 덩어리에서 다시 정리 — 지금은 기존 세 곳 유지)
  - 가게 위치: 네이버 캡처 픽셀(1px ≈ 0.30m)을 검단사거리 기준 닮음 변환으로 옮긴 뒤, 블록마다 상자 비율로 블록 안에 넣는다
    (SUMO 상가 골목이 실제와 비율이 달라 절대 위치 대신 블록 안 상대 배치를 따른다).
  - 구역: 네이버 배치의 길이 방향 순서대로 블록을 같은 면적의 띠로 나눈다. 구역 안에 네모 건물 하나(env.roundabout_building_style), 간판 하나.
    공영주차장은 건물 대신 주차장 바닥(주차 칸 + P 간판).
  - 건물·간판 규칙은 북서쪽 상가 구역(env.roundabout_nw_buildings)과 같다.
env.roundabout_shop_buildings.build 가 불러 roundabout.shops.add.xml 에 덧붙인다.
"""
import math
import os

# 네이버 캡처(2026-11-11) 픽셀 → SUMO: 검단사거리·117번길 입구·효담요양원·호텔 브리즈로 맞춘 닮음 변환 (m/px ≈ 0.302)
NAVER = dict(a=(0.30229, 0.00848), b=None)        # b 는 실행 때 검단사거리 중심 기준으로 정한다
NAVER_CENTER_PX = (560, 140)

SIDEWALK = 2.5
CUT_W = 4.0                  # 뷰티풀·큰손왕만두 사이 잔디 띠 폭 (m)
GRASS = '112,140,96'         # 바탕 잔디색 (roundabout.gui.xml backgroundColor)
MAX_ASPECT = 2.5
R, O, G, B, P_, Y, BR = ((200, 40, 60), (210, 100, 30), (40, 140, 90), (30, 80, 160), (120, 70, 160),
                         (220, 170, 20), (150, 90, 40))
# (블록, 키, 간판, 네이버 아이콘 픽셀, 간판색, 종류)  — 간판 이름은 2026-11-11 사용자 지정으로 바꿈 (위치는 그대로)
SHOPS = [
    ('A', 'sisaem', '시샘', (180, 445), (190, 60, 50), 'shop'),
    ('A', 'burgerking', '버거킹', (282, 393), O, 'shop'),
    ('A', 'paikdabang', '백다방', (348, 346), (40, 70, 140), 'shop'),
    ('A', 'luivita', '루이비타베이커리', (383, 357), BR, 'shop'),
    ('A', 'frank', '프랭크버거', (413, 320), O, 'shop'),
    ('A', 'beautiful', '뷰티풀', (443, 262), (200, 70, 140), 'shop'),
    ('A', 'hyodam', '올리브영', (545, 280), (120, 165, 30), 'shop'),     # 2026-11-12 사용자 요청: 효담요양원 → 올리브영 (자리 그대로)
    ('A', 'keunson', '큰손왕만두', (587, 212), (190, 50, 40), 'shop'),
    # 블록 S (명품대패·공영주차장·CU) 는 아래 BLOCK_S 로 옮김 (2026-11-11 117번길 연장 + 네이버 배치)
]
# 블록 S: 117번길·117 골목·V1/등뼈(int_spine_a)·117번길 연장(int_117)으로 닫힌 블록 (2026-11-11 네이버 캡처).
#   네이버 배치(서 → 동): 이름 없는 흰 건물들 | 드림오피스텔 | 동쪽 끝 = 북: 명품대패 / 남: 공영주차장(오른쪽 도로에 붙음).
#   CU 는 네이버에서 오른쪽 도로 건너편(동쪽)이라 등뼈 동쪽으로 옮긴다.
BLOCK_S_PROBE = (556.0, 282.0)          # 블록 안 점 (SUMO 좌표)
#   구역 = 블록(보도 뺀 땅) ∩ 아래 사각형 (SUMO 좌표 x0, x1, y0, y1). 네이버 상대 위치:
#   117번길 쪽 서쪽 열(이름 없는 흰 건물 둘, 북·남) | 가운데 드림오피스텔 | 오른쪽 도로 쪽 동쪽 열(북 명품대패, 남 공영주차장)
#   2026-11-12 사용자 요청: 명품대패·드림오피스텔 옆 이름 없는 건물 둘을 없애고, 드림오피스텔은 블록 남쪽 끝으로 내리고,
#   나머지는 큰 공영주차장 (env.roundabout_parking_s, 실제 입구·출구·주차 칸). 사각형은 블록 틀(block_s_geometry) 좌표
#   (s: 117 골목 방향, t: 117번길 방향 = 북서 모서리에서 잰 m) — 주차장(t ≲ 67) 남쪽 모서리.
BLOCK_S = [   # (키, 간판 목록, 간판색, 종류, 틀 사각형 s0, s1, t0, t1)
    ('dream', ['드림오피스텔'], (90, 80, 150), 'shop', (4.0, 52.0, 70.0, 104.0)),
]
S_REMOVED = ['dream_w1', 'dream_w2', 'myeongpum', 'parking_w']   # 예전 블록 S 그림 (다시 그릴 때 지운다)
CU_BOX = (604.0, 634.0, 262.0, 298.0)   # CU 자리: 등뼈 동쪽 (x0, x1, y0, y1), 도로·보도는 뺀다
# 완정로 남쪽 큰길 쪽 블록 (네이버 2026-11-11 캡처에서 잰 구역 길이 비율, C1→C3 순서)
# {블록: (블록 안 s 탐침, [(키, 간판 목록, 간판색, 길이 비율)])}
D1_PROBE_T = 11.0
FILL_MARGIN = 1.0                 # fill 블록: 구역 가장자리 ~ 건물 (m)
MAIN_BLOCKS = {
    'M1': (120.0, [
        ('epyeon', ['E편한이불'], [B], 41),
        ('lguplus', ['LG유플러스 대리점'], [(200, 0, 90)], 26.5),
        ('subway', ['써브웨이'], [(0, 130, 70)], 32.5),
    ]),
    'M2': (200.0, [
        ('twosome', ['투썸플레이스'], [(170, 30, 40)], 19),
        ('taebaek', ['태백산'], [O], 27),
        ('suhan', ['검단수한방병원'], [(200, 40, 50)], 22),          # 2026-11-11 사용자 요청으로 옆 칸(빈 건물 자리)으로 옮김
        ('emart24', ['이마트24'], [Y], 32),
    ]),
}
ZONE_FILL = [(214, 210, 202), (204, 198, 188), (196, 192, 186), (210, 206, 198), (200, 200, 196), (190, 196, 206)]


class LocalFrame:
    def __init__(self, o, u):
        L = math.hypot(*u)
        self.o, self.u = o, (u[0]/L, u[1]/L)
        self.v = (self.u[1], -self.u[0])            # u 오른쪽

    def P(self, s, t):
        return (self.o[0] + self.u[0]*s + self.v[0]*t, self.o[1] + self.u[1]*s + self.v[1]*t)

    def st(self, p):
        d = (p[0]-self.o[0], p[1]-self.o[1])
        return d[0]*self.u[0] + d[1]*self.u[1], d[0]*self.v[0] + d[1]*self.v[1]


def _rd(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def build_lines(net, img_dir):
    """roundabout.shops.add.xml 에 넣을 줄들과 보고."""
    from shapely.geometry import LineString, Polygon, Point, MultiPoint
    from shapely.ops import unary_union, voronoi_diagram, transform
    from env.roundabout_shops import Frame
    from env.roundabout_building_style import ROOF_PALETTE, PARAPET_INSET, polys as style_polys
    from env.roundabout_nw_buildings import _lot_buildings, _max_rect, sign_stack
    from env.roundabout_shop_buildings import draw_shop, PPM, PAVING
    F = Frame(net)
    es = {e.get('id'): e for e in net.findall('edge')}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    lane = lambda eid, k=0: _rd(next(l for l in es[eid].findall('lane') if l.get('index') == str(k)).get('shape'))
    # 도로 바닥 (차로 + 교차점), 보도 포함 버전
    paint, keep = [], []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = _rd(l.get('shape'))
            if len(pts) >= 2:
                g = LineString(pts)
                w = float(l.get('width', '3.2'))
                paint.append(g.buffer(w/2, cap_style=2))
                keep.append(g.buffer(w/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and len(_rd(s)) >= 3:
            p = Polygon(_rd(s)).buffer(0)
            paint.append(p)
            keep.append(p.buffer(SIDEWALK))
    paint, keep = unary_union(paint), unary_union(keep)

    # 네이버 픽셀 → SUMO (닮음 변환, 이미지 y 아래 방향)
    a = complex(*NAVER['a'])
    c = J['T_end_main_end']
    b = complex(*c) - a*complex(NAVER_CENTER_PX[0], -NAVER_CENTER_PX[1])
    naver = lambda px: (lambda w: (w.real, w.imag))(a*complex(px[0], -px[1]) + b)

    # ── 블록 ──
    # A: 검단로·완정로(동남쪽 큰길)·117번길·117 골목·C1 로 둘러싸인 ㄱ자 띠 (도로로 닫힌 면)
    from shapely.geometry import box as _box
    from env.roundabout_shops import _alley_117, SW_MAIN
    alley = _alley_117(net, F, J['shop_N1'])[1]['shop_117']
    corner = alley[len(alley)//2]                                  # 골목 모퉁이 곡선 가운데
    sw = lane(SW_MAIN, 0)
    fw = LocalFrame(sw[0], (sw[-1][0]-sw[0][0], sw[-1][1]-sw[0][1]))   # s: 117번길 → 검단사거리, t: 상가 쪽
    probe = fw.P(fw.st(J['shop_J0'])[0] + 15, fw.st(J['shop_J0'])[1] * 0.5 + 1.6)
    big = _box(*unary_union([Point(J['shop_J0']), Point(J['shop_N1']), Point(c)]).buffer(150).bounds)
    faces = list(getattr(big.difference(paint), 'geoms', [big.difference(paint)]))
    blockA = next(g for g in faces if g.contains(Point(probe)))
    # A 를 모퉁이에서 두 다리로: 검단로 쪽(117번길 ~ 모퉁이), 완정로 쪽(모퉁이 ~ C1)
    sq = fw.st(corner)[0]
    # 뷰티풀(검단로 다리 끝)과 큰손왕만두(완정로 다리 시작) 사이를 잔디 띠로 끊는다 (사용자 요청 2026-11-11)
    grass_cut = Polygon([fw.P(sq - CUT_W/2, -60), fw.P(sq + CUT_W/2, -60),
                         fw.P(sq + CUT_W/2, 80), fw.P(sq - CUT_W/2, 80)]).intersection(blockA)
    blockA = blockA.difference(grass_cut)
    legSW = blockA.intersection(Polygon([fw.P(-500, -500), fw.P(sq, -500), fw.P(sq, 500), fw.P(-500, 500)]))
    legSE = blockA.difference(legSW)
    # 블록 S 는 안쪽 도로(117번길 연장·등뼈)가 생긴 뒤 build_block_s 가 따로 그린다
    big_piece = lambda g: max(getattr(g, 'geoms', [g]), key=lambda x: x.area)
    blocks = {'A_SW': (big_piece(legSW), fw), 'A_SE': (big_piece(legSE), F)}
    # 가게를 다리에 나눈다: 네이버 배치에서 검단사거리 동쪽(완정로 쪽)에 있는 가게만 완정로 다리
    SE_LEG = {'hyodam', 'keunson'}
    lines, pois, report = [], [], {}
    for k, g in enumerate(getattr(blockA, 'geoms', [blockA])):
        lines.append(f'    <poly id="shopz_paving_A{"" if k == 0 else k}" type="sidewalk" color="{PAVING}" fill="1" '
                     f'layer="-2" shape="{" ".join(f"{x:.3f},{y:.3f}" for x, y in g.exterior.coords)}"/>')
    for k, g in enumerate(getattr(grass_cut, 'geoms', [grass_cut])):
        if g.area > 1:      # 보도(layer -2, 상가 골목 전체 보도 포함) 위, 구역 아래
            lines.append(f'    <poly id="shopz_grass_cut_{k}" type="grass" color="{GRASS}" fill="1" layer="-1.8" '
                         f'shape="{" ".join(f"{x:.3f},{y:.3f}" for x, y in g.exterior.coords)}"/>')
    for bk, (blk_world, fr) in blocks.items():
        to_st = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.st(p) for p in zip(x, y)]))), g)
        build = to_st(blk_world.difference(keep))
        build = max(getattr(build, 'geoms', [build]), key=lambda g: g.area)
        paint_st = to_st(paint)
        items = [s_ for s_ in SHOPS if (s_[0] == 'S' and bk == 'S') or
                 (s_[0] == 'A' and bk == ('A_SE' if s_[1] in SE_LEG else 'A_SW'))]
        if not items:
            continue
        real = [fr.st(naver(s[3])) for s in items]
        # 구역: 네이버 배치의 블록 길이 방향(s) 순서대로, 블록을 같은 면적의 띠로 나눈다 (구역 크기가 고르게)
        order = sorted(range(len(items)), key=lambda i: (real[i][0], real[i][1]))
        cells = [None]*len(items)
        s_lo, _, s_hi, _ = build.bounds
        total = build.area
        from shapely.geometry import box as _box
        t_lo, t_hi = build.bounds[1] - 1, build.bounds[3] + 1
        cut = lambda x: build.intersection(_box(s_lo - 1, t_lo, x, t_hi)).area
        bounds_s = [s_lo - 1]
        for k in range(1, len(items)):
            lo, hi = s_lo, s_hi
            for _ in range(40):
                mid = (lo + hi)/2
                lo, hi = (mid, hi) if cut(mid) < total*k/len(items) else (lo, mid)
            bounds_s.append((lo + hi)/2)
        bounds_s.append(s_hi + 1)
        for k, i in enumerate(order):
            cells[i] = build.intersection(_box(bounds_s[k], t_lo, bounds_s[k+1], t_hi))
        _emit(lines, report, [(it[1], [it[2]], it[4], it[5]) for it in items], cells, fr, paint_st, img_dir)

    # ── 완정로 남쪽 C1~C2, C2~C3 블록 (효담요양원 ~ 검단수한방병원): 네이버 배치 비율대로 구역 (2026-11-11) ──
    return _main_blocks(lines, report, F, paint, keep, img_dir)


def _lot_internal(eid):
    """블록 S 공영주차장 안쪽 통로·진출입로 (블록 면을 나누지 않게 도로 바닥에서 뺀다)."""
    return eid.startswith('pk_') or eid.startswith(':pk_n')     # 골목 위 진출입 교차점(pk_mid*)은 도로로 남긴다


def _roads(net):
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    paint, keep = [], []
    for e in net.findall('edge'):
        if _lot_internal(e.get('id', '')):
            continue
        for l in e.findall('lane'):
            pts = _rd(l.get('shape'))
            if len(pts) >= 2:
                g = LineString(pts)
                w = float(l.get('width', '3.2'))
                paint.append(g.buffer(w/2, cap_style=2))
                keep.append(g.buffer(w/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and len(_rd(s)) >= 3 and not _lot_internal(':' + j.get('id', '')):
            p = Polygon(_rd(s)).buffer(0)
            paint.append(p)
            keep.append(p.buffer(SIDEWALK))
    return unary_union(paint), unary_union(keep)


S_IDS = ('shopz_paving_S', 'shopz_zone_', 'shopz_house_', 'shopz_', 'shop_parking_w')
S_KEYS = [b[0] for b in BLOCK_S] + ['cu'] + S_REMOVED


def block_s_geometry(net):
    """블록 S (도로로 닫힌 면), 보도 뺀 땅, 블록 틀(LocalFrame).
    틀: 원점 = 보도 뺀 땅의 북서 모서리(117번길 × 117 골목), s = 117 골목 방향(북동), t = 117번길 방향(남동).
    두 길이 거의 직각(90.4°)이라 주차장·건물을 이 틀에 맞춰 놓는다."""
    from shapely.geometry import Point, box as _box
    paint, keep = _roads(net)
    big_piece = lambda g: max(getattr(g, 'geoms', [g]), key=lambda x: x.area)
    bigS = _box(*Point(BLOCK_S_PROBE).buffer(200).bounds)
    closed = paint.buffer(0.3)          # 차로·교차점 이음새의 아주 작은 틈으로 면이 새지 않게
    facesS = list(getattr(bigS.difference(closed), 'geoms', [bigS.difference(closed)]))
    blockS = next(g for g in facesS if g.contains(Point(BLOCK_S_PROBE))).buffer(0.3).difference(paint)
    blockS = big_piece(blockS)
    if blockS.area > 20000:
        raise RuntimeError(f'블록 S 가 도로로 닫히지 않음 (면적 {blockS.area:.0f} m²)')
    build = big_piece(blockS.difference(keep))
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    lane = {l.get('id'): _rd(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    a117 = lane['shop_117_f_0']
    u = (a117[1][0] - a117[0][0], a117[1][1] - a117[0][1])         # 117 골목 곧은 구간 방향
    o = min(build.exterior.coords, key=lambda p: math.dist(p, J['shop_J0']))
    return blockS, build, LocalFrame(o, u), paint, keep


def build_block_s(net, img_dir):
    """블록 S + CU 줄들. 안쪽 도로(roundabout_interior)가 적용된 망이어야 블록이 닫힌다."""
    from shapely.geometry import box as _box
    from shapely.ops import transform
    from env.roundabout_shop_buildings import PAVING
    blockS, build_w, fs, paint, keep = block_s_geometry(net)
    big_piece = lambda g: max(getattr(g, 'geoms', [g]), key=lambda x: x.area)
    lines, report = [], {}
    lines.append(f'    <poly id="shopz_paving_S" type="sidewalk" color="{PAVING}" fill="1" layer="-2" '
                 f'shape="{" ".join(f"{x:.3f},{y:.3f}" for x, y in blockS.exterior.coords)}"/>')
    to_s = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fs.st(p) for p in zip(x, y)]))), g)
    build = to_s(build_w)
    paint_s = to_s(paint)
    cellsS = [big_piece(build.intersection(_box(s0, t0, s1, t1))) for *_, (s0, s1, t0, t1) in BLOCK_S]
    _emit(lines, report, [(k_, nm, c_, kd) for k_, nm, c_, kd, _ in BLOCK_S], cellsS, fs, paint_s, img_dir)
    # CU: 등뼈 동쪽 (네이버에서 오른쪽 도로 건너편)
    cu_world = _box(CU_BOX[0], CU_BOX[2], CU_BOX[1], CU_BOX[3]).difference(keep)
    if not cu_world.is_empty:
        fc = LocalFrame((CU_BOX[0], CU_BOX[3]), (1.0, 0.0))
        to_c = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fc.st(p) for p in zip(x, y)]))), g)
        _emit(lines, report, [('cu', ['CU'], (110, 50, 150), 'shop')], [big_piece(to_c(cu_world))], fc,
              to_c(paint), img_dir)
    return lines, report


def append_block_s(net_path, add_path=None, img_dir=None):
    """roundabout.shops.add.xml 에서 예전 블록 S·CU 줄을 지우고 새로 그려 덧붙인다 (파이프라인 9-6 뒤)."""
    import re
    import xml.etree.ElementTree as ET
    from env.roundabout_shop_buildings import OUT, IMG_DIR
    add_path = add_path or OUT
    img_dir = img_dir or IMG_DIR
    net = ET.parse(net_path).getroot()
    new, report = build_block_s(net, img_dir)
    ids = {'shopz_paving_S', 'shop_parking_w'} | {f'{p}{k}' for k in S_KEYS
                                                   for p in ('shopz_zone_', 'shopz_house_', 'shopz_')}
    with open(add_path, encoding='utf-8') as fh:
        old = fh.read().split('\n')
    keep_lines = []
    for ln in old:
        m = re.search(r' id="([^"]+)"', ln)
        pid = m.group(1) if m else None
        if pid and (pid in ids or any(pid.startswith(f'shopz_house_{k}_') for k in S_KEYS)):
            continue
        keep_lines.append(ln)
    end = max(i for i, ln in enumerate(keep_lines) if ln.strip() == '</additional>')
    keep_lines[end:end] = new
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(keep_lines))
    return report


def _main_blocks(lines, report, F, paint, keep, img_dir):
    from shapely.geometry import Point, Polygon, box as _box
    from shapely.ops import transform
    big2 = Polygon([F.P(60, -6), F.P(260, -6), F.P(260, D1_PROBE_T*2 + 6), F.P(60, D1_PROBE_T*2 + 6)])
    faces2 = list(getattr(big2.difference(paint), 'geoms', [big2.difference(paint)]))
    paint_st = transform(lambda x, y, z=None: tuple(map(list, zip(*[F.st(p) for p in zip(x, y)]))), paint)
    for bk, (probe_s, items) in MAIN_BLOCKS.items():
        face = next((g for g in faces2 if g.contains(Point(F.P(probe_s, D1_PROBE_T)))), None)
        if face is None:
            report[bk] = 'no block'
            continue
        build = transform(lambda x, y, z=None: tuple(map(list, zip(*[F.st(p) for p in zip(x, y)]))),
                          face.difference(keep))
        build = max(getattr(build, 'geoms', [build]), key=lambda g: g.area)
        s_lo, t_lo, s_hi, t_hi = build.bounds
        tot = sum(it[3] for it in items)
        cuts, acc = [s_lo - 1], 0.0
        for it in items[:-1]:
            acc += it[3]
            cuts.append(s_lo + (s_hi - s_lo)*acc/tot)
        cuts.append(s_hi + 1)
        cells = [build.intersection(_box(cuts[k], t_lo - 1, cuts[k+1], t_hi + 1)) for k in range(len(items))]
        _emit(lines, report, [(it[0], it[1], it[2], 'shop') for it in items], cells, F, paint_st, img_dir, fill=True)
    return lines, report


def _emit(lines, report, items, cells, fr, paint_st, img_dir, fill=False, aspect=None, bld_max=None):
    """구역마다 바닥 + 네모 건물 하나 + 간판. items: (키, 간판 목록, 간판색(하나 또는 목록), 종류).
    fill: 건물이 구역을 거의 채운다 (네이버 지도에서 건물이 붙어 늘어선 블록).
    bld_max: {키: (s, t)} — 그 구역만 건물 최대 크기(env.roundabout_nw_buildings.BLD_MAX)를 바꾼다."""
    from env.roundabout_building_style import ROOF_PALETTE, PARAPET_INSET, polys as style_polys
    from env.roundabout_nw_buildings import _lot_buildings, _max_rect, sign_stack
    from env.roundabout_shop_buildings import draw_shop, PPM
    for n, ((key, names, color, kind), cell) in enumerate(zip(items, cells)):
        cell = cell.buffer(-0.75)
        if cell.is_empty:
            report[key] = 'dropped'
            continue
        if cell.geom_type != 'Polygon':
            cell = max(cell.geoms, key=lambda g: g.area)
        cell = cell.simplify(0.4)
        # 구역 (북서쪽 상가 구역과 같은 모양)
        zf = ZONE_FILL[n % len(ZONE_FILL)]
        shp = ' '.join(f'{x:.3f},{y:.3f}' for x, y in (fr.P(*q) for q in cell.exterior.coords))
        lines.append(f'    <poly id="shopz_zone_{key}" type="building" color="{",".join(map(str, zf))}" fill="1" '
                     f'layer="-1.5" shape="{shp}"/>')
        # 구역 테두리선은 그리지 않는다 (사용자 요청 2026-11-11)
        import env.roundabout_nw_buildings as _nwb
        _old = _nwb.BLD_MAX
        _nwb.BLD_MAX = (bld_max or {}).get(key, _old)
        try:
            rects = [_max_rect(cell.buffer(-FILL_MARGIN))] if fill else _lot_buildings(cell, paint_st, 1)
        finally:
            _nwb.BLD_MAX = _old
        rects = [r for r in rects if r]
        if not rects:
            report[key] = 'no room'
            continue
        s0, s1, t0, t1 = rects[0]
        # 가로세로 비율 최대 MAX_ASPECT (막대처럼 얇은 건물 방지)
        w_, h_ = s1 - s0, t1 - t0
        if fill and aspect:                # 구역을 채우되 너무 길쭉하면 가운데를 남겨 줄인다
            if w_ > aspect*h_:
                cs_ = (s0 + s1)/2
                s0, s1 = cs_ - aspect*h_/2, cs_ + aspect*h_/2
            elif h_ > aspect*w_:
                ct_ = (t0 + t1)/2
                t0, t1 = ct_ - aspect*w_/2, ct_ + aspect*w_/2
        elif fill:
            pass
        elif w_ > MAX_ASPECT*h_:
            cs_ = (s0 + s1)/2
            s0, s1 = cs_ - MAX_ASPECT*h_/2, cs_ + MAX_ASPECT*h_/2
        elif h_ > MAX_ASPECT*w_:
            ct_ = (t0 + t1)/2
            t0, t1 = ct_ - MAX_ASPECT*w_/2, ct_ + MAX_ASPECT*w_/2
        cx, cy = fr.P((s0 + s1)/2, (t0 + t1)/2)
        if kind == 'parking':
            # 주차장: 구역 안 가장 큰 사각형 전체를 주차장 바닥으로
            R_ = _max_rect(cell.buffer(-1.0)) or rects[0]
            s0, s1, t0, t1 = R_
            w, h = s1 - s0, t1 - t0
            cx, cy = fr.P((s0 + s1)/2, (t0 + t1)/2)
            ang = math.degrees(math.atan2(fr.v[0], fr.v[1]))
            fn = f'shop_{key}.png'
            draw_shop(key, names[0], 'parking', w, h, None, color, ang).save(os.path.join(img_dir, fn))
            lines.append(f'    <poi id="shop_{key}" type="parking" color="255,255,255" layer="-1" x="{cx:.3f}" '
                         f'y="{cy:.3f}" angle="{ang:.2f}" width="{w:.2f}" height="{h:.2f}" imgFile="shop_img/{fn}"/>')
            report[key] = (round(w), round(h))
            continue
        roof = ROOF_PALETTE[(len(key) + n*3) % len(ROOF_PALETTE)]
        k_ = PARAPET_INSET
        corners = [(s0, t0), (s1, t0), (s1, t1), (s0, t1)]
        inner = [(s0 + k_, t0 + k_), (s1 - k_, t0 + k_), (s1 - k_, t1 - k_), (s0 + k_, t1 - k_)]
        lines.extend(style_polys(f'shopz_house_{key}', [fr.P(*q) for q in corners], [fr.P(*q) for q in inner], roof))
        if not names:                  # 간판 없는 건물 (네이버 지도에서 이름 없는 건물)
            report[key] = (round(s1 - s0), round(t1 - t0))
            continue
        img = sign_stack(names, color)
        fit = max(s1 - s0, t1 - t0) * 1.05
        if img.width / PPM > fit:
            img = sign_stack(names, color, font_m=max(1.0, 2.0 * fit / (img.width / PPM)))
        fn = f'shopz_{key}.png'
        img.save(os.path.join(img_dir, fn))
        lines.append(f'    <poi id="shopz_{key}" type="shop" color="255,255,255" layer="-1" x="{cx:.3f}" y="{cy:.3f}" '
                     f'angle="0.00" width="{img.width / PPM:.2f}" height="{img.height / PPM:.2f}" '
                     f'imgFile="shop_img/{fn}"/>')
        report[key] = (round(s1 - s0), round(t1 - t0))
