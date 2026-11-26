"""마전동 블록(env.roundabout_majeon 골목으로 나뉜 면)을 간단한 건물 + 주요 이름으로 채운다. (화면 표시 전용, 2026-11-12)

    python -c "from env import roundabout_majeon_buildings as m; print(m.build())"

  - 블록 = 골목·큰길로 닫힌 면 (BLOCKS 의 probe 점이 든 면). 바닥은 보도색으로 칠한다.
  - 블록 안(보도 SIDEWALK m 뺀 땅)을 블록 길이 방향(axis)으로 같은 면적의 띠로 나눠 이름마다 구역 하나,
    구역 안에 네모 건물 하나 + 간판 하나 (BUILDING_FILL_RULES_PROMPT.txt: 한 건물 한 이름, 최대 24 x 18m,
    env.roundabout_shop_zones._emit 그대로). 공영주차장은 주차장 바닥.
  - 이름과 순서: 네이버 지도(2026-11-11 캡처, image 7~10)의 블록 안 상대 위치 (axis 방향 순서). 주요 이름만.
  - 우버니공원: ⑧ 블록 3번길 쪽 끝을 공원(잔디·나무·산책로, env.roundabout_nw_buildings.draw_park)으로.
출력: env/sumo_roundabout/roundabout.majeon.add.xml, env/sumo_roundabout/shop_img/shopz_mj_*.png
"""
import math
import os
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
OUT = os.path.join(NET_DIR, 'roundabout.majeon.add.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
PAVING = '204,200,192'
SIDEWALK = 2.5
WJ = (0.857, -0.515)                 # 완정로·상가 뒤 골목 방향 (북서 → 남동)
EAST = (0.985, -0.17)                # 45번길·7번길 방향 (서 → 동)
SOUTH = (-0.22, -0.975)              # 65번길 방향 (북 → 남)
R, O, G, B, P_, Y, BR, GR = ((200, 40, 60), (210, 100, 30), (40, 140, 90), (30, 80, 160), (120, 70, 160),
                             (220, 170, 20), (150, 90, 40), (90, 90, 120))
VILLA = (110, 90, 70)
# (블록 번호, 블록 안 점, 길이 방향, [(키, 이름, 간판색, 종류)] — 길이 방향 순서)
BLOCKS = [
    (1, (1465, -226), WJ, [('mj_yunseong', '윤성프라자', Y, 'shop'), ('mj_smile', '검단스마일마을한방병원', R, 'shop'),
                           ('mj_choegang', '최강마트', Y, 'shop'), ('mj_sulbing', '설빙', O, 'shop')]),
    (2, (1647, -317), WJ, [('mj_twosome', '투썸플레이스', (170, 30, 40), 'shop'), ('mj_cu', 'CU', P_, 'shop'),
                           # 2026-11-12 사용자 요청: 더벤티 → 탑유도관, 맘스터치·메가MGC커피 자리(오른쪽 두 띠)는 공영주차장
                           #   (env.roundabout_majeon_parking2 가 그린다 — 여기서는 비워 둔다)
                           ('mj_venti', '탑유도관', (40, 90, 160), 'shop'), ('mj_lot_a', '', None, 'reserved'),
                           ('mj_lot_b', '', None, 'reserved')]),
    (3, (1810, -386), WJ, [('mj_pizza', '피자스쿨', O, 'shop'), ('mj_italy', '이테리마담', O, 'shop'),
                           ('mj_shinhan', '신한은행', B, 'shop'), ('mj_olive', '올리브영', G, 'shop')]),
    (4, (1801, -499), EAST, [('mj_wash', '그린손세차장', B, 'shop'), ('mj_changwoo4', '창우아이빌', VILLA, 'shop'),
                             # 2026-11-14 사용자 요청: 공영주차장 → 동대문엽기떡볶이
                             ('mj_yupdduk', '동대문엽기떡볶이', R, 'shop'), ('mj_gogi', '고기싸롱', O, 'shop')]),
    # 2026-11-14 사용자 요청: 마전동행정복지센터 삭제 → 그 띠는 새마을금고 주차장 (env.roundabout_majeon_parking5)
    (5, (1770, -582), EAST, [('mj_sundae', '명천찹쌀진순대', O, 'shop'), ('mj_lot5', '', None, 'reserved'),
                             ('mj_saemaeul', '새마을금고', B, 'shop')]),
    (6, (1385, -328), WJ, [('mj_church', '예수사랑장로교회', GR, 'shop'), ('mj_gs25', 'GS25', B, 'shop'),
                           ('mj_seonkyung', '선경프라임빌', VILLA, 'shop')]),
    (7, (1311, -440), SOUTH, [('mj_samsung', '삼성쉐르빌', VILLA, 'shop'), ('mj_dongbo', '동보아트빌', VILLA, 'shop')]),
    (8, (1504, -470), EAST, [('mj_hyundai', '현대카운티', VILLA, 'shop'), ('mj_royal', '로얄클래스', VILLA, 'shop'),
                             ('mj_noble', '노블레스밸리', VILLA, 'shop'), ('mj_espo', '에스뽀아빌', VILLA, 'shop')]),
    (9, (1595, -515), EAST, [('mj_golden', '골든파크빌', VILLA, 'shop')]),
    (10, (1445, -553), EAST, [('mj_dain', '다인힐', VILLA, 'shop'), ('mj_geonyoung', '건영캐슬빌', VILLA, 'shop'),
                              ('mj_highvill', '하이빌리지', VILLA, 'shop'), ('mj_buwon', '부원파크빌', VILLA, 'shop')]),
]
# 2026-11-14 사용자 요청: 새마을금고·명천찹쌀진순대 건물을 크게 (기본 최대 24 x 18m 대신)
BIG = {'mj_saemaeul': (40.0, 30.0), 'mj_sundae': (40.0, 30.0)}
PARK_BLOCK, PARK_FRAC = 8, 0.30      # 우버니공원: ⑧ 블록 동쪽(3번길 쪽) 끝 30%


def _rd(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _fmt(pts):
    return ' '.join(f'{x:.3f},{y:.3f}' for x, y in pts)


def reserved_region(net, bid=2, skip_prefix='pm_'):
    """블록 bid 의 'reserved' 띠들을 합친 땅 (세계 좌표 다각형)과 블록 틀. build 와 같은 방식으로 나눈다.
    skip_prefix 로 시작하는 길(나중에 그 땅에 깐 주차장 통로)은 블록 면 계산에서 뺀다."""
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import unary_union, transform
    from env.roundabout_shop_zones import LocalFrame
    paint, keep = [], []
    for e in net.findall('edge'):
        if e.get('id', '').startswith((skip_prefix, f':{skip_prefix}n')):     # 주차장 통로·진출입로·안 교차점 경로
            continue
        for l in e.findall('lane'):
            p = _rd(l.get('shape'))
            if len(p) >= 2:
                w = float(l.get('width', '3.2'))
                paint.append(LineString(p).buffer(w/2, cap_style=2))
                keep.append(LineString(p).buffer(w/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        # 주차장 안 교차점(접두사 + n)만 뺀다 — 큰길 위 진출입 T자(접두사 + T)는 도로로 남긴다
        if s and j.get('type') != 'internal' and not j.get('id', '').startswith(skip_prefix + 'n') and len(_rd(s)) >= 3:
            g = Polygon(_rd(s)).buffer(0)
            paint.append(g)
            keep.append(g.buffer(SIDEWALK))
    paint, keep = unary_union(paint), unary_union(keep)
    area = box(1200, -720, 2050, -60)
    faces = list(getattr(area.difference(paint.buffer(0.3)), 'geoms', [area.difference(paint.buffer(0.3))]))
    _, probe, axis, items = next(b for b in BLOCKS if b[0] == bid)
    face = next(g for g in faces if g.contains(Point(probe)))
    face = face.buffer(0.3).difference(paint)
    face = max(getattr(face, 'geoms', [face]), key=lambda g: g.area)
    fr = LocalFrame(face.centroid.coords[0], axis)
    to_st = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.st(p) for p in zip(x, y)]))), g)
    land = face.difference(keep)
    land = max(getattr(land, 'geoms', [land]), key=lambda g: g.area)
    build = to_st(land)
    s_lo, t_lo, s_hi, t_hi = build.bounds

    def cut_s(frac):
        lo, hi = s_lo, s_hi
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if build.intersection(box(s_lo - 1, t_lo - 1, mid, t_hi + 1)).area <                 build.area * frac else (lo, mid)
        return (lo + hi) / 2
    n = len(items)
    bounds = [s_lo - 1] + [cut_s(k / n) for k in range(1, n)] + [s_hi + 1]
    res = [k for k, it in enumerate(items) if it[3] == 'reserved']
    g = build.intersection(box(bounds[res[0]], t_lo - 1, bounds[res[-1] + 1], t_hi + 1))
    g = max(getattr(g, 'geoms', [g]), key=lambda q: q.area)
    world = transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.P(*p) for p in zip(x, y)]))), g)
    return world, fr


def build(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), out=OUT):
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import unary_union, transform
    from env.roundabout_shop_zones import LocalFrame, _emit
    from env.roundabout_nw_buildings import draw_park, _max_rect
    net = ET.parse(net_path).getroot()
    os.makedirs(IMG_DIR, exist_ok=True)
    paint, keep = [], []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            p = _rd(l.get('shape'))
            if len(p) >= 2:
                w = float(l.get('width', '3.2'))
                paint.append(LineString(p).buffer(w/2, cap_style=2))
                keep.append(LineString(p).buffer(w/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and len(_rd(s)) >= 3:
            g = Polygon(_rd(s)).buffer(0)
            paint.append(g)
            keep.append(g.buffer(SIDEWALK))
    paint, keep = unary_union(paint), unary_union(keep)
    closed = paint.buffer(0.3)
    area = box(1200, -720, 2050, -60)
    faces = list(getattr(area.difference(closed), 'geoms', [area.difference(closed)]))
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_majeon_buildings.py 가 생성. 마전동 블록 보도·건물·간판·공원 (주행에는 영향 없음) -->',
             '<additional>']
    report = {}
    for bid, probe, axis, items in BLOCKS:
        face = next((g for g in faces if g.contains(Point(probe))), None)
        if face is None or face.area > 60000:
            report[f'block{bid}'] = 'no block'
            continue
        face = face.buffer(0.3).difference(paint)
        face = max(getattr(face, 'geoms', [face]), key=lambda g: g.area)
        lines.append(f'    <poly id="mjb_paving_{bid}" type="sidewalk" color="{PAVING}" fill="1" layer="-2" '
                     f'shape="{_fmt(face.exterior.coords)}"/>')
        fr = LocalFrame(face.centroid.coords[0], axis)
        to_st = lambda g: transform(lambda x, y, z=None: tuple(map(list, zip(*[fr.st(p) for p in zip(x, y)]))), g)
        land = face.difference(keep)
        land = max(getattr(land, 'geoms', [land]), key=lambda g: g.area)
        build = to_st(land)
        s_lo, t_lo, s_hi, t_hi = build.bounds

        def cut_s(frac):                 # 앞쪽 면적이 frac 이 되는 s
            lo, hi = s_lo, s_hi
            for _ in range(40):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if build.intersection(box(s_lo - 1, t_lo - 1, mid, t_hi + 1)).area < \
                    build.area * frac else (lo, mid)
            return (lo + hi) / 2
        if bid == PARK_BLOCK:            # 우버니공원: 동쪽 끝 PARK_FRAC 만큼
            sp = cut_s(1 - PARK_FRAC)
            park = build.intersection(box(sp, t_lo - 1, s_hi + 1, t_hi + 1)).buffer(-1.0)
            park = max(getattr(park, 'geoms', [park]), key=lambda g: g.area)
            rect = _max_rect(park)
            if rect:
                ps0, ps1, pt0, pt1 = rect
                pw, ph = ps1 - ps0, pt1 - pt0
                ang = math.degrees(math.atan2(fr.v[0], fr.v[1]))
                fn = 'mjb_park_ubeoni.png'
                draw_park('우버니공원', pw, ph, ang).save(os.path.join(IMG_DIR, fn))
                cx, cy = fr.P((ps0 + ps1) / 2, (pt0 + pt1) / 2)
                lines.append(f'    <poi id="mjb_park_ubeoni" type="park" color="255,255,255" layer="-1" '
                             f'x="{cx:.3f}" y="{cy:.3f}" angle="{ang:.2f}" width="{pw:.2f}" height="{ph:.2f}" '
                             f'imgFile="shop_img/{fn}"/>')
                report['mj_park'] = (round(pw), round(ph))
            build = build.intersection(box(s_lo - 1, t_lo - 1, sp - 2.0, t_hi + 1))
            build = max(getattr(build, 'geoms', [build]), key=lambda g: g.area)
            s_lo, t_lo, s_hi, t_hi = build.bounds
        n = len(items)
        bounds = [s_lo - 1] + [cut_s(k / n) for k in range(1, n)] + [s_hi + 1]
        cells = [build.intersection(box(bounds[k], t_lo - 1, bounds[k + 1], t_hi + 1)) for k in range(n)]
        cells = [max(getattr(c, 'geoms', [c]), key=lambda g: g.area) if not c.is_empty else c for c in cells]
        keep_i = [i for i, it in enumerate(items) if it[3] != 'reserved']
        _emit(lines, report, [(items[i][0], [items[i][1]], items[i][2], items[i][3]) for i in keep_i],
              [cells[i] for i in keep_i], fr, to_st(paint), IMG_DIR, bld_max=BIG)
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    report['out'] = out
    return report


if __name__ == '__main__':
    import sys
    sys.path.insert(0, BASE)
    print(build())
