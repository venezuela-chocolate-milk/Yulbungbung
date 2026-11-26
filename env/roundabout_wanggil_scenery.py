"""왕길동 (회전교차로 북서쪽 도로 · 검단로 대각선 · 445번길 · 왕길동 큰길로 둘러싸인 땅)을 뭉텅이로 채운다. (화면 표시 전용)

    python -m env.roundabout_wanggil_scenery

  - 2026-11-12 네이버 위성 지도(검단회전교차로 ~ 온누리병원) 기준. 길 순서·상대 배치만 살려 SUMO 모양에 맞게 놓았다.
  - 위 블록 (왕길초 앞길 nw_wg 북쪽): 서→동 마전주공아파트 · 인천검단동림디아망아파트 · 신명스카이뷰아파트,
    445번길 네거리 모퉁이에 동협 · 파리바게뜨.
  - 아래 블록 (왕길초 앞길 남쪽): 서쪽 검단1차풍림아이원아파트, 동쪽 왕길근린공원, 왕길초등학교 자리와 검단로 쪽은
    검단원흥아파트 (2026-11-14 사용자 요청: 왕길초 삭제). 두 단지 주차장은 env.roundabout_wanggil_south_parking.
  - 남는 땅은 숲(나무 점). 건물·간판 스타일은 env.roundabout_interior_scenery 와 같다.
출력: env/sumo_roundabout/roundabout.wanggil.add.xml (간판 그림은 interior_img/int_wg_*.png)
"""
import os
import random
import xml.etree.ElementTree as ET

from env import roundabout_interior_scenery as isc

NET_DIR = isc.NET_DIR
OUT = os.path.join(NET_DIR, 'roundabout.wanggil.add.xml')
SIDEWALK = 3.0
# 채울 땅을 고르는 대략 범위 (도로 안쪽 면 중 대표점이 이 안에 있는 면)
HINT = [(-140, 270), (-150, 400), (-105, 480), (0, 508), (215, 492), (250, 320), (283, 206), (0, 0),
        (-30, 60), (-65, 165)]
# (키, 모양(SUMO 좌표), 종류, 간판, 선택값) — 앞에 적은 구역이 땅을 먼저 차지한다
ZONES = [
    # ── 위 블록 ──
    ('wg_dh', [(165, 310), (232, 318), (226, 372), (165, 366)], 'shops', ['동협', '파리바게뜨'],
     dict(angle=6, bld=(18, 14))),
    # 2026-11-12 사용자 요청: 아래쪽은 아파트 주차장(env.roundabout_wanggil_parking, 직사각형) — 구역은 원래 크기,
    #   주차장 그림 자리만 빠지고(taken_files) 남는 땅은 마전주공 동으로 채운다
    ('wg_majeon', [(-240, 280), (-62, 288), (-62, 560), (-240, 560)], 'apt', ['마전주공아파트'],
     dict(angle=4, slab=(28, 11), gap=(9, 14))),
    ('wg_dimang', [(-56, 288), (88, 298), (88, 520), (-56, 520)], 'apt', ['인천검단동림디아망아파트'],
     dict(angle=4, slab=(44, 13), gap=(12, 22))),
    ('wg_skyview', [(94, 300), (240, 312), (232, 520), (94, 520)], 'apt', ['신명스카이뷰아파트'],
     dict(angle=4, slab=(40, 13), gap=(12, 22))),
    # ── 아래 블록 ──
    ('wg_park', [(148, 304), (250, 316), (262, 230), (175, 200), (148, 205)], 'park', ['왕길근린공원'], {}),
    ('wg_pungrim', [(-110, 270), (12, 290), (12, 110), (-25, 55), (-70, 165)], 'apt', ['검단1차풍림아이원아파트'],
     dict(angle=-20, slab=(30, 11), gap=(8, 14))),
    ('wg_wonheung', [(14, 182), (175, 196), (200, 140), (40, 20), (14, 30)], 'apt', ['검단원흥아파트'],
     dict(angle=36, slab=(40, 12), gap=(10, 18))),
    # 옛 왕길초 자리: 앞길 쪽은 원흥 주차장, 남는 땅은 원흥 동 (앞길과 나란히)
    ('wg_wonheung_n', [(18, 292), (142, 302), (142, 196), (18, 186)], 'apt', [],
     dict(angle=4, slab=(36, 12), gap=(10, 14))),
]
SCHOOL_GROUND = (210, 204, 190)
FIELD = (198, 172, 132)
PAVING = (204, 200, 192)


def build(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), out=OUT, zones=None, hint_pts=None, tag='wg',
          island=True, title='왕길동 아파트·학교·공원·숲', clip=False,
          taken_files=('nw', 'wanggil_parking', 'wanggil_south_parking'), ignore_prefix=()):
    """zones/hint_pts/tag 를 바꿔 다른 동네(예: env.roundabout_majeon_east_scenery)도 같은 방식으로 채운다.
    ignore_prefix: 도로로 치지 않을 길·교차점 접두사 (예: 나중에 깐 주차장 통로 'pd_' — 단지 동 배치가 밀리지 않게)"""
    zones = ZONES if zones is None else zones
    from shapely.geometry import LineString, Polygon, box
    from shapely.ops import unary_union
    from shapely import affinity
    net = ET.parse(net_path).getroot()
    os.makedirs(isc.IMG_DIR, exist_ok=True)
    rng = random.Random(23)
    roads, paint = [], []
    ign = tuple(ignore_prefix) + tuple(f':{q}' for q in ignore_prefix)
    for e in net.findall('edge'):
        if ign and e.get('id', '').startswith(ign):
            continue
        for l in e.findall('lane'):
            pts = isc._read(l.get('shape'))
            if len(pts) >= 2:
                w = float(l.get('width', '3.2'))
                roads.append(LineString(pts).buffer(w/2 + SIDEWALK, cap_style=2))
                paint.append(LineString(pts).buffer(w/2, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if ign and j.get('id', '').startswith(ign) and '_mid_' not in j.get('id', ''):
            continue                      # 큰길 위에 생긴 진출입 교차점(…_mid_…)은 도로로 친다
        if s and j.get('type') != 'internal' and len(isc._read(s)) >= 3:
            roads.append(Polygon(isc._read(s)).buffer(SIDEWALK))
            paint.append(Polygon(isc._read(s)).buffer(0))
    road, paint_u = unary_union(roads), unary_union(paint)
    # 이미 그림이 있는 자리 (북서쪽 상가 구역)
    taken = []
    for f in taken_files:
        fp = os.path.join(NET_DIR, f'roundabout.{f}.add.xml')
        if not os.path.exists(fp):
            continue
        for p in ET.parse(fp).getroot().findall('poly'):
            if p.get('fill') == '1' and p.get('shape') and len(isc._read(p.get('shape'))) >= 3:
                taken.append(Polygon(isc._read(p.get('shape'))).buffer(0))
    taken = unary_union(taken).buffer(2.0) if taken else Polygon()
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    tri = None
    if hint_pts is not None:
        hint = Polygon(hint_pts)
    elif island and 'nw_L' in J:
        # 원정삼거리 삼각형: 위 블록은 황수대로·우회전 길·완정로까지, 삼각형 안쪽은 따로 녹지 섬
        L_, Y_, K_ = J['nw_L'], J['nw_Y'], J['nw_K']
        hint = Polygon([(-125, 262), L_, K_] + HINT[4:])
        tri = Polygon([L_, Y_, K_])
    else:
        hint = Polygon(HINT)
    faces = hint.buffer(120).envelope.difference(paint_u)
    # 큰길 잔디 분리대처럼 가는 면은 빼고, 대표점이 범위 안에 있는 넓은 면만
    if clip:
        # 한쪽이 열린 동네(위쪽에 길이 없음): 범위 다각형 안에서 도로만 뺀 땅
        land = hint.difference(paint_u)
    else:
        land = unary_union([g for g in getattr(faces, 'geoms', [faces])
                            if g.area > 400 and not g.buffer(-5.0).is_empty and hint.contains(g.representative_point())])
    base = land.difference(paint_u.buffer(0.1)).difference(taken)   # 보도색 바닥: 차로 가장자리까지
    island = None
    if tri is not None:
        cand = [g for g in getattr(faces, 'geoms', [faces]) if g.area > 200 and tri.contains(g.representative_point())]
        island = max(cand, key=lambda g: g.area) if cand else None
    land = land.difference(road).difference(taken)

    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             f'<!-- env/roundabout_wanggil_scenery.py 가 생성. {title} (주행에는 영향 없음) -->',
             '<additional>']
    # 블록 바닥(보도색)을 먼저 깔아 구역 사이가 초록 틈으로 보이지 않게 한다
    lines += isc._ground(f'{tag}_block', base, PAVING, '-1.9')
    used, report = [], {}
    for key, shape, kind, names, opt in zones:
        angle = opt.get('angle', 0)
        zone = Polygon(shape).intersection(land)
        for u in used:
            zone = zone.difference(u)
        if zone.is_empty:
            report[key] = 'empty'
            continue
        if zone.geom_type != 'Polygon':
            zone = max(zone.geoms, key=lambda g: g.area)
        used.append(zone.buffer(1.0))
        ground = SCHOOL_GROUND if kind == 'school' else isc.GROUND[kind]
        lines += isc._ground(key, zone, ground, '-1.8')
        inner = zone.buffer(-3.0)
        if inner.is_empty:
            report[key] = 'thin'
            continue
        inner = max(getattr(inner, 'geoms', [inner]), key=lambda g: g.area)
        blds = []
        if kind == 'apt':
            blds = [(b, isc.APT_ROOFS[k % len(isc.APT_ROOFS)])
                    for k, b in enumerate(isc._rect_fill(inner, angle, opt['slab'], opt['gap']))]
        elif kind == 'shops':
            blds = [(b, isc.SHOP_ROOFS[k % len(isc.SHOP_ROOFS)])
                    for k, b in enumerate(isc._rect_fill(inner, angle, opt.get('bld', (20, 15)), (4, 5), 0.2, rng))]
        elif kind == 'villa':                       # 2026-11-12 검단 구시가지: 작은 빌라·주택
            blds = [(b, isc.VILLA_ROOFS[k % len(isc.VILLA_ROOFS)])
                    for k, b in enumerate(isc._rect_fill(inner, angle, opt.get('bld', (15, 13)), (6, 6), 0.25, rng))]
        elif kind == 'school':
            # 서쪽에 ㄱ자 교사, 동쪽에 운동장 (흙색 + 흰 트랙 선)
            c = inner.centroid
            rp = affinity.rotate(inner, -angle, origin=c)
            x0, y0, x1, y1 = rp.bounds
            w = x1 - x0
            wing_a = box(x0 + 4, y1 - 18, x0 + w*0.45, y1 - 4)
            wing_b = box(x0 + 4, y0 + 8, x0 + 18, y1 - 18)
            field = box(x0 + w*0.5, y0 + 8, x1 - 6, y1 - 8)
            for k, g in enumerate((wing_a, wing_b)):
                g = affinity.rotate(g, angle, origin=c).intersection(inner)
                if not g.is_empty and g.geom_type == 'Polygon':
                    blds.append((g, (222, 214, 196) if k == 0 else (214, 206, 190)))
            fg = affinity.rotate(field, angle, origin=c).intersection(inner)
            if not fg.is_empty and fg.geom_type == 'Polygon':
                lines += isc._ground(f'{key}_field', fg, FIELD, '-1.7')
                track = fg.buffer(-3.0)
                if not track.is_empty and track.geom_type == 'Polygon':
                    lines.append(f'    <poly id="{key}_track" type="marking" color="245,245,240" fill="0" '
                                 f'lineWidth="0.4" layer="-1.65" shape="{isc._fmt(track.exterior.coords)}"/>')
        elif kind == 'park':
            from shapely.geometry import Point
            c = inner.centroid
            ring = Point(c.x, c.y).buffer(min(22.0, (inner.area ** 0.5)/3)).exterior
            path = LineString(list(ring.coords)).intersection(inner)
            for k, g in enumerate(getattr(path, 'geoms', [path])):
                if g.geom_type == 'LineString':
                    lines.append(f'    <poly id="{key}_path{k}" type="path" color="226,214,180" fill="0" '
                                 f'lineWidth="2.0" layer="-1.7" shape="{isc._fmt(g.coords)}"/>')
            x0, y0, x1, y1 = inner.bounds
            n = 0
            for gy in range(int(y0) + 4, int(y1), 9):
                for gx in range(int(x0) + 4 + (gy // 9 % 2) * 4, int(x1), 9):
                    t = Point(gx + rng.uniform(-2, 2), gy + rng.uniform(-2, 2)).buffer(rng.uniform(2.0, 3.4), 6)
                    if inner.buffer(-1.0).contains(t) and not t.intersects(path.buffer(2.5)):
                        lines.append(f'    <poly id="{key}_tree{n}" type="tree" color="62,108,58" fill="1" '
                                     f'layer="-1.6" shape="{isc._fmt(t.exterior.coords)}"/>')
                        n += 1
        for k, (b, roof) in enumerate(blds):
            lines += isc._bld(f'{key}_b{k}', b, roof)
        # 간판: 단지·학교·공원은 이름 하나, 상가는 건물마다 이름 하나
        if names:
            if kind in ('shops', 'villa') and len(blds) >= len(names) > 1:
                picks = [blds[round(i*(len(blds) - 1)/(len(names) - 1))][0] for i in range(len(names))]
                for i, (nm, b) in enumerate(zip(names, picks)):
                    c = b.centroid
                    lines.append(isc._sign(f'{key}_{i}', [nm], isc.SIGN[kind], (c.x, c.y), font_m=2.0))
            else:
                c = inner.centroid if inner.contains(inner.centroid) else inner.representative_point()
                color = isc.SIGN['park'] if kind in ('park', 'school') else isc.SIGN[kind]
                lines.append(isc._sign(key, names[:1], (30, 80, 160) if kind == 'school' else color,
                                       (c.x, c.y), font_m=3.0 if kind == 'apt' else 2.6))
        report[key] = len(blds)
    # 숲: 남는 땅
    forest = land
    for u in used:
        forest = forest.difference(u.buffer(1.0))
    n_forest = 0
    from shapely.geometry import Point
    for g in getattr(forest, 'geoms', [forest]):
        if g.geom_type != 'Polygon' or g.area < 300 or g.buffer(-4.0).is_empty:
            continue                                # 구역 사이 가는 틈은 숲으로 칠하지 않는다 (블록 바닥색으로 둔다)
        g2 = g.buffer(-1.0)
        if g2.is_empty:
            continue
        for gg in isc._no_holes(g2):
            lines.append(f'    <poly id="{tag}_forest_{n_forest}" type="forest" color="{",".join(map(str, isc.FOREST))}" '
                         f'fill="1" layer="-1.85" shape="{isc._fmt(gg.exterior.coords)}"/>')
            x0, y0, x1, y1 = gg.bounds
            sp = 14.0
            y = y0 + sp/2
            while y < y1:
                x = x0 + sp/2 + (sp/2 if int((y - y0)/sp) % 2 else 0)
                while x < x1:
                    t = Point(x + rng.uniform(-4, 4), y + rng.uniform(-4, 4)).buffer(rng.uniform(2.2, 4.2), 6)
                    if gg.contains(t):
                        lines.append(f'    <poly id="{tag}_tree_{n_forest}_{len(lines)}" type="tree" '
                                     f'color="{",".join(map(str, isc.TREE))}" fill="1" layer="-1.82" '
                                     f'shape="{isc._fmt(t.exterior.coords)}"/>')
                    x += sp
                y += sp
            n_forest += 1
    if island is not None:
        isl = island.difference(paint_u.buffer(0.2))
        isl = max(getattr(isl, 'geoms', [isl]), key=lambda g: g.area)
        lines.append(f'    <poly id="wg_island_curb" type="sidewalk" color="{",".join(map(str, PAVING))}" fill="1" '
                     f'layer="-1.9" shape="{isc._fmt(isl.exterior.coords)}"/>')
        grass = isl.buffer(-2.5)
        if not grass.is_empty:
            grass = max(getattr(grass, 'geoms', [grass]), key=lambda g: g.area)
            lines.append(f'    <poly id="wg_island_grass" type="grass" color="104,146,84" fill="1" layer="-1.8" '
                         f'shape="{isc._fmt(grass.exterior.coords)}"/>')
            x0, y0, x1, y1 = grass.bounds
            k = 0
            for gy in range(int(y0), int(y1) + 1, 8):
                for gx in range(int(x0) + (gy // 8 % 2) * 4, int(x1) + 1, 8):
                    t = Point(gx + rng.uniform(-1.5, 1.5), gy + rng.uniform(-1.5, 1.5)).buffer(rng.uniform(2.2, 3.4), 8)
                    if grass.buffer(-0.8).contains(t):
                        lines.append(f'    <poly id="wg_island_tree_{k}" type="tree" color="62,108,58" fill="1" '
                                     f'layer="-1.6" shape="{isc._fmt(t.exterior.coords)}"/>')
                        k += 1
        report['island_trees'] = k if not grass.is_empty else 0
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    report['forest_polys'] = n_forest
    report['out'] = out
    return report


if __name__ == '__main__':
    import sys
    sys.path.insert(0, isc.BASE)
    print(build())
