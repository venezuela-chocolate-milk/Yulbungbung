"""완정로165번길 동쪽 · 왕길배수지공원 주변 그림 (2026-11-12 네이버 지도 캡처, 화면 표시 전용).

    python -m env.roundabout_baesuji_scenery

  - 순환 골목 안 빌라(유현샤인빌·파크빌·양지쉐르빌), 골목 남쪽 힐스타운빌라.
  - 왕길배수지공원: 네이버 모양(기울어진 사다리꼴) 그대로, 북쪽 가장자리 산책로·안쪽 순환 산책로·남서쪽 산책로(보행로, 그림만),
    가운데 배수지 시설, 나무.
  - 165번길 쪽·동쪽 골목 쪽 작은 건물: 네이버 캡처의 흰 건물 네모를 그대로 옮겼다 (도로와 겹치는 것은 뺀다).
  - 남는 땅은 숲이 아니라 빈 땅: env.roundabout_interior_scenery 의 'baesuji_vacant' 구역이 깐다. 마전동묘원 쪽 녹지만 숲.
  SUMO 의 165번길이 네이버보다 남쪽으로 갈수록 동쪽으로 벗어나 있어, 공원 위도부터는 그 차이만큼 동쪽으로 옮긴다
  (동쪽 골목 쪽으로 갈수록 0, 골목과 같은 좌표).
출력: env/sumo_roundabout/roundabout.baesuji.add.xml
"""
import os
import random
import xml.etree.ElementTree as ET

from env import roundabout_interior_scenery as isc
from env.roundabout_baesuji import NB, OLD_EAST

OUT = os.path.join(isc.NET_DIR, 'roundabout.baesuji.add.xml')
SIDEWALK = 2.0
# 165번길: 네이버(px) / SUMO 중심선
SPINE_NAV = [(1385, 173), (1392, 493), (1378, 693), (1358, 933), (1345, 1133), (1312, 1333)]
SPINE_SUMO = [(596, 190), (603, 90), (616, -20), (632, -130), (646, -230), (655, -322)]
PARK = [(1372, 647), (1685, 547), (1818, 507), (1892, 687), (1805, 727), (1685, 733), (1552, 793), (1458, 773), (1372, 700)]
PARK_PATHS = [
    [(1378, 655), (1530, 612), (1685, 568), (1805, 535)],                                    # 북쪽 가장자리 산책로
    [(1672, 653), (1685, 620), (1738, 593), (1778, 587), (1812, 607), (1818, 640), (1798, 667), (1765, 667),
     (1738, 680), (1712, 693), (1685, 687), (1672, 653)],                                    # 안쪽 순환 산책로
    [(1685, 687), (1625, 760)],                                                             # 남서쪽 산책로
    [(1530, 612), (1560, 700)],
]
RESERVOIR = [(1638, 645), (1700, 628), (1712, 672), (1650, 690)]                            # 배수지 시설
# 순환 골목 안 빌라 + 힐스타운빌라 (네이버에서 연한 회색이라 자동 추출이 안 돼 아이콘 자리에 직접)
# 순환 골목 안 빌라는 그 블록(165번길·133번길·순환 골목) 안에 2줄로 채운다. 힐스타운빌라는 골목 남쪽에 직접.
LOOP_SEED = (1470, 400)
LOOP_VILLA = (13.0, 11.0)
VILLAS = [((1405, 530), (1455, 562)), ((1470, 535), (1530, 567)), ((1420, 580), (1470, 603))]
NAMES = [('유현샤인빌', (1418, 327)), ('파크빌', (1492, 318)), ('양지쉐르빌', (1497, 448)), ('힐스타운빌라', (1472, 550))]
# 165번길 쪽·동쪽 골목 쪽 작은 건물: 2026-11-13 사용자 요청으로 네이버에서 옮긴 제각각 네모(예전 BUILDINGS) 대신
# 165번길과 나란한 반듯한 격자 하나로 다시 배치. 길(165번길·int_east)과 지운 동쪽 골목 자리 가까이만 채운다.
GRID_ANGLE = 8.0                 # 165번길 방향 (도, 반시계)
GRID_BLD = (18.0, 12.0)          # 건물 (길이, 폭) m
GRID_GAP = (8.0, 9.0)
GRID_NEAR = {'road': 42.0, 'old_east': 32.0}     # 길에서 이 거리 안 + 지운 골목 자리를 감싸는 격자 방향 네모 (여유 m)
GRID_ROADS = ('int_spine_b', 'int_spine_c', 'int_east')
PARK_GRASS = (146, 182, 112)
PATH = (226, 214, 180)


def _interp(pts, y, axis_y=1):
    """꺾은선에서 y 값(내림차순 꺾은선)의 x."""
    for a, b in zip(pts, pts[1:]):
        lo, hi = sorted((a[axis_y], b[axis_y]))
        if lo <= y <= hi and a[axis_y] != b[axis_y]:
            t = (y - a[axis_y]) / (b[axis_y] - a[axis_y])
            return a[0] + (b[0] - a[0])*t
    return (pts[0] if abs(y - pts[0][axis_y]) < abs(y - pts[-1][axis_y]) else pts[-1])[0]


def T(px, py):
    """네이버 px → SUMO, 공원 위도부터는 165번길 차이만큼 동쪽으로 (동쪽 골목 쪽으로 갈수록 0)."""
    x, y = NB(px, py)
    nx = NB(_interp(SPINE_NAV, py), py)[0]
    dx = _interp(SPINE_SUMO, y) - nx
    ramp = min(1.0, max(0.0, (py - 600) / 100.0))
    w = min(1.0, max(0.0, (1850 - px) / 450.0))
    return (x + dx*ramp*w, y)


def _grid_box(pts, pad, origin):
    """꺾은선을 감싸는 격자 방향(GRID_ANGLE) 네모 (pad 만큼 여유). 지운 골목 자리를 반듯한 한 덩어리로 채우려고."""
    from shapely import affinity
    from shapely.geometry import LineString, box
    r = affinity.rotate(LineString(pts), -GRID_ANGLE, origin=origin)
    x0, y0, x1, y1 = r.bounds
    return affinity.rotate(box(x0 - pad, y0 - pad, x1 + pad, y1 + pad), GRID_ANGLE, origin=origin)


def build(net_path=os.path.join(isc.NET_DIR, 'roundabout.net.xml'), out=OUT):
    from shapely.geometry import LineString, Polygon, Point, box
    from shapely.ops import unary_union
    net = ET.parse(net_path).getroot()
    os.makedirs(isc.IMG_DIR, exist_ok=True)
    rng = random.Random(7)
    roads = []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = isc._read(l.get('shape'))
            if len(pts) >= 2:
                roads.append(LineString(pts).buffer(float(l.get('width', '3.2'))/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and len(isc._read(s)) >= 3:
            roads.append(Polygon(isc._read(s)).buffer(SIDEWALK))
    road = unary_union(roads)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_baesuji_scenery.py 가 생성. 165번길 동쪽 빌라·왕길배수지공원·작은 건물 (주행에는 영향 없음) -->',
             '<additional>']
    rep = {}
    # ── 공원 ──
    park = Polygon([T(*p) for p in PARK]).buffer(0).difference(road)
    park = max(getattr(park, 'geoms', [park]), key=lambda g: g.area)
    lines += isc._ground('bj_park', park, PARK_GRASS, '-1.75')
    paths = []
    for k, pp in enumerate(PARK_PATHS):
        ln = LineString([T(*p) for p in pp]).intersection(park.buffer(-0.5))
        for m, g in enumerate(getattr(ln, 'geoms', [ln])):
            if g.geom_type == 'LineString' and g.length > 2:
                paths.append(g)
                lines.append(f'    <poly id="bj_park_path{k}_{m}" type="path" color="{",".join(map(str, PATH))}" fill="0" '
                             f'lineWidth="2.2" layer="-1.7" shape="{isc._fmt(g.coords)}"/>')
    res = Polygon([T(*p) for p in RESERVOIR])
    lines += isc._bld('bj_reservoir', res, (200, 204, 208), layer=-1.3)
    free = park.buffer(-1.5).difference(unary_union([g.buffer(2.6) for g in paths] + [res.buffer(2.5)]))
    n = 0
    x0, y0, x1, y1 = park.bounds
    for gy in range(int(y0) + 3, int(y1), 8):
        for gx in range(int(x0) + 3 + (gy // 8 % 2)*4, int(x1), 8):
            if rng.random() < 0.35:
                continue
            t = Point(gx + rng.uniform(-2, 2), gy + rng.uniform(-2, 2)).buffer(rng.uniform(1.8, 3.2), 6)
            if free.contains(t):
                lines.append(f'    <poly id="bj_park_tree{n}" type="tree" color="62,108,58" fill="1" layer="-1.6" '
                             f'shape="{isc._fmt(t.exterior.coords)}"/>')
                n += 1
    c = res.centroid
    lines.append(isc._sign('bj_park', ['왕길배수지공원'], isc.SIGN['park'], (c.x, c.y - 9.0), font_m=2.6))
    rep['park_trees'] = n
    # ── 빌라·작은 건물 ──
    blds = []
    for a, b in VILLAS:
        blds.append(('villa', Polygon([T(a[0], a[1]), T(b[0], a[1]), T(b[0], b[1]), T(a[0], b[1])])))
    # 순환 골목 안 블록
    from shapely import affinity
    paint = unary_union([LineString(isc._read(l.get('shape'))).buffer(float(l.get('width', '3.2'))/2, cap_style=2)
                         for e in net.findall('edge') for l in e.findall('lane') if len(isc._read(l.get('shape'))) >= 2])
    seed = Point(T(*LOOP_SEED))
    faces = seed.buffer(250).envelope.difference(paint)
    face = next((g for g in getattr(faces, 'geoms', [faces]) if g.contains(seed)), None)
    if face is not None:
        inner = face.difference(road).buffer(-1.0)
        inner = max(getattr(inner, 'geoms', [inner]), key=lambda g: g.area)
        for g in isc._rect_fill(inner, 0, LOOP_VILLA, (5.0, 6.0), 0.15, rng):
            blds.insert(0, ('villa', g))
    # 반듯한 격자 (빈 땅 구역 안, 공원·묘지·순환 골목 블록 빼고, 길·지운 골목 자리 가까이만)
    vac = next(Polygon(z[2]) for z in isc.ZONES if z[0] == 'baesuji_vacant')
    cem = next(Polygon(z[2]) for z in isc.ZONES if z[0] == 'cemetery')
    region = vac.difference(road.buffer(1.5)).difference(park.buffer(4.0)).difference(cem.buffer(4.0))
    if face is not None:
        region = region.difference(face.buffer(2.0))
    near = unary_union([LineString(isc._read(l.get('shape'))).buffer(GRID_NEAR['road'])
                        for e in net.findall('edge') if e.get('id', '').startswith(GRID_ROADS) and e.get('function') != 'internal'
                        for l in e.findall('lane')] + [_grid_box(OLD_EAST, GRID_NEAR['old_east'], region.centroid)])
    n_grid = 0
    for g in isc._rect_fill(region, GRID_ANGLE, GRID_BLD, GRID_GAP):
        if near.contains(g.centroid):
            blds.append(('small', g))
            n_grid += 1
    rep['grid'] = n_grid
    placed, dropped, nudged = [], 0, 0
    for kind, g in blds:
        g = g.buffer(0)
        ok = None
        for sc in (1.0, 0.85, 0.7):
            gg = affinity.scale(g, sc, sc, origin='centroid')
            if not gg.intersects(road):
                ok = gg
                break
            # 가장 가까운 도로에서 멀어지는 쪽으로 최대 8 m 밀어 본다
            c = gg.centroid
            from shapely.ops import nearest_points
            q = nearest_points(road, c)[0]
            dx, dy = c.x - q.x, c.y - q.y
            L = (dx*dx + dy*dy) ** 0.5 or 1.0
            for step in (2.0, 4.0, 6.0, 8.0):
                g2 = affinity.translate(gg, dx/L*step, dy/L*step)
                if not g2.intersects(road):
                    ok = g2
                    break
            if ok is not None:
                nudged += 1
                break
        if ok is None or ok.intersects(park) or any(ok.intersects(p.buffer(1.0)) for _, p in placed):
            dropped += 1
            continue
        placed.append((kind, ok))
    rep['nudged'] = nudged
    for k, (kind, g) in enumerate(placed):
        roofs = isc.VILLA_ROOFS if kind == 'villa' else isc.SHOP_ROOFS
        lines += isc._bld(f'bj_b{k}', g, roofs[k % len(roofs)])
    for i, (nm, (px, py)) in enumerate(NAMES):
        q = Point(T(px, py))
        best = min((p for kd, p in placed if kd == 'villa'), key=lambda p: p.distance(q))
        cc = best.centroid
        lines.append(isc._sign(f'bj_villa{i}', [nm], isc.SIGN['villa'], (cc.x, cc.y), font_m=2.0))
    rep['buildings'] = len(placed)
    rep['dropped'] = dropped
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    rep['out'] = out
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, isc.BASE)
    print(build())
