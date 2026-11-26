"""검단사거리 북동쪽 (완정로 북쪽 · 검단로 · 동쪽 큰길 · 마전역 북쪽 길) 블록을 뭉텅이로 채운다. (화면 표시 전용)

    python -m env.roundabout_geomdan_scenery

  - 2026-11-12 네이버 지도 캡처 기준. 블록 = env.roundabout_geomdan 도로로 둘러싸인 땅 (블록 둘레 교차점들의 가운데가 기준점).
  - 서쪽 구시가지: 상가 블록마다 가게 이름 몇 개(건물 하나 = 이름 하나), 빌라 블록.
  - 가운데: 서구청 검단행정과, 유호아파트·마을회관, 뉴월드빌, 사프빌라, 방현공원, 우방아이유쉘.
  - 동쪽: 인천검단아이파크, 다목적체육관, 힐스테이트 1·2차, 메가MGC커피·세븐일레븐, 검단아이파크2차.
  건물·간판 스타일은 env.roundabout_wanggil_scenery.build 그대로.
출력: env/sumo_roundabout/roundabout.geomdan.add.xml
"""
import os
import xml.etree.ElementTree as ET

from env import roundabout_wanggil_scenery as wg
from env.roundabout_geomdan import NV

OUT = os.path.join(wg.NET_DIR, 'roundabout.geomdan.add.xml')
WJ_ANGLE = -30          # 완정로 방향
GD_ANGLE = 22           # 검단로 방향
# (키, 블록 둘레 교차점들, 나누기(None | ('near', 교차점 a, b, 폭 m) | ('west', 교차점 a, b) | ('east', a, b)),
#  종류, 간판, 선택값) — 앞에 적은 구역이 땅을 먼저 차지한다
BLOCKS = [
    ('gd_a', ['T_end_main_end', 'gd_G1', 'gd_J1', 'gd_W1'], None, 'shops',
     ['신전떡볶이', '컴포즈커피', '파리바게뜨', '연탄불고기', '신협'], dict(angle=WJ_ANGLE, bld=(16, 12))),
    ('gd_b', ['gd_J1', 'gd_W1', 'gd_W2'], None, 'shops',
     ['클라우스빌', 'CU', '축협', '세븐일레븐', '포그니재활실버케어'], dict(angle=WJ_ANGLE, bld=(16, 12))),
    ('gd_c', ['gd_G1', 'gd_J1', 'gd_JS1', 'gd_JP', 'gd_G2'], None, 'shops',
     ['농협', '이지화인프라자', 'GS25'], dict(angle=GD_ANGLE, bld=(18, 12))),
    ('gd_d', ['gd_J1', 'gd_JS1', 'gd_JQ1', 'gd_J2'], None, 'shops',
     ['서구청 검단행정과', '달콤'], dict(angle=GD_ANGLE, bld=(26, 18))),
    ('gd_e', [NV(425, 205)], None, 'villa',                       # 마을회관 자리 (둘레 교차점 가운데는 156번안길 동쪽 블록)
     ['마을회관', '유호아파트'], dict(angle=0, bld=(16, 12))),
    ('gd_f', [NV(455, 140)], None, 'villa', ['뉴월드빌2차'], dict(angle=GD_ANGLE)),     # 156번안길 동쪽 (삼각형 가운데는 길이 휘어 옆 블록)
    ('gd_sapeu', ['gd_G2', 'gd_JT', 'gd_E1', 'gd_JD', 'gd_JE', 'gd_JP'], ('near', 'gd_JP', 'gd_JE', 32.0), 'villa',
     ['사프빌라'], dict(angle=0)),
    ('gd_park', ['gd_G2', 'gd_JT', 'gd_E1', 'gd_JD', 'gd_JE', 'gd_JP'], None, 'park', ['방현공원'], {}),
    ('gd_gym', ['gd_JD', 'gd_E1', 'gd_E2', 'gd_J540', 'gd_JH', 'gd_JN'], ('near', 'gd_J540', 'gd_E2', 55.0), 'shops',
     ['인천서구검단다목적체육관'], dict(angle=-10, bld=(30, 20))),
    ('gd_ipark', ['gd_JD', 'gd_E1', 'gd_E2', 'gd_J540', 'gd_JH', 'gd_JN'], None, 'apt', ['인천검단아이파크'],
     dict(angle=0, slab=(30, 11), gap=(9, 14))),
    ('gd_mega', ['gd_JH', 'gd_J540', 'gd_W4', 'gd_W3'], ('near', 'gd_J540', 'gd_W4', 34.0), 'shops',
     ['메가MGC커피', '세븐일레븐'], dict(angle=WJ_ANGLE, bld=(16, 12))),
    ('gd_hill2', ['gd_JH', 'gd_J540', 'gd_W4', 'gd_W3'], ('west', 'gd_JH', 'gd_W4'), 'apt', ['검단힐스테이트2차'],
     dict(angle=0, slab=(30, 11), gap=(9, 14))),
    ('gd_hill1', ['gd_JH', 'gd_J540', 'gd_W4', 'gd_W3'], None, 'apt', ['검단힐스테이트1차'],
     dict(angle=0, slab=(30, 11), gap=(9, 14))),
    ('gd_woobang', ['gd_J2', 'gd_W2', 'gd_W3', 'gd_JH', 'gd_JN', 'gd_JQ1'], None, 'apt', ['검단2차우방아이유쉘'],
     dict(angle=WJ_ANGLE, slab=(30, 11), gap=(9, 14))),
    ('gd_ipark2', ['gd_J540', 'gd_E2', 'gd_TE', 'X_se_end', 'gd_W4'], None, 'apt', ['검단아이파크2차'],
     dict(angle=WJ_ANGLE, slab=(30, 11), gap=(9, 14))),
]


def _faces(net_path):
    """도로 바닥(차로·교차점)을 뺀 면들."""
    from shapely.geometry import LineString, Polygon, box
    from shapely.ops import unary_union
    net = ET.parse(net_path).getroot()
    paint = []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = wg.isc._read(l.get('shape'))
            if len(pts) >= 2:
                paint.append(LineString(pts).buffer(float(l.get('width', '3.2'))/2, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal' and len(wg.isc._read(s)) >= 3:
            paint.append(Polygon(wg.isc._read(s)).buffer(0))
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    faces = box(450, -200, 1700, 800).difference(unary_union(paint))
    return [g for g in getattr(faces, 'geoms', [faces]) if g.area > 400], J


def zones(net_path):
    from shapely.geometry import LineString, Point, Polygon
    faces, J = _faces(net_path)
    out, used = [], {}
    for key, ring, part, kind, names, opt in BLOCKS:
        pts = [J[n] if isinstance(n, str) else n for n in ring]
        seed = Point(sum(p[0] for p in pts)/len(pts), sum(p[1] for p in pts)/len(pts))
        face = next((f for f in faces if f.contains(seed)), None)
        if face is None:
            face = min(faces, key=lambda f: f.distance(seed))
        g = face
        if part is not None:
            if part[0] == 'near':
                g = face.intersection(LineString([J[part[1]], J[part[2]]]).buffer(part[3]))
            else:
                a, b = J[part[1]], J[part[2]]
                d = (b[0] - a[0], b[1] - a[1])
                n = (-d[1], d[0])
                if (n[0] < 0) != (part[0] == 'west'):      # 'west' = 서쪽(x 감소) 반평면, 'east' = 동쪽
                    n = (-n[0], -n[1])
                big = 2000.0
                half = Polygon([(a[0] - d[0]*3, a[1] - d[1]*3), (b[0] + d[0]*3, b[1] + d[1]*3),
                                (b[0] + d[0]*3 + n[0]*big, b[1] + d[1]*3 + n[1]*big),
                                (a[0] - d[0]*3 + n[0]*big, a[1] - d[1]*3 + n[1]*big)])
                g = face.intersection(half)
        if g.geom_type != 'Polygon':
            g = max(getattr(g, 'geoms', [g]), key=lambda x: x.area)
        used[key] = round(g.area)
        out.append((key, list(g.exterior.coords)[:-1], kind, names, opt))
    hint = [(560, 420), (650, 480), (870, 562), (1002, 628), (1075, 665), (1194, 531), (1273, 440), (1326, 368),
            (1350, 275), (1390, 170), (1507, 84), (1530, 50), (1462, -17), (1380, -60)]
    return out, hint, used


def build(net_path=os.path.join(wg.NET_DIR, 'roundabout.net.xml'), out=OUT):
    zs, hint, _ = zones(net_path)
    return wg.build(net_path, out, zones=zs, hint_pts=hint, tag='gd', island=False, clip=False,
                    title='검단사거리 북동쪽 상가·빌라·아파트·공원', taken_files=('shops', 'nw', 'majeon_north'))


if __name__ == '__main__':
    import sys
    sys.path.insert(0, wg.isc.BASE)
    print(build())
