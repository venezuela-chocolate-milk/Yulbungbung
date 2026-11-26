"""마전동 동쪽 (원당대로 북쪽, 완정로 곡선 ~ 서곶로) 상가·주택·별무리공원을 뭉텅이로 그린다. (화면 표시 전용)

    python -m env.roundabout_majeon_east_scenery

  - 2026-11-12 네이버 위성 지도 기준, 길(env.roundabout_majeon_east)로 나뉜 블록마다 상대 배치만 살렸다.
  - 그리는 방식은 env.roundabout_wanggil_scenery.build 와 같다 (블록 바닥 → 구역 → 건물·간판, 남는 땅은 숲).
출력: env/sumo_roundabout/roundabout.majeon_east.add.xml
"""
import os

from env import roundabout_wanggil_scenery as wg

OUT = os.path.join(wg.NET_DIR, 'roundabout.majeon_east.add.xml')
# 채울 땅 범위: 완정로 곡선 · 위쪽 변 길 · 서곶로 연장 · 원당대로 안쪽
HINT = [(1945, -470), (1980, -330), (2025, -240), (2050, -185), (2092, -160), (2150, -152), (2270, -195), (2262, -400),
        (2245, -540), (2212, -645), (2180, -690), (2075, -682), (1940, -662), (1942, -575)]
# (키, 모양, 종류, 간판, 선택값) — 블록: 가운데 길 me_s1(x≈2050) 서/동, 동서 골목 me_e1(y≈-520)·me_e2(y≈-390)로 나뉜다
ZONES = [
    # ── 가운데 길 서쪽 ──
    ('me_kb', [(1938, -690), (2048, -700), (2048, -605), (1938, -592)], 'shops',
     ['KB국민은행', 'GS25', '검단종합동물병원'], dict(angle=-8, bld=(20, 14))),
    ('me_hong', [(1938, -600), (2048, -610), (2048, -518), (1938, -505)], 'shops',
     ['홍콩반점0410', '더벤티', '노랑강정'], dict(angle=-8, bld=(18, 13))),
    ('me_dream', [(1940, -515), (2048, -518), (2050, -455), (1945, -452)], 'shops', ['드림식자재마트'],
     dict(angle=-8, bld=(40, 22))),
    ('me_yeontan', [(1945, -458), (2050, -458), (2054, -392), (1975, -388)], 'shops',
     ['마전동연탄집', '짱어랑소랑', '왕뼈감자탕', '블루핸즈'], dict(angle=-8, bld=(16, 12))),
    ('me_byeol', [(1975, -392), (2058, -392), (2092, -150), (2010, -200)], 'park', ['별무리공원'], {}),
    # ── 가운데 길 동쪽 ──
    ('me_nakwon', [(2052, -700), (2215, -700), (2245, -530), (2052, -522)], 'shops',
     ['낙원요양원', '하루'], dict(angle=-8, bld=(26, 18))),
    ('me_arad', [(2054, -518), (2245, -535), (2262, -398), (2058, -392)], 'shops',
     ['아라들깨삼계탕', '완정프리미엄슈퍼손세차', '노블레스'], dict(angle=-8, bld=(22, 15))),
    ('me_eunha', [(2058, -388), (2262, -396), (2266, -290), (2074, -282)], 'shops',
     ['은하유치원', '마전5동마을회관', '쉐보레바로서비스'], dict(angle=-8, bld=(20, 14))),
    ('me_hongik', [(2072, -286), (2266, -294), (2272, -190), (2150, -150), (2092, -150)], 'shops',
     ['홍익돈까스', 'CU', '카리스카페'], dict(angle=-8, bld=(18, 13))),
]


def build(net_path=os.path.join(wg.NET_DIR, 'roundabout.net.xml'), out=OUT):
    return wg.build(net_path, out, zones=ZONES, hint_pts=HINT, tag='me', island=False,
                    title='마전동 동쪽 상가·주택·별무리공원')


if __name__ == '__main__':
    import sys
    sys.path.insert(0, wg.isc.BASE)
    print(build())
