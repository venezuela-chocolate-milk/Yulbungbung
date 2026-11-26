"""마전동 북쪽 (마전역 동쪽, 완정로 북쪽 ~ 서곶로) 아파트·마전초·근린공원·상가를 뭉텅이로 그린다. (화면 표시 전용)

    python -m env.roundabout_majeon_north_scenery

  - 2026-11-12 네이버 위성 지도 기준, 가운데 큰 길(mx_mc*) 남쪽: 서→동 마전영남탑스빌 · 마전초등학교/검단2차풍림아이원 ·
    동아아파트/태화프라자·가대지국밥, 북쪽: 검단지구택지19호근린공원 · 검단우림필유 · 인천마전현대.
  - 위쪽은 길이 없어 열려 있으므로 범위 다각형 안에서 도로만 뺀 땅을 쓴다 (clip).
출력: env/sumo_roundabout/roundabout.majeon_north.add.xml
"""
import os

from env import roundabout_wanggil_scenery as wg

OUT = os.path.join(wg.NET_DIR, 'roundabout.majeon_north.add.xml')
# 범위: 서쪽 입구 길 · 완정로 · 마전동 동쪽 위쪽 변 길 · 서곶로 연장 · 위 끝
HINT = [(1462, -12), (1452, -45), (1478, -98), (1530, -140), (1945, -457), (1979, -330), (2023, -242), (2048, -178), (2090, -157),
        (2246, -194), (2295, -190), (2318, 215), (1560, 232),
        # 2026-11-12: 동쪽 큰길(gd_e*)과 근린공원 사이 삼각형까지 (동쪽 큰길 중심선 따라)
        (1363, 232), (1384, 185), (1410, 155), (1494, 95), (1515, 70)]
# 2026-11-13 사용자 요청: 큰 길 북쪽 3구역(19호근린공원·검단우림필유·인천마전현대) 전체 삭제 — 바닥 블록도 비운다
ZONES = [
    ('mx_taehwa', [(2190, -185), (2290, -195), (2295, -60), (2195, -60)], 'shops', ['태화프라자', '가대지국밥'],
     dict(angle=-3, bld=(22, 15))),
    ('mx_donga', [(2095, -170), (2295, -200), (2300, 105), (2100, 110)], 'apt', ['동아아파트'],
     dict(angle=-3, slab=(36, 12), gap=(10, 16))),
    # 2026-11-14 사용자 요청: 마전초등학교 삭제 → 그 자리에 검단우림필유아파트 (주차장은 만들지 않음)
    ('mx_urim', [(1795, 0), (2075, -20), (2080, 120), (1800, 135)], 'apt', ['검단우림필유아파트'],
     dict(angle=-3, slab=(36, 12), gap=(10, 16))),
    ('mx_pungrim2', [(1760, -320), (1945, -460), (2085, -160), (2080, -15), (1790, 5)], 'apt',
     ['검단2차풍림아이원아파트'], dict(angle=-37, slab=(34, 12), gap=(10, 16))),
    ('mx_yeongnam', [(1500, -120), (1770, -320), (1790, 130), (1530, 100)], 'apt', ['마전영남탑스빌아파트'],
     dict(angle=-37, slab=(34, 12), gap=(10, 16))),
]
REMOVED_ZONES = [
    # 2026-11-12 사용자 요청: 서쪽 삼각형(동쪽 큰길 · 마전역 북쪽 길 사이)까지 같은 공원으로 자연스럽게
    ('mx_park19', [(1363, 232), (1384, 185), (1410, 155), (1494, 95), (1515, 70), (1530, 62), (1600, 100),
                   (1790, 140), (1790, 232)], 'park', ['검단지구택지19호근린공원'], {}),
    ('mx_urim_old', [(1795, 145), (2095, 120), (2100, 230), (1795, 230)], 'apt', ['검단우림필유아파트'],
     dict(angle=-3, slab=(36, 12), gap=(10, 16))),
    ('mx_hyundai', [(2105, 120), (2310, 90), (2320, 230), (2105, 230)], 'apt', ['인천마전현대아파트'],
     dict(angle=-3, slab=(32, 12), gap=(10, 16))),
]


def zones():
    """2026-11-12: 영남탑스빌 아래쪽 띠는 완정로 상가 줄 + 마전어린이집 (env.roundabout_majeon_daycare) — 단지에서 빼고 상가 구역을 앞에 둔다."""
    from shapely.geometry import Polygon
    from env.roundabout_majeon_daycare import scenery_zones
    strip, shop_zones = scenery_zones()
    out = []
    for z in ZONES:
        if z[0] == 'mx_yeongnam':
            g = Polygon(z[1]).difference(strip.buffer(1.0))
            g = max(getattr(g, 'geoms', [g]), key=lambda q: q.area)
            z = (z[0], [tuple(c) for c in g.exterior.coords][:-1]) + tuple(z[2:])
        out.append(z)
    return shop_zones + out


def _drop_removed_ground(out):
    """REMOVED_ZONES 자리에 남는 바닥(mx_block_*)과 빈 땅 자동 숲(mx_forest_k + mx_tree_k_*)을 지운다.
    큰 길 남쪽 작은 녹지 띠처럼 일부만 걸치는 것은 남긴다 (90% 이상 안에 든 것만 삭제)."""
    import re
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    gone = unary_union([Polygon(z[1]) for z in REMOVED_ZONES]).buffer(0)
    lines = open(out, encoding='utf-8').readlines()
    drop_forest = set()
    for line in lines:
        r = re.search(r'<poly id="mx_forest_(\d+)".*?shape="([^"]+)"', line)
        if r:
            g = Polygon([tuple(map(float, c.split(','))) for c in r.group(2).split()]).buffer(0)
            if g.area and g.intersection(gone).area / g.area > 0.9:
                drop_forest.add(r.group(1))
    keep = []
    for line in lines:
        r = re.search(r'<poly id="mx_block_\d+".*?shape="([^"]+)"', line)
        if r:
            g = Polygon([tuple(map(float, c.split(','))) for c in r.group(1).split()]).buffer(0)
            if g.area and g.intersection(gone).area / g.area > 0.9:
                continue
        r = re.search(r'<poly id="mx_(?:forest|tree)_(\d+)[_"]', line)
        if r and r.group(1) in drop_forest:
            continue
        keep.append(line)
    with open(out, 'w', encoding='utf-8') as f:
        f.writelines(keep)


DONGA_LOT = os.path.join(wg.NET_DIR, 'roundabout.donga_parking.add.xml')   # env.roundabout_donga_parking 이 그림


def _drop_under_lot(out, lot_file=DONGA_LOT):
    """2026-11-13: 동아아파트 주차장 자리의 동아아파트 동(겹치는 것)과 태화프라자 상가 구역(바닥·건물·간판 전부)을 뺀다.
    위쪽 동은 주차장이 없을 때와 같은 자리에 남는다 (주차장 통로는 build 에서 도로로 치지 않음)."""
    import re
    from shapely.geometry import Polygon
    if not os.path.exists(lot_file):
        return
    lot = None
    for line in open(lot_file, encoding='utf-8'):
        r = re.search(r'<poly id="pd_lot".*?shape="([^"]+)"', line)
        if r:
            lot = Polygon([tuple(map(float, c.split(','))) for c in r.group(1).split()]).buffer(2.0)
    if lot is None:
        return
    lines = open(out, encoding='utf-8').readlines()
    gone = set()
    for line in lines:
        r = re.search(r'<poly id="(mx_donga_b\d+)" .*?shape="([^"]+)"', line)
        if r and Polygon([tuple(map(float, c.split(','))) for c in r.group(2).split()]).intersects(lot):
            gone.add(r.group(1))
    keep = []
    for line in lines:
        r = re.search(r'<(?:poly|poi) id="([^"]+)"', line)
        if r:
            k = r.group(1)
            if k.startswith(('mx_taehwa', 'int_sign_mx_taehwa')) or re.sub(r'_(shadow|parapet|edge)$', '', k) in gone:
                continue
        keep.append(line)
    with open(out, 'w', encoding='utf-8') as f:
        f.writelines(keep)


def build(net_path=os.path.join(wg.NET_DIR, 'roundabout.net.xml'), out=OUT):
    # 어린이집 부지·주차장 그림(roundabout.daycare.add.xml)이 있으면 그 자리는 비운다
    info = wg.build(net_path, out, zones=zones(), hint_pts=HINT, tag='mx', island=False, clip=True,
                    title='마전동 북쪽 아파트·마전초·근린공원·완정로 상가', taken_files=('nw', 'majeon_east', 'daycare'), ignore_prefix=('pd_',))
    _drop_removed_ground(out)
    _drop_under_lot(out)
    return info


if __name__ == '__main__':
    import sys
    sys.path.insert(0, wg.isc.BASE)
    print(build())
