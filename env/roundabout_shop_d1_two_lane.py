"""큰손왕만두 오른쪽 상가 골목 (D1: shop_N1 ─ shop_C2N ─ shop_C3N) 편도 2차로 (2026-11-15 사용자 요청).

  불법 주정차 구간(env.roundabout_illegal_parking)이 이 골목 양방향 바깥 차로에 생기므로,
  마전동 골목처럼 중앙선 양쪽에 한 차로씩 더한다. 안쪽(중앙선 쪽) 차로 = 예전 차로 자리 그대로, 바깥에 새 차로.
  세 교차점만 다시 만든다 (env.roundabout_junction_patch). 연결 규칙:
    - 직진: 두 차로 모두 (2차로 ↔ 2차로는 차로끼리, 1차로 골목 → 2차로는 두 차로로 갈라짐)
    - 좌회전: 안쪽 차로(1)에서 / 안쪽 차로로,  우회전: 바깥 차로(0)에서 / 바깥 차로로
  노면 표시: 노란 중앙선, 같은 방향 두 차로 사이 흰 점선, 차로별 화살표, C2N 좌회전 유도선(실제 회전 경로 기준).
  골목 옆 보도·건물 그림이 넓어진 차로와 겹치면 차로 바깥 0.3 m 까지 깎는다.

roundabout_south_extend 가 늘어나는 차로 시작점 고치기(9-20) 바로 앞에서 부른다 (주차장 단계가 117 골목을 나눈 뒤).
"""
import math
import os
import xml.etree.ElementTree as ET

JS = ['shop_N1', 'shop_C2N', 'shop_C3N']
WIDE = ['shop_d1a_f', 'shop_d1a_r', 'shop_d1b_f', 'shop_d1b_r']
LANE_W = 3.2
CON = [
    # shop_N1 (서쪽 끝): 117 골목(주차장 쪽)·위 C1·아래 V1
    ('shop_117_f_pk_1', 0, 'shop_d1a_f', 0), ('shop_117_f_pk_1', 0, 'shop_d1a_f', 1),
    ('shop_c1_f', 0, 'shop_d1a_f', 1),
    ('shop_v1_r_int_1', 0, 'shop_d1a_f', 0),
    ('shop_d1a_r', 0, 'shop_c1_r', 0),
    ('shop_d1a_r', 0, 'shop_117_r', 0), ('shop_d1a_r', 1, 'shop_117_r', 0),
    ('shop_d1a_r', 1, 'shop_v1_f', 0),
    # shop_C2N (네 갈래)
    ('shop_c2a_f', 0, 'shop_d1a_r', 0), ('shop_c2a_f', 0, 'shop_d1b_f', 1),
    ('shop_c2b_r', 0, 'shop_d1b_f', 0), ('shop_c2b_r', 0, 'shop_d1a_r', 1),
    ('shop_d1a_f', 0, 'shop_c2b_f', 0), ('shop_d1a_f', 0, 'shop_d1b_f', 0), ('shop_d1a_f', 1, 'shop_d1b_f', 1),
    ('shop_d1a_f', 1, 'shop_c2a_r', 0),
    ('shop_d1b_r', 0, 'shop_c2a_r', 0), ('shop_d1b_r', 0, 'shop_d1a_r', 0), ('shop_d1b_r', 1, 'shop_d1a_r', 1),
    ('shop_d1b_r', 1, 'shop_c2b_f', 0),
    # shop_C3N (동쪽 끝, 세 갈래)
    ('shop_c3a_f', 0, 'shop_d1b_r', 0),
    ('shop_c3b_r', 0, 'shop_d1b_r', 1),
    ('shop_d1b_f', 0, 'shop_c3b_f', 0),
    ('shop_d1b_f', 1, 'shop_c3a_r', 0),
]
# 넓어진 차로와 겹치면 깎을 그림 파일
ADD_FILES = ['roundabout.shops.add.xml']   # 겹치는 것은 골목 보도(shop_paving_*)뿐 (건물은 안 겹침)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def apply_net(net_path):
    from env.roundabout_junction_patch import patch
    info = patch(net_path, JS, {e: 2 for e in WIDE}, CON)
    return {'shop_d1_links': info['patched_links'], 'shop_d1_new_paths': info['patched_new_paths']}


def _road_area(net_path):
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    root = ET.parse(net_path).getroot()
    parts = []
    for e in root.findall('edge'):
        if e.get('id') in WIDE:
            for l in e.findall('lane'):
                parts.append(LineString(_read(l.get('shape'))).buffer(LANE_W / 2 + 0.3, cap_style=2))
    return unary_union(parts)


def clip_scenery(map_dir, net_path):
    """넓어진 차로와 겹치는 채운 그림(보도·건물·블록 바닥)을 차로 바깥까지 깎는다."""
    from shapely.geometry import Polygon
    road = _road_area(net_path)
    n = 0
    for fn in ADD_FILES:
        path = os.path.join(map_dir, fn)
        if not os.path.exists(path):
            continue
        tree = ET.parse(path)
        root = tree.getroot()
        changed = False
        for p in list(root.findall('poly')):
            if p.get('fill') in ('0', 'false') or not p.get('shape'):
                continue
            pts = _read(p.get('shape'))
            if len(pts) < 3:
                continue
            poly = Polygon(pts).buffer(0)
            if not poly.intersects(road) or poly.intersection(road).area < 0.05:
                continue
            rest = poly.difference(road)
            parts = [g for g in getattr(rest, 'geoms', [rest]) if g.geom_type == 'Polygon' and g.area > 1.0]
            idx = list(root).index(p)
            root.remove(p)
            for k, g in enumerate(parts):
                el = ET.Element('poly', dict(p.attrib))
                el.set('id', p.get('id') if k == 0 else f"{p.get('id')}_d1c{k}")
                el.set('shape', _write(list(g.simplify(0.02).exterior.coords)))
                root.insert(idx + k, el)
            n += 1
            changed = True
        if changed:
            ET.indent(tree, space='    ')
            tree.write(path, encoding='utf-8', xml_declaration=True)
    return n


def _poi_rect(x, y, w, h, ang):
    """SUMO 그림 POI 의 네 모서리 (angle 은 시계 방향 도). (모서리, 가로축, 세로축)"""
    r = math.radians(-ang)
    ex, ey = (math.cos(r), math.sin(r)), (-math.sin(r), math.cos(r))
    pts = [(x + ex[0]*sx*w/2 + ey[0]*sy*h/2, y + ex[1]*sx*w/2 + ey[1]*sy*h/2)
           for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return pts, ex, ey


def shrink_images(map_dir, net_path):
    """넓어진 차로를 덮는 건물 그림(POI)을 차로 쪽 변만 뒤로 물려 줄인다 (먼 쪽 변은 그대로)."""
    from shapely.geometry import Polygon
    road = _road_area(net_path)
    path = os.path.join(map_dir, ADD_FILES[0])
    tree = ET.parse(path)
    root = tree.getroot()
    n = 0
    for p in root.findall('poi'):
        if not p.get('imgFile'):
            continue
        x, y, w, h, ang = (float(p.get(k, 0)) for k in ('x', 'y', 'width', 'height', 'angle'))
        pts, ex, ey = _poi_rect(x, y, w, h, ang)
        if not Polygon(pts).intersects(road):
            continue
        best = None
        for axis, size in ((ex, w), (ey, h)):
            for sgn in (1, -1):                       # 줄이는 쪽 (+축 쪽 변을 물림 / -축 쪽 변을 물림)
                lo, hi = 0.0, size * 0.6
                def ok(t):
                    c = (x - sgn*axis[0]*t/2, y - sgn*axis[1]*t/2)
                    ww, hh = (w - t, h) if axis is ex else (w, h - t)
                    return not Polygon(_poi_rect(c[0], c[1], ww, hh, ang)[0]).intersects(road)
                if not ok(hi):
                    continue
                for _ in range(30):
                    mid = (lo + hi) / 2
                    lo, hi = (lo, mid) if ok(mid) else (mid, hi)
                if best is None or hi < best[0]:
                    best = (hi, axis, sgn)
        if best is None:
            continue
        t, axis, sgn = best
        p.set('x', f'{x - sgn*axis[0]*t/2:.3f}')
        p.set('y', f'{y - sgn*axis[1]*t/2:.3f}')
        if axis is ex:
            p.set('width', f'{w - t:.2f}')
        else:
            p.set('height', f'{h - t:.2f}')
        n += 1
    ET.indent(tree, space='    ')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return n


def apply_deco(deco_path, net_path):
    """D1 골목 표시 다시: 중앙선, 차로 사이 흰 점선, 차로별 화살표, C2N 좌회전 유도선."""
    from env.roundabout_builder import _arrow, _read_lane_shapes, _movement_path
    from env.roundabout_geometry import _offset, _slice
    from env.roundabout_shops import movements, ARROW_KINDS, _dashes, _length, YELLOW
    tree = ET.parse(deco_path)
    root = tree.getroot()
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    shapes = _read_lane_shapes(net_path)
    bases = [w[:-2] for w in WIDE if w.endswith('_f')]
    # 지울 것: D1 중앙선, D1 차로 화살표, C2N 좌회전 유도선 전부 (D1 에서 나가거나 D1 로 들어가는 회전 경로가 바뀜)
    kill = tuple([f'{b}_center' for b in bases] + [f'shop_arrow_{w}_' for w in WIDE] + ['shop_guide_', 'shopd1_'])
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith(kill):
            root.remove(p)
    h = LANE_W / 2
    n_line = 0
    for b in bases:
        # 노란 중앙선: _f 안쪽 차로(1) 왼쪽 경계
        ET.SubElement(root, 'poly', dict(id=f'{b}_center', color=YELLOW, fill='0', lineWidth='0.15', layer='6',
                                          shape=_write(_offset(lane[f'{b}_f_1'], h))))
        # 같은 방향 두 차로 사이 흰 점선 (3 m / 3 m)
        for d in ('f', 'r'):
            div = _offset(lane[f'{b}_{d}_0'], h)
            L = _length(div)
            st, k = 1.5, 0
            while st + 3.0 <= L - 1.5:
                ET.SubElement(root, 'poly', dict(id=f'shopd1_dash_{b}_{d}_{k}', color='255,255,255', fill='0',
                                                  lineWidth='0.15', layer='6', shape=_write(_slice(div, st, st + 3.0))))
                st += 6.0
                k += 1
                n_line += 1
    # 화살표: D1 진입 차로마다 (교차점 7 m 앞)
    by = {}
    for jn, f, k, t, turn in movements(net_path):
        if f in WIDE:
            by.setdefault((f, k), set()).add(turn)
    marks = []
    for (f, k), turns in sorted(by.items()):
        L = _length(lane[f'{f}_{k}'])
        if L < 7.5:
            continue
        for kind in ARROW_KINDS.get(frozenset(turns), []):
            _arrow(marks, f'shop_arrow_{f}_{k}_{kind}', shapes, f, k, kind, min(7.0, L - 3.5))
    for m in marks:
        root.append(ET.fromstring(m))
    # C2N 좌회전 유도선: 실제 회전 경로(출발 차로 → 연결된 도착 차로)를 차로 폭 절반만큼 안쪽(왼쪽)으로
    n_guide = 0
    for c in net.findall('connection'):
        f, t = c.get('from'), c.get('to')
        if f.startswith(':') or not c.get('via', '').startswith(':shop_C2N_'):
            continue
        k, kt = int(c.get('fromLane')), int(c.get('toLane'))
        src, dst = lane[f'{f}_{k}'], lane[f'{t}_{kt}']
        a_dir = (src[-1][0] - src[-2][0], src[-1][1] - src[-2][1])
        b_dir = (dst[1][0] - dst[0][0], dst[1][1] - dst[0][1])
        ang = math.degrees(math.atan2(a_dir[0]*b_dir[1] - a_dir[1]*b_dir[0], a_dir[0]*b_dir[0] + a_dir[1]*b_dir[1]))
        if ang < 35:
            continue
        center = [src[-2]] + _movement_path(net_path, f, t, k) + [dst[1]]
        boundary = _offset(center, h)
        a = math.dist(src[-2], src[-1])
        b = _length(boundary) - math.dist(dst[0], dst[1])
        n_guide += _dashes(root, f'shop_guide_{f}_to_{t}', _slice(boundary, a, b), 1.0, 1.0)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'shop_d1_lane_dashes': n_line, 'shop_d1_arrows': len(marks), 'shop_d1_guide_dashes': n_guide}


def apply(net_path, deco_path):
    info = apply_net(net_path)
    info.update(apply_deco(deco_path, net_path))
    info['shop_d1_scenery_clipped'] = clip_scenery(os.path.dirname(net_path), net_path)
    info['shop_d1_images_shrunk'] = shrink_images(os.path.dirname(net_path), net_path)
    return info
