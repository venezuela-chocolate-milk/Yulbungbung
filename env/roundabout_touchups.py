"""사용자가 짚은 노면 점선 빈 칸 채우기 (맵 생성 마지막 단계).

FILLS: (점선 묶음 접두어, 기준 점선 번호들, 채울 점선 번호) — 같은 줄의 기존 점선 간격(주기)으로 위치를 정하고,
정지선과 0.5m 이상 떨어진 부분, 교차로 영역 밖(clip_junction=True 일 때) 부분만 그린다.
"""
import math
import xml.etree.ElementTree as ET

FILLS = [
    # 동남쪽 끝 교차로 북쪽 진입로: 바깥 차로 경계, 정지선 너머 빈 칸
    ('SW_new_straight_dash_upper_4', range(5), 5, 'SW_new_cross', False),
    # 같은 교차로에서 북쪽으로 나가는 가장 바깥 차로 경계, 교차로 쪽 첫 칸
    ('SW_new_straight_dash_upper_8', range(2, 5), 5, 'SW_new_cross', True),
]


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def apply(deco_path, net_path):
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    tree = ET.parse(deco_path)
    root = tree.getroot()
    net = ET.parse(net_path).getroot()
    polys = {p.get('id'): p for p in root.findall('poly')}
    stops = unary_union([LineString(_read(p.get('shape'))).buffer(0.225 + 0.5) for p in root.findall('poly')
                         if p.get('color') == '255,255,255' and p.get('fill') == '0'
                         and p.get('lineWidth') in ('0.45', '.45')])
    added = 0
    for prefix, ks, k_new, junction, clip in FILLS:
        if f'{prefix}_{k_new}' in polys or f'{prefix}_{k_new}_fill' in polys:
            continue
        ref = [_read(polys[f'{prefix}_{k}'].get('shape')) for k in ks if f'{prefix}_{k}' in polys]
        if len(ref) < 2:
            continue
        a, b = ref[0][0], ref[0][-1]
        L = math.dist(a, b)
        u = ((b[0]-a[0])/L, (b[1]-a[1])/L)
        st = lambda q: (q[0]-a[0])*u[0] + (q[1]-a[1])*u[1]
        starts = [min(st(q) for q in d) for d in ref]
        per = (starts[-1] - starts[0]) / (len(starts) - 1)
        s0 = starts[0] + per*(k_new - list(ks)[0])
        seg = LineString([(a[0]+u[0]*s0, a[1]+u[1]*s0), (a[0]+u[0]*(s0+L), a[1]+u[1]*(s0+L))])
        geom = seg.difference(stops)
        if clip:
            j = next(x for x in net.findall('junction') if x.get('id') == junction)
            geom = geom.difference(Polygon(_read(j.get('shape'))).buffer(0))
        pieces = [g for g in getattr(geom, 'geoms', [geom]) if g.geom_type == 'LineString' and g.length > 0.5]
        if not pieces:
            continue
        g = max(pieces, key=lambda x: x.length)
        tmpl = polys[f'{prefix}_{list(ks)[-1]}']
        el = ET.Element('poly', dict(tmpl.attrib))
        el.set('id', f'{prefix}_{k_new}_fill')
        el.set('shape', _write(list(g.coords)))
        root.insert(list(root).index(tmpl) + 1, el)
        added += 1
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'touchup_dashes_added': added}
