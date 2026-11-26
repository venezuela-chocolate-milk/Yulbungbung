"""기존 정지선(흰색 0.45m 직선)을 시뮬레이터의 실제 정지 위치(진입 차로 끝)에 맞춰 옮긴다.

SUMO 차량은 노면 정지선이 아니라 진입 차로 끝(교차점 경계)에서 멈춘다.
정지선마다 가로지르는(또는 바로 앞에서 끝나는) 진입 차로를 찾아,
차로 끝에서 TARGET m 뒤에 오도록 양 끝점을 각 차로 방향으로 평행이동한다.
정지선을 새로 만들지는 않는다. 이미 OK_RANGE 안에 있는 정지선은 그대로 둔다.
"""
import math
import xml.etree.ElementTree as ET

TARGET = 1.0
OK_RANGE = (0.5, 2.0)
# 정지선이 없어 사용자 요청으로 새로 그리는 진입 엣지
ADD_STOPS = ['SW_final_forward_out']        # 검은 도로 → T자 교차로 (남행 4차로)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _is_stop(p):
    return (p.get('color') == '255,255,255' and p.get('fill') == '0'
            and p.get('lineWidth') in ('0.45', '.45') and len(p.get('shape', '').split()) == 2)


def _lane_ends(net_root):
    out = []
    for e in net_root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        for l in e.findall('lane'):
            pts = _read(l.get('shape'))
            a, b = pts[-2], pts[-1]
            L = math.dist(a, b)
            if L < 1e-6:
                continue
            out.append((l.get('id'), b, ((b[0]-a[0])/L, (b[1]-a[1])/L), float(l.get('width', 3.2))))
    return out


def _measure(seg, ends):
    """정지선 seg 에 대해 (차로id, 끝점, 방향, d) 목록. d = 정지선에서 차로 끝까지(진행 방향, +면 정지선이 앞)."""
    (a, b) = seg
    s = (b[0]-a[0], b[1]-a[1])
    Ls = math.hypot(*s)
    u = (s[0]/Ls, s[1]/Ls)
    hits = []
    for lid, E, v, w in ends:
        cos = abs(v[0]*u[0] + v[1]*u[1])
        if cos > 0.5:                       # 정지선과 거의 나란한 차로는 제외
            continue
        # 차로 끝에서 거꾸로 진행 방향 직선이 정지선 직선과 만나는 점
        den = v[0]*s[1] - v[1]*s[0]
        if abs(den) < 1e-9:
            continue
        t = ((a[0]-E[0])*s[1] - (a[1]-E[1])*s[0]) / den      # E + t*v 가 정지선 위
        k = ((a[0]-E[0])*v[1] - (a[1]-E[1])*v[0]) / den      # a + k*s
        if not (-(w/2)/Ls <= k <= 1 + (w/2)/Ls):            # 정지선 폭 안(차로 반폭 여유)
            continue
        d = -t                                               # 정지선 → 차로 끝 거리
        if -15.0 <= d <= 70.0:
            hits.append((lid, E, v, d))
    if not hits:
        return []
    best = min(hits, key=lambda h: abs(h[3] - TARGET))[3]
    return [h for h in hits if abs(h[3] - best) <= 3.0]


def fix(deco_path, net_path, skip_prefixes=()):
    net = ET.parse(net_path).getroot()
    ends = _lane_ends(net)
    tree = ET.parse(deco_path)
    report = []
    # 요청된 진입 엣지에 정지선 추가 (이미 있으면 건너뜀)
    from env.roundabout_builder import _read_lane_shapes, _stop_line
    existing = {p.get('id') for p in tree.getroot().findall('poly')}
    shapes = _read_lane_shapes(net_path)
    for eid in ADD_STOPS:
        pid = f'added_stop_{eid}'
        if pid not in existing:
            marks = []
            _stop_line(marks, pid, shapes, eid, dist=TARGET)
            tree.getroot().append(ET.fromstring(marks[0]))
            report.append((pid, None, None, 'added'))
    for p in tree.getroot().findall('poly'):
        if not _is_stop(p) or p.get('id', '').startswith(skip_prefixes):
            continue
        seg = _read(p.get('shape'))
        hits = _measure(seg, ends)
        if not hits:
            report.append((p.get('id'), None, None, 'no-lane'))
            continue
        ds = [h[3] for h in hits]
        if all(OK_RANGE[0] <= d <= OK_RANGE[1] for d in ds):
            report.append((p.get('id'), round(min(ds), 2), round(max(ds), 2), 'ok'))
            continue
        # 양 끝점: 가장 가까운 차로의 방향으로 (d - TARGET) 만큼 이동
        new = []
        for q in seg:
            lid, E, v, d = min(hits, key=lambda h: math.dist(q, h[1]) )
            shift = d - TARGET
            new.append((q[0] + v[0]*shift, q[1] + v[1]*shift))
        p.set('shape', _write(new))
        report.append((p.get('id'), round(min(ds), 2), round(max(ds), 2), f'moved {round(min(ds)-TARGET,2)}~{round(max(ds)-TARGET,2)}m'))
    # 정지선과 겹치는 흰 표시 정리
    #  - 차로 점선: 지운다
    #  - 유도선(guide)·가장자리선(border): 옮긴 정지선이면 정지선 앞(진입 쪽)으로 튀어나온 부분만 자른다
    #  - 화살표(채운 도형): 옮긴 정지선과 겹치면 진입 방향 뒤로 물린다
    from shapely.geometry import LineString, Polygon
    root = tree.getroot()
    moved_ids = {r[0] for r in report if r[3].startswith('moved')}
    stops = []
    for p in root.findall('poly'):
        if _is_stop(p):
            seg = _read(p.get('shape'))
            hits = _measure(seg, ends)
            if hits:
                v = hits[0][2]
                mid = ((seg[0][0]+seg[1][0])/2, (seg[0][1]+seg[1][1])/2)
                n = (-v[1], v[0])
                half = math.dist(*seg)/2 + 3.0
                up = Polygon([(mid[0]-v[0]*0.3+n[0]*half, mid[1]-v[1]*0.3+n[1]*half),
                              (mid[0]-v[0]*0.3-n[0]*half, mid[1]-v[1]*0.3-n[1]*half),
                              (mid[0]-v[0]*60-n[0]*half, mid[1]-v[1]*60-n[1]*half),
                              (mid[0]-v[0]*60+n[0]*half, mid[1]-v[1]*60+n[1]*half)])
                stops.append((p.get('id'), LineString(seg).buffer(0.45/2 + 0.1), mid, v, up))
    removed = clipped = shifted = 0
    for p in list(root.findall('poly')):
        if p.get('color') != '255,255,255' or _is_stop(p):
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2:
            continue
        pid = p.get('id', '')
        if p.get('fill') == '1':
            if len(pts) < 3:
                continue
            g = Polygon(pts).buffer(0)
            for sid, area, mid, v, up in stops:
                if sid in moved_ids and g.intersects(area):
                    front = max((q[0]-mid[0])*v[0] + (q[1]-mid[1])*v[1] for q in pts)
                    k = front + 0.225 + 0.8
                    p.set('shape', _write([(q[0]-v[0]*k, q[1]-v[1]*k) for q in pts]))
                    shifted += 1
                    break
            continue
        g = LineString(pts)
        for sid, area, mid, v, up in stops:
            if not g.buffer(0.075).intersects(area):
                continue
            if 'guide' in pid or 'border' in pid:
                if sid in moved_ids:
                    rest = g.difference(up)
                    pieces = [x for x in getattr(rest, 'geoms', [rest]) if x.geom_type == 'LineString' and x.length > 0.3]
                    if pieces:
                        p.set('shape', _write(list(max(pieces, key=lambda x: x.length).coords)))
                    else:
                        root.remove(p)
                    clipped += 1
            else:
                root.remove(p)
                removed += 1
            break
    report.append(('dashes_removed', removed, None, ''))
    report.append(('guides_clipped', clipped, None, ''))
    report.append(('arrows_shifted', shifted, None, ''))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return report
