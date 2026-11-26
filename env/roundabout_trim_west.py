"""서쪽 가지(W_in1_far / W_out0_far, 회전교차로에서 남서쪽으로 길게 뻗은 도로)를
회전교차로 중심에서 TRIM_R m 까지만 남기고 자른다.

  - 두 엣지의 먼 쪽 끝(막다른 길)을 TRIM_R 지점으로 당기고, 끝 교차점(W_end0/W_end1)을 옮긴다.
  - 그 너머의 노면 표시(점선·중앙선·하부 포장 등)는 잘라낸다.
"""
import math
import xml.etree.ElementTree as ET

TRIM_R = 200.0
CENTER = (0.0, 0.0)
EDGES = {'W_out0_far': ('W_end0', 'end'), 'W_in1_far': ('W_end1', 'start')}


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _r(p):
    return math.dist(p, CENTER)


def _cut(pts, keep_start):
    """polyline 을 중심 거리 TRIM_R 에서 자른다. keep_start=True 면 앞쪽(가까운 쪽)을 남긴다."""
    seq = pts if keep_start else pts[::-1]
    out = [seq[0]]
    for a, b in zip(seq, seq[1:]):
        if _r(b) <= TRIM_R:
            out.append(b)
            continue
        # a 는 안쪽, b 는 바깥쪽: 선분 위 거리 TRIM_R 지점
        lo, hi = 0.0, 1.0
        for _ in range(50):
            m = (lo + hi) / 2
            q = (a[0] + (b[0]-a[0])*m, a[1] + (b[1]-a[1])*m)
            lo, hi = (m, hi) if _r(q) < TRIM_R else (lo, m)
        out.append((a[0] + (b[0]-a[0])*lo, a[1] + (b[1]-a[1])*lo))
        break
    return out if keep_start else out[::-1]


def apply(net_path, deco_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    junctions = {j.get('id'): j for j in root.findall('junction')}
    if all(_r(_read(edges[e].findall('lane')[0].get('shape'))[-1 if s == 'end' else 0]) <= TRIM_R + 1
           for e, (_, s) in EDGES.items()):
        return {'west_trim': 'already'}
    cut_pts = []
    for eid, (jid, side) in EDGES.items():
        e = edges[eid]
        keep_start = side == 'end'          # 먼 끝이 edge 끝이면 앞쪽을 남긴다
        if e.get('shape'):
            e.set('shape', _write(_cut(_read(e.get('shape')), keep_start)))
        ends = []
        for l in e.findall('lane'):
            pts = _cut(_read(l.get('shape')), keep_start)
            l.set('shape', _write(pts))
            l.set('length', f'{_length(pts):.6f}')
            ends.append((pts[-1] if keep_start else pts[0], pts))
        # 막다른 길 교차점: 차로 끝들을 가로지르는 선
        j = junctions[jid]
        ps = [p for p, _ in ends]
        a, b = ends[0][1][-2:] if keep_start else ends[0][1][:2][::-1]
        d = math.dist(a, b)
        u = ((b[0]-a[0])/d, (b[1]-a[1])/d)
        n = (-u[1], u[0])
        lat = sorted(ps, key=lambda p: p[0]*n[0] + p[1]*n[1])
        w = 1.6
        shape = [(lat[0][0]-n[0]*w, lat[0][1]-n[1]*w), (lat[-1][0]+n[0]*w, lat[-1][1]+n[1]*w)]
        cx, cy = sum(p[0] for p in ps)/len(ps), sum(p[1] for p in ps)/len(ps)
        j.set('x', f'{cx:.6f}')
        j.set('y', f'{cy:.6f}')
        j.set('shape', _write(shape))
        cut_pts += ps
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)

    # 노면 표시: 자른 지점 너머(중심에서 TRIM_R 바깥, 서쪽 가지 통로)를 잘라낸다
    from shapely.geometry import LineString, Polygon, Point
    cx = sum(p[0] for p in cut_pts)/len(cut_pts)
    cy = sum(p[1] for p in cut_pts)/len(cut_pts)
    ux, uy = cx/_r((cx, cy)), cy/_r((cx, cy))       # 바깥 방향
    nx, ny = -uy, ux
    W, FAR = 40.0, 2000.0
    beyond = Polygon([(cx + nx*W, cy + ny*W), (cx - nx*W, cy - ny*W),
                      (cx - nx*W + ux*FAR, cy - ny*W + uy*FAR), (cx + nx*W + ux*FAR, cy + ny*W + uy*FAR)])
    dtree = ET.parse(deco_path)
    droot = dtree.getroot()
    removed = clipped = 0
    for p in list(droot.findall('poly')):
        if p.get('id') == 'ground':
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2:
            continue
        filled = p.get('fill') == '1' and len(pts) >= 3
        g = Polygon(pts).buffer(0) if filled else LineString(pts)
        if not g.intersects(beyond):
            continue
        rest = g.difference(beyond)
        pieces = [x for x in getattr(rest, 'geoms', [rest])
                  if (x.geom_type == 'Polygon' and x.area > 0.05) or (x.geom_type == 'LineString' and x.length > 0.3)]
        if not pieces:
            droot.remove(p)
            removed += 1
            continue
        best = max(pieces, key=lambda x: x.area if x.geom_type == 'Polygon' else x.length)
        p.set('shape', _write(list(best.exterior.coords) if best.geom_type == 'Polygon' else list(best.coords)))
        clipped += 1
    ET.indent(dtree, space='    ')
    dtree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'west_trim': 'done', 'west_deco_removed': removed, 'west_deco_clipped': clipped}
