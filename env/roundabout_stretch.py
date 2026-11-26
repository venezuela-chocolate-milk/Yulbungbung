"""도로 한 쌍(왕복 엣지)을 축 위의 한 지점에서 잘라 D 만큼 늘리고, 그 너머 묶음 전체를 평행이동한다.

roundabout_south_link.stretch_black 을 일반화한 것.
  - 잘린 엣지: 절단점 너머의 점만 이동 → 직선 엣지가 그대로 길어진다.
  - 묶음: 장벽 노드를 지나지 않고 seed 노드에서 닿는 모든 노드/엣지/내부 엣지/교차점 → 통째로 이동.
  - 노면 표시: 점마다 '움직이는 도로'와 '고정 도로' 중 가까운 쪽을 따른다.
  - 늘어난 구간의 흰 점선은 주기 그대로 복제해 채운다(D 는 점선 주기의 배수로 맞춘다).
"""
import math
import re
import xml.etree.ElementTree as ET


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _cluster(root, seeds, barrier):
    adj = {}
    for e in root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        adj.setdefault(e.get('from'), set()).add(e.get('to'))
        adj.setdefault(e.get('to'), set()).add(e.get('from'))
    seen, stack = set(), list(seeds)
    while stack:
        n = stack.pop()
        if n in seen or n == barrier:
            continue
        seen.add(n)
        stack += list(adj.get(n, ()))
    return seen


def stretch(net_root, deco_root, origin, w, cut_hint, D, cut_edges, seeds, barrier, forbid=()):
    """origin, w: 축 원점과 이동 방향 단위벡터 (절단점 너머 = station 이 큰 쪽).
    cut_edges: 절단점을 가로지르는 엣지들. seeds/barrier: 이동 묶음 탐색."""
    from env.roundabout_markings import dash_style
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree
    st = lambda p: (p[0]-origin[0])*w[0] + (p[1]-origin[1])*w[1]
    nw = (-w[1], w[0])
    lat = lambda p: (p[0]-origin[0])*nw[0] + (p[1]-origin[1])*nw[1]
    move = lambda p, k=1.0: (p[0]+w[0]*D*k, p[1]+w[1]*D*k)
    cluster = _cluster(net_root, seeds, barrier)
    for f in forbid:
        assert f not in cluster, f'{f} 가 이동 묶음에 들어갔다'
    pat = '|'.join(re.escape(j) for j in cluster)
    internal = re.compile(rf'^:({pat})_\d+(_\d+)?$')

    # 절단 엣지 통로(횡 범위)
    corridor = []
    for e in net_root.findall('edge'):
        if e.get('id') in cut_edges:
            for l in e.findall('lane'):
                corridor += [lat(p) for p in _read(l.get('shape'))]
    lo_l, hi_l = min(corridor) - 2.0, max(corridor) + 2.0

    # 통로 안의 흰 점선 그룹과 주기
    dash, gap, _ = dash_style(20)
    period = dash + gap
    D = round(D/period)*period
    dashes = []
    for p in deco_root.findall('poly'):
        if p.get('color') != '255,255,255' or p.get('fill') != '0':
            continue
        pts = _read(p.get('shape'))
        if len(pts) != 2 or not (dash - 0.5 < math.dist(*pts) < dash + 0.5):
            continue
        if all(lo_l <= lat(q) <= hi_l for q in pts):
            dashes.append((min(st(q) for q in pts), max(st(q) for q in pts), round(lat(pts[0]), 1), p))
    spans = [(a, b) for a, b, _, _ in dashes]
    cut = min((cut_hint + d*0.1 for d in range(-300, 300)
               if all(not (a - 0.3 <= cut_hint + d*0.1 <= b + 0.3) for a, b in spans)),
              key=lambda c: abs(c - cut_hint))

    # 이동 전 도로 형상 (노면 표시 판단용)
    moving, static = [], []
    for e in net_root.findall('edge'):
        eid = e.get('id')
        is_mov = (e.get('function') == 'internal' and internal.match(eid)) or (
            e.get('function') != 'internal' and e.get('from') in cluster and e.get('to') in cluster)
        for l in e.findall('lane'):
            pts = _read(l.get('shape'))
            if len(pts) < 2:
                continue
            if eid in cut_edges:
                near = [p for p in pts if st(p) <= cut]
                far = [p for p in pts if st(p) > cut]
                if len(near) >= 1 and len(far) >= 1:
                    a, b = (near[-1], far[0]) if st(pts[0]) < st(pts[-1]) else (far[-1], near[0])
                    f = (cut - st(a))/(st(b) - st(a))
                    m = (a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f)
                    near_line = sorted(near + [m], key=st)
                    far_line = sorted([m] + far, key=st)
                    if len(near_line) >= 2:
                        static.append(LineString(near_line))
                    if len(far_line) >= 2:
                        moving.append(LineString(far_line))
                continue
            (moving if is_mov else static).append(LineString(pts))
    for j in net_root.findall('junction'):
        pts = _read(j.get('shape', ''))
        if len(pts) >= 3 and not j.get('id').startswith(':'):
            (moving if j.get('id') in cluster else static).append(Polygon(pts).exterior)
    t_mov, t_sta = STRtree(moving), STRtree(static)

    # --- net ---
    def tr(v, fn):
        return _write([fn(p) for p in _read(v)])
    move_all = move
    move_cut = lambda p: move(p) if st(p) > cut else p
    for e in net_root.findall('edge'):
        eid = e.get('id')
        if eid in cut_edges:
            fn = move_cut
        elif e.get('function') == 'internal':
            fn = move_all if internal.match(eid) else None
        elif e.get('from') in cluster and e.get('to') in cluster:
            fn = move_all
        else:
            fn = None
        if fn is None:
            continue
        if e.get('shape'):
            e.set('shape', tr(e.get('shape'), fn))
        for l in e.findall('lane'):
            l.set('shape', tr(l.get('shape'), fn))
            if eid in cut_edges:
                l.set('length', f'{_length(_read(l.get("shape"))):.6f}')
    for j in net_root.findall('junction'):
        jid = j.get('id')
        if jid in cluster or (jid.startswith(':') and internal.match(jid)):
            x, y = move((float(j.get('x')), float(j.get('y'))))
            j.set('x', f'{x:.6f}')
            j.set('y', f'{y:.6f}')
            if j.get('shape'):
                j.set('shape', tr(j.get('shape'), move_all))

    # --- deco ---
    cache = {}
    moved = 0
    for p in deco_root.findall('poly'):
        if p.get('id') == 'ground':
            continue
        pts = _read(p.get('shape'))
        new, changed = [], False
        for q in pts:
            key = (round(q[0], 3), round(q[1], 3))
            if key not in cache:
                P = Point(q)
                dm = moving[t_mov.nearest(P)].distance(P)
                ds = static[t_sta.nearest(P)].distance(P)
                cache[key] = dm < ds
            if cache[key]:
                q = move(q)
                changed = True
            new.append(q)
        if changed:
            p.set('shape', _write(new))
            moved += 1

    # 줄이는 경우(D < 0): 절단점 앞 |D| 구간은 넘어온 도로와 겹치므로 그 자리의 고정 점선을 지운다
    removed = 0
    if D < 0:
        for a, b, l, p in dashes:
            if b <= cut + 0.3 and a > cut + D - 0.3:
                deco_root.remove(p)
                removed += 1
        dashes = [t for t in dashes if not (t[1] <= cut + 0.3 and t[0] > cut + D - 0.3)]

    # 늘어난 구간 점선 채우기
    groups = {}
    for a, b, l, p in dashes:
        if a < cut and D > 0:
            groups.setdefault(l, []).append((a, p))
    filled = 0
    for l, items in groups.items():
        s_last, tmpl = max(items, key=lambda t: t[0])
        if cut - s_last > period + 1.0:      # 절단점 근처에 점선이 없는 줄(정지선 앞 등)은 건너뜀
            continue
        pts = _read(tmpl.get('shape'))
        k = 1
        while s_last + k*period < cut + D - 1e-6:
            el = ET.Element('poly', dict(tmpl.attrib))
            el.set('id', f'{tmpl.get("id")}_x{k}')
            el.set('shape', _write([(q[0]+w[0]*k*period, q[1]+w[1]*k*period) for q in pts]))
            deco_root.insert(list(deco_root).index(tmpl)+1, el)
            filled += 1
            k += 1
    return {'D': D, 'cut': cut, 'cluster_nodes': len(cluster), 'deco_moved': moved, 'dash_filled': filled,
            'dash_removed': removed}
