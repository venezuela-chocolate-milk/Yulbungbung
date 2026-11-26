"""대각선(검은) 도로 윗부분을 원호로 휘어, 그 아래 T자 묶음 전체를 모양 그대로 회전·이동한다.

2026-11-10: 아래 긴 도로를 수평(동서 방향)으로 펴기 위해 추가.
  - 묶음: 위쪽 삼각형 교차로(SE_triangle_SW)를 지나지 않고 T자 교차로(SW_final_forward)에서 닿는
    모든 노드/엣지/내부 엣지/교차점 → 같은 강체 변환(회전 ANGLE + 이동)을 적용한다.
  - 대각선 도로(묶음과 바깥을 잇는 엣지): 축 위 station s0 ~ s1 구간을 반지름 R 원호로 휜다.
    s0 앞은 그대로, s1 뒤는 묶음과 같은 강체 변환이라 차로 폭과 교차로 모양이 그대로 남는다.
  - 노면 표시: 점마다 가장 가까운 도로(묶음 / 대각선 / 그 밖)를 따른다. 대각선 위 선은
    원호를 따라 휘도록 2m 간격으로 점을 채운 뒤 옮긴다.
roundabout_south_extend 가 검은 도로 차로 축소(roundabout_t_black_lanes) 바로 다음에 호출한다.
"""
import math
import re

TRIANGLE, T_NODE = 'SE_triangle_SW', 'SW_final_forward'


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _densify(pts, step):
    out = [pts[0]]
    for p, q in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(math.dist(p, q)/step)))
        out += [(p[0]+(q[0]-p[0])*i/n, p[1]+(q[1]-p[1])*i/n) for i in range(1, n+1)]
    return out


class Bend:
    """원호 굽힘 사상. a: 축 원점, w: 묶음 쪽 단위벡터, phi: 반시계(+) 회전각(rad)."""

    def __init__(self, a, w, s0, R, phi):
        self.a, self.w, self.s0, self.R, self.phi = a, w, s0, R, phi
        self.sg = 1.0 if phi >= 0 else -1.0
        self.n = (-w[1]*self.sg, w[0]*self.sg)       # 회전 중심 쪽 법선
        self.s1 = s0 + R*abs(phi)
        self.C = (a[0] + w[0]*s0 + self.n[0]*R, a[1] + w[1]*s0 + self.n[1]*R)

    def st(self, p):
        return (p[0]-self.a[0])*self.w[0] + (p[1]-self.a[1])*self.w[1]

    def _rot(self, v, th):
        c, s = math.cos(th), math.sin(th)
        return (c*v[0] - s*v[1], s*v[0] + c*v[1])

    def rigid(self, p):
        v = (p[0] - self.a[0] - self.w[0]*self.s1 - self.n[0]*self.R,
             p[1] - self.a[1] - self.w[1]*self.s1 - self.n[1]*self.R)
        r = self._rot(v, self.phi)
        return (self.C[0] + r[0], self.C[1] + r[1])

    def __call__(self, p):
        s = self.st(p)
        if s <= self.s0:
            return p
        if s >= self.s1:
            return self.rigid(p)
        k = (p[0]-self.a[0])*self.n[0] + (p[1]-self.a[1])*self.n[1]    # 중심 쪽 횡거리
        th = self.sg*(s - self.s0)/self.R
        r = self._rot((-self.n[0]*(self.R - k), -self.n[1]*(self.R - k)), th)
        return (self.C[0] + r[0], self.C[1] + r[1])


def _cluster(root, seed=T_NODE, barrier=TRIANGLE):
    adj = {}
    for e in root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        adj.setdefault(e.get('from'), set()).add(e.get('to'))
        adj.setdefault(e.get('to'), set()).add(e.get('from'))
    seen, stack = set(), [seed]
    while stack:
        n = stack.pop()
        if n in seen or n == barrier:
            continue
        seen.add(n)
        stack += list(adj.get(n, ()))
    return seen


def apply(net_root, deco_root, target_heading_deg=None, radius=300.0, lead=12.0, tail=12.0,
          seed=T_NODE, barrier=TRIANGLE, phi_deg=None, heading_edge='SW_end_T_left_out'):
    """barrier 교차로에서 seed 쪽으로 나가는 도로를 휘어 seed 쪽 묶음 전체를 회전한다.

    phi_deg 를 주면 그 각도(반시계 +)만큼, 아니면 heading_edge 방향이 target_heading_deg 가 되도록 돌린다.
    기본값은 대각선(검은) 도로를 휘어 아래 긴 도로 방향을 맞추는 경우.
    """
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree
    from shapely.ops import unary_union
    edges = {e.get('id'): e for e in net_root.findall('edge')}
    jun = {j.get('id'): j for j in net_root.findall('junction')}
    cluster = _cluster(net_root, seed, barrier)
    assert barrier not in cluster
    cross = [e.get('id') for e in net_root.findall('edge') if e.get('function') != 'internal'
             and len({e.get('from'), e.get('to')} & cluster) == 1]
    pat = '|'.join(re.escape(j) for j in cluster)
    internal = re.compile(rf'^:({pat})_\d+(_\d+)?$')

    # 회전각: 아래 긴 도로 방향 → 목표 방향
    if phi_deg is not None:
        phi = math.radians(phi_deg)
    else:
        lo = _read(edges[heading_edge].find('lane').get('shape'))
        cur = math.atan2(lo[-1][1]-lo[0][1], lo[-1][0]-lo[0][0])
        phi = math.radians(target_heading_deg) - cur
        phi = math.atan2(math.sin(phi), math.cos(phi))

    # 축: 위쪽 삼각형 교차로 중심에서 대각선(남행 엣지) 방향
    a = (float(jun[barrier].get('x')), float(jun[barrier].get('y')))
    down = next(e for e in cross if edges[e].get('from') == barrier)
    sh = _read(edges[down].find('lane').get('shape'))
    d = math.dist(sh[0], sh[-1])
    w = ((sh[-1][0]-sh[0][0])/d, (sh[-1][1]-sh[0][1])/d)
    st = lambda p: (p[0]-a[0])*w[0] + (p[1]-a[1])*w[1]
    # 굽힐 수 있는 구간: 삼각형 교차로 모양 끝 ~ 묶음 쪽 가장 가까운 교차점 모양 시작
    s_top = max(st(p) for p in _read(jun[barrier].get('shape'))) + lead
    s_bot = min(st(p) for jid in cluster if jun.get(jid) is not None and jun[jid].get('shape')
                for p in _read(jun[jid].get('shape'))) - tail
    s0 = s_top
    bend = Bend(a, w, s0, radius, phi)
    assert bend.s1 <= s_bot, f'굽힘 구간이 모자란다: 필요 {bend.s1 - s0:.1f} m, 가능 {s_bot - s0:.1f} m'

    # 이동 전 도로 형상 (노면 표시 분류)
    groups = {'rigid': [], 'bend': [], 'static': []}
    for e in net_root.findall('edge'):
        eid = e.get('id')
        if e.get('function') == 'internal':
            kind = 'rigid' if internal.match(eid) else 'static'
        elif eid in cross:
            kind = 'bend'
        elif e.get('from') in cluster and e.get('to') in cluster:
            kind = 'rigid'
        else:
            kind = 'static'
        for l in e.findall('lane'):
            pts = _read(l.get('shape'))
            if len(pts) >= 2:
                groups[kind].append(LineString(pts))
    for j in net_root.findall('junction'):
        pts = _read(j.get('shape', ''))
        if len(pts) >= 3 and not j.get('id').startswith(':'):
            groups['rigid' if j.get('id') in cluster else 'static'].append(Polygon(pts).exterior)
    trees = {k: STRtree(v) for k, v in groups.items() if v}

    corridor = unary_union([g.buffer(12.0) for g in groups['bend']])

    def kind_of(p):
        P = Point(p)
        return min(trees, key=lambda k: groups[k][trees[k].nearest(P)].distance(P))

    # --- net ---
    for e in net_root.findall('edge'):
        eid = e.get('id')
        if eid in cross:
            fn = lambda pts: [bend(q) for q in _densify(pts, 2.0)]
        elif (e.get('function') == 'internal' and internal.match(eid)) or (
                e.get('function') != 'internal' and e.get('from') in cluster and e.get('to') in cluster):
            fn = lambda pts: [bend.rigid(q) for q in pts]
        else:
            continue
        if e.get('shape'):
            e.set('shape', _write(fn(_read(e.get('shape')))))
        for l in e.findall('lane'):
            pts = fn(_read(l.get('shape')))
            l.set('shape', _write(pts))
            if eid in cross:
                l.set('length', f'{_length(pts):.6f}')
    for j in net_root.findall('junction'):
        jid = j.get('id')
        if jid in cluster or (jid.startswith(':') and internal.match(jid)):
            x, y = bend.rigid((float(j.get('x')), float(j.get('y'))))
            j.set('x', f'{x:.6f}')
            j.set('y', f'{y:.6f}')
            if j.get('shape'):
                j.set('shape', _write([bend.rigid(q) for q in _read(j.get('shape'))]))

    # --- deco ---
    cache, moved, bent = {}, 0, 0
    for p in deco_root.findall('poly'):
        if p.get('id') == 'ground':
            continue
        pts = _read(p.get('shape'))
        # 흰 점선 한 조각(두 점): 가운데와 방향만 따라가고 길이는 그대로 (원호 안팎에서 길이가 변하지 않게)
        if len(pts) == 2 and p.get('fill') == '0' and p.get('color') == '255,255,255' and \
                bend.s0 - 5 < st(pts[0]) < bend.s1 + 5:
            m = ((pts[0][0]+pts[1][0])/2, (pts[0][1]+pts[1][1])/2)
            key = (round(m[0], 3), round(m[1], 3))
            k = cache.setdefault(key, kind_of(m))
            if k == 'bend':
                L = math.dist(*pts)
                u = ((pts[1][0]-pts[0][0])/L, (pts[1][1]-pts[0][1])/L)
                c = bend(m)
                f = bend((m[0]+u[0]*0.5, m[1]+u[1]*0.5))
                b = bend((m[0]-u[0]*0.5, m[1]-u[1]*0.5))
                d = math.dist(f, b)
                v = ((f[0]-b[0])/d, (f[1]-b[1])/d)
                p.set('shape', _write([(c[0]-v[0]*L/2, c[1]-v[1]*L/2), (c[0]+v[0]*L/2, c[1]+v[1]*L/2)]))
                moved += 1
                continue
        kinds = []
        for q in pts:
            key = (round(q[0], 3), round(q[1], 3))
            if key not in cache:
                cache[key] = kind_of(q)
            kinds.append(cache[key])
        if all(k == 'static' for k in kinds):
            continue
        # 굽힘 구간에 걸치는 선/면: 끝점이 고정·회전 쪽이어도 중간이 대각선 위를 지나므로 점을 채워 다시 분류한다
        if len(pts) >= 2 and max(st(q) for q in pts) > bend.s0 - 5 and min(st(q) for q in pts) < bend.s1 + 5 and \
                LineString(pts).distance(corridor) < 1.0:
            pts = _densify(pts, 2.0)
            kinds = [kind_of(q) for q in pts]
            bent += 1
        new = [q if k == 'static' else (bend.rigid(q) if k == 'rigid' else bend(q)) for q, k in zip(pts, kinds)]
        p.set('shape', _write(new))
        moved += 1
    return {'bend_deg': round(math.degrees(phi), 3), 'bend_radius': radius, 'bend_s0': round(s0, 1),
            'bend_s1': round(bend.s1, 1), 'bend_room': round(s_bot, 1), 'bend_cross_edges': cross,
            'bend_cluster_nodes': len(cluster), 'bend_deco_moved': moved, 'bend_deco_densified': bent}
