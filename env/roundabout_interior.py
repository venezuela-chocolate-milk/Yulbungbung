"""큰 루프(검단로·완정로·완정로65번길·원당대로·봉수대로) 안쪽 도로를 만든다. (2026-11-11, 네이버 지도 캡처 기준)

  가장 중요한 것: 새 길이 기존 큰길·골목과 자연스럽게 이어지고 차가 문제없이 지나가는 것.
  건물·산은 env.roundabout_interior_scenery 가 뭉텅이로 그린다.

  도로 (모두 왕복 2차로, 차로당 3.2m, 30 km/h):
    - 완정로165번길 → 원당대로507번길 (등뼈): 상가 골목 모퉁이(shop_N2)에서 남쪽으로 내려가 원당대로까지.
    - 완정로133번길: 등뼈에서 동쪽으로 갔다가 북동쪽으로 꺾여 완정로(X_se_out_s3)로.
    - 봉수대로1394번길 쪽 서쪽 길: 봉수대로(S_in1_ext)에서 동쪽으로 공장 지대를 지나 등뼈 네거리(int_J507)까지.
    - 원당대로507번안길 쪽 동쪽 길: 등뼈 네거리에서 동쪽으로 약수동·롯데마트 북쪽을 지나 완정로65번길까지.
    - 공장 지대 남북 길: 서쪽 길(int_L2)에서 원당대로로.
  큰길과 만나는 곳(T자)은 우회전 진입·진출만 둔다 (큰길 중앙분리대, 좌회전으로 가로지르지 않는다).
  큰길 모양은 그대로 두고 교차점만 끼운다. 큰길 엣지를 나눌 때 첫 조각은 원래 이름을 그대로 쓴다
  (교통 흐름 파일·다른 단계가 쓰는 이름이 바뀌지 않게).

roundabout_south_extend 가 북서쪽 상가 다음 단계로 호출한다.  단독 실행: python -m env.roundabout_interior
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

LANE_W = 3.2
SPEED = 8.33                        # 30 km/h
STUB = 40.0
YELLOW, WHITE = '235,190,40', '255,255,255'
PREFIX = 'int_'

# 큰길 위 T자: (교차점, 큰길 엣지, 대략 위치)
TEES = [
    ('int_T_W', 'S_in1_ext', (125.0, -420.0)),               # 봉수대로 (북행, 동쪽 차로) — 봉수대로1394번길
    ('int_T_W2', 'S_in1_ext', (160.0, -532.0)),              # 봉수대로 — 공구상가 앞 골목 (2026-11-12)
    ('int_T_S1', 'S_link_west', (470.0, -765.0)),            # 원당대로 (서행, 북쪽 차로) — 공장 지대 길
    ('int_T_S2', 'S_link_west', (645.0, -700.0)),            # 원당대로 — 507번길 (등뼈)
    ('int_T_65', 'SW_final_forward_out_up', (1295.0, -330.0)),   # 완정로65번길 (남행, 서쪽 차로)
    ('int_T_N', 'X_se_out_s3', (888.0, 230.0)),              # 완정로 (남동행, 남쪽 차로) — 133번길
]
# 다시 만드는 기존 교차점과, 그 교차점에 붙은 골목을 나눌 자리 (교차점에서 m)
#   골목 이름 'x' 는 x_f / x_r 두 엣지, 튜플이면 그 엣지들 그대로.
#   2026-11-11: 막다른 완정로117번길 끝(T_stop_side_end)을 이어 등뼈(int_P117)까지 (왕복 1차로 그대로 연장)
REBUILD = {'shop_N2': [('shop_v1', 22.0, 'int_V1M'), ('shop_d2a', 25.0, 'int_D2M')],
           'T_stop_side_end': [(('T_stop_side_out_b', 'T_stop_side_in_a'), 15.0, 'int_117M')]}
# 튜플로 적은 엣지는 기존 교차점 쪽 긴 조각이 원래 이름을 그대로 쓴다 (교통 흐름 파일이 쓰는 이름 유지)
# 안쪽 교차점 (대략 위치)
NODES = {
    'int_P117': (592.0, 228.0),        # 등뼈 위 T자: 117번길 연장이 동쪽에서 들어온다
    'int_P133': (596.0, 190.0),
    'int_J507': (655.0, -322.0),
    'int_L2': (430.0, -372.0),
    # 2026-11-12 봉수대로1394번길 안쪽 골목 (네이버 위성 지도): 1394번길 위 두 갈림점, 북쪽 공장 골목 끝,
    # 공구상가 골목 네거리, 공장 지대 남북 길(int_s1) 위 T자
    'int_PB': (215.0, -402.0),
    'int_PA': (330.0, -388.0),
    # 'int_NA' (359, -281): 북쪽 공장 골목 끝 — 2026-11-13 사용자 요청으로 골목(int_n1394)을 지우고 공장 건물로 채움
    'int_JB': (240.0, -515.0),
    'int_PS1': (437.0, -525.0),
}
# 길: (이름, 시작, 끝, 우선순위, 중간 조정점들). 시작·끝이 큰길 T자면 마지막 조정점을 T자에 수직으로 둔다.
STREETS = [
    ('int_spine_a', 'shop_N2', 'int_P117', 1, [(594.0, 268.0)]),
    ('int_spine_a2', 'int_P117', 'int_P133', 1, []),
    # 완정로117번길 연장: 끝에서 같은 방향으로 곧게 나와 원호로 동쪽으로 꺾여 등뼈에 거의 수직으로 (T자, 등뼈 우선)
    ('int_117', 'T_stop_side_end', 'int_P117', 0, ['corner117']),
    ('int_spine_b', 'int_P133', 'int_J507', 1, [(603.0, 90.0), (616.0, -20.0), (632.0, -130.0), (646.0, -230.0)]),
    ('int_spine_c', 'int_J507', 'int_T_S2', 1, [(654.0, -440.0), (645.0, -570.0)]),
    ('int_133', 'int_P133', 'int_T_N', 0, [(680.0, 172.0), (780.0, 168.0)]),
    # 봉수대로1394번길: 봉수대로 → 남쪽 골목 갈림(int_PB) → 북쪽 골목 갈림(int_PA) → int_L2 (모양은 예전 한 길 그대로)
    ('int_west_a', 'int_T_W', 'int_PB', 0, []),
    ('int_west_a2', 'int_PB', 'int_PA', 0, [(250.0, -397.0)]),
    ('int_west_a3', 'int_PA', 'int_L2', 0, [(345.0, -384.0)]),
    # 1394번길 안쪽 골목 (2026-11-12): 북쪽 공장 골목(막다른 길), 남쪽 공구상가 골목 → 네거리 int_JB →
    # 서쪽 봉수대로(우회전 진입·진출) / 동쪽 공장 지대 남북 길
    # ('int_n1394', 'int_PA', 'int_NA', 0, []),  # 2026-11-13 삭제: 막다른 북쪽 공장 골목 → env.roundabout_interior_scenery fac_1394n 공장이 채운다
    ('int_s1394', 'int_PB', 'int_JB', 0, [(232.0, -460.0)]),
    ('int_g1394', 'int_T_W2', 'int_JB', 0, []),
    ('int_e1394', 'int_JB', 'int_PS1', 0, [(340.0, -528.0)]),
    ('int_west_b', 'int_L2', 'int_J507', 0, [(530.0, -350.0), (600.0, -332.0)]),
    ('int_east', 'int_J507', 'int_T_65', 0, [(780.0, -350.0), (950.0, -425.0), (1120.0, -432.0), (1215.0, -345.0)]),
    ('int_s1', 'int_L2', 'int_PS1', 0, [(436.0, -480.0)]),
    ('int_s1b', 'int_PS1', 'int_T_S1', 0, [(444.0, -610.0)]),
]
APPROACH = 36.0                     # T자 앞 곧은 구간 (m)
DOUBLE_CENTER = {'int_117'}         # 이어지는 길(117번길)과 같은 겹 중앙선으로 그릴 길
# 두 방향 차로 사이 가운데 띠 폭 (m). 117번길은 차로 중심 간격 4.2m (3.2m 차로 + 1.0m 회색 띠 + 겹 노란선)
MEDIAN = {'int_117': 1.0}
# 2026-11-12 (블록 S 공영주차장, env.roundabout_parking_s): 다른 지역이 with_cfg 로 쓰는 추가 설정 (안쪽 도로는 비워 둔다)
#   MIDS: 왕복 골목 한가운데에 새 교차점 (교차점, (엣지들), 대략 위치) — 양 끝 교차점은 다시 만들지 않는다
#   ONEWAY: _f 한 방향만 만드는 길,  EXTRA_CON: 직접 적는 연결 (from, to, fromLane, toLane)
MIDS = []
ONEWAY = set()
EXTRA_CON = []
# 2026-11-13 (롯데마트 주차장, env.roundabout_lotte_parking): 차로 수가 바뀌는 합류부용 추가 설정 (안쪽 도로는 비워 둔다)
#   PIECE_OVERRIDE: {나눈 조각: dict(lanes=, shape=, spread=)} — 그 조각만 차로 수·모양을 바꾼다
#   NODE_POS: {교차점: 좌표} (REBUILD 자르는 점 위치 바꾸기),  NODE_SHAPES: {교차점: 바닥 다각형},  NODE_TYPES: {교차점: 종류}
PIECE_OVERRIDE = {}
# 2026-11-14 (마전동 골목 편도 2차로, env.roundabout_majeon): {길: 방향별 차로 수} (없으면 1)
LANES = {}
NODE_POS = {}
NODE_SHAPES = {}
NODE_TYPES = {}


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _cum(pts):
    c = [0.0]
    for a, b in zip(pts, pts[1:]):
        c.append(c[-1] + math.dist(a, b))
    return c


def _project(pts, q):
    best, bs = None, 0.0
    c = _cum(pts)
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        L2 = dx*dx + dy*dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0]-a[0])*dx + (q[1]-a[1])*dy) / L2))
        d = math.dist((a[0]+dx*t, a[1]+dy*t), q)
        if best is None or d < best:
            best, bs = d, c[i] + t*math.sqrt(L2)
    return bs


def _at(pts, s):
    c = _cum(pts)
    s = max(0.0, min(c[-1], s))
    for i in range(len(pts) - 1):
        if c[i+1] >= s:
            seg = c[i+1] - c[i]
            t = 0.0 if seg == 0 else (s - c[i]) / seg
            return (pts[i][0] + (pts[i+1][0]-pts[i][0])*t, pts[i][1] + (pts[i+1][1]-pts[i][1])*t)
    return pts[-1]


def _dir_at(pts, s):
    a, b = _at(pts, max(0.0, s - 1.0)), _at(pts, min(_length(pts), s + 1.0))
    L = math.dist(a, b) or 1.0
    return ((b[0]-a[0])/L, (b[1]-a[1])/L)


def _slice(pts, s0, s1):
    c = _cum(pts)
    out = [_at(pts, s0)]
    for p, cp in zip(pts, c):
        if s0 < cp < s1:
            out.append(p)
    out.append(_at(pts, s1))
    # 부동소수점 오차로 생기는 길이 0 조각(같은 점 두 번)을 뺀다 — 차로 오프셋이 접히는 원인
    ded = [out[0]]
    for q in out[1:]:
        if math.dist(q, ded[-1]) > 1e-3:
            ded.append(q)
    if len(ded) == 1:
        ded.append(out[-1])
    return ded


def _smooth(ctrl, step=4.0):
    """Catmull-Rom (centripetal) 곡선으로 조정점을 매끈하게 잇는다."""
    if len(ctrl) < 3:
        return ctrl
    P = [(2*ctrl[0][0] - ctrl[1][0], 2*ctrl[0][1] - ctrl[1][1])] + list(ctrl) + \
        [(2*ctrl[-1][0] - ctrl[-2][0], 2*ctrl[-1][1] - ctrl[-2][1])]
    out = [ctrl[0]]
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i-1], P[i], P[i+1], P[i+2]
        t0 = 0.0
        t1 = t0 + math.dist(p0, p1) ** 0.5 or 1e-6
        t2 = t1 + math.dist(p1, p2) ** 0.5 or 1e-6
        t3 = t2 + math.dist(p2, p3) ** 0.5 or 1e-6
        n = max(2, int(math.dist(p1, p2) / step))
        for k in range(1, n + 1):
            t = t1 + (t2 - t1) * k / n

            def L(a, b, ta, tb):
                if tb == ta:
                    return a
                return ((tb - t)/(tb - ta)*a[0] + (t - ta)/(tb - ta)*b[0],
                        (tb - t)/(tb - ta)*a[1] + (t - ta)/(tb - ta)*b[1])
            A1, A2, A3 = L(p0, p1, t0, t1), L(p1, p2, t1, t2), L(p2, p3, t2, t3)
            B1, B2 = L(A1, A2, t0, t2), L(A2, A3, t1, t3)
            out.append(L(B1, B2, t1, t2))
    return out


def _fillet(ctrl, radius=45.0, keep=None, step=3.0):
    """꺾은선의 꼭짓점마다 원호를 넣어 매끈하게 (실제 도로처럼 직선 + 원호).
    keep: 꼭짓점 번호 → 그 꼭짓점 원호가 앞뒤 구간을 쓸 수 있는 최대 길이 (T자 앞 곧은 구간 지키기)."""
    keep = keep or {}
    pts = [ctrl[0]]
    n = len(ctrl)
    avail = [math.dist(ctrl[i], ctrl[i+1]) for i in range(n - 1)]
    used_end = [0.0]*(n - 1)        # 각 구간 끝에서 원호가 쓴 길이
    used_start = [0.0]*(n - 1)
    for i in range(1, n - 1):
        a, b, c = ctrl[i-1], ctrl[i], ctrl[i+1]
        d1 = ((b[0]-a[0])/avail[i-1], (b[1]-a[1])/avail[i-1])
        d2 = ((c[0]-b[0])/avail[i], (c[1]-b[1])/avail[i])
        cross = d1[0]*d2[1] - d1[1]*d2[0]
        dot = max(-1.0, min(1.0, d1[0]*d2[0] + d1[1]*d2[1]))
        th = math.atan2(abs(cross), dot)
        if th < 1e-3:
            pts.append(b)
            continue
        T = radius*math.tan(th/2)
        lim = min(avail[i-1] - used_start[i-1], avail[i]) * 0.5
        lim = min(lim, keep.get(i, lim))
        if T > lim:
            T = lim
        r = T / math.tan(th/2)
        used_end[i-1] = T
        used_start[i] = T
        s_pt = (b[0] - d1[0]*T, b[1] - d1[1]*T)
        sgn = 1.0 if cross > 0 else -1.0
        nrm = (-d1[1]*sgn, d1[0]*sgn)
        ctr = (s_pt[0] + nrm[0]*r, s_pt[1] + nrm[1]*r)
        a0 = math.atan2(s_pt[1]-ctr[1], s_pt[0]-ctr[0])
        m = max(2, int(r*th/step))
        for k in range(m + 1):
            ang = a0 + sgn*th*k/m
            pts.append((ctr[0] + r*math.cos(ang), ctr[1] + r*math.sin(ang)))
    pts.append(ctrl[-1])
    out = [pts[0]]
    for q in pts[1:]:
        if math.dist(q, out[-1]) > 0.05:
            out.append(q)
    return out


def _edge_line(e, nodes_xy):
    s = e.get('shape')
    if s:
        return _read(s)
    return [nodes_xy[e.get('from')], nodes_xy[e.get('to')]]


def plan(root):
    """새 교차점 위치, 큰길·골목 나누기, 길 모양."""
    edges = {e.get('id'): e for e in root.findall('edge') if e.get('function') != 'internal'}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in root.findall('junction')
         if j.get('type') != 'internal'}
    nodes = dict(NODES)
    rebuilt = {k: J[k] for k in REBUILD}
    splits = {}                       # eid → [(node, s)]
    tee_info = {}                     # node → (eid, 방향 단위벡터, 안쪽 단위벡터)
    for node, eid, near in TEES:
        line = _edge_line(edges[eid], J)
        s = _project(line, near)
        p = _at(line, s)
        d = _dir_at(line, s)
        inward = (d[1], -d[0])        # 진행 방향 오른쪽 = 루프 안쪽 (우측 통행 바깥 차로 쪽)
        nodes[node] = p
        splits.setdefault(eid, []).append((node, s))
        tee_info[node] = (eid, d, inward)
    for node, eids, near in MIDS:
        ps_ = []
        for eid in eids:
            line = _edge_line(edges[eid], J)
            s = _project(line, near)
            ps_.append(_at(line, s))
            splits.setdefault(eid, []).append((node, s))
        nodes[node] = (sum(p[0] for p in ps_)/len(ps_), sum(p[1] for p in ps_)/len(ps_))
    keep_name_last = set()            # 원래 이름을 마지막(기존 교차점 쪽) 조각에 남길 엣지
    for rn, items in REBUILD.items():
        for base, dist, cut in items:
            eids = base if isinstance(base, tuple) else (base + '_f', base + '_r')
            for eid in eids:
                line = _edge_line(edges[eid], J)
                s = dist if edges[eid].get('from') == rn else _length(line) - dist
                nodes[cut] = _at(line, s)
                splits.setdefault(eid, []).append((cut, s))
                if isinstance(base, tuple) and edges[eid].get('from') == rn:
                    keep_name_last.add(eid)
    nodes.update(NODE_POS)
    for eid in splits:
        splits[eid].sort(key=lambda x: x[1])
    allxy = dict(J)
    allxy.update(nodes)
    named = {}
    if any('corner117' in s[4] for s in STREETS):
        # 117번길 연장 꺾는 점: 117번길 끝 방향 그대로 나간 직선과, int_P117 에서 동쪽으로 나간 직선의 교점
        so = _edge_line(edges['T_stop_side_out_b'], J)
        d117 = _dir_at(so, _length(so) - 1.0)
        e0, p117 = allxy['T_stop_side_end'], allxy['int_P117']
        k = (e0[1] - p117[1]) / -d117[1]
        named['corner117'] = (e0[0] + d117[0]*k, e0[1] + d117[1]*k)
    shapes = {}
    for name, a, b, prio, mids in STREETS:
        ctrl = [allxy[a]] + [named.get(m, m) for m in mids] + [allxy[b]]
        # T자 쪽 끝: 큰길에 수직으로 APPROACH m 곧게 들어간다 (그 꼭짓점 원호는 곧은 구간의 절반까지만)
        keep = {}
        for end in (a, b):
            if end in tee_info:
                _, d, inward = tee_info[end]
                p = allxy[end]
                q = (p[0] + inward[0]*APPROACH, p[1] + inward[1]*APPROACH)
                if end == a:
                    ctrl = [p, q] + ctrl[1:]
                else:
                    ctrl = ctrl[:-1] + [q, p]
        for end in (a, b):
            if end in tee_info:
                keep[1 if end == a else len(ctrl) - 2] = APPROACH*0.5
        shapes[name] = _fillet(ctrl, keep=keep)
    pieces = {}
    for eid, cuts in splits.items():
        if eid in keep_name_last:
            pieces[eid] = [f'{eid}_{PREFIX}{k + 1}' for k in range(len(cuts))] + [eid]
        else:
            pieces[eid] = [eid] + [f'{eid}_{PREFIX}{k + 1}' for k in range(len(cuts))]
    return nodes, rebuilt, splits, pieces, shapes, tee_info


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    if any(eid.startswith(PREFIX) for eid in edges):
        raise RuntimeError('안쪽 도로가 이미 적용됨')
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in root.findall('junction')
         if j.get('type') != 'internal'}
    nodes, rebuilt, splits, pieces, shapes, tee_info = plan(root)
    sub_nodes = set(nodes) | set(rebuilt)

    with tempfile.TemporaryDirectory() as work:
        nod = [(k, p, NODE_TYPES.get(k, 'priority')) for k, p in nodes.items()] + \
              [(k, p, NODE_TYPES.get(k, 'priority')) for k, p in rebuilt.items()]
        edg, con = [], []
        for eid, cuts in splits.items():
            e = edges[eid]
            line = _edge_line(e, J)
            n = len(e.findall('lane'))
            speed = e.find('lane').get('speed')
            fr, to = e.get('from'), e.get('to')
            spread = e.get('spreadType', 'right')
            seq, bounds = [], []
            if fr in rebuilt:
                seq.append(fr)
                bounds.append(0.0)
            else:
                u = f'U_{eid}'
                s0 = max(0.0, cuts[0][1] - STUB)
                nod.append((u, _at(line, s0), 'dead_end'))
                seq.append(u)
                bounds.append(s0)
            for j, s in cuts:
                seq.append(j)
                bounds.append(s)
            if to in rebuilt:
                seq.append(to)
                bounds.append(_length(line))
            else:
                d = f'D_{eid}'
                s1 = min(_length(line), cuts[-1][1] + STUB)
                nod.append((d, _at(line, s1), 'dead_end'))
                seq.append(d)
                bounds.append(s1)
            ps = pieces[eid]
            nl = {}
            for i, pid in enumerate(ps):
                ov = PIECE_OVERRIDE.get(pid, {})
                nl[pid] = ov.get('lanes', n)
                edg.append((pid, seq[i], seq[i+1], nl[pid], speed, e.get('priority', '1'),
                            ov.get('shape') or _slice(line, bounds[i], bounds[i+1]), ov.get('spread', spread)))
            for a, b in zip(ps, ps[1:]):
                con += [(a, b, k, k) for k in range(min(nl[a], nl[b]))]   # 차로 수가 바뀌면 나머지는 EXTRA_CON
        from env.roundabout_geometry import _offset
        for name, a, b, prio, _ in STREETS:
            pts = shapes[name]
            m = MEDIAN.get(name, 0.0) / 2      # 가운데 띠 절반만큼 각 방향 차로를 바깥(오른쪽)으로
            pf = _offset(pts, -m) if m else pts
            pr = _offset(pts[::-1], -m) if m else pts[::-1]
            nlan = LANES.get(name, 1)
            edg.append((f'{name}_f', a, b, nlan, SPEED, prio, pf, 'center' if name in ONEWAY else 'right'))
            if name not in ONEWAY:
                edg.append((f'{name}_r', b, a, nlan, SPEED, prio, pr, 'right'))
        con += list(EXTRA_CON)
        # 큰길 T자: 우회전 진입(큰길 바깥 차로 → 길), 우회전 진출(길 → 큰길 바깥 차로)
        # 2026-11-14: 골목에서 나가는 우회전은 큰길 직진에 양보 (prohibition). 골목 우선순위가 큰길과 같으면
        #   netconvert 가 골목을 우선으로 잡아 큰길 직진이 양보하던 곳이 있었다 (예: 완정로 × 3번길)
        prohib = []
        for node, (eid, _, _) in tee_info.items():
            ps = pieces[eid]
            k = [j for j, _ in splits[eid]].index(node)
            before, after = ps[k], ps[k + 1]
            n_after = PIECE_OVERRIDE.get(after, {}).get('lanes', len(edges[eid].findall('lane')))
            for name, a, b, _, _ in STREETS:
                one = name in ONEWAY            # 한 방향 길은 있는 쪽만 (진입 전용 / 진출 전용)
                # 2026-11-14: 편도 2차로 골목은 이중 우회전 진출 (골목 k차로 → 큰길 k차로, 차로끼리 겹치지 않게)
                out = range(min(LANES.get(name, 1), n_after))
                if a == node:
                    con += [(before, f'{name}_f', 0, 0)] + ([] if one else [(f'{name}_r', after, k, k) for k in out])
                    if not one:
                        prohib.append((before, after, f'{name}_r', after))
                elif b == node:
                    con += ([] if one else [(before, f'{name}_r', 0, 0)]) + [(f'{name}_f', after, k, k) for k in out]
                    prohib.append((before, after, f'{name}_f', after))
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            for nid, (x, y), t in nod:
                shp = f' shape="{_write(NODE_SHAPES[nid])}"' if nid in NODE_SHAPES else ''
                fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"{shp}/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            for eid, fr, to, n, speed, prio, pts, spread in edg:
                fh.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{speed}" priority="{prio}" '
                         f'width="{LANE_W}" spreadType="{spread}" shape="{_write(pts)}"/>\n')
            fh.write('</edges>\n')
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            for a, b, i, k in con:
                fh.write(f'  <connection from="{a}" to="{b}" fromLane="{i}" toLane="{k}"/>\n')
            for a, b, c, d in prohib:
                fh.write(f'  <prohibition prohibitor="{a}->{b}" prohibited="{c}->{d}"/>\n')
            fh.write('</connections>\n')
        from sumolib import checkBinary
        out = os.path.join(work, 'p.net.xml')
        r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                            '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                            '--junctions.corner-detail', '24', '--junctions.internal-link-detail', '12',
                            '--precision', '6', '-o', out],
                           cwd=work, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        sub = ET.parse(out).getroot()
    # 내부 경로(교차점 안 회전 경로)의 0.3m 미만 조각 점을 뺀다: 대기 지점에서 나뉜 좌회전 경로 끝에 생기는
    # 아주 짧은 조각 때문에 차로 폭(1.6m) 오프셋이 접히던 문제 (모양은 사실상 그대로)
    for e in sub.findall('edge'):
        if e.get('function') != 'internal':
            continue
        for ln in e.findall('lane'):
            p = _read(ln.get('shape'))
            if len(p) < 3:
                continue
            q = [p[0]]
            for pt in p[1:-1]:
                if math.dist(pt, q[-1]) >= 0.3:
                    q.append(pt)
            if math.dist(p[-1], q[-1]) < 0.3 and len(q) > 1:
                q.pop()
            q.append(p[-1])
            ln.set('shape', _write(q))
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}
    sub_lane = {(e.get('id'), int(l.get('index'))): _read(l.get('shape'))
                for e in sub.findall('edge') for l in e.findall('lane')}

    # 0) 다시 만드는 교차점: 기존 교차점·내부 엣지·연결 지우기
    rprefs = tuple(f':{x}_' for x in rebuilt)
    for j in list(root.findall('junction')):
        if j.get('id') in rebuilt or j.get('id', '').startswith(rprefs):
            root.remove(j)
    for e in list(root.findall('edge')):
        if e.get('id', '').startswith(rprefs):
            root.remove(e)
    split_set = set(splits)
    for c in list(root.findall('connection')):
        f, via = c.get('from'), c.get('via', '')
        if f.startswith(rprefs) or via.startswith(rprefs) or \
                (f in split_set and edges[f].get('to') in rebuilt):
            root.remove(c)

    # 1) 나눈 엣지: 원래 (다시 만들지 않는) 교차점에 붙은 끝 조각은 원래 차로를 교차점 경계에서 자르고,
    #    나머지 조각은 부분망 그대로.
    taken_from_sub = set()
    for eid, cuts in splits.items():
        e = edges[eid]
        line = _edge_line(e, J)
        lanes = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
        fr, to = e.get('from'), e.get('to')
        ps = pieces[eid]
        pos = list(root).index(e)
        root.remove(e)
        keep_first = fr not in rebuilt
        keep_last = to not in rebuilt
        els = {}
        for i, pid in enumerate(ps):
            if (i == 0 and keep_first) or (i == len(ps) - 1 and keep_last):
                el = ET.Element('edge', {k: v for k, v in e.attrib.items()})
                el.set('id', pid)
                el.set('from', fr if i == 0 else sub_edges[pid].get('from'))
                el.set('to', to if i == len(ps) - 1 else sub_edges[pid].get('to'))
                bnd = [0.0] + [s for _, s in cuts] + [_length(line)]
                if e.get('shape'):
                    el.set('shape', _write(_slice(line, bnd[i], bnd[i+1])))
                els[pid] = el
            else:
                el = sub_edges[pid]
                taken_from_sub.add(pid)
            root.insert(pos + i, el)
        for l in lanes:
            k = int(l.get('index'))
            pts = _read(l.get('shape'))
            for i, pid in enumerate(ps):
                if pid not in els:
                    continue
                if i == 0:
                    a = sub_lane[(pid, k)][-1]
                    shp = _slice(pts, 0.0, _project(pts, a))[:-1] + [a]
                else:
                    b = sub_lane[(pid, k)][0]
                    shp = [b] + _slice(pts, _project(pts, b), _length(pts))[1:]
                ln = ET.SubElement(els[pid], 'lane', dict(l.attrib))
                ln.set('id', f'{pid}_{k}')
                ln.set('shape', _write(shp))
                ln.set('length', f'{_length(shp):.6f}')
        last = ps[-1]
        if keep_last and last != eid:
            for c in root.findall('connection'):
                if c.get('from') == eid:
                    c.set('from', last)
            for j in root.findall('junction'):
                if j.get('id') == to or j.get('id', '').startswith(f':{to}_'):
                    j.set('incLanes', ' '.join(x.replace(f'{eid}_', f'{last}_', 1) if x.rsplit('_', 1)[0] == eid
                                               else x for x in j.get('incLanes', '').split()))

    # 2) 새 길, 새·다시 만든 교차점, 내부 엣지, 연결
    prefs = tuple(f':{x}_' for x in sub_nodes)
    street_edges = {f'{s[0]}_{d}' for s in STREETS for d in ('f', 'r')}
    sub_taken = street_edges | taken_from_sub
    all_pieces = {p for ps in pieces.values() for p in ps}

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in street_edges or e.get('id', '').startswith(prefs):
            root.insert(pos, e)
            pos += 1
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in sub_nodes or j.get('id', '').startswith(prefs):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        f, t = c.get('from'), c.get('to')
        if f.startswith(prefs) or (f in all_pieces and sub_edges[f].get('to') in sub_nodes) or \
                (f in sub_taken and sub_edges[f].get('to') in sub_nodes):
            root.insert(pos, c)
            pos += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'interior_streets': len(STREETS), 'interior_junctions': len(sub_nodes),
            'interior_split_edges': len(splits)}


def apply_deco(deco_path, net_path):
    """안쪽 길 노란 중앙선, 정지선(양보하는 쪽: 연결이 모두 'm'), 갈림길 진행 방향 화살표."""
    from env.roundabout_builder import _arrow, _read_lane_shapes, _stop_line
    from env.roundabout_geometry import _offset
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    shapes = _read_lane_shapes(net_path)
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith(PREFIX):
            root.remove(p)
    h = LANE_W / 2
    n_center = 0
    jxy = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    n_dash = 0
    for name, a, *_ in STREETS:
        nlan = LANES.get(name, 1)
        center = _offset(lane[f'{name}_f_{nlan - 1}'], h + MEDIAN.get(name, 0.0)/2)
        # 편도 여러 차로: 같은 방향 차로 사이 흰 점선 (3m 칠, 3m 띔)
        for d in ('f', 'r'):
            for k in range(nlan - 1):
                if f'{name}_{d}_{k}' not in lane:
                    continue
                div = _offset(lane[f'{name}_{d}_{k}'], h)
                L = _length(div)
                st = 1.5
                while st + 3.0 <= L - 1.5:
                    ET.SubElement(root, 'poly', dict(id=f'{name}_{d}_dash_{k}_{n_dash}', color=WHITE, fill='0',
                                                      lineWidth='0.15', layer='6', shape=_write(_slice(div, st, st + 3.0))))
                    n_dash += 1
                    st += 6.0
        if name in DOUBLE_CENTER:
            # 117번길 연장: 117번길과 같은 겹 중앙선(회색 바탕 1.0m + 노란 선 두 줄, 0.3m 간격)을 끝점에서 그대로 잇는다
            if math.dist(jxy[a], center[0]) > 0.05:
                center = [jxy[a]] + center
            ET.SubElement(root, 'poly', dict(id=f'{name}_median_surface', color='84,84,90', fill='0',
                                              lineWidth='1.0', layer='5', shape=_write(center)))
            for k, off in enumerate((0.15, -0.15)):
                ET.SubElement(root, 'poly', dict(id=f'{name}_center_{k}', color=YELLOW, fill='0', lineWidth='0.15',
                                                  layer='6', shape=_write(_offset(center, off))))
        elif name in MEDIAN:
            # 가운데 띠가 있는 홑 중앙선 (북서 모서리 막다른 길 SW_final_left 와 같은 모양: 회색 띠 + 노란 선 0.2)
            if math.dist(jxy[a], center[0]) > 0.05:
                center = [jxy[a]] + center
            ET.SubElement(root, 'poly', dict(id=f'{name}_median_surface', color='84,84,90', fill='0',
                                              lineWidth=f'{MEDIAN[name]:.1f}', layer='5', shape=_write(center)))
            ET.SubElement(root, 'poly', dict(id=f'{name}_center', color=YELLOW, fill='0', lineWidth='0.2',
                                              layer='6', shape=_write(center)))
        else:
            ET.SubElement(root, 'poly', dict(id=f'{name}_center', color=YELLOW, fill='0', lineWidth='0.15',
                                              layer='6', shape=_write(center)))
        n_center += 1
    # 정지선: 새·다시 만든 교차점으로 들어가는 길 중 모든 연결이 양보(m)인 쪽
    jset = set(NODES) | {t[0] for t in TEES} | set(REBUILD)
    edge_to = {e.get('id'): e.get('to') for e in net.findall('edge') if e.get('function') != 'internal'}
    states, turns, lturns = {}, {}, {}
    for c in net.findall('connection'):
        f = c.get('from')
        if f.startswith(':') or edge_to.get(f) not in jset:
            continue
        states.setdefault(f, set()).add(c.get('state', 'M'))
        a = lane[f'{f}_{c.get("fromLane")}']
        b = lane[f'{c.get("to")}_{c.get("toLane")}']
        d1 = (a[-1][0]-a[-2][0], a[-1][1]-a[-2][1])
        d2 = (b[1][0]-b[0][0], b[1][1]-b[0][1])
        ang = math.degrees(math.atan2(d1[0]*d2[1]-d1[1]*d2[0], d1[0]*d2[0]+d1[1]*d2[1]))
        kind = 'straight' if abs(ang) < 35 else ('left' if ang > 0 else 'right')
        turns.setdefault(f, set()).add(kind)
        lturns.setdefault((f, int(c.get('fromLane'))), set()).add(kind)
    marks = []
    n_stop = n_arrow = 0
    kinds = {frozenset({'straight'}): ['straight'], frozenset({'left'}): ['left'], frozenset({'right'}): ['right'],
             frozenset({'straight', 'left'}): ['straight_left'], frozenset({'straight', 'right'}): ['straight_right'],
             frozenset({'left', 'right'}): ['left', 'right'],
             frozenset({'straight', 'left', 'right'}): ['straight_left', 'right']}
    for f, st in sorted(states.items()):
        if not f.startswith(PREFIX):
            continue
        if st <= {'m', 's', 'w'}:
            _stop_line(marks, f'{PREFIX}stop_{f}', shapes, f, dist=1.0)
            n_stop += 1
        tee_multi = edge_to[f] in {t[0] for t in TEES} and len([k for k in lturns if k[0] == f]) > 1   # 이중 우회전 진출
        if ((edge_to[f] in NODES or edge_to[f] in REBUILD) and turns[f] != {'straight'}) or tee_multi:   # 그냥 이어지는 곳은 화살표 없음
            L = _length(lane[f'{f}_0'])
            nl = len([k for k in lturns if k[0] == f])
            if L > 20:
                for k in sorted(k for (ff, k) in lturns if ff == f):
                    for kind in kinds.get(frozenset(lturns[(f, k)]), []):
                        aid = f'{PREFIX}arrow_{f}_{kind}' if nl == 1 else f'{PREFIX}arrow_{f}_{k}_{kind}'
                        _arrow(marks, aid, shapes, f, k, kind, 8.0)
                        n_arrow += 1
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'interior_centerlines': n_center, 'interior_stoplines': n_stop, 'interior_arrows': n_arrow,
            'interior_lane_dashes': n_dash}


CFG_KEYS = ('TEES', 'REBUILD', 'NODES', 'STREETS', 'PREFIX', 'MEDIAN', 'DOUBLE_CENTER', 'APPROACH',
            'MIDS', 'ONEWAY', 'EXTRA_CON', 'SPEED', 'PIECE_OVERRIDE', 'LANES', 'NODE_POS', 'NODE_SHAPES', 'NODE_TYPES')


def with_cfg(cfg, fn, *args):
    """다른 지역 골목(예: env.roundabout_majeon)이 같은 방식으로 길을 깔 수 있게, 설정만 잠시 바꿔 fn 을 부른다."""
    g = globals()
    old = {k: g[k] for k in cfg if k in CFG_KEYS}
    g.update({k: v for k, v in cfg.items() if k in CFG_KEYS})
    try:
        return fn(*args)
    finally:
        g.update(old)


def main():
    import sys
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, base)
    net_path = os.path.join(base, 'env', 'sumo_roundabout', 'roundabout.net.xml')
    deco_path = os.path.join(base, 'env', 'sumo_roundabout', 'roundabout.deco.xml')
    print(apply_net(net_path))
    print(apply_deco(deco_path, net_path))


if __name__ == '__main__':
    main()
