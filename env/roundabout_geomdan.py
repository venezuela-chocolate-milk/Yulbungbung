"""검단사거리 북동쪽 (완정로 북쪽 ~ 검단로 ~ 검단고 앞 큰길 ~ 마전역 북쪽 길) 도로. 2026-11-12 네이버 지도 캡처 기준.

  가장 중요한 것: 새 길이 기존 큰길과 자연스럽게 이어지고 차가 문제없이 지나가는 것. 차로 수는 길 중간에서 바꾸지 않는다.
  - 검단로: 검단사거리에서 북동쪽으로 뻗은 막다른 짧은 길(X_main_in 들어감 3차로 / 회색 분리대 1.0m + 겹 노란선 /
    X_main_out 나감 2차로)을 같은 단면 그대로 끝에 이어(짧은 길 이름 유지, RL 배경 교통이 쓴다) 네이버처럼 북동쪽으로 간다.
  - 위쪽 갈림점(gd_JT): 검단로가 검단고·능내초 앞 큰길(동쪽 큰길, 2/2차로)로 꺾이고 북쪽으로 짧은 길(맵 끝까지)이 갈라진다.
    차로 수가 바뀌는 곳(3/2 → 2/2)은 길 중간이 아니라 이 교차점.
  - 동쪽 큰길: 남동쪽으로 내려와 마전역 북쪽 길(mx_ne_in 3 / mx_ne_out 1)에 T자(gd_TE)로 붙는다 (모든 방향 허용).
  - 골목(왕복 1차로 3.2m, 30 km/h): 네이버 골목망을 따라 구시가지 격자, 힐스테이트 둘레 길, 540번길, 아이파크 북쪽 길.
    큰길(완정로·검단로·동쪽 큰길)과 만나는 곳은 우회전 진입·진출만 (분리대·겹 노란선을 가로지르지 않는다).
  큰길 엣지를 나눌 때 기존 교차점 쪽 조각이 원래 이름을 쓴다 (X_se_in 은 검단사거리 쪽, mx_ne_* 는 마전역 쪽).

roundabout_south_extend 가 마전동 북쪽 다음 단계로 호출한다.
그림(건물·공원)은 env.roundabout_geomdan_scenery.
"""
import math
import xml.etree.ElementTree as ET

PREFIX = 'gd_'
LANE_W = 3.2
MED = 1.0                           # 검단로 짧은 길과 같은 회색 분리대 폭
SPEED_BIG = 13.89
SPEED = 8.33
APPROACH = 20.0                     # 골목이 큰길 T자에 수직으로 들어가는 곧은 구간 (m)
YELLOW, WHITE, MEDIAN_GRAY = '235,190,40', '255,255,255', '84,84,90'

STUB = ('X_main_in', 'X_main_out')  # 검단사거리로 들어감(3), 나감(2)
STUB_END = 'X_main_end'
WJ = 'X_se_in'                      # 완정로 북행 (북쪽 차로, 마전역 → 검단사거리)
MX = ('mx_ne_in', 'mx_ne_out')      # 마전역 북쪽 길: 마전역으로 들어감(3), 나감(1)

# 네이버 캡처(이 작업에서 사용자가 준 그림, 842x639 px) → SUMO: 검단사거리·마전역 두 교차점으로 맞춘 닮음 변환 (회전 없음)
NAV_SCALE = 1.322


def NV(px, py):
    return (570.0 + (px - 118.0)*NAV_SCALE, 414.0 - (py - 215.0)*NAV_SCALE)


# 검단로 (짧은 길 끝 → 갈림점), 동쪽 큰길 (갈림점 → 마전역 북쪽 길 T자) 조정점
GD_CTRL = [NV(343, 103), NV(445, 55)]
JT = NV(500, 25)
ER_CTRL = [NV(590, 127), NV(650, 195), NV(690, 250), NV(708, 320), (1390.0, 170.0)]
TE_NEAR = (1525.0, 54.0)
NORTH_LEN = 50.0
NORTH_DIR = (-0.17, 0.985)

# 큰길 위 골목 T자: (교차점, 큰길('GD' | 'ER' | 'WJ'), 대략 위치)
TEES = [
    ('gd_G1', 'GD', NV(247, 150)),
    ('gd_G2', 'GD', NV(470, 45)),
    ('gd_E1', 'ER', NV(590, 127)),
    ('gd_E2', 'ER', NV(707, 345)),
    ('gd_W1', 'WJ', NV(205, 262)),
    ('gd_W2', 'WJ', NV(338, 343)),
    ('gd_W3', 'WJ', NV(390, 362)),
    ('gd_W4', 'WJ', NV(605, 485)),
]
# 골목 안 교차점
NODES = {
    'gd_J1': NV(263, 182),       # 구시가지 네거리 (W-a · W-b · 501번안길)
    'gd_J2': NV(338, 262),       # 서구청 동쪽 길 · 유호아파트 앞길
    'gd_JS1': NV(370, 125),      # 501번안길 · 세븐일레븐 길
    'gd_JR': NV(400, 111),       # 501번안길 · 156번안길
    'gd_JP': NV(478, 73),        # 501번안길 동쪽 끝 · 뉴월드빌 길
    'gd_JQ1': NV(420, 240),      # 유호아파트 앞
    'gd_JE': NV(476, 198),       # 뉴월드빌 길 남쪽 끝
    'gd_JD': NV(514, 199),       # 아이파크 북서 모서리 (방현공원 쪽 비스듬한 길)
    'gd_JN': NV(513, 228),
    'gd_JH': NV(512, 287),       # 힐스테이트 북쪽 길 · 아이파크 서쪽 길
    'gd_J540': NV(618, 400),     # 540번길 · 힐스테이트 동쪽 길
}
# 골목: (이름, 시작, 끝, 우선순위, 중간 조정점)
STREETS = [
    ('gd_wa', 'gd_W1', 'gd_J1', 0, [NV(243, 193)]),
    ('gd_wb1', 'gd_G1', 'gd_J1', 1, []),
    ('gd_wb2', 'gd_J1', 'gd_J2', 1, [NV(287, 207), NV(310, 233)]),
    ('gd_wb3', 'gd_J2', 'gd_W2', 1, [NV(350, 300)]),
    ('gd_p1', 'gd_J1', 'gd_JS1', 1, [NV(343, 138)]),
    ('gd_p2', 'gd_JS1', 'gd_JR', 1, []),
    ('gd_p3', 'gd_JR', 'gd_JP', 1, [NV(430, 97)]),
    ('gd_gp', 'gd_G2', 'gd_JP', 0, []),
    ('gd_s1', 'gd_JS1', 'gd_JQ1', 0, [NV(377, 150), NV(400, 185)]),
    ('gd_q1', 'gd_J2', 'gd_JQ1', 0, [NV(380, 252)]),
    ('gd_q2', 'gd_JQ1', 'gd_JN', 0, [NV(465, 232)]),
    ('gd_156', 'gd_JR', 'gd_JE', 0, [NV(425, 167), NV(442, 186)]),
    ('gd_n475', 'gd_JP', 'gd_JE', 0, [NV(474, 110)]),
    ('gd_e200', 'gd_JE', 'gd_JD', 0, []),
    ('gd_n513a', 'gd_JD', 'gd_JN', 0, []),
    ('gd_n513b', 'gd_JN', 'gd_JH', 0, []),
    ('gd_sd', 'gd_JD', 'gd_E1', 0, [NV(555, 160)]),
    ('gd_hw', 'gd_W3', 'gd_JH', 1, [NV(400, 343), NV(430, 302), NV(490, 288)]),
    ('gd_he', 'gd_JH', 'gd_J540', 1, [NV(580, 287), NV(600, 307), NV(612, 350)]),
    ('gd_h2', 'gd_J540', 'gd_W4', 1, [NV(617, 450)]),
    ('gd_540', 'gd_J540', 'gd_E2', 1, [NV(653, 390), NV(680, 377)]),
]


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _unit(v):
    L = math.hypot(*v)
    return (v[0]/L, v[1]/L)


def plan(net):
    from env.roundabout_interior import _fillet, _project, _at, _dir_at, _slice, _length
    from env.roundabout_geometry import _offset
    es = {e.get('id'): e for e in net.findall('edge')}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    # ── 검단로 중심선: 짧은 길 나가는 차로(맨 왼쪽) 끝에서 왼쪽으로 1.6 + 0.5 m (분리대 가운데) ──
    lo = lane[f'{STUB[1]}_1']
    d = _unit((lo[-1][0] - lo[-2][0], lo[-1][1] - lo[-2][1]))
    lf = (-d[1], d[0])
    off = LANE_W/2 + MED/2
    c0 = (lo[-1][0] + lf[0]*off, lo[-1][1] + lf[1]*off)
    c1 = (c0[0] + d[0]*30.0, c0[1] + d[1]*30.0)
    gd = _fillet([c0, c1] + GD_CTRL + [JT], radius=160.0, step=3.0)
    # ── 동쪽 큰길: 갈림점 → 마전역 북쪽 길 T자 (마지막은 그 길에 수직으로) ──
    mxo = _read(es[MX[1]].get('shape'))
    s_te = _project(mxo, TE_NEAR)
    te = _at(mxo, s_te)
    u = _dir_at(mxo, s_te)
    nrm = (-u[1], u[0])                                   # 북동행 차로의 왼쪽 = 북서쪽 (동쪽 큰길이 오는 쪽)
    mxi = _read(es[MX[0]].get('shape'))
    te_c = _at(mxi, _project(mxi, te))                    # 두 방향 엣지 중심선 사이 = T자 중심
    te = ((te[0] + te_c[0])/2, (te[1] + te_c[1])/2)
    q = (te[0] + nrm[0]*35.0, te[1] + nrm[1]*35.0)
    er = _fillet([JT] + ER_CTRL + [q, te], radius=90.0, keep={len(ER_CTRL) + 1: 17.0}, step=3.0)
    nst = [JT, (JT[0] + NORTH_DIR[0]*NORTH_LEN, JT[1] + NORTH_DIR[1]*NORTH_LEN)]
    nodes = dict(NODES)
    nodes.update({'gd_JT': JT, 'gd_TE': te, 'gd_N0': nst[1]})
    # ── 큰길 위 T자 ──
    wj = _read(es[WJ].get('shape'))
    lines = {'GD': gd, 'ER': er, 'WJ': wj}
    tee = {}
    for node, road, near in TEES:
        line = lines[road]
        s = _project(line, near)
        p, dd = _at(line, s), _dir_at(line, s)
        if road == 'ER':
            inward = (dd[1], -dd[0])                     # 남행 오른쪽 = 서쪽
        else:
            inward = (dd[1], -dd[0])                     # 검단로 북동행 오른쪽 = 남동쪽, 완정로 북서행 오른쪽 = 북동쪽
        nodes[node] = p
        tee[node] = (road, s, inward)
    # ── 큰길 조각 (분리대 가운데 선 기준 오프셋, 가운데 퍼짐) ──
    streets = []

    def big(name, line, cuts, a, b, n_fwd, n_back, prio):
        """중심선 line 을 cuts(교차점 이름, s)에서 나눠 조각마다 fwd(오른쪽)/back(왼쪽) 엣지."""
        seq = [a] + [c for c, _ in cuts] + [b]
        bnd = [0.0] + [s for _, s in cuts] + [_length(line)]
        out = []
        for k in range(len(seq) - 1):
            seg = _slice(line, bnd[k], bnd[k+1])
            fid, bid = f'{name}{k + 1}_f', f'{name}{k + 1}_b'
            ofw = MED/2 + LANE_W*n_fwd/2
            obk = MED/2 + LANE_W*n_back/2
            streets.append((fid, seq[k], seq[k+1], n_fwd, LANE_W, SPEED_BIG, prio, _offset(seg, -ofw), 'center'))
            streets.append((bid, seq[k+1], seq[k], n_back, LANE_W, SPEED_BIG, prio, _offset(seg, obk)[::-1], 'center'))
            out.append((fid, bid))
        return out
    gcuts = sorted([(n, tee[n][1]) for n in tee if tee[n][0] == 'GD'], key=lambda x: x[1])
    ecuts = sorted([(n, tee[n][1]) for n in tee if tee[n][0] == 'ER'], key=lambda x: x[1])
    gpieces = big('gd_k', gd, gcuts, STUB_END, 'gd_JT', 2, 3, 3)       # f = 북동행 2, b = 남서행 3
    epieces = big('gd_e', er, ecuts, 'gd_JT', 'gd_TE', 2, 2, 3)        # f = 남행 2, b = 북행 2
    npieces = big('gd_n', nst, [], 'gd_JT', 'gd_N0', 2, 2, 1)          # f = 북행 2, b = 남행 2
    # ── 골목 모양 ──
    allxy = dict(J)
    allxy.update(nodes)
    for name, a, b, prio, mids in STREETS:
        ctrl = [allxy[a]] + list(mids) + [allxy[b]]
        keep = {}
        for end in (a, b):
            if end in tee:
                _, _, inward = tee[end]
                p = allxy[end]
                qq = (p[0] + inward[0]*APPROACH, p[1] + inward[1]*APPROACH)
                ctrl = [p, qq] + ctrl[1:] if end == a else ctrl[:-1] + [qq, p]
        for end in (a, b):
            if end in tee:
                keep[1 if end == a else len(ctrl) - 2] = APPROACH*0.5
        pts = _fillet(ctrl, radius=30.0, keep=keep, step=2.0)
        streets.append((f'{name}_f', a, b, 1, LANE_W, SPEED, prio, pts, 'right'))
        streets.append((f'{name}_r', b, a, 1, LANE_W, SPEED, prio, pts[::-1], 'right'))
    # ── 기존 엣지 나누기 ──
    wcuts = sorted([(n, tee[n][1]) for n in tee if tee[n][0] == 'WJ'], key=lambda x: x[1])
    wnames = [f'{WJ}_gd{k + 1}' for k in range(len(wcuts))] + [WJ]
    s_in = _project(mxi, te)
    splits = [(STUB[0], [], [STUB[0]]), (STUB[1], [], [STUB[1]]),
              (WJ, wcuts, wnames),
              (MX[0], [('gd_TE', s_in)], [f'{MX[0]}_gd', MX[0]]),
              (MX[1], [('gd_TE', s_te)], [MX[1], f'{MX[1]}_gd'])]
    # ── 연결 ──
    con = []
    (k1f, k1b) = gpieces[0]
    con += [(STUB[1], k1f, k, k) for k in range(2)] + [(k1b, STUB[0], k, k) for k in range(3)]

    def alley_io(node):
        for name, a, b, *_ in STREETS:
            if a == node:
                return f'{name}_f', f'{name}_r'          # (들어가는 엣지, 나오는 엣지)
            if b == node:
                return f'{name}_r', f'{name}_f'
        raise KeyError(node)

    def tee_con(pieces, cuts, n_fwd, n_back):
        for k, (node, _) in enumerate(cuts):
            (af, ab), (bf, bb) = pieces[k], pieces[k + 1]
            ain, aout = alley_io(node)
            con.extend([(af, bf, i, i) for i in range(n_fwd)] + [(bb, ab, i, i) for i in range(n_back)])
            con.extend([(af, ain, 0, 0), (aout, bf, 0, 0)])        # 우회전 진입·진출
    tee_con(gpieces, gcuts, 2, 3)
    tee_con(epieces, ecuts, 2, 2)
    # 갈림점 gd_JT: 검단로 ↔ 동쪽 큰길이 주 흐름, 북쪽 짧은 길은 양보
    kf, kb = gpieces[-1]
    ef, eb = epieces[0]
    nf, nb = npieces[0]
    con += [(kf, ef, 0, 0), (kf, ef, 1, 1), (kf, nf, 1, 1),
            (eb, kb, 0, 1), (eb, kb, 1, 2), (eb, nf, 0, 0),
            (nb, kb, 0, 0), (nb, ef, 1, 1)]
    # T자 gd_TE: 동쪽 큰길 남행 → 마전역(우회전 2차로) / mx_JC(좌회전), 마전역 북쪽 길 → 동쪽 큰길 북행
    ef, eb = epieces[-1]
    con += [(ef, MX[0], 0, 0), (ef, MX[0], 1, 1), (ef, f'{MX[1]}_gd', 1, 0),
            (f'{MX[0]}_gd', eb, 0, 0), (MX[1], eb, 0, 1)]
    # 완정로 T자: 우회전 진입·진출 (X_se_in 0번 = 바깥 차로)
    for k, (node, _) in enumerate(wcuts):
        ain, aout = alley_io(node)
        con += [(wnames[k], ain, 0, 0), (aout, wnames[k + 1], 0, 0)]
    return dict(nodes=nodes, streets=streets, splits=splits, con=con, gd=gd, er=er, nst=nst,
                gpieces=gpieces, epieces=epieces, npieces=npieces, tee=tee, c0=c0)


_PLAN = None


def apply_net(net_path):
    global _PLAN
    from env.roundabout_graft import graft
    from env.roundabout_nw_shops import _strip_short_internal
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(PREFIX) for e in net.findall('edge')):
        raise RuntimeError('검단 북동쪽 도로가 이미 적용됨')
    P = plan(net)
    _PLAN = P
    new_nodes = {k: v for k, v in P['nodes'].items() if k not in ('gd_TE',) and not k.startswith('gd_W')}
    graft(net_path, new_nodes, P['streets'], P['splits'], attach={STUB_END}, extra_con=P['con'],
          node_types={'gd_N0': 'dead_end'})
    _strip_short_internal(net_path, tuple(P['nodes']) + (STUB_END,))
    from env.roundabout_interior import _length
    return {'gd_streets': len(STREETS), 'gd_edges': len(P['streets']),
            'gd_geomdanro_m': round(_length(P['gd']), 1), 'gd_east_road_m': round(_length(P['er']), 1)}


# 골목 표시는 env.roundabout_interior.apply_deco 로 (이름·교차점만 쓴다)
def _cfg():
    nodes = dict(NODES)
    for n, *_ in TEES:
        nodes[n] = (0.0, 0.0)
    nodes.update({'gd_JT': (0.0, 0.0), 'gd_TE': (0.0, 0.0)})
    return dict(TEES=[], REBUILD={}, NODES=nodes, STREETS=STREETS, PREFIX=PREFIX, MEDIAN={}, DOUBLE_CENTER=set(),
                APPROACH=APPROACH)


# 기존 선을 끊는 교차점: 좌회전이 가로지르는 마전역 북쪽 길 T자만. 완정로 우회전 T자(gd_W*)와 짧은 길 이음(X_main_end)은
# 실제 도로처럼 차선 점선·중앙선이 그대로 지나간다 (끊으면 점선 길이가 달라져 기하 테스트에도 걸린다).
NEW_JUNCTIONS = ('gd_TE',)


def apply_deco(deco_path, net_path):
    """1) 기존 노면 선(완정로·마전역 북쪽 길·짧은 길 끝)이 새 교차점 안을 지나가면 끊는다.
    2) 검단로·동쪽 큰길·북쪽 짧은 길: 짧은 길과 같은 회색 분리대 1.0m + 겹 노란선, 같은 방향 차로 사이 흰 점선.
    3) 골목: 안쪽 길 공통 표시(노란 중앙선, 양보 쪽 정지선, 갈림길 화살표)."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    from env.roundabout_geometry import _offset
    from env.roundabout_markings import dash_style
    from env.roundabout_nw_shops import _dashes
    from env import roundabout_interior as it
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    jsh = {j.get('id'): Polygon(_read(j.get('shape'))) for j in net.findall('junction')
           if j.get('shape') and j.get('type') != 'internal' and len(_read(j.get('shape'))) >= 3}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith('gdr_'):
            root.remove(p)
    # 1) 새 교차점 안을 지나는 기존 선 끊기
    cut = unary_union([jsh[j].buffer(0.3) for j in NEW_JUNCTIONS if j in jsh])
    n_clip = 0
    for p in list(root.findall('poly')):
        if p.get('fill') != '0' or not p.get('shape') or p.get('id', '').startswith(PREFIX):
            continue
        pts = _read(p.get('shape'))
        if len(pts) < 2 or not LineString(pts).intersects(cut):
            continue
        rest = LineString(pts).difference(cut)
        idx = list(root).index(p)
        root.remove(p)
        for k, part in enumerate(x for x in getattr(rest, 'geoms', [rest]) if x.geom_type == 'LineString' and x.length > 0.5):
            el = ET.Element('poly', dict(p.attrib))
            el.set('id', p.get('id') if k == 0 else f"{p.get('id')}_gdc{k}")
            el.set('shape', _write(list(part.coords)))
            root.insert(idx + k, el)
        n_clip += 1
    # 2) 큰길 분리대·겹 노란선: 조각마다 북동행/남행(f) 맨 왼쪽 차로 왼쪽 1.6 + 0.5 m. 갈림점·T자 바닥 안은 비운다.
    big_cut = unary_union([jsh[j].buffer(0.2) for j in ('gd_JT', 'gd_TE') if j in jsh])
    roads = {'k': ('gd_k', 2, 3), 'e': ('gd_e', 2, 2), 'n': ('gd_n', 2, 2)}
    n_dash = 0
    dash, gap, _ = dash_style(20)
    for key, (base, nf, nb) in roads.items():
        k = 1
        center = []
        while f'{base}{k}_f_0' in lane:
            mid = _offset(lane[f'{base}{k}_f_{nf - 1}'], LANE_W/2 + MED/2)
            center += mid if not center else mid[1:]
            for i in range(nf - 1):
                n_dash += _dashes(root, f'gdr_{base}{k}_fd{i}', _offset(lane[f'{base}{k}_f_{i}'], LANE_W/2), dash, gap)
            for i in range(nb - 1):
                n_dash += _dashes(root, f'gdr_{base}{k}_bd{i}', _offset(lane[f'{base}{k}_b_{i}'], LANE_W/2), dash, gap)
            k += 1
        if key == 'k':
            # 짧은 길 분리대 끝(X_main_end)에서 끊김 없이 이어지게 그 끝점부터
            for p in root.findall('poly'):
                if p.get('id') == 'x_median_main':
                    e0 = _read(p.get('shape'))[-1]
                    if math.dist(e0, center[0]) > 0.05:
                        center = [e0] + center
        line = LineString(center).difference(big_cut)
        for m, part in enumerate(x for x in getattr(line, 'geoms', [line]) if x.geom_type == 'LineString' and x.length > 1.0):
            pts = list(part.coords)
            ET.SubElement(root, 'poly', dict(id=f'gdr_{key}_median_{m}', color=MEDIAN_GRAY, fill='0',
                                              lineWidth=f'{MED:.1f}', layer='5', shape=_write(pts)))
            for j, o in enumerate((0.15, -0.15)):
                ET.SubElement(root, 'poly', dict(id=f'gdr_{key}_center_{m}_{j}', color=YELLOW, fill='0',
                                                  lineWidth='0.15', layer='6', shape=_write(_offset(pts, o))))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    info = {'gd_deco_clipped': n_clip, 'gd_big_dashes': n_dash}
    info.update({f'gd_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(_cfg(), it.apply_deco, deco_path, net_path).items()})
    return info
