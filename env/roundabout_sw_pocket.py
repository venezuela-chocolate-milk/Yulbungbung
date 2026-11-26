"""서곶로 × 원당대로 사거리(SW_new_cross) 동행 좌회전 전용 차로 늘리기 (2026-11-15 사용자 요청).

  예전: 본선 4차로가 SW_end_T_taper_out(40 m) 동안 한 차로 폭만큼 오른쪽으로 비켜 나며 왼쪽에 노란 빗금
  안전지대(약 54 m)가 생기고, 그 끝에서야 좌회전 차로가 시작돼 정지선까지 약 24 m(차 3대)뿐이었다.
  안전지대를 통째로 뒤로 미는 것은 오른쪽 우회전 도로(SW_long_right)가 본선 바로 옆에 붙어 있어 안 되므로,
  안전지대 시작점은 그대로 두고 비켜 나는 구간을 TAPER m 로 줄이고(우회전 도로도 같이), 좌회전 차로 진입
  구간을 OPEN m 로 해서 좌회전 차로를 TAPER + OPEN 지점부터 시작하게 한다 → 약 50 m.

  - 본선 taper_out 0~3 차로 + SW_long_right + 그 옆 노면 그림: 비켜 나는 양(측방 이동) s_old(t) → s_new(t)
  - taper_out 을 TAPER 지점에서 나눠 뒤 토막(SW_end_T_pocket)을 5차로로: 좌회전 차로(4번)가 안전지대 끝의 새
    교차점에서 바로 시작 (옆 차로와 겹쳐 그려지지 않게). SW_end_T_widen 에 4→4 연결, extension_out 4번 차로는 곧게.
  - 안전지대(바닥·노란 테두리·빗금)는 새 모양으로 다시 그림, 3·4번 차로 사이 흰 점선, 좌회전 화살표.
roundabout_south_extend 가 늘어나는 차로 시작점 고치기(9-20) 바로 뒤에서 부른다.
"""
import math
import xml.etree.ElementTree as ET

TAPER, OPEN = 20.0, 0.0     # OPEN 0: 안전지대가 TAPER 에서 끝나고 바로 좌회전 차로 (새 교차점에서 시작)
POCKET_E, POCKET_J = 'SW_end_T_pocket', 'SW_end_T_pocket_start'
LANE_W = 3.2
TAPER_E, EXT_E, WIDEN_J, LONG_RIGHT = 'SW_end_T_taper_out', 'SW_end_T_extension_out', 'SW_end_T_widen', 'SW_long_right'
CENTER_L = -11.2          # 중앙선 (taper 0번 차로 시작점 기준 측방 좌표, 오른쪽 +)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _len(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Frame:
    """taper 시작(0번 차로) 기준 (t: 진행 방향 m, l: 오른쪽 + m)."""

    def __init__(self, lane0_start, u, s_old_pts):
        self.o, self.u = lane0_start, u
        self.n = (u[1], -u[0])
        self.s_old_pts = s_old_pts          # [(t, s_old)]

    def tl(self, q):
        d = (q[0] - self.o[0], q[1] - self.o[1])
        return d[0] * self.u[0] + d[1] * self.u[1], d[0] * self.n[0] + d[1] * self.n[1]

    def xy(self, t, l):
        return (self.o[0] + self.u[0] * t + self.n[0] * l, self.o[1] + self.u[1] * t + self.n[1] * l)

    def s_old(self, t):
        P = self.s_old_pts
        if t <= P[0][0]:
            return P[0][1] if t >= 0 else 0.0
        for (t0, s0), (t1, s1) in zip(P, P[1:]):
            if t <= t1:
                return s0 + (s1 - s0) * (t - t0) / max(t1 - t0, 1e-9)
        return LANE_W

    @staticmethod
    def s_new(t):
        return LANE_W * _smooth(t / TAPER) if t > 0 else 0.0

    def delta(self, t):
        if t <= 0 or t >= self.s_old_pts[-1][0]:
            return 0.0
        return max(self.s_new(t) - self.s_old(t), 0.0)

    def move(self, q):
        """오른쪽 무리(본선 차로·우회전 도로) 점이면 새 비켜 나기 양만큼 옮긴다."""
        t, l = self.tl(q)
        if l < CENTER_L + self.s_old(t) - 0.05:
            return q
        dl = self.delta(t)
        return self.xy(t, l + dl) if dl else q


def _densify(pts, step=1.0):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / step))
        for k in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return out


def _frame(root):
    edges = {e.get('id'): e for e in root.findall('edge')}
    lanes = {int(l.get('index')): _read(l.get('shape')) for l in edges[TAPER_E].findall('lane')}
    if len(lanes) != 4:
        raise RuntimeError(f'{TAPER_E} 는 4차로여야 함 (이미 적용됨?)')
    l0 = lanes[0]
    lm = [_read(l.get('shape')) for l in edges['SW_long_main_continue'].findall('lane') if l.get('index') == '0'][0]
    u = (lm[-1][0] - lm[0][0], lm[-1][1] - lm[0][1])
    L = math.hypot(*u)
    u = (u[0] / L, u[1] / L)
    fr = Frame(l0[0], u, [])
    l3 = lanes[3]
    pts = [fr.tl(q) for q in l3]
    fr.s_old_pts = [(t, l - (-3 * LANE_W)) for t, l in pts]     # 3번 차로 중심 기준 비켜 난 양
    return fr, edges


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    fr, edges = _frame(root)
    T_END = fr.s_old_pts[-1][0]
    # 1) taper 0~3 차로, 우회전 도로: 새 비켜 나기
    taper = edges[TAPER_E]
    for l in taper.findall('lane'):
        pts = [fr.move(q) for q in _densify(_read(l.get('shape')))]
        l.set('shape', _write(pts))
        l.set('length', f'{_len(pts):.6f}')
    if taper.get('shape'):
        taper.set('shape', _write([fr.move(q) for q in _densify(_read(taper.get('shape')))]))
    lr = edges[LONG_RIGHT]
    for l in lr.findall('lane'):
        pts = [fr.move(q) for q in _densify(_read(l.get('shape')))]
        l.set('shape', _write(pts))
        l.set('length', f'{_len(pts):.6f}')
    if lr.get('shape'):
        lr.set('shape', _write([fr.move(q) for q in _densify(_read(lr.get('shape')))]))
    # 2) taper_out 을 TAPER 지점에서 둘로 나눈다: 앞 토막(4차로, 비켜 나기) + 좌회전 차로 토막 POCKET_E(5차로).
    #    좌회전 차로(4번)는 나눈 지점(새 교차점)에서 바로 시작 → 옆 직진 차로와 겹쳐 그려지지 않는다
    #    (2026-11-15: 예전엔 4번 차로가 3번 차로 위에서 비스듬히 시작해 두 차로 차가 서로 통과해 보였다)
    def split(pts):
        a, b = [], []
        for q0, q1 in zip(pts, pts[1:]):
            t0, t1 = fr.tl(q0)[0], fr.tl(q1)[0]
            if not a:
                a.append(q0)
            if t1 <= TAPER:
                a.append(q1)
            elif t0 < TAPER:
                f = (TAPER - t0) / (t1 - t0)
                m = (q0[0] + (q1[0] - q0[0]) * f, q0[1] + (q1[1] - q0[1]) * f)
                a.append(m)
                b += [m, q1]
            else:
                b.append(q1) if b else b.extend([q0, q1])
        return a, b
    pocket = ET.Element('edge', {'id': POCKET_E, 'from': POCKET_J, 'to': taper.get('to'),
                                 'priority': taper.get('priority', '-1'), 'spreadType': taper.get('spreadType', 'center')})
    if taper.get('type'):
        pocket.set('type', taper.get('type'))
    ends = {}
    for l in sorted(taper.findall('lane'), key=lambda l: int(l.get('index'))):
        a, b = split(_read(l.get('shape')))
        l.set('shape', _write(a))
        l.set('length', f'{_len(a):.6f}')
        nl = ET.SubElement(pocket, 'lane', dict(l.attrib))
        nl.set('id', f'{POCKET_E}_{l.get("index")}')
        nl.set('shape', _write(b))
        nl.set('length', f'{_len(b):.6f}')
        ends[int(l.get('index'))] = (a[-1], b[0])
    l3 = pocket.find("lane[@index='3']")
    lane4 = [fr.xy(fr.tl(q)[0], fr.tl(q)[1] - LANE_W) for q in _read(l3.get('shape'))]
    el = ET.SubElement(pocket, 'lane', dict(l3.attrib))
    el.set('id', f'{POCKET_E}_4')
    el.set('index', '4')
    el.set('shape', _write(lane4))
    el.set('length', f'{_len(lane4):.6f}')
    if taper.get('shape'):
        a, b = split(_read(taper.get('shape')))
        taper.set('shape', _write(a))
        pocket.set('shape', _write(b))
    taper.set('to', POCKET_J)
    root.insert(list(root).index(taper) + 1, pocket)
    # 새 교차점 (단순 통과 4연결, 좌회전 차로는 들어오는 연결 없음)
    mid = fr.xy(TAPER, CENTER_L + 0.0)
    ring = [fr.xy(TAPER - 0.3, CENTER_L), fr.xy(TAPER - 0.3, 6.4), fr.xy(TAPER + 0.3, 6.4), fr.xy(TAPER + 0.3, CENTER_L)]
    jn = ET.Element('junction', {'id': POCKET_J, 'type': 'priority', 'x': f'{mid[0]:.6f}', 'y': f'{mid[1]:.6f}',
                                 'incLanes': ' '.join(f'{TAPER_E}_{k}' for k in range(4)),
                                 'intLanes': ' '.join(f':{POCKET_J}_0_{k}' for k in range(4)), 'shape': _write(ring)})
    for k in range(4):
        ET.SubElement(jn, 'request', {'index': str(k), 'response': '0000', 'foes': '0000', 'cont': '0'})
    js = root.findall('junction')
    root.insert(list(root).index(js[-1]) + 1, jn)
    ie = ET.Element('edge', {'id': f':{POCKET_J}_0', 'function': 'internal'})
    for k in range(4):
        a, b = ends[k]
        if math.dist(a, b) < 1e-3:
            b = (a[0] + 1e-3, a[1])
        ET.SubElement(ie, 'lane', {'id': f':{POCKET_J}_0_{k}', 'index': str(k), 'speed': l3.get('speed'),
                                   'length': '0.001', 'width': l3.get('width', '3.2'), 'shape': _write([a, b])})
    root.insert(list(root).index(next(e for e in root.findall('edge') if e.get('function') == 'internal')), ie)
    # 연결: taper → pocket (새 교차점), pocket → extension (예전 taper → extension 을 이름만 바꿈 + 4번 추가)
    cons = root.findall('connection')
    last = list(root).index(cons[-1]) + 1
    for k in range(4):
        root.insert(last, ET.Element('connection', {'from': TAPER_E, 'to': POCKET_E, 'fromLane': str(k), 'toLane': str(k),
                                                    'via': f':{POCKET_J}_0_{k}', 'dir': 's', 'state': 'M'}))
        root.insert(last + 1, ET.Element('connection', {'from': f':{POCKET_J}_0', 'to': POCKET_E, 'fromLane': str(k),
                                                        'toLane': str(k), 'dir': 's', 'state': 'M'}))
        last += 2
    for c in cons:
        if c.get('from') == TAPER_E and c.get('to') == EXT_E:
            c.set('from', POCKET_E)
    # 3) extension 4번 차로: 처음부터 곧게 (3번 차로를 한 차로 폭 왼쪽으로)
    from env.roundabout_geometry import _offset
    ext = edges[EXT_E]
    e3 = next(l for l in ext.findall('lane') if l.get('index') == '3')
    e4 = next(l for l in ext.findall('lane') if l.get('index') == '4')
    p4 = _offset(_read(e3.get('shape')), LANE_W)
    p4[0] = lane4[-1]
    e4.set('shape', _write(p4))
    e4.set('length', f'{_len(p4):.6f}')
    # 4) SW_end_T_widen: 들어오는 차로 이름 바꾸기 + pocket 4 → extension 4 연결
    jw = next(j for j in root.findall('junction') if j.get('id') == WIDEN_J)
    jw.set('incLanes', ' '.join(f'{POCKET_E}_{k}' for k in range(5)))
    jw.set('intLanes', jw.get('intLanes') + f' :{WIDEN_J}_0_4')
    reqs = jw.findall('request')
    for r in reqs:
        r.set('response', '0' + r.get('response'))
        r.set('foes', '0' + r.get('foes'))
    ET.SubElement(jw, 'request', {'index': str(len(reqs)), 'response': '0' * (len(reqs) + 1),
                                  'foes': '0' * (len(reqs) + 1), 'cont': '0'})
    iw = next(e for e in root.findall('edge') if e.get('id') == f':{WIDEN_J}_0')
    il3 = next(l for l in iw.findall('lane') if l.get('index') == '3')
    il4 = ET.SubElement(iw, 'lane', dict(il3.attrib))
    a, b = lane4[-1], p4[0]
    if math.dist(a, b) < 1e-3:
        b = (a[0] + 1e-3, a[1])
    il4.set('id', f':{WIDEN_J}_0_4')
    il4.set('index', '4')
    il4.set('shape', _write([a, b]))
    il4.set('length', f'{max(math.dist(a, b), 0.001):.6f}')
    cons = root.findall('connection')
    last = list(root).index(cons[-1]) + 1
    c3 = next(c for c in cons if c.get('from') == POCKET_E and c.get('fromLane') == '3')
    c = ET.Element('connection', dict(c3.attrib))
    c.set('fromLane', '4')
    c.set('toLane', '4')
    c.set('via', f':{WIDEN_J}_0_4')
    root.insert(last, c)
    c3i = next(c for c in cons if c.get('from') == f':{WIDEN_J}_0' and c.get('fromLane') == '3')
    c2 = ET.Element('connection', dict(c3i.attrib))
    c2.set('fromLane', '4')
    c2.set('toLane', '4')
    root.insert(last + 1, c2)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return fr, T_END


DASH_PREFIX = ('lane_dash_SW_end_T_left_out', 'lane_dash_SW_end_T_right_in', 'SW_long_right_divider')


def _redraw_dashes(root, fr, T_END, net_path):
    """비켜 나는 구간의 차로 사이 점선을 지우고 새 차로 경계를 따라 다시 (3 m 칠·3 m 띄움)."""
    from env.roundabout_builder import _read_lane_shapes
    from env.roundabout_geometry import _offset, _slice
    n_rm = 0
    for p in list(root.findall('poly')):
        if not p.get('id', '').startswith(DASH_PREFIX):
            continue
        ts = [fr.tl(q)[0] for q in _read(p.get('shape'))]
        if min(ts) > -0.5 and max(ts) < T_END + 3.5:
            root.remove(p)
            n_rm += 1
    sh = _read_lane_shapes(net_path)
    lane = {k: sh[(TAPER_E, k)] + sh[(POCKET_E, k)][1:] for k in range(4)}
    lines = [_offset(lane[k], LANE_W / 2) for k in range(3)]                 # 0|1, 1|2, 2|3
    lines.append(_offset(lane[0], -LANE_W / 2))                              # 0 | 우회전 도로
    n = 0
    for i, ln in enumerate(lines):
        L = _len(ln)
        st = 0.6
        while st + 3.0 <= L + 3.0 - 0.3:              # 마지막 토막은 다음 엣지로 넘어가는 자리까지
            if st + 3.0 > L:
                break
            ET.SubElement(root, 'poly', {'id': f'swpocket_lanedash_{i}_{n}', 'color': '255,255,255', 'fill': '0',
                                         'lineWidth': '0.15', 'layer': '6', 'shape': _write(_slice(ln, st, st + 3.0))})
            n += 1
            st += 6.0
    return n_rm, n


def apply_deco(deco_path, fr, T_END, net_path=None):
    from shapely.geometry import LineString, Polygon
    tree = ET.parse(deco_path)
    root = tree.getroot()
    # 1) 오른쪽 무리 노면 그림(차로 점선·바닥·우회전 도로 점선): 새 비켜 나기만큼 옮긴다
    wedge_ids = ('SW_end_T_extension_pavement', 'SW_end_T_extension_border', 'SW_end_T_extension_opening_pavement',
                 'SW_end_T_extension_hatch_', 'swpocket_')
    n_moved = 0
    for p in list(root.findall('poly')):
        pid = p.get('id', '')
        if pid.startswith(wedge_ids):
            root.remove(p)
            continue
        try:
            pts = _read(p.get('shape'))
        except Exception:
            continue
        if not any(0 < fr.tl(q)[0] < T_END for q in pts):
            continue
        dense = _densify(pts, 1.0) if len(pts) <= 4 and _len(pts) > 2.0 and p.get('fill') == '0' else pts
        new = [fr.move(q) for q in dense]
        if new != dense:
            p.set('shape', _write(new))
            n_moved += 1
    n_rm, n_ld = _redraw_dashes(root, fr, T_END, net_path)
    # 2) 새 안전지대: 중앙선 ~ (3번 차로 왼쪽 경계, 그 뒤 4번 차로 왼쪽 경계)
    tt = [i * 0.5 for i in range(int((TAPER + OPEN) / 0.5) + 1)]
    if OPEN <= 0:
        tt[-1] = TAPER - 0.05

    def lower(t):        # 안전지대 아래(오른쪽) 경계 l
        if t <= TAPER or OPEN <= 0:
            return CENTER_L + fr.s_new(t)
        return CENTER_L + LANE_W * (1 - _smooth((t - TAPER) / OPEN))

    top = [fr.xy(t, CENTER_L) for t in tt]
    bot = [fr.xy(t, lower(t)) for t in tt]
    wedge = top + bot[::-1]
    ET.SubElement(root, 'poly', {'id': 'swpocket_pavement', 'color': '84,84,90', 'fill': '1', 'layer': '4.9',
                                 'shape': _write(wedge)})
    ET.SubElement(root, 'poly', {'id': 'swpocket_border', 'color': '235,190,40', 'fill': '0', 'lineWidth': '0.15',
                                 'layer': '6', 'shape': _write(bot)})
    inner = Polygon([fr.tl(q) for q in wedge]).buffer(-0.2)
    n_h = 0
    t = 2.0
    while t < TAPER + OPEN:
        seg = LineString([(t, CENTER_L), (t + 3.0, CENTER_L + 3.0)]).intersection(inner)
        for g in getattr(seg, 'geoms', [seg]):
            if g.geom_type == 'LineString' and g.length > 0.4:
                ET.SubElement(root, 'poly', {'id': f'swpocket_hatch_{n_h}', 'color': '235,190,40', 'fill': '0',
                                             'lineWidth': '0.2', 'layer': '6',
                                             'shape': _write([fr.xy(*c) for c in g.coords])})
                n_h += 1
        t += 3.0
    # 3) 3·4번 차로 사이 흰 점선 (좌회전 차로가 다 열린 뒤부터 taper 끝까지)
    n_d = 0
    t = TAPER + OPEN + 1.0
    while t + 3.0 <= T_END - 0.5:
        ET.SubElement(root, 'poly', {'id': f'swpocket_dash_{n_d}', 'color': '255,255,255', 'fill': '0',
                                     'lineWidth': '0.15', 'layer': '6',
                                     'shape': _write([fr.xy(t, CENTER_L + LANE_W), fr.xy(t + 3.0, CENTER_L + LANE_W)])})
        n_d += 1
        t += 6.0
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'sw_pocket_moved_polys': n_moved, 'sw_pocket_hatch': n_h, 'sw_pocket_dashes': n_d,
            'sw_pocket_lane_dashes': n_ld, 'sw_pocket_old_dashes_removed': n_rm}


def add_arrow(deco_path, net_path):
    """좌회전 차로(POCKET_E 4번)에 좌회전 화살표."""
    from env.roundabout_builder import _arrow, _read_lane_shapes
    tree = ET.parse(deco_path)
    root = tree.getroot()
    marks = []
    _arrow(marks, 'swpocket_arrow_left', _read_lane_shapes(net_path), POCKET_E, 4, 'left', 6.0)
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)


def apply(net_path, deco_path):
    fr, T_END = apply_net(net_path)
    info = apply_deco(deco_path, fr, T_END, net_path)
    add_arrow(deco_path, net_path)
    pocket = T_END - (TAPER + OPEN)
    info['sw_pocket_left_lane_m'] = round(pocket + 16.0 + 7.6, 1)
    return info
