"""남쪽 끝 두 도로(S_out0_far, S_in1_far)를 방향별 3차선, 총 6차선 도로로 연장한다.

설계 원칙
  - 모든 차로 폭은 3.2m 로 동일하다(테이퍼 구간 노면은 차로가 아닌 장식 포장).
  - 기존 2차로의 바깥 두 차로는 접속점에서 그대로 이어지고(차로 정렬 일치),
    추가되는 3번째 차로는 중앙(안쪽) 쪽에 생긴다.
  - 추가 차로 앞쪽의 포장은 quintic smoothstep 테이퍼로 부드럽게 넓어진다.
  - 연장 도로는 같은 곡선으로 안쪽으로 0.77m 정도 옮겨가며 두 방향이 맞닿고,
    맞닿는 지점부터 황색 복선 중앙선이 이어진다. 가장자리는 모두 C2 연속이라 꺾임이 없다.

빌더 전체 재생성 없이 현재 net/deco 에 이 구간만 덧붙인다:
    python -m env.roundabout_south_six_lane
원본은 reports/south-road-before/ 에 보관되어 있고, 스크립트는 항상 그 원본에서 다시 만든다.
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
BEFORE_DIR = os.path.join(BASE, 'reports', 'south-road-before')

LANE_W = 3.2
LANES = 3
TAPER = 40.0        # 추가 차로 앞 포장 테이퍼 길이 (1:12.5)
SHIFT = 60.0        # 두 방향이 맞닿을 때까지의 완만한 이동 구간
LENGTH = 180.0      # 연장 길이
SPEED = '16.670000'
CENTER_GAP = 0.3    # 황색 복선 간격
YELLOW = '235,190,40'
WHITE = '255,255,255'
ASPHALT = '84,84,90'

OUT_EDGE, IN_EDGE = 'S_out0_far', 'S_in1_far'
OUT_NODE, IN_NODE = 'S_end0', 'S_end1'
OUT_EXT, IN_EXT = 'S_out0_ext', 'S_in1_ext'
OUT_FAR, IN_FAR = 'S_ext_end0', 'S_ext_end1'
# 남행 안전지대 + 가장 안쪽 차로 추가 (None 이면 사용 안 함)
#   from_end: 도로 끝에서 안전지대 시작까지 거리
#   length  : 안전지대가 넓어지는 길이 (남행 3개 차로가 바깥으로 한 차로 밀려남)
#   nose    : 안전지대 끝을 가로선 대신 접선 연속으로 좁혀 닫는 길이.
#             그 옆으로 가장 안쪽 새 차로가 열리고, 새 차로는 nose 끝부터 주행 가능.
#   (지도의 기존 x_se_end 안전지대와 같은 구조: 넓어짐 40m + 코 14m)
#   outer_from_end / outer_taper (선택): 도로 끝에서 outer_from_end 앞부터 남행 가장 바깥에
#             차로 하나 더(→5차로). 그 앞 outer_taper 동안 노면이 부드럽게 넓어진다.
BAY = None
OUT_BAY_NODE, OUT_EXT4 = 'S_bay0', 'S_out0_ext4'
OUT_BAY5_NODE, OUT_EXT5 = 'S_bay1', 'S_out0_ext5'
HATCH_ANGLE = 41.0      # 기존 안전지대 빗금과 같은 기울기(도로 축 기준)
HATCH_SPACING = 2.6


def H(t):
    t = min(1.0, max(0.0, t))
    return t*t*t*(10 - 15*t + 6*t*t)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class Frame:
    """도로 축 기준 좌표: s = 바깥(남쪽) 방향 거리, l = 동쪽(+, 진입 차로 쪽) 횡거리."""

    def __init__(self, root):
        edges = {e.get('id'): e for e in root.findall('edge')}
        junctions = {j.get('id'): j for j in root.findall('junction')}
        a, b = _read(edges[OUT_EDGE].get('shape'))[0], _read(edges[OUT_EDGE].get('shape'))[-1]
        d = math.dist(a, b)
        self.u = ((b[0]-a[0])/d, (b[1]-a[1])/d)
        self.n = (-self.u[1], self.u[0])
        p0 = tuple(float(junctions[OUT_NODE].get(k)) for k in 'xy')
        p1 = tuple(float(junctions[IN_NODE].get(k)) for k in 'xy')
        self.o = ((p0[0]+p1[0])/2, (p0[1]+p1[1])/2)
        self.s0, l0 = self.local(p0)
        self.s1, l1 = self.local(p1)
        self.c0, self.c1 = -l0, l1     # 각 도로 중심의 횡거리(양수)
        if BAY:
            self.z0 = self.s0 + LENGTH - BAY['from_end']
            self.z1 = self.z0 + BAY['length']
            self.z2 = self.z1 + BAY['nose']
            self.zo = self.s0 + LENGTH - BAY['outer_from_end'] if BAY.get('outer_from_end') else None

    def local(self, p):
        dx, dy = p[0]-self.o[0], p[1]-self.o[1]
        return dx*self.u[0]+dy*self.u[1], dx*self.n[0]+dy*self.n[1]

    def P(self, s, l):
        return (self.o[0]+s*self.u[0]+l*self.n[0], self.o[1]+s*self.u[1]+l*self.n[1])

    # 진입(+)/진출(-) 각 방향 연장 도로의 안쪽 경계 |l|
    def inner(self, side, s):
        c, s_start = (self.c1, self.s1) if side > 0 else (self.c0, self.s0)
        half = LANES*LANE_W/2
        if s < s_start:   # 테이퍼 포장 구간
            return (c - LANE_W) - LANE_W*H((s - (s_start - TAPER))/TAPER)
        if BAY and side < 0 and s >= self.z0:
            # 안전지대 구간: 남행 3개 차로가 바깥으로 한 차로 폭만큼 이동
            # z1~z2(안전지대 코): 차로는 이동한 위치 그대로 3차로, 새 차로는 코 끝(z2)부터
            if s < self.z1:
                return LANE_W*H((s - self.z0)/BAY['length'])
            return LANE_W if s < self.z2 else 0.0
        start_inner = c + LANE_W - 2*half
        return start_inner*(1 - H((s - s_start)/SHIFT))

    def lanes(self, side, s):
        if BAY and side < 0 and self.zo is not None and s >= self.zo:
            return LANES + 2
        return LANES + 1 if (BAY and side < 0 and s >= self.z2) else LANES

    def outer(self, side, s):
        return self.inner(side, s) + self.lanes(side, s)*LANE_W

    def zone(self, s):
        """안전지대 폭 (중앙선에서 이동한 남행 차로 안쪽 경계까지)."""
        if not BAY or s < self.z0 or s > self.z2:
            return 0.0
        if s < self.z1:
            return LANE_W*H((s - self.z0)/BAY['length'])
        return LANE_W*(1 - H((s - self.z1)/BAY['nose']))

    def center(self, side, s, lanes=None):
        lanes = self.lanes(side, s) if lanes is None else lanes
        return side*(self.inner(side, s) + lanes*LANE_W/2)

    def stations(self, side, a, b, step=1.0):
        n = max(1, int(math.ceil((b-a)/step)))
        return [a + (b-a)*i/n for i in range(n+1)]

    def ext_shape(self, side):
        s_start = self.s1 if side > 0 else self.s0
        if BAY and side < 0:
            ss = self.stations(side, s_start, s_start+SHIFT) + self.stations(side, self.z0, self.z2, 0.5)
            return [self.P(s, self.center(side, s, LANES)) for s in ss]
        ss = self.stations(side, s_start, s_start+SHIFT) + [s_start+LENGTH]
        pts = [self.P(s, self.center(side, s)) for s in ss]
        return pts if side < 0 else list(reversed(pts))

    def bay_shape(self):
        end = self.s0 + LENGTH if self.zo is None else self.zo
        return [self.P(s, self.center(-1, s, LANES+1)) for s in (self.z2, end)]

    def bay5_shape(self):
        return [self.P(s, self.center(-1, s, LANES+2)) for s in (self.zo, self.s0 + LENGTH)]


def _subnet(frame, net_root, work):
    edges = {e.get('id'): e for e in net_root.findall('edge')}
    out_pts = _read(edges[OUT_EDGE].get('shape'))
    in_pts = _read(edges[IN_EDGE].get('shape'))
    far0 = frame.P(frame.s0+LENGTH, frame.center(-1, frame.s0+LENGTH))
    far1 = frame.P(frame.s1+LENGTH, frame.center(1, frame.s1+LENGTH))
    nodes = [
        ('U0', out_pts[0], 'dead_end'), (OUT_NODE, out_pts[-1], 'priority'), (OUT_FAR, far0, 'dead_end'),
        ('U1', in_pts[-1], 'dead_end'), (IN_NODE, in_pts[0], 'priority'), (IN_FAR, far1, 'dead_end')]
    if BAY:
        nodes.append((OUT_BAY_NODE, frame.P(frame.z2, frame.center(-1, frame.z2, LANES+1)), 'priority'))
        if frame.zo is not None:
            nodes.append((OUT_BAY5_NODE, frame.P(frame.zo, frame.center(-1, frame.zo, LANES+2)), 'priority'))
    with open(os.path.join(work, 'p.nod.xml'), 'w') as f:
        f.write('<nodes>\n')
        for nid, (x, y), t in nodes:
            f.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}" radius="0"/>\n')
        f.write('</nodes>\n')
    with open(os.path.join(work, 'p.edg.xml'), 'w') as f:
        f.write('<edges>\n')
        spec = [(OUT_EDGE, 'U0', OUT_NODE, 2, [out_pts[0], out_pts[-1]]),
                (IN_EXT, IN_FAR, IN_NODE, LANES, frame.ext_shape(1)),
                (IN_EDGE, IN_NODE, 'U1', 2, [in_pts[0], in_pts[-1]])]
        if BAY:
            if frame.zo is None:
                spec += [(OUT_EXT, OUT_NODE, OUT_BAY_NODE, LANES, frame.ext_shape(-1)),
                         (OUT_EXT4, OUT_BAY_NODE, OUT_FAR, LANES+1, frame.bay_shape())]
            else:
                spec += [(OUT_EXT, OUT_NODE, OUT_BAY_NODE, LANES, frame.ext_shape(-1)),
                         (OUT_EXT4, OUT_BAY_NODE, OUT_BAY5_NODE, LANES+1, frame.bay_shape()),
                         (OUT_EXT5, OUT_BAY5_NODE, OUT_FAR, LANES+2, frame.bay5_shape())]
        else:
            spec += [(OUT_EXT, OUT_NODE, OUT_FAR, LANES, frame.ext_shape(-1))]
        for eid, fr, to, n, pts in spec:
            f.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{SPEED}" '
                    f'priority="1" width="{LANE_W}" spreadType="center" shape="{_write(pts)}"/>\n')
        f.write('</edges>\n')
    with open(os.path.join(work, 'p.con.xml'), 'w') as f:
        f.write('<connections>\n')
        for lane in (0, 1):
            f.write(f'  <connection from="{OUT_EDGE}" to="{OUT_EXT}" fromLane="{lane}" toLane="{lane}"/>\n')
            f.write(f'  <connection from="{IN_EXT}" to="{IN_EDGE}" fromLane="{lane}" toLane="{lane}"/>\n')
        if BAY:
            for lane in range(LANES):
                f.write(f'  <connection from="{OUT_EXT}" to="{OUT_EXT4}" fromLane="{lane}" toLane="{lane}"/>\n')
            if frame.zo is not None:
                for lane in range(LANES+1):   # 새 차로는 가장 바깥(0번), 기존 차로는 번호 +1
                    f.write(f'  <connection from="{OUT_EXT4}" to="{OUT_EXT5}" fromLane="{lane}" toLane="{lane+1}"/>\n')
        f.write('</connections>\n')
    from sumolib import checkBinary
    out = os.path.join(work, 'p.net.xml')
    r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                        '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                        '--junctions.corner-detail', '24', '--precision', '6', '-o', out],
                       cwd=work, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr)
    return ET.parse(out).getroot()


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    frame = Frame(root)
    with tempfile.TemporaryDirectory() as work:
        sub = _subnet(frame, root, work)
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}

    # 1) 기존 두 엣지: 접속점 쪽 끝점만 subnet 결과로 맞춘다 (나머지 형상 유지)
    for e in root.findall('edge'):
        if e.get('id') not in (OUT_EDGE, IN_EDGE):
            continue
        for lane in e.findall('lane'):
            new = _read(next(l for l in sub_edges[e.get('id')].findall('lane')
                             if l.get('index') == lane.get('index')).get('shape'))
            pts = _read(lane.get('shape'))
            if e.get('id') == OUT_EDGE:
                pts[-1] = new[-1]
            else:
                pts[0] = new[0]
            lane.set('shape', _write(pts))
            lane.set('length', f'{_length(pts):.6f}')

    # 2) 새 엣지 / 내부 엣지 / 교차점 / 연결 추가
    def last_index(tag, pred=lambda el: True):
        idx = [i for i, c in enumerate(list(root)) if c.tag == tag and pred(c)]
        return idx[-1]

    pref = (f':{OUT_NODE}_', f':{IN_NODE}_', f':{OUT_BAY_NODE}_', f':{OUT_BAY5_NODE}_')
    new_edges = [e for e in sub.findall('edge')
                 if e.get('id') in (OUT_EXT, IN_EXT, OUT_EXT4, OUT_EXT5) or e.get('id', '').startswith(pref)]
    pos = last_index('edge') + 1
    for e in new_edges:
        root.insert(pos, e)
        pos += 1
    for j in root.findall('junction'):
        if j.get('id') in (OUT_NODE, IN_NODE):
            root.remove(j)
    pos = last_index('junction') + 1
    for j in sub.findall('junction'):
        if j.get('id') in (OUT_NODE, IN_NODE, OUT_FAR, IN_FAR, OUT_BAY_NODE, OUT_BAY5_NODE):
            root.insert(pos, j)
            pos += 1
    pos = last_index('connection') + 1
    for c in sub.findall('connection'):
        if c.get('from') in (OUT_EDGE, IN_EXT, OUT_EXT, OUT_EXT4) or c.get('from', '').startswith(pref):
            root.insert(pos, c)
            pos += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return frame


def _lane_shapes(net_root):
    return {l.get('id'): _read(l.get('shape')) for e in net_root.findall('edge') for l in e.findall('lane')}


def _resample(pts, step=0.25):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(math.dist(a, b)/step)))
        out += [(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(1, n+1)]
    return out


def _separator(l1, l2):
    """두 이웃 차로 중심선의 가운데 선. 곡선에서 길이가 달라도 같은 길이 비율 위치끼리 짝짓는다."""
    a, b = _resample(l1), _resample(l2)
    ca, cb = [0.0], [0.0]
    for p, q in zip(a, a[1:]):
        ca.append(ca[-1]+math.dist(p, q))
    for p, q in zip(b, b[1:]):
        cb.append(cb[-1]+math.dist(p, q))
    out, j = [], 0
    for p, c in zip(a, ca):
        t = c/ca[-1]*cb[-1]
        while j < len(cb)-2 and cb[j+1] < t:
            j += 1
        f = (t-cb[j])/(cb[j+1]-cb[j]) if cb[j+1] > cb[j] else 0.0
        q = (b[j][0]+(b[j+1][0]-b[j][0])*f, b[j][1]+(b[j+1][1]-b[j][1])*f)
        out.append(((p[0]+q[0])/2, (p[1]+q[1])/2))
    return out


def apply_deco(deco_path, net_path, frame):
    from env.roundabout_markings import dash_style
    tree = ET.parse(deco_path)
    root = tree.getroot()
    net = ET.parse(net_path).getroot()
    shapes = _lane_shapes(net)
    dash, gap, _ = dash_style(20)
    period = dash + gap
    polys = []

    def poly(pid, color, pts, fill=False, layer='6', width='0.15'):
        attrs = dict(id=pid, color=color, fill='1' if fill else '0', layer=layer, shape=_write(pts))
        if not fill:
            attrs['lineWidth'] = width
        polys.append(attrs)

    # 기존 0|1 분리선 점선의 위상(축 방향 s 기준 dash 시작점)을 이어받는다
    def phase(prefix):
        starts = []
        for p in root.findall('poly'):
            if p.get('id', '').startswith(prefix):
                starts.append(min(frame.local(q)[0] for q in _read(p.get('shape'))))
        return max(starts) % period

    def separators(side, edge, nl):
        if BAY and side < 0:
            # 남행은 3차로/4차로 구간 경계에서 점선이 끊기지 않도록 축 좌표로 한 줄씩 잇는다.
            # 이동한 차로 오프셋 W(s): 안전지대 구간에서 0→3.2, 그 뒤로 3.2 유지.
            end = frame.s0 + LENGTH
            W = lambda s: frame.inner(-1, s) if s < frame.z2 else LANE_W
            ss = frame.stations(-1, frame.s0, end, 0.25)
            out = [[frame.P(s, -(W(s) + LANE_W*j)) for s in ss] for j in (2, 1)]
            # 새 안쪽 차로 경계: 안전지대가 가장 넓은 지점(노란 테두리 끝)부터 바로 이어진다
            out.append([frame.P(s, -LANE_W) for s in frame.stations(-1, frame.z1, end, 0.25)])
            if frame.zo is not None:
                # 가장 바깥 새 차로 경계: 차로가 온전한 폭이 되는 지점부터
                lead = BAY.get('outer_dash_lead', 0.0)   # 테이퍼 안쪽으로 점선을 미리 시작
                out.insert(0, [frame.P(s, -(LANES+1)*LANE_W) for s in frame.stations(-1, frame.zo - lead, end, 0.25)])
            return out
        lanes = [shapes[f'{edge}_{i}'] for i in range(nl)]
        if side > 0:
            lanes = [list(reversed(l)) for l in lanes]        # 남쪽(+s) 방향으로 통일
        return [_separator(lanes[k], lanes[k+1]) for k in range(nl-1)]

    dash_edges = [(-1, OUT_EXT, LANES, f'lane_dash_{OUT_EDGE.replace("_far", "")}_0_'),
                  (1, IN_EXT, LANES, f'lane_dash_{IN_EDGE}_0_')]
    for side, edge, nl, prefix in dash_edges:
        ph = phase(prefix)
        for k, sep in enumerate(separators(side, edge, nl)):
            st = [frame.local(p)[0] for p in sep]
            lo, hi = st[0], st[-1]
            m = math.floor((lo - ph)/period)
            j = 0
            while True:
                a0 = ph + m*period
                m += 1
                if a0 >= hi - 0.5:
                    break
                b0 = a0 + dash
                if b0 <= lo + 0.5:
                    continue
                a1, b1 = max(a0, lo), min(b0, hi)
                piece = [p for p, t in zip(sep, st) if a1 <= t <= b1]
                if len(piece) >= 2 and _length(piece) > 0.5:
                    poly(f'south6_dash_{edge}_{k}_{j}', WHITE, piece)
                    j += 1

    # 추가 차로 앞 테이퍼 포장 (양쪽)
    for side, s_start, c in ((-1, frame.s0, frame.c0), (1, frame.s1, frame.c1)):
        ss = frame.stations(side, s_start-TAPER, s_start, 0.5)
        old_inner = c - LANE_W
        curve = [frame.P(s, side*frame.inner(side, s)) for s in ss]
        edge_line = [frame.P(s, side*(old_inner+0.05)) for s in reversed(ss)]
        # 접속부 이음새까지 약간 덮는다
        cap = [frame.P(s_start+0.6, side*(frame.inner(side, s_start+0.6))),
               frame.P(s_start+0.6, side*(old_inner+0.05))]
        poly(f'south6_taper_pavement_{"in" if side > 0 else "out"}', ASPHALT,
             curve + cap + edge_line, fill=True, layer='4.9')

    # 곡선 차로 사이 미세한 틈을 메우는 하부 포장 (기존 under* 폴리곤과 같은 방식)
    for side, s_start in ((-1, frame.s0), (1, frame.s1)):
        ss = frame.stations(side, s_start-1.0, s_start+LENGTH, 1.0)
        outer = [frame.P(s, side*frame.outer(side, max(s, s_start))) for s in ss]
        inner = [frame.P(s, side*frame.inner(side, max(s, s_start))) for s in reversed(ss)]
        poly(f'south6_under_{"in" if side > 0 else "out"}', ASPHALT, outer+inner, fill=True, layer='-1')

    # 황색 복선 중앙선: 두 방향 포장이 맞닿는 지점부터
    s_lo = max(frame.s0, frame.s1)
    s_end = min(frame.s0, frame.s1) + LENGTH
    s_meet = next(s for s in frame.stations(1, s_lo, s_lo+SHIFT, 0.1)
                  if max(frame.inner(1, s), frame.inner(-1, s)) <= 0.075)
    for k, sgn in enumerate((-1, 1)):
        pts = [frame.P(s, sgn*CENTER_GAP/2) for s in frame.stations(1, s_meet, s_end, 1.0)]
        poly(f'south6_centerline_{k}', YELLOW, pts)

    if BAY:
        _safety_zone(frame, poly)
    if BAY and frame.zo is not None:
        # 가장 바깥 새 차로 앞 테이퍼 포장
        T = BAY['outer_taper']
        base = (LANES+1)*LANE_W
        ss = frame.stations(-1, frame.zo - T, frame.zo, 0.5)
        curve = [frame.P(s, -(base + LANE_W*H((s - (frame.zo - T))/T))) for s in ss]
        edge_line = [frame.P(s, -(base - 0.05)) for s in reversed(ss)]
        cap = [frame.P(frame.zo + 0.6, -(base + LANE_W)), frame.P(frame.zo + 0.6, -(base - 0.05))]
        poly('south6_outer_lane_taper', ASPHALT, curve + cap + edge_line, fill=True, layer='4.9')

    insert_at = len(root)
    for i, el in enumerate(list(root)):
        if el.get('id', '').startswith('lane_dash_'):
            insert_at = i
            break
    for attrs in reversed(polys):
        el = ET.Element('poly', attrs)
        root.insert(insert_at, el)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'dashes': sum(1 for p in polys if 'dash' in p['id']), 'centerline_start_s': s_meet - s_lo}


def _safety_zone(frame, poly):
    """남행 안전지대: 포장 + 노란 테두리 + 사선 빗금 (기존 x_se_end_safety 와 같은 스타일)."""
    from shapely.geometry import LineString, Polygon
    z0, z1 = frame.z0, frame.z2
    ss = frame.stations(-1, z0, z1, 0.5)
    # 포장: 중앙선 ~ 이동한 남행 차로 안쪽 경계
    poly('south6_bay_pavement', ASPHALT,
         [frame.P(s, 0.0) for s in ss] +
         [frame.P(s, -((LANE_W if s >= frame.z1 else frame.zone(s))+0.05)) for s in reversed(ss)],
         fill=True, layer='-1')
    # 테두리: 중앙선(복선 안쪽 선)에서 갈라져 차로 경계를 따라가다 끝에서 닫힌다
    edge_l = lambda s: -(CENTER_GAP/2 + (LANE_W - CENTER_GAP)*frame.zone(s)/LANE_W)
    border = [frame.P(s, edge_l(s)) for s in ss]
    poly('south6_bay_border', YELLOW, border, width='0.15')
    # 빗금: 테두리 안쪽만
    inside = Polygon([frame.P(s, -CENTER_GAP/2 - 0.1) for s in ss] +
                     [frame.P(s, edge_l(s) + 0.1) for s in reversed(ss)]).buffer(0)
    ang = math.atan2(frame.u[1], frame.u[0]) - math.radians(HATCH_ANGLE)
    d = (math.cos(ang), math.sin(ang))
    nrm = (-d[1], d[0])
    cx, cy = frame.P((z0+z1)/2, -LANE_W/2)
    span = (z1 - z0)
    k, i = -span, 0
    while k <= span:
        o = (cx + nrm[0]*k, cy + nrm[1]*k)
        seg = LineString([(o[0]-d[0]*span, o[1]-d[1]*span), (o[0]+d[0]*span, o[1]+d[1]*span)])
        cut = seg.intersection(inside)
        for g in getattr(cut, 'geoms', [cut]):
            if g.geom_type == 'LineString' and g.length > 0.3:
                poly(f'south6_bay_hatch_{i}', YELLOW, list(g.coords), width='0.2')
                i += 1
        k += HATCH_SPACING


def main():
    for name in ('roundabout.net.xml', 'roundabout.deco.xml'):
        shutil.copy(os.path.join(BEFORE_DIR, name), os.path.join(NET_DIR, name))
    net_path = os.path.join(NET_DIR, 'roundabout.net.xml')
    frame = apply_net(net_path)
    info = apply_deco(os.path.join(NET_DIR, 'roundabout.deco.xml'), net_path, frame)
    print({'c0': round(frame.c0, 3), 'c1': round(frame.c1, 3), **info})


if __name__ == '__main__':
    sys.path.insert(0, BASE)
    main()
