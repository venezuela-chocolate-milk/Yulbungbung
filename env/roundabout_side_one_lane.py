"""검단로445번길(종다리공원 서쪽 옆길, T_end_side_out/in)을 방향별 1차로로 줄인다. (2026-11-11 사용자 요청)

  - 남기는 차로: 중앙선 쪽(안쪽) 차로 자리. 위쪽 445번길 연장(nw_445b, 방향별 1차로)과 일직선으로 이어진다.
  - 아래쪽 T자(T_main_end)는 손으로 다듬은 교차로라 다시 만들지 않는다. 연결(링크) 수·순서·우선순위표는 그대로 두고
      옆길 진입: 바깥 차로 좌회전(:T_main_end_6_0) → 남은 차로에서 출발 (시작점만 안쪽 차로 끝으로 옮김)
                 안쪽 차로 우회전(:T_main_end_7_0) → 남은 차로(0번)에서 출발 (모양 그대로)
      옆길 진출: 큰길 우회전(:T_main_end_0_0) → 남은 차로로 도착 (끝점만 안쪽 차로 시작으로 옮김)
    내부 경로는 옮긴 끝에서 반대쪽 끝으로 갈수록 0 이 되게 부드럽게 옮긴다.
  - T자 바닥면과 아스팔트 바탕 그림: 빠진 바깥 차로 자리(띠)만큼 잘라낸다.
  - 위쪽 끝(T_end_side_end)은 다음 단계 env.roundabout_nw_shops 가 1차로 기준으로 다시 만든다.
  - 노면: 옆길 차로 사이 흰 점선을 지운다 (노란 중앙선은 그대로).
    python: apply(net_path, deco_path)
roundabout_south_extend 가 옆길 평행이동(roundabout_slide_side) 다음, 북서쪽 상가 골목 전에 호출한다.
"""
import math
import xml.etree.ElementTree as ET

EDGES = ('T_end_side_out', 'T_end_side_in')
MAIN_EDGES = ('T_end_main_in', 'T_end_main_near', 'T_end_main_out', 'T_main_in', 'T_main_out')
CORNER_R = 5.0                  # 옆길 입구 모서리 곡선 반지름 (m)
T = 'T_main_end'
LANE_W = 3.2


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def _blend(pts, delta, at_start):
    """polyline 한쪽 끝을 delta 만큼 옮기고 반대쪽 끝으로 갈수록 0 이 되게 (길이 비율, 부드럽게)."""
    c = [0.0]
    for a, b in zip(pts, pts[1:]):
        c.append(c[-1] + math.dist(a, b))
    L = c[-1] or 1.0
    out = []
    for p, s in zip(pts, c):
        f = 1 - s/L if at_start else s/L
        f = f*f*(3 - 2*f)
        out.append((p[0] + delta[0]*f, p[1] + delta[1]*f))
    return out


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    if len(edges['T_end_side_in'].findall('lane')) == 1:
        return {'side_one_lane': 'already'}, None
    lanes = {l.get('id'): l for e in root.findall('edge') for l in e.findall('lane')}
    old = {lid: _read(l.get('shape')) for lid, l in lanes.items() if lid.startswith(EDGES)}
    # 1) 엣지: 바깥(0번) 차로를 빼고 안쪽(1번) 차로를 0번으로
    for eid in EDGES:
        e = edges[eid]
        l0, l1 = e.findall('lane')
        e.remove(l0)
        l1.set('id', f'{eid}_0')
        l1.set('index', '0')
        e.set('shape', l1.get('shape'))
    # 2) 아래쪽 T자 내부 경로
    out_new, out_old = old['T_end_side_out_1'][0], old['T_end_side_out_0'][0]
    in_new, in_old = old['T_end_side_in_1'][-1], old['T_end_side_in_0'][-1]
    moved = {}
    for c in root.findall('connection'):
        f, t = c.get('from'), c.get('to')
        if f == 'T_end_side_in':
            if c.get('fromLane') == '0' and c.get('via'):              # 바깥 차로 좌회전 → 안쪽 차로 끝에서 출발
                lid = c.get('via')
                pts = _read(lanes[lid].get('shape'))
                pts = _blend(pts, (in_new[0] - in_old[0], in_new[1] - in_old[1]), at_start=True)
                lanes[lid].set('shape', _write(pts))
                lanes[lid].set('length', f'{_length(pts):.6f}')
                moved[lid] = 'start'
            c.set('fromLane', '0')
        if t == 'T_end_side_out':
            if c.get('via') and not f.startswith(':'):                 # 큰길 우회전 → 안쪽 차로 시작으로 도착
                lid = c.get('via')
                pts = _read(lanes[lid].get('shape'))
                pts = _blend(pts, (out_new[0] - out_old[0], out_new[1] - out_old[1]), at_start=False)
                lanes[lid].set('shape', _write(pts))
                lanes[lid].set('length', f'{_length(pts):.6f}')
                moved[lid] = 'end'
            c.set('toLane', '0')
        if f == 'T_end_side_out':                                      # (위쪽 끝은 아직 막다른 끝이라 보통 없음)
            c.set('fromLane', '0')
    # 위쪽 끝(T_end_side_end)으로 사라진 차로에 들어가던 연결 정리 (nw_shops 가 그 교차점을 다시 만든다)
    for c in list(root.findall('connection')):
        if c.get('to') == 'T_end_side_in' and c.get('toLane') != '0':
            root.remove(c)
    # 3) 교차점 incLanes / intLanes 정리
    for j in root.findall('junction'):
        inc = j.get('incLanes', '').split()
        if 'T_end_side_in_1' in inc or 'T_end_side_out_1' in inc:
            inc = [x for x in inc if x not in ('T_end_side_in_0', 'T_end_side_out_0')]
            inc = [x.replace('T_end_side_in_1', 'T_end_side_in_0').replace('T_end_side_out_1', 'T_end_side_out_0')
                   for x in inc]
            j.set('incLanes', ' '.join(inc))
    # 4) T자 바닥면: 옆길 입구를 새 폭으로 다시 만든다. 큰길 + 새 옆길 띠를 합쳐 반지름 CORNER_R 로
    #    오목한 모서리를 둥글게 채운 모양(모서리 곡선)을 입구 구역에만 쓴다.
    strips = _strips(old)
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    jt = next(j for j in root.findall('junction') if j.get('id') == T)
    jpoly = Polygon(_read(jt.get('shape'))).buffer(0)
    o_new = _read(lanes['T_end_side_out_1'].get('shape'))           # (이름은 아직 바꾸기 전 객체 = 남은 차로)
    i_new = _read(lanes['T_end_side_in_1'].get('shape'))
    m0 = ((o_new[0][0] + i_new[-1][0])/2, (o_new[0][1] + i_new[-1][1])/2)
    u = (o_new[1][0] - o_new[0][0], o_new[1][1] - o_new[0][1])
    L = math.hypot(*u)
    u = (u[0]/L, u[1]/L)                                           # 옆길 위쪽 방향
    nv = (-u[1], u[0])
    hw = math.dist(o_new[0], i_new[-1])/2 + LANE_W/2                # 새 옆길 반폭
    # 큰길 바닥: 큰길 엣지 차로 + T자 안의 큰길 직진 내부 차로 (옆길과 오가는 회전 경로는 빼고)
    side_vias = {c.get('via') for c in root.findall('connection')
                 if c.get('via') and ('T_end_side' in c.get('from') or 'T_end_side' in c.get('to'))}
    main_lanes = [l for eid in MAIN_EDGES if eid in edges for l in edges[eid].findall('lane')]
    main_lanes += [l for e in root.findall('edge') if e.get('function') == 'internal' and e.get('id', '').startswith(f':{T}_')
                   for l in e.findall('lane') if l.get('id') not in side_vias]
    main = unary_union([Polygon(_lane_poly(_read(l.get('shape')), float(l.get('width', LANE_W))))
                        for l in main_lanes if len(_read(l.get('shape'))) >= 2])
    d0 = next((t*0.25 for t in range(400) if main.contains(Point(m0[0] - u[0]*t*0.25, m0[1] - u[1]*t*0.25))), 20.0)
    P = lambda s_, l_: (m0[0] + u[0]*s_ + nv[0]*l_, m0[1] + u[1]*s_ + nv[1]*l_)
    W = hw + CORNER_R + 9.0
    zone = Polygon([P(-d0 - 0.3, -W), P(1.0, -W), P(1.0, W), P(-d0 - 0.3, W)])
    band = Polygon([P(-d0 - 1.5, -hw), P(1.0, -hw), P(1.0, hw), P(-d0 - 1.5, hw)])
    keep = jpoly.difference(zone)
    closed = unary_union([keep, main, band]).buffer(CORNER_R, quad_segs=16).buffer(-CORNER_R, quad_segs=16)
    g = unary_union([keep, closed.intersection(zone)]).buffer(0)
    g = max(getattr(g, 'geoms', [g]), key=lambda x: x.area)
    jt.set('shape', _write(list(g.exterior.coords)[:-1]))
    cut = zone.difference(closed)                                    # 입구 구역에서 길이 아닌 곳 (노면 바탕 그림도 잘라냄)
    strips = unary_union([strips.difference(zone), cut])
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'side_one_lane': 'done', 'side_internal_moved': moved}, strips


def _lane_poly(pts, w):
    """차로 중심선 → 폭 w 사각 띠 다각형."""
    from shapely.geometry import LineString
    return LineString(pts).buffer(w/2, cap_style=2).exterior.coords


def _strips(old):
    """빠진 바깥 차로 자리: 옛 0번 차로 모양을 아래쪽 T자 쪽으로 4m 늘려 차로 폭만큼 두껍게."""
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    out = []
    for lid, at_start in (('T_end_side_out_0', True), ('T_end_side_in_0', False)):
        pts = list(old[lid])
        a, b = (pts[1], pts[0]) if at_start else (pts[-2], pts[-1])
        L = math.dist(a, b)
        ext = (b[0] + (b[0]-a[0])/L*4.0, b[1] + (b[1]-a[1])/L*4.0)
        pts = [ext] + pts if at_start else pts + [ext]
        out.append(LineString(pts).buffer(LANE_W/2 + 0.05, cap_style=2))
    return unary_union(out)


def apply(net_path, deco_path):
    info, strips = apply_net(net_path)
    if isinstance(info, dict) and info.get('side_one_lane') == 'already':
        return info
    info.update(apply_deco(deco_path, strips))
    return info


def apply_deco(deco_path, strips):
    """차로 사이 흰 점선을 지우고, 아스팔트 바탕 그림을 빠진 차로 자리만큼 잘라낸다."""
    from shapely.geometry import Polygon
    tree = ET.parse(deco_path)
    root = tree.getroot()
    n = clipped = 0
    for p in list(root.findall('poly')):
        pid = p.get('id', '')
        if pid.startswith(('lane_dash_T_end_side_in', 'lane_dash_T_end_side_out')):
            root.remove(p)
            n += 1
            continue
        if p.get('fill') == '1' and p.get('color', '').startswith('84,84,90') and pid != 'ground':
            pts = _read(p.get('shape'))
            if len(pts) < 3:
                continue
            g = Polygon(pts).buffer(0)
            if not g.intersects(strips):
                continue
            rest = g.difference(strips)
            idx = list(root).index(p)
            root.remove(p)
            for k, part in enumerate(x for x in getattr(rest, 'geoms', [rest]) if x.area > 0.5):
                el = ET.Element('poly', dict(p.attrib))
                el.set('id', pid if k == 0 else f'{pid}_s1c{k}')
                el.set('shape', _write(list(part.exterior.coords)))
                root.insert(idx + k, el)
            clipped += 1
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'side_dashes_removed': n, 'side_asphalt_clipped': clipped}
