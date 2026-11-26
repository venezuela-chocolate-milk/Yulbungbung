"""마전동 골목 편도 2차로 (2026-11-14 사용자 요청: 불법 주정차 때문에 골목을 중앙선 기준 양쪽 한 차로씩 늘린다).

  - 마전동 블록 골목(env.roundabout_majeon, mj_*) 은 LANES 설정으로 편도 2차로가 된다 (여기서는 그 앞 단계만).
  - 상가 뒤 골목은 큰 사거리(SE_triangle_SW, 완정로65번길 × 완정로) 에서 짧은 토막(SW_final_left_in/out, 36m)으로
    시작한다. 이 토막도 편도 2차로로 넓혀 차로 수가 길 가운데서 바뀌지 않게 하고, 바뀌는 곳은 사거리 안
    (건너편 SW_final_right 는 편도 1차로 그대로) — [[naturalness-first]] 규칙: 차로 수 변화는 기존 교차점에서.
  - 사거리만 따로 작은 망으로 다시 만들어(netconvert) 교차점·내부 경로·연결을 바꿔 끼우고,
    붙은 큰길 차로는 교차점 경계에서만 잘라 맞춘다 (나머지 모양은 그대로).

roundabout_south_extend 가 마전동 골목(9-7) 바로 앞에서 widen_stub → 골목 → deco 순서로 부른다.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

J = 'SE_triangle_SW'
STUB_IN, STUB_OUT = 'SW_final_left_in', 'SW_final_left_out'
STUB_END = 'SW_final_left'
LANES = 2
LANE_W = 3.2
MEDIAN = 1.0            # 토막 가운데 회색 띠 (상가 뒤 골목과 같은 단면)
ARM = 45.0              # 작은 망에 넣는 큰길 길이 (교차점에서 m)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _stub_connections(old):
    """토막 차로 연결: 나가는 쪽 0차로 = 우회전·직진, 1차로 = 좌회전. 들어오는 좌회전은 1차로로, 나머지는 0차로로."""
    out = []
    for c in old:
        f, t = c['from'], c['to']
        if f == STUB_IN:
            c = dict(c, fromLane=str(LANES - 1) if c.get('dir') == 'l' else '0')
        if t == STUB_OUT:
            c = dict(c, toLane=str(LANES - 1) if c.get('dir') == 'l' else '0')
        out.append(c)
    return out


def widen_stub(net_path):
    from env.roundabout_interior import _edge_line, _length, _slice, _project, _at
    from env.roundabout_geometry import _offset
    tree = ET.parse(net_path)
    root = tree.getroot()
    jx = {j.get('id'): j for j in root.findall('junction')}
    xy = {k: (float(j.get('x')), float(j.get('y'))) for k, j in jx.items()}
    edges = {e.get('id'): e for e in root.findall('edge') if e.get('function') != 'internal'}
    if len(edges[STUB_IN].findall('lane')) >= LANES:
        raise RuntimeError('토막이 이미 넓혀짐')
    arms = [e for e in edges.values() if J in (e.get('from'), e.get('to'))]
    old_con = [dict(c.attrib) for c in root.findall('connection')
               if c.get('via', '').startswith(f':{J}_')]

    with tempfile.TemporaryDirectory() as work:
        nod = {J: (xy[J], 'priority', jx[J].get('radius'))}
        edg = []
        for e in arms:
            eid = e.get('id')
            line = _edge_line(e, xy)
            lanes = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
            n, w = len(lanes), lanes[0].get('width', '3.2')
            if eid in (STUB_IN, STUB_OUT):
                # 한 차로(중심 2.1m) → 두 차로 (가운데 띠 0.5 + 3.2 = 바깥 경계까지 줄 중심 3.7m): 오른쪽으로 1.6m
                line = _offset(line, -(LANES - 1) * LANE_W / 2)
                n = LANES
                far = STUB_END
                nod[far] = (xy[far], 'dead_end', None)
            else:
                L = _length(line)
                far = f'X_{eid}'
                if e.get('from') == J:
                    line = _slice(line, 0.0, min(ARM, L))
                    nod[far] = (line[-1], 'dead_end', None)
                else:
                    line = _slice(line, max(0.0, L - ARM), L)
                    nod[far] = (line[0], 'dead_end', None)
            fr = J if e.get('from') == J else far
            to = J if e.get('to') == J else far
            edg.append((eid, fr, to, n, lanes[0].get('speed'), e.get('priority', '1'), w,
                        e.get('spreadType', 'right'), line))
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            for k, ((x, y), t, rad) in nod.items():
                r = f' radius="{rad}"' if rad else ''
                fh.write(f'  <node id="{k}" x="{x:.6f}" y="{y:.6f}" type="{t}"{r}/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            for eid, fr, to, n, sp, pr, w, spread, pts in edg:
                fh.write(f'  <edge id="{eid}" from="{fr}" to="{to}" numLanes="{n}" speed="{sp}" priority="{pr}" '
                         f'width="{w}" spreadType="{spread}" shape="{_write(pts)}"/>\n')
            fh.write('</edges>\n')
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            for c in _stub_connections(old_con):
                extra = ''.join(f' {k}="{c[k]}"' for k in ('contPos', 'pass', 'keepClear', 'visibility', 'speed')
                                if k in c)
                fh.write(f'  <connection from="{c["from"]}" to="{c["to"]}" fromLane="{c["fromLane"]}" '
                         f'toLane="{c["toLane"]}"{extra}/>\n')
            fh.write('</connections>\n')
        from sumolib import checkBinary
        r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                            '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                            '--junctions.corner-detail', '24', '--junctions.internal-link-detail', '12',
                            '--precision', '6', '-o', 'p.net.xml'], cwd=work, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        sub = ET.parse(os.path.join(work, 'p.net.xml')).getroot()

    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}
    pref = f':{J}_'
    if any(j.get('id', '').startswith(pref) for j in sub.findall('junction')):
        raise RuntimeError('내부 교차점이 생김 (처리 안 함)')
    # 기존 사거리는 여러 단계에서 손질된 모양(바닥 다각형, 회전 경로)이라, 토막과 상관없는 움직임은
    # 예전 경로·큰길 차로 끝을 그대로 쓰고, 토막이 낀 움직임만 새 경로 (큰길 쪽 끝은 예전 차로 끝에 맞춰 휜다).
    old_lane = {}
    for e in root.findall('edge'):
        if e.get('id', '').startswith(pref):
            for l in e.findall('lane'):
                old_lane[l.get('id')] = _read(l.get('shape'))
    old_move = {(c.get('from'), c.get('fromLane'), c.get('to'), c.get('toLane')): old_lane[c.get('via')]
                for c in root.findall('connection') if c.get('via', '').startswith(pref)}
    lane_xy = {l.get('id'): _read(l.get('shape')) for e in root.findall('edge') if e.get('function') != 'internal'
               for l in e.findall('lane')}
    old_shape = Polygon(_read(jx[J].get('shape')))
    sub_lane = {l.get('id'): l for e in sub.findall('edge') for l in e.findall('lane')}
    stub_paths = []
    for c in sub.findall('connection'):
        if not c.get('via', '').startswith(pref):
            continue
        f, t = c.get('from'), c.get('to')
        key = (f, c.get('fromLane'), t, c.get('toLane'))
        ln = sub_lane[c.get('via')]
        if STUB_IN not in (f, t) and STUB_OUT not in (f, t):
            q = old_move[key]
        else:
            q = _read(ln.get('shape'))
            L = _length(q)
            cum = [0.0]
            for u, v in zip(q, q[1:]):
                cum.append(cum[-1] + math.dist(u, v))
            if f not in (STUB_IN, STUB_OUT):             # 큰길에서 들어옴: 시작점을 예전 큰길 차로 끝으로
                a0 = lane_xy[f'{f}_{c.get("fromLane")}'][-1]
                d = (a0[0] - q[0][0], a0[1] - q[0][1])
                q = [(x + d[0]*(1 - s/L), y + d[1]*(1 - s/L)) for (x, y), s in zip(q, cum)]
            if t not in (STUB_IN, STUB_OUT):             # 큰길로 나감: 끝점을 예전 큰길 차로 시작으로
                b0 = lane_xy[f'{t}_{c.get("toLane")}'][0]
                d = (b0[0] - q[-1][0], b0[1] - q[-1][1])
                q = [(x + d[0]*s/L, y + d[1]*s/L) for (x, y), s in zip(q, cum)]
            stub_paths.append(q)
        ln.set('shape', _write(q))
        ln.set('length', f'{_length(q):.6f}')
    # 바닥: 예전 모양 + 새 모양 중 토막 쪽 (토막 회전 경로·토막 끝 근처)
    stub_end = [_read(sub_lane[f'{STUB_IN}_{k}'].get('shape'))[-1] for k in range(LANES)] + \
               [_read(sub_lane[f'{STUB_OUT}_{k}'].get('shape'))[0] for k in range(LANES)]
    near = unary_union([LineString(p).buffer(LANE_W/2 + 0.2) for p in stub_paths] +
                       [Polygon(stub_end).convex_hull.buffer(6.0)])
    new_shape = Polygon(_read([j for j in sub.findall('junction') if j.get('id') == J][0].get('shape')))
    floor = unary_union([old_shape.buffer(0), new_shape.buffer(0).intersection(near)])
    if floor.geom_type != 'Polygon':
        floor = max(floor.geoms, key=lambda g: g.area)
    floor = floor.simplify(0.02)

    # 1) 사거리·토막 끝 교차점, 사거리 내부 경로, 사거리 연결 바꿔 끼우기
    for j in list(root.findall('junction')):
        if j.get('id') in (J, STUB_END) or j.get('id', '').startswith(pref):
            root.remove(j)
    for e in list(root.findall('edge')):
        if e.get('id', '').startswith(pref):
            root.remove(e)
    arm_ids = {e.get('id') for e in arms}
    for c in list(root.findall('connection')):
        if c.get('from', '').startswith(pref) or c.get('via', '').startswith(pref) or \
                c.get('from') in (STUB_IN, STUB_OUT):
            root.remove(c)
    # 2) 토막은 통째로 새 것, 다른 큰길 차로는 그대로
    for eid in (STUB_IN, STUB_OUT):
        e, se = edges[eid], sub_edges[eid]
        for l in list(e.findall('lane')):
            e.remove(l)
        for k, v in se.attrib.items():
            e.set(k, v)
        for l in se.findall('lane'):
            e.append(l)
    last = [i for i, c in enumerate(list(root)) if c.tag == 'edge'][-1] + 1
    for e in sub.findall('edge'):
        if e.get('id', '').startswith(pref):
            root.insert(last, e)
            last += 1
    last = [i for i, c in enumerate(list(root)) if c.tag == 'junction'][-1] + 1
    for j in sub.findall('junction'):
        if j.get('id') in (J, STUB_END):
            if j.get('id') == J:
                j.set('shape', _write(list(floor.exterior.coords)))
            root.insert(last, j)
            last += 1
    last = [i for i, c in enumerate(list(root)) if c.tag == 'connection'][-1] + 1
    for c in sub.findall('connection'):
        if c.get('from', '').startswith(pref) or (c.get('from') in arm_ids and c.get('via', '').startswith(pref)):
            root.insert(last, c)
            last += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'stub_lanes': LANES, 'stub_junction_links': len(old_move), 'stub_new_paths': len(stub_paths)}


def fix_deco(deco_path, net_path):
    """토막 정지선을 두 차로 폭으로(원래 자리), 화살표를 차로마다 (0차로 직진·우회전, 1차로 좌회전),
    토막에서 나가는 좌회전 유도선(SW_median_left_guide_right_*)을 새 1차로 경로에 맞춰 다시."""
    from shapely.geometry import LineString, Point
    from env.roundabout_builder import _arrow, _read_lane_shapes, _stop_line
    from env.roundabout_geometry import _slice
    from env.roundabout_markings import dash_style
    from env.roundabout_left_guides import median_boundary_path
    tree = ET.parse(deco_path)
    root = tree.getroot()
    polys = {p.get('id'): p for p in root.findall('poly')}
    shapes = _read_lane_shapes(net_path)
    lane1 = LineString(shapes[(STUB_IN, 1)])
    old_stop = _read(polys['SW_left_stop'].get('shape'))
    stop_dist = lane1.length - lane1.project(Point(old_stop[0]))
    guides = sorted((p for k, p in polys.items() if k.startswith('SW_median_left_guide_right_')),
                    key=lambda p: int(p.get('id').rsplit('_', 1)[1]))
    tip_a, tip_b = _read(guides[0].get('shape'))[0], _read(guides[-1].get('shape'))[-1]
    for p in list(root):
        if p.get('id') in ('SW_left_stop', 'SW_left_near_arrow', 'SW_left_far_arrow') or \
                p.get('id', '').startswith('SW_median_left_guide_right_'):
            root.remove(p)
    marks = []
    _stop_line(marks, 'SW_left_stop', shapes, STUB_IN, dist=stop_dist)
    _arrow(marks, 'SW_left_near_arrow', shapes, STUB_IN, 0, 'straight_right', stop_dist + 4.0)
    _arrow(marks, 'SW_left_far_arrow', shapes, STUB_IN, 1, 'left', stop_dist + 4.0)
    for m in marks:
        root.append(ET.fromstring(m))
    to = next(c.get('to') for c in ET.parse(net_path).getroot().findall('connection')
              if c.get('from') == STUB_IN and c.get('dir') == 'l')
    path = median_boundary_path(net_path, STUB_IN, to, tip_a, tip_b)
    dash, gap0, _ = dash_style(20)
    length = LineString(path).length
    count = max(2, round((length + gap0) / (dash + gap0)))
    gap = (length - count*dash) / (count - 1)
    for i in range(count):
        st = i*(dash + gap)
        ET.SubElement(root, 'poly', id=f'SW_median_left_guide_right_{i}', color='255,255,255', fill='0',
                      lineWidth='0.15', layer='6', shape=_write(_slice(path, st, st + dash)))
    # 토막(마전동 골목이 자른 조각 포함) 같은 방향 두 차로 사이 흰 점선 (마전동 골목과 같은 3m 칠 · 3m 띔)
    from env.roundabout_geometry import _offset
    n_dash = 0
    for eid in sorted({k[0] for k in shapes if k[0].startswith((STUB_IN, STUB_OUT))}):
        for k in range(LANES - 1):
            div = _offset(shapes[(eid, k)], LANE_W / 2)
            L = LineString(div).length
            st = 1.5
            while st + 3.0 <= L - 1.5:
                ET.SubElement(root, 'poly', id=f'SW_left_stub_dash_{eid}_{k}_{n_dash}', color='255,255,255', fill='0',
                              lineWidth='0.15', layer='6', shape=_write(_slice(div, st, st + 3.0)))
                n_dash += 1
                st += 6.0
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'stub_dashes': n_dash, 'stub_stop_dist': round(stop_dist, 2), 'stub_left_guides': count}
