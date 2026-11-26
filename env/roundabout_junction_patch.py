"""이미 손질된 교차점 몇 개만 다시 만들어 그 사이 길의 차로 수를 바꾼다 (2026-11-14, 일반화 버전).

  env.roundabout_majeon_two_lane.widen_stub 과 같은 방식을 여러 교차점·여러 엣지로 넓힌 것:
  - 교차점 JS 와 거기 붙은 엣지만으로 작은 망을 netconvert 로 만든다 (JS 끼리 잇는 엣지는 통째, 나머지는 ARM m 만).
  - 넓히는 엣지(WIDEN): 차로 수만 늘리고, 안쪽(중앙선 쪽) 차로 위치는 그대로 → 바깥쪽에 차로가 붙는다.
  - 넓히는 엣지가 낀 연결은 CON 으로 직접 준다. 나머지 연결은 예전 그대로.
  - 넓힌 엣지와 상관없는 움직임은 예전 회전 경로를 그대로 쓰고, 넓힌 엣지가 낀 경로만 새 것
    (넓히지 않은 큰길 쪽 끝은 예전 차로 끝에 맞춰 휜다). 교차점 바닥 = 예전 바닥 ∪ (새 바닥 중 바뀐 경로 근처).
  - 넓히지 않은 엣지의 차로 모양은 건드리지 않는다.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

LANE_W = 3.2
ARM = 45.0


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def patch(net_path, JS, WIDEN, CON):
    """JS: 다시 만들 교차점들, WIDEN: {엣지: 새 차로 수}, CON: [(from, fromLane, to, toLane)] 넓힌 엣지가 낀 연결 전부."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    from env.roundabout_interior import _edge_line, _length, _slice
    from env.roundabout_geometry import _offset
    tree = ET.parse(net_path)
    root = tree.getroot()
    JS = list(JS)
    jx = {j.get('id'): j for j in root.findall('junction')}
    xy = {k: (float(j.get('x')), float(j.get('y'))) for k, j in jx.items()}
    edges = {e.get('id'): e for e in root.findall('edge') if e.get('function') != 'internal'}
    for eid, n in WIDEN.items():
        if len(edges[eid].findall('lane')) >= n:
            raise RuntimeError(f'{eid} 는 이미 {n}차로 이상')
    prefs = tuple(f':{j}_' for j in JS)
    arms = [e for e in edges.values() if e.get('from') in JS or e.get('to') in JS]
    arm_ids = {e.get('id') for e in arms}
    old_con = [dict(c.attrib) for c in root.findall('connection') if c.get('via', '').startswith(prefs)]

    with tempfile.TemporaryDirectory() as work:
        nod = {j: (xy[j], jx[j].get('type'), jx[j].get('radius')) for j in JS}
        edg = []
        for e in arms:
            eid = e.get('id')
            line = _edge_line(e, xy)
            lanes = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
            n = len(lanes)
            spread = e.get('spreadType', 'right')
            if eid in WIDEN:
                add = WIDEN[eid] - n
                if spread == 'center':                 # 가운데 기준 → 바깥(오른쪽)으로 반 차로씩 밀어 안쪽 차로 고정
                    line = _offset(line, -add * LANE_W / 2)
                n = WIDEN[eid]
            fr, to = e.get('from'), e.get('to')
            if fr not in JS or to not in JS:
                L = _length(line)
                if fr in JS:
                    line = _slice(line, 0.0, min(ARM, L))
                    to = f'X_{eid}'
                    nod[to] = (line[-1], 'dead_end', None)
                else:
                    line = _slice(line, max(0.0, L - ARM), L)
                    fr = f'X_{eid}'
                    nod[fr] = (line[0], 'dead_end', None)
            edg.append((eid, fr, to, n, lanes[0].get('speed'), e.get('priority', '1'), lanes[0].get('width', '3.2'),
                        spread, line))
        # 교차로 안 대기 지점에서 이어지는 연결(from 이 ':' 로 시작)은 netconvert 가 다시 만든다 (2026-11-15)
        cons = [(c['from'], c['fromLane'], c['to'], c['toLane'], c) for c in old_con
                if c['from'] not in WIDEN and c['to'] not in WIDEN and not c['from'].startswith(':')]
        cons += [(f, str(fl), t, str(tl), {}) for f, fl, t, tl in CON]
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
            for f, fl, t, tl, c in cons:
                extra = ''.join(f' {k}="{c[k]}"' for k in ('contPos', 'pass', 'keepClear', 'visibility', 'speed')
                                if k in c)
                fh.write(f'  <connection from="{f}" to="{t}" fromLane="{fl}" toLane="{tl}"{extra}/>\n')
            fh.write('</connections>\n')
        from sumolib import checkBinary
        r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                            '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                            '--junctions.corner-detail', '24', '--junctions.internal-link-detail', '12',
                            '--precision', '6', '-o', 'p.net.xml'], cwd=work, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        sub = ET.parse(os.path.join(work, 'p.net.xml')).getroot()
    # 좌회전 대기 지점(내부 교차점, ':J_n_0')이 생기면 그대로 옮겨 심는다 (2026-11-15).
    # 대기 지점이 있는 움직임은 앞·뒤 토막 모두 새 경로를 쓴다 (예전 경로와 대기 지점 위치가 다를 수 있어서).
    internal_js = [j for j in sub.findall('junction') if j.get('id', '').startswith(prefs)]

    old_lane = {l.get('id'): _read(l.get('shape')) for e in root.findall('edge') if e.get('id', '').startswith(prefs)
                for l in e.findall('lane')}
    old_move = {(c['from'], c['fromLane'], c['to'], c['toLane']): old_lane[c['via']] for c in old_con}
    lane_xy = {l.get('id'): _read(l.get('shape')) for e in edges.values() for l in e.findall('lane')}
    sub_lane = {l.get('id'): l for e in sub.findall('edge') for l in e.findall('lane')}
    changed = {j: [] for j in JS}
    sub_con = [c for c in sub.findall('connection') if c.get('via', '').startswith(prefs)]
    split_from = {c.get('from') for c in sub_con if c.get('from', '').startswith(':')}   # 대기 지점 뒤 토막의 앞 토막 (엣지 이름)
    for c in sub_con:
        f, t = c.get('from'), c.get('to')
        key = (f, c.get('fromLane'), t, c.get('toLane'))
        ln = sub_lane[c.get('via')]
        j = next(j for j in JS if c.get('via').startswith(f':{j}_'))
        via_edge = c.get('via').rsplit('_', 1)[0]
        if f.startswith(':'):                           # 대기 지점 뒤 토막: 새 경로, 도착 쪽 끝만 맞춘다
            q = _read(ln.get('shape'))
            if t not in WIDEN and len(q) >= 2:
                L = _length(q) or 1.0
                cum = [0.0]
                for u, v in zip(q, q[1:]):
                    cum.append(cum[-1] + math.dist(u, v))
                b0 = lane_xy[f'{t}_{c.get("toLane")}'][0]
                d = (b0[0] - q[-1][0], b0[1] - q[-1][1])
                q = [(x + d[0]*s/L, y + d[1]*s/L) for (x, y), s in zip(q, cum)]
            changed[j].append(q)
        elif f not in WIDEN and t not in WIDEN and key in old_move and via_edge not in split_from:
            q = old_move[key]
        else:
            q = _read(ln.get('shape'))
            L = _length(q)
            if L < 0.05:                                # 길이 0 인 이음 경로: 양 끝 차로 끝을 곧게 잇는다
                a0 = lane_xy[f'{f}_{c.get("fromLane")}'][-1] if f not in WIDEN else q[0]
                b0 = lane_xy[f'{t}_{c.get("toLane")}'][0] if t not in WIDEN else q[-1]
                q = [a0, b0] if math.dist(a0, b0) > 1e-3 else [a0, (a0[0] + 1e-3, a0[1])]
                changed[j].append(q)
                ln.set('shape', _write(q))
                ln.set('length', f'{max(_length(q), 0.1):.6f}')
                continue
            cum = [0.0]
            for u, v in zip(q, q[1:]):
                cum.append(cum[-1] + math.dist(u, v))
            if f not in WIDEN:
                a0 = lane_xy[f'{f}_{c.get("fromLane")}'][-1]
                d = (a0[0] - q[0][0], a0[1] - q[0][1])
                q = [(x + d[0]*(1 - s/L), y + d[1]*(1 - s/L)) for (x, y), s in zip(q, cum)]
            if t not in WIDEN and via_edge not in split_from:   # 대기 지점에서 끊긴 앞 토막은 끝을 그대로
                b0 = lane_xy[f'{t}_{c.get("toLane")}'][0]
                d = (b0[0] - q[-1][0], b0[1] - q[-1][1])
                q = [(x + d[0]*s/L, y + d[1]*s/L) for (x, y), s in zip(q, cum)]
            changed[j].append(q)
        ln.set('shape', _write(q))
        ln.set('length', f'{_length(q):.6f}')
    sub_j = {j.get('id'): j for j in sub.findall('junction')}
    floors = {}
    for j in JS:
        ends = []
        for eid in WIDEN:
            for l in sub_edges_lanes(sub, eid):
                p = _read(l.get('shape'))
                if sub_edge(sub, eid).get('to') == j:
                    ends.append(p[-1])
                if sub_edge(sub, eid).get('from') == j:
                    ends.append(p[0])
        parts = [LineString(p).buffer(LANE_W/2 + 0.2) for p in changed[j]]
        if len(ends) >= 3:
            parts.append(Polygon(ends).convex_hull.buffer(6.0))
        elif ends:
            parts.append(LineString(ends + ends[:1]).buffer(6.0))
        old = Polygon(_read(jx[j].get('shape'))).buffer(0) if jx[j].get('shape') else Polygon()
        new = Polygon(_read(sub_j[j].get('shape'))).buffer(0)
        fl = unary_union([old, new.intersection(unary_union(parts))]) if parts else old
        if fl.geom_type != 'Polygon':
            fl = max(fl.geoms, key=lambda g: g.area)
        floors[j] = list(fl.simplify(0.02).exterior.coords)

    # 바꿔 끼우기
    for j in list(root.findall('junction')):
        if j.get('id') in JS or j.get('id', '').startswith(prefs):
            root.remove(j)
    for e in list(root.findall('edge')):
        if e.get('id', '').startswith(prefs):
            root.remove(e)
    for c in list(root.findall('connection')):
        if c.get('from', '').startswith(prefs) or c.get('via', '').startswith(prefs) or \
                (c.get('from') in WIDEN and c.get('to') in arm_ids and edges[c.get('from')].get('to') in JS):
            root.remove(c)
    sub_e = {e.get('id'): e for e in sub.findall('edge')}
    for eid in WIDEN:
        e, se = edges[eid], sub_e[eid]
        for l in list(e.findall('lane')):
            e.remove(l)
        for k, v in se.attrib.items():
            if k not in ('from', 'to'):
                e.set(k, v)
        for l in se.findall('lane'):
            e.append(l)
    last = [i for i, c in enumerate(list(root)) if c.tag == 'edge'][-1] + 1
    for e in sub.findall('edge'):
        if e.get('id', '').startswith(prefs):
            root.insert(last, e)
            last += 1
    last = [i for i, c in enumerate(list(root)) if c.tag == 'junction'][-1] + 1
    for j in sub.findall('junction'):
        if j.get('id') in JS:
            j.set('shape', _write(floors[j.get('id')]))
            root.insert(last, j)
            last += 1
    for j in internal_js:
        root.insert(last, j)
        last += 1
    last = [i for i, c in enumerate(list(root)) if c.tag == 'connection'][-1] + 1
    for c in sub.findall('connection'):
        if c.get('from', '').startswith(prefs) or (c.get('from') in arm_ids and c.get('via', '').startswith(prefs)):
            root.insert(last, c)
            last += 1
    # 넓힌 엣지의 반대쪽 끝이 JS 밖 교차점이면 그 교차점 incLanes 도 맞춘다 (여기 쓰는 경우는 모두 JS 끼리라 확인만)
    for eid in WIDEN:
        e = edges[eid]
        if e.get('from') not in JS or e.get('to') not in JS:
            raise RuntimeError(f'{eid}: 넓히는 엣지는 양 끝이 모두 다시 만드는 교차점이어야 한다')
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'patched_junctions': JS, 'patched_links': len(sub_con), 'patched_new_paths': sum(map(len, changed.values()))}


def sub_edge(sub, eid):
    return next(e for e in sub.findall('edge') if e.get('id') == eid)


def sub_edges_lanes(sub, eid):
    return sub_edge(sub, eid).findall('lane')
