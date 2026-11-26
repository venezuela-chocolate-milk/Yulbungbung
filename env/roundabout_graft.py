"""기존 도로망에 새 골목망을 접합하는 공용 도구.

  netconvert 로 '부분망'(새 교차점·새 골목 + 나눌 기존 엣지의 근처 조각)을 만들고, 그 결과의 교차점·내부 엣지·연결을
  원래 망에 옮겨 심는다. 원래 망의 나머지(손으로 다듬은 교차점, 노면 표시 기준 차로 모양)는 건드리지 않는다.

  splits: [(기존 엣지, [(새 교차점, 엣지 시작에서의 거리 m), ...], [조각 이름, ...])]
      조각 수 = 교차점 수 + 1. 바깥(기존) 교차점에 닿는 조각은 그 끝의 원래 차로 모양을 유지하고,
      부분망 교차점에 닿는 끝은 부분망 차로 모양으로 맞춘다.
  attach: 막다른 끝 같은 기존 교차점을 부분망 교차점으로 다시 만든다 (그 교차점에 닿는 엣지는 모두 splits 에 넣는다).
  streets: [(엣지, 시작, 끝, 차로 수, 차로 폭, 속도, 우선순위, 모양점)]
  extra_con: [(from, to, fromLane, toLane)] — splits 조각에서 나가는 회전 연결 (조각 사이 직진 연결은 자동).
      splits 조각의 연결을 직접 적으면 netconvert 가 그 엣지의 다른 연결을 만들지 않으므로 회전은 여기 적는다.
"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

STUB = 40.0


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


def project(pts, q):
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


def at(pts, s):
    c = _cum(pts)
    for i in range(len(pts)-1):
        if c[i+1] >= s:
            t = 0.0 if c[i+1] == c[i] else (s-c[i])/(c[i+1]-c[i])
            a, b = pts[i], pts[i+1]
            return (a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t)
    return pts[-1]


def slice_(pts, s0, s1):
    c = _cum(pts)
    return [at(pts, s0)] + [p for p, d in zip(pts, c) if s0 < d < s1] + [at(pts, s1)]


def graft(net_path, new_nodes, streets, splits, attach=(), extra_con=(), node_types=None, node_shapes=None):
    """node_shapes: {교차점: [좌표]} — 교차점 바닥 모양을 직접 정한다 (차로 수가 줄어드는 곳의 비스듬한 합류부 등).
    netconvert 가 엣지를 이 모양 경계에서 자르고 내부 차로로 잇는다."""
    node_shapes = node_shapes or {}
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    juncs = {j.get('id'): j for j in root.findall('junction')}
    attach = set(attach)
    node_types = node_types or {}
    nodes = dict(new_nodes)
    for a in attach:
        nodes[a] = (float(juncs[a].get('x')), float(juncs[a].get('y')))
    sub_nodes = set(nodes)
    for eid, cuts, _ in splits:
        shape = _read(edges[eid].get('shape') or edges[eid].find('lane').get('shape'))
        for j, s in cuts:
            nodes[j] = at(shape, s)
            sub_nodes.add(j)

    with tempfile.TemporaryDirectory() as work:
        nod = [(k, p, node_types.get(k, 'priority')) for k, p in nodes.items()]
        edg, con = [], []
        plans = []
        for eid, cuts, pieces in splits:
            e = edges[eid]
            shape = _read(e.get('shape') or e.find('lane').get('shape'))
            L = _length(shape)
            fr, to = e.get('from'), e.get('to')
            seq = [fr] + [j for j, _ in cuts] + [to]
            bounds = [0.0] + [s for _, s in cuts] + [L]
            n = len(e.findall('lane'))
            speed = e.find('lane').get('speed')
            width = e.find('lane').get('width', '3.2')
            sub_seq, sub_bounds = list(seq), list(bounds)
            if fr not in sub_nodes:                       # 바깥 끝 → 부분망에서는 가까운 곳에 임시 막다른 끝
                sub_seq[0] = f'U_{eid}'
                sub_bounds[0] = max(0.0, bounds[1] - STUB)
                nod.append((sub_seq[0], at(shape, sub_bounds[0]), 'dead_end'))
            if to not in sub_nodes:
                sub_seq[-1] = f'D_{eid}'
                sub_bounds[-1] = min(L, bounds[-2] + STUB)
                nod.append((sub_seq[-1], at(shape, sub_bounds[-1]), 'dead_end'))
            for i, pid in enumerate(pieces):
                edg.append((pid, sub_seq[i], sub_seq[i+1], n, width, speed, e.get('priority', '1'),
                            slice_(shape, sub_bounds[i], sub_bounds[i+1]), e.get('spreadType', 'right'), None))
            for a, b in zip(pieces, pieces[1:]):
                con += [(a, b, k, k) for k in range(n)]
            plans.append((eid, e, shape, fr, to, bounds, pieces))
        for eid, a, b, nl, w, sp, prio, pts, *rest in streets:   # rest: [spreadType, allow(차종)]
            edg.append((eid, a, b, nl, w, sp, prio, pts, rest[0] if rest else 'right',
                        rest[1] if len(rest) > 1 else None))
        con += list(extra_con)
        with open(os.path.join(work, 'p.nod.xml'), 'w') as fh:
            fh.write('<nodes>\n')
            for nid, (x, y), t in nod:
                shp = f' shape="{_write(node_shapes[nid])}"' if nid in node_shapes else ''
                fh.write(f'  <node id="{nid}" x="{x:.6f}" y="{y:.6f}" type="{t}"{shp}/>\n')
            fh.write('</nodes>\n')
        with open(os.path.join(work, 'p.edg.xml'), 'w') as fh:
            fh.write('<edges>\n')
            for eid, a, b, nl, w, sp, prio, pts, spread, allow in edg:
                al = f' allow="{allow}"' if allow else ''
                fh.write(f'  <edge id="{eid}" from="{a}" to="{b}" numLanes="{nl}" speed="{sp}" priority="{prio}" '
                         f'width="{w}" spreadType="{spread}"{al} shape="{_write(pts)}"/>\n')
            fh.write('</edges>\n')
        with open(os.path.join(work, 'p.con.xml'), 'w') as fh:
            fh.write('<connections>\n')
            for a, b, i, k in con:
                fh.write(f'  <connection from="{a}" to="{b}" fromLane="{i}" toLane="{k}"/>\n')
            fh.write('</connections>\n')
        from sumolib import checkBinary
        out = os.path.join(work, 'p.net.xml')
        r = subprocess.run([checkBinary('netconvert'), '-n', 'p.nod.xml', '-e', 'p.edg.xml', '-x', 'p.con.xml',
                            '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                            '--junctions.corner-detail', '24', '--precision', '6', '-o', out],
                           cwd=work, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr)
        sub = ET.parse(out).getroot()
    sub_edges = {e.get('id'): e for e in sub.findall('edge')}
    sub_lane = {(e.get('id'), int(l.get('index'))): _read(l.get('shape'))
                for e in sub.findall('edge') for l in e.findall('lane')}

    # 1) 기존 엣지를 조각으로 바꾼다
    for eid, e, shape, fr, to, bounds, pieces in plans:
        lanes = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
        pos = list(root).index(e)
        root.remove(e)
        for i, pid in enumerate(pieces):
            a_ext = i == 0 and fr not in sub_nodes
            b_ext = i == len(pieces) - 1 and to not in sub_nodes
            el = ET.Element('edge', dict(e.attrib))
            el.set('id', pid)
            el.set('from', fr if i == 0 else sub_edges[pid].get('from'))
            el.set('to', to if i == len(pieces) - 1 else sub_edges[pid].get('to'))
            el.set('shape', _write(slice_(shape, bounds[i], bounds[i+1])))
            for l in lanes:
                k = int(l.get('index'))
                orig = _read(l.get('shape'))
                sp = sub_lane[(pid, k)]
                if a_ext:                                  # 원래 시작 + 부분망 교차점 경계에서 끝
                    p = slice_(orig, 0.0, project(orig, sp[-1]))[:-1] + [sp[-1]]
                elif b_ext:                                # 부분망 교차점 경계에서 시작 + 원래 끝
                    p = [sp[0]] + slice_(orig, project(orig, sp[0]), _length(orig))[1:]
                else:
                    p = sp
                ln = ET.SubElement(el, 'lane', dict(l.attrib))
                ln.set('id', f'{pid}_{k}')
                ln.set('shape', _write(p))
                ln.set('length', f'{_length(p):.6f}')
            root.insert(pos + i, el)
        first, last = pieces[0], pieces[-1]
        # 바깥 교차점에서 쓰던 이름 바꾸기 (이름이 바뀐 경우만)
        for c in list(root.findall('connection')):
            if c.get('from') == eid:
                if to in sub_nodes:
                    root.remove(c)                         # 부분망 교차점의 연결은 새로 넣는다
                elif last != eid:
                    c.set('from', last)
            if c.get('to') == eid and fr not in sub_nodes and first != eid:
                c.set('to', first)
        if to not in sub_nodes and last != eid:
            for j in root.findall('junction'):
                if j.get('id') == to or j.get('id', '').startswith(f':{to}_'):
                    j.set('incLanes', ' '.join(f'{last}_{x.rsplit("_", 1)[1]}' if x.rsplit('_', 1)[0] == eid
                                               else x for x in j.get('incLanes', '').split()))

    # 2) 다시 만드는 기존 교차점: 원래 것과 그 내부 엣지·연결을 지운다
    for a in attach:
        for j in list(root.findall('junction')):
            if j.get('id') == a or j.get('id', '').startswith(f':{a}_'):
                root.remove(j)
        for e in list(root.findall('edge')):
            if e.get('id', '').startswith(f':{a}_'):
                root.remove(e)
        for c in list(root.findall('connection')):
            if c.get('from', '').startswith(f':{a}_'):
                root.remove(c)

    # 3) 부분망의 새 엣지·교차점·내부 엣지·연결
    prefs = tuple(f':{x}_' for x in sub_nodes)
    street_ids = {s[0] for s in streets}
    piece_ids = {p for _, _, ps in splits for p in ps}

    def last_index(tag):
        return [i for i, c in enumerate(list(root)) if c.tag == tag][-1]

    pos = last_index('edge') + 1
    for e in sub.findall('edge'):
        if e.get('id') in street_ids or e.get('id', '').startswith(prefs):
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
        if f.startswith(prefs) or ((f in street_ids or f in piece_ids) and (t in street_ids or t in piece_ids)):
            # 조각 → 조각 연결은 부분망 교차점에서만 (바깥 교차점의 연결은 원래 것을 쓴다)
            if f in piece_ids and t in piece_ids:
                ef = sub_edges[f]
                if ef.get('to') not in sub_nodes:
                    continue
            root.insert(pos, c)
            pos += 1
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return sub_nodes
