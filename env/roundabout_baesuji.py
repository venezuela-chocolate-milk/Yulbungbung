"""완정로165번길(등뼈) 동쪽 · 왕길배수지공원 주변 골목 (2026-11-12 네이버 지도 캡처 기준).

  - 빌라 순환 골목: 완정로133번길(int_133)에서 남쪽으로 내려가 둥글게 돌아 165번길(int_spine_b)로 나온다.
    유현샤인빌·파크빌·양지쉐르빌을 감싸고, 힐스타운빌라는 골목 남쪽.
  - (2026-11-13 삭제) 동쪽 긴 골목 bj_east: 공원 동쪽 막다른 끝 → int_east T자. 사용자 요청으로 지우고 그 자리는
    env.roundabout_baesuji_scenery 의 반듯한 건물 격자가 채운다.
  둘 다 왕복 1차로 3.2m (기존 안쪽 골목과 같은 단면), 기존 골목과 만나는 곳은 모든 방향 허용.
  기존 골목 엣지를 나눌 때 첫 조각이 원래 이름을 쓴다.
  roundabout_south_extend 가 검단사거리 북동쪽 다음 단계로 호출한다. 그림은 env.roundabout_baesuji_scenery.
"""
import math
import xml.etree.ElementTree as ET

PREFIX = 'bj_'
LANE_W = 3.2
SPEED = 8.33
APPROACH = 15.0
NAV_SCALE = 0.452                 # 이 캡처(2395x1372, 오른쪽 절반이 네이버) m/px


def NB(px, py, dx=0.0):
    """네이버 캡처 → SUMO. 165번길·133번길 교차점(1380, 195) = int_P133."""
    return (596.0 + (px - 1380.0)*NAV_SCALE + dx, 190.0 - (py - 195.0)*NAV_SCALE)


# 기존 골목 위 T자: (교차점, 엣지 이름(_f/_r), 대략 위치)
TEES = [
    ('bj_TA', 'int_133', NB(1532, 167)),
    ('bj_TB', 'int_spine_b', NB(1392, 513)),
]
# 2026-11-13: bj_east 삭제로 T자 ('bj_TC', 'int_east', (955.0, -360.0)) 과 막다른 끝 'bj_E0' NB(2132, 593) 도 뺐다
NODES = {}
# (이름, 시작, 끝, 우선순위, 중간 조정점)
STREETS = [
    ('bj_loop', 'bj_TA', 'bj_TB', 0, [NB(1538, 373), NB(1572, 440), NB(1565, 487), NB(1525, 513)]),
]
# 지운 동쪽 골목 자리 (그림에서 건물 격자를 둘 위치 기준으로만 쓴다)
OLD_EAST = [NB(2132, 593), NB(2120, 640), NB(1952, 760), NB(1903, 820), NB(1895, 1020), NB(1925, 1147), NB(2085, 1280),
            (930.0, -416.0)]


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def plan(net):
    from env.roundabout_interior import _fillet, _project, _at, _dir_at
    es = {e.get('id'): e for e in net.findall('edge')}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    nodes = dict(NODES)
    tee = {}
    splits, con = [], []
    for node, base, near in TEES:
        f = _read(es[f'{base}_f'].get('shape'))
        r = _read(es[f'{base}_r'].get('shape'))
        sf, sr = _project(f, near), _project(r, near)
        pf, pr = _at(f, sf), _at(r, sr)
        p = ((pf[0] + pr[0])/2, (pf[1] + pr[1])/2)             # 두 방향 차로 사이 = 골목 중심선
        d = _dir_at(f, sf)
        nodes[node] = p
        tee[node] = (base, d)
        splits += [(f'{base}_f', [(node, sf)], [f'{base}_f', f'{base}_f_bj']),
                   (f'{base}_r', [(node, sr)], [f'{base}_r', f'{base}_r_bj'])]
    allxy = dict(J)
    allxy.update(nodes)
    streets = []
    for name, a, b, prio, mids in STREETS:
        ctrl = [allxy[a]] + list(mids) + [allxy[b]]
        keep = {}
        for end in (a, b):
            if end not in tee:
                continue
            base, d = tee[end]
            p = allxy[end]
            nxt = ctrl[1] if end == a else ctrl[-2]
            n = (d[1], -d[0])                                    # 골목이 오는 쪽으로 수직
            if (nxt[0] - p[0])*n[0] + (nxt[1] - p[1])*n[1] < 0:
                n = (-n[0], -n[1])
            q = (p[0] + n[0]*APPROACH, p[1] + n[1]*APPROACH)
            ctrl = [p, q] + ctrl[1:] if end == a else ctrl[:-1] + [q, p]
        for end in (a, b):
            if end in tee:
                keep[1 if end == a else len(ctrl) - 2] = APPROACH*0.5
        pts = _fillet(ctrl, radius=18.0, keep=keep, step=2.0)
        streets.append((f'{name}_f', a, b, 1, LANE_W, SPEED, prio, pts, 'right'))
        streets.append((f'{name}_r', b, a, 1, LANE_W, SPEED, prio, pts[::-1], 'right'))
    # T자 연결: 기존 골목 양방향 → 새 골목 진입, 새 골목 → 기존 골목 양방향 (조각 사이 직진은 graft 가 넣는다)
    for node, (base, _) in tee.items():
        name = next(s for s in STREETS if node in (s[1], s[2]))
        ain, aout = (f'{name[0]}_f', f'{name[0]}_r') if name[1] == node else (f'{name[0]}_r', f'{name[0]}_f')
        con += [(f'{base}_f', ain, 0, 0), (f'{base}_r', ain, 0, 0),
                (aout, f'{base}_f_bj', 0, 0), (aout, f'{base}_r_bj', 0, 0)]
    return nodes, streets, splits, con


def apply_net(net_path):
    from env.roundabout_graft import graft
    from env.roundabout_majeon_north import _ensure_shapes
    from env.roundabout_nw_shops import _strip_short_internal
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(PREFIX) for e in net.findall('edge')):
        raise RuntimeError('배수지 골목이 이미 적용됨')
    _ensure_shapes(net_path, {f'{t[1]}_{d}' for t in TEES for d in 'fr'})
    net = ET.parse(net_path).getroot()
    nodes, streets, splits, con = plan(net)
    new_nodes = {k: v for k, v in nodes.items() if k not in {t[0] for t in TEES}}
    graft(net_path, new_nodes, streets, splits, extra_con=con, node_types={k: 'dead_end' for k in NODES})
    _strip_short_internal(net_path, tuple(nodes))
    from env.roundabout_interior import _length
    return {'bj_streets': len(STREETS), **{f'{s[0]}_m': round(_length(s[7]), 1) for s in streets if s[0].endswith('_f')}}


def _cfg():
    return dict(TEES=[], REBUILD={}, NODES={**NODES, **{t[0]: (0.0, 0.0) for t in TEES}}, STREETS=STREETS,
                PREFIX=PREFIX, MEDIAN={}, DOUBLE_CENTER=set(), APPROACH=APPROACH)


def apply_deco(deco_path, net_path):
    """기존 골목 중앙선이 새 T자 안을 지나가면 끊고, 새 골목은 안쪽 길 공통 표시."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    from env import roundabout_interior as it
    from env.roundabout_geomdan import _write
    net = ET.parse(net_path).getroot()
    jsh = [Polygon(_read(j.get('shape'))).buffer(0.3) for j in net.findall('junction')
           if j.get('id') in {t[0] for t in TEES} and j.get('shape')]
    cut = unary_union(jsh)
    tree = ET.parse(deco_path)
    root = tree.getroot()
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
            el.set('id', p.get('id') if k == 0 else f"{p.get('id')}_bjc{k}")
            el.set('shape', _write(list(part.coords)))
            root.insert(idx + k, el)
        n_clip += 1
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    info = {'bj_deco_clipped': n_clip}
    info.update({f'bj_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(_cfg(), it.apply_deco, deco_path, net_path).items()})
    return info
