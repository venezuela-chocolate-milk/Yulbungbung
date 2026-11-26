"""마전동 북쪽 (마전역 삼각형 교차로 동쪽, 완정로 북쪽 ~ 서곶로, 2026-11-12 네이버 위성 지도 기준) 도로.

  가장 중요한 것: 마전역 교차로(SE_triangle_*)는 최대한 그대로. 완정로 아래 마전동 상가 줄도 그대로.
  - 서쪽 입구: 마전역에서 북북동으로 뻗은 막다른 짧은 길(SE_triangle_NE_in 남행 3차로 / _out 북행 1차로, 겹 노란 중앙선)을
    같은 단면 그대로 끝에 이어(짧은 길 이름 유지, RL 배경 교통이 쓴다) 곧게 가다 동쪽으로 휘어 가운데 큰 길(mx_mc*)이 된다.
  - 가운데 큰 길: 동네를 동서로 휘어 지나 서곶로 연장(sg_ext_s, 우회전 진입·진출 T자)까지 (왕복 1차로).
  - 안쪽 길: 완정로(S_conn_up, 우회전 진입·진출 T자)에서 북쪽으로 가운데 큰 길까지(mx_c),
    마전동 동쪽 가운데 길(me_s1)을 위쪽 변 길 교차점(me_J3)에서 북쪽으로 이어 가운데 큰 길까지(mx_s1d).
  차로 수가 바뀌는 곳(서쪽 입구 3/1차로 → 큰 길 1/1차로)은 길 중간이 아니라 교차점(mx_JC)에서.

roundabout_south_extend 가 마전동 동쪽 다음 단계로 호출한다.
"""
import math
import xml.etree.ElementTree as ET

PREFIX = 'mx_'
STUB = ('SE_triangle_NE_in', 'SE_triangle_NE_out')      # 마전역으로 들어감(3), 나감(1)
STUB_END = 'SE_triangle_NE'
NE = ('mx_ne_in', 'mx_ne_out')
SPEED = 8.33
LANE_W = 3.2
NE_STRAIGHT = 70.0
NODES = {'mx_JC': (1790.0, 140.0), 'mx_JS': (2100.0, 115.0)}
T_C = ('mx_T_C', 'S_conn_up_me_1', (1730.0, -295.0))     # 완정로 (북서행, 북동쪽 차로)
T_SG = ('mx_T_SG', 'sg_ext_s', (2300.0, 40.0))           # 서곶로 연장 (남행, 서쪽 차로)
APPROACH = 30.0
ATTACH_J3 = ('me_s1c_f', 'me_s1c_r', 'me_top1_f', 'me_top1_r', 'me_top2_f', 'me_top2_r')
YELLOW, WHITE = '235,190,40', '255,255,255'
# 안쪽 길 표시는 env.roundabout_interior.apply_deco 로 (이름·교차점만 쓴다)
STREET_NAMES = ['mx_mc2', 'mx_mc3', 'mx_c', 'mx_s1d']
CFG = dict(TEES=[(T_C[0], T_C[1], T_C[2]), (T_SG[0], T_SG[1], T_SG[2])], REBUILD={'me_J3': []}, NODES=NODES,
           STREETS=[(n, 'mx_JC', 'mx_JS', 0, []) for n in STREET_NAMES], PREFIX=PREFIX, MEDIAN={},
           DOUBLE_CENTER=set(), APPROACH=APPROACH)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _unit(v):
    L = math.hypot(*v)
    return (v[0]/L, v[1]/L)


def _ensure_shapes(net_path, eids):
    """shape 가 없는 엣지에 실제 길 모양을 적어 둔다 (graft 가 0번 차로 선을 길 모양으로 써 차로가 밀리는 것 방지)."""
    from env.roundabout_geometry import _offset
    tree = ET.parse(net_path)
    for e in tree.getroot().findall('edge'):
        if e.get('id') not in eids or e.get('shape'):
            continue
        ls = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
        w = float(ls[0].get('width', '3.2'))
        if e.get('spreadType') == 'center':
            shp = _offset(_read(ls[0].get('shape')), w*(len(ls) - 1)/2)
        else:
            shp = _offset(_read(ls[-1].get('shape')), w/2)
        e.set('shape', _write(shp))
    tree.write(net_path, encoding='utf-8', xml_declaration=True)


def plan(net):
    from env.roundabout_interior import _fillet, _project
    from env.roundabout_geometry import _offset
    es = {e.get('id'): e for e in net.findall('edge')}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    nodes = dict(NODES)
    # 서쪽 입구: 짧은 길 북행 차로 가운데 선을 그대로 이어 간다. 중앙선 = 북행 차로 왼쪽 1.6m
    lo = _read(es[STUB[1]].find('lane').get('shape'))
    d = _unit((lo[-1][0] - lo[0][0], lo[-1][1] - lo[0][1]))
    lf = (-d[1], d[0])
    c0 = (lo[-1][0] + lf[0]*LANE_W/2, lo[-1][1] + lf[1]*LANE_W/2)
    jc = nodes['mx_JC']
    c1 = (c0[0] + d[0]*NE_STRAIGHT, c0[1] + d[1]*NE_STRAIGHT)
    center = _fillet([c0, c1, (jc[0] - 150.0, jc[1] - 15.0), jc], radius=90.0, step=3.0)
    n_in = len(es[STUB[0]].findall('lane'))
    # 남행 3차로 가운데 = 중앙선 왼쪽 (3 × 3.2)/2, 북행 1차로 가운데 = 중앙선 오른쪽 1.6
    p_out = _offset(center, -LANE_W/2)
    p_in = _offset(center, LANE_W*n_in/2)[::-1]
    streets = [(NE[1], STUB_END, 'mx_JC', 1, LANE_W, SPEED, 2, p_out, 'center'),
               (NE[0], 'mx_JC', STUB_END, n_in, LANE_W, SPEED, 2, p_in, 'center')]
    two = lambda name, a, b, prio, pts: [(f'{name}_f', a, b, 1, LANE_W, SPEED, prio, pts),
                                         (f'{name}_r', b, a, 1, LANE_W, SPEED, prio, pts[::-1])]

    def tee(t):
        """큰길 엣지 위 T자 교차점: (위치, 엣지 시작에서의 거리, 안쪽 방향)."""
        line = _read(es[t[1]].get('shape'))
        s = _project(line, t[2])
        from env.roundabout_interior import _at, _dir_at
        p, dd = _at(line, s), _dir_at(line, s)
        return p, s, (dd[1], -dd[0])
    pc, sc, inc = tee(T_C)
    psg, ssg, insg = tee(T_SG)
    nodes[T_C[0]], nodes[T_SG[0]] = pc, psg
    js = nodes['mx_JS']
    qc = (pc[0] + inc[0]*APPROACH, pc[1] + inc[1]*APPROACH)
    qsg = (psg[0] + insg[0]*APPROACH, psg[1] + insg[1]*APPROACH)
    streets += two('mx_mc2', 'mx_JC', 'mx_JS', 2, _fillet([jc, (1950.0, 150.0), js], radius=120.0, step=3.0))
    streets += two('mx_mc3', 'mx_JS', T_SG[0], 2, _fillet([js, (2220.0, 95.0), qsg, psg], radius=60.0, step=3.0))
    streets += two('mx_c', T_C[0], 'mx_JC', 1, _fillet([pc, qc, (1765.0, -80.0), jc], radius=80.0, step=3.0))
    streets += two('mx_s1d', 'me_J3', 'mx_JS', 1, [J['me_J3'], js])
    splits = [(STUB[0], [], [STUB[0]]), (STUB[1], [], [STUB[1]])] + [(e, [], [e]) for e in ATTACH_J3] + [
        (T_C[1], [(T_C[0], sc)], [T_C[1], 'S_conn_up_mx']),
        (T_SG[1], [(T_SG[0], ssg)], [T_SG[1], 'sg_ext_s_mx'])]
    extra = [(STUB[1], NE[1], 0, 0)] + [(NE[0], STUB[0], k, k) for k in range(n_in)] + [
        (T_C[1], 'mx_c_f', 0, 0), ('mx_c_r', 'S_conn_up_mx', 0, 0),         # 완정로 우회전 진입·진출
        (T_SG[1], 'mx_mc3_r', 0, 0), ('mx_mc3_f', 'sg_ext_s_mx', 0, 0),     # 서곶로 우회전 진입·진출
        # mx_JC: 남북 길에서 서쪽 입구로 좌회전은 가운데 차로(1)로만 (맨 안쪽 차로로 들면 바로 차로를 옮겨 좌회전 감사 실패)
        ('mx_c_f', 'mx_mc2_f', 0, 0), ('mx_c_f', NE[0], 0, 1)]
    return nodes, streets, splits, extra, center


_CENTER = None


def apply_net(net_path):
    global _CENTER
    from env.roundabout_graft import graft
    from env.roundabout_nw_shops import _strip_short_internal
    net = ET.parse(net_path).getroot()
    if any(e.get('id', '').startswith(PREFIX) for e in net.findall('edge')):
        raise RuntimeError('마전동 북쪽 도로가 이미 적용됨')
    _ensure_shapes(net_path, set(ATTACH_J3) | {T_C[1], T_SG[1]} | set(STUB))
    net = ET.parse(net_path).getroot()
    nodes, streets, splits, extra, _CENTER = plan(net)
    new_nodes = {k: v for k, v in nodes.items() if k not in (T_C[0], T_SG[0])}
    graft(net_path, new_nodes, streets, splits, attach={STUB_END, 'me_J3'}, extra_con=extra)
    _strip_short_internal(net_path, tuple(nodes) + ('me_J3', STUB_END))
    return {'mx_streets': len(streets), 'mx_ne_m': round(sum(math.dist(a, b) for a, b in zip(_CENTER, _CENTER[1:])), 1)}


def apply_deco(deco_path, net_path):
    """서쪽 입구: 짧은 길과 같은 겹 노란 중앙선 + 남행 차로 사이 흰 점선. 안쪽 길: 안쪽 길 공통 표시.
    교차점 me_J3 를 다시 만들었으므로 마전동 동쪽 표시도 다시 그린다."""
    from env import roundabout_majeon_east as me
    info = me.apply_deco(deco_path, net_path)
    from env import roundabout_interior as it
    info.update({f'mx_{k.split("_", 1)[1]}': v for k, v in it.with_cfg(CFG, it.apply_deco, deco_path, net_path).items()})
    from env.roundabout_geometry import _offset
    from env.roundabout_markings import dash_style
    from env.roundabout_nw_shops import _dashes
    net = ET.parse(net_path).getroot()
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith('mxne_'):
            root.remove(p)
    # 중앙선: 북행 차로 왼쪽 경계 (실제 차로 모양 기준)
    mid = _offset(lane[f'{NE[1]}_0'], LANE_W/2)
    for k, off in enumerate((-0.15, 0.15)):
        ET.SubElement(root, 'poly', dict(id=f'mxne_center_{k}', color=YELLOW, fill='0', lineWidth='0.15', layer='6',
                                          shape=_write(_offset(mid, off))))
    dash, gap, _ = dash_style(20)
    n = 0
    n_in = sum(1 for k in lane if k.startswith(f'{NE[0]}_'))
    for k in range(n_in - 1):
        n += _dashes(root, f'mxne_div{k}', _offset(lane[f'{NE[0]}_{k}'], LANE_W/2), dash, gap)
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    info['mx_ne_dashes'] = n
    return info
