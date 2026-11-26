"""검단사거리 남서쪽 큰길의 왼쪽 옆길(T_end_side, 실제 검단로445번길)을 교차점째로 큰길 방향으로 평행이동한다.

  - 움직이는 묶음: 옆길 교차점(T_main_end), 좌회전 차로가 생기는 지점(T_end_bay), 옆길 끝(T_end_side_end)과
    그 사이 엣지·내부 엣지, 교차점 주변 노면 표시(정지선·화살표·유도선·중앙선·좌회전 차로 안전지대).
  - 큰길: 검단사거리 쪽 구간(T_end_main_in/out)은 D 만큼 길어지고, 반대쪽(T_main_in/out)은 D 만큼 짧아진다.
    큰길 축 위 station(검단사거리 중심에서 남서쪽으로 m) 이 (C1, C2] 인 점만 D 만큼 옮긴다.
  - D 는 노면 점선 주기의 배수. 남서쪽 신호 교차로(T_junction, 약 355m) 바닥면이 331m 부터라
    안전지대 끝(280m)이 그 앞에 남도록 8주기(약 47.9m)로 한다. (실제 지도상 445번길 위치는 약 282m)
roundabout_south_extend 가 상가 골목 다음 단계로 호출한다.  단독 실행: python -m env.roundabout_slide_side
"""
import math
import re
import xml.etree.ElementTree as ET

CENTER = 'T_end_main_end'
AXIS_TO = 'T_main_end'
MOVE_NODES = {'T_main_end', 'T_end_bay', 'T_end_side_end'}
STRETCH = {'T_end_main_in', 'T_end_main_out'}          # 길어지는 쪽 (C1 을 지난다)
SHRINK = {'T_main_in', 'T_main_out'}                   # 짧아지는 쪽 (C2 를 지난다)
PERIODS = 8
C1, C2 = 180.0, 282.0
LAT = (-25.0, 112.0)                                   # 노면 표시를 옮길 큰길 축 옆 범위 (m, + 가 옆길 쪽)


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def _length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def apply(net_path, deco_path):
    from env.roundabout_markings import dash_style
    dash, gap, _ = dash_style(20)
    D = PERIODS * (dash + gap)
    tree = ET.parse(net_path)
    root = tree.getroot()
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in root.findall('junction')}
    o, a = J[CENTER], J[AXIS_TO]
    L = math.dist(o, a)
    u = ((a[0]-o[0])/L, (a[1]-o[1])/L)
    n = (u[1], -u[0])                                  # 옆길 쪽 (큰길 남서행 오른쪽)
    st = lambda p: (p[0]-o[0])*u[0] + (p[1]-o[1])*u[1]
    lat = lambda p: (p[0]-o[0])*n[0] + (p[1]-o[1])*n[1]
    mv = lambda p: (p[0] + u[0]*D, p[1] + u[1]*D)
    inside = lambda p: C1 < st(p) <= C2
    pat = '|'.join(re.escape(x) for x in MOVE_NODES)
    internal = re.compile(rf'^:({pat})_\d+(_\d+)?$')

    def cut_line(pts):
        """C1~C2 안의 점은 옮기고, 짧아지는 쪽에서 옮긴 점과 겹치게 되는 (C2, C2+D] 고정 점은 뺀다."""
        out = [mv(p) if inside(p) else p for p in pts]
        return [q for p, q in zip(pts, out) if inside(p) or not (C2 < st(p) <= C2 + D)]

    # --- 도로망 ---
    for e in root.findall('edge'):
        eid = e.get('id')
        if e.get('function') == 'internal':
            fn = (lambda pts: [mv(p) for p in pts]) if internal.match(eid) else None
        elif e.get('from') in MOVE_NODES and e.get('to') in MOVE_NODES:
            fn = lambda pts: [mv(p) for p in pts]
        elif eid in STRETCH | SHRINK:
            fn = cut_line
        else:
            fn = None
        if fn is None:
            continue
        if e.get('shape'):
            e.set('shape', _write(fn(_read(e.get('shape')))))
        for l in e.findall('lane'):
            pts = fn(_read(l.get('shape')))
            l.set('shape', _write(pts))
            if eid in STRETCH | SHRINK:
                l.set('length', f'{_length(pts):.6f}')
    for j in root.findall('junction'):
        jid = j.get('id')
        if jid in MOVE_NODES or (jid.startswith(':') and internal.match(jid)):
            x, y = mv((float(j.get('x')), float(j.get('y'))))
            j.set('x', f'{x:.6f}')
            j.set('y', f'{y:.6f}')
            if j.get('shape'):
                j.set('shape', _write([mv(p) for p in _read(j.get('shape'))]))
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)

    # --- 노면 표시 ---
    dtree = ET.parse(deco_path)
    droot = dtree.getroot()
    moved = removed = filled = 0
    period = dash + gap
    short = lambda pts: len(pts) == 2 and math.dist(*pts) < 5.0          # 점선 한 토막·짧은 표시
    on_leg = lambda pts: all(abs(lat(q)) <= 12.0 for q in pts)           # 큰길 차로 위
    groups = {}                                                         # 길어지는 쪽 점선 줄 (C1 앞 마지막 토막)
    for p in list(droot.findall('poly')):
        if p.get('id') == 'ground' or not p.get('shape'):
            continue
        pts = _read(p.get('shape'))
        if not all(LAT[0] <= lat(q) <= LAT[1] for q in pts):
            continue
        if short(pts):
            mid = ((pts[0][0]+pts[1][0])/2, (pts[0][1]+pts[1][1])/2)
            if inside(mid):
                p.set('shape', _write([mv(q) for q in pts]))           # 점선 토막은 가운데 기준 통째로
                moved += 1
            elif on_leg(pts) and C2 < st(mid) <= C2 + D:
                droot.remove(p)                                         # 짧아지는 쪽: 옮겨 온 구간 밑 점선
                removed += 1
            elif on_leg(pts) and p.get('fill') == '0' and C1 - period - 1.0 < st(mid) <= C1:
                key = round(lat(mid), 1)
                if key not in groups or st(mid) > groups[key][0]:
                    groups[key] = (st(mid), p)
            continue
        if not any(inside(q) for q in pts):
            continue
        if p.get('fill') == '1' and len(pts) > 4 and sum(inside(q) for q in pts) >= len(pts) / 2:
            new = [mv(q) for q in pts]                 # 모양 있는 도형(모퉁이·안전지대)은 통째로
        else:
            new = cut_line(pts)                        # 긴 선(중앙선·바닥 띠)은 안쪽 점만 → 늘어나거나 줄어든다
        if len(new) >= 2:
            p.set('shape', _write(new))
            moved += 1
    # 길어진 구간 (C1, C1+D] 의 점선은 같은 주기로 복제해 채운다
    for key, (s_last, tmpl) in groups.items():
        pts = _read(tmpl.get('shape'))
        k = 1
        while s_last + k*period < C1 + D - 1e-6:
            el = ET.Element('poly', dict(tmpl.attrib))
            el.set('id', f'{tmpl.get("id")}_slide{k}')
            el.set('shape', _write([(q[0]+u[0]*k*period, q[1]+u[1]*k*period) for q in pts]))
            droot.insert(list(droot).index(tmpl) + 1, el)
            filled += 1
            k += 1
    ET.indent(dtree, space='    ')
    dtree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'side_slide_m': round(D, 3), 'side_deco_moved': moved, 'side_deco_removed': removed,
            'side_dash_filled': filled}


def main():
    import os
    import sys
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, base)
    net_dir = os.path.join(base, 'env', 'sumo_roundabout')
    print(apply(os.path.join(net_dir, 'roundabout.net.xml'), os.path.join(net_dir, 'roundabout.deco.xml')))


if __name__ == '__main__':
    main()
