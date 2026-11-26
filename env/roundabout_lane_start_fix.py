"""차로가 늘어나는 곳의 '앞으로 순간이동' 고치기 (2026-11-14).

  한 엣지 안에서 늘어나는 차로(예: X_se_out_far 2차로)가 다른 차로보다 14~20m 늦게 시작하면, 엣지에 들어서자마자
  그 차로로 바꾸는 차는 SUMO 가 같은 위치값을 그 차로 시작점부터 재므로 그만큼 앞으로 튀어 보인다.
  늦게 시작하는 차로 앞에 옆 차로의 시작점을 붙여 (옆 차로 위에서 시작해 비스듬히 빠져나가는 테이퍼)
  모든 차로가 같은 곳에서 시작하게 한다. 그 차로로 들어오는 연결(회전 경로)이 있으면 건드리지 않는다.

roundabout_south_extend 가 모든 도로 단계가 끝난 뒤(신호등 그림 전) 부른다.
"""
import math
import xml.etree.ElementTree as ET

MIN_LAG = 3.0


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def apply(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    fed = {(c.get('to'), c.get('toLane')) for c in root.findall('connection')}
    fixed = []
    for e in root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        ls = sorted(e.findall('lane'), key=lambda l: int(l.get('index')))
        if len(ls) < 2:
            continue
        pts = {int(l.get('index')): _read(l.get('shape')) for l in ls}
        p0 = pts[0]
        d = (p0[1][0] - p0[0][0], p0[1][1] - p0[0][1])
        n = math.hypot(*d)
        d = (d[0]/n, d[1]/n)
        s = {k: (p[0][0] - p0[0][0])*d[0] + (p[0][1] - p0[0][1])*d[1] for k, p in pts.items()}
        first = min(s.values())
        for l in ls:
            k = int(l.get('index'))
            if s[k] - first <= MIN_LAG or (e.get('id'), str(k)) in fed:
                continue
            # 가장 가까운 이웃 차로 중 일찍 시작하는 쪽
            nb = min((j for j in (k - 1, k + 1) if j in s and s[j] - first <= MIN_LAG), key=lambda j: abs(j - k), default=None)
            if nb is None:
                continue
            q = [pts[nb][0]] + pts[k]
            l.set('shape', _write(q))
            L = sum(math.dist(a, b) for a, b in zip(q, q[1:]))
            l.set('length', f'{L:.6f}')
            fixed.append(f'{e.get("id")}_{k}')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'lane_start_fixed': fixed}
