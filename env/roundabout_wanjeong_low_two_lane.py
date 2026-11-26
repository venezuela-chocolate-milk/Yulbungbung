"""완정로 아래 끝 (원당대로 T자 SW_long_conn 바로 위 76m, KB국민은행 서쪽) 양방향 2차로 (2026-11-14 사용자 요청).

  원래 위쪽 2차로가 여기서 1차로로 줄어 원당대로에 붙었다 (S_conn_down1 남행 / S_conn_up1 북행, 각 1차로).
  두 엣지를 2차로로 넓히고(안쪽 차로 = 예전 차로 그대로, 바깥에 한 차로), 양 끝 교차점만 다시 만든다
  (env.roundabout_junction_patch). 원당대로로 나가는 쪽은 이중 우회전(바깥 → 원당대로 1차로, 안쪽 → 2차로),
  원당대로에서 들어오는 우회전은 예전처럼 원당대로 바깥 차로 → 바깥 차로.

roundabout_south_extend 가 마전동 동쪽 길(9-7-1) 바로 뒤에서 부른다 (그 단계가 원당대로·완정로를 나눈 이름이 필요).
"""
import math
import xml.etree.ElementTree as ET

DOWN, UP = 'S_conn_down1', 'S_conn_up1'
JS = ['S_conn_n1', 'S_conn_n2', 'SW_long_conn']
ABOVE_DOWN, ABOVE_UP = 'S_conn_down_mj_4', 'S_conn_up'
MAIN_OUT, MAIN_IN = 'SW_end_T_left_in_far', 'SW_end_T_left_in_far2_me_1'
CON = [(ABOVE_DOWN, 0, DOWN, 0), (ABOVE_DOWN, 1, DOWN, 1),
       (DOWN, 0, MAIN_OUT, 0), (DOWN, 1, MAIN_OUT, 1),
       (MAIN_IN, 0, UP, 0),
       (UP, 0, ABOVE_UP, 0), (UP, 1, ABOVE_UP, 1)]


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _write(pts):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in pts)


def apply_net(net_path):
    from env.roundabout_junction_patch import patch
    info = patch(net_path, JS, {DOWN: 2, UP: 2}, CON)
    return {'wanjeong_low_links': info['patched_links'], 'wanjeong_low_new_paths': info['patched_new_paths']}


def apply_deco(deco_path, net_path):
    """정지선을 두 차로 폭으로(원래 자리), 우회전 화살표를 차로마다, 같은 방향 두 차로 사이 흰 점선."""
    from shapely.geometry import LineString, Point
    from env.roundabout_builder import _arrow, _read_lane_shapes, _stop_line
    from env.roundabout_geometry import _offset, _slice
    tree = ET.parse(deco_path)
    root = tree.getroot()
    polys = {p.get('id'): p for p in root.findall('poly')}
    shapes = _read_lane_shapes(net_path)
    lane1 = LineString(shapes[(DOWN, 1)])
    stop_dist = 1.0
    if 'sconn_stop' in polys:
        mid = _read(polys['sconn_stop'].get('shape'))
        c = ((mid[0][0] + mid[-1][0]) / 2, (mid[0][1] + mid[-1][1]) / 2)
        stop_dist = max(0.4, lane1.length - lane1.project(Point(c)))
    for p in list(root):
        pid = p.get('id', '')
        if pid in ('sconn_stop', 'sconn_arrow_right') or pid.startswith('sconn2_'):
            root.remove(p)
    marks = []
    _stop_line(marks, 'sconn2_stop', shapes, DOWN, dist=stop_dist)
    for k in (0, 1):
        _arrow(marks, f'sconn2_arrow_right_{k}', shapes, DOWN, k, 'right', stop_dist + 7.0)
    for m in marks:
        root.append(ET.fromstring(m))
    n = 0
    for eid in (DOWN, UP):
        div = _offset(shapes[(eid, 0)], 1.6)
        L = LineString(div).length
        st = 1.5
        while st + 3.0 <= L - 1.5:
            ET.SubElement(root, 'poly', id=f'sconn2_dash_{eid}_{n}', color='255,255,255', fill='0', lineWidth='0.15',
                          layer='6', shape=_write(_slice(div, st, st + 3.0)))
            n += 1
            st += 6.0
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'wanjeong_low_stop_dist': round(stop_dist, 2), 'wanjeong_low_dashes': n}
