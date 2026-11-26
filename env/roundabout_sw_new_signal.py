"""서곶로 × 원당대로 사거리(SW_new_cross)에 신호 (2026-11-15 사용자 요청).

  원래 신호 없는 우선 교차로라, 6차로 서곶로가 5차로 원당대로를 양보만으로 건넜다 → 교통량 1.0 에서 좌회전 차가
  틈을 못 찾아 갇혔다가 순간이동(1시간 약 20대). 근처 S_cross 와 같은 4현시:
    1) 원당대로 직진·우회전   2) 원당대로 보호 좌회전   3) 서곶로 직진·우회전   4) 서곶로 보호 좌회전
  현시 사이 황색, 반대 방향 넘어갈 때 전적색. 우회전은 녹색이 아닐 때도 양보하며 진행(g).
  교차점 모양·회전 경로는 그대로 두고 신호 프로그램과 연결의 tl/linkIndex 만 단다
  (linkIndex = 교차점 intLanes 순서 = 우선순위 표 순서).

roundabout_south_extend 가 도로 단계가 다 끝난 뒤(정지선 점검 전) 부른다. 신호등 그림은 마지막 단계가 자동으로 그린다.
"""
import xml.etree.ElementTree as ET

J = 'SW_new_cross'
WONDANG = ('SW_slip_main', 'SW_end_T_extension_in')      # 원당대로 동·서 진입
SEOGOT = ('SW_new_lower_in', 'SW_new_upper_in')          # 서곶로 남·북 진입
# (길이 초, {(그룹, 방향): 신호}) — 정해지지 않은 움직임은 우회전 g / 나머지 r
# 2026-11-17: 원당대로 보호 좌회전 8 → 24초, 원당대로 직진 30 → 14초 (주기 90초 유지). 원당대로 직진은 4차로라 14초로도 약 960대/h. 동행 좌회전(마지막 주차장 가는 길)
#   수요 약 116대/h 가 보호 좌회전 8초 용량(약 120대/h)과 거의 같아 좌회전 줄이 늘 차 있었다
PLAN = [
    (14, {('W', 's'): 'G', ('W', 'r'): 'G'}),   # 2026-11-17: 22 → 14초 (좌회전 24초로 늘린 만큼, 주기 90초 유지 — 98초는 서곶로 적색이 길어져 원형교차로 남쪽 진입로까지 막힘)
    (4, {('W', 's'): 'y', ('W', 'r'): 'y'}),
    (24, {('W', 'l'): 'G'}),   # 2026-11-17: 8 → 16 → 24초 (좌회전 수요 약 200대/h, 좌회전 차로 약 36 m 가 자주 참). 주기 90초
    (3, {('W', 'l'): 'y'}),
    (2, {}),
    (26, {('S', 's'): 'G', ('S', 'r'): 'G'}),
    (4, {('S', 's'): 'y', ('S', 'r'): 'y'}),
    (8, {('S', 'l'): 'G'}),
    (3, {('S', 'l'): 'y'}),
    (2, {}),
]


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    jn = next(j for j in root.findall('junction') if j.get('id') == J)
    if jn.get('type') == 'traffic_light':
        raise RuntimeError('이미 신호 교차로')
    order = jn.get('intLanes').split()
    links = {}
    # 교차점 안에서 둘로 나뉜 좌회전(대기 지점이 있는 경로)은 intLanes 에 뒤쪽 조각이 들어 있다 → 앞 조각에서 따라간다
    nxt = {c.get('from'): c.get('via') for c in root.findall('connection')
           if c.get('from', '').startswith(f':{J}_') and c.get('via')}
    for c in root.findall('connection'):
        via = c.get('via', '')
        if not via.startswith(f':{J}_') or c.get('from').startswith(':'):
            continue
        if via not in order:
            via = nxt.get(via.rsplit('_', 1)[0], '')
        if via in order:
            f = c.get('from')
            g = 'W' if f in WONDANG else 'S' if f in SEOGOT else None
            if g is None:
                raise RuntimeError(f'모르는 진입로 {f}')
            d = 'l' if c.get('dir') in ('l', 'L') else 'r' if c.get('dir') in ('r', 'R') else 's'
            links[order.index(via)] = (c, g, d)
    n = len(order)
    if sorted(links) != list(range(n)):
        raise RuntimeError('연결 수와 내부 경로 수가 다르다')
    tl = ET.Element('tlLogic', id=J, type='static', programID='0', offset='0')
    for dur, rule in PLAN:
        st = [rule.get((g, d), 'g' if d == 'r' else 'r') for _, g, d in (links[i] for i in range(n))]
        # 같은 현시에 녹색(G) 둘이 같은 차로로 들어가면(공용 차로 직진이 옆 차로와 한 차로로 합쳐지는 곳) 하나만 G,
        # 나머지는 양보하며 진행(g) — 둘 다 G 면 교차점 안에서 합쳐지며 부딪힌다
        into = {}
        for i in range(n):
            if st[i] == 'G':
                c = links[i][0]
                into.setdefault((c.get('to'), c.get('toLane')), []).append(i)
        for (to, tl_), idx in into.items():
            if len(idx) > 1:
                keep = min(idx, key=lambda i: (abs(int(links[i][0].get('fromLane')) - int(tl_)), i))
                for i in idx:
                    if i != keep:
                        st[i] = 'g'
        ET.SubElement(tl, 'phase', duration=str(dur), state=''.join(st))
    # 같은 차로로 합쳐지는 두 경로가 우선순위 표에서 서로 상충(foes)으로 안 잡혀 있었다 (원래 망) → 상충으로 넣고,
    # 옆 차로에서 들어오는 쪽(차로 번호 차이가 큰 쪽)이 양보(response)
    req = {int(q.get('index')): q for q in jn.findall('request')}

    def setbit(q, key, k):
        s = list(q.get(key))
        s[n - 1 - k] = '1'
        q.set(key, ''.join(s))
    by_target = {}
    for i in range(n):
        c = links[i][0]
        by_target.setdefault((c.get('to'), c.get('toLane')), []).append(i)
    merged = 0
    for (to, tl_), idx in by_target.items():
        if len(idx) < 2:
            continue
        keep = min(idx, key=lambda i: (abs(int(links[i][0].get('fromLane')) - int(tl_)), i))
        for i in idx:
            for k in idx:
                if i != k:
                    setbit(req[i], 'foes', k)
            if i != keep:
                setbit(req[i], 'response', keep)
                merged += 1
    for i, (c, _, _) in links.items():
        c.set('tl', J)
        c.set('linkIndex', str(i))
    jn.set('type', 'traffic_light')
    last = [i for i, c in enumerate(list(root)) if c.tag in ('tlLogic', 'edge')][-1] + 1
    root.insert(last, tl)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'sw_new_signal_links': n, 'sw_new_signal_cycle_s': sum(p[0] for p in PLAN), 'sw_new_signal_merges': merged}
