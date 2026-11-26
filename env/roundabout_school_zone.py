"""어린이보호구역 (스쿨존) 속도 단속 구역 (2026-11-13 사용자 요청).

  법적 기준 (대한민국):
    - 「어린이·노인 및 장애인 보호구역의 지정 및 관리에 관한 규칙」 제3조: 학교 주 출입문 기준 반경 300m 이내 도로
      (필요하면 500m 까지). 여기서는 300m 를 넘지 않게, 넘는 엣지는 300m 지점에서 잘라 안쪽만 구역으로 잡는다.
    - 「도로교통법」 제12조 제1항: 보호구역 통행 속도 시속 30km 이내로 제한 → 구역 안 차로 제한 속도 30 km/h.
    - 「도로교통법」 제12조 제4항 (2020 개정, 민식이법): 보호구역 도로에 무인 교통단속용 장비(과속 단속 카메라) 설치.
    - 「도로교통법 시행규칙」 별표6 노면표시: 어린이보호구역 문자 표시, 속도제한(30) 표시, 미끄럼방지 적색 포장.
  SUMO 에서는 차로 제한 속도를 30 km/h 로 바꾸므로 모든 배경 차가 그 속도로 달린다 (SUMO 운전자 속도 편차 ±10% 는 다른 도로와 같음).
  RL ego 는 차로 제한 속도를 관측하고(env/urban_driving.py vlim) 1.1 배를 넘으면 과속 벌점을 받는다 → 구역에서 감속을 배운다.

  그림 (roundabout.school_zone.add.xml, 화면 표시 전용):
    - 구역 안 모든 차로 적색 포장 (층 4.5: 차로 위, 차선 표시(층 4.8~6) 아래).
    - 구역 들어가는 차로마다: 노면 '어린이 보호구역' 문자 + '30' 표시, 길가에 '어린이보호구역 30' 표지 + 과속 단속 카메라.

roundabout_south_extend 파이프라인 끝에서 apply 를 부른다.  단독 실행: python -m env.roundabout_school_zone
"""
import math
import os
import xml.etree.ElementTree as ET

NET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_roundabout')
ADD = os.path.join(NET_DIR, 'roundabout.school_zone.add.xml')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
SPEED = 30/3.6
RED = '178,62,52'
L_RED, L_TEXT, L_SIGN = 4.5, 5.6, 7.0
TEXT_S = 14.0            # 구역 시작에서 '어린이 보호구역' 문자까지 (m)
NUM_S = 30.0             # '30' 까지
SIGN_S = 4.0             # 길가 표지·카메라

# 구역: (키, 이름, 엣지들, 구역으로 들어가는 엣지들). 엣지는 도로 전체를 바꾼다.
# 2026-11-14: 법적 기준(주 출입문 반경 300m)에 맞춤 — 300m 를 넘던 완정로 두 엣지를 300m 지점에서 잘라(SPLITS) 안쪽만 구역.
#             마전초 구역은 사용자 요청으로 지움 (그 길은 원래 30 km/h 라 속도는 그대로).
GATES = ('school_wanjeong_gate', 'school_majeon_gate')     # 완정초·마전중 주 출입문
RADIUS = 300.0
# 2026-11-14: 동쪽 끝은 양쪽 차로 모두 X_se_end 앞 노란 빗금 안전지대(x_se_end_safety_border)가 시작하는 서쪽 끝 선
#             (노란 선이 갈라지기 시작하는 곳)에서 끝낸다. 빗금 구간은 구역 밖.
HATCH = 'x_se_end_safety_border'
# (자를 엣지, 새 교차점, 조각 이름들 [시작 쪽, 끝 쪽], 구역 쪽 조각 번호, 자르는 곳: 'radius' | 'hatch')
SPLITS = [
    ('X_se_out_s3_int_1', 'sz_wj_W_out', ['X_se_out_s3_int_1', 'X_se_out_sz'], 1, 'radius'),  # 동행 서쪽 끝 (밖은 원래 50)
    ('X_se_in_gd2', 'sz_wj_W_in', ['X_se_in_sz', 'X_se_in_gd2'], 0, 'radius'),                # 서행 서쪽 끝
    ('X_se_out_b', 'sz_wj_E_out', ['X_se_out_b_sz', 'X_se_out_b'], 0, 'hatch'),               # 동행 동쪽 끝
    ('X_se_in_gd1', 'sz_wj_E_in', ['X_se_in_gd1', 'X_se_in_gd1_sz'], 1, 'hatch'),             # 서행 동쪽 끝 (구역 입구)
]
ZONES = [
    # 완정초·마전중: 두 학교 공동 진입로(school_access) 앞 완정로, 주 출입문 반경 300m 안 + 진입로
    ('wj', '완정초등학교·마전중학교',
     ['X_se_out_sz', 'X_se_out_b_sz', 'X_se_in_gd1_sz', 'X_se_in_sz', 'gd_h2_f', 'gd_h2_r',
      'school_in', 'school_out', 'school_wanjeong_in', 'school_wanjeong_out', 'school_majeon_in', 'school_majeon_out',
      'school_wanjeong_in_skw_1', 'school_wanjeong_out_skw_1', 'school_majeon_in_skm_1', 'school_majeon_out_skm_1'],
     ['X_se_out_sz', 'X_se_in_gd1_sz', 'gd_h2_f']),
]


def _hatch_end(deco_path):
    """안전지대 노란 테두리가 시작하는 서쪽(학교 쪽) 끝 점."""
    for p in ET.parse(deco_path).getroot().iter('poly'):
        if p.get('id') == HATCH:
            return min(_rd(p.get('shape')), key=lambda q: q[0])
    raise RuntimeError(f'{HATCH} 없음')


def split_zone_ends(net_path, deco_path=None):
    """SPLITS 엣지를 구역 경계(주 출입문 반경 RADIUS, 또는 안전지대 끝 선)에서 둘로 자른다 (이미 잘렸으면 건너뜀).
    잘린 위치(엣지 시작에서 m)를 돌려준다."""
    from env.roundabout_graft import graft, at, _length, project
    hatch = _hatch_end(deco_path or os.path.join(NET_DIR, 'roundabout.deco.xml'))
    root = ET.parse(net_path).getroot()
    E = {e.get('id'): e for e in root.findall('edge')}
    J = {j.get('id'): j for j in root.findall('junction')}
    gates = [(float(J[g].get('x')), float(J[g].get('y'))) for g in GATES]
    inside = lambda p: min(math.dist(p, g) for g in gates) <= RADIUS
    rep, todo = {}, []
    for eid, node, pieces, zi, mode in SPLITS:
        if all(p in E for p in pieces):
            continue
        shape = _rd(E[eid].get('shape') or E[eid].find('lane').get('shape'))
        L = _length(shape)
        if mode == 'hatch':
            cut = project(shape, hatch)                         # 안전지대 끝에서 길에 수직인 선
        else:
            ss = [k*0.5 for k in range(int(L/0.5) + 1) if inside(at(shape, k*0.5))]
            cut = min(ss) if zi == 1 else max(ss)               # 구역 쪽이 끝(1)이면 처음 들어오는 곳, 시작(0)이면 마지막
        if not (5.0 < cut < L - 5.0):
            raise RuntimeError(f'{eid}: 300m 경계 {cut:.1f} 가 엣지 끝과 너무 가까움 (길이 {L:.1f})')
        todo.append((eid, [(node, cut)], pieces))
        rep[eid] = round(cut, 1)
    if todo:
        graft(net_path, {}, [], todo)
    return rep


def _rd(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _wr(pts):
    return ' '.join(f'{x:.3f},{y:.3f}' for x, y in pts)


def _at(pts, s):
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        L = math.dist(a, b)
        if acc + L >= s and L > 0:
            t = (s - acc)/L
            return (a[0] + (b[0] - a[0])*t, a[1] + (b[1] - a[1])*t), ((b[0] - a[0])/L, (b[1] - a[1])/L)
        acc += L
    a, b = pts[-2], pts[-1]
    L = math.dist(a, b) or 1.0
    return b, ((b[0] - a[0])/L, (b[1] - a[1])/L)


def set_speeds(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    want = {e for z in ZONES for e in z[2]}
    found, changed = set(), 0
    for e in root.findall('edge'):
        if e.get('id') in want:
            found.add(e.get('id'))
            for l in e.findall('lane'):
                if abs(float(l.get('speed')) - SPEED) > 0.01:
                    l.set('speed', f'{SPEED:.2f}')
                    changed += 1
    missing = sorted(want - found)
    if missing:
        raise RuntimeError(f'스쿨존 엣지 없음: {missing}')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return changed


def _road_text(lines, fn, h_m=1.6, stretch=2.2):
    """흰 노면 문자 그림 (투명 바탕, 진행 방향으로 길쭉하게). 위 = 먼 쪽."""
    from PIL import Image, ImageDraw
    from env.roundabout_shop_buildings import _font, PPM
    m = lambda v: int(round(v*PPM*2))           # 노면 문자는 두 배 해상도
    f = _font(m(h_m))
    tmp = ImageDraw.Draw(Image.new('RGBA', (1, 1)))
    boxes = [tmp.textbbox((0, 0), t, font=f) for t in lines]
    gap = m(0.6)
    W = max(b[2] - b[0] for b in boxes) + m(0.2)
    H = sum(b[3] - b[1] for b in boxes) + gap*(len(lines) - 1) + m(0.2)
    img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    y = m(0.1)
    for t, b in zip(lines, boxes):
        d.text(((W - (b[2] - b[0]))//2 - b[0], y - b[1]), t, font=f, fill=(255, 255, 255, 235))
        y += b[3] - b[1] + gap
    img = img.resize((W, int(H*stretch)))
    img.save(os.path.join(IMG_DIR, fn))
    return img.width/(PPM*2), img.height/(PPM*2)


def _num_mark(fn, d_m=2.4):
    """'30' 속도제한 노면 표시 (흰 원 테 + 숫자, 진행 방향으로 길쭉하게)."""
    from PIL import Image, ImageDraw
    from env.roundabout_shop_buildings import _font, PPM
    k = PPM*2
    W = int(d_m*k)
    img = Image.new('RGBA', (W, W), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.ellipse([2, 2, W - 3, W - 3], outline=(255, 255, 255, 235), width=max(2, int(0.15*k)))
    f = _font(int(d_m*0.5*k))
    b = d.textbbox((0, 0), '30', font=f)
    d.text(((W - (b[2] - b[0]))//2 - b[0], (W - (b[3] - b[1]))//2 - b[1]), '30', font=f, fill=(255, 255, 255, 235))
    img = img.resize((W, int(W*1.8)))
    img.save(os.path.join(IMG_DIR, fn))
    return img.width/k, img.height/k


def _camera(fn):
    """과속 단속 카메라 (기둥 위 상자, 위에서 본 모양 + 글자)."""
    from env.roundabout_nw_buildings import sign_stack
    img = sign_stack(['과속단속 카메라'], (30, 30, 34), font_m=1.2)
    img.save(os.path.join(IMG_DIR, fn))
    from env.roundabout_shop_buildings import PPM
    return img.width/PPM, img.height/PPM


def draw(net_path, add_path=ADD):
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    from env.roundabout_nw_buildings import sign_stack
    from env.roundabout_shop_buildings import PPM
    net = ET.parse(net_path).getroot()
    E = {e.get('id'): e for e in net.findall('edge')}
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- env/roundabout_school_zone.py 가 생성. 어린이보호구역 적색 포장·노면 표시·표지·단속 카메라 (화면 표시 전용) -->',
           '<additional>']
    tw, th = _road_text(['어린이', '보호구역'], 'sz_text.png')
    nw, nh = _num_mark('sz_30.png')
    cw, ch = _camera('sz_cam.png')
    rep = {}
    for key, name, edges, entries in ZONES:
        # 적색 포장: 차로마다 (끝은 평평하게), 엣지마다 하나로
        n_red = 0
        for eid in edges:
            g = unary_union([LineString(_rd(l.get('shape'))).buffer(float(l.get('width', 3.2))/2, cap_style=2)
                             for l in E[eid].findall('lane')])
            for k, p in enumerate(getattr(g, 'geoms', [g])):
                out.append(f'    <poly id="sz_{key}_red_{eid}_{k}" type="school_zone" color="{RED}" fill="1" '
                           f'layer="{L_RED}" shape="{_wr(p.exterior.coords)}"/>')
                n_red += 1
        # 교차점 바닥: 닿는 엣지가 모두 구역 안이면 교차점도 적색 포장 (회색으로 끊기지 않게)
        zs = set(edges)
        touch = {}
        for e in E.values():
            if e.get('function') == 'internal':
                continue
            for n in (e.get('from'), e.get('to')):
                touch.setdefault(n, set()).add(e.get('id'))
        for j in net.findall('junction'):
            jid = j.get('id')
            if j.get('type') == 'internal' or not touch.get(jid) or not touch[jid] <= zs or not j.get('shape'):
                continue
            out.append(f'    <poly id="sz_{key}_red_J_{jid}" type="school_zone" color="{RED}" fill="1" '
                       f'layer="{L_RED}" shape="{_wr(_rd(j.get("shape")))}"/>')
            n_red += 1
        assert set(entries) <= set(edges), entries
        for eid in entries:
            lanes = E[eid].findall('lane')
            for l in lanes:
                pts = _rd(l.get('shape'))
                for s, img, (w, h) in ((TEXT_S, 'sz_text.png', (tw, th)), (NUM_S, 'sz_30.png', (nw, nh))):
                    (x, y), d = _at(pts, s)
                    ang = math.degrees(math.atan2(d[0], d[1]))      # 그림 위쪽 = 진행 방향 (시계 방향 각)
                    sc = min(1.0, (float(l.get('width', 3.2)) - 0.6)/w)
                    out.append(f'    <poi id="sz_{key}_{img[3:-4]}_{l.get("id")}" type="road_marking" color="255,255,255" '
                               f'layer="{L_TEXT}" x="{x:.3f}" y="{y:.3f}" angle="{ang:.2f}" width="{w*sc:.2f}" '
                               f'height="{h*sc:.2f}" imgFile="shop_img/{img}"/>')
            # 길가(맨 오른쪽 차로 바깥): 보호구역 표지 + 단속 카메라
            pts = _rd(lanes[0].get('shape'))
            (x, y), d = _at(pts, SIGN_S)
            r = (d[1], -d[0])
            off = float(lanes[0].get('width', 3.2))/2 + 2.5
            img = sign_stack(['어린이보호구역', '30'], [(225, 170, 20), (200, 40, 40)], font_m=1.3)
            fn = f'sz_sign_{key}.png'
            img.save(os.path.join(IMG_DIR, fn))
            sx, sy = x + r[0]*off - d[0]*2.0, y + r[1]*off - d[1]*2.0
            out.append(f'    <poi id="sz_{key}_sign_{eid}" type="sign" color="255,255,255" layer="{L_SIGN}" x="{sx:.3f}" '
                       f'y="{sy:.3f}" angle="0.00" width="{img.width/PPM:.2f}" height="{img.height/PPM:.2f}" '
                       f'imgFile="shop_img/{fn}"/>')
            (x, y), d = _at(pts, SIGN_S + 12.0)
            out.append(f'    <poi id="sz_{key}_cam_{eid}" type="camera" color="255,255,255" layer="{L_SIGN}" '
                       f'x="{x + r[0]*off:.3f}" y="{y + r[1]*off:.3f}" angle="0.00" width="{cw:.2f}" height="{ch:.2f}" '
                       f'imgFile="shop_img/sz_cam.png"/>')
        rep[key] = dict(edges=len(edges), red=n_red, entries=entries)
    out.append('</additional>')
    with open(add_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(out) + '\n')
    return rep


def apply(net_path, deco_path=None):
    rep = {'school_zone_cut_m': split_zone_ends(net_path, deco_path)}
    rep['school_zone_lanes_30'] = set_speeds(net_path)
    rep['school_zone'] = draw(net_path)
    return rep


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(apply(os.path.join(NET_DIR, 'roundabout.net.xml')))
