"""북서쪽 상가 골목(env.roundabout_nw_shops) 블록(①~⑧)을 이름 있는 가게 건물로 채운다. (화면 표시 전용)

    python -m env.roundabout_nw_buildings

  - 블록: 검단로·445번길·완정로·서곶로로 둘러싸인 구역에서 도로(+보도 SIDEWALK m)와 종다리공원을 뺀 조각.
    도로가 블록 경계로만 지나므로 조각 하나가 네이버 지도의 건물 덩어리 하나다 (CLUSTERS 의 at 점으로 찾는다).
  - 건물: 네이버 지도(2026-11-10 캡처들)에서 한 건물에 든 가게들을 한 건물로 묶고, 그 아이콘 위치를 블록 안으로
    옮겨(덩어리 안 상대 배치 유지) 보로노이로 블록을 나눈다. 나눈 칸 모양 그대로 건물을 세워 블록이 꽉 찬다.
    이름 없는 건물은 따로 그리지 않는다 (이웃 건물이 그 자리를 채운다). 간판은 건물 위에 화면 기준으로 바로 세운다.
  - 2026-11-11: 위 칸은 '구역'으로 그대로 두고, 구역 안에 가게 이름 하나당 네모 건물 하나를 격자로 넣는다
    (env.roundabout_building_style 의 지붕·파라펫·테두리·그림자). 간판도 건물마다 하나.
  - 종다리공원: ④·⑤ 블록 서쪽을 따라 길게, 가운데에 R4 회차로(도로가 위에 그려진다).
출력: env/sumo_roundabout/roundabout.nw.add.xml, env/sumo_roundabout/shop_img/nw_*.png
"""
import math
import os
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

from env.roundabout_building_style import ROOF_PALETTE, PARAPET_INSET, polys as style_polys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
OUT = os.path.join(NET_DIR, 'roundabout.nw.add.xml')
PAVING = '204,200,192'
SIDEWALK = 2.5          # 차로 가장자리 ~ 건물
GAP = 1.5               # 건물 사이
MAX_W, MAX_H = 48.0, 40.0
# 구역(칸) 안 건물 (2026-11-11): 구역은 그대로 두고, 그 안에 이름 하나당 네모 건물 하나
LOT_MARGIN = 1.5        # 구역 가장자리 ~ 건물
BLD_GAP = 2.0           # 건물 사이
BLD_MAX = (24.0, 18.0)  # 건물 최대 (s 방향, t 방향) m — 칸이 커도 이 이상은 키우지 않는다
# 캡처마다 픽셀 → 기준 캡처(2026-11-10 첫 캡처, 1px ≈ 0.307m) 픽셀
PIX = {17: lambda x, y: (x, y),
       19: lambda x, y: ((x - 213) / 2 + 403, (y - 195) / 2 + 268),
       20: lambda x, y: ((x - 628) / 1.48 + 403, (y - 272) / 1.48 + 268),
       21: lambda x, y: ((x - 596) / 1.364 + 388, (y - 62) / 1.364 + 330)}
R, O, G, B, P_, Y, BR, GR = ((200, 40, 60), (210, 100, 30), (40, 140, 90), (30, 80, 160), (120, 70, 160),
                             (220, 170, 20), (150, 90, 40), (90, 90, 120))
# 블록(덩어리): at = 블록 안 대표점(국소 s, t),
# buildings = (키, [간판들], [(캡처, x, y) 아이콘들], 간판색). 네이버 지도에서 한 건물에 든 가게들은 한 건물로 묶는다.
CLUSTERS = {
    1: dict(at=(172, 168), buildings=[
        ('onnuri_h', ['온누리병원'], [(17, 92, 28)], R), ('hitel', ['온누리하이텔'], [(17, 185, 82)], P_),
        ('holrala', ['홀랄라숯불치킨'], [(17, 195, 122)], O), ('119', ['왕길119안전센터'], [(17, 328, 135)], R)]),
    2: dict(at=(225, 125), buildings=[
        ('nuri', ['누리약국'], [(17, 88, 158)], G), ('sutbul', ['숯불에닭'], [(17, 40, 255)], O),
        ('hanam', ['하남돼지집'], [(17, 40, 325)], BR)]),
    3: dict(at=(180, 100), buildings=[
        ('jeongga', ['정가며느리불태기', '신마포갈매기'], [(20, 250, 175), (20, 232, 236)], BR),
        ('artvill', ['인천아트빌', '지천산업개발'], [(20, 265, 342), (20, 273, 288)], GR),
        ('parktower', ['자동식주차타워'], [(20, 378, 238)], B),
        ('wonhalmeoni', ['원할머니보쌈족발'], [(20, 428, 316)], R)]),
    4: dict(at=(195, 62), buildings=[
        ('hangari', ['항아리보쌈'], [(21, 188, 150)], BR), ('ocelot', ['오셀롯PC'], [(21, 293, 172)], B),
        ('gogi', ['고기굽는방앗간'], [(21, 376, 152)], O), ('golden', ['7080골든타임'], [(21, 465, 148)], O),
        ('beer', ['역전할머니맥주'], [(21, 527, 187)], Y), ('yonsei', ['검단연세요양병원'], [(21, 285, 282)], R)]),
    5: dict(at=(195, 20), buildings=[
        ('ppuri', ['뿌리산업외국인근로자센터'], [(21, 336, 446)], B),
        ('live', ['리브동물의료센터', '돼지네'], [(21, 493, 385), (21, 415, 412)], (200, 70, 120)),
        ('goemul', ['괴물아구찜'], [(21, 597, 325)], O)]),
    6: dict(at=(128, 58), buildings=[
        ('seven', ['한신우동', '세븐일레븐', '황금짬뽕', '상구네솥뚜껑삼겹살'],
         [(19, 213, 195), (19, 183, 320), (19, 195, 365), (19, 240, 467)], G),
        ('majeon', ['마전복합타워', '엘스테이오피스텔', '이리스', '쥬씨', '아리오돌뼈신닭발', '왔다갔다황소곱창'],
         [(19, 375, 282), (19, 450, 312), (19, 345, 348), (19, 510, 343), (19, 368, 410), (19, 440, 392)], GR)]),
    7: dict(at=(95, 100), buildings=[
        ('dain', ['검단다인안과의원', '한솥도시락'], [(17, 430, 155), (17, 512, 155)], R),
        ('haemul', ['해물촌황태마을'], [(19, 410, 48)], (30, 120, 130)),
        ('police', ['검단지구대'], [(19, 612, 121)], B),
        ('sc', ['SC제일은행', '보성메디컬약국', '정정형외과의원'], [(19, 800, 140), (19, 768, 88), (19, 748, 168)], B)]),
    8: dict(at=(120, 18), buildings=[
        ('jogae', ['검단조개왕국', '모브', '이삭토스트', '와플대학'],
         [(19, 690, 425), (19, 672, 480), (19, 727, 488), (19, 640, 522)], B),
        ('dessert', ['디저트39', '검단미래안약국'], [(19, 825, 375), (19, 805, 455)], P_),
        ('yongyong', ['용용선생'], [(19, 543, 583)], R),
        ('insaeng', ['인생맥주', '현대프라자', '이마트24'], [(19, 340, 605), (19, 405, 632), (19, 325, 660)], Y),
        ('lotteria', ['롯데리아'], [(17, 747, 378)], R)]),
}
ROOFS = [(214, 210, 202), (204, 198, 188), (196, 192, 186), (210, 206, 198), (200, 200, 196), (190, 196, 206),
         (206, 210, 214), (222, 218, 212)]
PARK = ('jongdari', '종다리공원', (224.0, 245.0, 3.0, 96.0))   # ④·⑤ 블록 서쪽 (국소 s0, s1, t0, t1)


def draw_park(name, w, h, ang):
    """길쭉한 공원: 잔디, 나무, 산책로 (가운데 회차로는 도로가 위에 그려진다)."""
    from env.roundabout_shop_buildings import _sign, PPM
    m = lambda v: int(round(v*PPM))
    W, H = m(w), m(h)
    img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([0, 0, W-1, H-1], radius=m(2), fill=(118, 162, 92, 255), outline=(80, 120, 70, 255),
                        width=m(0.4))
    d.line([W//2, m(1), W//2, H - m(1)], fill=(214, 200, 168, 255), width=m(1.4))      # 긴 산책로
    d.rounded_rectangle([m(3), m(h*0.08), W - m(3), m(h*0.08) + m(12)], radius=m(1), fill=(150, 190, 120, 255),
                        outline=(220, 230, 210, 255), width=max(1, m(0.2)))             # 작은 운동장
    for k in range(int(h // 7)):
        y = m(4 + k*7)
        for x in (m(2.6), W - m(2.6)):
            r = m(2.0)
            d.ellipse([x - r, y - r, x + r, y + r], fill=(60, 110, 58, 255), outline=(42, 80, 42, 255))
    _sign(img, name, (60, 130, 70), (W/2, H*0.82), ang, m)
    return img


def _max_rect(cell_st, res=0.5):
    """국소 (s, t) 다각형 안에 들어가는 축 정렬 최대 사각형 (s0, s1, t0, t1)."""
    from shapely.geometry import Point
    from shapely.prepared import prep
    s0, t0, s1, t1 = cell_st.bounds
    nx, ny = int((s1 - s0)/res), int((t1 - t0)/res)
    if nx < 2 or ny < 2:
        return None
    pc = prep(cell_st)
    grid = [[pc.contains(Point(s0 + (i + 0.5)*res, t0 + (j + 0.5)*res)) for i in range(nx)] for j in range(ny)]
    best, heights = (0, None), [0]*nx
    for j in range(ny):
        for i in range(nx):
            heights[i] = heights[i] + 1 if grid[j][i] else 0
        stack = []
        for i in range(nx + 1):
            h = heights[i] if i < nx else 0
            start = i
            while stack and stack[-1][1] >= h:
                k, hk = stack.pop()
                area = hk * (i - k)
                if area > best[0]:
                    best = (area, (k, i, j - hk + 1, j + 1))
                start = k
            stack.append((start, h))
    if best[1] is None:
        return None
    a, b, c, d = best[1]
    return (s0 + a*res, s0 + b*res, t0 + c*res, t0 + d*res)


def sign_stack(names, color, font_m=2.0):
    """가게 간판들을 한 줄씩 쌓은 그림 (돌리지 않음 = 화면에서 바로 읽힘). font_m: 글자 높이 (m).
    color: 하나(모든 간판 같은 색) 또는 간판마다 색 목록."""
    from env.roundabout_shop_buildings import _font, PPM
    m = lambda v: int(round(v*PPM))
    f = _font(m(font_m))
    tmp = ImageDraw.Draw(Image.new('RGBA', (1, 1)))
    boxes = [tmp.textbbox((0, 0), nm, font=f) for nm in names]
    px, py, gap = m(0.6), m(0.4), m(0.35)
    ws = [b[2] - b[0] + 2*px for b in boxes]
    hs = [b[3] - b[1] + 2*py for b in boxes]
    W, H = max(ws), sum(hs) + gap*(len(names) - 1)
    img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    y = 0
    cols = color if isinstance(color, list) else [color]*len(names)
    for nm, b, w, h, col in zip(names, boxes, ws, hs, cols):
        x = (W - w) // 2
        d.rounded_rectangle([x, y, x + w - 1, y + h - 1], radius=m(0.4), fill=col + (245,),
                            outline=(255, 255, 255, 255), width=max(1, m(0.12)))
        d.text((x + px - b[0], y + py - b[1]), nm, font=f, fill=(255, 255, 255, 255))
        y += h + gap
    return img


def _lot_buildings(lot, paint_st, n):
    """구역(국소 s, t 다각형) 안에 n 개 네모 건물 (s0, s1, t0, t1): 들어가는 가장 큰 사각형을 격자로 나눈다.
    좁은 구역은 여백을 줄여 다시 시도한다."""
    best = []
    for margin, gap, road in ((LOT_MARGIN, BLD_GAP, 1.5), (0.8, 1.2, 1.0), (0.4, 0.8, 0.6)):
        out = _lot_try(lot, paint_st, n, margin, gap, road)
        if len(out) > len(best):
            best = out
        if len(best) >= n:
            break
    return best


def _lot_try(lot, paint_st, n, margin, gap, road):
    free = lot.buffer(-margin).difference(paint_st.buffer(road))
    if free.is_empty:
        return []
    if free.geom_type != 'Polygon':
        free = max(free.geoms, key=lambda g: g.area)
    grid, greedy = _lot_grid(free, n, gap), _lot_greedy(free, n, gap)
    score = lambda rs: (len(rs), min(((b - a)*(d - c) for a, b, c, d in rs), default=0))
    return max((grid, greedy), key=score)          # 다 들어가면서 가장 작은 건물이 더 큰 쪽


def _lot_greedy(free, n, gap):
    """남은 빈 곳에서 가장 큰 사각형을 차례로 골라 건물 하나씩 (한 건물이 구역을 독차지하지 않게 몫만큼만)."""
    from shapely.geometry import box
    share = math.sqrt(free.area * 0.5 / n)          # 건물 하나 몫의 한 변
    out, rest = [], free
    for _ in range(n):
        if rest.is_empty:
            break
        part = max(getattr(rest, 'geoms', [rest]), key=lambda g: g.area)
        R = _max_rect(part)
        if R is None:
            break
        s0, s1, t0, t1 = R
        w = min(s1 - s0, BLD_MAX[0], max(share*1.4, 6.0))
        h = min(t1 - t0, BLD_MAX[1], max(share*1.1, 5.0))
        if w < 2.0 or h < 2.0:
            break
        cs, ct = (s0 + s1)/2, (t0 + t1)/2
        r = (cs - w/2, cs + w/2, ct - h/2, ct + h/2)
        out.append(r)
        rest = rest.difference(box(r[0], r[2], r[1], r[3]).buffer(gap))
    return out


def _lot_grid(free, n, gap):
    R = _max_rect(free)
    if R is None:
        return []
    s0, s1, t0, t1 = R
    W, H = s1 - s0, t1 - t0
    cols = max(1, min(n, round(math.sqrt(n * W / max(H, 1e-6)))))
    rows = math.ceil(n / cols)
    out = []
    for r in range(rows):
        k = min(cols, n - r*cols)                    # 마지막 줄은 남은 수만큼 넓게
        cw, ch = W / k, H / rows
        for c in range(k):
            a0, a1 = s0 + c*cw + gap/2, s0 + (c + 1)*cw - gap/2
            b0, b1 = t0 + r*ch + gap/2, t0 + (r + 1)*ch - gap/2
            # 칸이 너무 크면 최대 크기로 줄여 칸 가운데에
            w, h = min(a1 - a0, BLD_MAX[0]), min(b1 - b0, BLD_MAX[1])
            if w < 2.0 or h < 2.0:
                continue
            cs, ct = (a0 + a1)/2, (b0 + b1)/2
            out.append((cs - w/2, cs + w/2, ct - h/2, ct + h/2))
    return out


def _assign(rects, pts_per_name):
    """이름마다 건물 하나: 이름 아이콘 위치와 건물 가운데 거리 합이 가장 작게 (이름 수 ≤ 7 이라 전수 탐색)."""
    from itertools import permutations
    names = len(pts_per_name)
    ctr = [((a + b)/2, (c + d)/2) for a, b, c, d in rects]
    pos = list(pts_per_name)                     # 이름마다 아이콘 한 점
    if not rects:
        return [None]*names
    best, best_cost = None, float('inf')
    slots = list(range(len(rects))) + [None]*max(0, names - len(rects))
    for perm in set(permutations(slots, names)):
        cost = sum(math.dist(pos[i], ctr[j]) for i, j in enumerate(perm) if j is not None)
        if cost < best_cost:
            best, best_cost = perm, cost
    return list(best)


def build(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), out=OUT):
    from shapely.geometry import LineString, Polygon, Point, MultiPoint
    from shapely.ops import unary_union, voronoi_diagram
    from env.roundabout_nw_shops import Frame, real_st
    from env.roundabout_shop_buildings import draw_shop
    net = ET.parse(net_path).getroot()
    F = Frame(net)
    os.makedirs(IMG_DIR, exist_ok=True)
    ang = math.degrees(math.atan2(F.n[0], F.n[1]))
    rd = lambda v: [tuple(map(float, p.split(','))) for p in v.split()]
    lanes = {l.get('id'): rd(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    # 구역 경계 (블록 쪽 차로 중심선을 이어서): 검단로 → 445번길 → 완정로 → 서곶로
    # 2026-11-12: 서쪽 경계 = 파란 도로(T_side_out) → 445번길 연장(nw_445c/d) → 사거리(nw_X)에서 완정로
    ring = [F.P(18, -1.6)]
    for lid in ('T_side_out_0', 'nw_445c_f_0', 'nw_445d_f_0'):
        ring += lanes[lid]
    for lid in ('nw_wj4_r_0', 'nw_wj3_r_0', 'nw_wj2_r_0', 'nw_wj1_r_0', 'X_nw_in_0'):
        ring += lanes[lid]
    district = Polygon(ring).buffer(0)
    for jid in ('X_nw_end',):                # 완정로가 넓어지는 바닥 (구역이 넓어진 차로 가장자리까지 닿게)
        xj = next(j for j in net.findall('junction') if j.get('id') == jid)
        district = district.union(Polygon(rd(xj.get('shape'))).buffer(4.0).intersection(
            Polygon(ring).buffer(12.0))).buffer(0)
    xj = next(j for j in net.findall('junction') if j.get('id') == 'X_nw_end')
    roads_paint, roads_keep = [], []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = rd(l.get('shape'))
            if len(pts) < 2:
                continue
            g = LineString(pts)
            if g.distance(district) > 40:
                continue
            w = float(l.get('width', '3.2'))
            roads_paint.append(g.buffer(w/2, cap_style=2))
            if not l.get('id', '').startswith('nw_119'):     # 119 출구는 센터 건물 안에서 시작한다
                roads_keep.append(g.buffer(w/2 + SIDEWALK, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal':
            pts = rd(s)
            if len(pts) >= 3:
                poly = Polygon(pts).buffer(0)
                if poly.distance(district) < 40:
                    roads_paint.append(poly)
                    roads_keep.append(poly.buffer(SIDEWALK))
    paint, keep = unary_union(roads_paint), unary_union(roads_keep)
    from shapely.ops import transform as _tf
    paint_st = _tf(lambda x, y, z=None: tuple(map(list, zip(*[F.st(p) for p in zip(x, y)]))), paint)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_nw_buildings.py 가 생성. 북서쪽 상가 골목 보도·가게·공원 그림 (주행에는 영향 없음) -->',
             '<additional>']
    paving = district.difference(paint)
    n_block = 0
    for k, b in enumerate(getattr(paving, 'geoms', [paving])):
        if b.area < 20:
            continue
        lines.append(f'    <poly id="nw_paving_{k}" type="sidewalk" color="{PAVING}" fill="1" layer="-2" '
                     f'shape="{" ".join(f"{x:.3f},{y:.3f}" for x, y in b.exterior.coords)}"/>')
        n_block += 1
    # 블록 = 구역 - 도로(+보도) - 공원 (국소 좌표)
    pk, pname, (_, _, pt0, pt1) = PARK
    # 2026-11-12: 공원은 새 서쪽 경계(파란 도로) 블록 쪽에 붙여 그 방향으로 기운 띠 (env.roundabout_nw_shops 의 회차로와 같은 자리)
    from env.roundabout_nw_shops import blue_s_at, PARK_W, PARK_GAP
    se0, se1 = blue_s_at(F, net, pt0) - PARK_GAP, blue_s_at(F, net, pt1) - PARK_GAP
    park_st = [(se0 - PARK_W, pt0), (se0, pt0), (se1, pt1), (se1 - PARK_W, pt1)]
    park_poly = Polygon([F.P(*q) for q in park_st])
    tilt = math.atan2(se1 - se0, pt1 - pt0)                 # 띠가 t 축에서 기운 각 (s 쪽으로)
    ps0, ps1 = (se0 + se1) / 2 - PARK_W, (se0 + se1) / 2
    to_st = lambda g: Polygon([F.st(p) for p in g.exterior.coords]).buffer(0)
    build_area = district.difference(keep).difference(park_poly.buffer(1.0))
    blocks = [to_st(b) for b in getattr(build_area, 'geoms', [build_area]) if b.area > 60]
    pois, report, used = [], {}, {}
    for cid, cl in CLUSTERS.items():
        at = Point(*cl['at'])
        bi = min(range(len(blocks)), key=lambda k: 0 if blocks[k].contains(at) else blocks[k].distance(at))
        if bi in used:
            report[f'block{cid}'] = f'블록이 {used[bi]} 번과 겹침'
        used[bi] = cid
        blk = blocks[bi]
        # 덩어리 안 상대 배치 유지: 실제 위치 상자 → 블록 상자(안쪽 3m)로 옮긴다
        real = []
        for _, _, icons, _ in cl['buildings']:
            pts = [real_st(*PIX[src](x, y)) for src, x, y in icons]
            real.append((sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)))
        rs0, rs1 = min(p[0] for p in real), max(p[0] for p in real)
        rt0, rt1 = min(p[1] for p in real), max(p[1] for p in real)
        bs0, bt0, bs1, bt1 = blk.bounds
        bs0, bs1, bt0, bt1 = bs0 + 3, bs1 - 3, bt0 + 3, bt1 - 3
        mp = lambda v, a0, a1, b0, b1: (b0 + b1) / 2 if a1 - a0 < 1e-6 else b0 + (v - a0) / (a1 - a0) * (b1 - b0)
        seeds = [Point(mp(p[0], rs0, rs1, bs0, bs1), mp(p[1], rt0, rt1, bt0, bt1)) for p in real]
        # 가게(이름)마다 아이콘 위치 (같은 옮김) — 구역 안 건물 배정에 쓴다
        name_pts = [[(mp(q[0], rs0, rs1, bs0, bs1), mp(q[1], rt0, rt1, bt0, bt1))
                     for q in (real_st(*PIX[src](x, y)) for src, x, y in icons)] for _, _, icons, _ in cl['buildings']]
        seeds = [sd if blk.contains(sd) else blk.representative_point() if not blk.exterior.distance(sd) < 30
                 else blk.exterior.interpolate(blk.exterior.project(sd)) for sd in seeds]
        if len(seeds) == 1:
            cells = [blk]
        else:
            vd = voronoi_diagram(MultiPoint(seeds), envelope=blk.envelope.buffer(60))
            cells = [min(vd.geoms, key=lambda c: c.distance(sd)).intersection(blk) for sd in seeds]
        for n, ((key, names, _, color), sd, cell) in enumerate(zip(cl['buildings'], seeds, cells)):
            cell = cell.buffer(-GAP/2)
            if cell.is_empty:
                report[key] = 'dropped'
                continue
            if cell.geom_type != 'Polygon':
                cell = max(cell.geoms, key=lambda g: g.area)
            # 건물 = 블록을 나눈 칸 모양 그대로 (블록이 꽉 차게), 지붕색 + 테두리
            cell = cell.simplify(0.4)
            roof = ROOFS[(cid + n) % len(ROOFS)]
            world = [F.P(x, y) for x, y in cell.exterior.coords]
            shp = " ".join(f"{x:.3f},{y:.3f}" for x, y in world)
            lines.append(f'    <poly id="nw_bld_{key}" type="building" color="{",".join(map(str, roof))}" fill="1" '
                         f'layer="-1.5" shape="{shp}"/>')
            # 구역 테두리선은 그리지 않는다 (사용자 요청 2026-11-11)
            # 구역 안 건물: 이름 하나당 네모 건물 하나, 간판도 건물마다 하나
            rects = _lot_buildings(cell, paint_st, len(names))
            if len(rects) < len(names):
                report[f'{key}_short'] = f'{len(rects)}/{len(names)}'
            order = _assign(rects, name_pts[n])
            from env.roundabout_shop_buildings import PPM
            for i, ri in enumerate(order):
                if ri is None:
                    continue
                s0, s1, t0, t1 = rects[ri]
                bid = f'{key}_{i}' if len(names) > 1 else key
                roof = ROOF_PALETTE[(cid*7 + n*3 + i) % len(ROOF_PALETTE)]
                corners = [(s0, t0), (s1, t0), (s1, t1), (s0, t1)]
                k_ = PARAPET_INSET
                inner = [(s0 + k_, t0 + k_), (s1 - k_, t0 + k_), (s1 - k_, t1 - k_), (s0 + k_, t1 - k_)]
                lines.extend(style_polys(f'nw_house_{bid}', [F.P(*c) for c in corners], [F.P(*c) for c in inner], roof))
                fn = f'nw_{bid}.png'
                # 간판은 건물 폭을 넘지 않게 (작은 건물이 붙어 있는 구역에서 간판끼리 겹치지 않도록)
                img = sign_stack([names[i]], color)
                fit = max(s1 - s0, t1 - t0) * 1.05
                if img.width / PPM > fit:
                    img = sign_stack([names[i]], color, font_m=max(1.0, 2.0 * fit / (img.width / PPM)))
                img.save(os.path.join(IMG_DIR, fn))
                pois.append((f'nw_{bid}', 'shop', ((s0 + s1)/2, (t0 + t1)/2), round(img.width / PPM, 2),
                             round(img.height / PPM, 2), fn, 0.0))
            report[key] = (round(cell.area), len(names))
    pw, ph = PARK_W, (pt1 - pt0) / math.cos(tilt)
    pang = ang - math.degrees(tilt)
    fn = f'nw_{pk}.png'
    draw_park(pname, pw, ph, pang).save(os.path.join(IMG_DIR, fn))
    pois.insert(0, (f'nw_{pk}', 'park', ((ps0 + ps1)/2, (pt0 + pt1)/2), pw, ph, fn, pang))
    for pid, kind, (s, t), w, h, fn, a in pois:
        x, y = F.P(s, t)
        lines.append(f'    <poi id="{pid}" type="{kind}" color="255,255,255" layer="-1" x="{x:.3f}" y="{y:.3f}" '
                     f'angle="{a:.2f}" width="{w}" height="{h}" imgFile="shop_img/{fn}"/>')
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    return {'nw_blocks': n_block, 'nw_buildings': len(pois), 'nw_sizes': report, 'out': out}


if __name__ == '__main__':
    import sys
    sys.path.insert(0, BASE)
    print(build())
