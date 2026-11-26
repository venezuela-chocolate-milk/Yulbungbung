"""상가 골목(env.roundabout_shops) 블록에 상가 건물과 보도를 그린다. (화면 표시 전용)

    python -m env.roundabout_shop_buildings

  - 보도: 골목·큰길로 둘러싸인 블록 전체를 보도블록 색으로 깐다 (도로 밑, 도로가 위에 그려진다).
  - 상가: 위에서 본 평지붕 건물(파라펫) + 길 쪽 차양 띠 + 한글 간판. 주차장은 주차 칸 + P 간판.
    모든 건물은 앞면(이미지 아래쪽)이 큰길·골목 쪽(t 가 작은 쪽)을 향한다.
  - 위치·크기는 env.roundabout_shops 의 국소 좌표(s: 큰길 따라, t: 큰길 남쪽 가장자리에서, m)로 정한다.
    네이버 지도(1px ≈ 0.47m)에서 잰 상가 폭(약 25~35m)·깊이(약 15~30m)에 맞췄다.
출력: env/sumo_roundabout/roundabout.shops.add.xml, env/sumo_roundabout/shop_img/*.png
"""
import math
import os
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
IMG_DIR = os.path.join(NET_DIR, 'shop_img')
OUT = os.path.join(NET_DIR, 'roundabout.shops.add.xml')
PPM = 10                        # 이미지 해상도 (픽셀/m)
FONTS = [r'C:\Windows\Fonts\malgunbd.ttf', '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
         '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc']
PAVING = '204,200,192'
# (키, 간판, 종류, s0, s1, t0, t1, 지붕색, 간판색)
SHOPS = [
    # ('lalacost', '라라코스트', 'shop', 28, 46, 6, 20, (196, 190, 180), (190, 60, 50)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones)
    # ('oliveyoung', '올리브영', 'shop', 50, 72, 3, 19, (214, 210, 200), (120, 170, 40)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones)
    # ('hyodam', '효담요양원', 'shop', 87, 118, 3, 19, (205, 200, 192), (200, 70, 80)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones MAIN_BLOCKS)
    # ('subway', '써브웨이', 'shop', 121, 153, 3, 18, (188, 192, 186), (0, 130, 70)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones MAIN_BLOCKS)
    # ('twosome', '투썸플레이스', 'shop', 167, 198, 3, 19, (210, 204, 196), (170, 30, 40)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones MAIN_BLOCKS)
    # ('suhan', '검단수한방병원', 'shop', 201, 233, 3, 19, (220, 218, 212), (200, 40, 50)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones MAIN_BLOCKS)
    ('breeze', '호텔 브리즈', 'shop', 87, 120, 31, 63, (182, 186, 194), (120, 70, 160)),
    ('parking_b1', '주차장', 'parking', 123, 153, 32, 62, None, (40, 90, 200)),
    ('daynight', '데이앤나잇', 'shop', 167, 199, 31, 63, (196, 186, 200), (140, 60, 170)),
    ('browndot', '브라운도트', 'shop', 202, 233, 31, 63, (200, 188, 172), (120, 80, 50)),
    # ('myeongpum', '명품대패', 'shop', 46, 72, 46, 60, (198, 192, 184), (200, 90, 40)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones)
    # ('cu', 'CU', 'shop', 22, 42, 57, 70, (210, 210, 214), (110, 50, 150)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones)
    # ('parking_w', '공영주차장', 'parking', 46, 72, 63, 84, None, (40, 90, 200)),   ← 2026-11-11 네이버 배치로 옮김 (env.roundabout_shop_zones)
]
# 보도를 깔 범위 (국소 좌표): 남서쪽 큰길 ~ C3 골목, 큰길 ~ D2 골목. D2 남쪽은 117번길 서쪽 블록만
AREA = [(5, 0), (243.2, 0), (243.2, 73.2), (74.3, 73.2), (74.3, 88), (-39, 88)]   # 서쪽 변은 남서쪽 큰길 따라


def _font(px):
    for f in FONTS:
        if os.path.exists(f):
            return ImageFont.truetype(f, px)
    return ImageFont.load_default()


def _sign(img, text, color, center, ang, m):
    """화면에서 가로로 읽히는 간판 (이미지가 ang 만큼 돌아가므로 반대로 돌려 붙인다)."""
    f = _font(m(2.2))
    d = ImageDraw.Draw(img)
    tb = d.textbbox((0, 0), text, font=f)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    px, py = m(0.7), m(0.5)
    sign = Image.new('RGBA', (tw + 2*px, th + 2*py), (0, 0, 0, 0))
    sd = ImageDraw.Draw(sign)
    sd.rounded_rectangle([0, 0, sign.width - 1, sign.height - 1], radius=m(0.4), fill=color + (245,),
                         outline=(255, 255, 255, 255), width=max(1, m(0.12)))
    sd.text((px - tb[0], py - tb[1]), text, font=f, fill=(255, 255, 255, 255))
    sign = sign.rotate(ang, expand=True, resample=Image.BICUBIC)
    img.alpha_composite(sign, (int(center[0] - sign.width/2), int(center[1] - sign.height/2)))


def draw_shop(key, name, kind, w, h, roof, color, ang, sign_y=None):
    """sign_y: 간판 세로 위치 (이미지 위에서 비율, None 이면 앞면 차양 바로 뒤)."""
    m = lambda v: int(round(v*PPM))
    W, H = m(w), m(h)
    img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    if kind == 'parking':
        d.rectangle([0, 0, W-1, H-1], fill=(92, 92, 96, 255), outline=(150, 150, 150, 255), width=m(0.3))
        stall, depth = 2.5, 5.0
        for row_y in (1.0, h - 1.0 - depth):            # 위아래 두 줄 주차 칸
            x = 1.0
            while x + stall <= w - 1.0:
                d.line([m(x), m(row_y), m(x), m(row_y + depth)], fill=(240, 240, 240, 255), width=max(1, m(0.12)))
                x += stall
            d.line([m(x), m(row_y), m(x), m(row_y + depth)], fill=(240, 240, 240, 255), width=max(1, m(0.12)))
        # 입구 (아래쪽 가운데)
        d.rectangle([W//2 - m(3), H - m(0.4), W//2 + m(3), H], fill=(120, 120, 124, 255))
        _sign(img, f'P {name}', color, (W/2, H/2), ang, m)
        return img
    # 평지붕 + 파라펫 + 테두리 (env.roundabout_building_style, 2026-11-11 시뮬레이터 스타일로 통일)
    from env.roundabout_building_style import draw_roof
    draw_roof(d, [0, 0, W-1, H-1], roof, m)
    # (옥상 설비 도형은 넣지 않는다 — 사용자 요청 2026-11-10)
    # 길 쪽(아래) 차양 띠
    d.rectangle([m(0.4), H - m(1.6), W - m(0.4), H - m(0.4)], fill=color + (255,))
    for x in range(m(1.0), W - m(1.0), m(1.2)):
        d.line([x, H - m(1.6), x, H - m(0.4)], fill=tuple(min(255, c + 40) for c in color) + (255,), width=1)
    _sign(img, name, color, (W/2, H*sign_y if sign_y is not None else (H - m(4.0) if h > 10 else H/2)), ang, m)
    return img


def build(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), out=OUT):
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    from env.roundabout_shops import Frame
    net = ET.parse(net_path).getroot()
    F = Frame(net)
    os.makedirs(IMG_DIR, exist_ok=True)
    ang = math.degrees(math.atan2(F.v[0], F.v[1]))          # 이미지 위쪽 = +t (큰길·골목 반대쪽)
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_shop_buildings.py 가 생성. 상가 골목 보도·상가 그림 (주행에는 영향 없음) -->',
             '<additional>']
    # 보도: 범위에서 도로(차로·교차점)를 뺀 블록 조각
    area = Polygon([F.P(s, t) for s, t in AREA])
    roads = []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = [tuple(map(float, p.split(','))) for p in l.get('shape').split()]
            if len(pts) >= 2:
                roads.append(LineString(pts).buffer(float(l.get('width', '3.2'))/2, cap_style=2))
    for j in net.findall('junction'):
        s = j.get('shape')
        if s and j.get('type') != 'internal':
            pts = [tuple(map(float, p.split(','))) for p in s.split()]
            if len(pts) >= 3:
                roads.append(Polygon(pts).buffer(0))
    blocks = area.difference(unary_union([r for r in roads if r.intersects(area)]))
    n_block = 0
    for k, b in enumerate(getattr(blocks, 'geoms', [blocks])):
        if b.area < 150:
            continue
        lines.append(f'    <poly id="shop_paving_{k}" type="sidewalk" color="{PAVING}" fill="1" layer="-2" '
                     f'shape="{" ".join(f"{x:.3f},{y:.3f}" for x, y in b.exterior.coords)}"/>')
        n_block += 1
    for key, name, kind, s0, s1, t0, t1, roof, color in SHOPS:
        w, h = s1 - s0, t1 - t0
        cx, cy = F.P((s0 + s1)/2, (t0 + t1)/2)
        fn = f'shop_{key}.png'
        draw_shop(key, name, kind, w, h, roof, color, ang).save(os.path.join(IMG_DIR, fn))
        if kind != 'parking':          # 건물 그림자 (주차장은 바닥이라 없음)
            from env.roundabout_building_style import shadow_poly
            lines.append(shadow_poly(f'shop_{key}_shadow', [F.P(s0, t0), F.P(s1, t0), F.P(s1, t1), F.P(s0, t1)], '-1.6'))
        lines.append(f'    <poi id="shop_{key}" type="{kind}" color="255,255,255" layer="-1" x="{cx:.3f}" y="{cy:.3f}" '
                     f'angle="{ang:.2f}" width="{w}" height="{h}" imgFile="shop_img/{fn}"/>')
    # 검단사거리 남쪽 상가 블록: 네이버 지도 배치대로 구역 + 건물 (2026-11-11)
    from env.roundabout_shop_zones import build_lines
    zl, zrep = build_lines(net, IMG_DIR)
    lines.extend(zl)
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    return {'shop_blocks': n_block, 'shop_buildings': len(SHOPS), 'shop_zones': zrep, 'out': out}


def check_overlap(net_path=os.path.join(NET_DIR, 'roundabout.net.xml')):
    """상가 건물이 도로(차로·교차점)와 겹치는지 확인 (겹친 면적 m²)."""
    from shapely.geometry import LineString, Polygon
    from env.roundabout_shops import Frame
    net = ET.parse(net_path).getroot()
    F = Frame(net)
    roads = []
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = [tuple(map(float, p.split(','))) for p in l.get('shape').split()]
            if len(pts) >= 2:
                roads.append(LineString(pts).buffer(float(l.get('width', '3.2'))/2 + 1.0, cap_style=2))
    out = {}
    for key, _, _, s0, s1, t0, t1, _, _ in SHOPS:
        b = Polygon([F.P(s0, t0), F.P(s1, t0), F.P(s1, t1), F.P(s0, t1)])
        a = sum(b.intersection(r).area for r in roads if r.intersects(b))
        if a > 0:
            out[key] = round(a, 2)
    return out


if __name__ == '__main__':
    import sys
    sys.path.insert(0, BASE)
    print(build())
    print('도로(+1m)와 겹침:', check_overlap() or '없음')
