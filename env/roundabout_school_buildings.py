"""학교 정문 길(env.roundabout_school_access) 끝에 완정초·마전중 그림을 붙인다. (화면 표시 전용)

    python -m env.roundabout_school_buildings

  - 학교마다 위에서 내려다본 이미지(PNG)를 만들어 SUMO 지점(POI)의 imgFile 로 붙인다.
    부지(담장·나무), 본관(평지붕·파라펫·테두리 + 그림자, 2026-11-11 시뮬레이터 스타일), 운동장(트랙·잔디), 정문, 한글 학교 이름.
    (roof 텍스처 파일은 더 쓰지 않지만 남겨 둔다)
  - 이미지 아래쪽 가운데가 정문이고, 학교 길이 정문을 지나 부지 안에서 끝나도록 놓는다.
  - 지붕 텍스처: OpenGameArt "Tiny Texture Pack" roof 6 / roof 3 (CC0, 저작자 표시 불필요)
      https://opengameart.org/content/tiny-texture-pack-roof-6-256x256png
      https://opengameart.org/content/tiny-texture-pack-roof-3-256x256png
출력: env/sumo_roundabout/roundabout.school.add.xml, env/sumo_roundabout/school_img/*.png
"""
import math
import os
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFilter, ImageFont

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
IMG_DIR = os.path.join(NET_DIR, 'school_img')
OUT = os.path.join(NET_DIR, 'roundabout.school.add.xml')
PPM = 12                       # 이미지 해상도 (픽셀/m)
FONTS = [r'C:\Windows\Fonts\malgunbd.ttf', '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
         '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc']
# gate: 정문 노드 (env.roundabout_school_access.GATES),  size: (폭, 깊이) m
SCHOOLS = [
    dict(key='wanjeong', gate='school_wanjeong_gate', name='완정초등학교', size=(42.0, 36.0),
         roof='roof_red_cc0.png', roof_color=(204, 190, 178), track_side='left'),
    dict(key='majeon', gate='school_majeon_gate', name='마전중학교', size=(46.0, 38.0),
         roof='roof_slate_cc0.png', roof_color=(190, 198, 208), track_side='right'),
]


def _font(px):
    for f in FONTS:
        if os.path.exists(f):
            return ImageFont.truetype(f, px)
    return ImageFont.load_default()


def draw_school(sc, ang=0.0):
    """위에서 본 학교 이미지. 아래쪽 가운데가 정문. ang: SUMO 에 놓일 각도(시계 방향, 도) — 이름표만 반대로 돌려 화면에서 바로 읽히게."""
    W, H = int(sc['size'][0]*PPM), int(sc['size'][1]*PPM)
    m = lambda v: int(round(v*PPM))
    img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    # 부지 + 담장
    d.rounded_rectangle([0, 0, W-1, H-1], radius=m(2), fill=(214, 210, 200, 255), outline=(70, 90, 70, 255),
                        width=m(0.5))
    # 정문 (폭 7m) + 정문 기둥
    gw = m(7.0)
    d.rectangle([W//2 - gw//2, H - m(0.6), W//2 + gw//2, H], fill=(150, 150, 150, 255))
    for x in (W//2 - gw//2, W//2 + gw//2):
        d.rectangle([x - m(0.5), H - m(1.2), x + m(0.5), H], fill=(120, 110, 100, 255))
    # 정문 → 본관 포장길
    d.rectangle([W//2 - m(2.5), m(13), W//2 + m(2.5), H - m(0.6)], fill=(176, 172, 164, 255))
    # 운동장: 트랙(붉은 우레탄) + 잔디, 정문길 한쪽 아래
    tw, th = sc['size'][0]*0.42, sc['size'][1]*0.52
    x0 = m(2.0) if sc['track_side'] == 'left' else W - m(2.0) - m(tw)
    y0 = H - m(2.0) - m(th)
    box = [x0, y0, x0 + m(tw), y0 + m(th)]
    d.rounded_rectangle(box, radius=m(min(tw, th)/2), fill=(176, 84, 62, 255))
    inner = [box[0] + m(2.6), box[1] + m(2.6), box[2] - m(2.6), box[3] - m(2.6)]
    d.rounded_rectangle(inner, radius=m(min(tw, th)/2 - 2.6), fill=(96, 150, 82, 255))
    for k in (0.9, 1.7):
        b = [box[0] + m(k), box[1] + m(k), box[2] - m(k), box[3] - m(k)]
        d.rounded_rectangle(b, radius=m(min(tw, th)/2 - k), outline=(240, 236, 228, 255), width=max(1, m(0.12)))
    # 나무 (담장 안쪽 둘레)
    other = 'right' if sc['track_side'] == 'left' else 'left'
    tx0 = W - m(2.0) - m(tw) if other == 'right' else m(2.0)
    for i in range(4):
        cx = tx0 + m(2.0) + i*(m(tw) - m(4.0))//3
        cy = H - m(3.0)
        r = m(1.6)
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(58, 104, 58, 255), outline=(40, 76, 40, 255))
    # 본관 (ㄷ자 대신 길쭉한 본관 + 짧은 별관), 그림자 → 지붕 텍스처 → 테두리
    bx0, by0, bx1, by1 = m(3.0), m(2.5), W - m(3.0), m(12.0)
    wing = ([bx0, by1 - m(1), bx0 + m(9), by1 + m(8)] if other == 'left'
            else [bx1 - m(9), by1 - m(1), bx1, by1 + m(8)])
    shadow = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow)
    off = m(0.9)
    for r in ([bx0, by0, bx1, by1], wing):
        sd.rectangle([r[0] + off, r[1] + off, r[2] + off, r[3] + off], fill=(0, 0, 0, 110))
    img.alpha_composite(shadow.filter(ImageFilter.GaussianBlur(m(0.5))))
    # 평지붕 + 파라펫 + 테두리 (env.roundabout_building_style, 2026-11-11 시뮬레이터 스타일로 통일)
    from env.roundabout_building_style import draw_roof
    for r in ([bx0, by0, bx1, by1], wing):
        draw_roof(d, r, sc['roof_color'], m)
    # 학교 이름 (본관 앞 마당, 흰 바탕 간판)
    f = _font(m(2.6))
    tb = d.textbbox((0, 0), sc['name'], font=f)
    tw_, th_ = tb[2] - tb[0], tb[3] - tb[1]
    pad_x, pad_y = m(0.8), m(0.7)
    sign = Image.new('RGBA', (tw_ + 2*pad_x, th_ + 2*pad_y), (0, 0, 0, 0))
    sdr = ImageDraw.Draw(sign)
    sdr.rounded_rectangle([0, 0, sign.width - 1, sign.height - 1], radius=m(0.5), fill=(250, 250, 246, 240),
                          outline=(40, 70, 130, 255), width=max(1, m(0.15)))
    sdr.text((pad_x - tb[0], pad_y - tb[1]), sc['name'], font=f, fill=(30, 50, 110, 255))
    sign = sign.rotate(ang, expand=True, resample=Image.BICUBIC)   # 화면에서 가로로 읽히게
    cx, cy = W//2, by1 + m(5.5)
    img.alpha_composite(sign, (cx - sign.width//2, cy - sign.height//2))
    return img


def build(net_path=os.path.join(NET_DIR, 'roundabout.net.xml'), out=OUT):
    net = ET.parse(net_path).getroot()
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    mid = J['school_access_mid']
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<!-- env/roundabout_school_buildings.py 가 생성. 학교 그림 (주행에는 영향 없음) -->',
             '<additional>']
    for sc in SCHOOLS:
        g = J[sc['gate']]
        L = math.dist(mid, g)
        v = ((g[0]-mid[0])/L, (g[1]-mid[1])/L)          # 정문 길 방향 = 학교 쪽 (이미지 위쪽)
        w, h = sc['size']
        # 학교 길은 정문을 지나 GATE_INSET m 더 들어가 끝난다 → 이미지 아래 변(정문)을 그만큼 앞으로
        from env.roundabout_school_access import GATE_INSET
        c = (g[0] + v[0]*(h/2 - GATE_INSET), g[1] + v[1]*(h/2 - GATE_INSET))
        # SUMO 지점 이미지 각도: 시계 방향(도), 0 = 이미지 위쪽이 북쪽
        ang = math.degrees(math.atan2(v[0], v[1]))
        fn = f'school_{sc["key"]}.png'
        draw_school(sc, ang).save(os.path.join(IMG_DIR, fn))
        lines.append(f'    <poi id="school_{sc["key"]}" type="school" color="255,255,255" layer="-0.5" '
                     f'x="{c[0]:.3f}" y="{c[1]:.3f}" angle="{ang:.2f}" width="{w}" height="{h}" '
                     f'imgFile="school_img/{fn}"/>')
    lines.append('</additional>')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    return {'school_images': len(SCHOOLS), 'out': out}


if __name__ == '__main__':
    print(build())
