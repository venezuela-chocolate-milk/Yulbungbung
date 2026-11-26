"""시뮬레이터 지도에서 흔히 쓰는 위에서 본 건물 스타일 (화면 표시 전용). 2026-11-11 사용자 요청.

  평평한 지붕(차분한 색 몇 가지) + 안쪽 파라펫 선 + 진한 테두리 + 오른쪽 아래로 떨어지는 그림자.
  옥상 설비 도형은 넣지 않는다 (사용자 요청 2026-11-10).

  - polys(): SUMO polygon 줄 (북서쪽 상가 건물처럼 다각형으로 그릴 때)
  - shadow_poly(): 이미지(POI) 건물 밑에 깔 그림자 polygon 한 줄
  - draw_roof(): PIL 이미지 안에 같은 스타일 지붕을 그린다 (상가·학교 이미지)
"""
ROOF_PALETTE = [(206, 204, 198), (214, 205, 188), (190, 198, 208), (204, 190, 178), (184, 192, 180),
                (220, 216, 208), (198, 188, 200), (178, 184, 192)]
SHADOW = '0,0,0,70'
SHADOW_OFF = (0.9, -0.9)          # 월드 m (화면 오른쪽 아래, 화면 위 = 북쪽일 때)
PARAPET_INSET = 0.6               # m
EDGE_DARK = 75
PARAPET_DARK = 26


def shade(c, d):
    return tuple(max(0, min(255, v - d)) for v in c)


def _fmt(pts):
    return ' '.join(f'{x:.3f},{y:.3f}' for x, y in pts)


def shadow_poly(pid, world, layer):
    sx, sy = SHADOW_OFF
    return (f'    <poly id="{pid}" type="building_shadow" color="{SHADOW}" fill="1" layer="{layer}" '
            f'shape="{_fmt([(x + sx, y + sy) for x, y in world])}"/>')


def polys(pid, world, inner_world, roof, layer=-1.3):
    """world: 건물 꼭짓점(월드), inner_world: 파라펫 선(안쪽으로 PARAPET_INSET 들어간 꼭짓점)."""
    ring = list(world) + [world[0]]
    inner = list(inner_world) + [inner_world[0]]
    rc = ','.join(map(str, roof))
    return [shadow_poly(f'{pid}_shadow', world, f'{layer - 0.05:.3f}'),
            f'    <poly id="{pid}" type="building" color="{rc}" fill="1" layer="{layer:.3f}" shape="{_fmt(world)}"/>',
            f'    <poly id="{pid}_parapet" type="building" color="{",".join(map(str, shade(roof, PARAPET_DARK)))}" '
            f'fill="0" lineWidth="0.25" layer="{layer + 0.01:.3f}" shape="{_fmt(inner)}"/>',
            f'    <poly id="{pid}_edge" type="building" color="{",".join(map(str, shade(roof, EDGE_DARK)))}" '
            f'fill="0" lineWidth="0.35" layer="{layer + 0.02:.3f}" shape="{_fmt(ring)}"/>']


def draw_roof(d, box, roof, m):
    """PIL ImageDraw d 에 box=[x0, y0, x1, y1] (px) 지붕: 채움 + 테두리 + 파라펫 선."""
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=roof + (255,), outline=shade(roof, EDGE_DARK) + (255,), width=max(1, m(0.35)))
    k = m(PARAPET_INSET)
    if x1 - x0 > 3*k and y1 - y0 > 3*k:
        d.rectangle([x0 + k, y0 + k, x1 - k, y1 - k], outline=shade(roof, PARAPET_DARK) + (255,),
                    width=max(1, m(0.25)))
