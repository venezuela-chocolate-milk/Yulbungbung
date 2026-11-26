"""네이버 일반 지도 + 위성 사진으로 도로 주변 풍경(건물, 숲, 공원, 공사장, 골목길)을 만든다. (화면 표시 전용)

    python3 -m env.roundabout_scenery          # 시스템 python3 (numpy, PIL, cv2, scipy, matplotlib 필요)

입력: reports/scenery_src/naver_map.png, naver_satellite.png (같은 범위·같은 축척으로 캡처한 한 쌍)
출력: env/sumo_roundabout/roundabout.scenery.add.xml (GUI 에서만 불러오는 polygon 파일)

  1) 좌표 맞추기: 실제 도로(지도 픽셀)와 SUMO 도로를 기준 도로선마다 짝지어 기준점을 촘촘히 만들고,
     기준점 삼각분할로 조각별 affine 변환을 한다. SUMO 도로는 실제와 100~200m 어긋난 곳이 있어서
     하나의 변환으로는 맞지 않는다. 조각별로 맞추면 건물·숲이 SUMO 도로 옆 제자리에 온다.
  2) 분류: 일반 지도 색으로 구역(도로/건물/단지/학교/녹지/빈 땅)을 나누고, 위성 색으로 실제 내용
     (숲/풀밭/파란 지붕/밝은 지붕/맨땅/포장)을 채운다. 글자·아이콘은 주변 값으로 메운다.
  3) SUMO 도로 둘레에는 인도 띠를 두르고, 실제 큰길(우리 도로와 겹치는 것)은 포장면으로 바꾼다.
  4) 칸 지도를 다각형으로 바꿔 큰 것부터 아래에 깐다(구멍 안의 작은 구역은 위에 그려진다).
"""
import math
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(BASE, 'reports', 'scenery_src')
NET_DIR = os.path.join(BASE, 'env', 'sumo_roundabout')
OUT = os.path.join(NET_DIR, 'roundabout.scenery.add.xml')
RES = 1.2               # SUMO 칸 크기 (m). 지도 1픽셀 ≈ 1.2 m
SIDEWALK = 4.0          # SUMO 도로 가장자리 바깥 인도 폭 (m)
MAIN_ROAD_NEAR = 45.0   # 이 거리 안의 실제 큰길은 SUMO 도로가 대신하므로 포장면으로 바꾼다 (m)

# 기준 도로선: (지도 픽셀 대략 경로, SUMO 경로 정의). 픽셀 중간점은 흰 도로 가운데로 자동 보정한다.
PIX = {
    'E':     [(148, 352), (250, 290), (350, 225), (450, 165), (560, 118)],
    'TOP':   [(560, 118), (650, 145), (750, 200), (850, 280), (950, 345), (1050, 400), (1140, 468)],
    'DIAG':  [(1140, 468), (1140, 560), (1137, 640), (1135, 715)],
    'BEAST': [(1135, 715), (1250, 718), (1370, 732), (1485, 755)],
    'BWEST': [(400, 893), (550, 830), (700, 790), (850, 760), (1000, 735), (1135, 715)],
    'LEFT':  [(148, 352), (200, 470), (250, 590), (300, 700), (350, 800), (400, 893)],
    'CURVE': [(1140, 468), (1250, 540), (1350, 600), (1440, 665)],
}

# 분류 결과
NONE, STREET, BUILDING, PAVED, TREES, FOREST, GRASS, BARE, ROOF_BLUE, ROOF_GRAY, YARD, WALK = range(12)
COLORS = {STREET: '104,104,108', BUILDING: '226,222,214', PAVED: '178,175,168', TREES: '72,104,62',
          FOREST: '56,82,50', GRASS: '118,152,90', BARE: '192,174,136', ROOF_BLUE: '86,130,182',
          ROOF_GRAY: '200,198,192', YARD: '160,157,148', WALK: '190,187,180'}
OUTLINE = {BUILDING: '150,146,138', ROOF_BLUE: '58,92,136', ROOF_GRAY: '140,138,132'}


def _read(v):
    return [tuple(map(float, p.split(','))) for p in v.split()]


def _resample(pts, n):
    pts = np.asarray(pts, float)
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
    t = np.linspace(0, d[-1], n)
    return np.c_[np.interp(t, d, pts[:, 0]), np.interp(t, d, pts[:, 1])], d[-1]


# ───────────────────────── 1. 기준점 ─────────────────────────

def _refine(pts, road):
    """중간점을 경로 법선 방향 ±18px 안의 흰 도로 가운데로 옮긴다."""
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        (x0, y0), (x, y), (x1, y1) = pts[i-1], pts[i], pts[i+1]
        tx, ty = x1 - x0, y1 - y0
        L = math.hypot(tx, ty)
        nx, ny = -ty/L, tx/L
        hits = [k for k in range(-18, 19)
                if 0 <= int(round(y + ny*k)) < road.shape[0] and 0 <= int(round(x + nx*k)) < road.shape[1]
                and road[int(round(y + ny*k)), int(round(x + nx*k))]]
        if hits:
            runs, cur = [], [hits[0]]
            for k in hits[1:]:
                if k == cur[-1] + 1:
                    cur.append(k)
                else:
                    runs.append(cur)
                    cur = [k]
            runs.append(cur)
            best = max(runs, key=lambda r: (len(r) >= 6, -min(abs(v) for v in r)))
            c = (best[0] + best[-1]) / 2
            out.append((x + nx*c, y + ny*c))
        else:
            out.append((x, y))
    out.append(pts[-1])
    return out


def _sumo_paths(net):
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    # 상가 골목(env.roundabout_shops)·학교 진입로(env.roundabout_school_access)로 나뉜 위쪽 긴 도로를 다시 잇는다
    lane['X_se_out_0'] = sum((lane[f'{e}_0'] for e in ('X_se_out', 'X_se_out_s1', 'X_se_out_s2', 'X_se_out_s3',
                                                       'X_se_out_b') if f'{e}_0' in lane), [])

    def mid(a, b):              # 두 방향 0번 차로 가운데 (b 는 반대 방향이라 뒤집는다)
        A, _ = _resample(lane[a], 60)
        B, _ = _resample(lane[b][::-1], 60)
        return [tuple(p) for p in (A + B) / 2]
    top = [J['T_end_main_end']] + mid('X_se_out_0', 'X_se_in_0') + [J['SE_triangle_corner']]
    diag = [J['SE_triangle_corner'], J['SE_triangle_SW']] + lane['SW_final_forward_in_0'][::-1] + [J['SW_final_forward']]
    beast = [J[k] for k in ('SW_final_forward', 'SW_long_uturn', 'SW_long_conn', 'SW_long_right_start',
                            'SW_end_T_left', 'SW_new_cross')]
    bwest = [J['S_cross']] + lane['S_link_east_0'] + [J['SW_end_T_right'], J['SW_final_forward']]
    curve = [J['SE_triangle_corner'], J['SE_triangle_forward']] + lane['S_conn_down_0']
    c, L = _resample(curve, 400)
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(c, axis=0).T))]
    curve = [tuple(p) for p in c[d <= 0.7*L]]       # 아래로 꺾여 내려가는 끝부분은 빼고 직선 쪽만
    return {'E': [(0.0, 0.0), J['T_end_main_end']], 'TOP': top, 'DIAG': diag, 'BEAST': beast,
            'BWEST': bwest, 'LEFT': [(0.0, 0.0), J['S_cross']], 'CURVE': curve}, J


def control_points(net, road):
    sumo, J = _sumo_paths(net)
    P, Q = [], []
    for k, pix in PIX.items():
        pix = _refine(pix, road)
        s, L = _resample(sumo[k], 2)
        n = max(3, int(L/70.0) + 1)
        a, _ = _resample(pix, n)
        b, _ = _resample(sumo[k], n)
        P += [tuple(p) for p in a]
        Q += [tuple(q) for q in b]
    # 중복 제거 (교차점이 여러 도로선에 들어간다)
    keep, seen = [], []
    for i, p in enumerate(P):
        if all(math.dist(p, s) > 6 for s in seen):
            seen.append(p)
            keep.append(i)
    P, Q = np.array([P[i] for i in keep]), np.array([Q[i] for i in keep])
    # 바깥: 전체 affine 으로 이미지 테두리 점을 보낸다 (외삽)
    A = np.c_[P, np.ones(len(P))]
    M, *_ = np.linalg.lstsq(A, Q, rcond=None)
    h, w = road.shape
    rim = [(x, y) for x in np.linspace(-200, w + 200, 9) for y in (-200, h + 200)] + \
          [(x, y) for y in np.linspace(-200, h + 200, 7)[1:-1] for x in (-200, w + 200)]
    rim = np.array(rim)
    P = np.r_[P, rim]
    Q = np.r_[Q, np.c_[rim, np.ones(len(rim))] @ M]
    return P, Q


class Warp:
    """기준점 삼각분할(픽셀 공간 Delaunay)로 SUMO 좌표 → 픽셀 좌표 (조각별 affine)."""

    def __init__(self, P, Q):
        import matplotlib.tri as mtri
        tri = mtri.Triangulation(P[:, 0], P[:, 1])
        self.P, self.Q, self.T = P, Q, tri.triangles
        sp = self._area(P[self.T])
        sq = self._area(Q[self.T])
        # 이미지는 y 가 아래로, SUMO 는 위로 → 정상 삼각형은 넓이 부호가 반대
        self.flipped = int(np.sum(np.sign(sp) == np.sign(sq)))
        self.qtri = mtri.Triangulation(Q[:, 0], Q[:, 1], triangles=self.T)
        self.finder = self.qtri.get_trifinder() if self.flipped == 0 else None
        self.pfinder = tri.get_trifinder()

    @staticmethod
    def _bary(src, dst, T, k, x, y):
        ok = k >= 0
        kk = np.where(ok, k, 0)
        q, p = src[T[kk]], dst[T[kk]]
        d = (q[:, 1, 1]-q[:, 2, 1])*(q[:, 0, 0]-q[:, 2, 0]) + (q[:, 2, 0]-q[:, 1, 0])*(q[:, 0, 1]-q[:, 2, 1])
        l0 = ((q[:, 1, 1]-q[:, 2, 1])*(x-q[:, 2, 0]) + (q[:, 2, 0]-q[:, 1, 0])*(y-q[:, 2, 1])) / d
        l1 = ((q[:, 2, 1]-q[:, 0, 1])*(x-q[:, 2, 0]) + (q[:, 0, 0]-q[:, 2, 0])*(y-q[:, 2, 1])) / d
        l2 = 1 - l0 - l1
        return (l0*p[:, 0, 0] + l1*p[:, 1, 0] + l2*p[:, 2, 0], l0*p[:, 0, 1] + l1*p[:, 1, 1] + l2*p[:, 2, 1], ok)

    def to_sumo(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        return self._bary(self.P, self.Q, self.T, self.pfinder(x, y), x, y)

    @staticmethod
    def _area(t):
        return 0.5*((t[:, 1, 0]-t[:, 0, 0])*(t[:, 2, 1]-t[:, 0, 1]) - (t[:, 2, 0]-t[:, 0, 0])*(t[:, 1, 1]-t[:, 0, 1]))

    def to_pixel(self, x, y):
        k = self.finder(x, y)
        ok = k >= 0
        kk = np.where(ok, k, 0)
        q = self.Q[self.T[kk]]
        p = self.P[self.T[kk]]
        d = (q[:, 1, 1]-q[:, 2, 1])*(q[:, 0, 0]-q[:, 2, 0]) + (q[:, 2, 0]-q[:, 1, 0])*(q[:, 0, 1]-q[:, 2, 1])
        l0 = ((q[:, 1, 1]-q[:, 2, 1])*(x-q[:, 2, 0]) + (q[:, 2, 0]-q[:, 1, 0])*(y-q[:, 2, 1])) / d
        l1 = ((q[:, 2, 1]-q[:, 0, 1])*(x-q[:, 2, 0]) + (q[:, 0, 0]-q[:, 2, 0])*(y-q[:, 2, 1])) / d
        l2 = 1 - l0 - l1
        px = l0*p[:, 0, 0] + l1*p[:, 1, 0] + l2*p[:, 2, 0]
        py = l0*p[:, 0, 1] + l1*p[:, 1, 1] + l2*p[:, 2, 1]
        return px, py, ok


# ───────────────────────── 2. 분류 ─────────────────────────

def classify(mp, sat):
    from scipy import ndimage
    m = mp.astype(int)
    r, g, b = m[..., 0], m[..., 1], m[..., 2]
    near = lambda c, t: (np.abs(m - np.array(c)).max(-1) <= t)
    zone = np.full(r.shape, -1, int)            # -1 = 모름(글자·아이콘)
    Z_ROAD, Z_BLD, Z_BLOCK, Z_GREEN, Z_BG = range(5)
    zone[(r >= 253) & (g >= 253) & (b >= 253)] = Z_ROAD
    zone[near((249, 247, 243), 3)] = Z_BLD
    zone[near((241, 238, 235), 4) | near((229, 237, 244), 4)] = Z_BLOCK           # 단지 바닥, 학교
    zone[(g - r >= 8) & (g - b >= 18) & (g >= 225)] = Z_GREEN
    neutral = (np.ptp(m, -1) <= 3)
    zone[neutral & (r >= 224) & (r <= 248)] = Z_BG                                 # 빈 땅, 개발구역(회색)
    unk = zone < 0
    unk = cv2.dilate(unk.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0          # 글자 테두리(흰색)까지
    # 위성 사진 위 주황 노선·글자도 모름으로
    s = cv2.GaussianBlur(sat, (5, 5), 0).astype(int)
    sr, sg, sb = s[..., 0], s[..., 1], s[..., 2]
    orange = (sr > 190) & (sb < 110) & (sr - sb > 90)
    unk |= cv2.dilate(orange.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    idx = ndimage.distance_transform_edt(unk, return_distances=False, return_indices=True)
    zone = zone[idx[0], idx[1]]
    s = s[idx[0], idx[1]]
    sr, sg, sb = s[..., 0], s[..., 1], s[..., 2]
    V = s.max(-1)
    veg = (sg >= sr + 5) & (sg >= sb + 3)
    blue = (sb >= sr + 22) & (sb >= sg + 2) & (V >= 95)
    bright = (V >= 140) & (np.ptp(s, -1) <= 40)
    tan = (sr >= sb + 12) & (V >= 105) & ~veg

    cls = np.full(r.shape, YARD, np.uint8)
    bg = zone == Z_BG
    cls[bg] = np.select([veg & (V < 105), veg, blue, bright, tan, V < 80],
                        [FOREST, GRASS, ROOF_BLUE, ROOF_GRAY, BARE, FOREST], YARD)[bg]
    gr = zone == Z_GREEN
    cls[gr] = np.select([veg & (V < 100), blue, tan | bright], [FOREST, ROOF_BLUE, BARE], GRASS)[gr]
    bl = zone == Z_BLOCK
    cls[bl] = np.select([veg | (V < 70), tan], [TREES, BARE], PAVED)[bl]
    cls[zone == Z_BLD] = BUILDING
    cls[zone == Z_ROAD] = STREET
    # 다수결로 잡티 정리 (5x5)
    votes = np.stack([cv2.blur((cls == k).astype(np.float32), (5, 5)) for k in range(12)])
    keep_sharp = (cls == BUILDING) | (cls == STREET)          # 건물·골목은 지도 그대로 날카롭게
    cls = np.where(keep_sharp, cls, votes.argmax(0).astype(np.uint8))
    return cls, zone == Z_ROAD


# ───────────────────────── 3. SUMO 칸 지도 ─────────────────────────

def _road_mask(net, x0, y1, W, H):
    mask = np.zeros((H, W), np.uint8)
    tc = lambda pts: np.array([[(x - x0)/RES, (y1 - y)/RES] for x, y in pts], np.int32)
    for e in net.findall('edge'):
        for l in e.findall('lane'):
            pts = _read(l.get('shape'))
            if len(pts) >= 2:
                cv2.polylines(mask, [tc(pts)], False, 1, thickness=max(1, int(round(float(l.get('width', 3.2))/RES))))
    for j in net.findall('junction'):
        pts = _read(j.get('shape', ''))
        if len(pts) >= 3:
            cv2.fillPoly(mask, [tc(pts)], 1)
    return mask


def _thin(mask):
    """Zhang-Suen 세선화 (골목길 중심선)."""
    img = (mask > 0).astype(np.uint8)
    img = np.pad(img, 1)
    while True:
        changed = False
        for step in (0, 1):
            P2, P3, P4 = img[:-2, 1:-1], img[:-2, 2:], img[1:-1, 2:]
            P5, P6, P7 = img[2:, 2:], img[2:, 1:-1], img[2:, :-2]
            P8, P9 = img[1:-1, :-2], img[:-2, :-2]
            c = img[1:-1, 1:-1]
            nb = [P2, P3, P4, P5, P6, P7, P8, P9]
            B = sum(x.astype(int) for x in nb)
            seq = nb + [P2]
            A = sum(((seq[i] == 0) & (seq[i+1] == 1)).astype(int) for i in range(8))
            if step == 0:
                m = (P2*P4*P6 == 0) & (P4*P6*P8 == 0)
            else:
                m = (P2*P4*P8 == 0) & (P2*P6*P8 == 0)
            rm = (c == 1) & (B >= 2) & (B <= 6) & (A == 1) & m
            if rm.any():
                c[rm] = 0
                changed = True
        if not changed:
            return img[1:-1, 1:-1] > 0


def build():
    from scipy import ndimage
    mp = np.array(Image.open(os.path.join(SRC, 'naver_map.png')).convert('RGB'))
    sat = np.array(Image.open(os.path.join(SRC, 'naver_satellite.png')).convert('RGB'))
    assert mp.shape == sat.shape, '일반 지도와 위성 사진은 같은 크기(같은 범위)여야 한다'
    net = ET.parse(os.path.join(NET_DIR, 'roundabout.net.xml')).getroot()
    cls, road = classify(mp, sat)
    P, Q = control_points(net, road)
    warp = Warp(P, Q)
    for _ in range(20):        # 같은 도로 위 거의 일직선인 세 점이 만드는 얇은 삼각형: 가운데 점을 뺀다
        if warp.flipped == 0:
            break
        sp, sq = warp._area(P[warp.T]), warp._area(Q[warp.T])
        drop = set()
        for t in warp.T[np.sign(sp) == np.sign(sq)]:
            d = P[t]
            far = max(((i, j) for i in range(3) for j in range(3)), key=lambda ij: np.hypot(*(d[ij[0]] - d[ij[1]])))
            drop.add(int(t[3 - far[0] - far[1]]))
        keep = np.array([i for i in range(len(P)) if i not in drop])
        P, Q = P[keep], Q[keep]
        warp = Warp(P, Q)
    assert warp.flipped == 0, f'기준점 변환에서 뒤집힌 삼각형 {warp.flipped}개 (기준 도로선 확인)'
    h, w = cls.shape

    # ── 픽셀 공간에서 골목길 중심선과 건물 조각을 먼저 떼어낸다 (실제 폭·크기를 지키기 위해) ──
    street = cls == STREET
    half = ndimage.distance_transform_edt(street)                 # 도로 반폭 (px)
    skel = _thin(street)
    sy, sx = np.nonzero(skel)
    shalf = half[sy, sx]
    pieces = []                                                   # (종류, 픽셀 꼭짓점)
    for k in (BUILDING, ROOF_BLUE, ROOF_GRAY):
        mask = (cls == k).astype(np.uint8)
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            area = cv2.contourArea(c)
            if area < 5:
                continue
            rect = cv2.minAreaRect(c)
            ra = rect[1][0]*rect[1][1]
            pts = cv2.boxPoints(rect) if ra > 0 and area/ra > 0.7 else cv2.approxPolyDP(c, 0.8, True).reshape(-1, 2)
            if len(pts) >= 3:
                pieces.append((k, np.asarray(pts, float)))
    # 땅(넓은 구역)용 분류: 골목길·건물 자리는 주변 땅으로 메운다
    hole = np.isin(cls, (STREET, BUILDING, ROOF_BLUE, ROOF_GRAY))
    idx = ndimage.distance_transform_edt(hole, return_distances=False, return_indices=True)
    ground = cls[idx[0], idx[1]]
    # 단지 안 건물 자리는 단지 바닥(포장)
    ground[(cls == BUILDING) & ~np.isin(ground, (TREES, BARE))] = PAVED

    # ── SUMO 칸 지도 (넓은 구역) ──
    xs, ys = Q[:, 0], Q[:, 1]
    x0, x1 = math.floor(xs.min()), math.ceil(xs.max())
    y0, y1 = math.floor(ys.min()), math.ceil(ys.max())
    W, H = int((x1 - x0)/RES) + 1, int((y1 - y0)/RES) + 1
    gx, gy = np.meshgrid(x0 + (np.arange(W) + 0.5)*RES, y1 - (np.arange(H) + 0.5)*RES)
    px, py, ok = warp.to_pixel(gx.ravel(), gy.ravel())
    ix, iy = np.round(px).astype(int), np.round(py).astype(int)
    inside = ok & (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    grid = np.zeros(W*H, np.uint8)
    grid[inside] = ground[iy[inside], ix[inside]]
    grid = grid.reshape(H, W)
    cell = lambda X, Y: (((np.asarray(X) - x0)/RES), ((y1 - np.asarray(Y))/RES))

    rm = _road_mask(net, x0, y1, W, H)
    dist = ndimage.distance_transform_edt(rm == 0) * RES
    at = lambda X, Y: dist[np.clip(cell(X, Y)[1].astype(int), 0, H-1), np.clip(cell(X, Y)[0].astype(int), 0, W-1)]

    # ── 골목길: 중심선 위치만 옮기고 실제 폭으로 다시 그린다 ──
    wx, wy, wok = warp.to_sumo(sx.astype(float), sy.astype(float))
    hw = np.clip(shalf*1.2, 2.5, 5.0)                             # 실제 반폭 (m), 2.5~5 m
    d_here = at(wx, wy)
    main = (shalf*1.2 >= 8.0) & (d_here < MAIN_ROAD_NEAR)        # 우리 도로가 대신하는 실제 큰길
    sgrid = np.zeros((H, W), np.uint8)
    cx, cy = cell(wx, wy)
    for X, Y, r_, good in zip(cx, cy, hw, wok & ~main):
        if good:
            cv2.circle(sgrid, (int(round(X)), int(round(Y))), max(1, int(round(r_/RES))), 1, -1)
    grid[(sgrid > 0) & (grid != NONE)] = STREET

    grid[dist < SIDEWALK] = WALK
    grid[rm > 0] = NONE                                                        # SUMO 도로 밑은 비움

    polys = []
    for k in (PAVED, TREES, FOREST, GRASS, BARE, YARD, WALK, STREET):
        mask = (grid == k).astype(np.uint8)
        if k not in (STREET, WALK):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        cs, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hier is None:
            continue
        for c, hh in zip(cs, hier[0]):
            area = cv2.contourArea(c)
            if hh[3] != -1 or area < 10:
                continue
            c = cv2.approxPolyDP(c, 0.9, True)
            if len(c) >= 3:
                polys.append((area, k, [(x0 + (float(p[0][0]) + 0.5)*RES, y1 - (float(p[0][1]) + 0.5)*RES) for p in c]))
    polys.sort(key=lambda t: -t[0])          # 큰 것부터 아래에 (구멍 안의 작은 구역은 위에)
    # (골목길 그물도 같은 순서: 고리 안의 블록은 면적이 작아 그 위에 그려진다)

    # ── 건물: 위치·방향만 옮기고 실제 크기(1px = 1.2 m) ──
    blds, dropped = [], 0
    for k, pts in pieces:
        c = pts.mean(0)
        e = np.array([[c[0], c[1]], [c[0] + 4, c[1]], [c[0], c[1] + 4]])
        X, Y, okk = warp.to_sumo(e[:, 0], e[:, 1])
        if not okk.all():
            continue
        Jm = np.array([[X[1]-X[0], X[2]-X[0]], [Y[1]-Y[0], Y[2]-Y[0]]]) / 4.0
        U, _, Vt = np.linalg.svd(Jm)
        R = U @ Vt                                                  # 회전(+ 이미지 y 뒤집기)
        new = (R @ ((pts - c).T * 1.2)).T + [X[0], Y[0]]
        if (at(new[:, 0], new[:, 1]) < SIDEWALK + 1.0).any() or at([X[0]], [Y[0]])[0] < SIDEWALK + 1.0:
            dropped += 1
            continue
        if not (inside.reshape(H, W)[np.clip(cell(new[:, 0], new[:, 1])[1].astype(int), 0, H-1),
                                     np.clip(cell(new[:, 0], new[:, 1])[0].astype(int), 0, W-1)]).all():
            continue
        blds.append((k, [tuple(p) for p in new]))
    polys += [(0, k, pts) for k, pts in blds]

    n = len(polys)
    root = ET.Element('additional')
    counts = {}
    for i, (area, k, pts) in enumerate(polys):
        layer = -9.0 + 6.5*i/max(1, n)
        shape = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        ET.SubElement(root, 'poly', id=f'scn_{i}', type=f'scenery_{k}', color=COLORS[k], fill='1',
                      layer=f'{layer:.6f}', shape=shape)
        if k in OUTLINE:
            ET.SubElement(root, 'poly', id=f'scn_{i}_o', type=f'scenery_{k}_edge', color=OUTLINE[k], fill='0',
                          lineWidth='0.4', layer=f'{layer + 1e-7:.7f}', shape=shape + f' {pts[0][0]:.1f},{pts[0][1]:.1f}')
        counts[k] = counts.get(k, 0) + 1
    ET.indent(root, space='    ')
    ET.ElementTree(root).write(OUT, encoding='utf-8', xml_declaration=True)
    names = {STREET: 'street', BUILDING: 'building', PAVED: 'paved', TREES: 'trees', FOREST: 'forest',
             GRASS: 'grass', BARE: 'bare', ROOF_BLUE: 'roof_blue', ROOF_GRAY: 'roof_gray', YARD: 'yard', WALK: 'sidewalk'}
    return {'control_points': len(P), 'grid': (W, H), 'polys': n, 'buildings_dropped_near_road': dropped,
            'by_class': {names[k]: v for k, v in sorted(counts.items())}, 'out': OUT}


if __name__ == '__main__':
    print(build())
