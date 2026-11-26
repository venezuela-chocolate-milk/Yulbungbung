"""검단사거리역 북서쪽 상가 골목 (검단로 북쪽, 검단로445번길 ~ 완정로 사이)을 만든다.

  국소 좌표: s = 검단사거리 중심에서 남서쪽 큰길(검단로) 따라 (m), t = 큰길 북서쪽 가장자리에서 블록 안쪽으로 (m).
  골목·상가 위치는 네이버 지도(1px ≈ 0.307m, REAL) 좌표를 이 국소 좌표로 옮긴 값에서 정했다
  (검단로 방향은 0.9 배: 옆길 445번길을 실제 위치까지 옮기지 못해서, env.roundabout_slide_side).

                     완정로 (서곶로를 블록 윗변 따라 연장, 왕복 4차로)
     WE ── AT ──────── FJ ──── C1T ───────── C2J ── X_nw_end ── 검단사거리
           │445   ①   (119)     │C1(북)   ⑦
          A_R1 ←─R1b── W1 ←R1a─ C1R1
           │   ②       │AW  ③   │
          A_R2 ─R2a→── W2 ─R2b→ C1R2
           │                      C1C2 ─C2(동)─╮
     (옛 막다른 끝)    ④           │    ⑥       │ (↘)
           │      ( UT )←─R4(양방향)─ C1S ←── R3(남서) ─ K2 ←─ K1 (검단로에서 우회전)
           │445   공원     ⑤       │     ⑧
       검단로 (T_main_end) ──────── C1L (검단로에서 C1 시작) ─────────── 검단사거리

  - 블록(①~⑧)은 네이버 지도의 건물 덩어리 그대로이고, 도로는 블록 경계로만 지난다 (블록 안에는 건물만).
  - 일방통행은 네이버 지도 화살표대로: R3 남서, C1 북(검단로에서 시작), R2 동, R1 서, C2 동→남.
    445번길·완정로·R4·AW 는 양방향. R4 는 종다리공원 가운데 둥근 회차로(UT)에서 끝난다.
  - 완정로 연장: 기존 구간(X_nw_in 4차로 / X_nw_out 2차로)과 중앙 분리대를 맞춰 잇는다.
    2026-11-11: 연장 구간 전체도 같은 단면(들어가는 4차로 / 나가는 2차로)이라 폭이 그대로 이어진다.
  - 왕길119안전센터: 센터 안에서 완정로로 나가는 소방차 전용 출구(일방, allow=emergency).
  - 우선순위: 큰길 3 > 완정로 2 > 445번길·C1·R3 1 > 나머지 골목 0. 신호 없음.
  - 표시: 양방향 길 노란 중앙선, 여러 차로 길 흰 점선, 우선순위 낮은 쪽 정지선, 차로별 진행 방향 화살표.
roundabout_south_extend 가 옆길 평행이동 다음 단계로 호출한다.  단독 실행: python -m env.roundabout_nw_shops
"""
import math
import os
import xml.etree.ElementTree as ET

from env.roundabout_graft import _read, _write, _length, at, project, slice_, graft

LANE_W = 3.2
ONE_W = 4.0                         # 일방통행 골목 차로 폭
SPEED = 8.33
WJ_SPEED = 13.89                    # 완정로 (기존 북서쪽 큰길과 같게)
CENTER = 'T_end_main_end'
LEG = 'T_stop_main_in'              # 검단사거리 → 남서쪽 (북서쪽 차로 = 블록 쪽)
YELLOW, WHITE = '235,190,40', '255,255,255'
K1_S = 58.0
TAPER = 14.0
# 네이버 지도(2026-11-10 캡처, 1px ≈ 0.307m) → 국소 좌표
REAL = dict(X=(958.0, 311.0), u=(-0.890, 0.455), K=0.307, SX=0.9, T0=6.0)
# 완정로 연장: 기존 끝(X_nw_end)에서 실제 완정로 방향(블록 윗변)으로. WJ_DIR 은 국소 (s, t) 방향
WJ_DIR = (0.743, 0.668)
WJ_AT = {'nw_C2J': None, 'nw_C1T': 112.0, 'nw_FJ': 133.0}   # s 위치 (C2J 는 기존 끝에서 25m)
NODES_LT = {'nw_C1S': (165.0, 40.0), 'nw_C1R2': (155.0, 72.0), 'nw_C1C2': (141.0, 98.0), 'nw_C1R1': (128.0, 115.0),
            'nw_UT': (232.0, 40.0)}
C1L_S = 170.0                       # C1 이 검단로에서 시작하는 곳 (s)
K2_S = 92.0                         # C2 가 R3 연결로와 만나는 곳 (s, 실제 지도 ⑥ 블록 동쪽 끝)
AW_S = 207.0                        # ②·③ 블록 사이 남북 골목 (s)
UT_R = 8.0                          # 종다리공원 회차로 반지름 (m)
FIRE_IN = (133.0, 130.0)            # 119 센터 안 소방차 출발점 (실제 지도 센터 위치 s≈133, 건물 안쪽)
                                    # 2026-11-11: 완정로를 4차로로 넓혀 출구 길이 2m 로 짧아졌으므로 센터 구역 안쪽으로 8m 옮김 (나가기 전용 출구가 보이게)
A_R2_T, A_R1_T = 111.0, 153.0       # 445번길 연장 위 R2·R1 이 만나는 높이 (실제 지도)
JUNCTIONS = ['nw_K1', 'nw_K2', 'nw_C1L', 'nw_C1S', 'nw_C1R2', 'nw_C1C2', 'nw_C1R1', 'T_side_end', 'nw_A_R1',
             'nw_W1', 'nw_W2', 'nw_UT', 'nw_C2J', 'nw_C1T', 'nw_FJ', 'nw_X', 'nw_JW', 'nw_L', 'nw_Y', 'nw_K', 'X_nw_end']
# 2026-11-12 (사용자 요청 — 검단 먹거리타운 확장): 블록 서쪽 경계를 파란 도로(T_junction 에서 올라오던 막다른 길
#   T_side_out 북행 2차로 / T_side_in 남행 4차로)로 옮긴다. 옛 445번길(T_end_side_*, 검단로 T자 T_main_end 옆길)은 없애고,
#   445번길 연장은 파란 도로 끝(T_side_end)에서 같은 방향으로 곧게 올라간다 (완정로와는 사거리 nw_X 에서 엇갈린다).
#   R2 는 파란 도로 끝 교차점(T_side_end)에서 시작한다 (옛 R2 높이와 8m 차이). 남행은 위에서 2차로 → R1 아래 3차로 →
#   파란 도로 4차로(끝 차로 수 그대로), 북행은 2차로 그대로 완정로(들어가는 4차로)로.
SIDE_DEAD = ('T_end_side_out', 'T_end_side_in')     # 없애는 옛 445번길 아래쪽
BLUE = ('T_side_out', 'T_side_in')                  # 서쪽 경계가 되는 파란 도로 (북행, 남행)
BLUE_SPEED = 11.11
W445_LANES = (2, 4)                                 # (북행, 남행) — 2026-11-12: 위에서 아래까지 그대로 (늘거나 줄지 않게)
# 사거리 (2026-11-12, 실제 완정로 × 검단로445번길): 두 길 모두 사거리를 지나 맵 끝까지 같은 차로 수로 이어진다
NORTH_EXT = 80.0                                    # 사거리에서 북쪽(445번길)으로 이어지는 길이 (m)
# 왕길동 큰길 = 황수대로 (2026-11-12): 회전교차로 북서쪽 도로(N_out0_far 북행 2차로 / N_in1_far 남행 1차로, 잔디 분리대)의
#   유턴 지점에서 BR_STUB_KEEP m 위까지만 남기고(나머지 분리 구간은 지운다) 그 끝에서 분리대 자리에 남행 안쪽 차로가 생겨
#   노란 중앙선 하나의 도로(북행 2 / 남행 2)가 된다. 남행은 내려오다 이 합류부(nw_BM 바닥, BR_TAPER m)에서 2 → 1차로로 모인다.
#   (북행 차로는 곧게, 차로 수 그대로). 그 뒤 곧게 올라가 원정삼거리(nw_Y)에서 완정로와 Y자로 만나고 위로 BR_N_EXT m.
#   완정로는 사거리(nw_X)에서 곧게 서쪽으로 와 세자봉삼거리(nw_K)를 지나 원정삼거리까지. 세자봉로는 nw_K 에서 북동쪽.
#   삼각형 아래 변 = 황수대로 nw_L 에서 갈라져 둥글게 nw_K 로 가는 1차로 우회전 전용 길(nw_slip). 안쪽은 녹지 섬.
#   차로: 황수대로 북행 2 → nw_L 에서 바깥 차로는 우회전 길, 안쪽 차로는 직진(1). 완정로 동행 2 = 우회전 길 1 + Y 가지 1.
#   완정로 4/2차로는 사거리에서 시작 (사거리 서쪽은 동행 2 / 서행 1).
BR_STUB = ('N_out0_far', 'N_in1_far')
BR_SPEED = 16.67                                    # 북서쪽 도로와 같게 (60 km/h)
BR_SB_OFF = 7.80                                    # 북서쪽 도로: 북행 차로들 가운데 ~ 남행 차로 가운데 (두 엣지 모두 spreadType center)
BR_STUB_KEEP = 60.0                                 # 유턴 지점(cluster_N_*)에서 남기는 분리 구간 길이 (m) — 2026-11-12 사용자: 50m 가깝게
BR_TAPER = 40.0                                     # 분리대 자리에 남행 안쪽 차로가 열리는 길이 (m)
BR_TA = 20.0                                        # 분리 구간 끝에서 합류부 바닥 시작까지 (m)
BR_SB_LANES = 2                                     # 합쳐진 뒤 남행 차로 수 (위 끝까지 그대로)
_BR_REMOVED = 0.0                                   # 지운 분리 구간 길이 (apply_net 이 채운다)
_BR_CUT = None                                      # 지운 분리 구간 자리 (노면 표시 지우기용)
BR_JW_S = 120.0                                     # 옛 북서쪽 도로 끝(지우기 전)에서 왕길초 앞길 교차로(nw_JW)까지 (m)
TRI_SIDE = 100.0                                    # 원정삼거리에서 nw_L(황수대로) · nw_K(완정로)까지 (m)
BR_N_EXT = 70.0                                     # 원정삼거리 위로 황수대로를 잇는 길이 (m)
SJ_LEN, SJ_DEG = 60.0, 55.0                         # 세자봉로 길이, 완정로(동쪽)에서 왼쪽으로 돈 각
CROSS_TLS = False                                   # 신호 사거리 여부 (시험 주행에서 문제가 생기면 True)
PARK_W, PARK_GAP = 21.0, 2.5                        # 종다리공원 폭, 파란 도로 쪽 보도 폭 (m)
_SIDE_CORRIDOR = None                               # 없앤 옛 옆길 자리 (노면 표시 지우기용, apply_net 이 채운다)


def blue_line(F, net):
    """파란 도로 가운데 선: (시작점, 단위방향(북행), 가운데 선 ~ 블록 쪽 가장자리 거리) — 세계 좌표."""
    es = {e.get('id'): e for e in net.findall('edge')}
    out_l = [_read(l.get('shape')) for l in es[BLUE[0]].findall('lane')]
    in_l = [_read(l.get('shape')) for l in es[BLUE[1]].findall('lane')]
    a = ((out_l[0][0][0] + in_l[0][-1][0]) / 2, (out_l[0][0][1] + in_l[0][-1][1]) / 2)
    b = ((out_l[0][-1][0] + in_l[0][0][0]) / 2, (out_l[0][-1][1] + in_l[0][0][1]) / 2)
    d = _unit((b[0] - a[0], b[1] - a[1]))
    rgt = (d[1], -d[0])                                     # 북행 오른쪽 = 블록 쪽
    edge = max(((q[0]-a[0])*rgt[0] + (q[1]-a[1])*rgt[1]) for l in out_l for q in l) + LANE_W / 2
    return a, d, edge


def blue_s_at(F, net, t):
    """국소 t 높이에서 파란 도로 블록 쪽 가장자리의 s."""
    a, d, edge = blue_line(F, net)
    rgt = (d[1], -d[0])
    ca, cd = F.st(a), F.st((a[0] + d[0], a[1] + d[1]))
    k = (t - ca[1]) / (cd[1] - ca[1])
    c = (a[0] + d[0]*k, a[1] + d[1]*k)
    return F.st((c[0] + rgt[0]*edge, c[1] + rgt[1]*edge))[0]


def real_st(px, py):
    d = (px - REAL['X'][0], py - REAL['X'][1])
    u = REAL['u']
    n = (-u[1], u[0])                               # 블록 안쪽 (지도 위쪽·왼쪽)
    return ((d[0]*u[0] + d[1]*u[1]) * REAL['K'] * REAL['SX'], (d[0]*n[0] + d[1]*n[1]) * REAL['K'] - REAL['T0'])


class Frame:
    def __init__(self, net):
        es = {e.get('id'): e for e in net.findall('edge')}
        J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
        l0 = _read(es[LEG].find("lane[@index='0']").get('shape'))
        a, b = l0[0], l0[-1]
        L = math.dist(a, b)
        self.u = ((b[0]-a[0])/L, (b[1]-a[1])/L)
        self.n = (self.u[1], -self.u[0])            # 블록 쪽 (남서행 오른쪽 = 북서)
        edge = (a[0] + self.n[0]*LANE_W/2, a[1] + self.n[1]*LANE_W/2)
        c = J[CENTER]
        sc = (c[0]-edge[0])*self.u[0] + (c[1]-edge[1])*self.u[1]
        self.o = (edge[0] + self.u[0]*sc, edge[1] + self.u[1]*sc)

    def P(self, s, t):
        return (self.o[0] + self.u[0]*s + self.n[0]*t, self.o[1] + self.u[1]*s + self.n[1]*t)

    def st(self, p):
        d = (p[0]-self.o[0], p[1]-self.o[1])
        return d[0]*self.u[0] + d[1]*self.u[1], d[0]*self.n[0] + d[1]*self.n[1]

    def vec(self, ds, dt):
        return (self.u[0]*ds + self.n[0]*dt, self.u[1]*ds + self.n[1]*dt)


def _bezier(a, da, b, db, n=24, k=0.45):
    """a 에서 da 방향으로 나가 b 에 db 방향으로 들어가는 3차 베지어 곡선 (양 끝 포함)."""
    L = math.dist(a, b) * k
    p1 = (a[0] + da[0]*L, a[1] + da[1]*L)
    p2 = (b[0] - db[0]*L, b[1] - db[1]*L)
    out = []
    for i in range(n + 1):
        t = i / n
        c = ((1-t)**3, 3*(1-t)**2*t, 3*(1-t)*t**2, t**3)
        out.append((c[0]*a[0] + c[1]*p1[0] + c[2]*p2[0] + c[3]*b[0],
                    c[0]*a[1] + c[1]*p1[1] + c[2]*p2[1] + c[3]*b[1]))
    return out


def _unit(v):
    L = math.hypot(*v)
    return (v[0]/L, v[1]/L)


def _cut_at_t(F, pts, t_target):
    """곡선 위에서 국소 t 가 t_target 인 점까지 / 그 뒤로 나눈다."""
    for i in range(len(pts) - 1):
        ta, tb = F.st(pts[i])[1], F.st(pts[i+1])[1]
        if (ta - t_target) * (tb - t_target) <= 0 and ta != tb:
            f = (t_target - ta) / (tb - ta)
            m = (pts[i][0] + (pts[i+1][0]-pts[i][0])*f, pts[i][1] + (pts[i+1][1]-pts[i][1])*f)
            return pts[:i+1] + [m], [m] + pts[i+1:]
    raise ValueError(f'곡선이 t={t_target} 를 지나지 않는다')


def taper_445(F, net):
    """445번길 2차로 → 연장 1차로 합류부 (옛 막다른 끝 앞 TAPER m)."""
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    s0, t0 = F.st(J['T_end_side_end'])
    n = len(next(e for e in net.findall('edge') if e.get('id') == 'T_end_side_in').findall('lane'))
    # 2026-11-11: 445번길을 방향별 1차로로 줄이면(env.roundabout_side_one_lane) 좁아지는 구간 없이 같은 폭
    wide, narrow = (2 * LANE_W + 0.5, LANE_W) if n > 1 else (LANE_W, LANE_W)
    return [F.P(s0 - wide, t0 - TAPER), F.P(s0 + wide, t0 - TAPER), F.P(s0 + narrow, t0), F.P(s0 - narrow, t0)]


WJ_MEDIAN = 1.0                     # 완정로 가운데 분리대 폭 (기존 북서쪽 큰길과 같게)
WJ_TAPER = 30.0                     # (합류부 바닥 길이)
# 2026-11-11: 완정로 전체를 검단사거리 쪽 앞부분과 같은 단면으로 — 들어가는(검단사거리 방향) 4차로, 나가는 2차로.
WJ_IN_LANES, WJ_OUT_LANES = 4, 2
def taper_wj(net, xnw, ax):
    """검단사거리 쪽 북서쪽 큰길(들어오는 4차로 + 나가는 2차로) ↔ 완정로 연장(왕복 4차로) 합류부.
    중앙 분리대와 나가는 쪽 가장자리는 곧게 이어지고, 들어오는 쪽만 WJ_TAPER m 에 걸쳐 2차로 → 4차로로 넓어진다."""
    es = {e.get('id'): e for e in net.findall('edge')}
    nx = (-ax[1], ax[0])                            # ax(검단사거리 → 연장 끝) 왼쪽
    lat = lambda p: (p[0]-xnw[0])*nx[0] + (p[1]-xnw[1])*nx[1]
    lats = []
    for eid in ('X_nw_in', 'X_nw_out'):
        for l in es[eid].findall('lane'):
            pts = _read(l.get('shape'))
            end = pts[0] if eid == 'X_nw_in' else pts[-1]          # 막다른 끝 쪽 차로 끝
            w = float(l.get('width', '3.2'))
            lats += [lat(end) - w/2, lat(end) + w/2]
    lo, hi = min(lats), max(lats)                   # 기존 도로 양쪽 가장자리
    ext_out = WJ_OUT_LANES * LANE_W + WJ_MEDIAN / 2  # 연장 도로 중심 ~ 나가는 쪽 가장자리 (lo 쪽)
    ext_in = WJ_IN_LANES * LANE_W + WJ_MEDIAN / 2    # 연장 도로 중심 ~ 들어오는 쪽 가장자리 (hi 쪽)
    f_lo, f_hi = max(lo, -ext_out), min(hi, ext_in)
    back = (xnw[0] - ax[0]*WJ_TAPER, xnw[1] - ax[1]*WJ_TAPER)
    P = lambda base, l: (base[0] + nx[0]*l, base[1] + nx[1]*l)
    return [P(back, lo), P(back, hi), P(xnw, f_hi), P(xnw, f_lo)]


def _wanggil_roads(net, nodes, west):
    """황수대로(분리대 합류 → 원정삼거리 → 위로), 원정삼거리 삼각형(완정로·우회전 길·세자봉로), 왕길초 앞길."""
    from env.roundabout_interior import _fillet
    from env.roundabout_geometry import _offset
    es = {e.get('id'): e for e in net.findall('edge')}
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    nb0 = _read(es[BR_STUB[0]].get('shape'))
    s0 = nb0[-1]                                        # 북행 차로들 가운데 선 끝 (N_end0 쪽)
    d0 = _unit((nb0[-1][0] - nb0[0][0], nb0[-1][1] - nb0[0][1]))
    lf, rg = (-d0[1], d0[0]), (d0[1], -d0[0])
    nb_l, sb_l = len(es[BR_STUB[0]].findall('lane')), len(es[BR_STUB[1]].findall('lane'))
    hw = LANE_W*nb_l/2                                  # 북행 가운데 ~ 중앙선
    c0 = (s0[0] + lf[0]*hw, s0[1] + lf[1]*hw)           # 합쳐진 뒤 중앙선이 지나는 선의 시작
    P = lambda t, off=0.0: (c0[0] + d0[0]*t + rg[0]*off, c0[1] + d0[1]*t + rg[1]*off)
    # 1) 합류부: 북행은 곧게. 남행 바깥 차로는 분리 구간 남행 차로를 그대로 잇고(0.2m 옮김), 안쪽 차로는 분리대 자리에서
    #    BR_TAPER m 에 걸쳐 열린다 (nw_BM 교차점 바닥을 쐐기 모양으로 직접 정한다)
    ta, tb = BR_TA, BR_TA + BR_TAPER
    sbo = BR_SB_OFF - hw                                # 분리 구간 남행 차로 가운데 (중앙선 왼쪽)
    nodes['nw_BM'] = P((ta + tb)/2)
    W = LANE_W*nb_l
    BM_SHAPE[:] = [P(ta, W), P(tb, W), P(tb, -LANE_W*BR_SB_LANES), P(ta, -sbo - LANE_W/2), P(ta, -sbo + LANE_W/2),
                   P(tb - 1.0, -0.1), P(ta, 0.0)]
    roads = [('nw_brt_n', 'N_end0', 'nw_BM', nb_l, LANE_W, BR_SPEED, 2, [s0, P(ta, hw)], 'center'),
             ('nw_brt_s', 'nw_BM', 'N_end1', sb_l, LANE_W, BR_SPEED, 2,
              [P(ta, -sbo), P(0.0, -sbo)], 'center')]
    BM_DASH[:] = [P(ta, hw), P(tb, hw)]                  # 합류부 바닥 위 북행 차로 사이 점선
    jw_s = BR_JW_S + _BR_REMOVED
    # 2) 원정삼거리 = 황수대로 중앙선과 완정로(사거리 서쪽) 중앙선의 교점
    xx = nodes['nw_X']
    east = (-west[0], -west[1])
    rgt_e = (east[1], -east[0])
    q0 = (xx[0] + rgt_e[0]*WJ_MEDIAN/2, xx[1] + rgt_e[1]*WJ_MEDIAN/2)   # 동행 차로가 완정로 안쪽 차로와 이어지게
    Q = lambda u, off=0.0: (q0[0] + west[0]*u + rgt_e[0]*off, q0[1] + west[1]*u + rgt_e[1]*off)
    den = d0[0]*(-west[1]) - d0[1]*(-west[0])
    rx, ry = q0[0] - c0[0], q0[1] - c0[1]
    ty = (rx*(-west[1]) - ry*(-west[0])) / den
    uy = (rx*(-d0[1]) - ry*(-d0[0])) / den
    tl, uk = ty - TRI_SIDE, uy - TRI_SIDE
    nodes['nw_JW'], nodes['nw_L'], nodes['nw_Y'] = P(jw_s), P(tl), P(ty)
    nodes['nw_H0'], nodes['nw_K'] = P(ty + BR_N_EXT), Q(uk)
    two = lambda name, a, b, nf, nr, prio, pts, sp=BR_SPEED: [(f'{name}_f', a, b, nf, LANE_W, sp, prio, pts),
                                                               (f'{name}_r', b, a, nr, LANE_W, sp, prio, pts[::-1])]
    sbl = BR_SB_LANES
    roads += two('nw_hs1', 'nw_BM', 'nw_JW', nb_l, sbl, 2, [P(tb), P(jw_s)])
    roads += two('nw_hs2', 'nw_JW', 'nw_L', nb_l, sbl, 2, [P(jw_s), P(tl)])
    roads += two('nw_hs3', 'nw_L', 'nw_Y', nb_l - 1, sbl, 2, [P(tl), P(ty)])
    roads += two('nw_hs4', 'nw_Y', 'nw_H0', nb_l, sbl, 2, [P(ty), P(ty + BR_N_EXT)])
    # 완정로 (동행 = _f 가 아니라 _r: _f 는 서쪽으로 가는 방향) — 사거리 ↔ 세자봉삼거리 ↔ 원정삼거리
    roads += two('nw_wak', 'nw_X', 'nw_K', sb_l, nb_l, 2, [Q(0), Q(uk)], WJ_SPEED)
    roads += two('nw_wa', 'nw_K', 'nw_Y', sb_l, 1, 2, [Q(uk), Q(uy)], WJ_SPEED)
    # 우회전 전용 길: 황수대로 북행 바깥 차로 가운데에서 둥글게 완정로 동행 바깥 차로 가운데로 (1차로 일방)
    a_ = P(tl, LANE_W*1.5)
    b_ = Q(uk, LANE_W*1.5)
    slip = _bezier(a_, d0, b_, east, n=28, k=0.5)
    roads.append(('nw_slip', 'nw_L', 'nw_K', 1, LANE_W, BLUE_SPEED, 2, slip, 'center'))   # 전용 차로로 들어가 정지선 없음
    # 세자봉로: 세자봉삼거리에서 북동쪽으로 (왕복 1차로)
    th = math.radians(SJ_DEG)
    dsj = (east[0]*math.cos(th) - east[1]*math.sin(th), east[0]*math.sin(th) + east[1]*math.cos(th))
    kk = nodes['nw_K']
    nodes['nw_S0'] = (kk[0] + dsj[0]*SJ_LEN, kk[1] + dsj[1]*SJ_LEN)
    roads += two('nw_sj', 'nw_K', 'nw_S0', 1, 1, 1, [kk, nodes['nw_S0']], BLUE_SPEED)
    # 3) 왕길초 앞길: 황수대로에서 수직으로 나와 동쪽으로, 445번길 T_side_end 에 R2 와 마주 보게 들어가 네거리
    te = J['T_side_end']
    jw = nodes['nw_JW']
    r2 = _unit((nodes['nw_W2'][0] - te[0], nodes['nw_W2'][1] - te[1]))
    p0 = (jw[0] + rg[0]*(LANE_W*nb_l + 25.0), jw[1] + rg[1]*(LANE_W*nb_l + 25.0))
    p3 = (te[0] - r2[0]*35.0, te[1] - r2[1]*35.0)
    wg = _fillet([jw, p0, p3, te], radius=90.0, step=3.0)
    roads += [('nw_wg_f', 'nw_JW', 'T_side_end', 1, LANE_W, BLUE_SPEED, 1, wg),
              ('nw_wg_r', 'T_side_end', 'nw_JW', 1, LANE_W, BLUE_SPEED, 1, wg[::-1])]
    return roads


BM_SHAPE = []                                        # 합류부 바닥 (plan 이 채운다)
BM_DASH = []                                         # 합류부 바닥 위 북행 차로 사이 점선 자리


# 황수대로·원정삼거리 삼각형 차로 연결 (0 = 맨 오른쪽 차로). 차로 수가 갈라지고 합쳐질 때 그대로 나뉘게 직접 정한다.
TRI_CON = (
    # 분리대 합류점: 그대로 이어진다
    [('nw_brt_n', 'nw_hs1_f', k, k) for k in range(2)] + [('nw_hs1_r', 'nw_brt_s', 0, 0)]   # 남행 안쪽 차로는 합류부 앞에서 미리 바깥 차로로 옮긴다 (차로 끝)
    # nw_L: 북행 바깥 차로 → 우회전 길, 안쪽 차로 → 직진 1차로 / 남행 그대로
    + [('nw_hs2_f', 'nw_slip', 0, 0), ('nw_hs2_f', 'nw_hs3_f', 1, 0)] + [('nw_hs3_r', 'nw_hs2_r', k, k) for k in range(2)]
    # 원정삼거리: 남행 2차로 → 직진 1 + 완정로 동행 1 / 북행 2 = 직진 1 + 완정로에서 우회전 1 / 완정로 서행 좌회전
    + [('nw_hs4_r', 'nw_hs3_r', 0, 0), ('nw_hs4_r', 'nw_hs3_r', 1, 1), ('nw_hs4_r', 'nw_wa_r', 1, 0),
       ('nw_hs3_f', 'nw_hs4_f', 0, 1), ('nw_wa_f', 'nw_hs4_f', 0, 0), ('nw_wa_f', 'nw_hs3_r', 0, 1)]
    # 세자봉삼거리: 동행 2 = 우회전 길(바깥) + Y 가지(안쪽) / 서행 → 직진·세자봉로 우회전 / 세자봉로에서 나오는 차
    + [('nw_slip', 'nw_wak_r', 0, 0), ('nw_wa_r', 'nw_wak_r', 0, 1), ('nw_wak_f', 'nw_wa_f', 0, 0),
       ('nw_wak_f', 'nw_sj_f', 0, 0), ('nw_sj_r', 'nw_wak_r', 0, 1), ('nw_sj_r', 'nw_wa_f', 0, 0)]
)


def plan(net):
    F = Frame(net)
    J = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net.findall('junction')}
    es = {e.get('id'): e for e in net.findall('edge')}
    P = F.P
    nodes = {k: P(*v) for k, v in NODES_LT.items()}
    xnw = J['X_nw_end']
    xs, xt = F.st(xnw)
    wd = _unit(WJ_DIR)
    # 완정로 연장: 첫 구간(X_nw_end → C2J 25m)은 기존 북서쪽 큰길과 같은 방향으로 곧게 이어 합류부를 바르게 하고,
    # C2J 부터 실제 완정로 방향(WJ_DIR)으로 튼다.
    xin = _read(es['X_nw_in'].get('shape'))
    axis = _unit((xin[0][0] - xin[-1][0], xin[0][1] - xin[-1][1]))      # 검단사거리 → X_nw_end
    nodes['nw_C2J'] = (xnw[0] + axis[0]*25.0, xnw[1] + axis[1]*25.0)
    cs, ct = F.st(nodes['nw_C2J'])
    for k, s in WJ_AT.items():
        if s is not None:
            d = (s - cs) / wd[0]
            nodes[k] = P(cs + wd[0]*d, ct + wd[1]*d)
    # 445번길 연장 (2026-11-12): 파란 도로 끝(T_side_end)에서 같은 방향으로 곧게 올라가 완정로와 사거리(nw_X)로
    # 엇갈리고 맵 위쪽 끝(nw_N0)까지. 서쪽에서는 왕길동 큰길(nw_br*)이 들어온다. 세로 도로는 남행 4 / 북행 2.
    side_end = J['T_side_end']
    _, d_blue, _ = blue_line(F, net)
    fj, c1t = nodes['nw_FJ'], nodes['nw_C1T']
    west = _unit((fj[0] - c1t[0], fj[1] - c1t[1]))
    # 세로 도로 선과 완정로 중심선의 교점
    den = d_blue[0]*(-west[1]) - d_blue[1]*(-west[0])
    rx, ry = fj[0] - side_end[0], fj[1] - side_end[1]
    tv = (rx*(-west[1]) - ry*(-west[0])) / den
    vp = lambda d: (side_end[0] + d_blue[0]*d, side_end[1] + d_blue[1]*d)
    nodes['nw_X'], nodes['nw_N0'] = vp(tv), vp(tv + NORTH_EXT)
    xx = nodes['nw_X']
    line = [vp(tv * i / 20) for i in range(21)]
    p_c, p_d = _cut_at_t(F, line, A_R1_T)
    nodes['nw_A_R1'] = p_c[-1]
    p_d = [p_d[0], xx]
    p_n = [xx, nodes['nw_N0']]
    # 종다리공원 회차로(UT): 공원과 함께 새 서쪽 경계(파란 도로) 옆으로
    ut_s = blue_s_at(F, net, NODES_LT['nw_UT'][1]) - PARK_GAP - PARK_W / 2
    nodes['nw_UT'] = P(ut_s, NODES_LT['nw_UT'][1])
    one = lambda eid, a, b, prio, pts, allow=None: (eid, a, b, 1, ONE_W, SPEED, prio, pts, 'right', allow)
    two = lambda name, a, b, prio, pts, nl=1, sp=SPEED: [(f'{name}_f', a, b, nl, LANE_W, sp, prio, pts),
                                                           (f'{name}_r', b, a, nl, LANE_W, sp, prio, pts[::-1])]
    leg_shape = _read(es[LEG].get('shape'))
    k1_station = project(leg_shape, P(K1_S, -3.2))
    k1 = at(leg_shape, k1_station)
    # R3: 검단로에서 수직으로 나와 R3 방향으로 휘는 곡선 → K2 에서 C2 가 합류 → C1S
    r3 = _bezier(k1, F.n, P(K1_S + 42, 40), F.u, n=32) + [nodes['nw_C1S']]
    ik = next(i for i, q in enumerate(r3) if F.st(q)[0] >= K2_S)
    nodes['nw_K2'] = r3[ik]
    r3a, r3b = r3[:ik + 1], r3[ik:]
    # C2: C1C2 에서 ⑥ 블록 윗변 따라 동쪽으로 가다가 남쪽으로 꺾여(↘) K2 에서 R3 로 합류
    k2_dir = _unit((r3[ik+1][0] - r3[ik-1][0], r3[ik+1][1] - r3[ik-1][1]))
    c2 = _bezier(nodes['nw_C1C2'], _unit(F.vec(-0.85, -0.5)), nodes['nw_K2'],
                 _unit((k2_dir[0]*0.3 - F.n[0], k2_dir[1]*0.3 - F.n[1])), n=24, k=0.5)
    # C1 시작: 검단로(T_end_main_in) 위
    em = _read(es['T_end_main_in'].get('shape'))
    c1l_station = project(em, P(C1L_S, -3.2))
    nodes['nw_C1L'] = at(em, c1l_station)

    def on_line(a, b, s_target):
        sa, sb = F.st(a)[0], F.st(b)[0]
        f = (s_target - sa) / (sb - sa)
        return (a[0] + (b[0]-a[0])*f, a[1] + (b[1]-a[1])*f)
    # ②·③ 사이 골목 AW: R1 · R2 위 s = AW_S 인 점
    nodes['nw_W1'] = on_line(nodes['nw_C1R1'], nodes['nw_A_R1'], AW_S)
    nodes['nw_W2'] = on_line(side_end, nodes['nw_C1R2'], AW_S)
    N = nodes
    streets = [
        one('nw_r3a', 'nw_K1', 'nw_K2', 1, r3a),
        one('nw_r3b', 'nw_K2', 'nw_C1S', 1, r3b),
        one('nw_c1l', 'nw_C1L', 'nw_C1S', 1, [N['nw_C1L'], N['nw_C1S']]),
        one('nw_c1a', 'nw_C1S', 'nw_C1R2', 1, [N['nw_C1S'], N['nw_C1R2']]),
        one('nw_c1b', 'nw_C1R2', 'nw_C1C2', 1, [N['nw_C1R2'], N['nw_C1C2']]),
        one('nw_c1c', 'nw_C1C2', 'nw_C1R1', 1, [N['nw_C1C2'], N['nw_C1R1']]),
        one('nw_c1d', 'nw_C1R1', 'nw_C1T', 1, [N['nw_C1R1'], N['nw_C1T']]),
        one('nw_r2a', 'T_side_end', 'nw_W2', 0, [side_end, N['nw_W2']]),
        one('nw_r2b', 'nw_W2', 'nw_C1R2', 0, [N['nw_W2'], N['nw_C1R2']]),
        one('nw_r1a', 'nw_C1R1', 'nw_W1', 0, [N['nw_C1R1'], N['nw_W1']]),
        one('nw_r1b', 'nw_W1', 'nw_A_R1', 0, [N['nw_W1'], N['nw_A_R1']]),
        one('nw_c2', 'nw_C1C2', 'nw_K2', 0, c2),
        one('nw_119', 'nw_F0', 'nw_FJ', 0, [P(*FIRE_IN), N['nw_FJ']], 'emergency'),
    ]
    streets += two('nw_r4', 'nw_C1S', 'nw_UT', 0, [N['nw_C1S'], N['nw_UT']])
    streets += two('nw_aw', 'nw_W1', 'nw_W2', 0, [N['nw_W1'], N['nw_W2']])
    for name, a_, b_, pts in (('nw_445c', 'T_side_end', 'nw_A_R1', p_c), ('nw_445d', 'nw_A_R1', 'nw_X', p_d),
                              ('nw_445n', 'nw_X', 'nw_N0', p_n)):
        nf, nr = W445_LANES
        # 우선순위 2: 이어지는 파란 도로(T_side_*)·완정로와 같게 (이어지는 곳에 정지선이 생기지 않게)
        streets += [(f'{name}_f', a_, b_, nf, LANE_W, BLUE_SPEED, 2, pts),
                    (f'{name}_r', b_, a_, nr, LANE_W, BLUE_SPEED, 2, pts[::-1])]
    # 완정로: 기존 큰길처럼 가운데 1m 분리대(WJ_MEDIAN). 방향마다 중심선에서 자기 오른쪽으로 0.5m 옮긴 선을
    # 엣지 모양으로 써서, 나가는 2차로는 기존 나가는 차로(X_nw_out)와, 들어오는 2차로는 기존 들어오는 차로의
    # 안쪽 두 차로와 정확히 이어지게 한다.
    seq = ['X_nw_end', 'nw_C2J', 'nw_C1T', 'nw_FJ', 'nw_X']
    for i, (a, b) in enumerate(zip(seq, seq[1:]), 1):
        pa = xnw if a == 'X_nw_end' else nodes[a]
        pb = nodes[b]
        d = _unit((pb[0] - pa[0], pb[1] - pa[1]))
        rgt = (d[1], -d[0])                                  # a→b 진행 방향 오른쪽
        h = WJ_MEDIAN / 2
        f_pts = [(pa[0] + rgt[0]*h, pa[1] + rgt[1]*h), (pb[0] + rgt[0]*h, pb[1] + rgt[1]*h)]
        r_pts = [(pb[0] - rgt[0]*h, pb[1] - rgt[1]*h), (pa[0] - rgt[0]*h, pa[1] - rgt[1]*h)]
        streets += [(f'nw_wj{i}_f', a, b, WJ_OUT_LANES, LANE_W, WJ_SPEED, 2, f_pts),
                    (f'nw_wj{i}_r', b, a, WJ_IN_LANES, LANE_W, WJ_SPEED, 2, r_pts)]
    nodes['nw_K1'] = k1
    nodes['nw_F0'] = P(*FIRE_IN)
    streets += _wanggil_roads(net, nodes, west)
    splits = [(LEG, [('nw_K1', k1_station)], [LEG, f'{LEG}_b']),
              ('T_end_main_in', [('nw_C1L', c1l_station)], ['T_end_main_in_a', 'T_end_main_in']),
              (BLUE[0], [], [BLUE[0]]), (BLUE[1], [], [BLUE[1]]),
              # 옛 옆길이 없어진 검단로 T자(T_main_end)를 직진 이음부로 다시 만든다
              ('T_end_main_near', [], ['T_end_main_near']), ('T_end_main_out', [], ['T_end_main_out']),
              ('T_main_in', [], ['T_main_in']),
              ('X_nw_in', [], ['X_nw_in']), ('X_nw_out', [], ['X_nw_out']),
              (BR_STUB[0], [], [BR_STUB[0]]), (BR_STUB[1], [], [BR_STUB[1]])]
    ut = nodes['nw_UT']
    circle = [(ut[0] + UT_R*math.cos(2*math.pi*k/24), ut[1] + UT_R*math.sin(2*math.pi*k/24)) for k in range(24)]
    shapes = {'X_nw_end': taper_wj(net, xnw, axis), 'nw_UT': circle, 'nw_BM': list(BM_SHAPE)}
    return F, nodes, streets, splits, shapes


def _remove_old_side(net_path):
    """옛 445번길 아래쪽(T_end_side_out/in)과 그 끝 교차점을 지운다. 지운 자리(노면 표시 정리용)를 기억한다."""
    global _SIDE_CORRIDOR
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    tree = ET.parse(net_path)
    root = tree.getroot()
    dead = set(SIDE_DEAD)
    geo = []
    for e in list(root.findall('edge')):
        if e.get('id') in dead:
            for l in e.findall('lane'):
                geo.append(LineString(_read(l.get('shape'))).buffer(float(l.get('width', '3.2'))/2 + 2.0))
            root.remove(e)
        elif e.get('id', '').startswith(':T_end_side_end_'):
            root.remove(e)
    for j in list(root.findall('junction')):
        if j.get('id') == 'T_end_side_end' or j.get('id', '').startswith(':T_end_side_end_'):
            root.remove(j)
        elif j.get('id') == 'T_main_end':
            geo.append(Polygon(_read(j.get('shape'))).buffer(0))
    for c in list(root.findall('connection')):
        if c.get('from') in dead or c.get('to') in dead:
            root.remove(c)
    _SIDE_CORRIDOR = unary_union(geo)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)


def _trim_br_stub(net_path):
    """회전교차로 북서쪽 분리 도로(BR_STUB)를 유턴 지점에서 BR_STUB_KEEP m 까지만 남기고 끝 교차점(N_end0/1)을 옮긴다."""
    global _BR_REMOVED, _BR_CUT
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    from env.roundabout_graft import slice_
    tree = ET.parse(net_path)
    root = tree.getroot()
    es = {e.get('id'): e for e in root.findall('edge')}
    js = {j.get('id'): j for j in root.findall('junction')}
    cut = []
    for eid, outward in ((BR_STUB[0], True), (BR_STUB[1], False)):
        e = es[eid]
        shp = _read(e.get('shape'))
        L = _length(shp)
        keep = (0.0, BR_STUB_KEEP) if outward else (L - BR_STUB_KEEP, L)
        e.set('shape', _write(slice_(shp, *keep)))
        for ln in e.findall('lane'):
            lp = _read(ln.get('shape'))
            LL = _length(lp)
            k2 = (0.0, BR_STUB_KEEP) if outward else (LL - BR_STUB_KEEP, LL)
            gone = slice_(lp, BR_STUB_KEEP, LL) if outward else slice_(lp, 0.0, LL - BR_STUB_KEEP)
            cut.append(LineString(gone).buffer(float(ln.get('width', '3.2'))/2 + 0.4, cap_style=2))
            new = slice_(lp, *k2)
            ln.set('shape', _write(new))
            ln.set('length', f'{_length(new):.6f}')
        end = _read(e.get('shape'))[-1 if outward else 0]
        j = js[e.get('to') if outward else e.get('from')]
        j.set('x', f'{end[0]:.6f}')
        j.set('y', f'{end[1]:.6f}')
        _BR_REMOVED = L - BR_STUB_KEEP
    _BR_CUT = unary_union(cut)
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)


def _strip_short_internal(net_path, junctions):
    """교차점 안 회전 경로 끝의 0.5m 미만 조각 점을 뺀다 (차로 폭 오프셋이 접히는 원인, env.roundabout_interior 와 같은 처리).
    양 끝점은 그대로라 연결은 바뀌지 않는다."""
    tree = ET.parse(net_path)
    prefs = tuple(f':{j}_' for j in junctions)
    for e in tree.getroot().findall('edge'):
        if e.get('function') != 'internal' or not e.get('id', '').startswith(prefs):
            continue
        for ln in e.findall('lane'):
            p = _read(ln.get('shape'))
            if len(p) < 3:
                continue
            q = [p[0]]
            for pt in p[1:-1]:
                if math.dist(pt, q[-1]) >= 0.5:
                    q.append(pt)
            if math.dist(p[-1], q[-1]) < 0.5 and len(q) > 1:
                q.pop()
            q.append(p[-1])
            ln.set('shape', _write(q))
    ET.indent(tree, space='    ')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)


def apply_net(net_path):
    net = ET.parse(net_path).getroot()
    if any(e.get('id') == 'nw_r3a' for e in net.findall('edge')):
        raise RuntimeError('북서쪽 상가 골목이 이미 적용됨')
    _remove_old_side(net_path)
    _trim_br_stub(net_path)
    net = ET.parse(net_path).getroot()
    F, nodes, streets, splits, shapes = plan(net)
    new_nodes = {k: v for k, v in nodes.items() if k not in ('nw_K1', 'nw_C1L')}
    extra = [(LEG, 'nw_r3a', 0, 0),                              # 검단로 → R3 우회전
             ('T_end_main_in_a', 'nw_c1l', 0, 0),                # 검단로 → C1 우회전
             ('nw_r4_f', 'nw_r4_r', 0, 0),                       # 종다리공원 회차로에서 유턴
             # R1(일방, 서쪽으로) 끝: 445번길 남행 좌회전은 가장 가까운 안쪽 차로로만, 북행 우회전은 바깥 차로로 (2026-11-12)
             ('nw_r1b', 'nw_445c_r', 0, W445_LANES[1] - 1), ('nw_r1b', 'nw_445d_f', 0, 0),
             ] + [('nw_wj1_r', 'X_nw_in', k, k) for k in range(WJ_IN_LANES)] + TRI_CON   # 완정로 4차로 → 검단사거리 쪽 4차로 그대로
    graft(net_path, new_nodes, streets, splits, attach={'T_side_end', 'X_nw_end', 'T_main_end', 'N_end0', 'N_end1'},
          extra_con=extra, node_types={'nw_F0': 'dead_end', 'nw_N0': 'dead_end', 'nw_H0': 'dead_end', 'nw_S0': 'dead_end',
                      **({'nw_X': 'traffic_light'} if CROSS_TLS else {})},
          node_shapes=shapes)
    _strip_short_internal(net_path, ('nw_JW', 'nw_X', 'T_side_end', 'nw_L', 'nw_Y', 'nw_K'))
    return {'nw_streets': len(streets), 'nw_junctions': len(JUNCTIONS)}


# ───────────────────────── 노면 표시 ─────────────────────────

def _turn(a_dir, b_dir):
    cross = a_dir[0]*b_dir[1] - a_dir[1]*b_dir[0]
    dot = a_dir[0]*b_dir[0] + a_dir[1]*b_dir[1]
    ang = math.degrees(math.atan2(cross, dot))
    return 'straight' if abs(ang) < 35 else ('left' if ang > 0 else 'right')


def _dir(pts, at_end):
    a, b = (pts[-2], pts[-1]) if at_end else (pts[0], pts[1])
    L = math.dist(a, b) or 1.0
    return ((b[0]-a[0])/L, (b[1]-a[1])/L)


ARROW_KINDS = {frozenset({'straight'}): ['straight'], frozenset({'left'}): ['left'],
               frozenset({'right'}): ['right'], frozenset({'straight', 'left'}): ['straight_left'],
               frozenset({'straight', 'right'}): ['straight_right'], frozenset({'left', 'right'}): ['left', 'right'],
               frozenset({'straight', 'left', 'right'}): ['straight_left', 'right']}


def _dashes(root, prefix, path, dash, gap):
    L = _length(path)
    if L < dash:
        return 0
    count = max(2, round((L + gap) / (dash + gap)))
    g = (L - count*dash) / (count - 1)
    for i in range(count):
        a = i*(dash + g)
        ET.SubElement(root, 'poly', dict(id=f'{prefix}_{i}', color=WHITE, fill='0', lineWidth='0.15',
                                          layer='6', shape=_write(slice_(path, a, a + dash))))
    return count


def apply_deco(deco_path, net_path):
    from env.roundabout_builder import _arrow, _read_lane_shapes, _stop_line
    from env.roundabout_geometry import _offset
    from env.roundabout_markings import dash_style
    net = ET.parse(net_path).getroot()
    edges = {e.get('id'): e for e in net.findall('edge') if e.get('function') != 'internal'}
    lane = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge') for l in e.findall('lane')}
    shapes = _read_lane_shapes(net_path)
    tree = ET.parse(deco_path)
    root = tree.getroot()
    for p in list(root.findall('poly')):
        if p.get('id', '').startswith('nw_'):
            root.remove(p)
    dash, gap, _ = dash_style(20)
    # 1) 양방향 길 노란 중앙선 (_f 엣지 맨 왼쪽 차로의 왼쪽 경계), 여러 차로 길 흰 점선
    n_center = n_div = 0
    wj_mid = {}
    for eid, e in edges.items():
        if not eid.startswith('nw_'):
            continue
        nl = len(e.findall('lane'))
        if eid.startswith('nw_wj') and eid.endswith('_f'):
            # 완정로: 기존 북서쪽 큰길처럼 1m 분리대(도로색 포장) 가운데 이중 노란선(0.3m 간격)
            mid = _offset(lane[f'{eid}_{nl-1}'], LANE_W/2 + WJ_MEDIAN/2)
            wj_mid[e.get('from')] = mid
            ET.SubElement(root, 'poly', dict(id=f'{eid[:-2]}_median', color='84,84,90', fill='0',
                                              lineWidth=f'{WJ_MEDIAN}', layer='5', shape=_write(mid)))
            for k2, off in enumerate((-0.15, 0.15)):
                ET.SubElement(root, 'poly', dict(id=f'{eid[:-2]}_center_{k2}', color=YELLOW, fill='0',
                                                  lineWidth='0.15', layer='6', shape=_write(_offset(mid, off))))
            n_center += 1
        elif eid.endswith('_f'):
            ET.SubElement(root, 'poly', dict(id=f'{eid[:-2]}_center', color=YELLOW, fill='0', lineWidth='0.15',
                                              layer='6', shape=_write(_offset(lane[f'{eid}_{nl-1}'], LANE_W/2))))
            n_center += 1
        for k in range(nl - 1):
            n_div += _dashes(root, f'{eid}_div{k}', _offset(lane[f'{eid}_{k}'], LANE_W/2), dash, gap)
    # 2) 없앤 옛 445번길 아래쪽(검단로 T자 옆길)의 노면 표시를 지운다 (2026-11-12 먹거리타운 확장)
    from shapely.geometry import LineString as _LS, Polygon as _PG
    from shapely.ops import unary_union as _uu
    n_taper = 0
    main_ids = ('T_end_main_in', 'T_end_main_near', 'T_end_main_out', 'T_main_in')
    keep_paint = _uu([_LS(v).buffer(1.2) for k, v in lane.items() if k.rsplit('_', 1)[0] in main_ids])
    side_pre = ('t_center_end_side', 't_median_surface_end_side', 't_user_guide_', 'end_t_smooth_corner', 'under',
                'lane_dash_T_end_side', 't_arrow_T_end_side', 't_stop_end_side', 'stop_T_end_side')
    for p in list(root.findall('poly')):
        pid = p.get('id', '')
        if pid == 't_arrow_T_end_main_in_0_right':          # 검단로 → 옛 옆길 우회전 화살표 (직진 부분만 남긴다)
            root.remove(p)
            n_taper += 1
            continue
        if pid.startswith('end_t_smooth_corner') and _SIDE_CORRIDOR is not None and p.get('shape'):
            # 옛 옆길 입구 모서리를 메우던 아스팔트: 큰길 위 부분만 남긴다
            g = _PG(_read(p.get('shape'))).buffer(0).intersection(keep_paint.buffer(0.5))
            g = max(getattr(g, 'geoms', [g]), key=lambda x: x.area) if not g.is_empty else g
            if g.is_empty or g.area < 1.0 or g.geom_type != 'Polygon':
                root.remove(p)
            else:
                p.set('shape', _write(list(g.exterior.coords)))
            n_taper += 1
            continue
        if _SIDE_CORRIDOR is None or not p.get('shape') or not pid.startswith(side_pre):
            continue
        pts = _read(p.get('shape'))
        g = _LS(pts) if (p.get('fill') == '0' or len(pts) < 3) else _PG(pts).buffer(0)
        if g.is_empty or not g.intersects(_SIDE_CORRIDOR):
            continue
        meas = (lambda x: x.length) if g.geom_type in ('LineString', 'MultiLineString') else (lambda x: x.area)
        frac = meas(g.intersection(_SIDE_CORRIDOR)) / max(meas(g), 1e-9)
        on_main = meas(g.intersection(keep_paint)) / max(meas(g), 1e-9) > 0.5
        if frac >= 0.6 and not on_main:
            root.remove(p)
            n_taper += 1
    if BM_DASH:
        n_div += _dashes(root, 'nw_BM_div', list(BM_DASH), dash, gap)
    # 2-0) 지운 분리 구간(회전교차로 북서쪽 도로 뒤쪽) 위 노면 표시: 점선은 지우고, 차로 밑 바닥은 남긴 구간까지만
    if _BR_CUT is not None:
        for p in list(root.findall('poly')):
            pid = p.get('id', '')
            if pid.startswith('nw_') or pid == 'ground' or not p.get('shape'):
                continue
            pts = _read(p.get('shape'))
            g = _LS(pts) if (p.get('fill') == '0' or len(pts) < 3) else _PG(pts).buffer(0)
            if g.is_empty or not g.intersects(_BR_CUT):
                continue
            if g.geom_type == 'LineString':
                if g.intersection(_BR_CUT).length / max(g.length, 1e-9) > 0.5:
                    root.remove(p)
                    n_taper += 1
            else:
                rest = g.difference(_BR_CUT)
                if rest.is_empty or rest.area < 1.0:
                    root.remove(p)
                else:
                    rest = max(getattr(rest, 'geoms', [rest]), key=lambda x: x.area)
                    p.set('shape', _write(list(rest.exterior.coords)))
                n_taper += 1
    # 2-1) 완정로 합류부(옛 북서쪽 큰길 막다른 끝): 끝 정지선과 합류부 안에 남은 점선 토막을 지운다
    from shapely.geometry import LineString, Polygon
    xj = next(j for j in net.findall('junction') if j.get('id') == 'X_nw_end')
    xpoly = Polygon(_read(xj.get('shape'))).buffer(0.5)
    # 기존 큰길 좌표 (끝 = X_nw_end, ax: 검단사거리 → 끝, lat: 왼쪽 +)
    xn = (float(xj.get('x')), float(xj.get('y')))
    xin_shape = _read(edges['X_nw_in'].get('shape'))
    ax = (xin_shape[0][0] - xin_shape[-1][0], xin_shape[0][1] - xin_shape[-1][1])
    al = math.hypot(*ax)
    ax = (ax[0]/al, ax[1]/al)
    lon = lambda q: (q[0]-xn[0])*ax[0] + (q[1]-xn[1])*ax[1]
    lat = lambda q: (q[0]-xn[0])*(-ax[1]) + (q[1]-xn[1])*ax[0]
    for p in list(root.findall('poly')):
        pid = p.get('id', '')
        if pid.startswith('nw_') or not p.get('shape'):
            continue
        pts = _read(p.get('shape'))
        short_white = p.get('fill') == '0' and len(pts) == 2 and math.dist(*pts) < 5.0 and p.get('color') == WHITE
        mid = ((pts[0][0]+pts[-1][0])/2, (pts[0][1]+pts[-1][1])/2)
        in_widening = -WJ_TAPER - 1.0 <= lon(mid) <= 0.5 and -8.0 <= lat(mid) <= 14.0
        on_median = pid.startswith('x_nw_sketched_divider') and abs(lat(mid)) < 1.0 and lon(mid) > -56.0
        if pid == 'x_nw_outer_stop' or pid.startswith('x_nw_outer_arrow') or on_median or \
                (short_white and (xpoly.contains(LineString(pts)) or in_widening)):
            root.remove(p)
            n_taper += 1
        elif pid.startswith('x_center_nw_'):
            # 옛 막다른 끝 앞에서 들어오는 차로 쪽으로 꺾이던 중앙선 → 분리대 가운데로 곧게 (연장 도로와 이어지게)
            keep = [q for q in pts if lon(q) <= -56.0]
            if keep and len(keep) < len(pts):
                l0 = lat(keep[-1])
                end = (xn[0] + (-ax[1])*l0, xn[1] + ax[0]*l0)
                p.set('shape', _write(keep + [end]))
    ET.SubElement(root, 'poly', dict(id='nw_wj_widening_surface', color='84,84,90', fill='1', layer='-1',
                                      shape=_write(_read(xj.get('shape')))))
    # 3) 정지선(우선순위 낮은 쪽)과 진행 방향 화살표
    by_j = {}
    for c in net.findall('connection'):
        f = c.get('from')
        if f.startswith(':') or f not in edges or edges[f].get('to') not in JUNCTIONS:
            continue
        a = lane[f'{f}_{c.get("fromLane")}']
        b = lane[f'{c.get("to")}_{c.get("toLane")}']
        by_j.setdefault(edges[f].get('to'), {}).setdefault((f, int(c.get('fromLane'))), set()).add(
            _turn(_dir(a, True), _dir(b, False)))
    marks = []
    n_stop = n_arrow = 0
    main_like = {LEG, f'{LEG}_b', 'T_end_main_in_a', 'T_end_main_in'}
    for jn, approaches in by_j.items():
        incoming = {f for f, _ in approaches}
        top = max(int(edges[f].get('priority', '0')) for f in incoming)
        arms = incoming | {c.get('to') for c in net.findall('connection')
                           if c.get('from') in incoming and not c.get('from').startswith(':')}
        if len(arms) < 3 and not (incoming & main_like):
            continue                                    # 굽은 길·차로 수만 바뀌는 곳은 표시 없음
        for f in sorted(incoming):
            if f in main_like:
                continue
            if int(edges[f].get('priority', '0')) < top:
                _stop_line(marks, f'nw_stop_{f}', shapes, f, dist=1.0)
                n_stop += 1
        for (f, k), turns in sorted(approaches.items()):
            if f in main_like:
                if turns == {'straight', 'right'} and _length(lane[f'{f}_{k}']) > 32.0:
                    _arrow(marks, f'nw_arrow_{f}_{k}', shapes, f, k, 'straight_right', 25.0)
                    n_arrow += 1
                continue
            if _length(lane[f'{f}_{k}']) < 7.5 or not f.startswith('nw_') and f not in BLUE:
                continue
            for kind in ARROW_KINDS.get(frozenset(turns), []):
                _arrow(marks, f'nw_arrow_{f}_{k}_{kind}', shapes, f, k, kind,
                       min(7.0, _length(lane[f'{f}_{k}']) - 3.5))
                n_arrow += 1
    for m in marks:
        root.append(ET.fromstring(m))
    ET.indent(tree, space='    ')
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    return {'nw_centerlines': n_center, 'nw_lane_dashes': n_div, 'nw_stoplines': n_stop, 'nw_arrows': n_arrow,
            'nw_taper_dashes_removed': n_taper}


def main():
    import sys
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, base)
    if os.environ.get('SUMO_HOME'):
        sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    net_dir = os.path.join(base, 'env', 'sumo_roundabout')
    net_path = os.path.join(net_dir, 'roundabout.net.xml')
    deco_path = os.path.join(net_dir, 'roundabout.deco.xml')
    info = apply_net(net_path)
    from env import roundabout_south_link as link
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(apply_deco(deco_path, net_path))
    print(info)


if __name__ == '__main__':
    main()
