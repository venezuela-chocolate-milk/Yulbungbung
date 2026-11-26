"""회전교차로 맵(env/sumo_roundabout)으로 강화학습할 때 쓰는 설정.

train.py / test.py 에서 MAP = "roundabout" 으로 두면 이 파일 값이 쓰인다.
(MAP = "highway" 면 기존 env/road_config.py 의 직선 고속도로)

★ 맵 파일은 학습 때 다시 만들지 않는다.
  env/sumo_roundabout/ 의 도로는 roundabout_builder 로 만든 뒤 여러 패치
  (env/roundabout_south_extend.py 파이프라인)로 손질한 결과라서,
  builder 를 다시 돌리면 그 손질이 전부 사라진다. 그래서 학습/평가는
  이미 있는 roundabout.sumocfg 를 그대로 불러오기만 한다.
  도로를 다시 만들어야 하면:  python view_roundabout.py --rebuild
"""
import os

MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sumo_roundabout")
SUMOCFG = os.path.join(MAP_DIR, "roundabout.sumocfg")

# ──────────────────────────────────────────────────────────────
# 1) 도로 (관측/보상 정규화용 값만 — 실제 도로는 net.xml)
# ──────────────────────────────────────────────────────────────
ROAD = {
    "speed_limit": 16.67,  # 맵의 최고 제한속도 (60 km/h). vmax 정규화 기준
    "num_lanes": 3,        # 직선도로 전용 값(프리필/중앙차선 계산). 여기선 프리필을 끈다
    "length": 0.0,         # 직선도로 전용 값. 회전교차로 맵에선 쓰지 않음
}

# ──────────────────────────────────────────────────────────────
# 2) ego 경로 — 엄마 심부름 경로 하나 (2026-11-15 확정, env/mom_route.py → sumo_roundabout/mom_route.json)
#    출발: 왕길유승한내들아파트 주차장 → 단봉로 정문 바로 좌회전 → 회전교차로 서쪽 아래 진입
#    들를 곳 (매번 주차장에 들어가 통로에 2초 정차): 1 큰손왕만두 공영주차장, 2 완정초, 3 마전어린이집,
#      4 새마을금고, 5 동아아파트(외할머니, 여기 정차 = 완주). 약 5.8 km.
#    지나는 불법 주정차 구간: 큰손왕만두 오른쪽 상가 골목, 새마을금고 위 윗 7번길 (env/roundabout_illegal_parking.py)
#    지나는 어린이보호구역(30 km/h, 단속 카메라): 완정초·마전중 앞 완정로
#    (예전 세 경로 north_curve_uturn / south_straight / north_south_left 는 reports/before-mom-route-20261115/ 에 있음)
# ──────────────────────────────────────────────────────────────
def _mom_route():
    from env import mom_route
    d = mom_route.load()
    return {"name": "mom", "edges": d["edges"], "stops": d["stops"],
            "depart_pos": d["depart"]["pos"], "depart_lane": "best", "depart_speed": 0.0, "weight": 1.0}


ROUTES = [_mom_route()]

EGO = {
    "accel": 5.4,          # 최대 가속도 (m/s^2) → 행동 a=+1  (수업 원본과 같게, 2026-11-08 3.0 → 5.4)
    "decel": 5.4,          # 최대 감속도 (m/s^2) → 행동 a=-1  (수업 원본과 같게, 2026-11-08 4.5 → 5.4)
    "max_speed": 16.67,
    "depart_lane": "center",  # 직선도로식 기본값. 엄마 경로는 출발 주차장 통로(1차로)라 경로 설정의 depart_lane 을 쓴다
    "depart_speed": 8.0,      # 엄마 경로는 경로 설정의 depart_speed 0 (주차장에서 멈춘 채 출발)
    # ── 개입 수준: 수업 원본(env/sumo_env.py 초기 버전)과 같게 (2026-11-08) ──
    # speed_mode 0: SUMO 속도 안전장치 전부 끔. 신호·양보·추돌 회피 모두 정책이 직접 한다 (원본과 같음)
    "speed_mode": 0,
    # lane_change_mode 0: SUMO 는 스스로 차로를 바꾸지 않고(경로 유지 차선변경 대행도 없음),
    #   정책이 명령한 차선변경에 대한 SUMO 쪽 안전 확인도 없다 (원본과 같음).
    #   정책 차선변경의 안전 확인은 원본 코드의 차선변경 안전 게이트(mdp_config ACTION
    #   lane_change_safety)만 한다. 경로에 맞는 차로로 가는 것도 정책이 배워야 한다.
    #   (이전 값: 514 = SUMO 가 경로 유지 차선변경 대행, 512 = SUMO 가 안전 확인)
    "lane_change_mode": 0,
}

# ──────────────────────────────────────────────────────────────
# 3) 배경 교통 — 맵의 roundabout.rou.xml + 새 도로용 roundabout.rl.rou.xml
#    (roundabout.rl.rou.xml 은 env/roundabout_rl_traffic.py 의 FLOWS 로 생성)
#    직선도로식 프리필은 끄고, 대신 ego 투입 전에 warmup 으로 도로를 채운다.
#
#    scale: 두 파일의 모든 흐름에 곱하는 배율 (SUMO --scale)
#      0.8 = 기준 교통량 (2026-11-07 안정성 점검으로 확정: 60분 배경 교통 충돌 0·순간이동 0)
#            북동 도로 차로당 약 280대/h, 유턴 때 마주 오는 차 차로당 약 150대/h,
#            양보 T자로 오는 서행 차 차로당 약 105대/h
#      0.9 부터 오른쪽 아래 십자 교차로·북동 도로 옆길에서 가끔 막힘, 1.0 이상은 정체·충돌 증가
#      커리큘럼: 학습 초반 0.4 → 본 학습/최종 평가 0.8 / 추가 시험 1.0 (혼잡)
#      학습 중에는 env.traffic_scale 을 바꾸면 다음 에피소드부터 적용된다.
# ──────────────────────────────────────────────────────────────
# 2026-11-17: 구간(출발 도로)별 교통량 최대치 (traffic_tune.py) — 아래 기본 경로 파일의 흐름마다
#   (파일 값 × 0.8 × 그 구간 배율) 로 쓴 사본(sumo_roundabout/scaled/)을 쓰고 SUMO --scale 은 1.0.
#   배율표: sumo_roundabout/traffic_profile.json (배경 교통만 6개 시드 2100초에서 순간이동·충돌 0, 출발 지연 기준 통과)
TRAFFIC_BASE_ROUTE_FILES = [os.path.join(MAP_DIR, "roundabout.rl_base.rou.xml"),   # 기존 흐름 (경로 수정 사본)
                    os.path.join(MAP_DIR, "roundabout.rl.rou.xml"),
                    # 2026-11-10: 학교 오가는 차(학교 안에서 사라지고 출발), 상가 골목 오가는 차
                    os.path.join(MAP_DIR, "roundabout.school.rou.xml"),
                    os.path.join(MAP_DIR, "roundabout.shops.rou.xml"),
                    os.path.join(MAP_DIR, "roundabout.nw.rou.xml"),      # 검단사거리역 북서쪽 상가 골목
                    os.path.join(MAP_DIR, "roundabout.majeon.rou.xml"),   # 2026-11-12: 마전동 골목
                    # 2026-11-12: 블록 S 공영주차장 주차 차량 (parkingArea 는 roundabout.deco.xml 에 있음)
                    os.path.join(MAP_DIR, "roundabout.parking.rou.xml"),
                    # 2026-11-12: 마전어린이집 주차장 차 (env/roundabout_majeon_daycare.py)
                    os.path.join(MAP_DIR, "roundabout.daycare.rou.xml"),
                    # 2026-11-12: 마전주공아파트 아래쪽 공영주차장 차 (env/roundabout_wanggil_parking.py)
                    os.path.join(MAP_DIR, "roundabout.wanggil_parking.rou.xml"),
                    # 2026-11-12: 마전동 ② 블록 공영주차장 차 (env/roundabout_majeon_parking2.py)
                    os.path.join(MAP_DIR, "roundabout.majeon_parking2.rou.xml"),
                    # 2026-11-13: 동아아파트 주차장 차 (env/roundabout_donga_parking.py)
                    os.path.join(MAP_DIR, "roundabout.donga_parking.rou.xml"),
                    # 2026-11-13: 롯데마트 주차장 차 (env/roundabout_lotte_parking.py)
                    os.path.join(MAP_DIR, "roundabout.lotte_parking.rou.xml"),
                    # 2026-11-13: 검단피오레대주1차 주차장 차 (env/roundabout_fiore_parking.py)
                    os.path.join(MAP_DIR, "roundabout.fiore_parking.rou.xml"),
                    # 2026-11-14: 검단1차풍림아이원·검단원흥아파트 주차장 차 (env/roundabout_wanggil_south_parking.py)
                    os.path.join(MAP_DIR, "roundabout.wanggil_south_parking.rou.xml"),
                    # 2026-11-14: 새마을금고 주차장 차 (env/roundabout_majeon_parking5.py)
                    os.path.join(MAP_DIR, "roundabout.majeon_parking5.rou.xml"),
                    # 2026-11-14: 왕길119안전센터 소방차·구급차 출동 (env/roundabout_emergency.py, 주기 PERIOD_S)
                    os.path.join(MAP_DIR, "roundabout.emergency.rou.xml"),
                    # 2026-11-14: 왕길유승한내들아파트 주차장 차 (env/roundabout_yuseung.py)
                    os.path.join(MAP_DIR, "roundabout.yuseung.rou.xml"),
                    # 2026-11-17: 유턴 마주 오는 방향 교통 (진입로가 포화라 그 도로에서 출발하는 흐름)
                    os.path.join(MAP_DIR, "roundabout.uturn_opposing.rou.xml"),
                    # 2026-11-18: 구간 안에서 들어오고 나가는 교통 (env/roundabout_local_traffic.py, traffic_tune.py local2 로 배율)
                    os.path.join(MAP_DIR, "roundabout.local.rou.xml")]
# 2026-11-18: 구간 안에서 들어오고 나가는 교통(env/roundabout_local_traffic.py → roundabout.local.rou.xml)은 아직 목록에 넣지 않음.
#   기본량(차로당 100/60 대/h)으로도 검단사거리 쪽 진입로(SW_final_left_in, T_stop_side_in_a)가 크게 막혀서 탐색 중단.
#   다음 작업: 기본량을 낮춰(예: 0.1~0.25배부터) traffic_tune.py local 로 다시 찾은 뒤 위 목록 끝에 추가하고 build.
# 2026-11-24 (사용자 확정 B안): scaled/ 위에 원형교차로 남·북·단봉 진입 ×2 + 북·동 진입 흐름 차종 *_rbN(jmTimegapMinor 0.5, impatience 0.3).
#   만든 스크립트 results/ring_test/make_var.py B (백업 results/backup_before_C_20261121/scaled_pre_B_20261124).
#   traffic_tune.py build 를 다시 돌리면 B안이 지워지므로, 그 뒤에는 make_var.py B 결과를 다시 덮어쓸 것.
_SCALED_DIR = os.path.join(MAP_DIR, "scaled")
_USE_SCALED = all(os.path.exists(os.path.join(_SCALED_DIR, os.path.basename(f))) for f in TRAFFIC_BASE_ROUTE_FILES)
TRAFFIC = {
    "prefill": {"enabled": False},
    "vehs_per_hour": 0,
    "route_files": ([os.path.join(_SCALED_DIR, os.path.basename(f)) for f in TRAFFIC_BASE_ROUTE_FILES]
                    if _USE_SCALED else TRAFFIC_BASE_ROUTE_FILES),
    # 2026-11-18: 배율 사본 위에 전체 0.9 (빠른 시작 버그 수정 후 강사 108판: 1.0 → 완주 90.7%·충돌 3.7%,
    #   0.9 → 96.3%·0.9%, 0.8 → 99.1%·0.9%). 학습 가능한 선에서 최대.
    "scale": 1.0 if _USE_SCALED else 0.8,     # 2026-11-18 구간 흐름(local3) 넣으며 1.0 으로 되돌림 (아래 강사 평가로 확인)
}

# GUI 로 띄울 때만 불러오는 그림 파일 (노면 표시 + 신호등 그림).
# 신호등 그림은 env/roundabout_signal_heads.py 가 만들고, 실행 중 실제 신호에 맞춰 켜진다.
GUI_ADDITIONAL = [os.path.join(MAP_DIR, "roundabout.deco.xml"),
                  os.path.join(MAP_DIR, "roundabout.signals.add.xml")]
# 주변 풍경(건물·숲·골목길, env/roundabout_scenery.py 가 네이버 지도/위성 사진으로 생성) — 있으면 맨 앞(맨 아래)에
# 2026-11-11 사용자 요청으로 꺼 둔다. 파일은 지우지 않고 남겨 두었다 — 다시 보려면 True 로.
SHOW_SCENERY = False
_SCENERY = os.path.join(MAP_DIR, "roundabout.scenery.add.xml")
if SHOW_SCENERY and os.path.exists(_SCENERY):
    GUI_ADDITIONAL.insert(0, _SCENERY)
# 학교 진입로 양쪽 완정초·마전중 그림 (env/roundabout_school_buildings.py 가 생성) — 있으면 도로 그림보다 아래에
_SCHOOL = os.path.join(MAP_DIR, "roundabout.school.add.xml")
if os.path.exists(_SCHOOL):
    GUI_ADDITIONAL.insert(0, _SCHOOL)
# 상가 골목 보도·상가 그림 (env/roundabout_shop_buildings.py 가 생성)
_SHOPS = os.path.join(MAP_DIR, "roundabout.shops.add.xml")
if os.path.exists(_SHOPS):
    GUI_ADDITIONAL.insert(0, _SHOPS)
# 검단사거리역 북서쪽 상가 골목 보도·상가·공원 그림 (env/roundabout_nw_buildings.py 가 생성)
_NW = os.path.join(MAP_DIR, "roundabout.nw.add.xml")
if os.path.exists(_NW):
    GUI_ADDITIONAL.insert(0, _NW)
# 큰 루프 안쪽 아파트·상가·공장·산 그림 (env/roundabout_interior_scenery.py 가 생성, 2026-11-11) — 맨 아래에
_INTERIOR = os.path.join(MAP_DIR, "roundabout.interior.add.xml")
if os.path.exists(_INTERIOR):
    GUI_ADDITIONAL.insert(0, _INTERIOR)
# 마전동 블록 보도·건물·간판·우버니공원 (env/roundabout_majeon_buildings.py 가 생성, 2026-11-12)
_MAJEON = os.path.join(MAP_DIR, "roundabout.majeon.add.xml")
if os.path.exists(_MAJEON):
    GUI_ADDITIONAL.insert(0, _MAJEON)
# 왕길동 아파트·왕길초·왕길근린공원·숲 (env/roundabout_wanggil_scenery.py 가 생성, 2026-11-12) — 맨 아래에
_MAJEON_N = os.path.join(MAP_DIR, "roundabout.majeon_north.add.xml")  # 마전동 북쪽 (env/roundabout_majeon_north_scenery.py)
if os.path.exists(_MAJEON_N):
    GUI_ADDITIONAL.insert(0, _MAJEON_N)
_MAJEON_E = os.path.join(MAP_DIR, "roundabout.majeon_east.add.xml")   # 마전동 동쪽 (env/roundabout_majeon_east_scenery.py)
if os.path.exists(_MAJEON_E):
    GUI_ADDITIONAL.insert(0, _MAJEON_E)
_WANGGIL = os.path.join(MAP_DIR, "roundabout.wanggil.add.xml")
if os.path.exists(_WANGGIL):
    GUI_ADDITIONAL.insert(0, _WANGGIL)
_GEOMDAN = os.path.join(MAP_DIR, "roundabout.geomdan.add.xml")   # 검단사거리 북동쪽 (env/roundabout_geomdan_scenery.py, 2026-11-12)
if os.path.exists(_GEOMDAN):
    GUI_ADDITIONAL.insert(0, _GEOMDAN)
_BAESUJI = os.path.join(MAP_DIR, "roundabout.baesuji.add.xml")   # 165번길 동쪽 빌라·왕길배수지공원 (env/roundabout_baesuji_scenery.py)
if os.path.exists(_BAESUJI):
    GUI_ADDITIONAL.insert(0, _BAESUJI)
SIGNAL_HEADS = os.path.join(MAP_DIR, "roundabout.signals.add.xml")

# mdp_config.SIMULATION 에서 덮어쓸 값
SIMULATION_OVERRIDES = {
    "warmup_steps": 400,   # 0.5초 × 400 = 200초 동안 배경차가 맵을 채운 뒤 ego 투입
    "max_steps": 6000,     # 3000초 (2026-11-18 사용자 승인: 교통량 v16 에서 전문가 평균 1724초라 2400→3000초로 여유 확대)
    "gui_zoom": 1500.0,
}

# ──────────────────────────────────────────────────────────────
# 4) 도심 경로 주행: 관측 추가 · 안전 마스킹 · 보상 (env/urban_driving.py)
#    회전교차로 맵에서는 이 보상이 train.py 의 REWARD(직선도로용) 대신 쓰인다.
# ──────────────────────────────────────────────────────────────
URBAN = {
    # 안전 마스킹(규칙 기반 실드)은 쓰지 않는다: 일반적인 SUMO 강화학습 프로젝트에는 없음.
    # 끈 상태에서도 "켜져 있었다면 개입했을 상황"은 가상 개입률로 기록만 한다 (분석용, 주행에 영향 없음).
    # 비교 실험을 하려면 True 또는 test.py --mask
    "mask_enabled": False,
    # 2026-11-17 컨설팅 반영(A안): 수업 원본 관측의 "차로별 앞뒤차" 20칸을 빼고 가려짐 있는 주변 차량 관측으로 통일
    #   (원본은 가려진 차까지 보이고 다른 도로의 차는 못 봄). 내 속도·차로 끊김·차로 밀도 11칸은 유지 → 관측 72차원
    "drop_lane_neighbors": True,
    # 2026-11-19 (사용자 승인): 수업 원본 20칸 중 "현재 차로 앞차" 2칸(범퍼 사이 간격/50 m, 상대속도)만 되살림.
    #   원본 _neighbor(0, leader=True) = traci getLeader 와 같은 정의. 건물에 가리면 "없음"(1, 1).
    #   주변 차량 6대를 고를 때는 이 앞차를 뺀다(겹침 없음; 단 긴급차는 주변 칸의 긴급차 표시를 위해 남김) → 관측 89차원
    #   이유: 87차원 모방학습 모델의 충돌이 전부 "멈춘 앞차를 뒤에서 박음" — 주변 6대 칸은 앞범퍼 위치라 차 길이를 모르고
    #   앞차의 칸 위치가 매번 바뀜 (results/coll_diag_d2s0_20261119.log)
    "ego_leader": False,   # 2026-11-19 오후: 아래 8방향 물체 인식의 "앞" 칸이 대신함 (urban_driving._sectors)
    # 수업 원본에 있는 차선변경 안전 게이트(mdp_config ACTION lane_change_safety: 앞뒤 간격
    # 3 m + 속도 비례, 출발 60 m 변경 금지)는 원본과 같게 켠다. 출발 60 m 금지는 원본의
    # "도로 시작점" 규칙을 경로의 첫 도로(출발 도로)에 적용한 것.
    "lane_change_gate": True,
    "comfort_decel": 3.0,   # 황색에서 "편하게 멈출 수 있는지" 판단 감속도 (m/s²)
    "mask_decel": 4.5,      # 마스킹이 가정하는 제동 감속도 (= ego 최대 감속)
    "stop_margin": 1.0,     # 정지선/교차점 앞 여유 (m)
    "min_gap": 2.0,         # 앞차 급정거 시에도 남길 최소 간격 (m)
    "yield_window": 2.0,    # 양보 대상과 교차점 도착 시간이 이 초 이내로 겹치면 진입 차단
    "junction_min_speed": 5.0,  # 교차로 안에서 앞이 비어 있을 때 최소 속도 (m/s, 18 km/h)
    "reward": {
        # 2026-11-08 마스킹 제거에 맞춰 조정: 신호 위반 = 즉시 실패, 충돌 30 → 50,
        #   앞차 간격은 시간 간격 기준, 개입 감점 제거
        "progress_weight": 0.1,       # 제한속도(16.67 m/s)로 달리면 스텝당 +0.1 (빨강 경로 완주 ≈ +45)
        "arrival_bonus": 10.0,        # 완주
        # 실패 감점은 서로 비슷하게: "서 있다가 막혀서 끝나기"가 "가다가 충돌"보다 싸면
        #   정책이 아예 안 가는 쪽을 배운다 (2026-11-08 짧은 학습에서 확인)
        "collision_penalty": 50.0,    # 충돌 (즉시 종료)
        "red_violation_penalty": 30.0,  # 적신호 통과 (즉시 종료)
        "lost_penalty": 50.0,         # 오래 막혀 순간이동 / 경로 이탈 / 틀린 차로로 막힘 (즉시 종료)
        # 2026-11-17 컨설팅 반영(보상 항은 꼭 필요한 것만): 틀린 차로 감점(차선 단위 경로 정보)·불가능한 차로변경
        #   감점·마스킹 개입 감점 제거, '오래 서 있기'+'이유 없이 멈춤'을 멈춤 감점 하나로 합침
        "stall_after_s": 30.0,        # 적·황색 신호 대기가 아닌데 이 시간(초) 넘게 서 있으면 멈춤 감점
        "stop_penalty": 0.05,         # 멈춤: 이유(가까운 적·황색 신호/양보할 상충 차량/15 m 안 앞차) 없이 서 있거나 위처럼 오래 서 있으면 스텝마다
        "time_penalty": 0.02,         # 매 스텝 (기어가며 버티기 방지. 제한속도 주행 시 순보상 +0.08)
        "speeding_penalty": 0.1,      # 차로 제한속도의 110% 초과 시 (초과 비율 × 이 값)
        "close_gap_threshold": 3.0,   # 앞차와 간격이 max(이 거리, 내 속도 × headway_s) 미만이면
        "headway_s": 1.0,             #   (시속 50 km 면 약 14 m)
        "close_gap_penalty": 0.05,    #   스텝마다 감점
        # 2026-11-24 A안: 교차로 안에서 움직이며(0.5 m/s 이상) 보이는 차와 몸체 사이가 3 m 안이고 줄어드는 중이면 스텝마다 감점
        #   (거리가 일정한 나란한 옆 차로 차는 감점 안 됨)
        "junction_close_m": 3.0,
        "junction_close_penalty": 0.2,
        "lane_change_penalty": 0.01,  # 실제로 차로를 바꿀 때마다 (지그재그 방지)
        # 2026-11-17 승차감: 저크(가감속 변화 / 0.5 s)가 기준을 넘은 만큼만 감점 (논문들의 편안한 저크 2~4 m/s³)
        "jerk_threshold": 8.0,        # m/s³ — 이하면 감점 없음. 2026-11-17 A안: 4 → 8 (강사가 8 을 넘는 스텝은 1.5% 뿐이라
                                      #   강사 흉내(BC)와 부딪히지 않고 정말 거친 조작만 억제)
        "jerk_penalty": 0.02,         # 감점 = 이 값 × (저크 − 기준) / 기준
    },
}

# ──────────────────────────────────────────────────────────────
# 5) 엄마 경로 과제 (env/mom_driving.py): 주차장 정차 · 긴급차 양보 · 단속 카메라 (2026-11-15)
#    보상 항목은 URBAN["reward"] 에 함께 둔다 (park_bonus, ev_block_penalty, camera_penalty)
#    완주 = 마지막 주차장 정차 → arrival_bonus. 들를 주차장을 지나치면 lost_penalty 로 실패.
# ──────────────────────────────────────────────────────────────
URBAN["reward"].update({
    "park_bonus": 10.0,          # 들를 주차장(1~4)에 들어가 통로에 2초 정차할 때마다
    "ev_block_penalty": 0.1,     # 긴급차가 내 뒤 같은 차로 60 m 안인데 옆 차로로 비키지 않은 스텝마다 (편도 1차로는 제외)
    "camera_penalty": 5.0,       # 어린이보호구역 단속 카메라를 제한속도(30 km/h) 넘게 지나가면 (한 번)
})
# 긴급차 투입 빈도 (용도별, 2026-11-15). 왕길119안전센터 정기 출동만으로는 ego 와 거의 안 만나서,
# ego 뒤에서 긴급차를 출발시켜 따라오게 한다 (env/mom_driving.py).
#   train  : 모방학습 데이터 수집·강화학습. 매 에피소드 1~2번 (2026-11-17 사용자 요청: 학습 중 매번 한 번은 만나게, 소방차만)
#            (정기 출동 소방차도 10분마다 따로 있음)
#   eval   : 결과 확인(test.py 기본). 만들어 넣는 긴급차 없음, 119 정기 출동만 (2026-11-17) — 실제 주행 조건
#   ev_only: 긴급차 대응만 따로 평가 (test.py --ev-only). 매 에피소드 1~2번
_EV_COMMON = {
    "behind_m": (200.0, 320.0),          # ego 뒤 이만큼 (그 도로 시작에서) 출발. 2026-11-17: 80~160 → 200~320 (시야·화면 밖)
    "inject_min_gap_steps": 500,         # 두 번 나올 때 출발 시각 최소 간격 (스텝)
    # 배경차도 긴급차에 비켜 준다 (TraCI 로 옆 차로 변경). 끄면 배경차가 긴급차를 막아
    # 긴급차가 ego 까지 오지 못하는 일이 잦고, 실제 도로와도 다르다.
    "yield_background": True,
}
EV_WINDOW = (40, 1800)                   # 이 스텝 범위의 무작위 시각. 2026-11-17: (40, 2800) → (40, 1800) — 엄마 경로 한 판 ≈ 2000~2300 스텝이라 늦게 잡히면 못 만남
# 2026-11-24 (사용자 승인): 학습·주 평가 모두 "긴급차와 한 번은 만날 때까지 투입" (119 정기 출동과 만나도 인정).
#   투입차는 ego 뒤 150~200 m 도로 시작에서 출발, 150초 안에 못 만나면 다시 투입, 빠져나갈 때는 119 정기 출동 경로로 합류.
#   교통 B안과 함께 켠다 (rl_C3 와 그 자동 평가가 끝날 때까지 False).
EV_MEET_UNTIL = True     # 11-24 사용자 확정 (B안 교통과 함께 켬)
_EV_MEET = {"meet_until": True, "retry_s": 150.0, "behind_m": (150.0, 200.0), "inject_window_steps": (40, 600)}
EMERGENCY_MODES = {
    "train":   {**_EV_COMMON, "inject_prob": 1.0,  "inject_count": (1, 2), "inject_window_steps": EV_WINDOW},  # 2026-11-17: 매 에피소드 최소 1번
    # 2026-11-17: 평가는 실제처럼 119안전센터 정기 출동만 (만들어 넣는 긴급차 없음). 긴급차 대응만 보려면 ev_only
    "eval":    {**_EV_COMMON, "inject_prob": 0.0,  "inject_count": (1, 1), "inject_window_steps": EV_WINDOW},
    "ev_only": {**_EV_COMMON, "inject_prob": 1.0,  "inject_count": (1, 2), "inject_window_steps": EV_WINDOW},
}
if EV_MEET_UNTIL:
    for _m in ("train", "eval"):
        EMERGENCY_MODES[_m] = {**EMERGENCY_MODES[_m], **_EV_MEET}
MOM = {"emergency": EMERGENCY_MODES["eval"]}   # 기본값. make_env(ev_mode=...) 가 용도별로 바꿔 끼운다

# 커리큘럼 (학습 때만, 기본 꺼짐 — 수업 원본에는 없음. train.py --curriculum default 로 켬):
# 누적 스텝에 따라 교통량 배율을 올린다. 평가(test.py)는 커리큘럼 없이 기준 교통량(TRAFFIC["scale"]).
# 커리큘럼 교통량 (TRAFFIC["scale"] 기준 비율로 곱함: 기존 0.4/0.6/0.8 = 최종의 0.5/0.75/1.0)
CURRICULUM = [
    {"until_step": 1_000_000, "scale": 0.5 * TRAFFIC["scale"]},
    {"until_step": 3_000_000, "scale": 0.75 * TRAFFIC["scale"]},
    {"until_step": None, "scale": 1.0 * TRAFFIC["scale"]},
]

NET_FILE = os.path.join(MAP_DIR, "roundabout.net.xml")

# 학습용 무작위 출발 (커리큘럼과 함께만 켜짐, 기본 꺼짐 — 수업 원본은 항상 도로 시작에서 출발):
# 에피소드의 prob 비율은 경로 중간의 무작위 도로(길이 min_edge_len 이상, 회전교차로·주차장 안 제외,
# 마지막 keep_last_edges 개 제외)에서 출발. 그 앞의 주차장은 들른 것으로 친다.
RANDOM_START = {"prob": 0.5, "min_edge_len": 50.0, "keep_last_edges": 2}

# 빠른 시작 (화면 없을 때만): SUMO 를 에피소드마다 다시 켜지 않고(7초), 배경차를 채운
# 준비 상태를 30초 간격으로 pool_size 개 저장해 두었다가 무작위로 불러온다(0.05초).
# 불러온 뒤 0~extra_steps_max 스텝(최대 10초) 더 진행해 매번 다른 교통에서 시작한다.
# 병렬 환경은 각자 다른 SUMO 시드로 자기 준비 상태를 만든다.
# 2026-11-18: 빠른 시작 끔. SUMO 1.27 loadState 를 한 프로세스에서 반복하면 판이 이어질수록 정체가 쌓임
#   (강사, 교통 0.7배, 9프로세스×8판: 불러오기 4~8판째 완주 11/45 vs 매 판 재시작 44/45 — 마전동 골목 시간초과).
#   매 판 SUMO 재시작 + 준비 진행 후 0~extra_steps_max 무작위 진행(신호 위상 다양화)은 유지.
FAST_RESET = {"enabled": False, "pool_size": 16, "spacing_steps": 47, "extra_steps_max": 180}
# 2026-11-08: 간격 30초(60스텝)는 신호 주기 90초와 맞물려 신호 위상이 3가지뿐이었다 →
#   23.5초 간격 + 불러온 뒤 0~90초(신호 한 주기) 무작위 진행으로 위상을 고르게

# SUMO 실행 옵션 추가: 교차로 안 충돌도 충돌로 판정 (회전교차로/신호교차로 필수)
# 불법 주정차 구간 (에피소드마다 무작위, 비상등) — env/roundabout_illegal_parking.py
# 2026-11-15: 엄마 경로의 상가 골목(큰손왕만두 오른쪽)·윗 7번길(새마을금고 위) 양방향 바깥 차로 (아래 7번길은 없앰)
ILLEGAL_PARKING = True

# time-to-teleport: 120 → 300 (2026-11-15). 서곶로×원당대로 좌회전 포켓(약 24 m)이 짧아 ego 가 포켓 진입을 기다리며
# 신호 두 번을 기다리면 120초를 넘겨 순간이동(실패) 처리되던 것. 배경 교통은 0.8 기준 60분 순간이동 0 이라 영향 없음.
SUMO_ARGS = ["--collision.check-junctions", "true",
             "--time-to-teleport", "300"]

# 2026-11-12: 마전어린이집 부지·주차장 그림 (env/roundabout_majeon_daycare.py) — 바닥이 층 198~200 이라 순서는 상관없다
_DAYCARE = os.path.join(MAP_DIR, "roundabout.daycare.add.xml")
if os.path.exists(_DAYCARE):
    GUI_ADDITIONAL.insert(0, _DAYCARE)
# 2026-11-12: 마전주공아파트 아래쪽 공영주차장 그림 (env/roundabout_wanggil_parking.py)
_WG_PARK = os.path.join(MAP_DIR, "roundabout.wanggil_parking.add.xml")
if os.path.exists(_WG_PARK):
    GUI_ADDITIONAL.insert(0, _WG_PARK)
# 2026-11-12: 마전동 ② 블록 공영주차장 그림 (env/roundabout_majeon_parking2.py)
_MJ_PARK2 = os.path.join(MAP_DIR, "roundabout.majeon_parking2.add.xml")
if os.path.exists(_MJ_PARK2):
    GUI_ADDITIONAL.insert(0, _MJ_PARK2)
# 2026-11-13: 동아아파트 주차장 그림 (env/roundabout_donga_parking.py)
_DONGA_PARK = os.path.join(MAP_DIR, "roundabout.donga_parking.add.xml")
if os.path.exists(_DONGA_PARK):
    GUI_ADDITIONAL.insert(0, _DONGA_PARK)
# 2026-11-13: 롯데마트 주차장 그림 (env/roundabout_lotte_parking.py)
_LOTTE_PARK = os.path.join(MAP_DIR, "roundabout.lotte_parking.add.xml")
if os.path.exists(_LOTTE_PARK):
    GUI_ADDITIONAL.insert(0, _LOTTE_PARK)
# 2026-11-13: 검단피오레대주1차 주차장 그림 (env/roundabout_fiore_parking.py)
_FIORE_PARK = os.path.join(MAP_DIR, "roundabout.fiore_parking.add.xml")
if os.path.exists(_FIORE_PARK):
    GUI_ADDITIONAL.insert(0, _FIORE_PARK)
# 2026-11-14: 왕길유승한내들아파트 단지·주차장 그림 (env/roundabout_yuseung.py)
_YUSEUNG = os.path.join(MAP_DIR, "roundabout.yuseung.add.xml")
if os.path.exists(_YUSEUNG):
    GUI_ADDITIONAL.insert(0, _YUSEUNG)
# 2026-11-14: 새마을금고 주차장 그림 (env/roundabout_majeon_parking5.py)
_MJ_PARK5 = os.path.join(MAP_DIR, "roundabout.majeon_parking5.add.xml")
if os.path.exists(_MJ_PARK5):
    GUI_ADDITIONAL.insert(0, _MJ_PARK5)
# 2026-11-14: 검단1차풍림아이원·검단원흥아파트 주차장 그림 (env/roundabout_wanggil_south_parking.py)
_WG_S_PARK = os.path.join(MAP_DIR, "roundabout.wanggil_south_parking.add.xml")
if os.path.exists(_WG_S_PARK):
    GUI_ADDITIONAL.insert(0, _WG_S_PARK)
# 2026-11-13: 어린이보호구역 적색 포장·노면 표시·표지·단속 카메라 (env/roundabout_school_zone.py, 제한 속도 30 km/h 는 net 에)
_SCHOOL_ZONE = os.path.join(MAP_DIR, "roundabout.school_zone.add.xml")
if os.path.exists(_SCHOOL_ZONE):
    GUI_ADDITIONAL.append(_SCHOOL_ZONE)
