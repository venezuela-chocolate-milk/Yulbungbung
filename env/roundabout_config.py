"""회전교차로 도로 설정 파일 — 검단회전교차로 (인천 검단구 왕길동) 모델.

이 파일의 값을 수정한 뒤 view_roundabout.py를 실행하면
roundabout_builder.py가 SUMO 도로 파일을 env/sumo_roundabout/ 에 다시 만든다.
(기존 직선 고속도로 환경 env/road_config.py, env/sumo/ 와는 완전히 별개)

재현 원칙:
    - 차로 수, 진입/진출 구성, 섬/안전지대 배치는 실제 도로와 동일하게
    - 원의 모양은 실제의 약간 찌그러진 형태 대신 "예쁜 정원(正圓)"으로

좌표계 (단위: m):
    - 원의 중심 = (0, 0)
    - 화면 위쪽(+y)이 실제 정북. 네이버 지도(북쪽 위)와 나란히 놓고 비교할 수 있다.
    - 방향 이름(N/S/E/W)은 편의상 붙인 이름이다: N = 왕길지하차도 쪽(실제 북북서),
      S = 그 반대편(남남동), E = 동쪽 검단로(북동), W = 서쪽 검단로(남서).
    - 각도(axis)는 수학 좌표 기준: 동=0°, 북=90°, 서=180°, 남=270°
      (반시계 방향으로 증가 = 회전교차로 안 차량의 진행 방향)
      지도 방위각(북=0°, 시계방향) b 와의 관계: axis = 90 - b
    - 각 axis 값은 네이버 지도 항공사진에서 원 근처 도로 방향을 잰 값이다.

단위 참고: 16.67 m/s ≈ 60 km/h,  8.33 m/s ≈ 30 km/h
"""

# ──────────────────────────────────────────────────────────────
# 1) 회전부 (원)
# ──────────────────────────────────────────────────────────────
RING = {
    "radius": 20.0,        # 회전부 "중심선" 반지름 (2개 차로의 한가운데).
                           # 실측(OSM) 평균 반지름 약 19.6 m → 20 m 정원으로 단순화
    "num_lanes": 2,        # 회전부 차로 수
    "speed_limit": 8.33,   # 회전부 제한속도 (≈ 30 km/h)
    "arc_step_deg": 3.0,   # 원호를 몇 도 간격의 점으로 그릴지 (작을수록 매끈)

    # 회전부 차로 사이 선:
    #   진입/진출 지점은 SUMO에서 교차 영역(junction)이 되고, 교차 영역 안에서는
    #   원래 차로 변경이 불가능하다 → 진출입 앞뒤는 이미 "실선"처럼 동작한다.
    #   그 사이 일반 구간은 점선(차로 변경 허용).
    #   solid_len > 0 이면 각 접속 노드 앞뒤 solid_len 미터를 추가로 잘라 실선으로
    #   만든다. 단, 이 길이는 노드 기준이라 대부분 교차 영역에 먹힌다.
    "solid_len": 0.0,

    # 원 위 각 지점에서 안쪽 차로 → 다음 구간 바깥 차로 연결도 추가한다.
    #   회전부 일반 구간이 아주 짧아 차로 변경할 자리가 없어서, 출구로 나가려는 안쪽
    #   차량과 안쪽으로 가려는 바깥 차량이 서로 비켜주길 기다리다 멈추는 교착이 생겼다.
    #   (0.1초 간격 5시드 1시간: 평균 대기 198초·teleport 125회 → 4.5초·0회)
    "inner_to_outer": True,
}

LANE_WIDTH = 3.2           # 차로 폭 (m) — 국내 도시부 일반 차로 폭

# 북동쪽 끝 삼거리: 사진을 참고한 초안. 횡단보도는 생성하지 않는다.
T_JUNCTION = {"distance": 350.0, "axis": 36.0, "branch_axis": 103.0,
              "main_extension": 150.0, "branch_length": 120.0,
              "turn_bay_length": 65.0, "taper_length": 45.0,
              "safety_tip_length": 16.0, "roundabout_stop_station": 325.0,
              "main_stop_shift": 8.0}

# 북동쪽 도로 끝의 두 번째 삼거리: 본선과 지선이 정확히 직각이다.
END_T_JUNCTION = {"main_extension": 205.0, "branch_length": 100.0,
                  "branch_marking_clearance": 15.0, "main_guide_clearance": 8.0,
                  "main_arrow_distance": 2.25, "turn_bay_length": 30.0,
                  "taper_length": 45.0, "junction_radius": 6.0,
                  "outgoing_stop_from_junction": 55.0}

STOP_T_JUNCTION = {"ahead_of_stop": 15.0, "branch_length": 100.0,
                   "clear_junction_markings": True, "keep_main_centerline": False,
                   "clear_white_dashes": True,
                   "clear_opposite_divider": True,
                   "main_clear_after": 12.0, "align_exit_dividers": True,
                   "keep_stop_line": True, "symmetric_side_dashes": False,
                   "side_stop_distance": 18.0}

END_CROSSROADS = {"arm_length": 100.0, "nw_arm_length": 102.0,
                  "se_arm_length": 1040.0, "se_end_widening": 100.0,
                  "se_end_taper": 40.0, "se_end_entry_taper": 14.0,
                  "sw_arm_length": 179.0,
                  "sw_forward_length": 150.0,
                  "sw_end_t_arm_length": 90.0,
                  "sw_end_t_left_length": 630.0,
                  "cross_axis": 156.0}

# 남쪽 분리섬과 북쪽 안전지대 끝에서 안쪽 차로끼리 연결하는 1차로 유턴.
UTURNS = [{"approach": "S", "island": 0, "clearance": 0.25},
          {"approach": "N", "island": 0, "clearance": 0.25, "wait_at_entry": True}]


# ──────────────────────────────────────────────────────────────
# 2) 진입로 / 진출로
#
#    approach(방향) 하나에 leg(진입로 또는 진출로) 여러 개가 붙는다.
#      axis      : 이 방향 도로가 원에서 뻗어 나가는 각도 (deg)
#      length    : 원 밖으로 만들 도로 길이 (m). 실제로는 계속 이어지지만
#                  시뮬레이션에서는 이 길이에서 자른다.
#      speed     : 진입/진출로 제한속도 (m/s)
#      legs      : 원을 반시계 방향으로 돌 때 만나는 순서대로 나열
#                    kind  : "exit"(원 → 밖) / "entry"(밖 → 원, 원 안 차량에 양보)
#                    lanes : 차로 수
#      islands   : leg 사이 구조물. legs 사이마다 하나씩 (legs 개수 - 1)
#                    type 종류 (모두 차가 다닐 수 없는 구역, 화면 표시만 다름)
#                      "sidewalk"     : 인도처럼 높인 분리섬 (회색)
#                      "island_hatch" : 화단 섬 + 둘레에 노란 빗금 안전지대
#                      "hatch"        : 바닥에 칠한 노란 빗금 안전지대만
#                    gap_ring : 원 쪽 끝에서 두 leg 사이 간격 (m)
#                    gap_far  : island_len 지점에서의 간격 (m).
#                               gap_ring보다 작으면 삼각형, 같으면 사각형 섬
#                    island_len : 섬/안전지대 길이 (원 바깥 가장자리부터, m)
#      edge_hatch: (선택) 특정 leg 바깥쪽 가장자리의 노란 안전지대
#                    leg : legs 안의 인덱스, width : 원 쪽 폭(m), len : 길이(m)
#                    (원 쪽에서 width로 시작해 len 지점에서 0이 되는 삼각형)
#
#    섬 사이 간격(gap)에 따라 leg가 원에 붙는 각도가 자동 계산된다.
# ──────────────────────────────────────────────────────────────
APPROACHES = {
    "N": {   # 왕길지하차도 쪽 (봉수대로 연결로)
        "axis": 112.0, "length": 150.0, "speed": 16.67,   # 방위 약 338° (북북서)
        "legs": [
            {"kind": "exit",  "lanes": 2},
            {"kind": "entry", "lanes": 1},
        ],
        "islands": [
            {"type": "hatch", "gap_ring": 5.0, "gap_far": 3.0, "island_len": 30.0},
        ],
    },
    "DANBONG": {   # 단봉로 — 원에서 나가기만 하는 일방통행 (서쪽 출구 A)
        # 서쪽 검단로에서 약 35° 벌어진 정서 방향.
        # 원 위 위치는 EXTRA_ISLANDS의 gap_ring에 따라 검단로 출구 옆에 자동으로 붙는다.
        "axis": 180.0, "length": 150.0, "speed": 16.67,   # 방위 약 270° (정서)
        "legs": [
            {"kind": "exit", "lanes": 2},
        ],
        "islands": [],
    },
    "W": {   # 검단로 서쪽 (서쪽 출구 B + 서쪽 입구)
        # 원 중심에서 남서쪽 도로 축을 따라 850 m까지 (시작 기준 24.2 m 제외).
        "axis": 215.0, "length": 850.0 - (RING["radius"] + RING["num_lanes"] * LANE_WIDTH / 2.0 + 1.0), "speed": 16.67,
        "legs": [
            {"kind": "exit",  "lanes": 2},
            {"kind": "entry", "lanes": 2},
        ],
        "islands": [
            # 출구 B와 입구 사이: 원 쪽 폭 약 2.5 m의 삼각형 띠, 사선 빗금만
            {"type": "hatch", "gap_ring": 3.5, "gap_far": 0.5, "island_len": 25.0,
             "pattern": "diagonal"},
        ],
    },
    "S": {   # 왕길지하차도 반대편 (봉수대로 연결로)
        "axis": 286.0, "length": 150.0, "speed": 16.67,   # 방위 약 164° (남남동)
        "legs": [
            {"kind": "exit",  "lanes": 2},
            {"kind": "entry", "lanes": 2},
        ],
        "islands": [
            # 네모난 섬 + 둘레 네모난 빗금 (간격이 일정 → 사각형)
            {"type": "island_hatch", "gap_ring": 8.0, "gap_far": 8.0, "island_len": 22.0,
             "toward_ring": 2.5},  # 표시만 원 쪽으로 이동하고 실제 차로 경계에서 잘라 맞춤
        ],
    },
    "E": {   # 검단로 동쪽 (후문 쪽)
        # 원 중심에서 북동쪽 도로 축을 따라 350 m까지.
        "axis": 36.0, "length": 350.0 - (RING["radius"] + RING["num_lanes"] * LANE_WIDTH / 2.0 + 1.0), "speed": 16.67,
        "legs": [
            {"kind": "exit",  "lanes": 2},
            {"kind": "entry", "lanes": 2},
        ],
        "islands": [
            # 세모난 섬 + 약간 바깥의 세모난 빗금
            {"type": "island_hatch", "gap_ring": 7.0, "gap_far": 1.0, "island_len": 30.0},
        ],
    },
}

# 서쪽 출구 A(단봉로)와 출구 B(검단로) 사이의 섬.
# 두 leg가 서로 다른 approach에 속하므로 따로 정의한다.
#   실제 구조: 두 출구가 원에서 거의 붙어 나오고, 그 사이 앞코(원 쪽)에
#   좁은 삼각형 V자 빗금이 있으며, 그 뒤로 모서리가 둥근 큰 화단 섬이 있다.
#   섬 둘레에는 빗금 없이 연석만 있다.  (type "island_nose")
#   gap_ring   : 원 바깥 가장자리에서 두 출구 사이 간격 (m)
#   nose_len   : 앞코 빗금 삼각형 길이 (m)
#   island_len : 앞코부터 화단 섬 끝까지 전체 길이 (m)
#   (type "split_fill") 단봉로 출구 · 대각선 길 · 검단로 출구로 둘러싸인 공간 전체를
#   노란 빗금 안전지대로 채우고, 그 위에 테두리(land_margin)만 남기고 화단(땅)을 올린다.
#   원 쪽 앞코 nose_len 구간은 빗금만.
#   gap_ring    : 원 바깥 가장자리에서 두 출구 사이 간격 (m)
#   nose_len    : 앞코 빗금 구간 길이 (m)
#   land_margin : 화단 둘레에 남길 빗금 폭 (m)
EXTRA_ISLANDS = [
    {"between": [("DANBONG", 0), ("W", 0)], "type": "split_fill",
     "gap_ring": 1.0, "nose_len": 15.0, "land_margin": 1.6, "pattern": "chevron"},
]

# 단봉로 출구 ↔ 검단로 입구 연결 구조:
#
#     ←← 나가는 2차로  ═══╗
#     ━━ 노란 중앙선 ━━━━ ╠═ 갈라지는 지점 ── ←← 단봉로 출구 (원에서 나감)
#     →→ 들어오는 2차로 ═══╝   ╲
#                               ╲ →→ 대각선 길 → 검단로 → 서쪽 입구로 원 진입
#
#   단봉로 출구는 split_dist에서 끝나고, 그 뒤로 단봉로가 양방향 4차로가 된다.
#   들어오는 2차로는 갈라지는 지점에서 대각선 길로 내려가, 검단로 출구 차로를
#   가로질러(교차점에서 양보) 검단로 입구에 합류한다.
#   split_dist / join_dist : 원 중심에서 각 도로 축을 따라 잰 거리 (m)
SPLIT_LINK = {
    "exit": ("DANBONG", 0),       # 양방향 도로가 되는 출구
    "join_exit": ("W", 0),        # 대각선 길이 가로지르는 출구 (검단로 출구 B)
    "join_entry": ("W", 1),       # 대각선 길이 합류하는 입구 (검단로 입구)
    "split_dist": 95.0,           # 곡선 진입 전 충분한 길이를 확보
    "join_dist": 72.0,            # 대각선 길이 검단로에 닿는 지점
    "diag_curve": 16.0,           # 대각선 길 곡선의 완만함 — 갈라진 뒤 원 쪽으로
                                  # 이만큼(m) 직진하듯 이어지다 부드럽게 꺾인다
    "terminal_straight": 26.0,    # 세 차로가 꺾이는 곳에 충분한 곡선 반경 확보 (m)
    "terminal_heading": -80.0,   # 교차로를 향하는 직선 방향 (deg)
    "far_length": 100.0,          # 갈라진 뒤 양방향 단봉로 길이
    "lanes_in": 2,                # 들어오는 방향 차로 수
    # 대각선 길 끝 구간: 오른쪽에 우회전 전용 차로가 하나 더 생겨 검단로의
    # 나가는 방향(남서)으로 이어진다. 기존 차로들은 검단로 입구(원 쪽)로 좌회전.
    "right_lane_len": 28.0,       # 차로가 추가되는 구간 길이 (m). 0이면 추가 차로 없음
                                  # (끝의 약 10 m는 검단로 교차점 안이라 실제로 보이는 건 ~16 m)
    "right_lane_from_upstream": True,  # 맨 바깥 차로를 단봉로 직선 구간부터 연속해서 연결
    "right_turn_lead": 3.5,      # 완만한 독립 곡선의 진입 구간 (m)
    "right_turn_merge": 8.0,     # 완만한 곡선을 위해 본선 합류 지점을 하류로 이동 (m)
    # 우회전 차로와 좌회전 차로 사이, 정지선 앞의 작은 삼각형 빗금 안전지대
    "right_lane_island": {"lane_gap": 3.2, "clearance": 0.15},
}

# ──────────────────────────────────────────────────────────────
# 노면 표시 (순수 시각 요소 — 주행 규칙에는 영향 없음)
#
# SUMO가 자동으로 그리는 화살표/정지선은 실제 노면과 달라서 GUI 설정에서 끄고,
# 실제 도로에 칠해진 표시만 여기 적어서 직접 그린다.
# ──────────────────────────────────────────────────────────────
MARKINGS = {
    "west_centerline": True,  # 검단로 서쪽의 좁은 중앙 틈 전체에 연속 노란 복선
    "east_centerline": True,  # 북동쪽 안전지대 끝에서 도로 끝까지 노란 복선
    "ring_divider": True,      # 회전부 2개 차로 사이 점선을 원 전체에 끊김 없이
    "junction_guides": ["W_out0"],  # 회전부에서 검단로 출구까지 차로 경계 연장
    "omit_dashes": ["lane_dash_W_out0_0_0"],  # 원형 점선과 겹치는 출구 첫 조각 제외
    "clear_crossing": {"source": "DANBONG_diag_end", "target": "W_in1", "margin": 0.5},
    "crossing_centerline_extension": 2.4,  # 원 쪽 중앙선 끝만 교차부 안으로 연장
    "dash": 3.0, "gap": 3.0,   # 목표 점선 길이/간격. 원형 이음까지 같도록 전체에 동일 미세 보정
    "yield": True,             # 모든 진입로 끝에 양보선 + 양보 표시(▽)
    "hatch_pattern": "chevron",  # 노란 안전지대 무늬: "chevron"(V자) / "diagonal"(사선)
}

# 화살표: 실제 도로에 칠해진 것만 나열한다.
#   approach / leg : APPROACHES의 방향 키와 legs 인덱스
#   lane           : 차로 번호 (0 = 맨 오른쪽 차로)
#   type           : "straight"(직진) / "left"(좌회전) / "right"(우회전)
#                    "ccw"(회전교차로 진입 — 오른쪽으로 꺾여 반시계 방향으로 도는 화살표)
#   dist           : 진입로는 차로 끝(원 쪽)에서 거꾸로, 진출로는 차로 시작
#                    (원에서 나온 지점)부터 잰 거리 (m)
ARROWS = [
    {"approach": "N", "leg": 1, "lane": 0, "type": "ccw", "dist": 10.0},
    # 서쪽 출구 B(검단로) 시작 부분 직진 화살표 — 두 차로 모두
    {"approach": "W", "leg": 0, "lane": 0, "type": "straight", "dist": 6.0},
    {"approach": "W", "leg": 0, "lane": 1, "type": "straight", "dist": 6.0},
    # 대각선 길 끝(검단로 교차점 직전, 3차로 구간).
    # leg가 아닌 길은 "edge"로 엣지 id를 직접 쓰고, dist는 차로 끝에서 거꾸로 잰다.
    #   0번(맨 오른쪽, 추가 차로) = 우회전 / 1·2번 = 좌회전
    {"edge": "DANBONG_diag_end", "lane": 0, "type": "right", "dist": 10.0},
    {"edge": "DANBONG_diag_end", "lane": 1, "type": "left", "dist": 8.6},
    {"edge": "DANBONG_diag_end", "lane": 2, "type": "left", "dist": 8.6},
]

# 흰색 정지선(실선): 엣지 끝(교차점 직전)에 차로를 가로질러 그린다.
#   문자열이면 모든 차로, lanes로 대상 차로, dist로 끝에서 거꾸로 잰 위치 지정
STOP_LINES = [
    {"edge": "DANBONG_diag_end", "lanes": [1, 2], "dist": 6.1},
    {"edge": "DANBONG_diag_end", "lanes": [0], "dist": 7.5},
]

# ──────────────────────────────────────────────────────────────
# 3) 확인용 배경 교통 (도로 연결 검증용 — 학습 설계는 아직 미정)
#    모든 진입로 → 모든 진출로 경로에 차량을 흘린다. 일부는 유턴(주황색 차량).
# ──────────────────────────────────────────────────────────────
TRAFFIC = {
    "vehs_per_hour_per_entry": 350,   # 진입로 하나당 시간당 투입 대수
                                      # (450: 붐비지만 소통됨, 550 이상: 수 분 단위 정체)
    "max_speed": 16.67,
    # 유턴 비율: 섬 뒤 유턴 연결(UTURNS, 남쪽과 북쪽)이 있는 방향에서 원에 들어가기
    #   전에 그 유턴으로 돌아 나가는 차량 몫 (주황색 차량). 다른 방향은 유턴 없음.
    "uturn_share": 0.3,
    # 출발 방식: "random" = 매초 확률로 출발 (실제처럼 들쭉날쭉).
    #   "uniform"은 모든 흐름이 같은 박자로 동시에 출발해 차가 한꺼번에 몰린다.
    "departure": "random",
}

# 도로 바깥 바닥(잔디) 표시 여부 — 순수 시각 요소
SCENERY = {
    "ground": True,
}
