"""SUMO 고속도로 RL 환경 — Gymnasium 인터페이스 + TraCI.

┌─────────────────────────────────────────────────────────────┐
│  강화학습 루프에서 이 클래스의 역할                              │
│                                                             │
│    obs, _ = env.reset()          # 에피소드 시작, 초기 관측     │
│    obs, r, term, trunc, _ = env.step(action)  # 1스텝 진행    │
│                                                             │
│  내부적으로는 TraCI(Traffic Control Interface)라는 소켓 API로  │
│  SUMO 시뮬레이터를 원격 조종한다:                               │
│    traci.simulationStep()  → 시뮬레이션을 1스텝 진행            │
│    traci.vehicle.setSpeed() → 차량 속도 명령                   │
│    traci.vehicle.getSpeed() 등 → 상태 조회                     │
└─────────────────────────────────────────────────────────────┘

관측/행동/보상의 구체적인 정의는 configs/mdp_config.py 에 있고,
이 파일은 그 설정을 받아서 실행만 한다. (MDP를 바꿀 때 이 파일 수정 불필요)
"""
import os
import sys

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# SUMO 설치 폴더의 tools/ 안에 traci, sumolib 파이썬 모듈이 들어있다.
# pip로도 설치했지만, 버전 불일치를 피하려면 SUMO_HOME 쪽을 우선 경로에 추가.
#
# SUMO_HOME이 설정 안 돼 있으면 OS별 기본 설치 경로를 순서대로 탐색해서
# 자동으로 잡아준다 (환경변수 설정을 깜빡해도 동작하도록).
if "SUMO_HOME" not in os.environ:
    _candidates = [
        # macOS (Homebrew) — Apple Silicon / Intel
        "/opt/homebrew/opt/sumo/share/sumo",
        "/usr/local/opt/sumo/share/sumo",
        # Linux (apt)
        "/usr/share/sumo",
        # Windows (기본 인스톨러 경로)
        r"C:\Program Files (x86)\Eclipse\Sumo",
        r"C:\Program Files\Eclipse\Sumo",
    ]
    # pip install eclipse-sumo 로 설치한 경우 (가상환경 안의 sumo 패키지)
    try:
        import sumo as _sumo_pkg
        _candidates.insert(0, getattr(_sumo_pkg, "SUMO_HOME",
                                      os.path.dirname(_sumo_pkg.__file__)))
    except ImportError:
        pass
    for _p in _candidates:
        if os.path.isdir(_p):
            os.environ["SUMO_HOME"] = _p
            print(f"[sumo_env] SUMO_HOME 자동 감지: {_p}")
            break
    else:
        raise RuntimeError(
            "SUMO_HOME 환경변수가 없고 기본 경로에서도 SUMO를 찾지 못했습니다.\n"
            "  Windows: 인스톨러 재설치(SUMO_HOME 체크) 또는 환경변수 수동 설정\n"
            "  macOS  : ./setup_mac.sh 실행 또는 README_MAC.md 참고\n"
            "  Linux  : sudo apt install sumo sumo-tools 후 "
            "export SUMO_HOME=/usr/share/sumo")

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import traci
import traci.constants  # noqa: F401  (libsumo 로 바꿔도 상수는 그대로 쓰도록 먼저 불러둠)

# 병렬 학습 프로세스는 libsumo 를 쓴다 (환경변수 SUMO_LIBSUMO=1, env/maps.make_vec_env 가 설정).
# libsumo 는 SUMO 를 파이썬 안에서 직접 돌려 TraCI 소켓 통신이 없다 → 스텝당 수십~수백 번의
# 조회가 훨씬 빠르다. 화면(sumo-gui)은 지원하지 않으므로 GUI 실행은 항상 traci 를 쓴다.
# 다른 모듈의 "import traci" 도 같은 것을 쓰도록 sys.modules 를 바꿔 둔다.
if os.environ.get("SUMO_LIBSUMO") == "1":
    import libsumo
    sys.modules["traci"] = libsumo
    traci = libsumo
from sumolib import checkBinary

PARK_IN_S = 6.0     # 엄마 경로: 통로에서 빈 칸으로 들어가는(나오는) 시간 (s). 2026-11-17: 4 → 6 (덜 급하게)
PARK_GUI_HOLD_S = 0.0   # 화면에서만 주차 장면을 따로 늦추는 시간 (0 = 안 늦춤: 다른 장면과 같은 5배속. 머무는 시간은 mom PARK_DWELL_S)


class SumoHighwayEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, cfg_path, road, ego, mdp_sim, mdp_obs, action, reward,
                 traffic=None, gui=False, gui_autostart=True,
                 route=None, sumo_args=None, gui_additional=None, signal_heads=None,
                 urban=None, net_path=None, curriculum=None, fast_reset=None,
                 curriculum_step_multiplier=1, random_start=None,
                 illegal_parking=None, mom=None):
        """
        Args:
            cfg_path: road_builder.build()가 만든 .sumocfg 경로
            road:     configs.road_config.ROAD
            ego:      configs.road_config.EGO (가속/감속 한계가 여기서 옴)
            mdp_sim:  configs.mdp_config.SIMULATION
            mdp_obs:  configs.mdp_config.OBSERVATION
            action:   configs.mdp_config.ACTION (행동 차원 수)
            reward:   configs.mdp_config.REWARD
            traffic:  configs.road_config.TRAFFIC (프리필/컨트롤러 정보).
                None이면 프리필 없이 flow만으로 동작 (하위호환)
            gui:      True면 sumo-gui(화면 있는 버전)로 실행
            gui_autostart: (gui=True일 때만 의미)
                True  → 창이 뜨면 시뮬레이션이 자동으로 시작
                False → 사용자가 ▶(플레이) 버튼을 누를 때까지 대기.
                        sumo-gui는 --start 옵션이 없으면 TraCI의
                        simulationStep() 호출도 플레이 전까지 블록되므로,
                        에피소드 전체가 "▶를 눌러야 시작"되는 효과가 난다.
            route:    None이면 rou.xml의 경로 "r0"(직선도로)을 쓴다.
                {"from": edge, "to": edge, "via": [edge...]} 를 주면
                매 에피소드 SUMO 최단경로로 ego 경로를 만든다 (회전교차로 맵).
                이런 dict의 리스트를 주면 에피소드마다 "weight" 비율로 하나를 고른다.
                reset(options={"route": 이름 또는 번호})로 경로를 지정할 수도 있다.
            sumo_args: SUMO 실행 옵션 추가 (예: 교차로 충돌 판정)
            gui_additional: GUI일 때만 불러올 그림 파일들 (노면 표시, 신호등 그림)
            signal_heads: 신호등 그림 파일. GUI일 때 매 스텝 실제 신호에 맞춰 등을 켠다
            urban:    도심 경로 주행 설정 (env/urban_driving.py). 주면 관측 추가 + 안전 마스킹
                      + 진행 거리 보상을 쓴다. net_path 가 함께 필요하다.
            curriculum: [{"until_step": N 또는 None, "scale": 배율, "route_weights": {...}}, ...]
                      누적 스텝 수에 따라 교통량 배율/경로 비율을 단계적으로 바꾼다 (학습용)
            curriculum_step_multiplier: 병렬 환경 수. 이 환경의 스텝 × 이 값 = 전체 누적 스텝 추정
            fast_reset: {"enabled", "pool_size", "spacing_steps", "extra_steps_max"}
                      (화면 없을 때만) SUMO 를 에피소드마다 다시 켜지 않고, 배경차를 채운
                      준비 상태를 여러 개 저장해 두었다가 불러온다. SUMO 켜기 7초 → 0.05초
        """
        super().__init__()
        self.cfg_path = cfg_path
        self.gui = gui
        self.gui_autostart = gui_autostart
        self.routes = ([route] if isinstance(route, dict) else list(route)) if route else []
        self.route = None              # 이번 에피소드 경로 (reset에서 결정)
        self.route_name = None
        self.sumo_args = list(sumo_args or [])
        # 불법 주정차 구간 (회전교차로 맵, env.roundabout_illegal_parking 모듈): reset 때마다 무작위로 다시 뿌린다
        self.illegal_parking = illegal_parking
        self.gui_additional = list(gui_additional or [])
        self._heads = None
        if gui and signal_heads:
            from env.roundabout_signal_heads import SignalHeads
            self._heads = SignalHeads(signal_heads)
        self.ego_route_id = "ego_route" if self.routes else "r0"
        self.ego_cfg = dict(ego)
        # SUMO 안전장치/자율 차선변경 모드 (기본 0 = 모두 끔, 기존 고속도로 동작)
        self.ego_speed_mode = int(ego.get("speed_mode", 0))
        self.ego_lane_change_mode = int(ego.get("lane_change_mode", 0))

        # ----- 도로/차량 설정에서 필요한 값들 -----
        self.vmax = float(road["speed_limit"])   # 속도 정규화 기준
        self.num_lanes = int(road["num_lanes"])
        # 시작 차선: "center"면 중앙 차선(차선수//2)으로 자동 결정.
        #   3차선 → 1번(가운데), 5차선 → 2번. 차선 수를 바꿔도 항상 가운데라
        #   좌/우 양쪽으로 변경 여지가 대칭으로 생긴다 (탐험에 유리).
        _dl = ego["depart_lane"]
        if str(_dl).lower() in ("best", "free", "random", "allowed", "first"):
            self.ego_depart_lane = str(_dl).lower()
        elif str(_dl).lower() == "center":
            self.ego_depart_lane = str(int(road["num_lanes"]) // 2)
        else:
            self.ego_depart_lane = str(_dl)
        # ---- 프리필(도로 전체 사전 배치) 설정 ----
        self.traffic = traffic or {}
        # 회전교차로 맵: 여러 route 파일 + 교통량 배율(SUMO --scale)
        self.route_files = list(self.traffic.get("route_files") or [])
        self.traffic_scale = self.traffic.get("scale")
        pf = (self.traffic.get("prefill") or {})
        self.prefill_enabled = bool(pf.get("enabled", False))
        self.prefill_start = float(pf.get("start_offset", 80.0))
        self.prefill_end_margin = float(pf.get("end_margin", 50.0))
        self.prefill_jitter = float(pf.get("spacing_jitter", 0.3))
        # ---- 차선변경 안전 게이트 설정 (mdp_config.ACTION["lane_change_safety"]) ----
        lcs = (action.get("lane_change_safety") or {})
        self.lc_safety_enabled = bool(lcs.get("enabled", True))
        self.lc_front_min = float(lcs.get("min_front_gap", 5.0))   # m
        self.lc_rear_min = float(lcs.get("min_rear_gap", 5.0))     # m
        self.lc_front_tau = float(lcs.get("front_tau", 0.3))       # s
        self.lc_rear_tau = float(lcs.get("rear_tau", 0.5))         # s
        self.lc_spawn_protect = float(lcs.get("spawn_protect", 60.0))  # m
        self.lc_horizon = float(lcs.get("approach_horizon", 1.0))       # s
        self.road_length = float(road["length"])
        self.traffic_max_speed = float(self.traffic.get("max_speed", self.vmax))
        # 차선당 배치 간격: 지정값이 없으면 교통류 이론대로 자동 계산
        #   차선당 유량 q_lane = (vehs/h) / 3600 / 차선수  [대/초]
        #   간격 = 속도 / q_lane   (그 속도로 흐를 때 차 사이 평균 거리)
        if pf.get("spacing"):
            self.prefill_spacing = float(pf["spacing"])
        else:
            vph = float(self.traffic.get("vehs_per_hour", 1800))
            q_lane = max(vph / 3600.0 / max(self.num_lanes, 1), 1e-6)
            self.prefill_spacing = self.traffic_max_speed / q_lane
        # 컨트롤러 타입 샘플링 준비 (rou.xml의 car_<이름> vType과 대응)
        ctrls = self.traffic.get("controllers") or {}
        if ctrls:
            names = list(ctrls.keys())
            probs = np.array([float(ctrls[n].get("probability", 1.0)) for n in names])
            self._prefill_types = [f"car_{n}" for n in names]
            self._prefill_probs = probs / probs.sum()
        else:
            self._prefill_types, self._prefill_probs = ["car"], np.array([1.0])
        self.ego_depart_speed = str(ego["depart_speed"])
        # 행동 a ∈ [-1,1] 을 실제 가속도로 변환할 때 쓰는 스케일:
        #   a = +1 → +max_accel (풀 액셀),  a = -1 → -max_decel (풀 브레이크)
        self.max_accel = float(ego["accel"])   # m/s^2
        self.max_decel = float(ego["decel"])   # m/s^2 (양수로 저장, 부호는 계산 시)

        # ----- MDP 설정 -----
        self.step_length = float(mdp_sim["step_length"])
        self.max_steps = int(mdp_sim["max_steps"])
        self.warmup_steps = int(mdp_sim["warmup_steps"])
        self.visibility = float(mdp_obs["visibility"])   # 관측 거리 W
        # 밀도 정규화용 "차량 1대가 점유하는 길이" (차체 + 최소 간격)
        self.density_veh_len = float(mdp_obs.get("density_veh_length", 7.5))
        self.act_dim = int(action["dim"])
        self.accel_index = int(action.get("accel_index", 0))
        self.lane_change_index = int(action.get("lane_change_index", 1))
        self.lane_change_threshold = float(action.get("lane_change_threshold", 0.5))
        self.lane_change_duration = float(action.get("lane_change_duration", 1.0))
        self.rw = reward

        # GUI는 ego를 따라가며 가까이 보이도록 기본값을 둔다.
        self.gui_zoom = float(mdp_sim.get("gui_zoom", 1200.0))
        self.gui_delay = float(mdp_sim.get("gui_delay", 100.0))
        self.gui_track_ego = bool(mdp_sim.get("gui_track_ego", True))

        self.ego_id = "ego"        # SUMO 안에서 ego 차량을 식별하는 ID
        self.total_steps = 0       # 이 환경이 진행한 누적 스텝 (커리큘럼 기준)
        self._finished_stats = []  # 끝난 에피소드의 개입/결과 요약 (학습 로그용, drain_episode_stats)
        self.curriculum = list(curriculum or [])
        self.curriculum_step_multiplier = int(curriculum_step_multiplier)
        # 학습용: 일부 에피소드는 경로 중간의 무작위 도로에서 출발 (평가는 항상 처음부터)
        self.random_start = dict(random_start or {})
        self._this_depart_lane = None
        fr = fast_reset or {}
        self.fast_reset = bool(fr.get("enabled", False)) and not gui
        self.fast_pool_size = int(fr.get("pool_size", 16))
        self.fast_spacing = int(fr.get("spacing_steps", 60))
        self.fast_extra_max = int(fr.get("extra_steps_max", 20))
        self._state_dir = None
        self._state_pool = []
        self._pool_scale = None
        self._episode_count = 0
        self.curriculum_stage = None
        self.step_count = 0
        self._started = False      # SUMO 프로세스가 떠 있는지 여부

        # Gymnasium 행동 공간:
        #   action[0] = 종방향 가감속 raw ∈ [-1,1]
        #   action[1] = 차선변경 raw ∈ [-1,1], 환경 내부에서 {-1,0,+1}로 양자화
        # PPO의 Gaussian 정책과 호환하기 위해 Box를 유지한다. 실제 적용된
        # 차선변경 명령은 info["lane_change"]에 정확한 -1/0/+1로 반환한다.
        self.action_space = spaces.Box(low=-1.0, high=1.0,
                                       shape=(self.act_dim,), dtype=np.float32)
        # 관측: 19차원, 범위 [-1,1]. 레이아웃은 env/mdp_config.py 주석 참고.
        #   [0] 내 속도, [1..12] 3개 차선 × (선행/후행 상대거리·상대속도),
        #   [13..15] 3개 차선의 연결성(끊김 정도),
        #   시야 차선: 좌2, 좌1, 현재, 우1, 우2 (5개) — 3차선 도로에서
        #   맨 왼쪽에 있어도 오프셋 -2로 맨 오른쪽 차선까지 보인다.
        self.obs_offsets = (+2, +1, 0, -1, -2)
        n_lanes_view = len(self.obs_offsets)
        # 2026-11-17 (컨설팅 반영 A안): 도심 맵에서는 수업 원본의 "차로별 앞뒤차" 20칸을 쓰지 않는다.
        #   차로 기준으로 SUMO 에 앞차·뒤차를 물어 가려진 차까지 보이고(현실 센서로는 불가능), 다른 도로(회전교차로
        #   안·옆길·교차로 안)의 차는 못 보는 고속도로용이라, 가려짐을 반영한 주변 차량 관측(urban_driving)과 겹친다.
        #   내 속도·차로 연결성(끊김)·차로 밀도 11칸은 그대로. 직선 고속도로 맵(urban 없음)은 원본 그대로.
        self.obs_lane_neighbors = not bool(urban and urban.get("drop_lane_neighbors", False))
        self.obs_dim = 1 + (n_lanes_view * 4 if self.obs_lane_neighbors else 0) + n_lanes_view + n_lanes_view
        # 2026-11-19: 원본 20칸을 뺀 도심 맵에서 "현재 차로 앞차" 2칸만 되살림 (roundabout_env_config URBAN ego_leader)
        self.obs_ego_leader = bool(urban and not self.obs_lane_neighbors and urban.get("ego_leader", False))
        self.obs_dim += 2 if self.obs_ego_leader else 0
        self.urban = None
        if urban:
            from env.urban_driving import UrbanDriving, EXTRA_DIM
            self.urban = UrbanDriving(self, urban, net_path)
            self.obs_dim += EXTRA_DIM
        # 엄마 심부름 경로 과제 (주차장 정차·긴급차 양보·단속 카메라, env/mom_driving.py): 경로에 stops 가 있을 때
        self.mom = None
        if self.urban and any(r.get("stops") for r in self.routes):
            from env.mom_driving import MomTask, MOM_DIM
            self.mom = MomTask(self, mom or {}, self.urban.net, os.path.dirname(net_path))
            self.obs_dim += MOM_DIM
        self._this_depart_pos = "0"
        self._this_depart_speed = None
        self._route_offset = 0
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(self.obs_dim,),
                                            dtype=np.float32)

    # ==================================================================
    # SUMO 프로세스 시작
    # ==================================================================
    def _start_sumo(self):
        # gui=True면 화면이 있는 sumo-gui, 아니면 화면 없는 sumo (학습용, 훨씬 빠름)
        binary = checkBinary("sumo-gui" if self.gui else "sumo")
        cmd = [
            binary, "-c", self.cfg_path,
            "--step-length", str(self.step_length),
            # 충돌한 차량을 시뮬레이션에서 제거. (기본값 teleport는 순간이동시켜버려서
            # RL 학습에는 부적합. remove로 해야 "충돌 → 에피소드 종료"가 깔끔함)
            "--collision.action", "remove",
            "--no-warnings", "true",   # 콘솔 경고 끄기
            "--no-step-log", "true",   # 스텝 로그 끄기
            # 매 에피소드 다른 시드 → 배경 교통 패턴이 달라져서 과적합 방지
            "--seed", str(np.random.randint(0, 100000)),
        ]
        cmd += self.sumo_args
        if self.route_files:
            cmd += ["--route-files", ",".join(self.route_files)]
        if self.traffic_scale is not None:
            cmd += ["--scale", str(self.traffic_scale)]
        if self.gui:
            if self.gui_additional:
                cmd += ["--additional-files", ",".join(self.gui_additional)]
            cmd += ["--quit-on-end"]      # 에피소드 종료 시 창 정리
            cmd += ["--delay", str(self.gui_delay)]
            if self.gui_autostart:
                cmd += ["--start"]        # 자동 재생
            # --start가 없으면: 창이 뜬 뒤 ▶(플레이)를 눌러야 진행된다.
            #   매 에피소드가 reset()에서 SUMO를 새로 띄우므로,
            #   에피소드마다 ▶를 눌러 시작하게 된다.
        traci.start(cmd)   # SUMO를 자식 프로세스로 띄우고 소켓 연결
        self._started = True
        if self._heads:
            self._heads.reset()

    # ==================================================================
    # 에피소드 초기화
    # ==================================================================
    def _prefill_traffic(self):
        """도로 전 구간에 배경차를 일정 간격(±jitter)으로 미리 배치한다.

        flow는 도로 입구에서만 차를 만들기 때문에, 프리필이 없으면
        에피소드 초반의 긴 도로가 텅 비어 "빈 도로 주행"만 잔뜩 학습된다.
        프리필은 첫 스텝부터 ego가 실제 교통류 한가운데서 시작하게 만든다.

        - 위치: start_offset ~ (길이 - end_margin), 차선별 독립 배치
        - 간격: prefill_spacing × (1 ± jitter) 무작위
        - 타입: controllers 확률 비율로 샘플링 (rou.xml의 vType과 동일)
        - 속도: departSpeed="max" → 각 개체가 "자기 허용 최고속도"로 시작.
          컨트롤러별 max_speed/speedDev 차등이 프리필에도 그대로 반영되고,
          개체별 무작위 speedFactor 때문에 숫자 속도를 주면 SUMO가
          "허용속도 초과"로 거부할 수 있는 문제도 함께 피한다.
        배치된 차들은 SUMO의 각자 컨트롤러(IDM 등)가 정상적으로 운전한다.
        """
        rng = np.random.default_rng(np.random.randint(2**31 - 1))   # 11-23: 판별 고정 seed 를 따르게 (강건성 평가 짝 비교)
        count = 0
        # 차선별 간격 배율 (없으면 전 차선 동일). 인덱스 0 = 맨 오른쪽.
        factors = (self.traffic.get("prefill", {})
                   .get("lane_spacing_factor")
                   or [1.0] * self.num_lanes)
        for lane in range(self.num_lanes):
            f = float(factors[lane]) if lane < len(factors) else 1.0
            lane_spacing = self.prefill_spacing * f
            pos = self.prefill_start + rng.uniform(0, lane_spacing)
            while pos < self.road_length - self.prefill_end_margin:
                vid = f"pre_{lane}_{count}"
                vtype = str(rng.choice(self._prefill_types, p=self._prefill_probs))
                try:
                    traci.vehicle.add(
                        vid, "r0", typeID=vtype,
                        departLane=str(lane),
                        departPos=f"{pos:.1f}",
                        departSpeed="max")
                    count += 1
                except traci.TraCIException:
                    pass  # 겹침 등으로 실패한 자리는 건너뜀
                step = lane_spacing * rng.uniform(
                    1.0 - self.prefill_jitter, 1.0 + self.prefill_jitter)
                pos += max(step, 10.0)
        return count

    def _update_heads(self):
        """GUI 신호등 그림을 실제 신호 상태에 맞춘다 (화면 표시 전용). 불법 주정차 차 비상등도 여기서 켠다."""
        if self._heads:
            self._heads.update()
        if self.illegal_parking:
            self.illegal_parking.tick(traci)

    def _spawn_illegal_parking(self):
        if self.illegal_parking:
            self.illegal_parking.clear()
            self.illegal_parking.spawn(traci)

    def drain_episode_stats(self):
        """지금까지 끝난 에피소드 요약을 꺼내고 비운다 (학습 로그용)."""
        out, self._finished_stats = self._finished_stats, []
        return out

    def set_mask(self, enabled: bool):
        """안전 마스킹 켜기/끄기 (도심 경로 주행일 때만 의미 있음)."""
        if self.urban:
            self.urban.mask_enabled = bool(enabled)

    def _apply_curriculum(self):
        """누적 스텝에 맞는 커리큘럼 단계의 교통량 배율/경로 비율을 적용한다."""
        if not self.curriculum:
            return
        steps = self.total_steps * self.curriculum_step_multiplier
        for k, st in enumerate(self.curriculum):
            if st.get("until_step") is None or steps < st["until_step"]:
                break
        if k != self.curriculum_stage:
            self.curriculum_stage = k
            print(f"[curriculum] 단계 {k}: 누적 약 {steps} 스텝, 교통량 배율 "
                  f"{st.get('scale', self.traffic_scale)}, 경로 비율 {st.get('route_weights', '기본')}")
        if "scale" in st:
            self.traffic_scale = st["scale"]
        self._route_weights = st.get("route_weights")

    def _load_state(self, path):
        """준비 상태 불러오기. 주차장(parkingArea)에 서 있는 차가 남아 있으면 SUMO 1.27 의 loadState 가
        죽는다(연결 끊김, 2026-11-14 확인) → 지금 있는 차를 모두 지운 뒤 불러온다."""
        for v in traci.vehicle.getIDList():
            try:
                traci.vehicle.remove(v)
            except traci.TraCIException:
                pass
        traci.simulation.loadState(path)
        self._spawn_illegal_parking()

    def _build_state_pool(self):
        """배경차를 채운 뒤(warmup) spacing_steps 간격으로 준비 상태 pool_size 개를 저장한다."""
        import shutil
        import tempfile
        if self._state_dir:
            shutil.rmtree(self._state_dir, ignore_errors=True)
        self._state_dir = tempfile.mkdtemp(prefix="sumo_states_")
        for _ in range(self.warmup_steps):
            traci.simulationStep()
        self._state_pool = []
        for k in range(self.fast_pool_size):
            path = os.path.join(self._state_dir, f"state_{k}.xml.gz")
            traci.simulation.saveState(path)
            self._state_pool.append(path)
            for _ in range(self.fast_spacing):
                traci.simulationStep()
        self._pool_scale = self.traffic_scale

    def _pick_route(self, options=None):
        want = (options or {}).get("route")
        if want is None:
            rw = getattr(self, "_route_weights", None) or {}
            w = np.array([float(rw.get(r.get("name"), r.get("weight", 1.0))) for r in self.routes])
            idx = int(np.random.choice(len(self.routes), p=w / w.sum()))
        elif isinstance(want, int):
            idx = want
        else:
            idx = next(i for i, r in enumerate(self.routes) if r.get("name") == want)
        self.route = self.routes[idx]
        self.route_name = self.route.get("name", str(idx))

    def _prepare_route(self):
        """ego 경로 등록. route 에 "edges" 가 있으면 그대로(엄마 경로, env/mom_route.py),
        없으면 from → via → to 를 SUMO 최단경로로 잇는다."""
        if self.route.get("edges"):
            edges = list(self.route["edges"])
        else:
            stops = [self.route["from"], *self.route.get("via", []), self.route["to"]]
            edges = []
            for a, b in zip(stops, stops[1:]):
                r = traci.simulation.findRoute(a, b)
                if not r.edges:
                    raise RuntimeError(f"ego 경로를 찾을 수 없습니다: {a} → {b}")
                edges += list(r.edges) if not edges else list(r.edges)[1:]
        # 출발 차로·위치·속도 (경로 설정에 있으면 그것: 엄마 경로는 출발 주차장 통로에서 멈춘 채 출발)
        self._this_depart_lane = self.route.get("depart_lane", self.ego_depart_lane)
        self._this_depart_pos = str(self.route.get("depart_pos", "0"))
        self._this_depart_speed = self.route.get("depart_speed")
        self._route_offset = 0
        # 무작위 출발 (학습 전용, 기본 꺼짐): 경로 중간의 충분히 긴 도로(교차로·회전교차로·주차장 안 제외)에서 출발.
        rs = self.random_start
        if rs and np.random.rand() < float(rs.get("prob", 0.0)):
            from env.mom_driving import LOT_PREFIXES
            min_len = float(rs.get("min_edge_len", 50.0))
            keep_end = int(rs.get("keep_last_edges", 2))
            cand = [i for i, e in enumerate(edges[:-keep_end]) if i > 0 and not e.startswith(("ring",) + LOT_PREFIXES)
                    and traci.lane.getLength(f"{e}_0") >= min_len]
            if cand:
                i = int(np.random.choice(cand))
                edges = edges[i:]
                self._route_offset = i
                self._this_depart_lane = "best"
                self._this_depart_pos = "0"
                self._this_depart_speed = None
        self._register_route(edges)

    def _register_route(self, edges):
        # 준비 상태를 불러와 재사용하면 이전에 등록한 경로가 남아 있을 수 있어 매번 새 이름
        self._route_counter = getattr(self, "_route_counter", 0) + 1
        self.ego_route_id = f"ego_route_{self._route_counter}"
        traci.route.add(self.ego_route_id, edges)
        self.ego_route_edges = edges
        self.ego_depart_edge = edges[0]

    @staticmethod
    def _car_polys(x, y, a, L, W, unit):
        """주차 동작 그림: 실제 ego 와 비슷한 차 모양 (빨간 차체 + 어두운 앞·뒤 유리). (x, y) = 앞범퍼, a = 방향."""
        ux, uy = unit(a)
        lx, ly = -uy, ux                                   # 왼쪽

        def pt(f, l):                                      # 앞범퍼에서 뒤로 f m, 왼쪽으로 l m
            return (x - ux * f + lx * l, y - uy * f + ly * l)
        h = W / 2
        body = [pt(0.0, h * 0.7), pt(0.35, h), pt(L - 0.3, h), pt(L, h * 0.8),
                pt(L, -h * 0.8), pt(L - 0.3, -h), pt(0.35, -h), pt(0.0, -h * 0.7)]
        ws = [pt(1.2, h * 0.8), pt(1.9, h * 0.85), pt(1.9, -h * 0.85), pt(1.2, -h * 0.8)]
        rw = [pt(L - 1.3, h * 0.8), pt(L - 0.8, h * 0.75), pt(L - 0.8, -h * 0.75), pt(L - 1.3, -h * 0.8)]
        return [("ego_park_anim", body, (255, 0, 0, 255)),
                ("ego_park_anim_ws", ws, (40, 40, 50, 255)),
                ("ego_park_anim_rw", rw, (40, 40, 50, 255))]

    def _park_ego(self, space, final):
        """후면 주차 (2026-11-17, 사용자 요청 (b)): 정책이 빈 칸 옆에 스스로 멈춘 뒤, 환경은 이 동작만 한다.
        space = (통로 위 위치, 칸 x, 칸 y, 칸 각도, 칸 길이).
          1) 칸을 지나 통로를 따라 조금 앞으로 (PARK_IN_S/2 초)
          2) 후진하며 곡선(3차 베지어)으로 칸에 들어가기 (PARK_IN_S 초) → 차 앞이 통로 쪽을 보고 선다
          3) 머물기 mom.dwell 초 (마지막 곳은 들어가면 끝)
          4) 앞으로 나와 ego 가 서 있는 자리로 (PARK_IN_S 초) → 정책이 다시 운전
        실제 ego 는 그동안 통로의 그 자리에 서 있고(경로·위치를 바꾸지 않음 — 칸으로 옮기면 SUMO 가 옆 통로로 잡아
        경로가 바뀌는 문제), 화면(GUI)에서는 ego 를 잠깐 숨기고 같은 모양의 차 그림이 위 동작을 한다.
        그림은 ego 가 서 있는 자리에서 끝나므로 ego 는 끝까지 옮기지 않는다 (옮기면 그 자리 차와 겹쳐 충돌 판정).
        이 동안 정책 행동은 없고 시뮬레이션만 돈다. ego 가 사라지면 True."""
        import math as _m
        import sumolib
        ego = self.ego_id
        dt = self.step_length
        _, sx, sy, sang, slen = space
        lane = traci.vehicle.getLaneID(ego)
        L = traci.vehicle.getLength(ego)
        W = traci.vehicle.getWidth(ego)
        shape = traci.lane.getShape(lane)
        llen = traci.lane.getLength(lane)
        pos0 = traci.vehicle.getLanePosition(ego)

        def unit(a):
            r = _m.radians(a)
            return _m.sin(r), _m.cos(r)

        def lane_pt(p):
            p = min(max(p, 0.1), llen - 0.1)
            x, y = sumolib.geomhelper.positionAtShapeOffset(shape, p)
            x2, y2 = sumolib.geomhelper.positionAtShapeOffset(shape, min(p + 0.5, llen))
            x1, y1 = sumolib.geomhelper.positionAtShapeOffset(shape, max(p - 0.5, 0.0))
            return x, y, _m.degrees(_m.atan2(x2 - x1, y2 - y1)) % 360.0

        # 칸 안쪽 방향 d_in (통로 → 칸). SUMO 칸 좌표 = 주차된 차의 앞범퍼, 칸 각도 = 주차된 차 방향.
        fx, fy = unit(sang)
        x0, y0 = traci.vehicle.getPosition(ego)
        if fx * (sx - x0) + fy * (sy - y0) >= 0:
            deep, din = (sx, sy), (fx, fy)
        else:
            din = (-fx, -fy)
            deep = (sx + din[0] * L, sy + din[1] * L)
        head_out = _m.degrees(_m.atan2(-din[0], -din[1])) % 360.0          # 주차 완료: 차 앞이 통로 쪽
        front_park = (deep[0] - din[0] * L, deep[1] - din[1] * L)
        pA = max(min(space[0] + L + 2.0, llen - 1.0), min(pos0, llen - 0.5))
        xA, yA, hA = lane_pt(pA)
        hx, hy = unit(hA)

        def bez(P, t):
            u = 1 - t
            x = u**3*P[0][0] + 3*u*u*t*P[1][0] + 3*u*t*t*P[2][0] + t**3*P[3][0]
            y = u**3*P[0][1] + 3*u*u*t*P[1][1] + 3*u*t*t*P[2][1] + t**3*P[3][1]
            dx = 3*u*u*(P[1][0]-P[0][0]) + 6*u*t*(P[2][0]-P[1][0]) + 3*t*t*(P[3][0]-P[2][0])
            dy = 3*u*u*(P[1][1]-P[0][1]) + 6*u*t*(P[2][1]-P[1][1]) + 3*t*t*(P[3][1]-P[2][1])
            return x, y, _m.degrees(_m.atan2(dx, dy)) % 360.0

        # 동작 순서 (앞범퍼 x, y, 차 방향)
        n_in = max(2, int(PARK_IN_S / dt))
        n_fwd = max(2, int(PARK_IN_S / 2 / dt))
        poses = [lane_pt(pos0 + (pA - pos0) * k / n_fwd) for k in range(1, n_fwd + 1)]
        rA = (xA - hx * L, yA - hy * L)
        P = [rA, (rA[0] - hx * 3.0, rA[1] - hy * 3.0), (deep[0] - din[0] * 3.5, deep[1] - din[1] * 3.5), deep]
        for k in range(1, n_in + 1):
            rx, ry, mv = bez(P, k / n_in)
            hd = (mv + 180.0) % 360.0                       # 뒤로 가므로 차 앞은 진행 반대쪽
            ux, uy = unit(hd)
            poses.append((rx + ux * L, ry + uy * L, hd))
        n_park = len(poses)                                    # 여기부터 칸 안에 들어가 있는 장면
        n_dwell = max(1, int(self.mom.dwell / dt))      # 마지막 곳도 같은 시간만큼 주차된 모습을 보여 주고 끝
        poses += [(front_park[0], front_park[1], head_out)] * n_dwell
        if not final:
            ox, oy = unit(head_out)
            xE, yE, hE = lane_pt(pos0)                        # ego 가 서 있는 자리 (앞범퍼)
            ex_, ey_ = unit(hE)
            Q = [front_park, (front_park[0] + ox * 2.5, front_park[1] + oy * 2.5),
                 (xE - ex_ * 2.5, yE - ey_ * 2.5), (xE, yE)]
            poses += [bez(Q, k / n_in) for k in range(1, n_in + 1)]

        poly = "ego_park_anim"
        gui = bool(self.gui)
        try:
            if gui:
                for pid in (poly, poly + "_ws", poly + "_rw"):
                    if pid in traci.polygon.getIDList():
                        traci.polygon.remove(pid)
                traci.vehicle.setColor(ego, (255, 0, 0, 0))          # 실제 ego 는 숨김 (통로 그 자리에 서 있음)
            for i, (x, y, a) in enumerate(poses):
                traci.vehicle.setSpeed(ego, 0.0)
                if gui:
                    for pid, shp, col in self._car_polys(x, y, a, L, W, unit):
                        if i == 0:
                            traci.polygon.add(pid, shp, col, fill=True, layer=250 if pid == poly else 260)   # 유리는 차체보다 확실히 위
                        else:
                            traci.polygon.setShape(pid, shp)
                traci.simulationStep()
                self._update_heads()
                if gui and PARK_GUI_HOLD_S > 0 and n_park <= i < n_park + n_dwell:
                    import time as _t
                    _t.sleep(PARK_GUI_HOLD_S / n_dwell)        # 화면에서만 칸 안 장면을 천천히 (학습은 화면 없음)
                if ego not in traci.vehicle.getIDList():
                    return True
            if final:
                return False                                   # 마지막 곳: 칸에 들어간 그림으로 끝 (에피소드 종료)
            if gui:
                for pid in (poly, poly + "_ws", poly + "_rw"):
                    traci.polygon.remove(pid)
                traci.vehicle.setColor(ego, (255, 0, 0, 255))
        except traci.TraCIException as e:
            print(f"[park] 주차 동작 중 TraCI 오류: {e}", flush=True)
        traci.vehicle.setSpeed(ego, 0.0)
        return False

    def _ensure_ego_type(self):
        if "ego_type" in traci.vehicletype.getIDList():
            return
        traci.vehicletype.copy("DEFAULT_VEHTYPE", "ego_type")
        traci.vehicletype.setAccel("ego_type", self.max_accel)
        traci.vehicletype.setDecel("ego_type", self.max_decel)
        traci.vehicletype.setMaxSpeed("ego_type",
                                      float(self.ego_cfg.get("max_speed", self.vmax)))
        traci.vehicletype.setColor("ego_type", (255, 0, 0, 255))

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._veh_ids = {}                 # 데이터셋 차량 번호는 에피소드마다 새로

        self._apply_curriculum()
        self._episode_count += 1

        if self.fast_reset:
            # 빠른 시작: SUMO 는 켜 둔 채, 저장해 둔 준비 상태 중 하나를 불러온다.
            # (처음이거나 교통량 배율이 바뀌었을 때만 SUMO 를 새로 켜고 준비 상태를 만든다)
            if not self._started or self._pool_scale != self.traffic_scale:
                if self._started:
                    traci.close()
                    self._started = False
                self._start_sumo()
                self._build_state_pool()
            self._load_state(self._state_pool[np.random.randint(len(self._state_pool))])
            # 같은 준비 상태라도 매번 다른 교통에서 시작하도록 무작위로 조금 더 진행
            for _ in range(np.random.randint(0, self.fast_extra_max + 1)):
                traci.simulationStep()
        else:
            # SUMO 상태를 완전히 초기화하는 가장 확실한 방법은 껐다 켜는 것.
            if self._started:
                traci.close()
                self._started = False
            self._start_sumo()

            # 도로 전 구간에 배경차를 미리 배치 (첫 스텝부터 실제 교통류 상태)
            if self.prefill_enabled:
                self._prefill_traffic()
            self._spawn_illegal_parking()
            # 배치된 차들이 자리를 잡도록 몇 스텝만 정착시킨다.
            for _ in range(self.warmup_steps):
                traci.simulationStep()
                self._update_heads()       # GUI 신호등 그림 (준비 구간에도 실제 신호에 맞춤)
            # 2026-11-18: 빠른 시작(상태 불러오기)을 끄면서, 그쪽에 있던 "출발 전 0~extra_steps_max 무작위 진행"
            #   (신호 위상이 매 판 달라지게)을 여기서도 유지. GUI 는 예전 그대로(바로 출발).
            if not self.gui:
                for _ in range(np.random.randint(0, self.fast_extra_max + 1)):
                    traci.simulationStep()

        # ego 경로/차종 준비 (회전교차로 맵: 경로를 최단경로로 만들고,
        # rou.xml에 ego_type이 없으면 기본 차종을 복사해 만든다)
        # ego 투입 (출발 지점이 막혀 있으면 최대 5번 다른 상태/지점으로 재시도)
        for _attempt in range(5):
            if self.routes:
                self._pick_route(options)
                self._prepare_route()
            self._ensure_ego_type()

            # ego 차량 투입 — 반드시 "도로 시작점(0m)"에서 출발
            traci.vehicle.add(self.ego_id, self.ego_route_id, typeID="ego_type",
                              departLane=self._this_depart_lane or self.ego_depart_lane,
                              departPos=self._this_depart_pos,
                              departSpeed=str(self._this_depart_speed if self._this_depart_speed is not None
                                              else self.ego_depart_speed))

            # ★ 중요: speedMode=0 → SUMO의 내장 안전장치(자동 안전제동 등)를 전부 끔.
            #   이걸 꺼야 RL이 속도를 온전히 제어하고, 잘못 배우면 실제로 충돌한다.
            #   (켜두면 SUMO가 알아서 브레이크를 밟아줘서 충돌이 거의 안 나고,
            #    그러면 "충돌 회피"라는 학습 신호 자체가 사라진다)
            traci.vehicle.setSpeedMode(self.ego_id, self.ego_speed_mode)
            # SUMO의 자율 차선변경은 끄고, RL이 내리는 TraCI 차선변경 명령만 사용한다.
            # (회전교차로 맵은 경로 유지용 차선변경만 SUMO에 맡길 수 있다 — lane_change_mode)
            traci.vehicle.setLaneChangeMode(self.ego_id, self.ego_lane_change_mode)

            traci.simulationStep()  # ego가 실제로 도로에 나타나도록 1스텝 진행
            self._update_heads()

            # ── ego 삽입 보증 ──
            # 도로 입구는 flow가 차를 스폰하는 자리라, 시작 차선 입구가 잠시
            # 막혀 있으면 SUMO가 삽입을 "지연"시킨다. 그 경우 ego 없이
            # 에피소드가 진행되어 즉시 종료되는 버그가 되므로, 삽입될 때까지
            # (최대 30스텝) 기다린다. 지연되어도 요청한 차선은 유지된다.
            _wait = 0
            while self.ego_id not in traci.vehicle.getIDList() and _wait < 30:
                traci.simulationStep()
                self._update_heads()
                _wait += 1
            if self.ego_id in traci.vehicle.getIDList():
                break
            # 출발 지점이 계속 막혀 있으면 (교통이 많을 때 경로 중간 출발 등) 넣지 못한 ego 를 지우고
            # 다른 준비 상태 / 다른 출발 지점으로 다시 시도한다
            try:
                traci.vehicle.remove(self.ego_id)
            except traci.TraCIException:
                pass
            if self.fast_reset and self._state_pool:
                self._load_state(self._state_pool[np.random.randint(len(self._state_pool))])
                for _ in range(np.random.randint(0, self.fast_extra_max + 1)):
                    traci.simulationStep()
        else:
            raise RuntimeError(
                "ego 삽입 실패: 5번 시도했지만 출발 지점이 계속 막혀 있습니다. "
                "교통량을 낮추거나 depart_speed를 확인하세요.")

        # GUI 기본 시점: ego를 화면 중앙에서 추적하고 충분히 가까이 확대한다.
        # 긴 직선도로 전체를 한 화면에 맞추는 기본 뷰는 차량이 점/삼각형처럼
        # 보이기 쉬우므로 학습/평가 관찰용으로는 이 설정이 더 직관적이다.
        if self.gui and self.ego_id in traci.vehicle.getIDList():
            try:
                view_ids = traci.gui.getIDList()
                if view_ids:
                    view_id = view_ids[0]
                    # tracking을 먼저 켜고 zoom을 마지막에 적용해야 SUMO가
                    # 차량 추적을 시작하면서 viewport를 다시 덮어쓰지 않는다.
                    if self.gui_track_ego:
                        traci.gui.trackVehicle(view_id, self.ego_id)
                    traci.gui.setZoom(view_id, self.gui_zoom)
            except traci.TraCIException:
                pass

        self.step_count = 0
        if self.urban:
            self.urban.reset()
        if self.mom:
            self.mom.reset(self.ego_route_edges, self.route.get("stops", []), self._route_offset,
                           float(self._this_depart_pos))
        return self._get_obs(), {"route": self.route_name, "traffic_scale": self.traffic_scale}

    # ==================================================================
    # 관측 만들기 (레이아웃은 env/mdp_config.py 주석 참고)
    #
    # 핵심 설계: 특정 도로에 대한 하드코딩 없이, 매 스텝 SUMO 네트워크를
    # 실시간 조회해서 "지금 내 주변"을 계산한다. 그래서 도로 구조가
    # 어떻게 바뀌어도(차선 수, 합류, 차선 감소 등) 이 코드는 그대로 동작한다.
    #
    # 사용하는 TraCI API:
    #   getLeader / getFollower : 같은 차선의 앞/뒤 차량 (edge 경계도 넘어 탐색)
    #   getNeighbors(mode)      : 옆 차선의 앞/뒤 차량.
    #       mode 비트: bit0 = 0:왼쪽 / 1:오른쪽,  bit1 = 0:후행 / 1:선행
    #   lane.getLinks           : 차선이 다음 edge로 이어지는지 (연결성 판단)
    # ==================================================================

    # (차선 오프셋, 선행차 getNeighbors mode, 후행차 mode)
    #   오프셋 +1 = 왼쪽 차선 (SUMO는 차선 index가 왼쪽으로 갈수록 커짐)
    _NEIGHBOR_MODES = {+1: (0b10, 0b00),   # 왼쪽: 선행=2, 후행=0
                       -1: (0b11, 0b01)}   # 오른쪽: 선행=3, 후행=1

    def _neighbor(self, offset: int, leader: bool):
        """오프셋 차선(-1=오른쪽, 0=현재, +1=왼쪽)의 가장 가까운
        선행(leader=True)/후행 차량을 (거리 m, 그 차량 속도)로 반환.
        visibility 밖이거나 없으면 None."""
        if offset == 0:
            if leader:
                res = traci.vehicle.getLeader(self.ego_id, self.visibility)
                if res is None:
                    return None
                vid, gap = res
            else:
                vid, gap = traci.vehicle.getFollower(self.ego_id, self.visibility)
                if vid == "":
                    return None
        elif abs(offset) >= 2:
            # 두 칸 이상 옆 차선: SUMO getNeighbors는 "바로 옆"만 지원하므로
            # 차선 직접 스캔(_scan_lane_neighbors — 같은 edge의 차량 목록을
            # 훑어 가장 가까운 앞/뒤를 찾음)으로 감지한다.
            front, rear = self._scan_lane_neighbors(offset)
            res = front if leader else rear
            if res is None or res[0] < 0 or res[0] > self.visibility:
                return None
            return float(res[0]), float(res[1])
        else:
            mode_leader, mode_follower = self._NEIGHBOR_MODES[offset]
            pairs = traci.vehicle.getNeighbors(
                self.ego_id, mode_leader if leader else mode_follower)
            if not pairs:
                return None
            vid, gap = min(pairs, key=lambda p: p[1])   # 가장 가까운 차량

        if gap < 0 or gap > self.visibility:
            return None
        return float(gap), traci.vehicle.getSpeed(vid)

    def _lane_connectivity(self, offset: int) -> float:
        """오프셋 차선의 '끊김 정도'를 반환.

        -1 : 그 차선이 존재하지 않음
         1 : visibility 이내에서 계속 이어짐
        0~1: (차선이 끝나는 지점까지의 거리 / visibility)
             → 값이 작을수록 곧 차선이 사라진다는 경고

        일반화 원리: 차선의 남은 길이와, 차선 끝에서 다음 edge로의
        연결(link) 존재 여부를 SUMO 네트워크에서 직접 조회한다.
        차선 감소/합류 지점은 "링크 없는 차선의 끝"으로 자연히 감지된다.
        """
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            if edge.startswith(":"):          # 교차로(junction) 내부 통과 중
                return 1.0
            lane_index = traci.vehicle.getLaneIndex(self.ego_id) + offset
            if lane_index < 0 or lane_index >= traci.edge.getLaneNumber(edge):
                return -1.0                   # 그런 차선이 없음
            lane_id = f"{edge}_{lane_index}"
            remaining = (traci.lane.getLength(lane_id)
                         - traci.vehicle.getLanePosition(self.ego_id))
            if remaining > self.visibility:
                return 1.0                    # 시야 내에선 계속 이어짐
            if traci.lane.getLinks(lane_id):
                return 1.0                    # edge가 끝나도 다음으로 연결됨
            return float(np.clip(remaining / self.visibility, 0.0, 1.0))
        except traci.TraCIException:
            return 1.0                        # 조회 실패 시 중립값

    def _lane_density(self, offset: int) -> float:
        """오프셋 차선의 '전방 차량 밀도'를 0~1로 반환.

        밀도 = (시야 W 이내 전방 차량 수) / (그 구간의 최대 수용 대수)
               최대 수용 대수 = 사용가능거리 / density_veh_length

        인코딩 관례 (막힘 정도로 읽히도록):
            1.0 : 차선이 없거나 사실상 진입 불가  ← "꽉 찬 것"처럼 취급해서
                  에이전트가 자연스럽게 회피하게 만든다
            0.0 : 텅 빈 차선
        일반화: 차선이 시야보다 짧으면 link를 따라 다음 차선까지 이어서 센다.
        곧 끊기는 차선(link 없음)은 남은 거리만큼만 세므로 분모가 줄어
        같은 대수여도 밀도가 높게 나온다 → "끊기는 차선에 차가 몰려있음" 신호.
        """
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            if edge.startswith(":"):          # 교차로 내부: 중립값
                return 0.0
            lane_index = traci.vehicle.getLaneIndex(self.ego_id) + offset
            if lane_index < 0 or lane_index >= traci.edge.getLaneNumber(edge):
                return 1.0                    # 차선 없음 = 꽉 참 취급
            lane_id = f"{edge}_{lane_index}"
            ego_pos = traci.vehicle.getLanePosition(self.ego_id)
            remaining = traci.lane.getLength(lane_id) - ego_pos
            links = traci.lane.getLinks(lane_id)

            # 이 차선에서 앞으로 실제로 쓸 수 있는 거리
            usable = self.visibility if links else min(self.visibility, remaining)
            if usable <= 0:
                return 1.0

            count = 0
            # 2026-11-17: 도심 맵은 가려지지 않은(보이는) 차만 센다 (urban_driving._near 의 가시성 판정과 같음)
            vis = self.urban.visible if (self.urban is not None and getattr(self.urban, "visible", None) is not None) else None
            # (a) 현재 차선 구간의 전방 차량
            for vid in traci.lane.getLastStepVehicleIDs(lane_id):
                if vid == self.ego_id:
                    continue
                d = traci.vehicle.getLanePosition(vid) - ego_pos
                if 0.0 < d <= min(self.visibility, remaining) and (vis is None or vid in vis):
                    count += 1
            # (b) 시야가 edge 끝을 넘고 다음 차선으로 이어지면 이어서 센다
            if links and remaining < self.visibility:
                next_lane_id = links[0][0]    # 첫 번째 연결 차선
                for vid in traci.lane.getLastStepVehicleIDs(next_lane_id):
                    if traci.vehicle.getLanePosition(vid) <= \
                            self.visibility - remaining and (vis is None or vid in vis):
                        count += 1

            max_count = usable / self.density_veh_len
            return float(np.clip(count / max_count, 0.0, 1.0))
        except traci.TraCIException:
            return 0.0

    def get_detected_vehicles(self):
        """현재 관측 시야에 들어온(=감지된) 배경차들의 좌표 목록을 반환.

        데이터셋 수집용 부가 정보 — 관측 벡터에는 들어가지 않는다.
        "감지"의 정의는 관측과 동일: 관측 대상 차선(obs_offsets 범위)에서
        ego 기준 종방향 ±visibility(W) 이내에 있는 차량.

        Returns: list[dict]  각 원소 =
            {"id": 차량 id, "x": 절대 x, "y": 절대 y,
             "rel_pos": ego 기준 종방향 상대거리(m, 앞 +), "speed": m/s,
             "lane_offset": ego 차선 기준 오프셋(+왼쪽)}
        ego가 없으면 빈 리스트.
        """
        out = []
        if self.ego_id not in traci.vehicle.getIDList():
            return out
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            if edge.startswith(":"):
                return out
            ego_lane = traci.vehicle.getLaneIndex(self.ego_id)
            ego_pos = traci.vehicle.getLanePosition(self.ego_id)
            n_lanes = traci.edge.getLaneNumber(edge)
            for offset in self.obs_offsets:
                li = ego_lane + offset
                if li < 0 or li >= n_lanes:
                    continue
                for vid in traci.lane.getLastStepVehicleIDs(f"{edge}_{li}"):
                    if vid == self.ego_id:
                        continue
                    rel = traci.vehicle.getLanePosition(vid) - ego_pos
                    if abs(rel) > self.visibility:
                        continue
                    x, y = traci.vehicle.getPosition(vid)
                    out.append({"id": self._veh_num(vid), "x": float(x), "y": float(y),
                                "rel_pos": float(rel),
                                "speed": float(traci.vehicle.getSpeed(vid)),
                                "lane_offset": int(offset)})
        except traci.TraCIException:
            pass
        return out

    def _get_obs(self):
        # ego가 이미 사라졌다면(충돌/도착) 0벡터 반환 — 종료 직전 마지막 관측용
        if self.ego_id not in traci.vehicle.getIDList():
            return np.zeros(self.obs_dim, dtype=np.float32)

        v = traci.vehicle.getSpeed(self.ego_id)
        # 도심 관측을 먼저 계산 (주변 차량 가시성 판정 → 아래 차로 밀도가 보이는 차만 세도록). 벡터 순서는 그대로
        lead_f = self.urban.leader_obs() if (self.urban and self.obs_ego_leader) else []   # 앞차를 먼저 정해야 주변 6대에서 뺄 수 있음
        urban_f = self.urban.features() if self.urban else []
        obs = [v / self.vmax]                      # [0] 내 절대속도
        obs += lead_f                              # [1] 현재 차로 앞차 간격/50 m, [2] 상대속도 (원본 현재 차로 선행차 2칸과 같은 정의)

        # ----- 5개 차선(좌2→좌1→현재→우1→우2)의 선행/후행 상대거리·상대속도 (도심 맵은 뺌, 위 obs_dim 주석) -----
        for offset in (self.obs_offsets if self.obs_lane_neighbors else ()):
            lead = self._neighbor(offset, leader=True)
            if lead is None:
                obs += [1.0, 1.0]                  # 없음: 멀고 + 위협 없음
            else:
                gap, lv = lead
                obs += [gap / self.visibility,
                        float(np.clip((lv - v) / self.vmax, -1.0, 1.0))]

            foll = self._neighbor(offset, leader=False)
            if foll is None:
                obs += [-1.0, -1.0]                # 없음: 멀고 + 위협 없음
            else:
                gap, fv = foll
                obs += [-gap / self.visibility,    # 후행은 음수 부호
                        float(np.clip((fv - v) / self.vmax, -1.0, 1.0))]

        # ----- 5개 차선의 연결성 (같은 순서) -----
        for offset in self.obs_offsets:
            obs.append(self._lane_connectivity(offset))

        # ----- 5개 차선의 전방 차량 밀도 (같은 순서) -----
        for offset in self.obs_offsets:
            obs.append(self._lane_density(offset))

        # ----- 도심 경로 주행: 신호·경로·교차로 상충 차량·주변 차량 -----
        if self.urban:
            obs += urban_f
        if self.mom:
            obs += self.mom.features()

        return np.clip(np.array(obs, dtype=np.float32), -1.0, 1.0)

    # ==================================================================
    # 1 스텝 진행: 행동 적용 → 시뮬레이션 1스텝 → 보상/종료 판정
    # ==================================================================
    def _quantize_lane_change(self, raw: float) -> int:
        """연속 policy 출력을 실제 차선 명령 {-1,0,+1}로 변환한다.

        +1 = 왼쪽, 0 = 유지, -1 = 오른쪽. SUMO 차선 index가 왼쪽으로
        갈수록 증가하는 규약과 동일해서 해석이 직관적이다.
        """
        raw = float(np.clip(raw, -1.0, 1.0))
        if raw >= self.lane_change_threshold:
            return 1
        if raw <= -self.lane_change_threshold:
            return -1
        return 0

    def _scan_lane_neighbors(self, direction: int):
        """목표 차선을 직접 스캔해 (가장 가까운 앞차 gap·속도, 뒷차 gap·속도) 반환.

        getNeighbors의 백업. getNeighbors는 SUMO 차선변경모델의 이웃 캐시를
        읽는데, "방금 스폰된 차량"은 다음 스텝까지 이 캐시에 없어서
        도로 입구 근처에서 사각지대가 생긴다. 차선의 차량 목록을 직접
        훑으면 이미 삽입된 차량은 전부 잡힌다.
        """
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            li = traci.vehicle.getLaneIndex(self.ego_id) + direction
            # 존재하지 않는 차선이면 조회하지 않는다.
            # (없는 lane id를 traci.lane.*로 조회하면 SUMO가 매번
            #  에러 로그를 찍으므로, 범위 검사를 먼저 해서 조용히 건너뛴다)
            if edge.startswith(":") or li < 0 \
                    or li >= traci.edge.getLaneNumber(edge):
                return None, None
            lane_id = f"{edge}_{li}"
            my_pos = traci.vehicle.getLanePosition(self.ego_id)
            front, rear = None, None
            for vid in traci.lane.getLastStepVehicleIDs(lane_id):
                d = traci.vehicle.getLanePosition(vid) - my_pos
                if d >= 0:
                    g = d - traci.vehicle.getLength(vid)
                    if front is None or g < front[0]:
                        front = (g, traci.vehicle.getSpeed(vid))
                else:
                    g = -d - traci.vehicle.getLength(self.ego_id)
                    if rear is None or g < rear[0]:
                        rear = (g, traci.vehicle.getSpeed(vid))
            return front, rear
        except traci.TraCIException:
            return None, None

    def _lane_change_is_safe(self, direction: int) -> bool:
        """목표 차선으로 지금 이동해도 측면충돌이 나지 않는지 검사한다.

        3중 안전 규칙:
          (0) 입구 보호구간: ego가 도로 시작 spawn_protect(m) 이내면 변경 금지.
              flow가 입구(0m)에서 차를 새로 만드는데, 생성 "직전" 차량은
              어떤 API로도 미리 볼 수 없어 원천적으로 사각지대이기 때문.
          (1) 앞: gap > min_front_gap + front_tau × 내속도
              + 접근 여유: gap > (내속도 − 앞차속도)⁺ × 1s   (내가 더 빠를 때)
          (2) 뒤: gap > min_rear_gap + rear_tau × 뒤차속도
              + 접근 여유: gap > (뒤차속도 − 내속도)⁺ × 1s   (뒤가 더 빠를 때)
        이웃은 getNeighbors 결과와 차선 직접 스캔 결과 중 "더 가까운 쪽"을
        쓴다 (스폰 직후 차량 사각지대 보완).
        """
        my_pos = traci.vehicle.getLanePosition(self.ego_id)
        # 입구 보호구간은 출발 edge에서만 (회전교차로 맵은 edge가 여러 개라
        # 모든 edge 시작마다 변경을 막으면 안 된다)
        on_depart_edge = (not self.routes or
                          traci.vehicle.getRoadID(self.ego_id) == self.ego_depart_edge)
        if on_depart_edge and my_pos < self.lc_spawn_protect:
            return False

        v = traci.vehicle.getSpeed(self.ego_id)
        scan_front, scan_rear = self._scan_lane_neighbors(direction)

        def closer(a, b):
            if a is None:
                return b
            if b is None:
                return a
            return a if a[0] <= b[0] else b

        lead = closer(self._neighbor(direction, leader=True), scan_front)
        if lead is not None:
            gap, lv = lead
            if gap <= self.lc_front_min + self.lc_front_tau * v:
                return False
            if gap <= max(v - lv, 0.0) * self.lc_horizon:  # 시간여유 내 접촉 위험
                return False

        foll = closer(self._neighbor(direction, leader=False), scan_rear)
        if foll is not None:
            gap, fv = foll
            if gap <= self.lc_rear_min + self.lc_rear_tau * fv:
                return False
            if gap <= max(fv - v, 0.0) * self.lc_horizon:  # 시간여유 내 접촉 위험
                return False
        return True

    def _apply_lane_change(self, lane_change: int) -> bool:
        """가능하고 "안전한" 경우에만 차선변경을 적용하고 성공 여부를 반환.

        안전 게이트(_lane_change_is_safe)를 통과하지 못한 명령은 실행하지
        않고 False를 반환한다 → step()에서 "불가능한 명령"과 동일하게
        invalid_action_penalty가 부과된다. 즉 에이전트 입장에서
        "위험한 변경 = 실행도 안 되고 감점"이라, 측면충돌로 즉사하며
        배우는 대신 안전한 타이밍을 고르는 쪽으로 학습이 유도된다.
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return False
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            if edge.startswith(":"):
                return False
            current = traci.vehicle.getLaneIndex(self.ego_id)
            n_lanes = traci.edge.getLaneNumber(edge)
            target = current + lane_change
            if target < 0 or target >= n_lanes:
                return False
            # ★ 측면충돌 방지: 실제 이동 명령(0이 아닌 변경)에만 안전 검사
            if (lane_change != 0 and self.lc_safety_enabled
                    and not self._lane_change_is_safe(lane_change)):
                return False
            traci.vehicle.changeLane(
                self.ego_id, target, self.lane_change_duration)
            return True
        except traci.TraCIException:
            return False

    # ---- 데이터셋 숫자 ID (2026-11-17 컨설팅 반영: 칼럼은 가능하면 문자열 대신 숫자) ----
    def edge_index(self):
        """도로(edge) ID → 정수 (맵의 모든 도로, 교차로 내부 포함, 이름순). 데이터셋과 함께 저장해 되돌릴 수 있다."""
        if getattr(self, "_edge_idx", None) is None:
            self._edge_idx = {e: i for i, e in enumerate(sorted(traci.edge.getIDList()))}
        return self._edge_idx

    def _veh_num(self, vid):
        """차량 ID → 정수 (에피소드 안에서 처음 본 순서, ego = 0)."""
        if vid == self.ego_id:
            return 0
        m = self.__dict__.setdefault("_veh_ids", {})
        if vid not in m:
            m[vid] = len(m) + 1
        return m[vid]

    def _tls_columns(self):
        """다음 신호를 숫자 칼럼으로 쪼갠 것 (있음 0/1, 정지선까지 거리 m, 적·황·녹 0/1)."""
        nt = self.urban.last.get("tls") if self.urban is not None else None
        if not nt:
            return {"tls_exists": 0, "tls_dist": -1.0, "tls_red": 0, "tls_yellow": 0, "tls_green": 0}
        c = nt[2]
        return {"tls_exists": 1, "tls_dist": float(nt[1]),
                "tls_red": int(c in "rRs"), "tls_yellow": int(c in "yYu"), "tls_green": int(c in "Gg")}

    def get_privileged_state(self, radius=None) -> dict:
        """POMDP 데이터셋용 simulator-side state.

        agent가 실제로 받는 입력은 _get_obs()의 부분관측이고, 이 메서드는
        분석/지도신호용 privileged state만 수집한다. 정책 입력으로 쓰지 않는다.
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return {"ego_alive": False}
        try:
            x, y = traci.vehicle.getPosition(self.ego_id)
            vehicles = []
            for vid in traci.vehicle.getIDList():
                if vid == self.ego_id:
                    continue
                try:
                    vx, vy = traci.vehicle.getPosition(vid)
                    if radius is not None and (vx - x) ** 2 + (vy - y) ** 2 > radius * radius:
                        continue   # 데이터셋 용량: ego 주변 radius m 안의 차량만 기록
                    vehicles.append({
                        "id": self._veh_num(vid),
                        "x": float(vx),
                        "y": float(vy),
                        "speed": float(traci.vehicle.getSpeed(vid)),
                        "lane": max(int(traci.vehicle.getLaneIndex(vid)), -1),   # 주차 칸 안 = -1
                        "road": self.edge_index().get(traci.vehicle.getRoadID(vid), -1),
                        "lane_pos": float(traci.vehicle.getLanePosition(vid)),
                        "emergency": int(traci.vehicle.getVehicleClass(vid) == "emergency"),
                    })
                except traci.TraCIException:
                    continue
            return {
                "ego_alive": True,
                "ego_x": float(x),
                "ego_y": float(y),
                "ego_speed": float(traci.vehicle.getSpeed(self.ego_id)),
                "ego_lane": int(traci.vehicle.getLaneIndex(self.ego_id)),
                "ego_road": self.edge_index().get(traci.vehicle.getRoadID(self.ego_id), -1),
                "ego_lane_pos": float(traci.vehicle.getLanePosition(self.ego_id)),
                "ego_angle": float(traci.vehicle.getAngle(self.ego_id)),
                "ego_accel": float(traci.vehicle.getAcceleration(self.ego_id)),
                **self._tls_columns(),
                "vehicles": vehicles,
            }
        except traci.TraCIException:
            return {"ego_alive": False}

    def step(self, action):
        # action = [가감속 raw, 차선변경 raw]
        action_arr = np.asarray(action, dtype=np.float32).reshape(-1)
        if action_arr.size < self.act_dim:
            raise ValueError(
                f"action은 {self.act_dim}차원이어야 합니다. 받은 shape={np.asarray(action).shape}")

        accel_raw = float(np.clip(action_arr[self.accel_index], -1.0, 1.0))
        lane_raw = float(np.clip(action_arr[self.lane_change_index], -1.0, 1.0))
        lane_change = self._quantize_lane_change(lane_raw)
        lane_change_applied = False
        lane_before = None
        mask_why = []          # 이번 스텝 실제로 개입한 이유
        mask_would = []        # 마스킹이 꺼져 있을 때: 켜져 있었다면 개입했을 이유
        red_violation = False

        # ----- (1) 종방향 + 횡방향 행동 적용 -----
        if self.ego_id in traci.vehicle.getIDList():
            v = traci.vehicle.getSpeed(self.ego_id)
            lane_before = int(traci.vehicle.getLaneIndex(self.ego_id))

            accel_cmd = (accel_raw * self.max_accel
                         if accel_raw >= 0.0 else accel_raw * self.max_decel)
            dv = accel_cmd * self.step_length
            v_cmd = float(np.clip(v + dv, 0.0, self.vmax))
            if self.urban:
                # 안전 마스킹: 적신호 진입·양보 위반·추돌이 확실한 경우에만 속도를 줄인다
                v_masked, why = self.urban.mask(v, v_cmd)
                if self.urban.mask_enabled:
                    v_cmd, mask_why = v_masked, why
                else:
                    mask_would = why
            traci.vehicle.setSpeed(self.ego_id, v_cmd)

            lane_change_applied = self._apply_lane_change(lane_change)
        if self.mom:
            self.mom.before_step()        # 학습용 긴급차 투입 + 배경차가 긴급차에 비켜 주기 (ego 는 건드리지 않음)

        # ----- (2) 시뮬레이션 1스텝 -----
        traci.simulationStep()
        self.step_count += 1
        self.total_steps += 1
        self._update_heads()
        teleported = self.ego_id in traci.simulation.getStartingTeleportIDList()
        if self.urban:
            red_violation = self.urban.after_step(mask_why, mask_would)

        # ----- (3) 이벤트/관측 -----
        collided = self.ego_id in traci.simulation.getCollidingVehiclesIDList()
        # 충돌로 제거된 차도 SUMO는 arrived 목록에 넣으므로 충돌이면 도착이 아니다
        arrived = (not collided and
                   self.ego_id in traci.simulation.getArrivedIDList())
        alive = self.ego_id in traci.vehicle.getIDList()
        lane_after = int(traci.vehicle.getLaneIndex(self.ego_id)) if alive else None

        obs = self._get_obs()
        terminated, truncated = False, False
        ego_speed, ego_gap = None, None

        terms = {}
        if self.urban:
            ego_speed = traci.vehicle.getSpeed(self.ego_id) if alive else None
            lead = self._neighbor(0, leader=True) if alive else None
            ego_gap = lead[0] if lead else None
            urw = self.urban.rw
            # 엄마 경로: 주차장 정차 판정 (마지막 곳 정차 = 완주, 지나치면 실패) + 긴급차 길막기·단속 카메라 감점
            mom_r, mom_done, mom_failed, mom_terms = (self.mom.after_step(alive and not collided, arrived)
                                                      if self.mom else (0.0, False, False, {}))
            if self.mom:
                arrived = mom_done           # 완주 = 외할머니 댁 주차장에 정차 (경로 끝에 그냥 닿은 것은 실패)
            if collided:
                reward = -float(urw.get("collision_penalty", self.rw["collision_penalty"]))
                terminated = True
            elif arrived:
                reward = float(urw.get("arrival_bonus", self.rw["arrival_bonus"]))
                terminated = True
            elif mom_failed:
                # 들를 주차장에 서지 않고 지나침 (되돌아갈 수 없음) = 경로 이탈과 같은 실패
                teleported = True
                reward = -float(urw.get("lost_penalty", 15.0))
                terms["missed_stop"] = reward
                terminated = True
            elif teleported or not alive or self.urban.wrong_lane_dead_end():
                # 오래 막혀 순간이동 / 경로를 벗어나 사라짐 / 틀린 차로로 차로 끝에 갇힘 = 실패
                teleported = True
                reward = -float(urw.get("lost_penalty", 15.0))
                terminated = True
            elif red_violation:
                # 신호 위반 = 실패로 즉시 종료 (교통 규칙 위반을 충돌에 가깝게 취급)
                reward = -float(urw.get("red_violation_penalty", 20.0))
                terms["red_violation"] = reward
                terminated = True
            else:
                reward, terms = self.urban.step_reward(ego_speed, mask_why, ego_gap,
                                                       lane_change, lane_change_applied)
                reward += mom_r
                terms.update(mom_terms)
        # ----- (4) 기존 보상 유지 -----
        elif collided:
            reward = -float(self.rw["collision_penalty"])
            terminated = True
        elif arrived:
            reward = float(self.rw["arrival_bonus"])
            terminated = True
        elif not alive:
            reward = 0.0
            terminated = True
        else:
            ego_speed = traci.vehicle.getSpeed(self.ego_id)
            same_lane_leader = self._neighbor(0, leader=True)
            if same_lane_leader is not None:
                ego_gap = same_lane_leader[0]
            reward = float(self.rw["speed_weight"]) * ego_speed / self.vmax
            if (ego_gap is not None and
                    ego_gap < float(self.rw["close_gap_threshold"])):
                reward -= float(self.rw["close_gap_penalty"])

            # ---- 막힘 페널티: 느린 앞차 뒤에 갇혀 있는 동안 매 스텝 감점 ----
            # "차선을 유지한 채 막혀 있기"와 "변경으로 탈출하기" 사이에
            # 즉각적인 보상 차등을 만들어 추월 학습 신호를 조밀하게 한다.
            # (자세한 설계 이유는 env/mdp_config.py의 REWARD 주석 참고)
            bp = float(self.rw.get("blocked_penalty", 0.0))
            if (bp > 0.0 and ego_gap is not None
                    and ego_gap < float(self.rw.get("blocked_gap", 20.0))
                    and ego_speed < float(self.rw.get("blocked_speed_frac", 0.7))
                        * self.vmax):
                reward -= bp

            # ---- 차선변경 보상 항 ----
            # lane_change        : 이번 스텝에 policy가 낸 양자화 명령 (-1/0/+1)
            # lane_change_applied: 그 명령이 실제로 실행됐는지
            # 주의: _apply_lane_change는 유지 명령(0)에도 현재 차선으로
            # changeLane을 호출해 True를 반환한다(차선 고정 용도).
            # 따라서 "실제 변경"은 lane_change != 0 과 함께 판정해야 한다.
            if lane_change != 0 and lane_change_applied:
                # 실행된 변경마다 소액 감점 → 이득 없는 지그재그 억제.
                # 변경 덕에 속도 보상이 그 이상 늘면 순이득이므로,
                # "필요할 때만 바꾸는" 정책으로 수렴하게 만드는 항이다.
                reward -= float(self.rw.get("lane_change_penalty", 0.0))
            elif lane_change != 0:
                # 명령은 냈는데 실행 불가(차선 없음/교차로)였던 경우 감점
                # → 의미 없는 행동 출력을 줄이는 정칙화 (rl_action_penalty 대응)
                reward -= float(self.rw.get("invalid_action_penalty", 0.0))

        # 엄마 경로 (2026-11-17 사용자 요청 (b)): 정책이 빈 칸 옆에 스스로 정차했으면 환경은 후진 주차 동작만 한다
        if self.mom and self.mom.park_request and not collided:
            space, final = self.mom.park_request
            self.mom.park_request = None
            gone = self._park_ego(space, final)
            if gone and not terminated:
                terminated = True
                reward = -float(self.urban.rw.get("lost_penalty", 15.0))
                teleported = True
            obs = self._get_obs()

        if self.step_count >= self.max_steps:
            truncated = True

        info = {
            "collided": collided,
            "arrived": arrived,
            "speed": ego_speed,
            "gap": ego_gap,
            "accel_raw": accel_raw,
            "lane_change_raw": lane_raw,
            "lane_change": lane_change,
            "lane_change_applied": lane_change_applied,
            "lane_before": lane_before,
            "lane_after": lane_after,
            "route": self.route_name,
            "lost": bool(self.urban and teleported),
            "mask": mask_why,              # 이번 스텝 안전 마스킹 개입 이유 (red_light / yield / rear_end / junction_clear)
            "mask_would": mask_would,      # 마스킹을 끈 경우: 켜져 있었다면 개입했을 이유
            "red_violation": red_violation,
            "reward_terms": terms,
        }
        if self.mom:
            # 엄마 경로 과제 이벤트 (데이터 수집 팀별 추가 항목)
            info["mom"] = {"parked": int(self.mom.ep["parked"]), "park_now": "park" in terms or bool(arrived),
                           "missed_stop": bool(self.mom.ep["missed_stop"]),
                           "ev_near": bool(self.mom.last.get("ev_near")), "ev_block": bool(self.mom.last.get("ev_block")),
                           "camera_ticket": "camera" in terms}
        if self.urban and (terminated or truncated):
            ep = self.urban.episode_summary()
            ep.update(route=self.route_name, collided=collided, arrived=arrived,
                      red_violation_end=bool(red_violation and not collided and not arrived),
                      lost=bool(teleported or (not alive and not arrived and not collided)),
                      truncated=truncated, traffic_scale=self.traffic_scale)
            if self.mom:
                ep.update(self.mom.ep, stops_total=self.mom.n_total)
            info["episode_stats"] = ep
            self._finished_stats.append(ep)
        return obs, float(reward), terminated, truncated, info

    # ==================================================================
    def close(self):
        """환경 정리. 학습이 끝나면 반드시 호출해서 SUMO 프로세스를 종료할 것."""
        if self._started:
            traci.close()
            self._started = False
        if self._state_dir:
            import shutil
            shutil.rmtree(self._state_dir, ignore_errors=True)
            self._state_dir = None
