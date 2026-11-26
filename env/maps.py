"""학습/평가에 쓸 맵을 고르는 곳.

    env, info = make_env("roundabout", reward=REWARD, gui=False)

  "highway"    : env/road_config.py 로 직선 고속도로를 매번 새로 만든다 (기존 방식)
  "roundabout" : env/sumo_roundabout/ 의 회전교차로 맵을 그대로 불러온다 (다시 만들지 않음)
"""
import os

from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env.sumo_env import SumoHighwayEnv

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS = ("highway", "roundabout")


def config_files(map_name):
    """run 폴더에 스냅샷할 설정 파일 목록."""
    env_dir = os.path.join(BASE, "env")
    main = "road_config.py" if map_name == "highway" else "roundabout_env_config.py"
    return [os.path.join(env_dir, main), os.path.join(env_dir, "mdp_config.py")]


def make_env(map_name, reward, gui=False, gui_autostart=True, build=True, curriculum=False,
             n_envs=1, fast_reset=True, urban_reward=None, ev_mode="eval"):
    """build=False: highway 도로를 다시 만들지 않고 기존 파일 사용 (병렬 워커용).
    curriculum=True: (회전교차로) 학습용 커리큘럼(교통량 단계 + 무작위 출발) 사용. 기본 False — 수업 원본처럼
                기준 교통량, 항상 경로 처음에서 출발.
    n_envs: 병렬 환경 수 (커리큘럼 누적 스텝 계산용). fast_reset: 준비 상태 재사용 (화면 없을 때).
    urban_reward: (회전교차로) URBAN["reward"] 중 덮어쓸 값 (예: {"lane_change_penalty": 0.02}).
    ev_mode: (회전교차로) 긴급차 투입 빈도 — "train" (데이터 수집·학습, 자주) / "eval" (결과 확인, 원래 빈도) /
             "ev_only" (긴급차 대응만 평가, 매번). roundabout_env_config.EMERGENCY_MODES 참고."""
    if map_name == "highway":
        from env.road_config import ROAD, EGO, TRAFFIC
        from env import road_builder
        out = os.path.join(BASE, "env", "sumo")
        sumocfg = (road_builder.build(ROAD, EGO, TRAFFIC, out) if build
                   else os.path.join(out, "highway.sumocfg"))
        env = SumoHighwayEnv(cfg_path=sumocfg, road=ROAD, ego=EGO,
                             mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                             action=ACTION, reward=reward, traffic=TRAFFIC,
                             gui=gui, gui_autostart=gui_autostart)
        desc = (f"직선도로 {ROAD['length']}m, {ROAD['num_lanes']}차선, "
                f"교통량 {TRAFFIC['vehs_per_hour']}대/시간")
        return env, {"sumocfg": sumocfg, "desc": desc}

    if map_name == "roundabout":
        from env import roundabout_env_config as rc
        if not os.path.exists(rc.SUMOCFG):
            raise FileNotFoundError(
                f"회전교차로 맵이 없습니다: {rc.SUMOCFG}\n"
                f"  python view_roundabout.py --rebuild 로 먼저 만드세요.")
        sim = {**SIMULATION, **rc.SIMULATION_OVERRIDES}
        env = SumoHighwayEnv(cfg_path=rc.SUMOCFG, road=rc.ROAD, ego=rc.EGO,
                             mdp_sim=sim, mdp_obs=OBSERVATION,
                             action=ACTION, reward=reward, traffic=rc.TRAFFIC,
                             gui=gui, gui_autostart=gui_autostart,
                             route=rc.ROUTES, sumo_args=rc.SUMO_ARGS,
                             gui_additional=rc.GUI_ADDITIONAL, signal_heads=rc.SIGNAL_HEADS,
                             urban=({**rc.URBAN, "reward": {**rc.URBAN["reward"], **urban_reward}}
                                    if urban_reward else rc.URBAN),
                             net_path=rc.NET_FILE,
                             curriculum=rc.CURRICULUM if curriculum else None,
                             curriculum_step_multiplier=n_envs,
                             fast_reset=rc.FAST_RESET if fast_reset else None,
                             random_start=rc.RANDOM_START if curriculum else None,
                             illegal_parking=(__import__('env.roundabout_illegal_parking', fromlist=['x'])
                                              if getattr(rc, 'ILLEGAL_PARKING', False) else None),
                             mom=({**rc.MOM, "emergency": rc.EMERGENCY_MODES[ev_mode]}
                                  if getattr(rc, "MOM", None) is not None else None))
        # 차선변경 안전 게이트(+ 출발 60 m 차선변경 금지): 수업 원본과 같게 켬 (설정 lane_change_gate)
        env.lc_safety_enabled = bool(rc.URBAN.get("lane_change_gate", True))
        desc = "회전교차로 맵, ego 경로 " + ", ".join(r["name"] for r in rc.ROUTES)
        return env, {"sumocfg": rc.SUMOCFG, "desc": desc}

    raise ValueError(f"MAP은 {MAPS} 중 하나여야 합니다: {map_name!r}")


def make_worker_env(map_name, reward, curriculum, n_envs, seed, urban_reward=None, ev_mode="train"):
    """병렬 환경의 프로세스 하나에서 환경을 만든다 (utils/vec_env.SubprocVecEnv 용).
    프로세스마다 시드를 달리해 SUMO 교통·경로 선택이 서로 겹치지 않게 한다."""
    import numpy as np
    np.random.seed(seed)
    env, _ = make_env(map_name, reward=reward, gui=False, build=False,
                      curriculum=curriculum, n_envs=n_envs, urban_reward=urban_reward, ev_mode=ev_mode)
    return env


def make_vec_env(map_name, reward, n_envs, curriculum=False, seed=0, use_libsumo=True,
                 urban_reward=None, ev_mode="train"):
    """SUMO 학습 환경 n_envs 개를 CPU 프로세스로 병렬 실행한다. (학습용이라 긴급차 빈도 기본 "train")"""
    from functools import partial
    from utils.vec_env import SubprocVecEnv
    if map_name == "highway":
        from env.road_config import ROAD, EGO, TRAFFIC
        from env import road_builder
        road_builder.build(ROAD, EGO, TRAFFIC, os.path.join(BASE, "env", "sumo"))   # 한 번만 생성
    fns = [partial(make_worker_env, map_name, reward, curriculum, n_envs, seed * 1000 + i + 1,
                   urban_reward, ev_mode) for i in range(n_envs)]
    # 병렬 프로세스는 libsumo(소켓 없는 SUMO) 사용. 자식 프로세스가 환경변수를 물려받는다.
    old = os.environ.get("SUMO_LIBSUMO")
    os.environ["SUMO_LIBSUMO"] = "1" if use_libsumo else "0"
    try:
        return SubprocVecEnv(fns)
    finally:
        if old is None:
            os.environ.pop("SUMO_LIBSUMO", None)
        else:
            os.environ["SUMO_LIBSUMO"] = old
