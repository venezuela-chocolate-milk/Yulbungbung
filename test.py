"""학습된 모델을 sumo-gui로 재생하고 주행 지표를 출력한다.

사용법:
    python test.py results/run_20260824_153012/model.pt   ← 모델 경로 지정
    python test.py                                        ← 최근 run 자동 선택
    python test.py --episodes 5 --nogui                   ← 화면 없이 지표만 측정
    python test.py results/bc_example/model.pt --algorithm bc --nogui
    python test.py --episodes 100 --nogui                 ← 최종 평가 (엄마 경로, 마스킹 없음)
    python test.py --episodes 100 --nogui --mask          ← 비교용: 안전 마스킹 켜고

빨간 차가 ego. 앞차가 느리면 감속하고 앞이 비면 가속하는지 관찰해보자.
"""
import argparse
import glob
import os
import torch

from env.maps import make_env

from algorithms.ppo import PPO
from algorithms.bc import BCPolicy
from utils.evaluator import evaluate_policy

# train.py와 동일한 네트워크 구조로 만들어야 가중치를 불러올 수 있으므로
# 하이퍼파라미터를 그대로 가져온다 (구조 관련 항목만 실제로 쓰임).
from train import HPARAMS, RESULTS_DIR, REWARD, MAP  # ← 보상/맵도 train과 동일 조건

BASE = os.path.dirname(os.path.abspath(__file__))


def find_latest_model() -> str:
    """results/*/model.pt 중 가장 최근에 저장된 것을 찾는다."""
    candidates = glob.glob(os.path.join(BASE, RESULTS_DIR, "*", "model.pt"))
    if not candidates:
        raise FileNotFoundError(
            f"'{RESULTS_DIR}/' 안에 model.pt가 없습니다. "
            f"먼저 train.py 또는 train_bc.py로 학습하세요.")
    return max(candidates, key=os.path.getmtime)


def load_agent(model_path, obs_dim, act_dim, algorithm="auto"):
    """BC 체크포인트와 기존 PPO state_dict를 구분해 평가 정책을 복원한다."""
    checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
    if not isinstance(checkpoint, dict):
        raise ValueError("지원하지 않는 모델 파일 형식입니다.")
    if checkpoint.get("algorithm") == "bc":
        detected = "bc"
    elif "trunk.0.weight" in checkpoint and "mu_head.weight" in checkpoint:
        detected = "ppo"
    else:
        raise ValueError("현재 BC 체크포인트 또는 PPO state_dict가 아닙니다.")
    if algorithm != "auto" and algorithm != detected:
        raise ValueError(f"--algorithm {algorithm}과 모델 유형 {detected}가 다릅니다.")

    if detected == "bc":
        if not {"state_dim", "action_dim", "state_dict"}.issubset(checkpoint):
            raise ValueError("현재 BCPolicy 형식이 아닙니다. 현재 train_bc.py로 학습한 모델을 사용하세요.")
        dims = (checkpoint["state_dim"], checkpoint["action_dim"])
        if dims != (obs_dim, act_dim):
            raise ValueError(f"BC 모델 입출력 차원 {dims} != 환경 {(obs_dim, act_dim)}")
        agent = BCPolicy.load(model_path, device="cpu")
    else:
        from algorithms.ppo import hidden_sizes_from_checkpoint
        hs = hidden_sizes_from_checkpoint(model_path) or HPARAMS["hidden_sizes"]
        agent = PPO(obs_dim=obs_dim, act_dim=act_dim, **{**HPARAMS, "hidden_sizes": hs})
        try:
            agent.load(model_path)   # 별도 critic 없는 예전 모델도 허용 (평가에는 critic 불필요)
        except RuntimeError as exc:
            raise ValueError("PPO 모델과 현재 환경 차원 또는 HPARAMS의 네트워크 구조가 다릅니다.") from exc
        agent.policy.eval()
    print(f"평가 알고리즘: {detected.upper()}")
    return agent


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="학습된 모델 재생/평가")
    parser.add_argument("model", nargs="?", default=None,
                        help="model.pt 경로 (생략 시 최근 run 자동 선택)")
    parser.add_argument("--episodes", type=int, default=3,
                        help="재생/평가할 에피소드 수")
    parser.add_argument("--nogui", action="store_true",
                        help="화면 없이 주행 지표만 측정 (빠름)")
    parser.add_argument("--algorithm", choices=["auto", "ppo", "bc"], default="auto",
                        help="기본은 모델 파일에서 PPO/BC 자동 판별")
    parser.add_argument("--no-mask", action="store_true",
                        help="(회전교차로 맵) 안전 마스킹 끄기 (현재 기본값이 끔)")
    parser.add_argument("--mask", action="store_true",
                        help="(회전교차로 맵) 비교용: 안전 마스킹을 켜고 평가")
    parser.add_argument("--route", default=None,
                        help="(회전교차로 맵) 이 경로만 평가. 생략하면 학습 경로를 번갈아 평가")
    parser.add_argument("--ev-only", action="store_true",
                        help="(회전교차로 맵) 긴급차 대응만 평가: 매 에피소드 긴급차 1~2번 (기본은 원래 빈도 50%%·1번)")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes는 1 이상이어야 합니다.")

    model_path = args.model or find_latest_model()
    if not os.path.isabs(model_path):
        model_path = os.path.join(BASE, model_path)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"모델 파일이 없습니다: {model_path}")
    print(f"모델 로드: {model_path}")

    # 주의: 현재 env/ 설정과 다른 설정으로 학습된 모델이면 성능이 이상할 수 있다.
    # 그 run의 설정 스냅샷은 model.pt와 같은 폴더의 *_config.py 에서 확인 가능.
    env, map_info = make_env(MAP, reward=REWARD, gui=not args.nogui,
                             gui_autostart=False,   # ▶(플레이) 버튼을 눌러야 에피소드 시작
                             ev_mode="ev_only" if args.ev_only else "eval")
    print(f"맵: {map_info['desc']}")
    if env.urban is not None:
        if args.mask:
            env.set_mask(True)
        elif args.no_mask:
            env.set_mask(False)
        args.no_mask = not env.urban.mask_enabled
        print("※ 안전 마스킹 " + ("끔: 학습된 정책만으로 주행 (개입했을 상황은 '가상 개입률'로 집계)"
                                  if args.no_mask else "켬 (비교용)"))
    reset_options = None
    if env.routes:
        names = ([args.route] if args.route else
                 [r["name"] for r in env.routes if float(r.get("weight", 1.0)) > 0])
        reset_options = lambda i: {"route": names[i % len(names)]}   # noqa: E731
        print(f"평가 경로: {names} (번갈아)")

    if not args.nogui:
        print("※ 창이 뜨면 ▶(플레이) 버튼을 눌러야 에피소드가 시작됩니다.")
        print("  에피소드가 끝나면 다음 에피소드 창이 다시 뜨고, 또 ▶를 누르면 됩니다.")

    try:
        agent = load_agent(model_path, env.observation_space.shape[0],
                           env.action_space.shape[0], args.algorithm)
        episode_log = []
        metrics = evaluate_policy(agent, env, n_episodes=args.episodes,
                                  deterministic=True, episode_log=episode_log,
                                  reset_options=reset_options)
    finally:
        env.close()

    print(f"\n─── 주행 지표 ({args.episodes} 에피소드 평균) ───")
    print(f"  충돌률       : {metrics.collision_rate:.0%}")
    print(f"  완주율       : {metrics.success_rate:.0%}")
    print(f"  시간초과율   : {metrics.timeout_rate:.0%}")
    print(f"  평균 속도    : {metrics.mean_speed:.2f} m/s "
          f"({metrics.mean_speed * 3.6:.1f} km/h)")
    print(f"  평균 차간거리: {metrics.mean_gap:.2f} m")
    print(f"  최소 차간거리: {metrics.min_gap:.2f} m")
    print(f"  차선변경     : {metrics.lane_changes:.1f} 회/에피소드")
    print(f"  평균 보상    : {metrics.ep_return:.2f}")
    print(f"  평균 길이    : {metrics.ep_length:.0f} 스텝")
    if env.routes:
        print(f"  막힘/이탈률  : {metrics.lost_rate:.0%}")
        print(f"  신호위반 종료: {metrics.red_violation_rate:.0%}")
        if args.no_mask:
            print(f"  가상 개입률  : {metrics.would_intervene_rate:.1%} (마스킹이 켜져 있었다면 개입했을 스텝 비율)")
            print(f"  신호 위반    : {metrics.red_violations:.2f} 회/에피소드")
        else:
            print(f"  마스킹 개입률: {metrics.intervention_rate:.1%} (스텝 기준, 학습될수록 줄어야 함)")
        print("─── 경로별 ───")
        by = {}
        for e in episode_log:
            by.setdefault(e["route"], []).append(e)
        for route, es in by.items():
            n = len(es)
            rate = lambda k: sum(x["result"] == k for x in es) / n   # noqa: E731
            key = "would_intervene_rate" if args.no_mask else "intervention_rate"
            iv = sum(x.get(key, 0.0) for x in es) / n
            rv = sum(x.get("red_violations", 0) for x in es) / n
            print(f"  {route:20s} n={n:3d}  완주 {rate('arrival'):.0%}  충돌 {rate('collision'):.0%}  "
                  f"막힘 {rate('lost'):.0%}  신호위반 {rate('red_violation'):.0%}  시간초과 {rate('timeout'):.0%}  "
                  f"{'가상 ' if args.no_mask else ''}개입률 {iv:.1%}  신호위반 {rv:.2f}")
