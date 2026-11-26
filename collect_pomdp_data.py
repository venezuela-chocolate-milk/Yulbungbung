"""POMDP trajectory data collector for the SUMO highway environment.

저장 단위는 POMDP transition 하나다:
    (o_t, a_t, r_t, o_{t+1}, terminated, truncated, s_t, s_{t+1})

- o_t / o_{t+1}: agent가 실제로 보는 부분관측 (19차원)
- a_t: [가감속 raw, 차선변경 raw]
- lane_change: 실제 환경에 적용된 {-1, 0, +1}
- s_t / s_{t+1}: SUMO 내부의 privileged simulator state.
  정책 입력에는 쓰지 않고 데이터 검증/representation/belief 학습용으로만 저장한다.

용량을 줄이기 위해:
- JSONL은 gzip으로 압축해서 쓰고(.jsonl.gz), observation/state/detected_vehicles의
  float는 --ndigits 자리로 반올림한다 (NPZ는 float32 그대로).
- next_observation / next_state는 다음 레코드의 observation / state와 같으므로
  에피소드 마지막 스텝(terminated or truncated)에만 기록한다.
  읽을 때는 iter_transitions()가 채워서 돌려준다.
- NPZ의 감지 차량 좌표는 object 배열(pickle) 대신 flat (M, 2) + offsets로 저장한다.

속도:
- headless면 sumo_env가 libsumo(in-process SUMO)를 써서 TraCI 소켓 왕복이 사라진다
  (pip install libsumo). --workers N으로 SUMO 인스턴스 N개를 병렬로 돌려 더 줄인다.

예시:
    python collect_pomdp_data.py --episodes 100
    python collect_pomdp_data.py --episodes 10 --gui
    python collect_pomdp_data.py --policy keep-lane --episodes 20
    python collect_pomdp_data.py --model results/run_xxx/model.pt --episodes 100   # 학습한 PPO로 수집
    python collect_pomdp_data.py --model results/run_xxx/model.pt --stochastic    # 샘플링 행동
    python collect_pomdp_data.py --episodes 200 --workers 8   # SUMO 8개 병렬 수집

출력:
    data/pomdp_YYYYMMDD_HHMMSS.jsonl.gz
    data/pomdp_YYYYMMDD_HHMMSS.npz

읽기:
    from collect_pomdp_data import iter_transitions
    for rec in iter_transitions("data/pomdp_....jsonl.gz"):
        rec["observation"], rec["next_observation"], rec["state"], rec["next_state"], ...

    d = np.load("data/pomdp_....npz")          # allow_pickle 불필요
    off = d["detected_offsets"]
    xy_i = d["detected_xy"][off[i]:off[i + 1]]  # 스텝 i의 감지 차량 (N_i, 2)
"""
import argparse
import gzip
import json
import multiprocessing
import os
import shutil
import time
from datetime import datetime

import numpy as np

from env.maps import make_env
from train import HPARAMS, REWARD, MAP  # 학습과 같은 보상/맵 조건 + PPO 네트워크 구조

BASE = os.path.dirname(os.path.abspath(__file__))


def detect_algorithm(model_path: str) -> str:
    """model.pt가 어느 학습 스크립트 출력인지 파일 내용으로 판별한다.

    train.py(PPO)  : policy.state_dict()를 그대로 저장 → 텐서만 있는 dict
    train_bc.py(BC): {"algorithm": "bc", "state_dim", ..., "state_dict"} dict
    """
    import torch
    ck = torch.load(model_path, map_location="cpu", weights_only=True)
    return "bc" if isinstance(ck, dict) and ck.get("algorithm") == "bc" else "ppo"


def make_policy(args, env, worker_seed: int):
    """args.policy에 맞는 obs -> action_raw(np.float32, shape (2,)) 함수를 만든다.

    ppo / bc 출력은 [-1, 1]로 클리핑해 저장한다. 환경의 입력 범위이기도 하고,
    train_bc.py가 |action_raw| <= 1을 검증하므로 이 데이터로 바로 BC를 돌릴 수 있다.
    """
    if args.policy == "ppo":
        # test.py와 동일: train.py의 HPARAMS로 같은 구조를 만든 뒤 가중치만 덮어쓴다.
        # seed는 워커마다 다르게 (--stochastic 샘플링이 워커 간에 겹치지 않도록).
        from algorithms.ppo import PPO, hidden_sizes_from_checkpoint
        hs = hidden_sizes_from_checkpoint(args.model) or HPARAMS["hidden_sizes"]
        agent = PPO(obs_dim=env.observation_space.shape[0],
                    act_dim=env.action_space.shape[0],
                    **{**HPARAMS, "device": args.device, "seed": worker_seed, "hidden_sizes": hs})
        agent.load(args.model)
        deterministic = not args.stochastic   # hybrid: (μ, argmax 차선) / 샘플링

        def act(obs):
            a = agent.predict(obs, deterministic=deterministic)
            return np.clip(a, -1.0, 1.0).astype(np.float32)
        return act

    if args.policy == "bc":
        from algorithms.bc import BCPolicy
        model = BCPolicy.load(args.model, device=args.device)

        def act(obs):
            return np.clip(model.predict(obs), -1.0, 1.0).astype(np.float32)
        return act

    if args.policy == "expert":
        # 규칙 기반 강사 (env/expert_driver.py): 모방학습 데이터용. 시뮬레이터 정보로 판단하지만
        # 행동은 정책과 똑같은 [가감속, 차선변경] 이고 환경 개입 수준도 같다.
        from env.expert_driver import RuleBasedExpert
        expert = RuleBasedExpert(env)

        def act(obs):
            return expert.act()
        act.reset = expert.reset
        return act

    if args.policy == "keep-lane":
        # 완만한 가속 + 차선 유지. GUI/파이프라인 점검용.
        return lambda obs: np.asarray([0.25, 0.0], dtype=np.float32)

    # random: 가감속은 연속 uniform[-1,1], 차선변경은 {-1,0,+1} 직접 샘플링 (유지를 더 자주)
    rng = np.random.default_rng(worker_seed)

    def act(obs):
        accel = rng.uniform(-1.0, 1.0)
        lane = rng.choice(np.asarray([-1.0, 0.0, 1.0]), p=[0.15, 0.70, 0.15])
        return np.asarray([accel, lane], dtype=np.float32)
    return act


def compact(obj, ndigits: int):
    """중첩 dict/list/ndarray를 float가 ndigits 자리로 반올림된 순수 Python 객체로 바꾼다.

    float32 값을 그대로 json.dumps하면 0.10000000149011612처럼 17자리로 찍히는데,
    반올림하면 0.1이 되어 텍스트 길이가 크게 준다. numpy 스칼라/배열도 함께 변환한다.
    """
    if isinstance(obj, (float, np.floating)):
        return round(float(obj), ndigits)
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, np.ndarray):
        return compact(obj.tolist(), ndigits)
    if isinstance(obj, dict):
        return {k: compact(v, ndigits) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [compact(v, ndigits) for v in obj]
    return obj


def iter_transitions(jsonl_path: str):
    """저장된 JSONL(.gz 또는 평문)을 읽어, 생략된 next_observation / next_state를
    다음 레코드에서 채운 완전한 transition dict를 순서대로 yield한다.

    (수집이 중간에 끊긴 파일이면 마지막 레코드에만 next_*가 없을 수 있다.)
    """
    opener = gzip.open if jsonl_path.endswith(".gz") else open
    with opener(jsonl_path, "rt", encoding="utf-8") as f:
        prev = None
        for line in f:
            if not line.strip():
                continue
            rec = json.loads(line)
            if prev is not None:
                # done 레코드는 next_*를 이미 갖고 있으므로 setdefault가 건드리지 않는다.
                prev.setdefault("next_observation", rec["observation"])
                prev.setdefault("next_state", rec["state"])
                yield prev
            prev = rec
        if prev is not None:
            yield prev


ROUTE_IDS = {"mom": 0}
SIG_K = None              # 관측 안 신호 칸 시작 번호 (collect 에서 환경 관측 차원으로 정함, env/obs_layout.py)     # 경로 이름 → 숫자 (2026-11-17: 데이터 칼럼은 숫자로)


def _write_index_files(env, out_dir):
    """숫자 칼럼을 되돌리는 표: 도로 번호(state.ego_road, vehicles.road), 경로 번호(route)."""
    try:
        with open(os.path.join(out_dir, "edge_index.json"), "w", encoding="utf-8") as f:
            json.dump({str(i): e for e, i in env.edge_index().items()}, f, ensure_ascii=False)
        with open(os.path.join(out_dir, "route_index.json"), "w", encoding="utf-8") as f:
            json.dump({str(v): k for k, v in ROUTE_IDS.items()}, f, ensure_ascii=False)
    except Exception as e:   # noqa: BLE001
        print("index 파일 저장 실패:", e)


def collect(args, sumocfg, episodes, worker_id, jsonl_path, npz_path, tag=""):
    """SUMO 인스턴스 하나로 episodes(전역 에피소드 번호 목록)를 수집해 두 파일로 저장한다.

    --workers > 1이면 이 함수가 프로세스마다 하나씩 돌고, main()이 결과를 합친다.
    반환값: 수집한 transition 수.
    """
    worker_seed = (args.seed * 1000 + worker_id) % 2**32
    if args.workers > 1:
        # 프로세스마다 torch가 코어 수만큼 스레드를 띄우면 서로 뺏는다. 관측 1개씩
        # 추론이라 1스레드면 충분.
        try:
            import torch
            torch.set_num_threads(1)
        except ImportError:
            pass

    env, _ = make_env(MAP, reward=REWARD, gui=args.gui, gui_autostart=True,
                      build=False, ev_mode=args.ev_mode)   # 도로 파일은 main()에서 한 번만 준비
    global SIG_K
    if getattr(env, "urban", None) is not None:
        from env.obs_layout import signal_index
        SIG_K = signal_index(env.observation_space.shape[0])
    act = make_policy(args, env, worker_seed)
    # sumo_env가 SUMO --seed를 np.random(전역)에서 뽑는다. PPO 생성자가 전역 시드를
    # HPARAMS["seed"]로 되돌려 놓으므로 정책을 만든 "뒤에" 워커별로 다시 시드해야
    # 워커들이 같은 교통 패턴을 반복하지 않는다.
    np.random.seed(worker_seed)
    rng_noise = np.random.default_rng(worker_seed + 7)

    # 회전교차로 맵 옵션: 워커별 교통량, 경로 중간 출발 비율
    scale = None
    if getattr(env, "routes", None):
        scales = [float(x) for x in args.traffic_scales.split(",")] if args.traffic_scales else []
        if scales:
            scale = scales[worker_id % len(scales)]
            env.traffic_scale = scale
        if args.random_start > 0:
            from env import roundabout_env_config as rc
            env.random_start = {**rc.RANDOM_START, "prob": args.random_start}
    write_jsonl = not args.no_jsonl
    # DAgger: 정책이 운전하고, 매 스텝 강사(규칙 기반 운전자)가 "자기라면 했을 행동"을 정답으로 단다
    labeler = None
    if args.label_expert:
        from env.expert_driver import RuleBasedExpert
        labeler = RuleBasedExpert(env)

    # NPZ는 학습에서 바로 쓰기 좋은 고정 크기 값만 모은다.
    obs_buf, next_obs_buf, action_raw_buf = [], [], []
    lane_cmd_buf, reward_buf = [], []
    term_buf, trunc_buf, episode_buf, step_buf = [], [], [], []
    exec_buf, success_buf, noisy_buf = [], [], []   # 실제 실행 행동, 에피소드 완주 여부, 잡음 에피소드 여부
    # 스텝별 감지 차량 (x,y)는 한 줄로 이어 붙이고, 스텝 경계는 offsets로 기록한다.
    detected_xy_buf, detected_offsets = [], [0]

    total = 0
    try:
        # compresslevel 6: 9와 크기 차이는 거의 없고 훨씬 빠르다.
        # 더 줄이고 싶으면 gzip.open 대신 lzma.open(..., "wt") (.jsonl.xz) —
        # 느린 대신 보통 20~30% 추가 절감.
        jf = gzip.open(jsonl_path, "wt", encoding="utf-8", compresslevel=6) if write_jsonl else None
        try:
            for ep in episodes:
                obs, _ = env.reset(seed=args.seed + ep)
                if hasattr(act, "reset"):
                    act.reset()
                if labeler is not None:
                    labeler.reset()
                state_t = env.get_privileged_state(args.state_radius) if write_jsonl else None
                if worker_id == 0 and ep == episodes[0]:
                    _write_index_files(env, os.path.dirname(jsonl_path) or ".")
                done = False
                t = 0
                ep_return = 0.0
                ep_start = len(obs_buf)
                # 잡음 에피소드: 실행 행동에만 잡음을 섞고, 기록(action_raw)은 정책의 원래 행동
                noisy = rng_noise.random() < args.noise_episodes
                burst_left, burst_val = 0, 0.0

                while not done:
                    # 이번 관측 시점에 시야(W) 안에 들어온 차량들의 좌표.
                    # 관측 벡터와 별도로 저장하는 부가 데이터 (분석/시각화용).
                    detected = env.get_detected_vehicles()
                    action = act(obs)
                    executed = np.asarray(action, dtype=np.float32).copy()
                    if labeler is not None:
                        action = labeler.act()            # 기록(action_raw)은 강사의 행동, 실행은 정책 행동
                        # DAgger β 섞기: 이 확률로 그 스텝은 강사 행동을 실행 → 정책이 초반에 실패해도
                        # 경로 뒷부분까지 가서 정책이 갈 법한 상태를 모은다 (2026-11-16)
                        if args.dagger_beta > 0 and rng_noise.random() < args.dagger_beta:
                            executed = np.asarray(action, dtype=np.float32).copy()
                    if noisy:
                        if args.noise_burst_prob > 0:
                            # 묶음(burst) 잡음: 가끔 몇 스텝 동안 같은 방향으로 밀어 실제로 복구가 필요한
                            # 어긋남을 만든다. 정지선 앞(적·황 40 m 이내)과 거의 멈춘 상태(주차 정차)에서는
                            # 넣지 않는다 — 매 스텝 잡음은 정지선을 밀고 넘어가 신호위반으로 바로 끝났음 (2026-11-16)
                            o = np.asarray(obs)
                            k = SIG_K if SIG_K is not None else 31
                            calm = (o[0] < 0.05) or ((o[k] > 0) and (o[k + 2] > 0 or o[k + 3] > 0) and o[k + 1] * 200 < 40)
                            if burst_left <= 0 and not calm and rng_noise.random() < args.noise_burst_prob:
                                burst_left = int(rng_noise.integers(args.noise_burst_len[0], args.noise_burst_len[1] + 1))
                                burst_val = rng_noise.normal(0, args.accel_noise)
                            if burst_left > 0:
                                burst_left -= 1
                                if not calm:
                                    executed[0] = np.clip(executed[0] + burst_val, -1, 1)
                        else:
                            executed[0] = np.clip(executed[0] + rng_noise.normal(0, args.accel_noise), -1, 1)
                        if rng_noise.random() < args.lane_noise:
                            executed[1] = rng_noise.choice([-1.0, 1.0])
                    next_obs, reward, terminated, truncated, info = env.step(executed)
                    state_tp1 = env.get_privileged_state(args.state_radius) if write_jsonl else None
                    done = terminated or truncated

                    record = {
                        "episode": int(ep),
                        "t": t,
                        "observation": compact(obs, args.ndigits),
                        # 스칼라 몇 개는 용량에 영향이 없으니 반올림하지 않는다.
                        "action_raw": np.asarray(action, dtype=float).tolist(),
                        "action": {
                            "accel_raw": float(info["accel_raw"]),
                            "lane_change_raw": float(info["lane_change_raw"]),
                            "lane_change": int(info["lane_change"]),
                            "lane_change_applied": bool(info["lane_change_applied"]),
                        },
                        "reward": float(reward),
                        "terminated": bool(terminated),
                        "truncated": bool(truncated),
                        "state": compact(state_t, args.ndigits),
                        # 감지 차량 좌표 (관측과 같은 시점 t 기준)
                        "detected_vehicles": compact(detected, args.ndigits),
                        "event": {
                            "collided": bool(info["collided"]),
                            "arrived": bool(info["arrived"]),
                            "lane_before": info["lane_before"],
                            "lane_after": info["lane_after"],
                            # ---- 팀별 추가 항목 (회전교차로 엄마 경로) ----
                            "red_violation": bool(info.get("red_violation", False)),
                            "lost": bool(info.get("lost", False)),
                            **({k: v for k, v in info["mom"].items()} if info.get("mom") else {}),
                        },
                        "reward_terms": {k: round(float(v), 5) for k, v in (info.get("reward_terms") or {}).items()},
                        # 2026-11-17: 문자열 대신 숫자 (0 = mom 엄마 심부름 경로, data/route_index.json)
                        "route": ROUTE_IDS.get(info.get("route"), -1),
                    }
                    if done:
                        # 중간 스텝의 next_*는 다음 레코드의 observation/state와 같으므로
                        # 에피소드 마지막 스텝에만 기록한다 (iter_transitions가 복원).
                        record["next_observation"] = compact(next_obs, args.ndigits)
                        record["next_state"] = compact(state_tp1, args.ndigits)
                    if jf is not None:
                        jf.write(json.dumps(record, ensure_ascii=False,
                                            separators=(",", ":")) + "\n")

                    obs_buf.append(np.asarray(obs, dtype=np.float32))
                    next_obs_buf.append(np.asarray(next_obs, dtype=np.float32))
                    action_raw_buf.append(np.asarray(action, dtype=np.float32))
                    exec_buf.append(executed)
                    noisy_buf.append(noisy)
                    lane_cmd_buf.append(int(info["lane_change"]))
                    reward_buf.append(float(reward))
                    term_buf.append(bool(terminated))
                    trunc_buf.append(bool(truncated))
                    episode_buf.append(int(ep))
                    step_buf.append(t)
                    detected_xy_buf.extend((d["x"], d["y"]) for d in detected)
                    detected_offsets.append(len(detected_xy_buf))

                    total += 1
                    ep_return += reward
                    t += 1
                    obs = next_obs
                    state_t = state_tp1  # 스텝 끝의 상태 = 다음 스텝 시작의 상태

                ok = bool(info.get("arrived"))
                success_buf.extend([ok] * (len(obs_buf) - ep_start))
                print(f"{tag}episode {ep + 1:4d}/{args.episodes} | "
                      f"steps={t:4d} return={ep_return:8.3f} | "
                      f"{'완주' if ok else '실패'}{' (잡음)' if noisy else ''}"
                      + (f" | 교통량 {scale}" if scale is not None else ""), flush=True)
        finally:
            if jf is not None:
                jf.close()
    finally:
        env.close()

    np.savez_compressed(
        npz_path,
        observation=np.asarray(obs_buf, dtype=np.float32).reshape(total, -1),
        action_raw=np.asarray(action_raw_buf, dtype=np.float32).reshape(total, -1),
        lane_change=np.asarray(lane_cmd_buf, dtype=np.int8),
        reward=np.asarray(reward_buf, dtype=np.float32),
        next_observation=np.asarray(next_obs_buf, dtype=np.float32).reshape(total, -1),
        terminated=np.asarray(term_buf, dtype=np.bool_),
        truncated=np.asarray(trunc_buf, dtype=np.bool_),
        episode=np.asarray(episode_buf, dtype=np.int32),
        t=np.asarray(step_buf, dtype=np.int32),
        # 감지 차량 (x,y)를 전 스텝에 걸쳐 이어 붙인 (M, 2) float32.
        # 스텝 i의 좌표: detected_xy[detected_offsets[i]:detected_offsets[i+1]]
        detected_xy=np.asarray(detected_xy_buf, dtype=np.float32).reshape(-1, 2),
        detected_offsets=np.asarray(detected_offsets, dtype=np.int64),
        # 추가 항목 (모방학습용): 실제 실행한 행동(잡음 포함), 그 에피소드가 완주했는지,
        # 잡음 에피소드인지, 교통량 배율
        action_exec=np.asarray(exec_buf, dtype=np.float32).reshape(total, -1),
        ep_success=np.asarray(success_buf, dtype=np.bool_),
        noisy=np.asarray(noisy_buf, dtype=np.bool_),
        # DAgger: 정답(action_raw)을 강사가 다시 단 데이터인지 (실패 에피소드여도 정답은 올바름)
        relabeled=np.full(total, labeler is not None, dtype=np.bool_),
        traffic_scale=np.full(total, scale if scale is not None else -1.0, dtype=np.float32),
    )
    return total


def merge_parts(parts, jsonl_path, npz_path):
    """워커별 부분 파일들을 하나로 합치고 부분 파일은 지운다."""
    # gzip은 멤버를 이어 붙인 파일도 유효하므로 바이트 그대로 연결한다.
    if all(os.path.exists(p) for p, _ in parts):
        with open(jsonl_path, "wb") as out:
            for part_jsonl, _ in parts:
                with open(part_jsonl, "rb") as f:
                    shutil.copyfileobj(f, out)
                os.remove(part_jsonl)

    arrays, xy, offsets, base = {}, [], [], 0
    for _, part_npz in parts:
        with np.load(part_npz) as d:
            for key in d.files:
                if key not in ("detected_xy", "detected_offsets"):
                    arrays.setdefault(key, []).append(d[key])
            xy.append(d["detected_xy"])
            off = d["detected_offsets"]
            offsets.append(off[:-1] + base)   # 부분 파일의 offset을 전체 기준으로 옮김
            base += int(off[-1])
        os.remove(part_npz)
    merged = {key: np.concatenate(v) for key, v in arrays.items()}
    merged["detected_xy"] = np.concatenate(xy).reshape(-1, 2)
    merged["detected_offsets"] = np.concatenate(
        offsets + [np.asarray([base], dtype=np.int64)]).astype(np.int64)
    np.savez_compressed(npz_path, **merged)


def main():
    parser = argparse.ArgumentParser(description="SUMO POMDP trajectory collector")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--policy", choices=["random", "keep-lane", "ppo", "bc", "expert"], default=None,
                        help="행동 정책. 생략 시 --model이 있으면 파일을 보고 ppo/bc 자동 판별, "
                             "없으면 random")
    parser.add_argument("--model", default=None,
                        help="train.py(PPO) 또는 train_bc.py(BC)가 저장한 model.pt. "
                             "주면 그 학습된 정책으로 수집한다")
    parser.add_argument("--stochastic", action="store_true",
                        help="ppo: 결정적(μ, argmax 차선) 대신 분포에서 샘플링해서 행동. "
                             "상태 다양성이 필요할 때")
    parser.add_argument("--device", default="cpu",
                        help="정책 추론 장치 (cpu/cuda). 관측 1개씩 추론이라 cpu로 충분")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--gui", action="store_true",
                        help="SUMO GUI로 수집. 기본은 headless")
    parser.add_argument("--ev-mode", choices=["train", "eval", "ev_only"], default="train",
                        help="(회전교차로 맵) 긴급차 빈도. 학습 데이터 수집은 train (85%% 에피소드, 1~2번)")
    parser.add_argument("--workers", type=int, default=1,
                        help="병렬 SUMO 인스턴스 수. 코어 수만큼 주면 거의 그 배수로 빨라진다. "
                             "(--gui면 1로 고정)")
    parser.add_argument("--out-dir", default=os.path.join(BASE, "data"))
    parser.add_argument("--name", default=None,
                        help="출력 basename. 생략 시 날짜/시각 자동 생성")
    parser.add_argument("--ndigits", type=int, default=4,
                        help="JSONL의 observation/state/detected float 소수 자릿수 "
                             "(NPZ에는 영향 없음)")
    parser.add_argument("--label-expert", action="store_true",
                        help="(DAgger) 정책(--model)이 운전하고, 기록하는 정답(action_raw)은 규칙 기반 강사의 행동")
    parser.add_argument("--dagger-beta", type=float, default=0.0,
                        help="(DAgger, --label-expert 와 함께) 스텝마다 이 확률로 강사 행동을 실행 (예: 0.5)")
    parser.add_argument("--state-radius", type=float, default=None,
                        help="state.vehicles에 ego 반경 몇 m 안의 차량만 기록 (기본: 전체 차량)")
    parser.add_argument("--no-jsonl", action="store_true",
                        help="용량이 큰 JSONL(전체 상태 포함)은 만들지 않고 학습용 NPZ만 저장")
    parser.add_argument("--noise-episodes", type=float, default=0.0,
                        help="이 비율의 에피소드는 실행 행동에 잡음을 섞는다 (기록은 정책의 원래 행동). "
                             "모방학습에서 실수 회복 장면을 얻기 위함")
    parser.add_argument("--accel-noise", type=float, default=0.3, help="잡음 에피소드의 가감속 잡음 표준편차")
    parser.add_argument("--lane-noise", type=float, default=0.02,
                        help="잡음 에피소드에서 스텝마다 무작위 차선변경 명령을 섞을 확률")
    parser.add_argument("--noise-burst-prob", type=float, default=0.0,
                        help="주면 가감속 잡음을 매 스텝 대신 묶음으로: 스텝마다 이 확률로 시작 (예: 0.01). "
                             "정지선 앞·정차 중에는 잡음 없음")
    parser.add_argument("--noise-burst-len", type=int, nargs=2, default=[4, 12],
                        help="묶음 잡음 길이 범위 (스텝, 최소 최대)")
    parser.add_argument("--traffic-scales", default=None,
                        help="(회전교차로 맵) 워커별로 나눠 쓸 교통량 배율 목록, 예: 0.2,0.4,0.6,0.8,1.0")
    parser.add_argument("--random-start", type=float, default=0.0,
                        help="(회전교차로 맵) 경로 중간의 무작위 도로에서 출발하는 에피소드 비율")
    args = parser.parse_args()
    if args.policy in ("ppo", "bc") and args.model is None:
        parser.error(f"--policy {args.policy}에는 --model이 필요합니다.")
    if args.model is not None and not os.path.exists(args.model):
        parser.error(f"모델 파일이 없습니다: {args.model}")
    if args.policy is None:
        args.policy = detect_algorithm(args.model) if args.model else "random"
    if args.model:
        print(f"정책: {args.policy} ({args.model})"
              + (" [stochastic]" if args.stochastic and args.policy == "ppo" else ""))
    workers = 1 if args.gui else max(1, min(args.workers, args.episodes))

    os.makedirs(args.out_dir, exist_ok=True)
    name = args.name or datetime.now().strftime("pomdp_%Y%m%d_%H%M%S")
    jsonl_path = os.path.join(args.out_dir, name + ".jsonl.gz")
    npz_path = os.path.join(args.out_dir, name + ".npz")

    # 도로 파일은 부모에서 한 번만 생성한다 (워커들이 동시에 쓰면 파일이 깨진다).
    _env, map_info = make_env(MAP, reward=REWARD)   # highway면 여기서 도로 생성
    sumocfg = map_info["sumocfg"]
    print(f"맵: {map_info['desc']}")

    # 병렬 워커는 libsumo(소켓 없는 SUMO)로 실행 — 자식 프로세스가 환경변수를 물려받는다
    if workers > 1 and not args.gui:
        os.environ["SUMO_LIBSUMO"] = "1"

    t0 = time.time()
    if workers == 1:
        total = collect(args, sumocfg, list(range(args.episodes)), 0,
                        jsonl_path, npz_path)
    else:
        chunks = [c.tolist() for c in np.array_split(np.arange(args.episodes), workers)]
        parts = [(os.path.join(args.out_dir, f".{name}.w{w}.jsonl.gz"),
                  os.path.join(args.out_dir, f".{name}.w{w}.npz"))
                 for w in range(workers)]
        jobs = [(args, sumocfg, chunks[w], w, parts[w][0], parts[w][1], f"[w{w}] ")
                for w in range(workers)]
        # spawn: 워커마다 깨끗한 프로세스에서 SUMO(libsumo)를 하나씩 띄운다.
        with multiprocessing.get_context("spawn").Pool(workers) as pool:
            total = sum(pool.starmap(collect, jobs))
        merge_parts(parts, jsonl_path, npz_path)
    elapsed = time.time() - t0

    mb = lambda p: os.path.getsize(p) / 2**20  # noqa: E731
    print("\n수집 완료")
    print(f"  transitions: {total}  ({elapsed:.1f}s, {total / max(elapsed, 1e-9):.0f} step/s, "
          f"workers={workers})")
    if os.path.exists(jsonl_path):
        print(f"  JSONL(full POMDP + privileged state): {jsonl_path} ({mb(jsonl_path):.1f} MB)")
    print(f"  NPZ(training-friendly):               {npz_path} ({mb(npz_path):.1f} MB)")


if __name__ == "__main__":
    main()