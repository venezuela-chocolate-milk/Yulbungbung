"""강건성 평가용 (eval_parallel.py 사본, 11-23): 판마다 seed+판번호로 고정 → 모델 간 같은 판 짝 비교.
학습된 모델(PPO / BC) 또는 규칙 기반 강사를 여러 SUMO 로 병렬 평가한다 (회전교차로 맵).

test.py 와 같은 조건: 커리큘럼·무작위 출발 없음, 경로 처음부터 끝까지, 기준 교통량(기본 0.8),
안전 마스킹 없음(--mask 로 켤 수 있음). 경로마다 같은 수의 에피소드를 돌린다.

    python eval_parallel.py results/bcppo_v1/model.pt --episodes 30 --workers 15
    python eval_parallel.py expert --episodes 30          # 강사 성능 (참고용)
"""
import argparse
import collections
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
def _routes():
    """회전교차로 맵 경로 이름 (지금은 엄마 심부름 경로 하나, env/roundabout_env_config.py)."""
    sys.path.insert(0, BASE)
    from env.roundabout_env_config import ROUTES as R
    return [r["name"] if isinstance(r, dict) else r for r in R]


def _worker(model, jobs, scale, mask, seed, out_q, ev_mode="eval", ev_prob=None):
    os.environ["SUMO_LIBSUMO"] = "1"
    sys.path.insert(0, BASE)
    np.random.seed(seed)
    from env.maps import make_env
    from train import REWARD
    import traci
    env, _ = make_env("roundabout", reward=REWARD, build=False, ev_mode=ev_mode)
    if scale is not None:                     # 2026-11-17: 기본은 환경 설정 교통량 그대로 (배율 사본 1.0)
        env.traffic_scale = scale
    env.set_mask(mask)
    if ev_prob is not None:                   # 강건성 평가: 판마다 확률 ev_prob 로 긴급차 1대
        env.mom.inject_prob = float(ev_prob); env.mom.inject_count = (1, 1)
    if model == "expert":
        from env.expert_driver import RuleBasedExpert
        ex = RuleBasedExpert(env)
        act, reset = (lambda o: ex.act()), ex.reset
    else:
        from test import load_agent
        paths = model.split("+")   # "a.pt+b.pt+c.pt" = 앙상블 (가감속 평균, 차선 확률 평균 후 argmax)
        agents = [load_agent(m, env.observation_space.shape[0], env.action_space.shape[0]) for m in paths]
        if len(agents) == 1:
            agent = agents[0]
            act = lambda o: agent.predict(o, deterministic=True)
        else:
            import torch
            def act(o):
                with torch.no_grad():
                    mus, probs = [], []
                    for ag in agents:
                        t = torch.as_tensor(o, dtype=torch.float32, device=ag.device).unsqueeze(0)
                        mu, _, logits, _ = ag.policy.forward(t)
                        mus.append(mu.cpu()); probs.append(torch.softmax(logits, -1).cpu())
                    mu = torch.stack(mus).mean(0)
                    idx = int(torch.stack(probs).mean(0).argmax(-1))
                    lane = agents[0].policy.LANE_CMDS[idx]
                    return np.array([float(mu[0, 0]), float(lane)], dtype=np.float32)
        reset = lambda: None
    np.random.seed(seed)   # PPO 생성자가 np 시드를 0으로 덮어쓰므로 워커별 시드 다시 지정
    for ep_i, route in jobs:
        np.random.seed(seed + ep_i)            # 판마다 고정 seed: 모델이 달라도 같은 판은 같은 출발 조건
        obs, _ = env.reset(options={"route": route})
        reset()
        ret, n, speeds, lcs = 0.0, 0, [], 0
        last_road = ""
        while True:
            if env.ego_id in traci.vehicle.getIDList():
                last_road = traci.vehicle.getRoadID(env.ego_id)
            obs, r, te, tr, inf = env.step(act(obs))
            ret += r
            n += 1
            if inf.get("speed") is not None:
                speeds.append(inf["speed"])
            lcs += bool(inf["lane_change"] != 0 and inf["lane_change_applied"])
            if te or tr:
                break
        end = ("arrived" if inf["arrived"] else "collided" if inf["collided"] else "lost" if inf["lost"]
               else "red_violation" if inf["red_violation"] else "timeout")
        road = traci.vehicle.getRoadID(env.ego_id) if env.ego_id in traci.vehicle.getIDList() else last_road
        stats = inf.get("episode_stats") or {}
        out_q.put(dict(ep=ep_i, route=route, result=end, ret=ret, steps=n, time_s=n * env.step_length,
                       speed=float(np.mean(speeds)) if speeds else 0.0, lane_changes=lcs,
                       would=stats.get("would_intervene_rate", 0.0), end_road=road,
                       parked=int(stats.get("parked", 0)), stops_total=int(stats.get("stops_total", 0)),
                       missed_stop=bool(stats.get("missed_stop", False)),
                       ev_injected=int(stats.get("ev_injected", 0)),
                       ev_block_steps=int(stats.get("ev_block_steps", 0)),
                       camera_tickets=int(stats.get("camera_tickets", 0)),
                       jerk_mean=float(stats.get("jerk_mean", 0.0)), jerk_max=float(stats.get("jerk_max", 0.0)),
                       jerk_over=float(stats.get("jerk_over_rate", 0.0)), hard_acc=int(stats.get("hard_acc", 0)),
                       hard_brake=int(stats.get("hard_brake", 0))))
    env.close()
    out_q.put(None)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("model", help="model.pt 경로 또는 expert")
    ap.add_argument("--episodes", type=int, default=30, help="경로당 에피소드 수")
    ap.add_argument("--workers", type=int, default=15)
    ap.add_argument("--scale", type=float, default=None, help="교통량 배율 (기본: 환경 설정 그대로)")
    ap.add_argument("--mask", action="store_true", help="비교용: 안전 마스킹 켜기")
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--out", default=None, help="결과 JSON 저장 경로")
    ap.add_argument("--ev-mode", choices=["train", "eval", "ev_only"], default="eval",
                    help="긴급차 빈도 (기본 eval = test.py 와 같음)")
    ap.add_argument("--ev-prob", type=float, default=None, help="강건성: 판마다 긴급차 1대 넣을 확률")
    args = ap.parse_args()
    ROUTES = _routes()

    jobs = list(enumerate(r for _ in range(args.episodes) for r in ROUTES))
    w = max(1, min(args.workers, len(jobs)))
    chunks = [jobs[i::w] for i in range(w)]
    ctx = mp.get_context("spawn")
    q = ctx.Queue()
    procs = [ctx.Process(target=_worker, args=(args.model, c, args.scale, args.mask, args.seed, q, args.ev_mode, args.ev_prob))
             for i, c in enumerate(chunks)]
    t0 = time.time()
    for p in procs:
        p.start()
    res, done = [], 0
    while done < len(procs):
        x = q.get()
        if x is None:
            done += 1
        else:
            res.append(x)
    for p in procs:
        p.join()

    print(f"\n=== {args.model} | 교통량 {args.scale if args.scale is not None else '설정값'} | 마스킹 {'켬' if args.mask else '끔'} | "
          f"{len(res)} 에피소드 | {time.time() - t0:.0f}s ===")
    by = collections.defaultdict(list)
    for r in res:
        by[r["route"]].append(r)
    for route in ROUTES + ["전체"]:
        es = res if route == "전체" else by[route]
        if not es:
            continue
        c = collections.Counter(e["result"] for e in es)
        n = len(es)
        ok = [e for e in es if e["result"] == "arrived"]
        print(f"  {route:18s} n={n:3d}  완주 {c['arrived']/n:5.1%}  충돌 {c['collided']/n:5.1%}  "
              f"신호위반 {c['red_violation']/n:5.1%}  막힘/이탈 {c['lost']/n:5.1%}  시간초과 {c['timeout']/n:5.1%}  "
              f"| 완주 시 평균 {np.mean([e['time_s'] for e in ok]) if ok else 0:5.0f}s, "
              f"속도 {np.mean([e['speed'] for e in es]):4.1f}m/s, 차선변경 {np.mean([e['lane_changes'] for e in es]):4.1f}회")
        if es[0].get("stops_total"):
            ev_eps = [e for e in es if e["ev_injected"]]
            print(f"  {'':18s} 정차 평균 {np.mean([e['parked'] for e in es]):.2f}/{es[0]['stops_total']}  "
                  f"정차 놓침 {np.mean([e['missed_stop'] for e in es]):5.1%}  "
                  f"단속 카메라 {np.mean([e['camera_tickets'] for e in es]):.2f}회/에피소드  "
                  f"긴급차 길막기 {np.mean([e['ev_block_steps'] for e in ev_eps]) if ev_eps else 0:.1f}스텝 "
                  f"(긴급차 나온 {len(ev_eps)}개 에피소드 기준)")
        print(f"  {'':18s} 승차감: 평균 저크 {np.mean([e['jerk_mean'] for e in es]):.2f} m/s³, "
              f"최대 저크 {np.mean([e['jerk_max'] for e in es]):.1f} (에피소드 평균), "
              f"저크 기준(8) 초과 {np.mean([e['jerk_over'] for e in es]):5.1%}, "
              f"급가속(>3) {np.mean([e['hard_acc'] for e in es]):.1f}회, 급제동(<-4) {np.mean([e['hard_brake'] for e in es]):.1f}회 /에피소드")
    fails = collections.Counter((e["route"], e["result"], e["end_road"]) for e in res if e["result"] != "arrived")
    if fails:
        print("  실패 위치 (경로, 결과, 도로): " + ", ".join(f"{k}×{v}" for k, v in fails.most_common(8)))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
