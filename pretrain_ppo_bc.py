"""PPO 신경망을 모방학습(Behavior Cloning)으로 먼저 학습한다 → 강화학습(train.py)의 출발점.

수업 코드의 BC(train_bc.py, algorithms/bc.py)는 관측 → 행동 2개를 MSE 로 맞추는 별도 신경망이라
PPO 로 이어서 학습할 수 없다. 그래서 같은 방식(관측 → 강사 행동 따라 하기)을 PPO 신경망
(utils/networks.py HybridActorCritic: 가감속 연속 + 차선 3분류 + 상태 가치)에 적용한다.

  가감속 : 정책 평균(mu)을 강사 값에 맞춘다 (MSE). 강한 제동 장면은 가중치 ↑
  차선   : 왼쪽/유지/오른쪽 3분류 (cross entropy). 드문 차선변경은 빈도 반비례 가중치
  가치   : 데이터의 실제 누적 보상(할인 γ)에 맞춘다 → 강화학습 시작 때 critic 이 이미 맞아 있게
  가감속 탐험 폭(log_std)은 학습하지 않는다 (강화학습에서 쓸 값으로 따로 정함)

    python pretrain_ppo_bc.py --data data/expert_bc_v1_all.npz --out results/bcppo_v1
    python pretrain_ppo_bc.py --data a.npz b.npz --init results/bcppo_v1/model.pt --out ...   # DAgger 재학습
"""
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.nn.functional as F

from train import HPARAMS
from algorithms.ppo import PPO


def discounted_returns(reward, episode, done_end, gamma):
    """에피소드별 할인 누적 보상. 마지막 스텝이 종료(terminated)가 아니면 그 뒤 가치는 0 으로 둔다."""
    ret = np.zeros_like(reward, dtype=np.float32)
    g = 0.0
    for i in range(len(reward) - 1, -1, -1):
        if i == len(reward) - 1 or episode[i + 1] != episode[i]:
            g = 0.0
        g = reward[i] + gamma * g
        ret[i] = g
    return ret


def _keep_mask(d, success_only):
    keep = (d["ep_success"].astype(bool) if success_only and "ep_success" in d.files
            else np.ones(len(d["reward"]), dtype=bool))
    if success_only and "noisy" in d.files:
        keep = keep | d["noisy"].astype(bool)
    if success_only and "relabeled" in d.files:
        keep = keep | d["relabeled"].astype(bool)
    return keep


def load(paths, success_only=True, max_steps=None, seed=0):
    """npz 들을 이어 붙인다. success_only: 완주한 에피소드만 (ep_success 가 있는 파일).
    충돌·막힘으로 끝난 에피소드는 강사가 실수한 장면이라 따라 배우면 안 된다 (2026-11-16).
    max_steps: 합계가 넘으면 최근 2개 파일(최신 DAgger)은 전부 두고 나머지에서 에피소드 단위로 같은 비율만 뽑는다
    (데이터 전체를 GPU 에 올리므로 DAgger 라운드가 쌓이면 메모리가 넘침, 2026-11-22)."""
    frac = {}
    if max_steps:
        n_keep = []
        for p in paths:
            with np.load(p) as d:
                n_keep.append(int(_keep_mask(d, success_only).sum()))
        tot = sum(n_keep)
        if tot > max_steps:
            fixed = sum(n_keep[-2:]) if len(paths) > 2 else 0
            rest = tot - fixed
            f = max(0.05, (max_steps - fixed) / rest) if rest > 0 else 1.0
            for p in (paths[:-2] if len(paths) > 2 else paths):
                frac[p] = min(1.0, f)
            print(f"  데이터 {tot:,} 스텝 > 상한 {max_steps:,} → 앞쪽 파일 에피소드 {min(1.0, f):.0%} 사용 (최근 2개 파일은 전부)")
    rng_s = np.random.default_rng(seed + 12345)
    obs, act, rew, ep = [], [], [], []
    base = 0
    for p in paths:
        with np.load(p) as d:
            keep = (d["ep_success"].astype(bool) if success_only and "ep_success" in d.files
                    else np.ones(len(d["observation"]), dtype=bool))
            # 잡음 에피소드는 실패했어도 쓴다: 실행은 잡음 섞인 행동이지만 기록된 정답(action_raw)은
            # 강사가 그 상태에서 한 복구 행동이라 올바르다 (DART 방식, 2026-11-16)
            if success_only and "noisy" in d.files:
                keep = keep | d["noisy"].astype(bool)
            # DAgger 데이터(강사가 정답을 다시 단 것)도 실패 에피소드까지 전부 쓴다
            if success_only and "relabeled" in d.files:
                keep = keep | d["relabeled"].astype(bool)
            print(f"  {p}: {len(keep)} 스텝 중 {int(keep.sum())} 사용"
                  + (" (완주 + 잡음 에피소드)" if success_only and "noisy" in d.files and d["noisy"].any() else " (완주 에피소드만)" if success_only and "ep_success" in d.files else ""))
            if p in frac and frac[p] < 1.0:
                e_all = d["episode"]
                ue = np.unique(e_all[keep])
                pick = ue[rng_s.random(len(ue)) < frac[p]]
                keep = keep & np.isin(e_all, pick)
                print(f"    → 에피소드 {len(pick)}/{len(ue)} 뽑음, {int(keep.sum())} 스텝")
            if not keep.any():
                continue
            obs.append(d["observation"][keep].astype(np.float32))
            act.append(d["action_raw"][keep].astype(np.float32))
            rew.append(d["reward"][keep].astype(np.float32))
            ep.append(d["episode"][keep].astype(np.int64) + base)
            base = int(ep[-1].max()) + 1
    return (np.concatenate(obs), np.concatenate(act), np.concatenate(rew), np.concatenate(ep))


def shape_hold(obs, act, hold_target=None, scope="all"):
    """정지 유지 정답 조정: 거의 멈춘 상태(속도 관측 < 0.01)에서 강사가 가감속 <= 0.005 (그대로 서 있기)를 냈으면
    정답을 min(원래 값, hold_target) 으로 바꾼다 (예: -0.1). 멈춘 상태의 음수 가감속은 속도가 0 아래로 못 내려가
    실제 행동은 같다. 모델 출력이 0 근처에서 흔들려 양수가 나와 정지선을 조금씩 밀고 넘어가던 문제 대응 (2026-11-16).
    act 를 복사해서 돌려준다."""
    act = act.copy()
    if hold_target is None:
        return act, 0
    m = (obs[:, 0] < 0.01) & (act[:, 0] <= 0.005)
    if scope == "red":   # 적·황신호 정지선 5 m 앞에서 멈춘 장면만 (신호 칸 k: [k] 있음, [k+1] 거리/200, [k+2] 적, [k+3] 황)
        from env.obs_layout import signal_index
        k = signal_index(obs.shape[1])
        m &= (obs[:, k] > 0) & ((obs[:, k + 2] > 0) | (obs[:, k + 3] > 0)) & (obs[:, k + 1] * 200 < 5)
    act[m, 0] = np.minimum(act[m, 0], hold_target)
    return act, int(m.sum())


def bc_targets(obs, act, brake_weight=3.0, signal_weight=3.0, lane_weight_cap=20.0):
    """BC 정답과 가중치.

    Returns: lane_idx (차선 정답 0/1/2), cls_w (차선 클래스 가중치 3개), acc_w (가감속 샘플 가중치),
             counts (차선 정답 개수), sig_near (신호 앞 장면 여부)
    """
    lane_idx = (1 - np.round(act[:, 1])).astype(np.int64).clip(0, 2)    # +1(왼쪽)→0, 0(유지)→1, -1(오른쪽)→2
    counts = np.bincount(lane_idx, minlength=3).astype(np.float64)
    cls_w = counts.sum() / (3 * np.maximum(counts, 1))
    cls_w = np.minimum(cls_w / cls_w[1], lane_weight_cap)                # "유지" = 1, 차선변경은 최대 cap 배
    # (예전에는 정규화 전에 상한을 걸어 실제로는 ~60배가 되어 차선변경을 너무 자주 하는 모델이 됨, 2026-11-16)
    acc_w = np.where(act[:, 0] < -0.3, brake_weight, 1.0).astype(np.float32)
    # 적·황신호가 40 m 안 (신호 칸 k: [k] 있음, [k+1] 거리/200, [k+2] 적, [k+3] 황 — env/obs_layout.py)
    from env.obs_layout import signal_index
    k = signal_index(obs.shape[1])
    sig_near = (obs[:, k] > 0) & ((obs[:, k + 2] > 0) | (obs[:, k + 3] > 0)) & (obs[:, k + 1] * 200 < 40)
    acc_w = acc_w * np.where(sig_near, signal_weight, 1.0).astype(np.float32)
    return lane_idx, cls_w, acc_w, counts, sig_near


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--init", default=None, help="이어서 학습할 PPO model.pt (DAgger 재학습)")
    ap.add_argument("--epochs", type=int, default=15)
    ap.add_argument("--batch-size", type=int, default=4096)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--val-fraction", type=float, default=0.05)
    ap.add_argument("--brake-weight", type=float, default=3.0, help="강한 제동(정답 가감속 < -0.3) 가중치")
    ap.add_argument("--lane-weight-cap", type=float, default=20.0, help="차선변경 클래스 가중치 상한")
    ap.add_argument("--signal-weight", type=float, default=3.0,
                    help="적·황신호 40 m 앞에서 다가가는 장면의 가감속 가중치 (정지선 앞 정밀 정지)")
    ap.add_argument("--value-coef", type=float, default=0.05)
    ap.add_argument("--hold-target", type=float, default=None,
                    help="정지 유지 정답 조정 (예: -0.1). 멈춘 상태에서 강사가 0 이하를 낸 장면의 정답을 이 값 이하로")
    ap.add_argument("--hold-scope", choices=["all", "red"], default="all",
                    help="정지 유지 조정 범위: all = 멈춘 장면 전부, red = 적·황신호 정지선 5 m 앞만")
    ap.add_argument("--hidden", type=int, nargs="+", default=None,
                    help="신경망 은닉층 크기 (예: 512 512 512). 생략하면 train.py HPARAMS")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-steps", type=int, default=int(os.environ.get("BC_MAX_STEPS", 14_000_000)),
                    help="학습 데이터 상한 (GPU 메모리). 0 이면 상한 없음")
    ap.add_argument("--all-episodes", action="store_true",
                    help="실패(충돌·막힘) 에피소드도 사용. 기본은 완주 에피소드만")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)

    t0 = time.time()
    obs, act, rew, ep = load(args.data, success_only=not args.all_episodes, max_steps=args.max_steps or None, seed=args.seed)
    act, n_hold = shape_hold(obs, act, args.hold_target, args.hold_scope)
    if args.hold_target is not None:
        print(f"정지 유지 정답 조정: {n_hold:,} 스텝 ({n_hold / len(act):.1%}) → 가감속 정답 <= {args.hold_target}")
    gamma = HPARAMS["gamma"]
    ret = discounted_returns(rew, ep, None, gamma)
    lane_idx, cls_w, acc_w, counts, sig_near = bc_targets(
        obs, act, args.brake_weight, args.signal_weight, args.lane_weight_cap)
    print(f"데이터 {len(obs):,} 스텝 ({time.time() - t0:.0f}s) | 차선 정답 왼/유지/오 = {counts.astype(int)} "
          f"→ 클래스 가중치 {cls_w.round(2)} | 강한 제동 {(act[:, 0] < -0.3).mean():.1%} | 신호 앞 {sig_near.mean():.1%}")

    # 에피소드 단위로 검증 세트 분리
    eps = np.unique(ep)
    val_eps = set(rng.choice(eps, size=max(1, int(len(eps) * args.val_fraction)), replace=False).tolist())
    is_val = np.isin(ep, list(val_eps))
    tr, va = np.where(~is_val)[0], np.where(is_val)[0]

    hs = tuple(args.hidden) if args.hidden else HPARAMS["hidden_sizes"]
    agent = PPO(obs_dim=obs.shape[1], act_dim=2, **{**HPARAMS, "device": "auto", "hidden_sizes": hs})
    print(f"신경망: 은닉층 {hs}, 별도 critic {HPARAMS.get('separate_critic', False)}")
    if args.init:
        agent.load(args.init)
    pol, dev = agent.policy, agent.device
    params = [p for n, p in pol.named_parameters() if n != "log_std"]
    opt = torch.optim.Adam(params, lr=args.lr)
    steps_total = args.epochs * int(np.ceil(len(tr) / args.batch_size))
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=steps_total)

    T = lambda x: torch.as_tensor(x, device=dev)                          # noqa: E731
    O, A, L, W, R = T(obs), T(act[:, 0]), T(lane_idx), T(acc_w), T(ret)
    CW = T(cls_w.astype(np.float32))
    ret_scale = float(np.std(ret) + 1e-6)

    def batch_loss(idx, train):
        mu, std, logits, v = pol(O[idx])
        la = (W[idx] * (mu.squeeze(-1) - A[idx]) ** 2).mean()
        ll = F.cross_entropy(logits, L[idx], weight=CW)
        lv = F.mse_loss(v / ret_scale, R[idx] / ret_scale)
        loss = la + ll + args.value_coef * lv
        with torch.no_grad():
            pred = logits.argmax(-1)
            lc = L[idx] != 1
            stats = dict(accel_mse=float(((mu.squeeze(-1) - A[idx]) ** 2).mean()),
                         lane_acc=float((pred == L[idx]).float().mean()),
                         lane_change_recall=float((pred[lc] == L[idx][lc]).float().mean()) if lc.any() else float("nan"),
                         value_mse=float(lv))
        return loss, stats

    history = []
    for epoch in range(1, args.epochs + 1):
        pol.train()
        perm = rng.permutation(tr)
        tl, n = 0.0, 0
        for s in range(0, len(perm), args.batch_size):
            idx = T(perm[s:s + args.batch_size])
            loss, _ = batch_loss(idx, True)
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(params, 1.0)
            opt.step()
            sched.step()
            tl += float(loss.detach()) * len(idx)
            n += len(idx)
        pol.eval()
        agg = {}
        with torch.no_grad():
            for s in range(0, len(va), 65536):
                idx = T(va[s:s + 65536])
                _, st = batch_loss(idx, False)
                for k, x in st.items():
                    agg.setdefault(k, []).append(x)
        val = {k: float(np.nanmean(v)) for k, v in agg.items()}
        history.append({"epoch": epoch, "train_loss": tl / n, **val})
        print(f"epoch {epoch:2d}/{args.epochs} | train_loss {tl / n:.4f} | 검증: 가감속 MSE {val['accel_mse']:.4f}, "
              f"차선 정확도 {val['lane_acc']:.3f}, 차선변경 재현율 {val['lane_change_recall']:.3f}, "
              f"가치 MSE {val['value_mse']:.3f} | {time.time() - t0:.0f}s", flush=True)

    agent.save(os.path.join(args.out, "model.pt"))
    with open(os.path.join(args.out, "pretrain_log.json"), "w") as f:
        json.dump({"args": vars(args), "class_weights": cls_w.tolist(), "history": history}, f,
                  ensure_ascii=False, indent=2)
    print("저장:", os.path.join(args.out, "model.pt"))


if __name__ == "__main__":
    main()
