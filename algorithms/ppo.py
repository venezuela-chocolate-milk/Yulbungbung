"""PPO(Proximal Policy Optimization) — 알고리즘 본체.

이 파일에는 "알고리즘"만 있다:
    네트워크 구조  → utils/networks.py
    경험 버퍼      → utils/buffer.py   (RolloutBuffer — 메인에서 주입받음)
    정책 평가      → utils/evaluator.py
    로깅          → utils/logger.py
덕분에 아래 코드는 PPO 수식과 1:1로 대응해서 읽을 수 있다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PPO 요약 (Schulman et al., 2017)

policy gradient의 기본:
    advantage A가 양수인 행동의 확률은 높이고, 음수면 낮춘다.

문제: 한 번의 업데이트로 정책이 너무 크게 변하면 학습이 붕괴한다.
해법: 새/옛 정책의 확률비를 클리핑해서 과도한 업데이트의 이득을 제거.

    r(θ) = π_θ(a|s) / π_θ_old(a|s)

    L^CLIP(θ) = E[ min( r(θ)·A ,  clip(r(θ), 1-ε, 1+ε)·A ) ]

전체 손실:
    L = -L^CLIP  +  c_v · L^VF  -  c_e · H[π]  (+ λ_BC · L^BC)
        ────────     ─────────      ───────       ────────────
        정책 개선     가치 회귀      탐험 유지     전문가 따라 하기 (선택)
    (L^VF = (V(s) - R_target)², H = 정책 엔트로피)

BC 손실 (set_bc_data 로 전문가 데이터를 주었을 때만, hybrid 정책 전용):
    L^BC = 가중 MSE(가감속 μ, 전문가 가감속) + 가중 CE(차선 logits, 전문가 차선)
    모방학습(pretrain_ppo_bc.py)과 같은 식. 미니배치마다 전문가 데이터에서 따로 뽑아 계산한다.
    λ_BC 는 처음 bc_coef 에서 bc_decay_steps 동안 bc_coef_final 까지 직선으로 줄인다.
    → 이어서 학습할 때 정책이 모방학습으로 얻은 행동에서 멀리 벗어나 무너지는 것을 막고,
      학습이 진행될수록 강화학습이 전문가보다 나은 행동을 찾을 여지를 준다.

advantage는 GAE(λ)로 추정:
    δ_t = r_t + γ·V(s_{t+1})·(1-done) - V(s_t)
    A_t = δ_t + (γλ)·(1-done)·A_{t+1}          ← 뒤에서 앞으로 재귀
    R_t = A_t + V(s_t)                          ← critic의 회귀 타깃
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

학습 루프:
    반복 {
      1) rollout 수집   : 현재 정책으로 n_steps만큼 환경과 상호작용
      2) advantage 계산 : GAE(λ)
      3) 정책 업데이트  : 같은 데이터로 n_epochs번 미니배치 경사하강
      4) (주기적) 평가  : 노이즈 없는 정책으로 주행 지표 측정
    }
"""
from collections import deque

import os

import numpy as np
import torch
import torch.nn as nn

from utils.networks import GaussianActorCritic, HybridActorCritic
from utils.evaluator import evaluate_policy
from utils.buffer import RolloutBuffer


# ======================================================================
def hidden_sizes_from_checkpoint(path):
    """저장된 PPO 모델(state_dict)의 몸통 은닉층 크기를 읽는다 (예: (256, 256)). 없으면 None.
    신경망 크기를 바꿔 학습한 모델도 HPARAMS 를 고치지 않고 불러오기 위함 (2026-11-16)."""
    state = torch.load(path, map_location="cpu", weights_only=True)
    if not isinstance(state, dict):
        return None
    sizes = []
    i = 0
    while f"trunk.{i}.weight" in state or any(k.startswith(f"trunk.{j}.") for j in range(i, i + 3) for k in state):
        if f"trunk.{i}.weight" in state:
            sizes.append(int(state[f"trunk.{i}.weight"].shape[0]))
        i += 1
        if i > 50:
            break
    return tuple(sizes) or None


class PPO:
    """PPO 에이전트.

    하이퍼파라미터는 생성자 인자로 명시적으로 받는다
    (train.py에서 한눈에 보고 바꿀 수 있도록).
    """

    def __init__(
        self,
        obs_dim: int,
        act_dim: int,
        # ---- 최적화 ----
        lr: float = 3e-4,
        n_steps: int = 2048,          # rollout 길이 (업데이트 1회당 수집량)
        n_epochs: int = 10,           # 같은 데이터 재사용 횟수
        minibatch_size: int = 64,
        # ---- 할인 / advantage ----
        gamma: float = 0.99,          # 할인율
        gae_lambda: float = 0.95,     # GAE λ
        # ---- PPO 고유 ----
        clip_eps: float = 0.2,        # 클리핑 범위 ε
        value_coef: float = 0.5,      # 가치 손실 가중치 c_v
        entropy_coef: float = 0.0,    # 엔트로피 보너스 c_e (전체 행동)
        lane_entropy_coef: float = 0.0,  # hybrid 전용: "차선 head만"의
                                         # 엔트로피 보너스. 차선변경 확률이
                                         # 초기에 0으로 붕괴하는 것을 늦춘다.
                                         # (가감속 σ에는 영향 없음)
        max_grad_norm: float = 0.5,   # gradient clipping
        normalize_advantage: bool = True,
        target_kl: float = None,      # 주면: 한 에폭의 평균 approx_kl 이 1.5 × target_kl 을 넘으면
                                      #   그 업데이트의 남은 에폭을 건너뜀 (정책이 한 번에 너무 바뀌는 것 방지)
        # ---- 네트워크 ----
        policy_type: str = "hybrid",  # "hybrid"(가감속 연속 + 차선 이산, 권장)
                                      # | "gaussian"(모든 축 연속 — dead zone 있음)
        lane_keep_bias: float = 1.0,  # hybrid 전용: 차선 "유지" 초기 편향
        separate_critic: bool = False,  # hybrid 전용: 가치 함수를 별도 신경망으로 (몸통 공유 안 함)
        critic_warmup_updates: int = 0,  # 처음 이 횟수의 업데이트는 가치 함수만 학습 (정책 고정)
        hidden_sizes=(64, 64),
        activation: str = "tanh",
        init_log_std=-0.5,            # 초기 탐험 폭 σ = exp(·). 스칼라 또는
                                      # 차원별 튜플 (utils/networks.py 참고)
        # ---- 기타 ----
        seed: int = 0,
        device: str = "auto",
        buffer: RolloutBuffer = None,   # 경험 버퍼 (메인에서 주입).
                                        # None이면 내부에서 기본 생성.
    ):
        torch.manual_seed(seed)
        np.random.seed(seed)

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)

        # 네트워크는 utils/networks.py 에서 가져온다.
        # hybrid: 가감속(Normal) + 차선변경(Categorical 3-way).
        #   양자화 dead zone이 없어 deterministic 평가에서도 변경이 나온다.
        # gaussian: 모든 축 연속 — 비교 실험용으로 남겨둠.
        if policy_type == "hybrid":
            assert act_dim == 2, "hybrid 정책은 [가감속, 차선] 2차원 행동 전용"
            self.policy = HybridActorCritic(
                obs_dim,
                hidden_sizes=hidden_sizes,
                activation=activation,
                init_log_std=init_log_std,
                lane_keep_bias=lane_keep_bias,
                separate_critic=separate_critic,
            ).to(self.device)
        else:
            self.policy = GaussianActorCritic(
                obs_dim, act_dim,
                hidden_sizes=hidden_sizes,
                activation=activation,
                init_log_std=init_log_std,
            ).to(self.device)
        self.policy_type = policy_type
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr)

        self.n_steps = n_steps
        self.n_epochs = n_epochs
        self.minibatch_size = minibatch_size
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_eps = clip_eps
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.lane_entropy_coef = lane_entropy_coef
        self.max_grad_norm = max_grad_norm
        self.normalize_advantage = normalize_advantage
        self.target_kl = target_kl
        self.critic_warmup_updates = int(critic_warmup_updates)
        self.n_updates_done = 0

        # 버퍼 주입(dependency injection):
        # train.py(메인)가 utils.buffer.RolloutBuffer를 만들어 넘겨준다.
        # 어떤 버퍼를 쓰는지가 메인에서 보이고, 나중에 다른 버퍼로 교체할 때
        # 이 파일을 수정할 필요가 없다.
        self.buffer = buffer if buffer is not None else RolloutBuffer()
        self.num_timesteps = 0
        self.bc = None          # set_bc_data 로 설정 (없으면 BC 손실 없음)

    # ==================================================================
    # (선택) 강화학습 중 BC 손실
    # ==================================================================
    def set_bc_data(self, obs, accel, lane_idx, acc_w, cls_w, coef=0.5, coef_final=0.0,
                    decay_steps=2_000_000, batch_size=256):
        """전문가 데이터와 BC 손실 설정. 정답·가중치는 pretrain_ppo_bc.bc_targets 로 만든다.

        obs (N, obs_dim), accel (N,) 전문가 가감속, lane_idx (N,) 0=왼/1=유지/2=오,
        acc_w (N,) 가감속 샘플 가중치, cls_w (3,) 차선 클래스 가중치.
        coef → coef_final 을 decay_steps 동안 직선으로 (decay_steps=0 이면 coef 고정).
        batch_size: PPO 미니배치 1개마다 전문가 데이터에서 뽑는 개수.
        """
        assert hasattr(self.policy, "det_action"), "BC 손실은 hybrid 정책 전용"
        T = lambda x, dt: torch.as_tensor(np.asarray(x), dtype=dt, device=self.device)  # noqa: E731
        self.bc = dict(obs=T(obs, torch.float32), accel=T(accel, torch.float32),
                       lane=T(lane_idx, torch.int64), acc_w=T(acc_w, torch.float32),
                       cls_w=T(cls_w, torch.float32), coef=float(coef),
                       coef_final=float(coef_final), decay_steps=int(decay_steps),
                       batch_size=int(batch_size), n=len(obs), start_step=None)

    def bc_coef_now(self) -> float:
        """현재 BC 손실 가중치 λ_BC (설정 안 했으면 0). 처음 update 한 스텝부터 줄이기 시작."""
        if self.bc is None:
            return 0.0
        b = self.bc
        if b["start_step"] is None:
            b["start_step"] = self.num_timesteps
        if b["decay_steps"] <= 0:
            return b["coef"]
        frac = min(max((self.num_timesteps - b["start_step"]) / b["decay_steps"], 0.0), 1.0)
        return b["coef"] + (b["coef_final"] - b["coef"]) * frac

    def _bc_loss(self):
        """전문가 데이터 미니배치의 BC 손실 (pretrain_ppo_bc.py 의 가감속·차선 손실과 같은 식)."""
        b = self.bc
        idx = torch.randint(0, b["n"], (b["batch_size"],), device=self.device)
        mu, _, logits, _ = self.policy(b["obs"][idx])
        la = (b["acc_w"][idx] * (mu.squeeze(-1) - b["accel"][idx]) ** 2).mean()
        ll = nn.functional.cross_entropy(logits, b["lane"][idx], weight=b["cls_w"])
        return la + ll

    # ==================================================================
    # 행동 선택
    # ==================================================================
    @torch.no_grad()
    def act(self, obs):
        """학습용 확률적 행동. Returns (action, log_prob, value).

        hybrid: 가감속은 Normal 샘플, 차선은 Categorical 샘플(-1/0/+1).
        gaussian: 기존과 동일한 전차원 Normal 샘플.
        """
        obs_t = torch.as_tensor(obs, dtype=torch.float32,
                                device=self.device).unsqueeze(0)
        if hasattr(self.policy, "sample"):          # hybrid 경로
            action, log_prob, value = self.policy.sample(obs_t)
            return (action.squeeze(0).cpu().numpy().astype(np.float32),
                    float(log_prob.item()), float(value.item()))
        mu, std, value = self.policy(obs_t)          # gaussian 경로
        dist = torch.distributions.Normal(mu, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(-1)
        return (action.squeeze(0).cpu().numpy().astype(np.float32),
                float(log_prob.item()), float(value.item()))

    @torch.no_grad()
    def predict(self, obs, deterministic: bool = True):
        """평가/실행용. hybrid의 deterministic = (μ, argmax 차선).

        utils/evaluator.py 가 요구하는 인터페이스이기도 하다.
        """
        obs_t = torch.as_tensor(obs, dtype=torch.float32,
                                device=self.device).unsqueeze(0)
        if hasattr(self.policy, "det_action"):       # hybrid 경로
            if deterministic:
                a = self.policy.det_action(obs_t)
            else:
                a, _, _ = self.policy.sample(obs_t)
            return a.squeeze(0).cpu().numpy().astype(np.float32)
        mu, std, _ = self.policy(obs_t)              # gaussian 경로
        if deterministic:
            return mu.squeeze(0).cpu().numpy().astype(np.float32)
        return (torch.normal(mu, std).squeeze(0)
                .cpu().numpy().astype(np.float32))

    @torch.no_grad()
    def act_batch(self, obs):
        """병렬 환경용: 관측 N개를 한 번에. Returns (actions (N,A), log_probs (N,), values (N,))."""
        obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=self.device)
        if hasattr(self.policy, "sample"):
            action, log_prob, value = self.policy.sample(obs_t)
        else:
            mu, std, value = self.policy(obs_t)
            dist = torch.distributions.Normal(mu, std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(-1)
        return (action.cpu().numpy().astype(np.float32),
                log_prob.reshape(-1).cpu().numpy().astype(np.float32),
                value.reshape(-1).cpu().numpy().astype(np.float32))

    @torch.no_grad()
    def get_values(self, obs):
        obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=self.device)
        return self.policy(obs_t)[-1].reshape(-1).cpu().numpy().astype(np.float32)

    @torch.no_grad()
    def get_value(self, obs) -> float:
        obs_t = torch.as_tensor(obs, dtype=torch.float32,
                                device=self.device).unsqueeze(0)
        value = self.policy(obs_t)[-1]   # 마지막 원소 = V(s) (정책 종류 무관)
        return float(value.item())

    # ==================================================================
    # 1단계) rollout 수집 — 병렬 환경 (utils/vec_env.py)
    # ==================================================================
    def collect_rollout_vec(self, venv, obs, logger=None, ep_stats=None):
        """N개 환경에서 환경당 n_steps 씩 동시에 모은다 (총 n_steps × N).

        에피소드가 끝난 환경은 venv 가 바로 다시 시작하고, 끝난 에피소드의 마지막
        관측은 info["final_obs"] 에 있다. 시간 초과로 잘린 경우 그 관측의 가치로
        부트스트랩한다 (1개 환경일 때와 같은 처리).
        """
        n = venv.num_envs
        finished_returns = []
        self.buffer.clear()
        for _ in range(self.n_steps):
            actions, log_probs, values = self.act_batch(obs)
            next_obs, rewards, terms, truncs, infos = venv.step(actions)
            self.num_timesteps += n
            stored = rewards.copy()
            cut = [i for i in range(n) if truncs[i] and not terms[i]]
            if cut:
                v_final = self.get_values(np.stack([infos[i]["final_obs"] for i in cut]))
                for i, v in zip(cut, v_final):
                    stored[i] += self.gamma * v
            self.buffer.add(obs, actions, log_probs, stored, values, terms)
            ep_stats[0] += rewards
            ep_stats[1] += 1
            for i in range(n):
                if terms[i] or truncs[i]:
                    finished_returns.append(float(ep_stats[0][i]))
                    if logger is not None:
                        logger.log_episode(self.num_timesteps, float(ep_stats[0][i]), int(ep_stats[1][i]))
                    ep_stats[0][i], ep_stats[1][i] = 0.0, 0
                    if infos[i].get("episode_stats"):
                        self._pending_ep_stats.append(infos[i]["episode_stats"])
            obs = next_obs
        return obs, self.get_values(obs), finished_returns

    # ==================================================================
    # 1단계) rollout 수집
    # ==================================================================
    def collect_rollout(self, env, state, logger=None, ep_stats=None):
        """현재 정책으로 n_steps만큼 경험을 모은다.

        Args:
            state: 직전 rollout이 끝난 시점의 관측 (에피소드를 이어서 수집)
            ep_stats: [ep_return, ep_length] 리스트 (호출 간 누적 유지)
        Returns:
            (last_obs, last_value, finished_returns)
        """
        obs = state
        finished_returns = []
        self.buffer.clear()

        for _ in range(self.n_steps):
            action, log_prob, value = self.act(obs)
            next_obs, reward, terminated, truncated, _ = env.step(action)

            self.num_timesteps += 1
            ep_stats[0] += reward
            ep_stats[1] += 1

            # ★ truncation(시간 초과) 보정:
            #   MDP가 끝난 게 아니라 시간 제한으로 잘린 것이므로,
            #   이어졌다면 받았을 미래 가치를 V(s')로 근사해 보상에 더한다.
            #   생략하면 "시간초과 = 이후 가치 0"으로 잘못 학습된다.
            stored_reward = reward
            if truncated and not terminated:
                stored_reward += self.gamma * self.get_value(next_obs)

            self.buffer.add(obs, action, log_prob, stored_reward, value,
                            terminated)

            if terminated or truncated:
                finished_returns.append(ep_stats[0])
                if logger is not None:
                    logger.log_episode(self.num_timesteps, ep_stats[0],
                                       int(ep_stats[1]))
                ep_stats[0], ep_stats[1] = 0.0, 0
                obs, _ = env.reset()
            else:
                obs = next_obs

        # rollout이 에피소드 중간에 끊겼다면 이후 가치를 부트스트랩
        return obs, self.get_value(obs), finished_returns

    # ==================================================================
    # 3단계) 정책 업데이트 (clipped surrogate)
    # ==================================================================
    def update(self, advantages, returns) -> dict:
        obs_t, act_t, old_log_prob_t = self.buffer.to_tensors(self.device)
        adv_t = torch.as_tensor(advantages, device=self.device)
        ret_t = torch.as_tensor(returns, device=self.device)

        # advantage 정규화: 스케일을 평균0/표준편차1로 → 학습률 민감도 감소
        if self.normalize_advantage:
            adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)

        T = len(self.buffer)
        indices = np.arange(T)
        stats = {"policy_loss": 0.0, "value_loss": 0.0,
                 "entropy": 0.0, "approx_kl": 0.0, "clip_frac": 0.0}
        # critic 워밍업 중에는 가치 손실만 (정책 고정). separate_critic 과 함께 써야 정책이 정말 고정된다.
        warmup = self.n_updates_done < self.critic_warmup_updates
        bc_coef = 0.0 if warmup else self.bc_coef_now()   # BC 가중치 감소는 워밍업이 끝난 뒤부터
        if bc_coef > 0:
            stats["bc_loss"] = 0.0
        n_batches = 0

        epochs_run = 0
        for _ in range(self.n_epochs):
            np.random.shuffle(indices)
            epoch_kl, epoch_n = 0.0, 0
            epochs_run += 1
            for start in range(0, T, self.minibatch_size):
                mb = indices[start:start + self.minibatch_size]

                # 현재 정책으로 "그때 그 행동"을 다시 평가
                log_prob, entropy, value = self.policy.evaluate_actions(
                    obs_t[mb], act_t[mb])

                # 확률비 r(θ) = exp(log π_new - log π_old)
                ratio = torch.exp(log_prob - old_log_prob_t[mb])

                # ---- PPO 핵심: clipped surrogate objective ----
                surrogate_1 = ratio * adv_t[mb]
                surrogate_2 = torch.clamp(ratio, 1 - self.clip_eps,
                                          1 + self.clip_eps) * adv_t[mb]
                policy_loss = -torch.min(surrogate_1, surrogate_2).mean()

                # ---- critic: V(s)를 GAE 타깃에 회귀 ----
                value_loss = ((value - ret_t[mb]) ** 2).mean()

                # ---- 엔트로피 보너스: 탐험 유지 ----
                entropy_loss = -entropy.mean()

                if warmup:
                    # critic 워밍업: 가치 손실만 (정책 쪽 파라미터에는 기울기 없음 → 정책 고정)
                    loss = self.value_coef * value_loss
                else:
                    loss = (policy_loss
                            + self.value_coef * value_loss
                            + self.entropy_coef * entropy_loss)
                # 차선 head 전용 엔트로피 보너스 (hybrid에서만 동작)
                if not warmup and self.lane_entropy_coef > 0 and \
                        hasattr(self.policy, "lane_entropy"):
                    loss = loss - (self.lane_entropy_coef
                                   * self.policy.lane_entropy(obs_t[mb]).mean())
                # 전문가 따라 하기 (set_bc_data 를 했을 때만)
                if bc_coef > 0 and not warmup:
                    bc_loss = self._bc_loss()
                    loss = loss + bc_coef * bc_loss
                    stats["bc_loss"] += bc_loss.item()

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(),
                                         self.max_grad_norm)
                self.optimizer.step()

                # 진단 지표
                with torch.no_grad():
                    # approx_kl: 정책이 얼마나 변했는지. 0.02를 크게 넘으면
                    #   업데이트가 과격하다는 신호 (lr을 낮출 것)
                    approx_kl = (old_log_prob_t[mb] - log_prob).mean()
                    # clip_frac: 클리핑에 걸린 샘플 비율. 0.1~0.3이 보통
                    clip_frac = ((ratio - 1).abs() > self.clip_eps).float().mean()

                stats["policy_loss"] += policy_loss.item()
                stats["value_loss"] += value_loss.item()
                stats["entropy"] += entropy.mean().item()
                stats["approx_kl"] += approx_kl.item()
                stats["clip_frac"] += clip_frac.item()
                n_batches += 1
                epoch_kl += approx_kl.item()
                epoch_n += 1
            # KL 조기 종료 (target_kl 을 준 경우만)
            if self.target_kl is not None and epoch_kl / max(epoch_n, 1) > 1.5 * self.target_kl:
                break

        out = {k: v / max(n_batches, 1) for k, v in stats.items()}
        out["epochs_run"] = epochs_run
        self.n_updates_done += 1
        if self.critic_warmup_updates:
            out["critic_warmup"] = int(warmup)
        if self.bc is not None:
            out["bc_coef"] = bc_coef
        # hybrid 진단: 업데이트 "후" 정책의 평균 차선변경(좌+우) 확률.
        #   0으로 붕괴 → 차선유지 지역최적 경고 / 적당히 유지되며
        #   eval의 차선변경 횟수가 오르면 제대로 배우는 중.
        if hasattr(self.policy, "lane_change_prob"):
            with torch.no_grad():
                out["p_lane_change"] = float(
                    self.policy.lane_change_prob(obs_t).mean().item())
        return out

    # ==================================================================
    # 메인 학습 루프
    # ==================================================================
    def learn(self, env, total_timesteps: int, logger=None,
              log_interval: int = 1, eval_interval: int = 0,
              eval_episodes: int = 5, eval_env=None,
              checkpoint_dir=None, checkpoint_every: int = 0):
        """학습 실행.

        Args:
            logger: utils.logger.RunLogger (없으면 콘솔 출력만)
            eval_interval: 몇 번의 업데이트마다 평가할지 (0이면 평가 안 함)
            eval_env: 평가에 쓸 별도 환경. 병렬 환경(env.num_envs)으로 학습할 때 필요
                      (없으면 학습 환경으로 평가, 병렬이면 평가 생략)
            checkpoint_dir / checkpoint_every: 이 스텝마다 model_<스텝>.pt 중간 저장.
                      평가할 때마다 가장 좋은 모델은 model_best.pt 로 저장 (완주율 ↑, 충돌률 ↓, 보상 ↑ 순)
        """
        vec = hasattr(env, "num_envs")
        self._pending_ep_stats = []
        if vec:
            obs = env.reset()
            ep_stats = [np.zeros(env.num_envs), np.zeros(env.num_envs, dtype=int)]
        else:
            obs, _ = env.reset()
            ep_stats = [0.0, 0]                # [현재 에피소드 보상, 길이]
        eval_target = eval_env if eval_env is not None else (None if vec else env)
        recent_returns = deque(maxlen=20)
        n_updates = 0
        next_ckpt = checkpoint_every
        best_key = None

        while self.num_timesteps < total_timesteps:
            # 1) rollout
            if vec:
                obs, last_value, finished = self.collect_rollout_vec(
                    env, obs, logger=logger, ep_stats=ep_stats)
            else:
                obs, last_value, finished = self.collect_rollout(
                    env, obs, logger=logger, ep_stats=ep_stats)
            recent_returns.extend(finished)

            # 2) advantage
            advantages, returns = self.buffer.compute_gae(
                last_value, self.gamma, self.gae_lambda)

            # 3) update
            stats = self.update(advantages, returns)
            n_updates += 1

            # 로깅
            metrics = {
                "update": n_updates,
                "ep_rew_mean": float(np.mean(recent_returns))
                               if recent_returns else float("nan"),
                "std": self.policy.current_std,
                **stats,
            }
            # 도심 경로 주행 환경이면: 이번 rollout 동안 끝난 에피소드의
            # 안전 마스킹 개입률(학습될수록 줄어야 함)과 결과 비율을 함께 기록
            if vec:
                eps, self._pending_ep_stats = self._pending_ep_stats, []
            else:
                drain = getattr(env, "drain_episode_stats", None)
                eps = drain() if drain else []
            if eps:
                tot = max(sum(e["steps"] for e in eps), 1)
                metrics["intervention_rate"] = sum(e["intervention_rate"] * e["steps"] for e in eps) / tot
                for k in ("red_light", "yield", "rear_end", "junction_clear"):
                    metrics[f"int_{k}_rate"] = sum(e[f"int_{k}_rate"] * e["steps"] for e in eps) / tot
                metrics["would_intervene_rate"] = sum(e["would_intervene_rate"] * e["steps"] for e in eps) / tot
                metrics["train_red_violation"] = float(np.mean([e.get("red_violation_end", False) for e in eps]))
                metrics["train_success"] = float(np.mean([e["arrived"] for e in eps]))
                metrics["train_collision"] = float(np.mean([e["collided"] for e in eps]))
                metrics["train_lost"] = float(np.mean([e["lost"] for e in eps]))
                metrics["traffic_scale"] = float(eps[-1].get("traffic_scale") or 0.0)
                if "parked" in eps[-1]:                 # 엄마 경로: 주차장 정차 수·긴급차 길막기·단속 카메라
                    metrics["train_parked"] = float(np.mean([e["parked"] for e in eps]))
                    metrics["train_missed_stop"] = float(np.mean([e["missed_stop"] for e in eps]))
                    metrics["train_ev_block_steps"] = float(np.mean([e["ev_block_steps"] for e in eps]))
                    metrics["train_camera_tickets"] = float(np.mean([e["camera_tickets"] for e in eps]))
            if logger is not None:
                logger.log_train(self.num_timesteps, metrics,
                                 verbose=(n_updates % log_interval == 0))

            # 4) 주기적 평가 (utils/evaluator.py 사용)
            # 중간 저장 (긴 학습이 중간에 멈춰도 결과가 남도록)
            if checkpoint_dir and checkpoint_every > 0 and self.num_timesteps >= next_ckpt:
                self.save(os.path.join(checkpoint_dir, f"model_{self.num_timesteps}.pt"))
                if logger is not None:
                    logger.save_csv()
                next_ckpt += checkpoint_every

            if eval_interval > 0 and n_updates % eval_interval == 0 and eval_target is not None:
                m = evaluate_policy(self, eval_target, n_episodes=eval_episodes)
                if logger is not None:
                    logger.log_eval(self.num_timesteps, m.as_dict(),
                                    m.summary(), eval_episodes)
                key = (m.success_rate, -m.collision_rate, m.ep_return)
                if checkpoint_dir and (best_key is None or key > best_key):
                    best_key = key
                    self.save(os.path.join(checkpoint_dir, "model_best.pt"))
                    print(f"  ★ 최고 모델 갱신 (스텝 {self.num_timesteps}): 완주 {m.success_rate:.0%}, "
                          f"충돌 {m.collision_rate:.0%}, 보상 {m.ep_return:.1f}", flush=True)
                if eval_target is env:
                    # 평가가 학습 중이던 에피소드를 끊었으므로 리셋.
                    # (끊긴 경험은 이미 버퍼에 반영됐고 GAE가 부트스트랩하므로 무해)
                    obs, _ = env.reset()
                    ep_stats[0], ep_stats[1] = 0.0, 0

    # ==================================================================
    def save(self, path: str):
        torch.save(self.policy.state_dict(), path)

    def load(self, path: str):
        state = torch.load(path, map_location=self.device)
        missing, unexpected = self.policy.load_state_dict(state, strict=False)
        # 별도 critic 이 없는 예전 모델(공유 몸통 v_head)을 불러오면 critic.* 만 비어 있다 → 새로 초기화된 채로 사용
        bad = [k for k in missing if not k.startswith("critic.")] + list(unexpected)
        if bad:
            raise RuntimeError(f"모델 구조가 다릅니다: missing/unexpected = {bad[:8]}")
        if missing:
            print(f"  (불러온 모델에 별도 critic 이 없어 새로 초기화: {len(missing)}개 텐서)")
