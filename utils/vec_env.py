"""병렬 환경: SUMO 학습 환경 N개를 각자 별도 프로세스(CPU 코어)에서 동시에 돌린다.

학습 시간의 대부분은 SUMO 시뮬레이션(CPU)이라, 환경을 여러 개 동시에 돌리면
같은 시간에 경험을 N배 가까이 모을 수 있다. 신경망 계산은 메인 프로세스에서
N개 관측을 한 번에 묶어 한다 (GPU 가 있으면 GPU, 없으면 CPU).

    venv = SubprocVecEnv([make_fn_0, make_fn_1, ...])
    obs = venv.reset()                       # (N, obs_dim)
    obs, rew, term, trunc, infos = venv.step(actions)   # actions: (N, act_dim)

에피소드가 끝난 환경은 그 자리에서 바로 다시 시작한다. 그때 돌려주는 obs 는
"새 에피소드의 첫 관측"이고, 끝난 에피소드의 마지막 관측은 infos[i]["final_obs"] 에 있다
(시간 초과로 잘린 에피소드의 가치 부트스트랩용).
"""
import multiprocessing as mp

import numpy as np


def _worker(remote, parent_remote, make_fn):
    parent_remote.close()
    env = make_fn()
    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "step":
                obs, rew, term, trunc, info = env.step(data)
                if term or trunc:
                    info = dict(info)
                    info["final_obs"] = obs
                    obs, _ = env.reset()
                remote.send((obs, rew, term, trunc, info))
            elif cmd == "reset":
                obs, _ = env.reset()
                remote.send(obs)
            elif cmd == "spaces":
                remote.send((env.observation_space, env.action_space))
            elif cmd == "call":
                name, args = data
                remote.send(getattr(env, name)(*args))
            elif cmd == "close":
                env.close()
                remote.close()
                break
    except KeyboardInterrupt:
        env.close()


class SubprocVecEnv:
    def __init__(self, make_fns):
        ctx = mp.get_context("spawn")       # 프로세스마다 깨끗한 상태에서 SUMO/TraCI 시작
        self.num_envs = len(make_fns)
        self.remotes, work_remotes = zip(*[ctx.Pipe() for _ in range(self.num_envs)])
        self.procs = []
        for wr, r, fn in zip(work_remotes, self.remotes, make_fns):
            p = ctx.Process(target=_worker, args=(wr, r, fn), daemon=True)
            p.start()
            self.procs.append(p)
            wr.close()
        self.remotes[0].send(("spaces", None))
        self.observation_space, self.action_space = self.remotes[0].recv()
        self.closed = False

    def reset(self):
        for r in self.remotes:
            r.send(("reset", None))
        return np.stack([r.recv() for r in self.remotes])

    def step(self, actions):
        for r, a in zip(self.remotes, actions):
            r.send(("step", a))
        res = [r.recv() for r in self.remotes]
        obs, rew, term, trunc, infos = zip(*res)
        return (np.stack(obs), np.asarray(rew, dtype=np.float32),
                np.asarray(term, dtype=bool), np.asarray(trunc, dtype=bool), list(infos))

    def call(self, name, *args):
        """모든 환경에서 메서드를 호출하고 결과 목록을 돌려준다 (예: set_mask)."""
        for r in self.remotes:
            r.send(("call", (name, args)))
        return [r.recv() for r in self.remotes]

    def close(self):
        if self.closed:
            return
        for r in self.remotes:
            try:
                r.send(("close", None))
            except (BrokenPipeError, EOFError):
                pass
        for p in self.procs:
            p.join(timeout=30)
            if p.is_alive():
                p.terminate()
        self.closed = True
