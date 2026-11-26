"""트랙(B/C) 관측 점검: 시작 전에 반드시 통과해야 함. 사용: python check_track.py <B|C> [판 수=2] [스텝=800]
검사 항목
 1. 관측 길이 (B 123, C 163) 와 값이 유한한지, 범위 [-1, 1] 근처인지
 2. 같은 차가 관측 두 곳에 들어간 경우 = 0  (B: 8칸 ∩ 교차 목록, C: 물체 목록 안 중복)
 3. 안 보이는 차가 관측에 들어간 경우 = 0  (50 m 안: 건물·차 가림 / 먼 교차 차: 건물 가림)
 4. 안전 마스킹 꺼짐, 관측 코드에 강사 전용 정보(남은 신호 시간·상대 가속) 없음
"""
import sys, os, math, inspect
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

def main():
    T = sys.argv[1]; n_ep = int(sys.argv[2]) if len(sys.argv) > 2 else 2; n_st = int(sys.argv[3]) if len(sys.argv) > 3 else 800
    want = {"B": 123, "C": 163}[T]
    from env.maps import make_env
    from env import urban_driving as ud
    from env import roundabout_env_config as C
    from train import REWARD
    from env.expert_driver import RuleBasedExpert
    import traci
    ok = True
    def chk(name, cond, ev=""):
        nonlocal ok
        ok &= bool(cond)
        print(("✅" if cond else "❌"), name, ev, flush=True)
    chk("안전 마스킹 꺼짐", C.URBAN.get("mask_enabled") is False)
    src = inspect.getsource(ud.UrbanDriving.features) + inspect.getsource(ud.UrbanDriving._sectors) + \
          (inspect.getsource(ud.UrbanDriving._objects) if hasattr(ud.UrbanDriving, "_objects") else "")
    chk("관측 코드에 남은 신호 시간 없음", "getNextSwitch" not in src)
    chk("관측 코드에 상대 가속 없음", "getAcceleration" not in src)
    env, _ = make_env("roundabout", reward=REWARD, build=False, ev_mode="train")
    chk(f"관측 길이 {want}", env.observation_space.shape[0] == want, f"(환경 {env.observation_space.shape[0]})")
    dup = hid = steps = 0; vmax_abs = 0.0; nonfinite = 0; seen_obj = 0
    for ep in range(n_ep):
        np.random.seed(7000 + ep)
        obs, _ = env.reset(options={"route": "mom"})
        ex = RuleBasedExpert(env); ex.reset()
        for t in range(n_st):
            u = env.urban
            nonfinite += int(not np.all(np.isfinite(obs)))
            vmax_abs = max(vmax_abs, float(np.max(np.abs(obs))))
            ego = env.ego_id
            if T == "B":
                cells = set(u.last.get("sectors", {}).values())
                foes = [f["id"] for f in u.last.get("foes", [])]
                dup += len(cells & set(foes))
                ids = list(cells) + foes
            else:
                ids = list(getattr(u, "obj_ids", []))
                dup += len(ids) - len(set(ids))
            seen_obj += len(ids)
            for vid in ids:
                if vid in u._ctx_ids:
                    hid += int(vid not in u.visible)
                else:
                    hid += int(not u._visible(vid))
            steps += 1
            obs, r, te, tr, inf = env.step(ex.act())
            if te or tr:
                break
    env.close()
    chk("값이 모두 유한", nonfinite == 0, f"(문제 스텝 {nonfinite})")
    chk("값 범위", vmax_abs <= 1.5, f"(최대 |값| {vmax_abs:.2f})")
    chk("같은 차 두 곳 = 0", dup == 0, f"({dup}건 / {steps}스텝, 관측에 들어간 차 {seen_obj}대·스텝)")
    chk("안 보이는 차 관측 = 0", hid == 0, f"({hid}건)")
    chk("관측에 차가 실제로 들어감", seen_obj > 0)
    print("ALL OK" if ok else "SOME FAILED")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
