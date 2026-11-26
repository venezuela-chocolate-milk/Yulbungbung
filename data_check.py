"""수집 데이터 품질 점검 (2026-11-24). 사용: python data_check.py a.npz [b.npz ...]
판 수·행 수·완주율, 관측 길이 163, NaN/Inf, 관측 값 범위, 행동 범위·분포, 차선변경 비율, 판당 길이, 같은 판 중복 행."""
import sys, numpy as np
for f in sys.argv[1:]:
    d = np.load(f, allow_pickle=True)
    o, a, ep = d["observation"], d["action_raw"], d["episode"]
    eps = np.unique(ep); n = len(o)
    succ = d["ep_success"] if "ep_success" in d.files else None
    s_ep = {e: bool(succ[ep == e][0]) for e in eps} if succ is not None else {}
    lens = np.bincount(ep - ep.min())[np.bincount(ep - ep.min()) > 0]
    bad = []
    if o.shape[1] != 163: bad.append(f"관측 길이 {o.shape[1]}")
    nanc = int((~np.isfinite(o)).sum() + (~np.isfinite(a)).sum())
    if nanc: bad.append(f"NaN/Inf {nanc}")
    if np.abs(o).max() > 50: bad.append(f"관측 최대 {np.abs(o).max():.1f}")
    if np.abs(a).max() > 1.0001: bad.append(f"행동 범위 {np.abs(a).max():.3f}")
    key = np.concatenate([ep[:, None].astype(np.float32), d["t"][:, None].astype(np.float32)], 1)
    dup = n - len(np.unique(key, axis=0))
    if dup: bad.append(f"중복 행 {dup}")
    comp = np.mean(list(s_ep.values())) if s_ep else float("nan")
    lc = d["lane_change"]
    print(f"== {f}\n  판 {len(eps)}  행 {n:,}  완주 {comp:.1%}  판당 길이 평균 {lens.mean():.0f} (최소 {lens.min()}, 최대 {lens.max()})")
    print(f"  가감속: 평균 {a[:,0].mean():+.3f}  표준편차 {a[:,0].std():.3f}  |a|>0.9 비율 {np.mean(np.abs(a[:,0])>0.9):.1%}")
    print(f"  차선변경 명령: 왼쪽 {np.mean(lc<0):.2%}  유지 {np.mean(lc==0):.2%}  오른쪽 {np.mean(lc>0):.2%}")
    for k in ("noisy", "relabeled"):
        if k in d.files: print(f"  {k}: {d[k].mean():.1%}")
    print("  " + ("품질 OK" if not bad else "문제: " + ", ".join(bad)))
