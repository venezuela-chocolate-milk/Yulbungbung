"""평가 JSON 들 중 최고 모델 고르기: 완주율 ↑, 충돌률 ↓ 순. 사용: v5_pick.py a.eval.json b.eval.json ... → 'model.pt 완주 충돌' 출력"""
import json, sys
best = None
for f in sys.argv[1:]:
    try:
        d = json.load(open(f))
    except Exception:
        continue
    eps = d["episodes"] if isinstance(d, dict) else d
    n = len(eps)
    comp = sum(e["result"] == "arrived" for e in eps) / n
    coll = sum(e["result"] == "collided" for e in eps) / n
    key = (comp, -coll)
    if best is None or key > best[0]:
        best = (key, f.replace(".eval.json", "/model.pt"), comp, coll)
print(best[1], f"{best[2]:.3f}", f"{best[3]:.3f}")
