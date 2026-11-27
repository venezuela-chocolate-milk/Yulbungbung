#!/bin/bash
# 11-26 강건성: 최종 모델 E200k 만, 처음 보는 seed × 1000판, 긴급차 확률 0.5, 판마다 seed+판번호 고정
# 사용: bash scripts/robust_N.sh 작업자수 seed...   (예: bash scripts/robust_N.sh 8 100000 200000)
cd "$(dirname "$(readlink -f "$0")")/.."   # 저장소 맨 위 폴더
M=models/final_rl/model.pt; W=$1; shift; mkdir -p results/robust_N
[ "$(sha256sum $M | cut -c1-16)" = 90be09f4626c1e28 ] || { echo "모델 sha 다름" >> results/robust_N/done.log; exit 1; }
for s in "$@"; do O=results/robust_N/RL_s$s
  [ -f $O.json ] || ~/sumo-venv/bin/python eval_robust.py $M --episodes 1000 --workers $W --seed $s --ev-prob 0.5 --out $O.json > $O.log 2>&1
  echo "[$(date '+%m-%d %H:%M')] 강건 RL seed $s : $(grep '전체' $O.log | head -1)" >> results/robust_N/done.log
done
echo "[$(date '+%m-%d %H:%M')] 강건 끝 $*" >> results/robust_N/done.log
