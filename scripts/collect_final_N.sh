#!/bin/bash
# 11-26 최종 데이터 1000만 행: 최종 모델 E200k (rl_NE/model_200704.pt, sha256 90be09f4...), 강사 데이터 안 섞음
# 4 묶음 × 700판 (판당 약 3,600행 → 약 1,008만 행), 작업자 20, seed 1000000*p (평가 seed 와 겹치지 않음)
# 끝나면 묶음별 data_check + sha256 → 5070ti 전송(sha 확인) → 데스크탑 강건성 seed 500000
cd "$(dirname "$(readlink -f "$0")")/.."   # 저장소 맨 위 폴더
PY=~/sumo-venv/bin/python; M=models/final_rl/model.pt; L=results/track_N.log; D=data/final_N
P(){ echo "[$(date '+%m-%d %H:%M')] $*" >> $L; }
[ "$(sha256sum $M | cut -c1-16)" = 90be09f4626c1e28 ] || { P "⑨ 모델 sha 다름 — 중단"; exit 1; }
for p in 1 2 3 4; do
  [ -f $D/final_N_rl_part$p.npz ] && continue
  P "⑨ 수집 묶음 $p 시작 (700판, seed $((1000000*p)))"
  $PY collect_pomdp_data.py --policy ppo --model $M --episodes 700 --workers 20 --seed $((1000000*p)) --ev-mode eval --state-radius 60 --out-dir $D --name final_N_rl_part$p > $D/part$p.log 2>&1 || { P "⑨ 수집 묶음 $p 실패"; exit 1; }
  P "⑨ 수집 묶음 $p 끝: $(grep transitions $D/part$p.log)"
done
$PY data_check.py $D/final_N_rl_part*.npz > $D/data_check.txt 2>&1
(cd $D && sha256sum final_N_rl_part*.npz final_N_rl_part*.jsonl.gz > SHA256SUMS)
T=$($PY -c "import numpy as np,glob; print(sum(len(np.load(f)['t']) for f in sorted(glob.glob('$D/final_N_rl_part*.npz'))))")
P "⑨ 수집 끝: 총 $T 행, $(grep -c '품질 OK' $D/data_check.txt)/4 품질 OK, sha256 → $D/SHA256SUMS"
R=AICAD_N_rl_5070ti
ssh -o BatchMode=yes aicad-5070ti "mkdir -p $R/data/final_N" < /dev/null
scp -q -o BatchMode=yes $D/final_N_rl_part* $D/SHA256SUMS $D/data_check.txt aicad-5070ti:$R/data/final_N/ < /dev/null \
 && ssh -o BatchMode=yes aicad-5070ti "cd $R/data/final_N && sha256sum -c SHA256SUMS" < /dev/null > $D/sha_5070ti.txt 2>&1 \
 && P "⑨ 5070ti 전송 끝, sha256 $(grep -c ': OK' $D/sha_5070ti.txt)/8 OK" || P "⑨ 5070ti 전송/검증 실패 — 확인 필요"
O=results/robust_N/RL_s500000
$PY eval_robust.py $M --episodes 1000 --workers 20 --seed 500000 --ev-prob 0.5 --out $O.json > $O.log 2>&1
P "⑧강건 RL seed 500000 (데스크탑): $(grep '전체' $O.log | head -1)"
