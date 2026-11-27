#!/bin/bash
# 2026-11-24 새 환경(B안 교통 + 긴급차 만남) 처음부터: ③ 강사 1000 + DART 1000 → ④ BC 3개 → ⑤ DAgger.
#   run_track.sh 와 같은 방식 + 시간 단축: DART 는 노트북이 동시에 (HDD 로 옮김), 라운드 평가 100판(seed 5000).
#   강화학습은 자동 시작하지 않음. 이어하기: 같은 명령 다시 실행.
# 사용: bash run_new.sh [작업자 수=20]   기록: results/track_N.log
set -u
cd "$(dirname "$(readlink -f "$0")")"
mkdir -p data results   # 새로 받은 저장소에는 두 폴더가 없음
T=N; W=${1:-20}; EVN=${EVN:-100}; WAIT_MAX=${WAIT_MAX:-60}
PY=${PY:-$HOME/sumo-venv/bin/python}
export SUMO_HOME=$($PY -c "import sumo,os;print(os.path.dirname(sumo.__file__))") OMP_NUM_THREADS=1
LOG=results/track_$T.log
P() { echo "[$(date '+%m-%d %H:%M')] $*" | tee -a $LOG; }
fail() { P "실패: $*"; exit 1; }
N=mom_$T

# ③ 수집: 강사 1000판은 데스크톱, DART 1000판은 노트북(HDD 로 data/${N}_dart.npz 가져옴).
#   데스크톱 강사 수집이 끝난 뒤 WAIT_MAX 분 안에 노트북 DART 가 안 오면 데스크톱이 직접 수집.
if [ ! -f data/${N}_expert.npz ]; then
  P "③ 강사 1000판 수집 (데스크톱, 작업자 $W) / DART 1000판은 노트북"
  $PY collect_pomdp_data.py --policy expert --episodes 1000 --workers $W --seed 100000 \
    --ev-mode train --state-radius 60 --name ${N}_expert > data/${N}_expert.log 2>&1 || fail "강사 수집"
  P "③ 강사 수집 끝: $(grep -m1 'transitions' data/${N}_expert.log | xargs) / 완주 $(grep -c '| 완주$' data/${N}_expert.log)/1000"
fi
t0=$(date +%s)
while [ ! -f data/${N}_dart.npz ]; do
  if [ $(( ($(date +%s) - t0) / 60 )) -ge $WAIT_MAX ]; then
    P "③ 노트북 DART 안 옴 → 데스크톱이 직접 수집"
    $PY collect_pomdp_data.py --policy expert --episodes 1000 --workers $W --seed 200000 \
      --ev-mode train --no-jsonl --noise-episodes 1.0 --accel-noise 0.3 --lane-noise 0.02 \
      --noise-burst-prob 0.03 --noise-burst-len 4 12 --name ${N}_dart.tmp > data/${N}_dart.log 2>&1 || fail "DART 수집"
    mv data/${N}_dart.tmp.npz data/${N}_dart.npz; break
  fi
  sleep 60
done
P "③ DART 준비됨 ($(du -h data/${N}_dart.npz | cut -f1))"
$PY data_check.py data/${N}_expert.npz data/${N}_dart.npz > results/data_check_N.log 2>&1
P "③ 데이터 점검: $(grep -E '판 |품질' results/data_check_N.log | xargs)"
grep -q "문제:" results/data_check_N.log && fail "데이터 품질 문제 (results/data_check_N.log)"

ev() {  # ev <모델 폴더> : EVN판 평가
  [ -f $1.eval.json ] || $PY eval_parallel.py $1/model.pt --episodes $EVN --workers $W --seed 5000 --out $1.eval.json > $1.eval.log 2>&1 || fail "평가 $1"
}
# ④ 모방학습 3개
D="data/${N}_expert.npz data/${N}_dart.npz"
for s in 0 1 2; do
  [ -f results/bc_${T}_seed$s/model.pt ] || { P "④ BC seed$s 학습"; $PY pretrain_ppo_bc.py --data $D --hold-target -0.1 --hidden 512 512 512 --seed $s \
      --out results/bc_${T}_seed$s > results/bc_${T}_seed$s.train.log 2>&1 || fail "BC seed$s"; }
done
for s in 0 1 2; do ev results/bc_${T}_seed$s; P "④ BC seed$s: $(grep -A1 '^===' results/bc_${T}_seed$s.eval.log | tail -1 | cut -c1-110 | xargs)"; done
read M C X <<< "$($PY tools_pick.py results/bc_${T}_seed*.eval.json)"
P "④ BC 최고 $M 완주 $C 충돌 $X"
mkdir -p results/bc_N_best; cp $M results/bc_N_best/model.pt; echo "$C $X" > results/bc_N_best/score.txt   # 노트북 줄 2 출발점
BEST_M=$M; BEST_C=$C; BEST_X=$X; STALL=0
better() { $PY -c "import sys; c,x,bc,bx=map(float,sys.argv[1:]); sys.exit(0 if c>bc or (c==bc and x<bx) else 1)" "$@"; }

# ⑤ DAgger
MAXR=${MAXR:-14}; STALL_MAX=${STALL_MAX:-4}
for R in $(seq 1 $MAXR); do
  FREE=$(df -BG --output=avail . | tail -1 | tr -dc 0-9); [ "$FREE" -lt 3 ] && { P "디스크 여유 ${FREE}GB < 3GB → 멈춤"; break; }
  case $R in 1) BETA=0.5;; 2) BETA=0.3;; 3) BETA=0.2;; 4) BETA=0.1;; *) BETA=0;; esac
  if [ ! -f data/${N}_dagger_r$R.npz ]; then
    P "⑤ DAgger $R 라운드 수집 (운전 $M, β $BETA)"
    $PY collect_pomdp_data.py --model $M --label-expert --dagger-beta $BETA --episodes 600 --workers $W \
      --seed $((300000 + R * 10000)) --ev-mode train --no-jsonl --name ${N}_dagger_r$R > data/${N}_dagger_r$R.log 2>&1 || fail "DAgger $R 수집"
    $PY data_check.py data/${N}_dagger_r$R.npz > results/data_check_N_r$R.log 2>&1
    grep -q "문제:" results/data_check_N_r$R.log && fail "DAgger $R 데이터 품질 문제"
  fi
  DD="$D $(for k in $(seq 1 $R); do echo -n "data/${N}_dagger_r$k.npz "; done)"
  DD="$DD $(ls -v data/${N}_dag[A-Z]_r*.npz 2>/dev/null | tr '\n' ' ')"   # 교환으로 받은 노트북 줄 2 데이터 (있으면)
  P "  학습 데이터 $(echo $DD | wc -w)개 파일 (노트북 것 $(ls data/${N}_dag[A-Z]_r*.npz 2>/dev/null | wc -l)개)"
  for s in 0 1 2; do
    [ -f results/bc_${T}_d${R}_seed$s/model.pt ] || $PY pretrain_ppo_bc.py --data $DD --hold-target -0.1 --hidden 512 512 512 --seed $s \
        --out results/bc_${T}_d${R}_seed$s > results/bc_${T}_d${R}_seed$s.train.log 2>&1 || fail "DAgger $R BC seed$s"
    ev results/bc_${T}_d${R}_seed$s
  done
  read M C X <<< "$($PY tools_pick.py results/bc_${T}_d${R}_seed*.eval.json)"
  P "⑤ $R 라운드 최고 $M 완주 $C 충돌 $X (${EVN}판)"
  if better $C $X $BEST_C $BEST_X; then BEST_M=$M; BEST_C=$C; BEST_X=$X; STALL=0; else STALL=$((STALL+1)); fi
  [ $STALL -ge $STALL_MAX ] && { P "${STALL_MAX}라운드 연속 개선 없음 → 멈춤"; break; }
done
echo "$BEST_M $BEST_C $BEST_X" > results/best_$T.txt
P "DAgger 끝. 최고 $BEST_M 완주 $BEST_C 충돌 $BEST_X (강화학습은 사용자 확인 후)"
P "TRACK_DONE"
