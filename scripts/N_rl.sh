#!/bin/bash
# 11-25 새 환경 강화학습 (C_rl.sh 와 같은 설정, 데이터만 mom_N). 원래: 11-22 ⑥ C 강화학습: DAgger 최고 모델에서 시작, BC 손실을 계속 0.5로 붙잡은 채 PPO (예전 rl_mom_r4는 BC 0.5→0.2·lr 1e-5에서 충돌이 6→17%로 흔들림 → 더 보수적으로).
# 사용: setsid nohup bash results/N_rl.sh <시작모델.pt> [run이름=rl_C] [총스텝=6000000] > results/rl_N.log 2>&1 &
set -u
INIT=$1; RUN=${2:-rl_N}; TOT=${3:-6000000}
cd "$(dirname "$(readlink -f "$0")")/.."   # 저장소 맨 위 폴더
export BC_MAX_STEPS=${BC_MAX_STEPS:-8000000}  # 11-23 RL은 워커와 같이 돌아 RAM 한계 → 800만
export SUMO_HOME=~/sumo-venv/lib/python3.10/site-packages/sumo PATH=~/sumo-venv/bin:$PATH OMP_NUM_THREADS=1 SUMO_LIBSUMO=1
python train.py --run-name $RUN --init $INIT --total-timesteps $TOT \
  --bc-data data/mom_N_expert.npz data/mom_N_dart.npz $(ls -v data/mom_N_dagger_r*.npz data/mom_N_dag[A-Z]_r*.npz | tr "\n" " ") \
  --bc-hold-target -0.1 --bc-coef 0.5 --bc-coef-final 0.5 --bc-decay-steps 1 \
  --critic-warmup 30 --lane-entropy-coef 0 --target-kl 0.01 --lr ${LR:-5e-6} --init-log-std ${LSTD:--2.5}
