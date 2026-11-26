#!/bin/bash
# 온도·메모리 감시 (리눅스): 30초마다 CPU 패키지 온도, GPU 온도, 남은 메모리를 기록하고,
#   CPU ≥ CPU_HOT 3번 연속 또는 GPU ≥ GPU_HOT 또는 남은 메모리 < 2 GB → 이 폴더에서 돌아가는 python·sumo 를 일시정지(SIGSTOP)
#   CPU ≤ CPU_OK, GPU ≤ GPU_OK, 메모리 ≥ 4 GB 로 돌아오면 재개(SIGCONT). 프로세스를 죽이지는 않는다.
# 사용: nohup bash guard_track.sh [CPU_HOT=90] [CPU_OK=82] > /dev/null 2>&1 &      기록: results/guard_<날짜>.log
#   데스크톱(i9) 기준은 85/78 이었음. 노트북 CPU 는 원래 더 뜨겁게 도는 설계라 기본 90/82.
P="$(dirname "$(readlink -f "$0")")"
CPU_HOT=${1:-90}; CPU_OK=${2:-82}; GPU_HOT=83; GPU_OK=75
LOG=$P/results/guard_$(date +%Y%m%d).log; hot=0; paused=0
procs() {
  for p in $(pgrep -u "$(id -u)" -f "python|bin/sumo"); do
    [ "$p" = $$ ] && continue
    [ "$(readlink /proc/$p/cwd 2>/dev/null)" = "$P" ] && echo $p
  done
}
ctemp() {  # 인텔: "Package id 0", AMD: "Tctl"
  sensors 2>/dev/null | sed -n -e 's/^Package id 0:\s*+\([0-9.]*\).*/\1/p' -e 's/^Tctl:\s*+\([0-9.]*\).*/\1/p' | head -1 | cut -d. -f1
}
while true; do
  c=$(ctemp); g=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader 2>/dev/null | head -1)
  m=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo)
  c=${c:-0}; g=${g:-0}
  [ "$c" -ge "$CPU_HOT" ] && hot=$((hot+1)) || hot=0
  echo "$(date '+%m-%d %H:%M:%S') CPU ${c}C GPU ${g}C 메모리여유 ${m}GB 정지=${paused}" >> "$LOG"
  if [ $paused = 0 ] && { [ $hot -ge 3 ] || [ "$g" -ge $GPU_HOT ] || [ "$m" -lt 2 ]; }; then
    kill -STOP $(procs) 2>/dev/null; paused=1; echo "  !! 일시정지 (CPU ${c} GPU ${g} 메모리 ${m}GB)" >> "$LOG"
  elif [ $paused = 1 ] && [ "$c" -le "$CPU_OK" ] && [ "$g" -le $GPU_OK ] && [ "$m" -ge 4 ]; then
    kill -CONT $(procs) 2>/dev/null; paused=0; hot=0; echo "  재개" >> "$LOG"
  fi
  sleep 30
done
