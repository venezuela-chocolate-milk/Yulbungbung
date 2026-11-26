"""reports/audit_all_left_turns.py 를 리눅스·윈도우 어디서나 돌린다 (sumo.exe 경로 고정 문제 해결).

    python tools/audit_left_turns.py <라벨> [맵폴더(기본 env/sumo_roundabout)]
  결과: reports/all-left-turns-<라벨>.json, 화면에 통과 수와 ISSUE 목록.
  지금 기준 '원래 있던 실패' 2개: left_audit (S_out0_ext5 4번 차로 → 3번으로 나감), nw_119 (소방차 전용 출구라 승용차 시험 불가).
"""
import os
import re
import subprocess
import sys
import tempfile

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
label = sys.argv[1]
net_dir = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else os.path.join(BASE, 'env', 'sumo_roundabout')
src = open(os.path.join(BASE, 'reports', 'audit_all_left_turns.py'), encoding='utf-8').read()
exe = 'sumo.exe' if os.name == 'nt' else 'sumo'
src = src.replace("bin/sumo.exe", f"bin/{exe}")
src = src.replace("NET=BASE/'env/sumo_roundabout'", f"NET=Path(r'{net_dir}')")
src = re.sub(r"^BASE=Path\(__file__\)\.resolve\(\)\.parents\[1\]", lambda m: f"BASE=Path(r'{BASE}')", src,
             flags=re.M)   # 함수로 넘겨야 윈도우 경로의 \U 등을 정규식 escape 로 읽지 않는다
with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False, encoding='utf-8') as f:
    f.write(src)
    tmp = f.name
r = subprocess.run([sys.executable, tmp, '--label', label], cwd=BASE)
os.unlink(tmp)
sys.exit(r.returncode)
