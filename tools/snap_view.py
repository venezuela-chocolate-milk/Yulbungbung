"""SUMO 화면에서 지정한 영역만 캡처한다 (데스크톱 전체 스크린샷 대신 — 다른 창이 찍히지 않는다).

    python tools/snap_view.py <출력폴더> 이름:x:y:줌 [이름:x:y:줌 ...]
    예) python tools/snap_view.py reports/snap nw:360:385:900 nw_zoom:330:415:1800

  x, y = SUMO 좌표(m) 화면 가운데, 줌 = sumo-gui 줌 값 (900 ≈ 블록 여러 개, 3000 ≈ 교차로 하나).
  GUI 그림 파일은 env/roundabout_env_config.GUI_ADDITIONAL 그대로(풍경 레이어는 꺼진 설정 따름).
  끝나면 sumo-gui 를 스스로 닫는다.
"""
import os
import sys
import time

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE)
import traci  # noqa: E402
from sumolib import checkBinary  # noqa: E402
from env import roundabout_env_config as rc  # noqa: E402

out = sys.argv[1]
views = [v.split(':') for v in sys.argv[2:]]
os.makedirs(out, exist_ok=True)
cfg = os.path.join(rc.MAP_DIR, 'roundabout.sumocfg')
import socket  # noqa: E402
import subprocess  # noqa: E402
with socket.socket() as sk:                  # 빈 포트
    sk.bind(('localhost', 0))
    port = sk.getsockname()[1]
proc = subprocess.Popen([checkBinary('sumo-gui'), '-c', cfg, '--additional-files', ','.join(rc.GUI_ADDITIONAL),
                         '--window-size', '1500,1000', '--no-warnings', 'true', '--start', 'true',
                         '--route-files', os.path.join(rc.MAP_DIR, 'roundabout.rou.xml'),
                         '--remote-port', str(port)])
traci.init(port, numRetries=60)
vid = 'View #0'
saved = []
try:
    for name, x, y, z in views:
        traci.gui.setOffset(vid, float(x), float(y))
        traci.gui.setZoom(vid, float(z))
        traci.simulationStep()
        path = os.path.join(out, f'{name}.png')
        traci.gui.screenshot(vid, path)
        traci.simulationStep()            # 스크린샷은 다음 스텝에 저장된다
        saved.append(path)
    traci.simulationStep()
    time.sleep(1.5)
finally:
    proc.terminate()                      # traci.close() 는 sumo-gui 에서 멈출 때가 있어 프로세스를 직접 끝낸다
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
print('saved:', saved)
