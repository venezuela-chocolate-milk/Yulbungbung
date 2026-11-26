"""시험 차량을 경로마다 한 대씩 달려 도착·충돌을 확인한다.

    python tools/drive_test.py 이름=출발엣지:도착엣지 [이름=출발엣지:도착엣지 ...] [--vtype firetruck]
    예) python tools/drive_test.py left=T_end_side_in:T_end_main_out right=T_end_side_in:T_main_in

  경로는 SUMO 최단경로로 잇는다. 모든 차가 'arrived' 이고 collisions 0 이어야 통과.
  배경 교통 없이 도로망만 쓴다 (vType 이 rou 파일에 정의된 것이면 --routes 로 rou 파일들을 같이 준다).
"""
import argparse
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE)
import traci  # noqa: E402
from sumolib import checkBinary  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tests', nargs='+')
ap.add_argument('--vtype', default=None)
ap.add_argument('--routes', default=None, help='vType 정의가 든 rou 파일들 (쉼표)')
a = ap.parse_args()
net = os.path.join(BASE, 'env', 'sumo_roundabout', 'roundabout.net.xml')
cmd = [checkBinary('sumo'), '-n', net, '--step-length', '0.1', '--no-step-log', 'true',
       '--collision.action', 'warn', '--no-warnings', 'true']
if a.routes:
    cmd += ['-r', a.routes, '--scale', '0.01']
traci.start(cmd)
tests = {}
for t in a.tests:
    name, fr_to = t.split('=')
    fr, to = fr_to.split(':')
    r = traci.simulation.findRoute(fr, to, vType=a.vtype or '')
    if not r.edges:
        print(name, 'NO ROUTE', fr, '->', to)
        continue
    traci.route.add(name, r.edges)
    kw = {'typeID': a.vtype} if a.vtype else {}
    traci.vehicle.add(name, name, depart='0', departLane='best', **kw)
    tests[name] = []
done = {}
for step in range(4000):
    traci.simulationStep()
    ids = set(traci.vehicle.getIDList())
    for k, seen in tests.items():
        if k in ids:
            e = traci.vehicle.getRoadID(k)
            if not seen or seen[-1] != e:
                seen.append(e)
        elif seen and k not in done:
            done[k] = step/10
    if len(done) == len(tests):
        break
for k, seen in tests.items():
    print(k, 'arrived' if k in done else 'NOT arrived', done.get(k), [e for e in seen if not e.startswith(':')])
print('collisions', traci.simulation.getCollidingVehiclesNumber())
traci.close()
