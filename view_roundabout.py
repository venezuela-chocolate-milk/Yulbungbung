"""회전교차로 도로 확인 스크립트 — 검단회전교차로 모델을 학습 없이 눈으로 확인한다.

기본은 env/sumo_roundabout/ 에 있는 맵을 그대로 띄운다 (다시 만들지 않음).
이 맵은 roundabout_builder 결과에 여러 패치(env/roundabout_south_extend.py)를
덧붙인 것이라, builder 만 다시 돌리면 손질한 도로가 전부 사라진다.
(기존 직선 고속도로는 view_road.py, 회전교차로는 이 파일)

사용법:
    python view_roundabout.py             # sumo-gui로 띄우기 (▶ 눌러야 차가 움직임)
    python view_roundabout.py --rebuild   # 백업(reports/south-road-before)에서 패치를 전부 다시 적용한 뒤 띄우기
    python view_roundabout.py --netedit   # netedit(도로 편집기)로 열기
    python view_roundabout.py --nogui     # 화면 없이 설정 검증 + 차량 통계만

관찰 포인트:
    - 원 안 2차로, 방향별 진입/진출 차로 수가 config대로인지
    - 노란 빗금 / 섬 위치가 실제와 맞는지 (순수 시각 요소)
    - (▶ 재생 후) 모든 진입로에서 들어온 차가 원을 돌아 각 출구로 나가는지
"""
import argparse
import os
import subprocess

from env.mdp_config import SIMULATION
# sumo_env를 import하면 SUMO_HOME 자동 감지 로직이 함께 실행된다
from env.sumo_env import SumoHighwayEnv  # noqa: F401  (자동 감지 목적의 import)
from sumolib import checkBinary

BASE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(BASE, "env", "sumo_roundabout")


def print_summary():
    from env.roundabout_config import RING, APPROACHES
    names = {"N": "북 (왕길지하차도)", "DANBONG": "단봉로", "W": "서 (검단로)",
             "S": "남", "E": "동 (검단로)"}
    print("─" * 46)
    print(" 회전교차로 설정 (env/roundabout_config.py)")
    print("─" * 46)
    print(f"  회전부        : 반지름 {RING['radius']} m, {RING['num_lanes']}차로, "
          f"{RING['speed_limit'] * 3.6:.0f} km/h")
    for key, ap in APPROACHES.items():
        legs = ", ".join(f"{'진출' if l['kind'] == 'exit' else '진입'} {l['lanes']}차로"
                         for l in ap["legs"])
        print(f"  {names.get(key, key):14s}: {legs}")
    print("─" * 46)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="회전교차로 도로를 GUI로 확인")
    parser.add_argument("--netedit", action="store_true", help="netedit로 열기")
    parser.add_argument("--nogui", action="store_true", help="화면 없이 설정 검증만")
    parser.add_argument("--rebuild", action="store_true",
                        help="패치 파이프라인(env/roundabout_south_extend)으로 맵을 다시 만든다")
    parser.add_argument("--seconds", type=int, default=600,
                        help="--nogui 검증 시 시뮬레이션 길이(초). 기본 600초.")
    args = parser.parse_args()

    sumocfg = os.path.join(OUT, "roundabout.sumocfg")
    if args.rebuild:
        from env import roundabout_south_extend
        roundabout_south_extend.main()
        print(f"도로 다시 생성 완료 (패치 파이프라인): {sumocfg}")
    elif not os.path.exists(sumocfg):
        raise FileNotFoundError(f"맵이 없습니다: {sumocfg}  (python view_roundabout.py --rebuild)")
    net_file = os.path.join(OUT, "roundabout.net.xml")
    print(f"맵 불러오기: {sumocfg}")
    print_summary()

    if args.netedit:
        print("netedit 실행 중... (창을 닫으면 종료)")
        subprocess.run([checkBinary("netedit"), net_file])

    elif args.nogui:
        result = subprocess.run(
            [checkBinary("sumo"), "-c", sumocfg,
             "--step-length", str(SIMULATION["step_length"]),
             "--end", str(args.seconds),
             "--no-step-log", "true",
             "--duration-log.statistics", "true"],
            capture_output=True, text=True, encoding="utf-8", errors="replace")
        # 경고(급정거 등)는 stderr, 차량 통계는 stdout으로 나온다
        print(result.stderr.strip())
        print(result.stdout.strip())
        print("설정 검증 OK" if result.returncode == 0 else "설정 오류!")

    else:
        # 학습 환경과 같은 배경 교통(기준 배율)과 신호등 그림으로 띄운다.
        # 신호등 그림은 TraCI 로 매 스텝 실제 신호에 맞춰 켜야 해서 traci 로 실행한다.
        import traci
        from env import roundabout_env_config as rc
        from env.roundabout_signal_heads import SignalHeads
        print("sumo-gui 실행 중...")
        print("  · 창이 뜨면 도로만 보입니다. ▶(플레이) 버튼을 눌러야 차가 움직입니다.")
        print("  · 속도 조절: 상단 Delay(ms) 값을 조정 (크게 = 느리게)")
        print("  · 종료하려면 창을 닫으세요.")
        routes = list(rc.TRAFFIC["route_files"])   # 학교·상가 골목 차도 여기에 들어 있다
        traci.start([checkBinary("sumo-gui"), "-c", sumocfg,
                     "--route-files", ",".join(routes),
                     "--scale", str(rc.TRAFFIC["scale"]),
                     "--additional-files", ",".join(rc.GUI_ADDITIONAL),
                     "--step-length", str(SIMULATION["step_length"]),
                     "--delay", str(SIMULATION["gui_delay"]), "--no-warnings", "true"])
        heads = SignalHeads(rc.SIGNAL_HEADS)
        ip = None
        if getattr(rc, "ILLEGAL_PARKING", False):   # 불법 주정차 구간 (env/roundabout_illegal_parking.py)
            from env import roundabout_illegal_parking as ip
            ip.spawn(traci)
        try:
            while True:
                traci.simulationStep()
                heads.update()
                if ip:
                    ip.tick(traci)
        except (traci.FatalTraCIError, traci.TraCIException):
            pass   # 창을 닫으면 연결이 끊긴다
        finally:
            try:
                traci.close()
            except Exception:
                pass
