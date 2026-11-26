"""서곶로 × 원당대로 사거리(SW_new_cross) 동행 맨 왼쪽 차로(SW_slip_main 4번)를 좌회전 전용으로 (2026-11-17).

roundabout_sw_pocket 의 설계 의도는 "동행 좌회전 전용 차로"였는데, 연결은 4번 차로에서 직진(SW_new_cross_out 4번)
까지 허용돼 있었다. 그래서 직진 차가 짧은 좌회전 차로(약 36 m)에 들어와 보호 좌회전 신호(8초)를 기다리는
좌회전 차 뒤에 서고, 줄이 늘 꽉 차 좌회전하려는 차가 그 차로로 들어갈 수 없었다.

교차로 모양·내부 경로·신호 번호(linkIndex)는 그대로 두고, 그 직진 연결의 교차로 내부 차로만 승용·화물·버스 등
일반 차가 못 쓰게 한다(allow="emergency"). SUMO 는 경로 차로를 고를 때 연결의 허용 차종을 보므로 직진 차는
4번 차로를 쓰지 않는다. 소방차는 경로상 이 차로에서 좌회전한다.

roundabout_south_extend 가 사거리 신호(roundabout_sw_new_signal) 바로 뒤에 부른다. 지금 맵에 바로 적용:
    python -m env.roundabout_sw_left_only
"""
import os
import re

FROM_EDGE, TO_EDGE, LANE = 'SW_slip_main', 'SW_new_cross_out', '4'


def apply_net(net_path):
    txt = open(net_path, encoding='utf-8').read()
    m = re.search(rf'<connection from="{FROM_EDGE}" to="{TO_EDGE}" fromLane="{LANE}" toLane="{LANE}" via="([^"]+)"', txt)
    if not m:
        return {'sw_left_only': 'connection not found (already removed?)'}
    via = m.group(1)
    lane_re = re.compile(rf'(<lane id="{re.escape(via)}"[^>]*?)(\s+allow="[^"]*"|\s+disallow="[^"]*")?(\s+speed=)')
    if f'<lane id="{via}"' not in txt:
        return {'sw_left_only': f'internal lane {via} not found'}
    new, n = lane_re.subn(lambda mm: f'{mm.group(1)} allow="emergency"{mm.group(3)}', txt, count=1)
    if n != 1:
        return {'sw_left_only': 'lane pattern mismatch'}
    open(net_path, 'w', encoding='utf-8').write(new)
    return {'sw_left_only': via}


if __name__ == '__main__':
    from env.roundabout_env_config import NET_FILE
    print(apply_net(NET_FILE))
