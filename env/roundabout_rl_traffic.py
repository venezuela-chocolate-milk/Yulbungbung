"""강화학습용 추가 배경 교통 (env/sumo_roundabout/roundabout.rl.rou.xml 생성).

맵의 기존 roundabout.rou.xml 은 도로를 늘리기 전 목적지로만 차를 보내서,
새로 만든 도로(남쪽 6차선·연결 도로, 아래 긴 도로, 반원 곡선, 위쪽 긴 도로,
오른쪽 아래 십자 교차로)가 거의 비어 있다. 여기서 그 도로들을 지나는 흐름을 추가한다.
기존 맵 파일은 건드리지 않고, 학습 때 두 route 파일을 함께 불러온다.

목표 (기준 배율 1.0, 차로당 시간당):
  - 신호 있는 도로: 약 250대
  - 유턴 때 마주 오는 차(아래 긴 도로 동행), 양보 T자로 오는 차(서행): 약 150대
    → 6초 이상 틈이 날 확률 약 37%. 상대 차는 있지만 오래 막히지 않는 수준
위 목표는 FLOWS 를 정할 때의 기준이고, 안정성 점검 결과 실제 기준 교통량은 배율 0.8 로 확정했다
(env/roundabout_env_config.py TRAFFIC["scale"]). 커리큘럼도 그 배율로 조절한다.

    python -m env.roundabout_rl_traffic          # 파일 생성 + 경로 존재 검사
  생성: roundabout.rl.rou.xml (새 도로 흐름), roundabout.rl_base.rou.xml (기존 흐름의 경로 수정 사본)
"""
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sumo_roundabout",
                   "roundabout.rl.rou.xml")

# (id, 출발 edge, 도착 edge, 시간당 대수, 꼭 거칠 edge)
FLOWS = [
    # ── 아래 긴 도로 동행 (유턴할 때 마주 오는 차) ──
    ("lowEB_A_w",     "S_cross_w_in",          "SW_new_cross_out",   120, None),
    ("lowEB_A_s",     "S_cross_s_in",          "SW_new_upper_out",    60, None),
    # lowEB_B_n (SW_cross_n_in → SW_new_cross_out, 60) 는 2026-11-13 롯데마트 주차장 출차 차로 옮김 (아래 crossB 참고)
    ("lowEB_B_n",     "SW_cross_n_in",         "SW_new_cross_out",     0, None),
    ("lowEB_B_s",     "SW_cross_s_in",         "SW_new_lower_out",    40, None),
    # ── 아래 긴 도로 서행 (양보 T자로 오는 차) ──
    ("lowWB_A_w",     "SW_end_T_extension_in", "S_cross_w_out",      120, None),
    ("lowWB_A_s",     "SW_new_upper_in",       "S_cross_s_out",       80, None),
    ("lowWB_B_s",     "SW_new_lower_in",       "SW_cross_s_out",      60, None),
    ("lowWB_up",      "SW_end_T_extension_in", "SE_triangle_NE_out", 120, "S_conn_up1"),
    ("lowWB_north",   "SW_new_upper_in",       "W_out0_far",          80, None),
    # ── 반원 곡선 내려가기 (위쪽 긴 도로 → 곡선 → 양보 T자 우회전) ──
    ("curve_down_1",  "X_main_in",             "S_cross_w_out",       40, "S_conn_down1"),   # 2026-11-11: 곡선 끝 조각 (S_conn_down 이 마전동 골목 T자로 나뉘어 지름길로 빠지지 않게)
    ("curve_down_2",  "X_nw_in",               "SW_cross_s_out",      20, "S_conn_down1"),
    # ── 위쪽 긴 도로 남동행 ──
    ("upSE_1",        "X_main_in",             "SE_triangle_NE_out",  60, None),
    ("upSE_2",        "T_side_in",             "SW_final_right_out",  30, None),
    ("upSE_3",        "X_nw_in",               "SE_triangle_NE_out",  60, None),
    ("upSE_black",    "X_main_in",             "SW_cross_n_out",       0, "SE_triangle_cross_out"),
    # ── 위쪽 긴 도로 북서행 ──
    ("upNW_1",        "SE_triangle_NE_in",     "X_main_out",         120, None),
    ("upNW_2",        "SE_triangle_NE_in",     "T_side_out",          60, None),
    ("upNW_3",        "SE_triangle_NE_in",     "W_out0_far",          60, None),
    ("upNW_4",        "SW_final_left_in",      "X_nw_out",            40, None),
    # ── 남쪽 도로 남행 (회전교차로 → 남쪽 6차선 → 연결 도로) ──
    ("southS_1",      "N_in1_far",             "S_cross_s_out",       40, None),
    ("southS_2",      "DANBONG_infar",         "SW_new_cross_out",    60, None),
    ("southS_3",      "X_main_in",             "S_cross_w_out",        0, "S_out0_ext"),
    ("southS_4",      "T_side_in",             "S_cross_s_out",       60, None),
    ("southS_5",      "W_in1_far",             "SW_new_upper_out",   100, "S_out0_ext"),
    # ── 남쪽 도로 북행 ──
    ("southN_1",      "S_cross_s_in",          "N_out0_far",         120, None),
    ("southN_2",      "S_cross_w_in",          "DANBONG_outfar",     100, None),
    ("southN_3",      "SW_cross_n_in",         "X_main_out",           0, "S_link_west"),
    ("southN_4",      "SW_end_T_extension_in", "W_out0_far",         100, None),
    # ── 십자 교차로 B (아래 긴 도로 중간) 남북 ──
    # 2026-11-13: 북쪽 끝(SW_cross_n_*)이 롯데마트 주차장이 되면서, 그 길 끝에서 사라지거나 생겨나던
    # 파란 차 흐름(crossB_ns 120, crossB_sn 120, lowEB_B_n 60)을 같은 양의 롯데마트 주차 차로 바꿨다.
    # → env/roundabout_lotte_parking.py LOTTE_FROM_RL. 신호 교차로를 지나는 교통량은 그대로.
    ("crossB_ns",     "SW_cross_n_in",         "SW_cross_s_out",       0, None),
    ("crossB_sn",     "SW_cross_s_in",         "SW_cross_n_out",       0, None),
    # ── 오른쪽 아래 끝 십자 교차로 남북 ──
    ("newX_ns",       "SW_new_upper_in",       "SW_new_lower_out",    60, None),
    ("newX_sn",       "SW_new_lower_in",       "SW_new_upper_out",    60, None),
]


BASE_IN = os.path.join(os.path.dirname(OUT), "roundabout.rou.xml")
BASE_OUT = os.path.join(os.path.dirname(OUT), "roundabout.rl_base.rou.xml")
# 맵의 기존 흐름 중 도로를 늘리기 전 목적지를 그대로 써서, 지금 맵에서는 반원 곡선으로 내려가
# 유턴해 돌아가는 우회 경로를 타는 것들 → 원래 의도대로 검은 도로 → T자 교차로로 지나게 한다.
BASE_VIA = {
    "SW_side_west_right": "SW_final_forward_out",
    "SW_side_east_left": "SW_final_forward_out",
    "se_final_crossroads_3": "SW_final_forward_out",
}


def write_base(src=BASE_IN, out=BASE_OUT):
    """기존 roundabout.rou.xml 은 그대로 두고, 경로만 고친 학습용 사본을 만든다."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(src)
    fixed = 0
    for fl in tree.getroot().iter("flow"):
        via = BASE_VIA.get(fl.get("id"))
        if via:
            fl.set("via", via)
            fixed += 1
    assert fixed == len(BASE_VIA), fixed
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return out


def write(out=OUT):
    lines = ["<?xml version='1.0' encoding='utf-8'?>",
             "<!-- env/roundabout_rl_traffic.py 가 생성. 직접 고치지 말고 FLOWS 를 고칠 것 -->",
             "<routes>"]
    for fid, frm, to, vph, via in FLOWS:
        if vph <= 0:          # 0 이면 꺼 둔 흐름
            continue
        v = f' via="{via}"' if via else ""
        lines.append(f'    <flow id="rl_{fid}" type="car" from="{frm}" to="{to}"{v} begin="0" '
                     f'end="100000" vehsPerHour="{vph}" departLane="best" departSpeed="max" />')
    lines.append("</routes>")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    return out


def check(net_path=None):
    """모든 흐름의 경로가 실제로 이어지는지 확인."""
    import sumolib
    net_path = net_path or os.path.join(os.path.dirname(OUT), "roundabout.net.xml")
    net = sumolib.net.readNet(net_path)
    bad = []
    for fid, frm, to, vph, via in FLOWS:
        stops = [frm] + ([via] if via else []) + [to]
        for a, b in zip(stops, stops[1:]):
            path, _ = net.getShortestPath(net.getEdge(a), net.getEdge(b))
            if not path:
                bad.append((fid, a, b))
    return bad


if __name__ == "__main__":
    print(write_base())
    print(write())
    print("경로 없는 흐름:", check() or "없음")
    print("총", sum(f[3] for f in FLOWS), "대/시간 추가")

