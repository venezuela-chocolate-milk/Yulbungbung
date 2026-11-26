"""남쪽 6차선 도로를 직선으로 더 늘리고, 검은 도로(SW_final_forward)를 늘려 T자 묶음을 평행이동한다.

  - 위쪽 긴 도로(X 교차로 → 위쪽 삼각형 교차로)를 SE_EXTRA 만큼 늘려 오른쪽 묶음 전체를 평행이동한다
    (2026-11-10: 실제 지도처럼 윗변을 길고 곧게 펴기 위해 추가. 교차로 모양은 그대로).
  - 대각선 도로 윗부분을 원호로 휘어 T자 묶음을 회전, 아래 긴 도로를 수평으로 (env.roundabout_bend).
  - 긴 도로는 유턴을 만들기 전에 늘리고, 유턴은 T자 쪽에서 U_ST m, 합류(곡선 끝)는 그 동쪽 280m(J_FROM_UTURN).

  - 빨간 도로: env.roundabout_south_six_lane 의 6차선 연장을 회전 없이 RED_EXTRA 만큼 더 길게.
  - 검은 도로: env.roundabout_south_link.stretch_black 으로 BLACK_EXTRA 만큼 늘리고,
    T자 교차로 아래 묶음 전체를 모양 그대로 같은 방향으로 이동한다.
    점선 주기(약 5.98m)가 끊기지 않도록 실제 이동량은 주기의 배수로 맞춘다.
  - 빨간 도로 남행 끝부분: 노란 빗금 안전지대로 차로를 바깥으로 옮기고,
    부드러운 코 옆 가장 안쪽에 차로 추가(3→4).
  - 두 도로 끝을 직선 도로(방향별 5차로)로 잇고 양쪽 끝을 십자 신호 교차로로 만든다
    (env.roundabout_south_cross).

    python -m env.roundabout_south_extend
원본은 reports/south-road-before/ 에서 매번 다시 읽는다.
"""
import os
import sys
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 네이버 지도(검단) 실측 거리에 맞춰 도로 모양 맞추기 (2026-11-10). 교차로 묶음은 모양 그대로 두고
# 교차로 사이 도로의 길이와 방향만 바꾼다. 지도 실측: 회전교차로→검단사거리역(X) 720m, X→마전역(위쪽 삼각형)
# 880m(남동 약 31°), 마전역→T자 315m(거의 정남), T자→완정역 494m(동쪽 약 -8°), 회전교차로→SK 교차로 885m,
# SK→T자 약 1040m(동쪽 약 +16°), 마전역→남동 직선 600m→꺾여 완정역 앞 합류.
# 위쪽 긴 도로(X 교차로 → 위쪽 삼각형 교차로, X_se_*): SE_EXTRA 만큼 늘이고(음수면 줄임) 오른쪽 묶음 평행이동,
# 이어서 X 교차로 바로 뒤를 TOP_BEND_DEG 만큼 원호로 휘어 지도 방향(남동 약 31°)에 맞춘다.
SE_EXTRA = -100.0
SE_CUT = 500.0          # X 교차로(T_end_main_end) 기준 절단 위치 (직선 구간 한가운데)
TOP_BEND_DEG = -7.0
TOP_BEND_RADIUS = 600.0
RED_EXTRA = 520.0
# 남행 안전지대 (지도의 기존 x_se_end 안전지대와 같은 구조):
# 끝에서 160m 앞부터 40m 동안 넓어지며 남행 차로가 바깥으로 한 차로 이동,
# 14m 코로 부드럽게 닫히고 그 옆 가장 안쪽에 4번째 차로가 열린다.
# 끝에서 40m 앞(사용자 표시 지점)부터 남행 가장 바깥에 차로 하나 더 → 5차로 (앞 40m 테이퍼).
BAY = dict(from_end=160.0, length=40.0, nose=14.0, outer_from_end=40.0, outer_taper=40.0,
           outer_dash_lead=15.0)   # 바깥 새 차로 점선을 테이퍼 안으로 15m(점선 2~3개) 앞당겨 시작
BLACK_EXTRA = 200.0     # 대각선(검은) 도로: 윗부분 원호 + T자 앞 직선이 들어갈 길이
# T자 ~ 동남쪽 끝 긴 도로 연장: T자 쪽 시작점 기준 LONG_CUT 지점(T자 정지선 구간과 차로 넓어지는 구간 사이)에서
# 유턴을 만들기 전에 자른다. 오른쪽 곡선이 완만하게 내려오도록 유턴/합류 지점을 동쪽으로 옮길 자리를 만든다.
LONG_EXTRA = 200.0
LONG_CUT = 250.0
U_ST = 370.0            # T자 쪽 시작점에서 유턴 지점까지. 합류 지점(곡선 끝)은 그 동쪽 280m (roundabout_connector.J_FROM_UTURN)
# 아래 긴 도로를 수평(동쪽 0°)으로: 대각선 도로 윗부분을 반지름 BEND_RADIUS 원호로 휘어 T자 묶음을 회전.
# T자 앞 BEND_TAIL m 는 직선으로 남긴다 (검은 도로 3차로 축소 구간 + 테이퍼).
BEND_HEADING = -8.4
BEND_RADIUS = 220.0
BEND_TAIL = 150.0


def main():
    sys.path.insert(0, BASE)
    from env import roundabout_south_six_lane as six
    from env import roundabout_south_link as link
    net_path = os.path.join(six.NET_DIR, 'roundabout.net.xml')
    deco_path = os.path.join(six.NET_DIR, 'roundabout.deco.xml')

    from env import roundabout_stretch as stretch
    # 0) 위쪽 긴 도로 연장 + 오른쪽 묶음 평행이동 (원본에서 시작)
    net_tree = ET.parse(os.path.join(six.BEFORE_DIR, 'roundabout.net.xml'))
    deco_tree = ET.parse(os.path.join(six.BEFORE_DIR, 'roundabout.deco.xml'))
    jxy = {j.get('id'): (float(j.get('x')), float(j.get('y'))) for j in net_tree.getroot().findall('junction')}
    o, f = jxy['T_end_main_end'], jxy['X_se_end']
    d = ((f[0]-o[0])**2 + (f[1]-o[1])**2) ** 0.5
    rep0 = stretch.stretch(net_tree.getroot(), deco_tree.getroot(), o, ((f[0]-o[0])/d, (f[1]-o[1])/d),
                           cut_hint=SE_CUT, D=SE_EXTRA, cut_edges={'X_se_in', 'X_se_out'},
                           seeds=['X_se_end'], barrier='T_end_main_end', forbid=['T_junction'])
    from env import roundabout_bend as bend
    rep_top = bend.apply(net_tree.getroot(), deco_tree.getroot(), radius=TOP_BEND_RADIUS,
                         seed='X_se_end', barrier='T_end_main_end', phi_deg=TOP_BEND_DEG)

    # 1) 검은 도로 연장 + T자 묶음 평행이동
    import copy
    rep = link.stretch_black(net_tree.getroot(), deco_tree.getroot(), BLACK_EXTRA,
                             pre=copy.deepcopy(net_tree.getroot()))
    # 1-1) 대각선 도로 윗부분을 휘어 아래 T자 묶음을 회전 (아래 긴 도로 수평)
    rep_bend = bend.apply(net_tree.getroot(), deco_tree.getroot(), BEND_HEADING, BEND_RADIUS, tail=BEND_TAIL)
    link.update_boundary(net_tree.getroot())
    ET.indent(net_tree, space='    ')
    net_tree.write(net_path, encoding='utf-8', xml_declaration=True)
    ET.indent(deco_tree, space='    ')
    deco_tree.write(deco_path, encoding='utf-8', xml_declaration=True)

    # 2) 남쪽 6차선 연장 (직선, 회전 없음) + 남행 안전지대
    six.LENGTH = 180.0 + RED_EXTRA
    six.BAY = BAY
    frame = six.apply_net(net_path)
    info = six.apply_deco(deco_path, net_path, frame)

    # 3) 빨간 도로 끝과 파란 도로 끝을 직선으로 잇고 양쪽을 십자 교차로로
    from env import roundabout_south_cross as cross
    plan = cross.apply_net(net_path, frame, six.LENGTH)
    info.update(cross.apply_deco(deco_path, net_path, plan))

    # 4) T자 ~ 동남쪽 끝 긴 도로를 LONG_EXTRA 만큼 늘리고 그 너머 묶음을 평행이동한 뒤,
    #    긴 도로 중간 유턴 (서행 1차로)
    from env import roundabout_long_uturn as uturn
    from env import roundabout_stretch as stretch
    if LONG_EXTRA:
        net_tree = ET.parse(net_path)
        deco_tree = ET.parse(deco_path)
        lane0 = next(e for e in net_tree.getroot().findall('edge') if e.get('id') == uturn.OUT).find('lane')
        lax = uturn.Axis(uturn._read(lane0.get('shape')))
        rep2 = stretch.stretch(net_tree.getroot(), deco_tree.getroot(), lax.a, lax.u,
                               cut_hint=LONG_CUT, D=LONG_EXTRA, cut_edges={uturn.IN, uturn.OUT},
                               seeds=[uturn.FAR_IN_NODE, uturn.FAR_OUT_NODE], barrier='SW_final_forward',
                               forbid=['SE_triangle_SW'])
        link.update_boundary(net_tree.getroot())
        ET.indent(net_tree, space='    ')
        net_tree.write(net_path, encoding='utf-8', xml_declaration=True)
        ET.indent(deco_tree, space='    ')
        deco_tree.write(deco_path, encoding='utf-8', xml_declaration=True)
        info.update({f'long_{k}': (round(v, 3) if isinstance(v, float) else v) for k, v in rep2.items()})
    uturn.U_ST = U_ST
    ax, median = uturn.apply_net(net_path)
    info.update(uturn.apply_deco(deco_path, net_path, ax, median))

    # 6) 위쪽 삼각형 교차로 동쪽 끝 도로를 곡선으로 내려 긴 도로(유턴 +280m)에 양보 T자로 연결
    from env import roundabout_connector as conn
    cv = conn.apply_net(net_path, ax, median)
    info.update(conn.apply_deco(deco_path, net_path, cv))

    # 6-1) T자 교차로 동행 4번 차로: 좌회전 → 직진·좌회전
    from env import roundabout_t_straight_left as tsl
    info.update(tsl.apply_net(net_path))
    info.update(tsl.apply_deco(deco_path, net_path))

    # 6-2) 검은 도로 남행: T자 앞 100m 를 3차로로 줄이고 진입 연결을 좌·좌·우로
    from env import roundabout_t_black_lanes as black
    r_net, bax, bsN, blat = black.apply_net(net_path)
    info.update(r_net)
    info.update(black.apply_deco(deco_path, net_path, bax, bsN, blat))

    # 7) 기존 정지선을 실제 정지 위치(진입 차로 끝 1m 뒤)로 옮긴다 (새로 만들지 않음)
    from env import roundabout_stoplines as stoplines
    rep3 = stoplines.fix(deco_path, net_path)
    info['stoplines_moved'] = sum(1 for r in rep3 if r[3].startswith('moved'))
    for key in ('dashes_removed', 'guides_clipped', 'arrows_shifted'):
        info[f'stoplines_{key}'] = next(r[1] for r in rep3 if r[0] == key)

    # 7-1) 서쪽 가지(남서쪽 긴 도로)를 회전교차로에서 200m 까지만 남긴다
    from env import roundabout_trim_west as trimw
    info.update(trimw.apply(net_path, deco_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)

    # 8) 사용자가 짚은 점선 빈 칸 채우기
    from env import roundabout_touchups as touchups
    info.update(touchups.apply(deco_path, net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    # 8-1) 원당대로 동행 우회전 차로 경계 점선: 테이퍼 안(차로가 덜 벌어진 곳) 앞 4개 빼기 (2026-11-12 사용자 요청,
    #      원본 deco 에 이미 그려져 있어 여기서 지운다. env.roundabout_long_right 도 같은 자리부터 그리게 고침)
    _dt = ET.parse(deco_path)
    for _p in list(_dt.getroot().findall('poly')):
        if _p.get('id') in {f'SW_long_right_divider_{k}' for k in range(4)}:
            _dt.getroot().remove(_p)
    # 8-2) 완정로 남동행 넓어지는 곳(X_se_widen 뒤): 차로 경계 점선 lane_dash_X_se_out_far_1_* 끝과 다음 점선
    #      (se_triangle_aligned_dash_*) 사이 빠진 점선 하나 채우기 (2026-11-12 사용자 요청)
    _r = _dt.getroot()
    _far = sorted((p for p in _r.findall('poly') if p.get('id', '').startswith('lane_dash_X_se_out_far_1_')),
                  key=lambda p: int(p.get('id').rsplit('_', 1)[1]))
    if len(_far) >= 2 and not any(p.get('id') == 'lane_dash_X_se_out_far_1_fill' for p in _r.findall('poly')):
        _pt = lambda p: [tuple(map(float, q.split(','))) for q in p.get('shape').split()]
        _a, _b = _pt(_far[-2]), _pt(_far[-1])
        _d = (_b[0][0] - _a[0][0], _b[0][1] - _a[0][1])
        _new = ET.SubElement(_r, 'poly', dict(_far[-1].attrib))
        _new.set('id', 'lane_dash_X_se_out_far_1_fill')
        _new.set('shape', ' '.join(f'{x + _d[0]:.6f},{y + _d[1]:.6f}' for x, y in _b))
    _dt.write(deco_path, encoding='utf-8', xml_declaration=True)
    # 9) 검은 도로 끝 T자 교차로를 신호 교차로로 (좌회전 2차로가 양보만 하다 막히던 문제)
    from env import roundabout_black_t_signal as tsig
    info.update(tsig.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    tree = ET.ElementTree(root)
    link.update_boundary(root)
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    # 9-1) 오른쪽 아래 끝 십자 교차로: 안전지대를 가로지르는 우회전 연결 삭제 (우회전은 전용 도로로만)
    from env import roundabout_remove_cross_right as xright
    info.update(xright.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    # 9-2) 위쪽 긴 도로에서 완정초·마전중 사이로 들어가는 학교 진입로 (우회전 진입·진출),
    #      끝에서 양쪽 학교 정문으로 짧은 길 (정문에서 회차) + 학교 그림 (GUI 표시 전용)
    from env import roundabout_school_access as school
    info.update(school.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(school.apply_deco(deco_path, net_path))
    from env import roundabout_school_buildings as school_img
    info.update({k: v for k, v in school_img.build(net_path).items() if k != 'out'})
    # 9-3) 검단사거리 남동쪽 상가 골목 (117번길 포함) + 상가·보도 그림 (GUI 표시 전용)
    from env import roundabout_shops as shops
    info.update(shops.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(shops.apply_deco(deco_path, net_path))
    from env import roundabout_shop_buildings as shop_img
    info.update({k: v for k, v in shop_img.build(net_path).items() if k != 'out'})
    # 9-4) 검단사거리 남서쪽 큰길의 왼쪽 옆길(검단로445번길)을 교차점째로 큰길 방향으로 평행이동
    from env import roundabout_slide_side as slide
    info.update(slide.apply(net_path, deco_path))
    # 9-4-1) 검단로445번길(종다리공원 서쪽 옆길)을 방향별 1차로로 (2026-11-11)
    from env import roundabout_side_one_lane as side1
    info.update(side1.apply(net_path, deco_path))

    # 9-5) 검단사거리역 북서쪽 상가 골목 (일방통행 포함) + 상가·공원·보도 그림 (GUI 표시 전용)
    from env import roundabout_nw_shops as nw
    info.update(nw.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(nw.apply_deco(deco_path, net_path))
    from env import roundabout_nw_buildings as nw_img
    info.update({k: v for k, v in nw_img.build(net_path).items() if k not in ('out', 'nw_sizes')})
    # 9-6) 큰 루프 안쪽 도로 (2026-11-11 네이버 지도 캡처 기준, 큰길과는 우회전 진입·진출 T자)
    from env import roundabout_interior as interior
    info.update(interior.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(interior.apply_deco(deco_path, net_path))
    # 9-6-1) 117번길 연장으로 닫힌 블록 S (드림오피스텔·명품대패·공영주차장) + 등뼈 동쪽 CU, 네이버 배치 (2026-11-11)
    #        안쪽 도로가 있어야 블록이 닫히므로 여기서 상가 그림에 덧붙인다 (안쪽 숲 그림보다 먼저)
    from env import roundabout_shop_zones as shop_zones
    info['block_s'] = shop_zones.append_block_s(net_path)
    # 9-7) 마전동 블록 골목 (완정로65번길·완정로·원당대로 삼각형, 2026-11-11 네이버 지도 기준)
    #      (2026-11-14: 골목 전부 편도 2차로 — 먼저 상가 뒤 골목이 시작하는 사거리 토막을 2차로로 넓혀 사거리만 다시 만든다)
    from env import roundabout_majeon_two_lane as mj_two
    info.update(mj_two.widen_stub(net_path))
    from env import roundabout_majeon as majeon
    info.update(majeon.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(majeon.apply_deco(deco_path, net_path))
    info.update(mj_two.fix_deco(deco_path, net_path))
    # 9-7-1) 마전동 동쪽 (원당대로 북쪽, 완정로 곡선 ~ 서곶로): 서곶로 연장 + 위쪽 변 길 + 안쪽 골목 (2026-11-12)
    from env import roundabout_majeon_east as majeon_e
    info.update(majeon_e.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(majeon_e.apply_deco(deco_path, net_path))
    # 9-7-1b) 완정로 아래 끝(원당대로 T자 바로 위) 1차로 구간을 양방향 2차로로, 원당대로로는 이중 우회전 (2026-11-14)
    from env import roundabout_wanjeong_low_two_lane as wj_low
    info.update(wj_low.apply_net(net_path))
    info.update(wj_low.apply_deco(deco_path, net_path))
    # 9-7-2) 마전동 북쪽 (마전역 동쪽, 완정로 북쪽 ~ 서곶로): 마전역 북쪽 짧은 길 연장 + 가운데 큰 길 + 안쪽 길 (2026-11-12)
    from env import roundabout_majeon_north as majeon_n
    info.update(majeon_n.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(majeon_n.apply_deco(deco_path, net_path))
    # 9-7-3) 검단사거리 북동쪽 (완정로 북쪽): 검단로 연장 + 검단고 앞 동쪽 큰길(마전역 북쪽 길 T자) + 골목 (2026-11-12)
    from env import roundabout_geomdan as geomdan
    info.update(geomdan.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(geomdan.apply_deco(deco_path, net_path))
    # 9-7-4) 완정로165번길 동쪽 빌라 순환 골목 + 왕길배수지공원 동쪽 긴 골목 (2026-11-12)
    from env import roundabout_baesuji as baesuji
    info.update(baesuji.apply_net(net_path))
    root = ET.parse(net_path).getroot()
    link.update_boundary(root)
    ET.ElementTree(root).write(net_path, encoding='utf-8', xml_declaration=True)
    info.update(baesuji.apply_deco(deco_path, net_path))
    # 9-8) 마전동 블록 간단한 건물·주요 이름·우버니공원 (GUI 표시 전용, 2026-11-12)
    from env import roundabout_majeon_buildings as majeon_img
    info['majeon_buildings'] = {k: v for k, v in majeon_img.build(net_path).items() if k != 'out'}
    from env import roundabout_interior_scenery as interior_img       # 안쪽 아파트·상가·공장·산 그림
    info.update({'interior_zones': len([k for k in interior_img.build(net_path) if k not in ('out', 'forest_polys')])})
    # 9-8-1) 왕길동 (회전교차로 북서쪽 도로 ~ 445번길, 2026-11-12 네이버 위성 지도) 아파트·학교·공원 그림
    from env import roundabout_wanggil_scenery as wanggil_img
    info['wanggil'] = {k: v for k, v in wanggil_img.build(net_path).items() if k != 'out'}
    # 9-8-2) 마전동 동쪽 상가·주택·별무리공원 그림 (2026-11-12)
    from env import roundabout_majeon_east_scenery as majeon_e_img
    info['majeon_east'] = {k: v for k, v in majeon_e_img.build(net_path).items() if k != 'out'}
    # 9-8-3) 마전동 북쪽 아파트·마전초·근린공원 그림 (2026-11-12)
    from env import roundabout_majeon_north_scenery as majeon_n_img
    info['majeon_north'] = {k: v for k, v in majeon_n_img.build(net_path).items() if k != 'out'}
    # 9-8-4) 검단사거리 북동쪽 상가·빌라·아파트·방현공원 그림 (2026-11-12)
    from env import roundabout_geomdan_scenery as geomdan_img
    info['geomdan'] = {k: v for k, v in geomdan_img.build(net_path).items() if k != 'out'}
    # 9-8-5) 165번길 동쪽 빌라·왕길배수지공원·작은 건물 (2026-11-12)
    from env import roundabout_baesuji_scenery as baesuji_img
    info['baesuji'] = {k: v for k, v in baesuji_img.build(net_path).items() if k != 'out'}
    # 9-9) 정지선 뒤 진행 방향 화살표가 없는 진입 차로에 화살표 (맵 전체, 2026-11-12)
    from env import roundabout_stop_arrows as stop_arrows
    info.update(stop_arrows.apply(deco_path, net_path))
    # 9-10) 블록 S 큰 공영주차장: 117번길 입구·117 골목 출구, 통로·주차 칸(parkingArea), 바닥·칸 선 그림, 주차 차량 (2026-11-12)
    #       블록 S 그림(9-6-1)이 블록 면을 찾은 뒤에 통로를 깐다
    from env import roundabout_parking_s as parking_s
    info.update(parking_s.apply(net_path, deco_path))
    # 9-11) 완정초·마전중 학교 주차장: 정문(입구 겸 출구) 바로 안 주차장, 학교 그림은 그 뒤로, 학교 오가는 차 주차 (2026-11-12)
    from env import roundabout_school_parking as school_parking
    info.update(school_parking.apply(net_path, deco_path))
    # 9-12) 마전동 완정로 북쪽 상가 줄(네이버 위성 지도 이식) + 뒷길·골목 + 마전어린이집(들어가자마자 주차장) (2026-11-12)
    from env import roundabout_majeon_daycare as daycare
    info.update(daycare.apply(net_path, deco_path))
    # 9-13) 마전주공아파트 아래쪽 동 4개 자리 공영주차장: 왕길초 앞길 입구, 서쪽 도로 출구(좌·우회전) (2026-11-12)
    from env import roundabout_wanggil_parking as wg_parking
    info.update(wg_parking.apply(net_path, deco_path))
    # 9-14) 마전동 ② 블록 오른쪽 두 띠(예전 맘스터치·메가커피) 공영주차장: 완정로 우회전 입구·출구 (2026-11-12)
    #       (더벤티 → 탑유도관, 두 띠 비우기는 env.roundabout_majeon_buildings 9-8 에서)
    from env import roundabout_majeon_parking2 as mj_parking2
    info.update(mj_parking2.apply(net_path, deco_path))
    # 9-14b) 마전동 ⑤ 블록: 행정복지센터 자리 → 새마을금고 주차장 (남쪽 큰길 서행 우회전 진출입) (2026-11-14)
    from env import roundabout_majeon_parking5 as mj_parking5
    info.update(mj_parking5.apply(net_path, deco_path))
    # 9-15) 동아아파트 아래쪽 동 + 태화프라자 상가 자리 아파트 주차장: 서곶로 입구(남행 우회전·북행 좌회전)·출구(우회전) (2026-11-13)
    from env import roundabout_donga_parking as donga_parking
    info.update(donga_parking.apply(net_path, deco_path))
    # 9-16) 롯데마트 줄이고 3+3 막다른 길 끝을 주차장 앞 네거리로: 우회전·직진·좌회전으로 세 통로에 (2026-11-13)
    from env import roundabout_lotte_parking as lotte_parking
    info.update(lotte_parking.apply(net_path, deco_path))
    # 9-17) 검단피오레대주1차: 막다른 짧은 골목(SW_final_right) 끝을 주차장 앞 네거리로, 골목 두 차로가 입구·출구 (2026-11-13)
    from env import roundabout_fiore_parking as fiore_parking
    info.update(fiore_parking.apply(net_path, deco_path))
    # 9-17b) 왕길동 아래 블록: 왕길초 삭제 → 검단1차풍림아이원·검단원흥아파트 주차장 (왕길초 앞길 진출입) (2026-11-14)
    from env import roundabout_wanggil_south_parking as wg_s_parking
    info.update(wg_s_parking.apply(net_path, deco_path))
    # 9-17c) 왕길유승한내들아파트 이식 + 단지 주차장, 단봉로 정문(좌회전 진출 → 회전교차로 서쪽 아래 진입) (2026-11-14)
    from env import roundabout_yuseung as yuseung
    info.update(yuseung.apply(net_path, deco_path))
    # 9-17d) 회전교차로 서쪽 아래 합류부 신호 (단봉로 좌회전 보호, 정문 신호와 같은 65초 주기) (2026-11-14)
    from env import roundabout_w_merge_signal as w_merge
    info.update(w_merge.apply_net(net_path))
    info.update(w_merge.apply_deco(deco_path, net_path))
    # 9-18) 어린이보호구역 (완정초·마전중 앞 완정로, 주 출입문 반경 300m; 마전초 구역은 2026-11-14 지움): 제한 속도 30 km/h + 적색 포장·노면 표시·표지·단속 카메라 (2026-11-13)
    from env import roundabout_school_zone as school_zone
    info.update(school_zone.apply(net_path, deco_path))
    # 9-19) 왕길119안전센터 소방차·구급차 출동 경로·주기 (엣지 이름이 다 정해진 뒤) (2026-11-14)
    from env import roundabout_emergency as emergency
    info.update(emergency.write(net_path))
    # 9-19b) 큰손왕만두 오른쪽 상가 골목(D1) 편도 2차로 — 불법 주정차 구간 (2026-11-15)
    from env import roundabout_shop_d1_two_lane as shop_d1
    info.update(shop_d1.apply(net_path, deco_path))
    # 9-20) 늘어나는 차로가 늦게 시작하는 엣지: 옆 차로 시작점에서 비스듬히 시작 (차로 바꿀 때 앞으로 튀는 문제, 2026-11-14)
    from env import roundabout_lane_start_fix as lane_start
    info.update(lane_start.apply(net_path))
    # 9-20a) 서곶로 × 원당대로 사거리 동행 좌회전 전용 차로 늘리기: 안전지대를 짧게, 좌회전 차로 24 → 약 34 m (2026-11-15)
    from env import roundabout_sw_pocket as sw_pocket
    info.update(sw_pocket.apply(net_path, deco_path))
    info.update({f'ev2_{k}': v for k, v in emergency.write(net_path).items()})   # 긴급차 경로에 새 토막 반영
    # 9-20b) 서곶로 × 원당대로 사거리(SW_new_cross) 신호 4현시 (2026-11-15, 신호 없는 큰 교차로라 교통량 1.0 에서 좌회전 갇힘)
    from env import roundabout_sw_new_signal as sw_new_signal
    info.update(sw_new_signal.apply_net(net_path))
    from env import roundabout_sw_left_only as sw_left_only     # 2026-11-17: 동행 맨 왼쪽 차로 좌회전 전용
    info.update(sw_left_only.apply_net(net_path))
    # 9-21) 맵 전체 정지선 점검(빠진 정지선 추가) + 정지선 뒤 화살표 다시 (뒤 단계에서 생긴 교차점까지) (2026-11-14)
    from env import roundabout_stopline_audit as stop_audit
    info.update(stop_audit.apply(deco_path, net_path))
    info.update({f'final_{k}': v for k, v in stop_arrows.apply(deco_path, net_path).items()})
    # 9-22) 엄마 심부름 경로 (ego 학습 경로, 주차장 다섯 곳) 다시 계산 → mom_route.json (2026-11-15)
    from env import mom_route
    info.update(mom_route.build())
    # 10) 신호 교차로 신호등 그림 (GUI 표시 전용, 신호 교차로가 바뀌면 다시 만들어야 함)
    from env import roundabout_signal_heads as heads
    heads.build(net_path)
    print({**{f'top_{k}': v for k, v in rep_top.items()}, **rep_bend, 'se_extra_m': round(rep0['D'], 3), 'black_extra_m': round(rep['D'], 3), 'red_length_m': six.LENGTH,
           'deco_moved': rep['deco_moved'], 'dash_filled': rep['dash_filled'], **info})


if __name__ == '__main__':
    main()
