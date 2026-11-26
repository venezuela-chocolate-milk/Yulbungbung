"""관측 벡터 안의 위치 (데이터의 관측 차원으로 계산). 2026-11-17: 도심 맵 기본 관측이 31 → 11칸이 되어
신호 칸 위치가 31 → 11 로 바뀌었으므로, 번호를 직접 쓰지 말고 여기서 구한다.
  [0] 내 속도 / ... 기본 관측 끝 = 신호 칸 시작 (있음, 정지선 거리/200, 적, 황, 녹)"""
from env.urban_driving import EXTRA_DIM
from env.mom_driving import MOM_DIM


def signal_index(obs_dim):
    """신호 칸 시작 번호 = 기본 관측 길이 (기본 관측 바로 뒤가 도심 관측의 신호 5칸)."""
    return int(obs_dim) - EXTRA_DIM - MOM_DIM


def object_layout(obs_dim):
    """C 관측(물체 목록)이면 (시작 번호, 물체 수, 물체당 값 수), 아니면 None. 신경망이 관측 차원으로 판단한다."""
    try:
        from env import urban_driving as ud
    except Exception:
        return None
    if getattr(ud, "OBS_KIND", None) != "objects":
        return None
    start = int(obs_dim) - MOM_DIM - ud.N_OBJ * ud.N_OBJ_F
    if start != 11 + ud.EXTRA_DIM - ud.N_OBJ * ud.N_OBJ_F:
        return None                      # 다른 맵(고속도로 등)의 관측
    return start, ud.N_OBJ, ud.N_OBJ_F
