"""규칙 기반 운전자 ("강사") — 모방학습 데이터를 만들 때만 쓴다.

정책과 똑같은 행동 [가감속 raw, 차선변경 raw] 으로만 운전하고, 학습 환경도 정책과 같은
개입 수준(마스킹 없음, 원본 차선변경 안전 게이트만)으로 둔다. 다만 판단할 때는 시뮬레이터
정보(TraCI)를 직접 본다 — 교재를 만드는 강사이므로 허용되고, 학습되는 정책은 관측만 본다.

규칙:
  - 목표 속도 = 지금 차로의 허용 속도 (회전교차로·유턴 구간처럼 낮은 곳은 그만큼)
  - 신호·양보·앞차: env/urban_driving 의 안전 규칙(mask)을 판단에 재사용해 그 속도 이하로
  - 부드럽게: 가속은 COMFORT_ACCEL 까지만, 감속은 필요한 만큼 (환경 한계 5.4 m/s²)
  - 엄마 경로: 들를 주차장 통로의 빈 칸 옆에 멈춘다 (그러면 환경이 칸에 넣었다 꺼낸다), 불법 주정차를 옆 차로로 피한다, 긴급차가 뒤에서 막히면 옆 차로로 비킨다
  - 엄마 경로: 들를 주차장 통로의 빈 칸 옆에 멈춘다 (그러면 환경이 칸에 넣었다 꺼낸다), 불법 주정차를 옆 차로로 피한다, 긴급차가 뒤에서 막히면 옆 차로로 비킨다,
    회전교차로 진입 전 감속, 옆 차로 바로 옆에 차가 있으면 차선변경 안 함
  - 차로: 경로에 맞는 차로(SUMO bestLanes)로 미리 옮긴다. 원본 안전 게이트에 막혀 못 옮기면
    차로가 끝나기 전에 속도를 줄여 틈을 만들고, 끝 STOP_BEFORE_END m 앞에서 멈춰 기다린다
"""
import os
import numpy as np

COMFORT_ACCEL = 2.5      # m/s²
# 2026-11-17 승차감: 가감속을 한 스텝(0.5 s)에 이만큼까지만 바꾼다 (= 저크 4 m/s³). 안전을 위한 제동만 예외:
#   양보·추돌 규칙이 속도를 낮추라고 하거나, 제한을 지키면 정지 계획(정지선·앞차 줄·주차 지점)보다
#   LAG_TOL m/s 넘게 빨라지면 바로 필요한 만큼 제동
# 2026-11-17 비교 시험(56판): 2.0 으로 제한하면 저크 4 초과 6.0 → 3.7% 로 조금 줄었지만 완주 96.4 → 91.1%,
#   충돌 1.8 → 5.4% 로 나빠져 끔 (사용자 결정 A안). 승차감은 보상의 급조작 감점(기준 8 m/s³)으로만 다룬다.
ACCEL_STEP = float(os.environ.get("EXPERT_ACCEL_STEP", 1e9))   # m/s² per step (기본: 제한 없음)
LAG_TOL = 0.5            # m/s
STOP_DECEL = 3.0         # 신호·양보로 멈출 때 계획하는 제동 (m/s², 환경 한계 5.4 보다 여유 있게)
STOP_MARGIN = 1.5        # 정지선 앞 여유 (m)
SLOW_ZONE = 120.0        # 경로 차로로 못 옮긴 채 차로 끝까지 이 거리(m) 안이면 감속 시작
STOP_BEFORE_END = 16.0   # 그래도 못 옮기면 차로 끝 이 거리 앞에서 정지 (정지선 12 m 안은 차선변경 금지라 그보다 앞에서 기다림)
SHORT_EDGE = 40.0       # 이보다 짧은 엣지에서는 차로를 바꿀 수 없다고 보고 앞 엣지에서 미리 옮긴다
EV_LC_COOLDOWN_S = 6.0  # 긴급차에 비켜 차로를 바꾼 뒤 다시 비키기까지 최소 시간
RING_GAP_S = 3.0        # 회전 차가 "내가 진입선에 닿는 시각 + 이 시간" 안에 진입부에 오면 양보
MERGE_APPROACH_V = 5.0  # 그 차로 시작점 50 m 앞부터 이 속도(m/s) 이하로
MERGE_ROOM = 7.0        # 좌회전 차로 시작점에 이만큼 비어 있어야 들어감 (차 길이 5 m + 여유; 들어가자마자 서므로 앞 간격은 차로 시작점 기준 ~6 m)
QUEUE_GAP = 4.0         # 옆 차로 줄 맨 뒤 차와 띄우는 거리 (SUMO 최소 간격 2.5 m + 여유 — 이보다 붙으면 끼어들 틈으로 안 봄)


RING_SLOW_D = 60.0       # 회전교차로 진입로 끝 이 거리부터 감속
RING_ENTRY_V = 6.0       # 진입로 끝 15 m 안 최고 속도 (m/s)
PARKED_LOOK = 35.0       # 이 거리 안 앞에 불법 주정차 차가 있으면 옆 차로로
PARK_AT = 4.0            # 주차 구역 시작에서 이만큼 들어가 정차
STOP_RANGE_EXPERT = 150.0
HARD_DECEL = 5.0         # 황색에서 멈추기로 했거나 늦게 본 적색: 강하게 제동 (환경 한계 5.4)
# 2026-11-21 공통 수정 4~7 (강사만; 학생 관측·안전 마스킹과 무관 — 컨설팅 G항)
#   4: 신호·양보 정지를 편한 감속으로 미리 시작 (마지막 순간 최대 제동 → 학생이 평균내서 약하게 따라 하던 문제)
SIG_DECEL = float(os.environ.get("EXPERT_SIG_DECEL", 2.0))      # 신호 정지 계획 감속 (m/s²); 늦게 봤으면 STOP_DECEL → HARD_DECEL
YIELD_BRAKE = float(os.environ.get("EXPERT_YIELD_BRAKE", 2.5))  # 양보 정지 계획 감속 (m/s², 예전 STOP_DECEL 3.0)
#   5: 황색: 켜지는 순간 "STOP_DECEL 로 설 수 있으면 정지, 못 서면 통과"를 한 번만 정함 (적색까지 남은 시간은 안 씀)
#      못 서는 경우 정지선까지 시간 d/v < v/(2·STOP_DECEL) ≤ 16.7/6 = 2.8 s < 황색 3~4 s → 적색 전에 통과
#   6: 주차 칸이 붙은 차로가 들어오는 교차로 근처에서는 서행 (주차 칸에서 막 나오는 차는 실제로도 갑자기 나타남)
PARK_JUNC_V = float(os.environ.get("EXPERT_PARK_JUNC_V", 3.0))  # m/s
PARK_JUNC_D = 15.0                                               # 그 교차로까지 이 거리(m) 안 또는 교차로 안
#   7: 멈춰 있거나 느릴 때 출발 판단은 틈을 더 넉넉히 (SE 삼각 모퉁이: 아슬아슬하게 출발했다가 급제동)
START_GAP_EXTRA = float(os.environ.get("EXPERT_START_GAP", 1.0))  # s (속도 2 m/s 미만일 때 양보 시간창에 더함)


def _stop_speed(d, b, dt):
    """거리 d 앞에서 멈출 수 있는 최대 속도 (반응 지연 1스텝 포함)."""
    d = max(d, 0.0)
    return max(0.0, -b * dt + np.sqrt((b * dt) ** 2 + 2.0 * b * d))


class RuleBasedExpert:
    def __init__(self, env, target_speed_frac=1.0):
        self.env = env
        self.frac = float(target_speed_frac)
        self._stopping_for = None
        self._ev_lc_until = -1

    def reset(self):
        self._stopping_for = None
        self._yellow_dec = {}
        self._park_junc = None
        self._a_prev = 0.0
        self._ev_lc_until = -1

    def _side_clear(self, direction, front=4.0, rear=8.0):
        import traci
        ego = self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        if not lane or lane.startswith(":"):
            return True
        # 신호 정지선 12 m 안에서는 바꾸지 않는다 (실제 도로: 교차로 앞 실선 구간). 도로 토막 끝이 아니라
        # 정지선까지 거리로 판단 (짧은 토막이 이어지는 좌회전 차로 앞에서 영영 못 옮기던 문제, 2026-11-15)
        nt = self.env.urban._next_tls() if getattr(self.env, "urban", None) else None
        if nt and nt[1] < 12.0:
            return False
        # 막힌 줄에 끼어들 때(내가 거의 서 있을 때)는 실제 운전처럼 작은 틈(뒤 3.5 m)에도 들어간다
        # (좌회전 대기 줄은 차 사이가 2~3 m 라 8 m 를 기다리면 영영 못 들어감, 2026-11-15)
        if traci.vehicle.getSpeed(ego) < 2.0:
            rear, front = 3.5, 1.0
        # 다른 엣지(앞뒤 토막)에 걸친 옆 차도 SUMO 이웃 조회로 확인 (mode: bit0 오른쪽, bit1 앞)
        side = 0 if direction > 0 else 1
        try:
            for mode in (side, side | 2):
                for vid, gap in traci.vehicle.getNeighbors(ego, mode):
                    if gap < (rear if not mode & 2 else front):
                        return False
        except traci.TraCIException:
            pass
        edge, k = lane.rsplit("_", 1)
        target = f"{edge}_{int(k) + direction}"
        pos, L = traci.vehicle.getLanePosition(ego), traci.vehicle.getLength(ego)
        try:
            for v in traci.lane.getLastStepVehicleIDs(target):
                d = traci.vehicle.getLanePosition(v) - pos           # 상대 앞범퍼 - 내 앞범퍼
                # 뒤 차는 차선변경 시간(약 1~2초) 동안 다가올 거리만큼 더 비어 있어야 한다
                back = rear + 2.0 * max(traci.vehicle.getSpeed(v) - traci.vehicle.getSpeed(ego), 0.0)
                if -(L + back) < d < traci.vehicle.getLength(v) + front:
                    return False
        except traci.TraCIException:
            return False
        return True

    @staticmethod
    def _parked_ahead(lane, pos, look=60.0):
        import traci
        try:
            for v in traci.lane.getLastStepVehicleIDs(lane):
                if v.startswith("illegal_") and 0.0 < traci.vehicle.getLanePosition(v) - pos < look:
                    return True
        except traci.TraCIException:
            pass
        return False

    def _queue_tail(self, direction, look=80.0):
        """옮겨 갈 옆 차로에 서 있는 줄(좌회전 대기 등)의 맨 뒤 차 뒤범퍼까지 거리 (내 앞범퍼 기준).
        옆 줄이 없거나 이미 내 옆에 차가 붙어 있으면 None. 지금 엣지와 경로상 다음 엣지의 옆 차로를 본다."""
        import traci
        env, ego = self.env, self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        if not lane or lane.startswith(":"):
            return None
        edge, k = lane.rsplit("_", 1)
        tgt = int(k) + direction
        pos, L = traci.vehicle.getLanePosition(ego), traci.vehicle.getLength(ego)
        cands = [(f"{edge}_{tgt}", 0.0)]
        ri = traci.vehicle.getRouteIndex(ego)
        route = env.ego_route_edges
        if 0 <= ri < len(route) - 1:
            cands.append((f"{route[ri + 1]}_{tgt}", traci.lane.getLength(lane)))
        tail = None
        for tl, off in cands:
            e_, i_ = tl.rsplit("_", 1)
            if not (0 <= int(i_) < traci.edge.getLaneNumber(e_)):
                continue
            try:
                vids = traci.lane.getLastStepVehicleIDs(tl)
            except traci.TraCIException:
                continue
            for v in vids:
                if traci.vehicle.getSpeed(v) > 2.0 or v.startswith("illegal_") or traci.vehicle.isStopped(v):
                    continue                     # 움직이는 차 / 불법 주정차·정차 차는 줄이 아님
                rear = off + traci.vehicle.getLanePosition(v) - traci.vehicle.getLength(v) - pos
                if rear < -L - 1.0 or rear > look:
                    continue                     # 이미 지나친 차 / 너무 먼 차
                tail = rear if tail is None else min(tail, rear)
        if tail is None or tail < 0.5:
            return None                          # 옆에 이미 붙어 있음 → 줄 뒤에 맞출 수 없음
        return tail

    def _upcoming_merge(self, horizon=4, near=120.0):
        """앞으로 지날 엣지 중 "지금 차로를 그대로 따라가면 다음 경로로 못 가고, 그 엣지에서 옆 차로로 옮겨야
        하는" 첫 엣지를 찾는다 (예: 좌회전 차로가 따로 시작하는 엣지). (그 엣지 시작까지 거리, 옮겨 갈 차로 ID) 또는 None."""
        import traci
        env, ego = self.env, self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        if not lane or lane.startswith(":"):
            return None
        ri = traci.vehicle.getRouteIndex(ego)
        route = env.ego_route_edges
        cur = lane
        for k in range(ri, min(ri + horizon, len(route) - 2)):
            nxt, nxt2 = route[k + 1], route[k + 2]
            land = [l[0] for l in traci.lane.getLinks(cur) if l[0].rsplit("_", 1)[0] == nxt]
            if not land:
                return None
            j = int(land[0].rsplit("_", 1)[1])
            if any(l[0].rsplit("_", 1)[0] == nxt2 for l in traci.lane.getLinks(land[0])):
                cur = land[0]
                continue
            for dd in (1, -1):
                if 0 <= j + dd < traci.edge.getLaneNumber(nxt) and \
                        any(l[0].rsplit("_", 1)[0] == nxt2 for l in traci.lane.getLinks(f"{nxt}_{j + dd}")):
                    # 차로를 바꿔야 하는 엣지가 짧으면(SHORT_EDGE 미만, 신호 앞 짧은 토막) 거기서는 못 바꾸므로,
                    # 그 차로가 이어지는 앞 엣지들로 거슬러 올라가 "그 차로가 시작하는 엣지"를 목표로 잡는다
                    # (서곶로 좌회전 차로 = 20.1 + 16.0 + 7.6 m 세 토막, 2026-11-16)
                    m, tj = k + 1, j + dd
                    while (m - 1 > ri and traci.lane.getLength(f"{route[m]}_0") < SHORT_EDGE
                           and tj < traci.edge.getLaneNumber(route[m - 1])):
                        m -= 1
                    tgt = route[m]
                    try:
                        d = traci.vehicle.getDrivingDistance(ego, tgt, 0.0)
                    except traci.TraCIException:
                        return None
                    return (d, f"{tgt}_{tj}") if 0 <= d < near else None
            return None
        return None

    def _ring_traffic(self, edge, t_window=4.0):
        """edge 끝의 회전교차로 진입부를 t_window 초 안에 지나갈 회전 차가 있는지.
        회전 차마다 자기 경로를 따라 "진입부 다음 회전 차로" 시작까지 남은 주행 거리로 판단한다 (진입부 앞 회전 차로
        토막이 0.2 m 처럼 짧아 그 토막만 보면 놓침, 2026-11-16). 그 전에 빠져나가는 차는 경로에 없어 저절로 빠진다."""
        import traci
        ego = self.env.ego_id
        try:
            junc = traci.edge.getToJunction(edge)
            outs = [e for e in traci.junction.getOutgoingEdges(junc) if e.startswith("ring")]
            if not outs:
                return False
            out = outs[0]
            jx, jy = traci.junction.getPosition(junc)
            for v in traci.vehicle.getIDList():
                if v == ego:
                    continue
                r = traci.vehicle.getRoadID(v)
                if not (r.startswith("ring") or r.startswith(":ring")) or r == out:
                    continue
                x, y = traci.vehicle.getPosition(v)
                if (x - jx) ** 2 + (y - jy) ** 2 > 60.0 ** 2:
                    continue
                d = traci.vehicle.getDrivingDistance(v, out, 0.0)
                if not 0.0 <= d < 80.0:
                    continue                      # 이 진입부를 지나지 않음 (먼저 빠져나감)
                sv = max(traci.vehicle.getSpeed(v), 0.5)
                if d < 12.0 or d / sv < t_window:
                    return True
        except traci.TraCIException:
            return False
        return False

    @staticmethod
    def _lane_room(lane_id):
        """차로 시작점부터 그 차로 맨 뒤 차의 뒤범퍼까지 빈 거리 (차가 없으면 차로 길이)."""
        import traci
        room = traci.lane.getLength(lane_id)
        for v in traci.lane.getLastStepVehicleIDs(lane_id):
            room = min(room, traci.vehicle.getLanePosition(v) - traci.vehicle.getLength(v))
        return room

    def _next_edge_need(self, near=60.0):
        """지금 차로로 다음 엣지에 들어가면 그 다음 엣지로 못 가고 옆 차로(+1 왼쪽 / -1 오른쪽)로 옮겨야 하는지"""
        import traci
        env, ego = self.env, self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        if not lane or lane.startswith(":"):
            return 0
        if traci.lane.getLength(lane) - traci.vehicle.getLanePosition(ego) > near:
            return 0
        ri = traci.vehicle.getRouteIndex(ego)
        route = env.ego_route_edges
        if not (0 <= ri < len(route) - 2):
            return 0
        nxt, nxt2 = route[ri + 1], route[ri + 2]
        land = [l[0] for l in traci.lane.getLinks(lane) if l[0].rsplit("_", 1)[0] == nxt]
        if not land:
            return 0

        def ok(ln):
            e_, i_ = ln.rsplit("_", 1)
            if not (0 <= int(i_) < traci.edge.getLaneNumber(e_)):
                return False
            try:
                return any(l[0].rsplit("_", 1)[0] == nxt2 for l in traci.lane.getLinks(ln))
            except traci.TraCIException:
                return False
        j = int(land[0].rsplit("_", 1)[1])
        if ok(land[0]):
            return 0
        for d in (1, -1):
            if ok(f"{nxt}_{j + d}"):
                return d
        return 0

    def _lane_need(self):
        """(옮겨야 할 차로 수 (+왼쪽), 지금 차로로 경로를 따라 갈 수 있는 남은 거리)"""
        import traci
        env, ego = self.env, self.env.ego_id
        lane = traci.vehicle.getLaneID(ego)
        if not lane or lane.startswith(":"):
            return 0, 1e9
        pos = traci.vehicle.getLanePosition(ego)
        for bl in traci.vehicle.getBestLanes(ego):
            if bl[0] == lane:
                return int(bl[3]), float(bl[1]) - pos
        return 0, 1e9

    def act(self):
        import traci
        env, ego = self.env, self.env.ego_id
        if ego not in traci.vehicle.getIDList():
            return np.zeros(2, dtype=np.float32)
        dt = env.step_length
        v = traci.vehicle.getSpeed(ego)
        v_des = min(traci.vehicle.getAllowedSpeed(ego), env.vmax) * self.frac

        # 차로: 경로에 맞는 차로로
        need, left = self._lane_need()
        lc = 0.0
        if need != 0:
            lc = 1.0 if need > 0 else -1.0
            if left < SLOW_ZONE:                     # 못 옮긴 채 차로 끝이 다가옴 → 감속해 틈 만들기
                v_des = min(v_des, max(3.0, v_des * left / SLOW_ZONE))
            stop_d = left - STOP_BEFORE_END           # 끝 앞에서 멈춰 기다리기
            # 이 엣지 다음이 짧은 토막(SHORT_EDGE 미만)이면 거기서는 못 옮기므로 이 엣지 끝에서 기다린다
            # (서곶로 좌회전 차로: 20.1 m 토막에서 못 옮기고 16 m·7.6 m 토막으로 밀려 들어가 갇히던 문제, 2026-11-16)
            lane_c = traci.vehicle.getLaneID(ego)
            ri_c = traci.vehicle.getRouteIndex(ego)
            tgt_i = (traci.vehicle.getLaneIndex(ego) + (1 if need > 0 else -1)) if lane_c and not lane_c.startswith(":") else -1
            # 옮겨 갈 차로가 이 엣지에서 새로 시작하는 차로(좌회전 차로처럼 앞 엣지에 없는 차로)일 때만
            new_lane = (ri_c >= 1 and tgt_i >= traci.edge.getLaneNumber(env.ego_route_edges[ri_c - 1]))
            if (new_lane and 0 <= ri_c < len(env.ego_route_edges) - 1
                    and traci.lane.getLength(f"{env.ego_route_edges[ri_c + 1]}_0") < SHORT_EDGE):
                stop_d = min(stop_d, traci.lane.getLength(lane_c) - traci.vehicle.getLanePosition(ego) - 1.0)
            v_des = min(v_des, np.sqrt(max(0.0, 2 * 3.0 * max(stop_d, 0.0))))
            # 옮겨 갈 차로에 줄이 서 있으면 그 줄 맨 뒤 차 바로 뒤(옆 차로)에서 멈춰 옆이 빈 틈으로 들어간다
            # (줄 옆을 지나치면 옆 차들이 틈을 안 줘서 끝까지 못 들어감 — 서곶로 좌회전 차로, 2026-11-15)
            tail = self._queue_tail(int(lc))
            if tail is not None and not self._side_clear(int(lc)):
                v_des = min(v_des, _stop_speed(max(tail - QUEUE_GAP, 0.0), STOP_DECEL, dt))
        else:
            # 다음 엣지에서 옆 차로로 옮겨야 하는데 (좌회전 차로가 다음 토막에서 시작) 그 차로에 줄이 이미
            # 시작점까지 차 있으면 이 엣지 끝(줄 맨 뒤) 앞에서 기다린다
            # → 실제 운전처럼 좌회전 차로 시작점 앞에서 기다렸다가, 줄 끝에 한 대 들어갈 자리가 나면
            #   차로 시작점에서 바로 옮긴다 (시작점에서는 뒤에서 오는 좌회전 차가 없어 틈을 뺏기지 않음, 2026-11-16)
            up = self._upcoming_merge()
            if up is not None:
                d_start, tgt_lane = up
                if d_start < 50.0:
                    v_des = min(v_des, MERGE_APPROACH_V)    # 옮겨 갈 차로가 곧 시작: 천천히 다가가 시작점에서 바로 옮긴다
                if self._lane_room(tgt_lane) < MERGE_ROOM:
                    v_des = min(v_des, _stop_speed(max(d_start - 1.0, 0.0), STOP_DECEL, dt))

        # 엄마 경로 과제 (env/mom_driving.py)
        ev_lc_issued = False
        mom = getattr(env, "mom", None)
        if mom is not None:
            lane_id = traci.vehicle.getLaneID(ego)
            internal = not lane_id or lane_id.startswith(":")
            idx = traci.vehicle.getLaneIndex(ego) if not internal else 0
            n_lanes = traci.edge.getLaneNumber(traci.vehicle.getRoadID(ego)) if not internal else 1
            # 긴급차가 뒤에서 막히면 옆 차로로 비켜 준다 (오른쪽 우선) — 경로 차로보다 먼저
            if mom.last.get("ev_block"):
                ev_lc = -1.0 if idx > 0 else 1.0
                # 교차로 앞에서 경로상 반대쪽으로 꼭 옮겨야 할 때 (예: 좌회전 차로로 가는 중)는 그 반대로 비키지 않는다
                # (오른쪽으로 비켰다가 좌회전 차로까지 두 칸을 다시 건너야 해서 막히던 문제, 2026-11-16)
                # 비켜 갈 차로 앞에 불법 주정차가 있으면 그 차로로는 가지 않는다 (오갔다 하다 주정차 차에 부딪히던 문제)
                tgt_idx = idx + int(ev_lc)
                blocked = (not internal and 0 <= tgt_idx < n_lanes and
                           self._parked_ahead(f"{traci.vehicle.getRoadID(ego)}_{tgt_idx}",
                                              traci.vehicle.getLanePosition(ego), look=60.0))
                # 비켜 갈 차로로는 다음 경로 도로로 못 가는데 교차로가 150 m 안이면 비키지 않는다
                # (우회전 차로에서 왼쪽으로 비켰다가 우회전을 못 해 경로 이탈하던 문제, 2026-11-16)
                d_end = (traci.lane.getLength(lane_id) - traci.vehicle.getLanePosition(ego)) if not internal else 1e9
                if not blocked and not internal and 0 <= tgt_idx < n_lanes and d_end < 150.0:
                    ri_ = traci.vehicle.getRouteIndex(ego)
                    rt = env.ego_route_edges
                    if 0 <= ri_ < len(rt) - 1:
                        tl_ = f"{traci.vehicle.getRoadID(ego)}_{tgt_idx}"
                        if not any(l[0].rsplit("_", 1)[0] == rt[ri_ + 1] for l in traci.lane.getLinks(tl_)):
                            blocked = True
                # 긴급차 때문에 한 번 옮겼으면 EV_LC_COOLDOWN_S 동안은 다시 옮기지 않는다 (긴급차가 따라 옮겨 와서
                # 오른쪽·왼쪽을 번갈아 오가던 문제, 2026-11-16)
                if env.step_count < self._ev_lc_until:
                    blocked = True
                if not blocked and not (need != 0 and np.sign(need) != np.sign(ev_lc) and left < 150.0):
                    lc = ev_lc
                    ev_lc_issued = True
            elif not internal:
                # 불법 주정차(비상등 켠 정차 차)가 앞에 있으면 옆 차로로 피한다
                lead = traci.vehicle.getLeader(ego, PARKED_LOOK)
                # 같은 도로 위의 주정차 차만 (교차로 너머 다른 도로의 차를 보고 교차로 앞에서 차로를 바꾸면
                # 경로로 못 가는 차로에 갇힘, 2026-11-16)
                if (lead and lead[0] and lead[0].startswith("illegal_") and idx + 1 < n_lanes
                        and traci.vehicle.getRoadID(lead[0]) == traci.vehicle.getRoadID(ego)):
                    lc = 1.0
                elif lc < 0 and idx > 0 and self._parked_ahead(f"{traci.vehicle.getRoadID(ego)}_{idx - 1}",
                                                                 traci.vehicle.getLanePosition(ego)):
                    lc = 0.0                            # 경로 차로(오른쪽)가 주정차로 막혀 있으면 아직 옮기지 않음
            # 들를 주차장: 통로의 주차 구역 안 (시작 + PARK_AT m) 에 멈춘다
            tgt = mom.next_stop_target()
            if tgt is not None:
                ds, de, on = tgt                         # ds = 고른 빈 칸 옆까지 거리
                d_stop = ds
                if d_stop < STOP_RANGE_EXPERT:
                    v_stop = _stop_speed(max(d_stop - 0.3, 0.0), STOP_DECEL, dt)
                    if on and abs(d_stop) < 1.0:
                        v_stop = 0.0                    # 통로 차로의 주차 구역 안: 멈춘다
                    elif not on:
                        v_stop = max(v_stop, 1.5)       # 아직 통로 차로 전 (거리 추정 오차): 천천히라도 들어간다
                    v_des = min(v_des, v_stop)

        # 회전교차로 진입: 진입로 끝 RING_SLOW_D m 안에서는 RING_ENTRY_V 까지 줄여 양보 판단할 여유를 둔다
        lane_now = traci.vehicle.getLaneID(ego)
        if lane_now and not lane_now.startswith(":") and not lane_now.startswith("ring"):
            ri = traci.vehicle.getRouteIndex(ego)
            route = env.ego_route_edges
            if 0 <= ri < len(route) - 1 and route[ri + 1].startswith("ring"):
                d_end = traci.lane.getLength(lane_now) - traci.vehicle.getLanePosition(ego)
                if d_end < RING_SLOW_D:
                    v_des = min(v_des, np.sqrt(RING_ENTRY_V ** 2 + 2 * 2.0 * max(d_end - 15.0, 0.0)))
                # 회전 중인 차 우선 (도로교통법): 진입선 15 m 안에서 회전 차로로 RING_YIELD_D m 안에 다가오는
                # 차나 진입 교차로 안을 지나는 회전 차가 있으면 진입선 앞에서 기다린다 (2026-11-16: 양보 판단이
                # 바로 앞 교차로 안쪽의 회전 차를 놓쳐 진입하다 부딪히던 문제)
                committed = d_end < v * v / (2 * STOP_DECEL) + 0.5       # 이미 진입선 앞에 설 수 없음 → 그대로 진입
                if d_end < 30.0 and not committed and \
                        self._ring_traffic(traci.vehicle.getRoadID(ego), d_end / max(v, 1.0) + RING_GAP_S):
                    v_des = min(v_des, _stop_speed(max(d_end - 0.5, 0.0), STOP_DECEL, dt))

        # 신호 (실제 운전처럼): 적색이면 정지선 앞에서 멈춤. 황색이면 적색 전에 정지선을 넘을 수
        # 있을 때만 진행하고, 아니면 정지 (한 번 멈추기로 하면 녹색까지 유지 → 판단이 오락가락하지 않게)
        nt = env.urban._next_tls()
        if nt:
            tl, dist, st = nt[0], nt[1], nt[2]
            d = dist - STOP_MARGIN
            if st not in "yY":
                self._yellow_dec.pop(tl, None)
            if st in "GgOo":
                self._stopping_for = None
            elif st in "rRs" or self._stopping_for == tl:
                self._stopping_for = tl
                if v * v / (2 * SIG_DECEL) <= d + 2:
                    b_sig = SIG_DECEL                   # 편한 감속으로 미리 (공통 수정 4)
                elif v * v / (2 * STOP_DECEL) <= d + 2:
                    b_sig = STOP_DECEL
                else:
                    b_sig = HARD_DECEL                  # 늦게 본 적색
                v_des = min(v_des, _stop_speed(d, b_sig, dt))
            else:                                       # 황색 (공통 수정 5): 켜지는 순간 한 번만 결정
                if tl not in self._yellow_dec:
                    self._yellow_dec[tl] = "stop" if v * v / (2 * STOP_DECEL) <= d else "go"
                if self._yellow_dec[tl] == "go":
                    v_des = max(v_des, v)               # 못 서므로 속도 유지하며 통과
                else:
                    self._stopping_for = tl
                    b_sig = SIG_DECEL if v * v / (2 * SIG_DECEL) <= d + 2 else STOP_DECEL
                    v_des = min(v_des, _stop_speed(d, b_sig, dt))
        else:
            self._stopping_for = None

        # 공통 수정 6: 주차 칸이 붙은 차로가 들어오는 교차로 근처 서행
        if self._park_junc is None:
            self._park_junc = set()
            try:
                for pa in traci.parkingarea.getIDList():
                    ln = traci.parkingarea.getLaneID(pa)
                    for lk in traci.lane.getLinks(ln):
                        if lk[4]:
                            self._park_junc.add(lk[4][1:].rsplit("_", 2)[0])
            except traci.TraCIException:
                pass
        if self._park_junc:
            ln_now = traci.vehicle.getLaneID(ego)
            jn, dj = None, 1e9
            if ln_now.startswith(":"):
                jn, dj = ln_now[1:].rsplit("_", 2)[0], 0.0
            else:
                nl = traci.vehicle.getNextLinks(ego)
                if nl and nl[0][4]:
                    jn = nl[0][4][1:].rsplit("_", 2)[0]
                    dj = traci.lane.getLength(ln_now) - traci.vehicle.getLanePosition(ego)
            if jn in self._park_junc and dj < PARK_JUNC_D:
                v_des = min(v_des, max(PARK_JUNC_V, _stop_speed(0.0, STOP_DECEL, dt)) if dj <= 0 else
                            max(PARK_JUNC_V, np.sqrt(PARK_JUNC_V ** 2 + 2 * SIG_DECEL * max(dj - 3.0, 0.0))))

        # 양보·앞차 (환경의 안전 규칙을 판단에 재사용; 환경은 마스킹 없이 그대로 실행)
        v_safe, _ = env.urban.mask(v, v_des, brake=YIELD_BRAKE, margin=STOP_MARGIN, skip_signal=True,
                                   start_extra=START_GAP_EXTRA)
        v_cmd = min(v_des, v_safe) if v_safe < v_des else v_des
        # 교차로 안 최소 속도 규칙은 mask 가 v_cmd 를 올려서 돌려줄 수 있음
        if v_safe > v_des:
            v_cmd = v_safe

        # 옆 차로 바로 옆(앞뒤로 겹치는 자리)에 차가 있으면 차선변경 명령을 내지 않는다
        # (원본 안전 게이트는 나란히 붙은 차를 놓칠 때가 있다 — 강사는 거울을 보고 바꾼다)
        if lc != 0 and not self._side_clear(int(lc)):
            lc = 0.0
        if lc != 0 and ev_lc_issued:
            self._ev_lc_until = env.step_count + int(EV_LC_COOLDOWN_S / dt)   # 실제로 비켜 옮길 때만
        a = (v_cmd - v) / dt
        a = min(a, COMFORT_ACCEL)
        # 승차감: 가감속 변화 제한 (위 ACCEL_STEP). 올리는 쪽은 항상, 내리는(더 세게 밟는) 쪽은 급하지 않을 때만
        a_prev = getattr(self, "_a_prev", 0.0)
        if a > a_prev + ACCEL_STEP:
            a = a_prev + ACCEL_STEP
        elif a < a_prev - ACCEL_STEP:
            urgent = (v_safe < v_des - 0.05) or (v + (a_prev - ACCEL_STEP) * dt - v_cmd > LAG_TOL)
            if not urgent:
                a = a_prev - ACCEL_STEP
        a = float(np.clip(a, -env.max_decel, env.max_accel))
        self._a_prev = max(a, -v / dt)                 # 실제로 적용되는 가감속 (속도는 0 아래로 안 감)
        raw = a / env.max_accel if a >= 0 else a / env.max_decel
        return np.array([np.clip(raw, -1.0, 1.0), lc], dtype=np.float32)
