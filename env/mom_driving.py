"""엄마 심부름 경로 주행 과제: 주차장 정차 · 긴급차 양보 · 어린이보호구역 단속 카메라 (2026-11-15).

env/sumo_env.py 가 회전교차로 맵 + 경로에 "stops" 가 있을 때 쓴다. 정책 행동에는 손대지 않는다
(SUMO 쪽 자동 정차·자동 양보 없음). 판정·관측·보상만 한다.

1) 주차장 정차 (경로의 stops, env/mom_route.py)
   - 들를 곳마다: 그 주차 구역(parkingArea) 통로 차로의 [start, end] 안에서 HOLD_S 초 동안 계속 멈춰 있으면(0.3 m/s 미만) 정차 성공
     → 보상 park_bonus. 마지막 곳(외할머니 댁)은 정차 = 완주(arrival_bonus, 종료).
   - 정차 안 하고 그 통로 엣지를 지나치면 (경로상 다음 엣지로 넘어가면) 되돌아갈 수 없으므로 실패 (missed_stop, lost_penalty, 종료).
   - 관측 3: 다음 정차 구역 시작·끝까지 거리 (경로를 따라, /STOP_RANGE), 정차한 곳 수 / 전체.

2) 긴급차 (소방차·구급차, vClass emergency)
   - 규칙 (도로교통법 제29조): 긴급차가 다가오면 진로를 양보한다. 여기서는 "긴급차가 내 뒤 같은 차로 BLOCK_DIST m 안에 있는데
     옆 차로가 있는 도로(교차로 밖)면" 길을 막고 있는 것 → 스텝마다 ev_block_penalty. 옆 차로로 비켜 주면 감점이 멈춘다.
     편도 1차로는 비킬 차로가 없으므로 감점하지 않는다 (긴급차가 뒤따라온다).
   - 만남을 늘리기: 에피소드의 inject_prob 비율은 inject_count 번 (무작위 시각, 서로 inject_min_gap_steps 이상 떨어짐)
     ego 뒤 BEHIND m 쯤에서 긴급차를 출발시켜
     ego 경로를 따라오게 한다 (주차장 앞까지, 그 뒤는 SUMO 최단 경로로 맵 끝 EV_END 까지). 왕길119안전센터 정기 출동도 그대로 있음.
     빈도는 용도별 설정 (roundabout_env_config.EMERGENCY_MODES: train 자주 / eval 원래 빈도 / ev_only 매번).
   - 배경차도 비켜 준다 (yield_background): 긴급차 앞 같은 차로 YIELD_LOOK m 안의 차는 옆 차로가 있으면 그쪽으로 바꾼다
     (오른쪽 우선, 맨 오른쪽이면 왼쪽). SUMO 사이렌 장치(bluelight)는 SUMO 1.27 이 이 맵에서 죽어서 못 쓰므로 TraCI 로 대신.
   - 관측: 별도 칸 없음 (2026-11-17). 주변 차량 관측(env/urban_driving.py)의 "긴급차 여부" 칸으로 본다. 길막기 판정은 보상에만 쓴다.

3) 어린이보호구역 단속 카메라 (env/roundabout_school_zone.py 가 구역 입구 길가에 둠, 제한 30 km/h)
   - 카메라 지점을 차로 제한속도 + CAM_TOL 초과로 지나가면 camera_penalty (한 번).
   - 관측 1: 경로상 다음 카메라까지 거리 /CAM_RANGE (없으면 1)
"""
import glob
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

STOP_RANGE = 300.0
HOLD_S = 2.0
EV_RANGE = 150.0
MIN_SPAWN_BACK = 150.0  # 학습용 긴급차는 ego 뒤 이 거리(m) 이상에서만 출발 (시야·화면 밖)
MEET_FIRST_EDGES = ('X_se_out_s3', 'X_se_out_s3_int_1', 'X_se_out_sz')   # ① 직선 (119 정기 출동과 겹침, 큰손왕만두 → 완정초등학교)
MEET_FIRST_FRAC = 0.5   # ①-1·①-2 (앞 절반) 은 정기 출동에게만 기회, ①-3 시작에서 투입
MEET_AHEAD_M = 200.0    # 예비 투입: 앞으로 이 거리까지 2차로 이상·주차장 아님·유턴 대기 아님
MEET_119_LOOK = 400.0   # ①-3 에서 정기 출동이 이 거리 안 뒤(119 경로 위)에서 오고 있으면 투입 안 함
MEET_CLOSE_BACK = (110.0, 140.0)   # ①-3 투입: 우리 뒤 이만큼 (① 안, 도로 중간 출발)
MEET_DIST = 60.0        # 11-24: '만남' = 긴급차가 ego 뒤 같은 도로 이 거리 안 (측정 results/ev_meet 과 같은 기준)
BLOCK_DIST = 50.0       # 2026-11-17: 주변 차량 관측 반경(50 m)과 맞춤
YIELD_LOOK = 60.0
CAM_RANGE = 200.0
CAM_TOL = 0.3          # m/s (약 1 km/h)
PARK_TOL = 3.0          # 빈 칸 옆 (차 가운데가 칸 위치 ± 이 m) 에 멈춰야 주차 시작
STALL_SEE = 15.0        # 2026-11-17: 빈 칸은 그 통로 안에서 이 거리 안에 있을 때만 보인다 (실제 운전자처럼)
PARK_DWELL_S = 10.0     # 칸에 머무는 시간 (시뮬레이션 s). 화면 기본 속도(스텝 0.5 s, 지연 0.1 s ≈ 5배속)에서 약 2초로 보임
EV_TYPES = ('em_fire',)      # 2026-11-17: 구급차 제외, 소방차만
EV_END = 'sg_ext_n_pd_2'    # 긴급차 정기 출동이 사라지는 맵 북쪽 끝
LOT_PREFIXES = ('pm_', 'pn_', 'pk_', 'pw_', 'pd_', 'pl_', 'pf_', 'pj_', 'mz_k', 'skw_', 'skm_', 'pu_', 'school_')
LOT_AISLES = tuple(p for p in LOT_PREFIXES if p != 'school_')   # 주차장 통로만 (학교 진입로 제외)
# 2026-11-17 컨설팅 반영: 긴급차 관측은 주변 차량 칸(env/urban_driving.py, 긴급차 여부 1칸)으로 합침
MOM_DIM = 3 + 1 + 1     # 정차 구역 3 + 가까운 빈 칸 1 (통로 안 STALL_SEE m) + 카메라 1


def _em_route(map_dir):
    """119 정기 출동 경로 (roundabout.emergency.rou.xml 의 em_route) 도로 목록."""
    f = os.path.join(map_dir, 'roundabout.emergency.rou.xml')
    try:
        for r in ET.parse(f).getroot().iter('route'):
            if r.get('id') == 'em_route':
                return r.get('edges').split()
    except (OSError, ET.ParseError):
        pass
    return []


def _lane_angle(shape, pos):
    """차로 모양의 pos 지점 진행 방향 (SUMO 각도: 북 0°, 시계 방향)."""
    acc = 0.0
    for a, b in zip(shape, shape[1:]):
        seg = math.dist(a, b)
        if acc + seg >= pos or b == shape[-1]:
            return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.0
        acc += seg
    return 0.0


def _spaces(map_dir, net):
    """roundabout.deco.xml 의 parkingArea 칸 → {구역: [(통로 차로 위 위치, x, y)]} (위치 순)."""
    import sumolib
    out = {}
    root = ET.parse(os.path.join(map_dir, 'roundabout.deco.xml')).getroot()
    for pa in root.iter('parkingArea'):
        try:
            shape = net.getLane(pa.get('lane')).getShape()
        except Exception:
            continue
        lst = []
        for sp in pa.findall('space'):
            x, y = float(sp.get('x')), float(sp.get('y'))
            p = sumolib.geomhelper.polygonOffsetWithMinimumDistanceToPoint((x, y), shape, perpendicular=False)
            # 주차된 차 방향: SUMO 는 칸 각도와 상관없이 통로 차로 방향 + 90° 로 세운다 (2026-11-15 실측, 모든 칸)
            #   → ego 도 같은 방향으로 넣어 옆 차들과 나란히
            ang = (_lane_angle(shape, p) + 90.0) % 360.0
            lst.append((p, x, y, ang, float(sp.get('length', 5.0))))
        out[pa.get('id')] = sorted(lst)
    return out


def _cameras(map_dir):
    """[(엣지, 위치)] — 카메라 POI 이름 sz_<구역>_cam_<엣지>, 위치는 차로 시작에서 SIGN_S + 12 m."""
    from env.roundabout_school_zone import SIGN_S
    out = []
    for f in glob.glob(os.path.join(map_dir, '*.add.xml')):
        for p in ET.parse(f).getroot().iter('poi'):
            m = re.match(r'sz_.+?_cam_(.+)$', p.get('id', ''))
            if m:
                out.append((m.group(1), SIGN_S + 12.0))
    return out


class MomTask:
    def __init__(self, env, cfg, net, map_dir):
        self.env = env
        self.cfg = dict(cfg)
        self.net = net
        self.rw = dict(self.cfg.get('reward', {}))
        self.cams = _cameras(map_dir)
        ev = dict(self.cfg.get('emergency', {}))
        self.inject_prob = float(ev.get('inject_prob', 0.0))
        self.inject_window = tuple(ev.get('inject_window_steps', (40, 1600)))
        self.inject_count = tuple(ev.get('inject_count', (1, 1)))      # 투입하는 에피소드에서 몇 번 (최소, 최대)
        self.inject_gap = int(ev.get('inject_min_gap_steps', 500))     # 두 번 이상이면 출발 시각 최소 간격
        self.behind = tuple(ev.get('behind_m', (80.0, 160.0)))
        self.yield_bg = bool(ev.get('yield_background', True))
        # 11-24 (사용자 승인): 만날 때까지 투입. 어떤 긴급차(119 정기 출동 포함)든 ego 뒤 같은 도로 MEET_DIST m 안에 한 번
        #   들어오면 '만남' → 추가 투입 멈춤. 못 만나면 retry_s 초마다 다시 투입. 투입차는 ego 경로를 따라오다 119 정기 출동 경로로 합류해 빠져나감.
        self.meet_until = bool(ev.get('meet_until', False))
        self.retry_steps = max(1, int(round(float(ev.get('retry_s', 150.0)) / env.step_length)))
        self.em_route = _em_route(map_dir)
        self.hold_steps = max(1, int(round(HOLD_S / env.step_length)))
        self.spaces = _spaces(map_dir, net)          # 주차 구역 → [(통로 위 위치, x, y)]
        self.dwell = float(self.cfg.get('park_dwell_s', PARK_DWELL_S))
        self.target = None
        self.park_request = None
        self._is_ev = {}
        self._n_inject = 0

    # ------------------------------------------------------------------
    def reset(self, edges, stops, route_offset, depart_pos):
        """edges: 이번 에피소드 ego 경로, stops: 경로 정의의 stops (원래 경로 순번), route_offset: 중간 출발이면 잘라낸 엣지 수."""
        import traci
        self.edges = list(edges)
        self.stops = [dict(s, route_index=s['route_index'] - route_offset) for s in stops
                      if s['route_index'] - route_offset >= 0]
        self.n_total = len(stops)
        self.k = 0
        self.hold = 0
        self.target = None
        self.park_request = None
        self.depart_pos = float(depart_pos)
        # 경로 위 각 엣지 시작 위치 (교차로 내부 경로 길이 포함, 첫 연결 기준 근사)
        cum, pos = [], 0.0
        for i, e in enumerate(self.edges):
            cum.append(pos)
            pos += self.net.getEdge(e).getLength()
            if i + 1 < len(self.edges):
                cons = self.net.getEdge(e).getConnections(self.net.getEdge(self.edges[i + 1]))
                via = cons[0].getViaLaneID() if cons else ''
                if via:
                    pos += self.net.getLane(via).getLength()
        self.cum = cum
        self.cam_pts = [(i, p) for i, e in enumerate(self.edges) for ce, p in self.cams if ce == e]
        self.cam_done = set()
        self._prev_lane_pos = None
        self._prev_ri = None
        # 이번 에피소드에 긴급차를 출발시킬 스텝들 (오름차순). 주차장 안 등으로 못 내보내면 다음 스텝에 다시 시도
        lo, hi = self.inject_window
        self.inject_times = []
        if self.inject_prob > 0 and np.random.rand() < self.inject_prob:
            n = int(np.random.randint(self.inject_count[0], self.inject_count[1] + 1))
            for _ in range(50):
                ts = sorted(int(t) for t in np.random.randint(lo, hi, size=n))
                if all(b - a >= self.inject_gap for a, b in zip(ts, ts[1:])):
                    self.inject_times = ts
                    break
            else:
                self.inject_times = ts[:1]
        self.injected = False
        self.met = False
        self.next_try = None
        if self.meet_until:
            self.inject_times = []                  # 정해진 시각 대신 '만날 때까지' (before_step)
            self.next_try = 0
            self._meet_plan()
        self.ep = dict(parked=0, missed_stop=False, ev_block_steps=0, ev_seen_steps=0, camera_tickets=0,
                       ev_injected=0, ev_met=0, ev_met_step=-1, ev_met_by='', ev_met_rp=-1.0, ev_met_edge='')
        self.last = {}
        self._is_ev = {k: v for k, v in self._is_ev.items() if k in set(traci.vehicle.getIDList())}

    # ------------------------------------------------------------------
    def _route_pos(self):
        import traci
        ego = self.env.ego_id
        ri = traci.vehicle.getRouteIndex(ego)
        lane = traci.vehicle.getLaneID(ego)
        if 0 <= ri < len(self.cum) and lane and not lane.startswith(':'):
            return ri, self.cum[ri] + traci.vehicle.getLanePosition(ego)
        if lane and lane.startswith(':') and 0 <= ri + 1 < len(self.cum):
            # 교차로 안: 다음 경로 엣지 시작까지 남은 내부 경로 길이로 (주행거리 누적 오차 없이)
            rest = traci.lane.getLength(lane) - traci.vehicle.getLanePosition(ego)
            nxt = self._internal_next(lane)
            while nxt:
                rest += traci.lane.getLength(nxt)
                nxt = self._internal_next(nxt)
            return ri, self.cum[ri + 1] - rest
        return ri, self.depart_pos + traci.vehicle.getDistance(ego)

    def _internal_next(self, lane):
        """교차로 내부 차로 다음이 또 내부 차로(좌회전 대기 지점 뒤 토막)면 그 차로."""
        try:
            ln = self.net.getLane(lane)
            for c in ln.getOutgoing():
                via = c.getViaLaneID()
                tgt = via or c.getToLane().getID()
                return tgt if tgt.startswith(':') else None
        except Exception:
            return None
        return None

    def _ev(self, vid):
        if vid not in self._is_ev:
            import traci
            try:
                self._is_ev[vid] = traci.vehicle.getTypeID(vid) in EV_TYPES
            except traci.TraCIException:
                return False
        return self._is_ev[vid]

    def ev_ids(self):
        import traci
        return [v for v in traci.vehicle.getIDList() if self._ev(v)]

    # ------------------------------------------------------------------
    def features(self):
        import traci
        env, ego = self.env, self.env.ego_id
        ri, rp = self._route_pos()
        self.last = {'ri': ri, 'rp': rp}
        f = []
        # 주차 정차
        if self.k < len(self.stops):
            s = self.stops[self.k]
            base = self.cum[s['route_index']] if s['route_index'] < len(self.cum) else rp
            dt_ = base + s['start'] - rp      # 2026-11-17: 정차 구역 시작까지 (빈 칸 위치를 골라 주지 않음)
            de = base + s['end'] - rp
            f += [float(np.clip(dt_ / STOP_RANGE, -1, 1)), float(np.clip(de / STOP_RANGE, -1, 1))]
        else:
            f += [1.0, 1.0]
        f.append(self.ep['parked'] / max(self.n_total, 1))
        # 가까운 빈 칸 (2026-11-17, 사용자 요청 (b)): 들를 구역의 통로 차로 위에 있을 때만, 차 가운데 기준
        #   앞쪽 STALL_SEE m (뒤로 PARK_TOL m 까지) 안의 가장 가까운 빈 칸까지 거리 / STALL_SEE. 안 보이면 1
        f.append(self._seen_stall_feature())
        # 긴급차
        evs = self.ev_ids()
        blocking = False
        best = None
        if evs:
            ex, ey = traci.vehicle.getPosition(ego)
            a = math.radians(traci.vehicle.getAngle(ego))
            hx, hy = math.sin(a), math.cos(a)
            v = traci.vehicle.getSpeed(ego)
            for vid in evs:
                x, y = traci.vehicle.getPosition(vid)
                d = math.hypot(x - ex, y - ey)
                if d > EV_RANGE or (best and d >= best[0]):
                    continue
                b = math.radians(traci.vehicle.getAngle(vid))
                sv = traci.vehicle.getSpeed(vid)
                vx, vy = sv * math.sin(b) - v * hx, sv * math.cos(b) - v * hy
                best = (d, (x - ex) * hx + (y - ey) * hy, (x - ex) * (-hy) + (y - ey) * hx, vx * hx + vy * hy)
            blocking = self._blocking()
            if not self.met:
                er, ep_ = traci.vehicle.getRoadID(ego), traci.vehicle.getLanePosition(ego)
                for vid in evs:
                    if traci.vehicle.getRoadID(vid) == er and 0.0 < ep_ - traci.vehicle.getLanePosition(vid) < MEET_DIST:
                        self.met = True
                        self.ep['ev_met'] = 1
                        self.ep['ev_met_step'] = int(self.env.step_count)
                        self.ep['ev_met_by'] = 'inj' if vid.startswith('ev_inj') else '119'
                        self.ep['ev_met_rp'] = float(rp)
                        self.ep['ev_met_edge'] = er
                        break
        if best:
            self.ep['ev_seen_steps'] += 1           # (관측에는 넣지 않음: 주변 차량 칸의 긴급차 여부로 봄)
        self.last['ev_block'] = blocking
        self.last['ev_near'] = best is not None
        # 단속 카메라
        dc = 1.0
        for i, p in self.cam_pts:
            if (i, p) in self.cam_done or i < ri:
                continue
            d = self.cum[i] + p - rp
            if d >= -1.0:
                dc = float(np.clip(d / CAM_RANGE, 0, 1))
                break
        f.append(dc)
        return f

    def _blocking(self):
        """긴급차가 내 뒤 같은 차로 BLOCK_DIST m 안 + 실제로 비킬 수 있는 옆 차로 자리가 있음 (교차로 밖)."""
        import traci
        ego = self.env.ego_id
        road = traci.vehicle.getRoadID(ego)
        if road.startswith(':') or traci.edge.getLaneNumber(road) < 2:
            return False
        fol = traci.vehicle.getFollower(ego, BLOCK_DIST)
        if not (fol and fol[0] and fol[1] <= BLOCK_DIST and self._ev(fol[0])):
            return False
        # 비킬 수 없는 상황은 진로 방해로 치지 않는다 (2026-11-16): 앞차 바로 뒤에 서 있는 대기 줄 안이거나,
        # 양옆 차로 모두 내 옆자리에 차가 있거나 주정차 차로 막혀 있으면 (좌회전 대기 줄 안에서 수백 스텝 감점되던 문제)
        lead = traci.vehicle.getLeader(ego, 10.0)
        if lead and lead[0] and traci.vehicle.getSpeed(ego) < 1.0 and traci.vehicle.getSpeed(lead[0]) < 1.0:
            return False
        idx, n = traci.vehicle.getLaneIndex(ego), traci.edge.getLaneNumber(road)
        pos, L = traci.vehicle.getLanePosition(ego), traci.vehicle.getLength(ego)
        for k in (idx - 1, idx + 1):
            if not 0 <= k < n:
                continue
            free = True
            for v in traci.lane.getLastStepVehicleIDs(f'{road}_{k}'):
                p = traci.vehicle.getLanePosition(v)
                if v.startswith('illegal_') and 0 < p - pos < 60.0:
                    free = False
                elif -(L + 3.0) < p - pos < traci.vehicle.getLength(v) + 3.0:   # 내 옆자리(앞뒤 3 m 여유)에 걸침
                    free = False
                if not free:
                    break
            if free:
                return True                         # 비킬 자리가 있는데 안 비킴
        return False

    # ------------------------------------------------------------------
    def before_step(self):
        """시뮬레이션 스텝 직전: 긴급차 투입(학습용) + 배경차 비켜 주기 + 주차장 통로 막는 배경차 정리."""
        if self.inject_times and self.env.step_count >= self.inject_times[0]:
            if self._try_inject():
                self.inject_times.pop(0)
        if self.meet_until and not self.met and self.env.step_count >= self.next_try and self._meet_can_inject():
            if self._try_inject():                  # 못 내보내면(교차로·주차장 안 등) 다음 스텝에 다시
                self.next_try = self.env.step_count + self.retry_steps
        if self.yield_bg:
            self._yield_background()
        # 2026-11-17 컨설팅 반영: 끼어들기 양보(ego 를 위해 배경차를 세우던 것)·주차장 통로 배경차 정리 제거

    def _try_inject(self):
        import traci
        ego = self.env.ego_id
        if ego not in traci.vehicle.getIDList():
            return False
        lane = traci.vehicle.getLaneID(ego)
        ri = traci.vehicle.getRouteIndex(ego)
        if not lane or lane.startswith(':') or ri < 0 or self.edges[ri].startswith(LOT_PREFIXES):
            return False
        close = getattr(self, '_meet_close', False)
        back = float(np.random.uniform(*(MEET_CLOSE_BACK if close else self.behind)))
        i, p = ri, traci.vehicle.getLanePosition(ego) - back
        while p < 0 and i > 0 and not self.edges[i - 1].startswith(LOT_PREFIXES):
            i -= 1
            p += self.net.getEdge(self.edges[i]).getLength()
        if close:
            # 11-24: ① 직선(①-3)에서는 학교에 들어가기 전에 따라잡도록 도로 시작이 아니라 정확히 back m 뒤에서 출발
            #   (① 은 2차로 이상 직선 큰길, back ≥ MEET_CLOSE_BACK[0] m 라 관측 50 m·화면 밖). ① 밖으로 넘어가면 다음 스텝에 다시
            if p < 0 or self.edges[i] not in MEET_FIRST_EDGES:
                return False
        else:
            # 2026-11-17: 도로 중간에서 갑자기 생기지 않게, 그 도로의 시작(교차로를 막 지난 곳)에서 출발 — 옆길에서
            #   들어오는 것처럼. AV 시야(50 m)와 화면에서 먼 곳만 (MIN_SPAWN_BACK m 이상 뒤)
            p = 1.0
            if self.cum[ri] + traci.vehicle.getLanePosition(ego) - (self.cum[i] + p) < MIN_SPAWN_BACK:
                return False                            # 뒤 공간이 너무 짧음 (주차장 바로 앞 등) → 다음 스텝에 다시
        j = ri
        while j + 1 < len(self.edges) and not self.edges[j + 1].startswith(LOT_PREFIXES):
            j += 1
        edges = self.edges[i:j + 1]
        if self.meet_until and self.em_route:
            edges += self._join_em_route(edges[-1])
        else:
            try:
                tail = traci.simulation.findRoute(edges[-1], EV_END, vType=EV_TYPES[0]).edges
                if tail and not any(e.startswith(LOT_PREFIXES) for e in tail):
                    edges += list(tail)[1:]
            except traci.TraCIException:
                pass
        self._n_inject += 1
        vid, rid = f'ev_inj_{self._n_inject}', f'ev_inj_route_{self._n_inject}'
        try:
            traci.route.add(rid, edges)
            traci.vehicle.add(vid, rid, typeID=EV_TYPES[0], departPos=f'{p:.1f}',
                              departLane='best', departSpeed='max')
        except traci.TraCIException:
            return False
        self.injected = True
        self.ep['ev_injected'] += 1
        return True

    def _meet_plan(self):
        """경로 기반 투입 계획 (11-24 사용자 결정):
        ① 직선 앞 절반(①-1·①-2)은 정기 출동에게만 기회 → ①-3 시작에서 투입 (정기 출동이 뒤 MEET_119_LOOK m 안에서 오면 안 함)
        → 그래도 못 만나면 예비: 2차로 이상·주차장 아님·유턴 대기 아님 도로에서, 앞 MEET_AHEAD_M m 도 그런 곳일 때만."""
        import math as _m
        n = len(self.edges)
        ok = []
        for i, e in enumerate(self.edges):
            edge = self.net.getEdge(e)
            good = edge.getLaneNumber() >= 2 and not e.startswith(LOT_PREFIXES)
            if good and i + 1 < n:                  # 다음 도로가 거의 반대 방향 = 이 도로 끝에서 유턴 → 유턴 대기 구간
                a, b = edge.getShape(), self.net.getEdge(self.edges[i + 1]).getShape()
                h1 = _m.atan2(a[-1][1] - a[-2][1], a[-1][0] - a[-2][0]); h2 = _m.atan2(b[1][1] - b[0][1], b[1][0] - b[0][0])
                if abs((h2 - h1 + _m.pi) % (2 * _m.pi) - _m.pi) > 2.6:
                    good = False
            ok.append(good)
        self.meet_ok = ok
        idx = [i for i, e in enumerate(self.edges) if e in MEET_FIRST_EDGES]
        if idx:
            a, b = idx[0], idx[-1]
            end = self.cum[b + 1] if b + 1 < len(self.cum) else self.cum[b] + self.net.getEdge(self.edges[b]).getLength()
            self.meet_first = (self.cum[a] + MEET_FIRST_FRAC * (end - self.cum[a]), end, b)
        else:
            self.meet_first = None

    def _meet_ahead_ok(self, ri, rp):
        if ri < 0 or ri >= len(self.edges) or not self.meet_ok[ri]:
            return False
        j = ri
        while j < len(self.edges) and self.cum[j] < rp + MEET_AHEAD_M:
            if not self.meet_ok[j]:
                return False
            j += 1
        return j < len(self.edges)

    def _meet_can_inject(self):
        import traci
        ego = self.env.ego_id
        if ego not in traci.vehicle.getIDList():
            return False
        ri, rp = self._route_pos()
        self._meet_close = False
        if self.meet_first:
            start, end, last_i = self.meet_first
            if rp < start:
                return False                        # ①-1·①-2: 정기 출동에게만 기회 (그 전 구간도 투입 안 함)
            if ri <= last_i:                        # ①-3·①-4: 정기 출동이 뒤에서 오고 있으면 기다림
                ex, ey = traci.vehicle.getPosition(ego)
                for v in self.ev_ids():
                    if v.startswith('ev_inj'):
                        return False                # 이미 투입한 차가 따라오는 중
                    if traci.vehicle.getRoadID(v) in self.em_route:
                        x, y = traci.vehicle.getPosition(v)
                        if math.hypot(x - ex, y - ey) < MEET_119_LOOK and \
                                self.em_route.index(traci.vehicle.getRoadID(v)) <= self.em_route.index(self.edges[ri]):
                            return False
                self._meet_close = True
                return True
        return self._meet_ahead_ok(ri, rp)

    def _join_em_route(self, last):
        """last 에서 119 정기 출동 경로(em_route)의 가장 가까운 도로로 합류한 뒤 그 경로 그대로 끝까지. [last 다음 도로들]"""
        import traci
        em = self.em_route
        if last in em:
            return em[em.index(last) + 1:]
        best = None
        for k, e in enumerate(em):
            try:
                r = traci.simulation.findRoute(last, e, vType=EV_TYPES[0])
            except traci.TraCIException:
                continue
            if not r.edges or any(x.startswith(LOT_PREFIXES) for x in r.edges):
                continue
            if best is None or r.length < best[0]:
                best = (r.length, list(r.edges)[1:] + em[k + 1:])
        return best[1] if best else []

    def _yield_background(self):
        import traci
        ego = self.env.ego_id
        for ev in self.ev_ids():
            cur, look = ev, YIELD_LOOK
            for _ in range(4):
                try:
                    lead = traci.vehicle.getLeader(cur, look)
                except traci.TraCIException:
                    break
                if not lead or not lead[0] or lead[1] > look:
                    break
                vid, gap = lead
                look -= gap + 5.0
                cur = vid
                if vid == ego or vid.startswith('illegal_') or self._ev(vid):
                    continue
                try:
                    road = traci.vehicle.getRoadID(vid)
                    if road.startswith(':'):
                        continue
                    n = traci.edge.getLaneNumber(road)
                    if n < 2:
                        continue
                    k = traci.vehicle.getLaneIndex(vid)
                    traci.vehicle.changeLane(vid, k - 1 if k > 0 else k + 1, 2.0)
                except traci.TraCIException:
                    continue
                if look <= 0:
                    break

    # ------------------------------------------------------------------
    def after_step(self, alive, arrived):
        """시뮬레이션 스텝 뒤: (보상 추가분, 정차 완료로 완주했는지, 정차 놓쳐 실패했는지, 보상 항목)."""
        import traci
        r, terms = 0.0, {}
        done_ok, failed = False, False
        ego = self.env.ego_id
        if alive and self.k < len(self.stops):
            s = self.stops[self.k]
            ri = traci.vehicle.getRouteIndex(ego)
            lane = traci.vehicle.getLaneID(ego)
            pos = traci.vehicle.getLanePosition(ego)
            v = traci.vehicle.getSpeed(ego)
            # 2026-11-17 (사용자 요청 (b)): 정책이 스스로 빈 칸을 보고 그 칸 옆(차 가운데가 칸 ± PARK_TOL m)에
            #   HOLD_S 초 서면 정차 성공 → 환경은 후진으로 칸에 넣었다 빼는 동작만 한다 (sumo_env._park_ego).
            #   구역에 빈 칸이 하나도 없으면 구역 안 어디든 서면 성공 (통로 정차, 칸에 넣지 않음)
            alongside = None
            in_zone = lane == s['lane'] and ri == s['route_index'] and s['start'] <= pos <= s['end'] and v < 0.3
            if in_zone:
                free = self._free_stalls(s)
                mid = pos - traci.vehicle.getLength(ego) / 2
                near = [sp for sp in free if abs(sp[0] - mid) <= PARK_TOL]
                alongside = min(near, key=lambda sp: abs(sp[0] - mid)) if near else (False if free else None)
            if in_zone and alongside is not False:
                self.hold += 1
            else:
                self.hold = 0
            if self.hold >= self.hold_steps:
                self.k += 1
                self.hold = 0
                self.ep['parked'] += 1
                self.park_request = (alongside, self.k >= len(self.stops)) if alongside else None
                self.target = None
                if self.k >= len(self.stops):
                    done_ok = True
                else:
                    r += self.rw.get('park_bonus', 10.0)
                    terms['park'] = self.rw.get('park_bonus', 10.0)
            elif ri > s['route_index']:
                failed = True
        if arrived and not done_ok:
            failed = True                       # 마지막 주차장에 서지 않고 경로 끝까지 가 버림
        if failed:
            self.ep['missed_stop'] = True
        if alive and not failed:
            # 긴급차 길막기
            if self.last.get('ev_block'):
                p = self.rw.get('ev_block_penalty', 0.1)
                r -= p
                terms['ev_block'] = -p
                self.ep['ev_block_steps'] += 1
            # 단속 카메라: 카메라 지점을 이번 스텝에 지났는지
            ri = traci.vehicle.getRouteIndex(ego)
            lane = traci.vehicle.getLaneID(ego)
            pos = traci.vehicle.getLanePosition(ego)
            if lane and not lane.startswith(':'):
                for i, p in self.cam_pts:
                    if i == ri and (i, p) not in self.cam_done and pos >= p:
                        self.cam_done.add((i, p))
                        v = traci.vehicle.getSpeed(ego)
                        if v > traci.lane.getMaxSpeed(lane) + CAM_TOL:
                            pen = self.rw.get('camera_penalty', 5.0)
                            r -= pen
                            terms['camera'] = -pen
                            self.ep['camera_tickets'] += 1
        return r, done_ok, failed, terms

    def _free_stalls(self, s):
        """들를 구역 s 의 빈 칸 [(통로 위 위치, x, y, 칸 각도, 칸 길이)] (구역 안, 위치 순)."""
        import traci
        spaces = [sp for sp in self.spaces.get(s['pa'], []) if s['start'] - 1.5 <= sp[0] <= s['end'] + 1.5]
        if not spaces:
            return []
        occ = []
        try:
            for v in traci.parkingarea.getVehicleIDs(s['pa']):
                occ.append(traci.vehicle.getPosition(v))
        except traci.TraCIException:
            pass
        return [sp for sp in spaces if not any(math.hypot(sp[1] - a, sp[2] - b) < 1.5 for a, b in occ)]

    def _seen_stall_feature(self):
        import traci
        if self.k >= len(self.stops):
            return 1.0
        s = self.stops[self.k]
        ego = self.env.ego_id
        if not (traci.vehicle.getLaneID(ego) == s['lane'] and traci.vehicle.getRouteIndex(ego) == s['route_index']):
            return 1.0
        mid = traci.vehicle.getLanePosition(ego) - traci.vehicle.getLength(ego) / 2
        ahead = [sp[0] - mid for sp in self._free_stalls(s) if -PARK_TOL <= sp[0] - mid <= STALL_SEE]
        return float(np.clip(min(ahead, key=abs) / STALL_SEE, -1, 1)) if ahead else 1.0

    def next_stop_target(self):
        """(강사용, 시뮬레이터 정보) 다음 정차 구역 (멈출 지점까지 거리, 구역 끝까지 거리, 지금 그 통로 차로 위인지).
        멈출 지점 = 앞쪽 가장 가까운 빈 칸 옆 (차 가운데가 칸 위치) 일 때의 앞범퍼 위치. 12 m 안으로 오면 고정.
        빈 칸이 없으면 구역 시작에서 min(8 m, 구역 길이 절반)."""
        import traci
        if self.k >= len(self.stops):
            return None
        s = self.stops[self.k]
        ri, rp = self._route_pos()
        base = self.cum[s['route_index']]
        ego = self.env.ego_id
        on = traci.vehicle.getLaneID(ego) == s['lane'] and ri == s['route_index']
        L = traci.vehicle.getLength(ego)
        pos = traci.vehicle.getLanePosition(ego) if on else -1e9
        free_p = [sp[0] + L / 2 for sp in self._free_stalls(s)]
        if self.target is not None and on and -PARK_TOL <= self.target - pos < 12.0 and \
                any(abs(fp - self.target) < 0.5 for fp in free_p):
            tgt = self.target
        else:
            ahead = [fp for fp in free_p if fp >= max(pos + PARK_TOL, s['start'] + L)]
            tgt = min(ahead) if ahead else (s['start'] + min(8.0, (s['end'] - s['start']) / 2))
            self.target = tgt
        return base + tgt - rp, base + s['end'] - rp, on
