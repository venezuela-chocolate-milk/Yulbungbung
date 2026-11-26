"""구간(진입 도로)별 교통량 최대치 찾기 (2026-11-17, 컨설팅 반영: 트래픽·상호작용 충분히).

배경 교통의 모든 흐름(flow)을 출발 도로(from)별 그룹으로 묶고, 그룹마다 교통량 배율을 따로 올려 가며
배경 교통만 돌린 SUMO 시뮬레이션이 "안정"한 최대 배율을 찾는다. 원형 교차로처럼 용량이 작은 구간은 낮게,
직선 도로·유턴처럼 여유 있는 구간은 높게 정해진다.

안정 조건 (시드마다, 워밍업 WARM 초 이후 MEAS 초 동안):
  - 순간이동(SUMO 정체 판정) 0, 충돌 0
  - 그 그룹 차의 평균 출발 지연(도로에 못 들어가고 기다린 시간) < MAX_DEPART_DELAY 초
  (교착은 SUMO 순간이동 판정 = 300초 연속 정지로 잡는다)
  - 기준 교통량에서 이미 출발 지연이 큰 그룹(진입로 포화)은 배율 1.0 고정, 다른 그룹 시험에서 그 지연은 무시
기준 교통량 = 기존 설정(파일 값 × 0.8). 배율 1.0 = 기존과 같음.

  python traffic_tune.py search     # 그룹별 최대 배율 (다른 그룹은 1.0)
  python traffic_tune.py combine    # 모두 합쳐서 안정해질 때까지 낮추기 → traffic_profile.json
  python traffic_tune.py build      # traffic_profile.json 으로 배율 적용한 경로 파일 생성 (env 가 이것을 씀)
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)
from env import roundabout_env_config as C  # noqa: E402

MAP_DIR = os.path.dirname(C.SUMOCFG)
OUT_DIR = os.path.join(MAP_DIR, "scaled")
PROFILE = os.path.join(MAP_DIR, "traffic_profile.json")
BASE_SCALE = 0.8                       # 기존 기준 교통량 (파일 값 × 0.8)
LEVELS = [1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0]
WARM, MEAS = 300, 1800
MAX_DEPART_DELAY = 5.0
MAX_WAIT = 240.0
SEEDS = (11, 22)


# 2026-11-18: AV 가 들르는 큰손왕만두 앞 블록 S 공영주차장(pk_)은 회전율 2배에서 강사가 통로 출구·입구에서 막힘·충돌
#   (pk_mid_out·pk_drv_out 이탈, pk_nW1 충돌) → 원래 회전율(1배)
TURNOVER_BY_FILE = {"roundabout.parking.rou.xml": 1.0}
TURNOVER = 2.0      # 주차장 회전율 (2026-11-17): 주차장에 들르는 흐름은 대수 ×2, 머무는 시간 ÷2 → 칸 점유는 그대로, 움직이는 차 2배


def gkey(x):
    """흐름의 배율 그룹: 구간 흐름(loc_<구간>_<k>)은 구간 단위, 나머지는 출발 도로."""
    fid = x.get("id") or ""
    if fid.startswith("loc_"):
        return fid.rsplit("_", 1)[0]
    return x.get("from") or x.get("route") or fid


def base_files():
    return list(C.TRAFFIC_BASE_ROUTE_FILES)


def flows():
    """[(파일, flow id, 그룹)]"""
    out = []
    for f in base_files():
        for x in ET.parse(f).getroot().findall("flow"):
            if (x.get("type") or "").startswith("em_"):
                continue                       # 소방차 정기 출동은 배율 조정 대상 아님 (주기 고정)
            out.append((f, x.get("id"), gkey(x)))
    return out


def group_of_flow():
    return {fid: g for _, fid, g in flows()}


def write_scaled(mult, out_dir):
    """mult: 그룹 → 배율. 파일 값 × BASE_SCALE × 배율로 쓴 경로 파일 목록."""
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for f in base_files():
        tree = ET.parse(f)
        drop = []
        for x in tree.getroot().findall("flow"):
            if (x.get("type") or "").startswith("em_"):
                continue                       # 소방차: 파일 값 그대로 (--scale 1)
            g = gkey(x)
            # 주차장에 들르는 흐름(<stop>)은 구간 배율로 올리지 않는다: 주차 칸 수가 정해져 있어 칸이 차면 칸을 기다리는
            #   차가 통로·골목을 막는다 (2026-11-17 강사 시험: 5배로 올렸더니 마전어린이집 골목에서 전부 막힘).
            #   대신 회전율(TURNOVER): 대수 ×TURNOVER, 머무는 시간 ÷TURNOVER → 칸 점유는 그대로, 드나드는 차만 늘어남
            stops = x.findall("stop")
            if stops:
                to = TURNOVER_BY_FILE.get(os.path.basename(f), TURNOVER)
                k = BASE_SCALE * to
                for st in stops:
                    if st.get("duration"):
                        st.set("duration", f"{max(30.0, float(st.get('duration')) / to):.0f}")
            else:
                k = BASE_SCALE * float(mult.get(g, 1.0))
                if k <= 0:
                    drop.append(x)                 # 배율 0 = 이 흐름 없음 (기준 측정용)
                    continue
            if x.get("vehsPerHour"):
                x.set("vehsPerHour", f"{float(x.get('vehsPerHour')) * k:.3f}")
            elif x.get("probability"):
                x.set("probability", f"{min(float(x.get('probability')) * k, 1.0):.6f}")
            elif x.get("period"):
                p = x.get("period")
                if p.startswith("exp("):
                    x.set("period", f"exp({float(p[4:-1]) * k:.6f})")
                else:
                    x.set("period", f"{float(p) / k:.3f}")
            elif x.get("number"):
                x.set("number", str(max(1, round(int(x.get("number")) * k))))
        for x in drop:
            tree.getroot().remove(x)
        p = os.path.join(out_dir, os.path.basename(f))
        tree.write(p, encoding="utf-8", xml_declaration=True)
        paths.append(p)
    return paths


def run_bg(route_files, seed, end=WARM + MEAS):
    """배경 교통만 돌려서 지표를 돌려준다."""
    sumo = os.path.join(os.environ.get("SUMO_HOME", ""), "bin", "sumo")
    if not os.path.exists(sumo):
        sumo = "sumo"
    with tempfile.TemporaryDirectory() as td:
        trip = os.path.join(td, "trip.xml")
        log = os.path.join(td, "log.txt")
        cmd = [sumo, "-c", C.SUMOCFG, "--route-files", ",".join(route_files), "--scale", "1",
               "--end", str(end), "--seed", str(seed), "--no-step-log", "true",
               "--collision.action", "remove", "--tripinfo-output", trip,
               "--tripinfo-output.write-unfinished", "true", "--message-log", log,
               "--step-length", "0.5"] + list(C.SUMO_ARGS)
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3600)
        tele, coll = [], []
        txt = open(log, encoding="utf-8", errors="ignore").read() if os.path.exists(log) else ""
        for m in re.finditer(r"Teleporting vehicle '([^']+)'.*?time=([\d.]+)", txt):
            if float(m.group(2)) >= WARM:
                tele.append(m.group(1))
        for m in re.finditer(r"Vehicle '([^']+)'; collision with vehicle '([^']+)'.*?time=([\d.]+)", txt):
            if float(m.group(3)) >= WARM:
                coll.append(m.group(1))
        dd, wmax = defaultdict(list), 0.0
        if os.path.exists(trip):
            for t in ET.parse(trip).getroot().findall("tripinfo"):
                vid = t.get("id")
                fid = vid.rsplit(".", 1)[0]
                dep = float(t.get("depart", -1))
                if dep >= WARM or dep < 0:
                    dd[fid].append(float(t.get("departDelay", 0)))
                wmax = max(wmax, float(t.get("waitingTime", 0)) if dep >= WARM else 0.0)
    return dict(tele=tele, coll=coll, dd={k: sum(v) / len(v) for k, v in dd.items()}, wmax=wmax)


BASE_DELAY = {}
BD_FILE = os.path.join(MAP_DIR, "traffic_base_delay.json")
if os.path.exists(BD_FILE):
    BASE_DELAY = json.load(open(BD_FILE))


def evaluate(mult, seeds=SEEDS, tag="x"):
    """mult 로 시드마다 돌려 (안정 여부, 문제 그룹들, 상세)."""
    g_of = group_of_flow()
    with tempfile.TemporaryDirectory(dir=os.path.join(BASE, "results")) as td:
        files = write_scaled(mult, td)
        if len(seeds) > 2:
            with ProcessPoolExecutor(len(seeds)) as ex:
                res = list(ex.map(run_bg, [files] * len(seeds), seeds))
        else:
            res = [run_bg(files, s) for s in seeds]
    bad = defaultdict(list)
    for r in res:
        for v in r["tele"]:
            bad[g_of.get(v.rsplit(".", 1)[0], v)].append("teleport")
        for v in r["coll"]:
            bad[g_of.get(v.rsplit(".", 1)[0], v)].append("collision")
        gdd = defaultdict(list)
        for fid, d in r["dd"].items():
            if fid in g_of:
                gdd[g_of[fid]].append(d)
        for g, ds in gdd.items():
            if g in SATURATED:
                continue
            d = sum(ds) / len(ds)
            if g in mult and mult[g] != 1.0:
                lim = MAX_DEPART_DELAY                       # 올린 그룹: 자기 출발 지연 5초
            else:
                lim = max(MAX_DEPART_DELAY, 2 * BASE_DELAY.get(g, 0.0) + 5.0)   # 나머지: 기준보다 확실히 나빠졌을 때만
            if d > lim:
                bad[g].append(f"delay{d:.0f}")
    return (len(bad) == 0), dict(bad), res


SATURATED = set()
SAT_FILE = os.path.join(MAP_DIR, "traffic_saturated.json")
if os.path.exists(SAT_FILE):
    SATURATED = set(json.load(open(SAT_FILE)))


def baseline():
    """기준 교통량에서 출발 지연이 이미 큰 그룹 → traffic_saturated.json"""
    global SATURATED, BASE_DELAY
    SATURATED, BASE_DELAY = set(), {}
    g_of = group_of_flow()
    seeds = (11, 22, 33, 44)
    with tempfile.TemporaryDirectory(dir=os.path.join(BASE, "results")) as td:
        files = write_scaled({}, td)
        res = [run_bg(files, sd) for sd in seeds]
    acc = defaultdict(list)
    for r in res:
        gdd = defaultdict(list)
        for fid, d in r["dd"].items():
            if fid in g_of:
                gdd[g_of[fid]].append(d)
        for g, ds in gdd.items():
            acc[g].append(sum(ds) / len(ds))
    BASE_DELAY = {g: max(v) for g, v in acc.items()}
    json.dump(BASE_DELAY, open(BD_FILE, "w"), indent=1, ensure_ascii=False)
    bad = {"teleports": sum(len(r["tele"]) for r in res), "collisions": sum(len(r["coll"]) for r in res)}
    sat = sorted(g for g, d in BASE_DELAY.items() if d > 3.0)
    json.dump(sat, open(SAT_FILE, "w"), indent=1, ensure_ascii=False)
    SATURATED = set(sat)
    print("기준 교통량 결과:", bad, "\n포화 그룹(1.0 고정, 기준 출발 지연 > 3초):",
          {g: round(BASE_DELAY[g], 1) for g in sat}, flush=True)
    return sat


def _test(args):
    g, lvl = args
    ok, bad, res = evaluate({g: lvl}, tag=f"{g}_{lvl}")
    return g, lvl, ok, bad


def search(workers):
    groups = sorted({g for _, _, g in flows()})
    best = {g: 1.0 for g in groups}
    alive = set(groups) - SATURATED
    for lvl in LEVELS[1:]:
        jobs = [(g, lvl) for g in sorted(alive)]
        if not jobs:
            break
        print(f"== 배율 {lvl}: {len(jobs)} 그룹 시험", flush=True)
        with ProcessPoolExecutor(workers) as ex:
            for g, l, ok, bad in ex.map(_test, jobs):
                if ok:
                    best[g] = l
                else:
                    alive.discard(g)
                    print(f"   {g}: {l} 불안정 {bad} → 최대 {best[g]}", flush=True)
        json.dump(best, open(os.path.join(MAP_DIR, "traffic_search.json"), "w"), indent=1, ensure_ascii=False)
    print("그룹별 최대 배율:", json.dumps(best, ensure_ascii=False), flush=True)
    return best


def group_edges(mult, seed=11):
    """그룹별로 그 그룹 차들이 지나간 도로 집합 + 출발 쪽 처음 몇 도로 (경로 기록에서)."""
    g_of = group_of_flow()
    sumo = os.path.join(os.environ.get("SUMO_HOME", ""), "bin", "sumo")
    used, head = defaultdict(set), defaultdict(set)
    with tempfile.TemporaryDirectory(dir=os.path.join(BASE, "results")) as td:
        files = write_scaled(mult, td)
        vr = os.path.join(td, "vr.xml")
        subprocess.run([sumo if os.path.exists(sumo) else "sumo", "-c", C.SUMOCFG, "--route-files", ",".join(files),
                        "--scale", "1", "--end", str(WARM + MEAS), "--seed", str(seed), "--no-step-log", "true",
                        "--no-warnings", "true", "--vehroute-output", vr, "--step-length", "0.5"] + list(C.SUMO_ARGS),
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3600)
        for v in ET.parse(vr).getroot().iter("vehicle"):
            g = g_of.get(v.get("id").rsplit(".", 1)[0])
            r = v.find("route")
            if g is None or r is None:
                continue
            e = r.get("edges").split()
            used[g].update(e)
            head[g].update(e[:6])
    return used, head


def combine(seeds=(11, 22, 33, 44, 55, 66)):
    best = json.load(open(os.path.join(MAP_DIR, "traffic_search.json")))
    mult = dict(best)
    used, head = group_edges(mult)
    for it in range(30):
        ok, bad, res = evaluate(mult, seeds=seeds)
        print(f"[합치기 {it}] 안정={ok} 문제={bad}", flush=True)
        if ok:
            break
        # 문제 그룹이 올린 그룹이면 그 그룹을, 1.0 그룹이면 그 그룹 출발 쪽 도로(처음 6개)를 지나가는 올린 그룹만 한 단계
        cut = set(g for g in bad if g in mult and mult[g] > 1.0)
        for g in bad:
            if g in mult and mult[g] <= 1.0:
                cut.update(h for h in mult if mult[h] > 1.0 and used.get(h, set()) & head.get(g, set()))
        if not cut:
            cut = set(g for g in mult if mult[g] > 1.0)        # 원인을 못 찾으면 올린 그룹 전부 한 단계
        print(f"   낮춤: {sorted(cut)}", flush=True)
        for g in cut:
            lower = [l for l in LEVELS if l < mult[g]]
            mult[g] = lower[-1] if lower else 1.0
    json.dump(mult, open(PROFILE, "w"), indent=1, ensure_ascii=False)
    print("저장:", PROFILE, flush=True)
    return mult


REFINE_LEVELS = [1.0, 1.1, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]


def _test_up(args):
    mult, g, lvl = args
    m = dict(mult); m[g] = lvl
    ok, bad, _ = evaluate(m, seeds=(11, 22))
    return g, lvl, ok, bad


def refine(workers, fixed=("SW_end_T_left_out",)):
    """지금 배율표에서 출발해, 다른 구간을 지금 값으로 둔 채 구간마다 한 단계씩 올려 보고(합친 상태에서 시험),
    통과한 구간을 모아 6개 시드로 확인 → 막히면 원인 구간만 되돌리고 고정. 더 못 올릴 때까지 반복.
    fixed: 강사 시험으로 정한 구간(유턴 흐름)은 건드리지 않음. 포화 구간(SATURATED)도 고정."""
    mult = json.load(open(PROFILE))
    frozen = set(fixed) | SATURATED
    used, head = group_edges(mult)
    for rnd in range(20):
        cand = []
        for g in sorted(mult):
            if g in frozen:
                continue
            up = [l for l in REFINE_LEVELS if l > mult[g]]
            if up:
                cand.append((mult, g, up[0]))
        if not cand:
            break
        print(f"== 다듬기 {rnd}: {len(cand)} 구간 한 단계 올려 시험", flush=True)
        passed = {}
        with ProcessPoolExecutor(workers) as ex:
            for g, l, ok, bad in ex.map(_test_up, cand):
                if ok:
                    passed[g] = l
                else:
                    frozen.add(g)
        if not passed:
            break
        # 통과한 구간을 합쳐 6개 시드로 확인. 막히면 절반씩 나눠 원인 구간만 찾아 고정 (2026-11-17)
        def try_set(base, items):
            trial = dict(base); trial.update(items)
            ok, bad, _ = evaluate(trial, seeds=(11, 22, 33, 44, 55, 66))
            print(f"   {len(items)}개 합쳐서: 안정={ok} {'' if ok else bad}", flush=True)
            if ok:
                return trial
            if len(items) == 1:
                frozen.update(items)
                return base
            keys = sorted(items)
            half = len(keys) // 2
            base = try_set(base, {k: items[k] for k in keys[:half]})
            return try_set(base, {k: items[k] for k in keys[half:]})
        mult = try_set(mult, passed)
        json.dump(mult, open(PROFILE, "w"), indent=1, ensure_ascii=False)
    print("다듬기 끝:", json.dumps(mult, ensure_ascii=False), flush=True)
    return mult


SEEDS12 = (11, 22, 33, 44, 55, 66, 77, 88, 99, 111, 122, 133)
LOCAL_LEVELS = [1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0]


def densities(mult, seed=11):
    """구간 흐름 그룹별 평균 밀도 (차로 1 km 당, 그 구간 도로들 기준)."""
    segs = json.load(open(os.path.join(MAP_DIR, "local_segments.json")))
    sumo = os.path.join(os.environ.get("SUMO_HOME", ""), "bin", "sumo")
    with tempfile.TemporaryDirectory(dir=os.path.join(BASE, "results")) as td:
        td = os.path.abspath(td)
        files = write_scaled(mult, td)
        add = os.path.join(td, "ed.add.xml")
        open(add, "w").write(f'<additional><edgeData id="ed" file="{td}/ed.xml" begin="{WARM}" end="{WARM + MEAS}"/></additional>')
        deco = C.SUMOCFG.replace("roundabout.sumocfg", "roundabout.deco.xml")
        subprocess.run([sumo if os.path.exists(sumo) else "sumo", "-c", C.SUMOCFG, "--route-files", ",".join(files),
                        "--scale", "1", "--end", str(WARM + MEAS), "--seed", str(seed), "--no-step-log", "true",
                        "--no-warnings", "true", "--step-length", "0.5", "--additional-files", f"{deco},{add}"]
                       + list(C.SUMO_ARGS), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3600)
        ed = {e.get("id"): e for e in ET.parse(os.path.join(td, "ed.xml")).getroot().iter("edge")}
    import sumolib
    net = sumolib.net.readNet(C.NET_FILE)
    out = {}
    for sg in segs:
        L = num = 0.0
        for e in sg["edges"]:
            E = net.getEdge(e)
            x = ed.get(e)
            d = float(x.get("density", 0)) / E.getLaneNumber() if x is not None else 0.0
            L += E.getLength(); num += d * E.getLength()
        out[sg["group"]] = (num / L if L else 0.0, sg["target"])
    return out


def local_search():
    """구간 흐름(loc_*) 배율: 목표 밀도에 닿거나 배경 교통이 불안정해질 때까지 한 단계씩 (seed 12개, 막히면 절반씩 나눠 원인만 고정)."""
    mult = json.load(open(PROFILE))
    groups = sorted({g for _, _, g in flows() if g.startswith("loc_")})
    for g in groups:
        mult.setdefault(g, 1.0)
    frozen = set()
    ok, bad, _ = evaluate(mult, seeds=SEEDS12)
    print(f"구간 흐름 기본(1.0) 넣고: 안정={ok} {'' if ok else bad}", flush=True)
    if not ok:
        culprits = [g for g in bad if g in groups] or groups
        for g in culprits:
            mult[g] = 0.5
        ok, bad, _ = evaluate(mult, seeds=SEEDS12)
        print(f"   문제 구간 0.5 로: 안정={ok} {'' if ok else bad}", flush=True)
        if not ok:
            for g in groups:
                mult[g] = 0.5
    json.dump(mult, open(PROFILE, "w"), indent=1, ensure_ascii=False)

    def try_set(base, items):
        trial = dict(base); trial.update(items)
        ok, bad, _ = evaluate(trial, seeds=SEEDS12)
        print(f"   {len(items)}개 올려서: 안정={ok} {'' if ok else bad}", flush=True)
        if ok:
            return trial
        if len(items) == 1:
            frozen.update(items)
            return base
        keys = sorted(items); half = len(keys) // 2
        base = try_set(base, {k: items[k] for k in keys[:half]})
        return try_set(base, {k: items[k] for k in keys[half:]})

    for rnd in range(12):
        dens = densities(mult)
        reached = {g for g in groups if dens.get(g, (0, 1))[0] >= dens.get(g, (0, 1))[1]}
        cand = {}
        for g in groups:
            if g in frozen or g in reached:
                continue
            up = [l for l in LOCAL_LEVELS if l > mult[g]]
            if up:
                cand[g] = up[0]
        print(f"== 구간 {rnd}: 목표 도달 {len(reached)} / 고정 {len(frozen)} / 올릴 후보 {len(cand)}", flush=True)
        if not cand:
            break
        mult = try_set(mult, cand)
        json.dump(mult, open(PROFILE, "w"), indent=1, ensure_ascii=False)
    dens = densities(mult)
    json.dump({g: [round(d, 1), t, mult[g]] for g, (d, t) in dens.items()},
              open(os.path.join(MAP_DIR, "local_density.json"), "w"), indent=1, ensure_ascii=False)
    print("구간 흐름 끝:", {g: (round(d, 1), t, mult[g]) for g, (d, t) in dens.items()}, flush=True)
    return mult


# ── 2026-11-18: 구간 흐름 다시 찾기 (local2) ──────────────────────────────────────────────
# 노트북의 local_search 는 판정 기준(traffic_base_delay.json, 11/17 17:05)이 신호 변경 전 값이라 원래 교통에서도 생기는
# 지연(검단사거리 쪽 진입로)을 "새 정체"로 보고 멈췄다. local2 는 같은 seed 12개로 "구간 흐름 없이" 돌린 지금 교통을
# 기준으로 새로 생긴 정체만 본다:
#   순간이동 0·충돌 0 / 기존 그룹 평균 출발 지연 ≤ 기준×1.5+5초 / 구간 흐름 자기 지연 ≤ 5초 / 최대 대기 ≤ max(기준×1.5, 600초)
LOCAL2_LEVELS = [0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0]
REF_FILE = os.path.join(MAP_DIR, "traffic_ref_delay.json")


def _run_seeds(mult, seeds):
    with tempfile.TemporaryDirectory(dir=os.path.join(BASE, "results")) as td:
        files = write_scaled(mult, td)
        with ProcessPoolExecutor(len(seeds)) as ex:
            return list(ex.map(run_bg, [files] * len(seeds), seeds))


def _group_delay(res, g_of):
    acc = defaultdict(list)
    for r in res:
        for fid, d in r["dd"].items():
            if fid in g_of:
                acc[g_of[fid]].append(d)
    return {g: sum(v) / len(v) for g, v in acc.items()}


def evaluate_rel(mult, ref, seeds=SEEDS12):
    g_of = group_of_flow()
    res = _run_seeds(mult, seeds)
    bad = defaultdict(list)
    for r in res:
        for v in r["tele"]:
            bad[g_of.get(v.rsplit(".", 1)[0], v)].append("teleport")
        for v in r["coll"]:
            bad[g_of.get(v.rsplit(".", 1)[0], v)].append("collision")
    for g, d in _group_delay(res, g_of).items():
        lim = MAX_DEPART_DELAY if g.startswith("loc_") else max(MAX_DEPART_DELAY, 1.5 * ref["delay"].get(g, 0.0) + 5.0)
        if d > lim:
            bad[g].append(f"delay{d:.0f}>{lim:.0f}")
    wmax = max(r["wmax"] for r in res)
    if wmax > max(600.0, 1.5 * ref["wmax"]):
        bad["_wait"].append(f"wmax{wmax:.0f}")
    return (len(bad) == 0), dict(bad), res


def _temp():
    try:
        out = subprocess.run(["sensors"], capture_output=True, text=True).stdout
        m = re.search(r"Package id 0:\s*\+([\d.]+)", out)
        return m.group(1) if m else "?"
    except Exception:
        return "?"


def local2_search():
    mult = json.load(open(PROFILE))
    groups = sorted({g for _, _, g in flows() if g.startswith("loc_")})
    assert groups, "roundabout.local.rou.xml 이 TRAFFIC_BASE_ROUTE_FILES 에 없음"
    for g in groups:
        mult[g] = 0.0
    g_of = group_of_flow()
    res = _run_seeds(mult, SEEDS12)
    ref = dict(delay=_group_delay(res, g_of), wmax=max(r["wmax"] for r in res),
               tele=sum(len(r["tele"]) for r in res), coll=sum(len(r["coll"]) for r in res))
    json.dump(ref, open(REF_FILE, "w"), indent=1, ensure_ascii=False)
    print(f"기준(구간 흐름 없음): 순간이동 {ref['tele']} 충돌 {ref['coll']} 최대대기 {ref['wmax']:.0f}s  CPU {_temp()}C", flush=True)
    frozen = set()

    def try_set(base, items):
        trial = dict(base); trial.update(items)
        ok, bad, _ = evaluate_rel(trial, ref)
        print(f"   {len(items)}개 올려서: 안정={ok} {'' if ok else bad}  CPU {_temp()}C", flush=True)
        if ok:
            return trial
        if len(items) == 1:
            frozen.update(items)
            return base
        keys = sorted(items); half = len(keys) // 2
        base = try_set(base, {k: items[k] for k in keys[:half]})
        return try_set(base, {k: items[k] for k in keys[half:]})

    for rnd in range(len(LOCAL2_LEVELS) + 2):
        dens = densities(mult) if rnd else {}
        reached = {g for g in groups if g in dens and dens[g][0] >= dens[g][1]}
        cand = {}
        for g in groups:
            if g in frozen or g in reached:
                continue
            up = [l for l in LOCAL2_LEVELS if l > mult[g]]
            if up:
                cand[g] = up[0]
        print(f"== 단계 {rnd}: 목표 도달 {len(reached)} / 고정 {len(frozen)} / 올릴 후보 {len(cand)}", flush=True)
        if dens:
            print("   밀도:", {g: (round(d, 1), t, mult[g]) for g, (d, t) in dens.items()}, flush=True)
        if not cand:
            break
        mult = try_set(mult, cand)
        json.dump(mult, open(PROFILE, "w"), indent=1, ensure_ascii=False)
    dens = densities(mult)
    json.dump({g: [round(d, 1), t, mult[g]] for g, (d, t) in dens.items()},
              open(os.path.join(MAP_DIR, "local_density.json"), "w"), indent=1, ensure_ascii=False)
    print("구간 흐름 끝:", {g: (round(d, 1), t, mult[g]) for g, (d, t) in dens.items()}, flush=True)
    return mult


# ── 2026-11-18: local3 — 밀도 측정으로 배율을 바로 정하고 불안정하면 원인만 줄이기 (local2 가 너무 느리고 예민해서) ──
# 판정: 순간이동 0·충돌 0 / 최대 대기 ≤ max(600초, 기준×1.5) / 구간 흐름 자기 출발 지연 ≤ 5초 /
#   기존 그룹 출발 지연은 기준에서 이미 5초 넘던 맵 경계 진입로(줄 서서 들어오는 곳, 화면에 안 보임)는 빼고 ≤ max(5, 기준×1.5+5)
LOCAL3_MAX = 16.0
PAR = 10                                    # 동시에 도는 SUMO 수 (데스크톱 발열: 12개에서 100°C 순간치)


def _run_seeds_par(mult, seeds):
    with tempfile.TemporaryDirectory(dir=os.path.join(BASE, "results")) as td:
        files = write_scaled(mult, td)
        with ProcessPoolExecutor(min(PAR, len(seeds))) as ex:
            return list(ex.map(run_bg, [files] * len(seeds), seeds))


def evaluate3(mult, ref, seeds=SEEDS12):
    g_of = group_of_flow()
    res = _run_seeds_par(mult, seeds)
    bad = defaultdict(list)
    for r in res:
        for v in r["tele"]:
            bad[g_of.get(v.rsplit(".", 1)[0], v)].append("teleport")
        for v in r["coll"]:
            bad[g_of.get(v.rsplit(".", 1)[0], v)].append("collision")
    for g, d in _group_delay(res, g_of).items():
        r0 = ref["delay"].get(g, 0.0)
        if g.startswith("loc_"):
            lim = MAX_DEPART_DELAY
        elif r0 > MAX_DEPART_DELAY:
            continue                         # 이미 포화된 맵 경계 진입로
        else:
            lim = max(MAX_DEPART_DELAY, 1.5 * r0 + 5.0)
        if d > lim:
            bad[g].append(f"delay{d:.0f}>{lim:.0f}")
    wmax = max(r["wmax"] for r in res)
    if wmax > max(600.0, 1.5 * ref["wmax"]):
        bad["_wait"].append(f"wmax{wmax:.0f}")
    return (len(bad) == 0), dict(bad), res


def _dens_avg(mult, seeds=(11, 22, 33)):
    with ProcessPoolExecutor(len(seeds)) as ex:
        outs = list(ex.map(densities, [mult] * len(seeds), seeds))
    return {g: (sum(o[g][0] for o in outs) / len(outs), outs[0][g][1]) for g in outs[0]}


def local3_search():
    mult = json.load(open(PROFILE))
    groups = sorted({g for _, _, g in flows() if g.startswith("loc_")})
    ref = json.load(open(REF_FILE))         # local2 가 같은 기존 교통으로 잰 기준 (구간 흐름 없음, seed 12개)
    m0 = dict(mult); m0.update({g: 0.0 for g in groups})
    m1 = dict(mult); m1.update({g: 0.25 for g in groups})
    d0, d1 = _dens_avg(m0), _dens_avg(m1)
    for g in groups:
        slope = max((d1[g][0] - d0[g][0]) / 0.25, 0.5)
        need = (d1[g][1] - d0[g][0]) / slope
        mult[g] = round(min(LOCAL3_MAX, max(0.25, need)), 2)
    print("밀도 0 / 0.25배:", {g: (round(d0[g][0], 1), round(d1[g][0], 1), d1[g][1]) for g in groups}, flush=True)
    print("첫 배율:", {g: mult[g] for g in groups}, f"CPU {_temp()}C", flush=True)
    for it in range(12):
        ok, bad, _ = evaluate3(mult, ref)
        print(f"== 시험 {it}: 안정={ok} {'' if ok else bad}  CPU {_temp()}C", flush=True)
        if ok:
            break
        loc_bad = [g for g in bad if g in groups]
        if loc_bad:
            for g in loc_bad:
                mult[g] = 0.0 if mult[g] <= 0.25 else round(max(0.25, mult[g] * 0.5), 2)
        else:
            for g in groups:                  # 원인이 기존 흐름 쪽이면 많이 올린 구간부터 조금씩
                if mult[g] > 1.0:
                    mult[g] = round(max(1.0, mult[g] * 0.7), 2)
            if all(mult[g] <= 1.0 for g in groups):
                for g in groups:
                    mult[g] = round(mult[g] * 0.7, 2)
        print("   다음 배율:", {g: mult[g] for g in groups}, flush=True)
    json.dump(mult, open(PROFILE, "w"), indent=1, ensure_ascii=False)
    dens = _dens_avg(mult)
    json.dump({g: [round(d, 1), t, mult[g]] for g, (d, t) in dens.items()},
              open(os.path.join(MAP_DIR, "local_density.json"), "w"), indent=1, ensure_ascii=False)
    print("구간 흐름 끝:", {g: (round(d, 1), t, mult[g]) for g, (d, t) in dens.items()}, flush=True)
    return mult


def build():
    mult = json.load(open(PROFILE))
    paths = write_scaled(mult, OUT_DIR)
    print("\n".join(paths))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["baseline", "search", "combine", "refine", "local", "local2", "local3", "build", "eval"])
    ap.add_argument("--workers", type=int, default=28)
    a = ap.parse_args()
    if a.cmd == "baseline":
        baseline()
    elif a.cmd == "search":
        search(a.workers)
    elif a.cmd == "combine":
        combine()
    elif a.cmd == "refine":
        refine(a.workers)
    elif a.cmd == "local":
        local_search()
    elif a.cmd == "local2":
        local2_search()
    elif a.cmd == "local3":
        local3_search()
    elif a.cmd == "build":
        build()
    else:
        ok, bad, res = evaluate({})
        print("기준(배율 1.0) 안정:", ok, bad, [dict(tele=len(r["tele"]), coll=len(r["coll"]), wmax=r["wmax"]) for r in res])
