"""collect_pomdp_data.py 가 만든 JSONL(.jsonl.gz)을 수업 예시(sample_15.csv)와 같은 칼럼의 CSV 로 바꾼다.

    python export_csv.py data/xxx.jsonl.gz                  → data/xxx.csv (전체)
    python export_csv.py data/xxx.jsonl.gz --rows 15        → 앞 15 줄만 (예시 제출용)

칼럼: 수업 필수 항목 19개 (같은 순서, 중첩 항목은 action.xxx / event.xxx 처럼 점으로 펼침,
벡터·사전은 JSON 문자열) + 팀별 추가 항목 (event.red_violation ... reward_terms, route).
next_observation / next_state 는 다음 줄의 observation / state 로 채운다 (iter_transitions).
"""
import argparse
import csv
import json
import os

from collect_pomdp_data import iter_transitions

REQUIRED = ["episode", "t", "observation", "action_raw", "action.accel_raw", "action.lane_change_raw",
            "action.lane_change", "action.lane_change_applied", "reward", "terminated", "truncated", "state",
            "detected_vehicles", "event.collided", "event.arrived", "event.lane_before", "event.lane_after",
            "next_observation", "next_state"]
EXTRA = ["event.red_violation", "event.lost", "event.parked", "event.park_now", "event.missed_stop",
         "event.ev_near", "event.ev_block", "event.camera_ticket", "reward_terms", "route"]


def _get(rec, key):
    v = rec
    for k in key.split("."):
        v = v.get(k) if isinstance(v, dict) else None
    if isinstance(v, (list, dict)):
        return json.dumps(v, ensure_ascii=False, separators=(",", ":"))
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("--out", default=None)
    ap.add_argument("--rows", type=int, default=0, help="이만큼만 (0 = 전체)")
    ap.add_argument("--start", type=int, default=0, help="이 transition 번호부터")
    ap.add_argument("--state-radius", type=float, default=None,
                    help="state/next_state 의 주변 차량을 ego 반경 이 m 안만 남김 (CSV 용량 줄이기)")
    a = ap.parse_args()
    out = a.out or a.jsonl.replace(".jsonl.gz", ".csv").replace(".jsonl", ".csv")
    n = 0
    with open(out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(REQUIRED + EXTRA)
        for i, rec in enumerate(iter_transitions(a.jsonl)):
            if i < a.start:
                continue
            if a.state_radius:
                for k in ("state", "next_state"):
                    st = rec.get(k) or {}
                    if st.get("vehicles") is not None:
                        ex, ey = st["ego_x"], st["ego_y"]
                        st["vehicles"] = [v for v in st["vehicles"]
                                          if (v["x"] - ex) ** 2 + (v["y"] - ey) ** 2 <= a.state_radius ** 2]
            w.writerow([_get(rec, k) for k in REQUIRED + EXTRA])
            n += 1
            if a.rows and n >= a.rows:
                break
    print(f"{n} 줄 → {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
