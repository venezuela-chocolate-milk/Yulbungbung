"""2026-11-21 공통 수정 1 (교통): 교차로 바로 앞 짧은 토막에서 출발하던 배경차 흐름의 출발 도로를 뒤쪽 윗길로 옮긴다.

문제: 출발 도로가 12~35 m 짜리 토막이고 그 끝이 교차로인 흐름은, SUMO 가 토막 시작점에 차를 최고 속도로
  만들어 넣어 차가 허공에서 나타나 1~2초 만에 교차로에 들어왔다 (실제라면 뒤에서 달려오는 게 보였을 차).
수정: 출발 도로 → 이어지는 윗길을 거슬러 올라가 교차로까지 최소 MIN_RUN m 를 달려오게 출발 도로만 바꾼다.
  교통량·도착지·차종은 그대로. 지도 가장자리(막다른 끝)에서 출발하는 흐름은 원래대로 둔다.
  기본 경로 파일과 배율 사본(scaled/) 모두에 같은 변경을 쓴다. 다시 실행해도 결과가 같다 (이미 옮긴 흐름은 그대로).
사용: python env/fix_spawn_upstream.py [--dry]
"""
import os, sys, re, argparse
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MIN_RUN = 50.0
HERE = os.path.dirname(os.path.abspath(__file__))
MAP = os.path.join(HERE, "sumo_roundabout")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dry", action="store_true"); a = ap.parse_args()
    if "SUMO_HOME" not in os.environ:
        import sumo; os.environ["SUMO_HOME"] = os.path.dirname(sumo.__file__)
    import sumolib
    from env.roundabout_env_config import TRAFFIC_BASE_ROUTE_FILES
    net = sumolib.net.readNet(os.path.join(MAP, "roundabout.net.xml"))
    import glob
    pa_edge = {}                                   # 주차 구역 → 도로 (via 를 주면 SUMO 가 정차 지점을 경유지로 안 쓰므로 직접 넣음)
    for af in glob.glob(os.path.join(MAP, "*.xml")):
        try:
            for pa in ET.parse(af).getroot().iter("parkingArea"):
                pa_edge[pa.get("id")] = pa.get("lane").rsplit("_", 1)[0]
        except ET.ParseError:
            pass

    def upstream_chain(eid):
        """eid 앞에 붙일 윗길 목록 (먼 쪽부터). 교차로까지 달리는 거리 >= MIN_RUN 이 될 때까지."""
        chain, run, cur = [], net.getEdge(eid).getLength(), net.getEdge(eid)
        while run < MIN_RUN:
            node = cur.getFromNode()
            if node.getType() == "dead_end":
                break
            cands = []
            # 윗길이 하나뿐인 경우만 (그 길로만 올 수 있는 차 → 옮겨도 다른 교차로의 교통 흐름이 바뀌지 않음).
            #   윗길이 여럿인 교차로(합류·교차)를 지나게 옮기면 그 교차로에 없던 교통이 생긴다 (loc_22: 주차장 입구 막힘)
            ups = [u for u in cur.getIncoming() if u.getFunction() != "internal" and u.allows("passenger") and u.getConnections(cur)]
            if len(ups) != 1:
                break
            for up in cur.getIncoming():
                if up.getFunction() == "internal" or not up.allows("passenger"):
                    continue
                conns = up.getConnections(cur)
                if not conns:
                    continue
                straight = any(c.getDirection() == "s" for c in conns)
                done = run + up.getLength() >= MIN_RUN            # 이 윗길 하나로 50 m 를 채우면 우선 (주차장 통로 등으로 깊이 들어가지 않게)
                cands.append((done, straight, up.getLength(), up))
            if not cands:
                break
            cands.sort(key=lambda x: (x[0], x[1], x[2]), reverse=True)   # 한 번에 채우는 길 > 직진 > 긴 길
            up = cands[0][3]
            if up.getID() in chain or up.getID() == eid:
                break
            chain.insert(0, up.getID()); run += up.getLength(); cur = up
        return chain, run

    files = []
    for f in TRAFFIC_BASE_ROUTE_FILES:
        files.append(f)
        s = os.path.join(MAP, "scaled", os.path.basename(f))
        if os.path.exists(s):
            files.append(s)
    total = 0
    for f in files:
        txt = open(f, encoding="utf-8").read()
        root = ET.fromstring(txt)
        changed = []
        for el in list(root.iter("flow")) + list(root.iter("trip")):
            frm = el.get("from")
            if not frm or not net.hasEdge(frm):
                continue
            try:                                   # 이미 옮긴 흐름 (숫자 출발 위치 + via) → 그대로 (다시 실행해도 같은 결과)
                float(el.get("departPos", "x")); 
                if el.get("via"):
                    continue
            except ValueError:
                pass
            if (el.get("id") or "").startswith("loc_"):   # 구간 안 골목·출입구에서 들어오는 교통 (env/roundabout_local_traffic.py) → 그대로
                continue                                  #   (윗길로 옮기면 그 차들이 앞 교차로를 지나가게 되어 교통 흐름 자체가 바뀜)
            if el.get("departPos") == "stop":      # 주차 칸에 서 있다가 출발하는 차 (실제로도 거기 있던 차) → 그대로
                continue
            E = net.getEdge(frm)
            if E.getFromNode().getType() == "dead_end" or E.getLength() >= MIN_RUN:
                continue
            chain, run = upstream_chain(frm)
            if not chain:
                continue
            new_from = chain[0]
            stops = [pa_edge[st.get("parkingArea")] for st in el.findall("stop") if st.get("parkingArea") in pa_edge]
            stops += [st.get("lane").rsplit("_", 1)[0] for st in el.findall("stop") if st.get("lane")]
            via_l = chain[1:] + [frm] + ((el.get("via") or "").split())
            for se in stops:                          # 정차 도로를 경유지로 (이미 있으면 생략)
                if se not in via_l:
                    via_l.append(se)
            via = " ".join(via_l)
            # 교차로까지 정확히 MIN_RUN m 가 되도록 첫 윗길 안에서의 출발 위치 (긴 윗길을 통째로 달리게 하면 그 길 교통이 늘어남)
            L0 = net.getEdge(new_from).getLength()
            dpos = min(max(run - MIN_RUN, 0.0), max(L0 - 1.0, 0.0)) if run >= MIN_RUN else 0.0
            changed.append((el.get("id"), frm, new_from, via, round(min(run, MIN_RUN) if run >= MIN_RUN else run, 1), round(dpos, 1)))
        if not changed:
            continue
        for fid, frm, new_from, via, run, dpos in changed:
            m = re.search(r'<(?:flow|trip)\b[^>]*\bid="%s"[^>]*>' % re.escape(fid), txt)
            if not m:
                print("  !! 못 찾음", f, fid); continue
            tag = m.group(0)
            t2 = re.sub(r'\svia="[^"]*"', '', tag)          # 기존 via 는 새 via 뒤에 이어 붙였음
            t2 = re.sub(r'\bfrom="%s"' % re.escape(frm), 'from="%s" via="%s"' % (new_from, via), t2, count=1)
            if 'departPos="' in t2:
                t2 = re.sub(r'departPos="[^"]*"', 'departPos="%s"' % dpos, t2, count=1)
            else:
                t2 = t2.replace('from="%s"' % new_from, 'from="%s" departPos="%s"' % (new_from, dpos), 1)
            txt = txt[:m.start()] + t2 + txt[m.end():]
            total += 1
            if "scaled" not in f:
                print(f"  {os.path.basename(f):38s} {fid:28s} {frm} -> {new_from} @ {dpos} m (교차로까지 {run} m)")
        if not a.dry:
            open(f, "w", encoding="utf-8").write(txt)
    print("바꾼 흐름", total, "(dry)" if a.dry else "")


if __name__ == "__main__":
    main()
