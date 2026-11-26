"""오른쪽 아래 끝 십자 교차로(SW_new_cross)에서 안전지대를 가로지르는 우회전 연결을 없앤다.

이 교차로의 우회전은 약 200 m 앞에서 갈라지는 우회전 전용 도로(SW_long_right)로 하도록
만들었는데, 교차로로 직진해 들어가는 SW_slip_main 의 맨 오른쪽 차로(0번)에서도 교차로 안으로
바로 우회전하는 연결(:SW_new_cross_20_0 → SW_new_lower_near)이 남아 있었다.
그 경로는 28 m 중 13 m 가 안전지대(SW_drawn_island_fill_3) 위를 지나간다.

교차로를 netconvert 로 다시 만들면 손질한 모양이 바뀌므로, 이 연결 하나만 뺀다:
  - 연결 2개 (SW_slip_main_0 → 내부 경로, 내부 경로 → SW_new_lower_near) 와 내부 경로 엣지 삭제
  - 교차로 intLanes / 규칙표에서 그 링크를 빼고 뒤 번호를 당김 (비트 문자열 오른쪽 끝 = 링크 0)
  - 교차로 중간 대기 지점(internal junction)의 상충 목록에서도 삭제
노면 표시는 그 진입로에 우회전 화살표·유도선이 없어 바꿀 것이 없다.
"""
import xml.etree.ElementTree as ET

J = "SW_new_cross"
LANE = ":SW_new_cross_20_0"
EDGE = ":SW_new_cross_20"


def _drop(bits, index):
    """비트 문자열(오른쪽 끝 = 링크 0)에서 index 자리를 뺀다."""
    k = len(bits) - 1 - index
    return bits[:k] + bits[k + 1:]


def apply_net(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    edge = next((e for e in root.findall("edge") if e.get("id") == EDGE), None)
    if edge is None:
        return {"cross_right_removed": "already"}
    j = next(x for x in root.findall("junction") if x.get("id") == J)
    ints = j.get("intLanes").split()
    k = ints.index(LANE)
    ints.pop(k)
    j.set("intLanes", " ".join(ints))
    for r in list(j.findall("request")):
        i = int(r.get("index"))
        if i == k:
            j.remove(r)
            continue
        r.set("response", _drop(r.get("response"), k))
        r.set("foes", _drop(r.get("foes"), k))
        if i > k:
            r.set("index", str(i - 1))
    for x in root.findall("junction"):
        if x.get("type") == "internal" and LANE in x.get("intLanes", "").split():
            x.set("intLanes", " ".join(t for t in x.get("intLanes").split() if t != LANE))
    root.remove(edge)
    removed = 0
    for c in list(root.findall("connection")):
        if c.get("via") == LANE or c.get("from") == EDGE:
            root.remove(c)
            removed += 1
    ET.indent(tree, space="    ")
    tree.write(net_path, encoding="utf-8", xml_declaration=True)
    return {"cross_right_removed": removed}
