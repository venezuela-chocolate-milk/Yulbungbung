"""신호 교차로 진입로마다 한국식 4색 신호등(적·황·좌회전 화살표·녹)을 그리고, 실제 신호에 맞춰 켠다.

SUMO 의 기본 신호 표시(showLinkRules)는 정지선 위 색 막대인데, 신호 없는 교차로에도
양보/우선 막대가 함께 그려져 화면이 복잡하다. 그래서 그 표시는 끄고, 신호 교차로에만
길가(정지선 오른쪽)에 신호등 그림을 놓는다.

  생성:  python -m env.roundabout_signal_heads   → env/sumo_roundabout/roundabout.signals.add.xml
  실행 중 색 바꾸기:  heads = SignalHeads(path); heads.update()   (매 스텝, TraCI 연결 상태에서)

신호등 그림은 polygon 이라 SUMO 화면에만 보이고 주행/학습에는 영향이 없다.
등 켜는 규칙 (진입로 하나 기준):
  적  : 직진이 정지 (직진이 없는 진입로는 좌회전이 정지)
  황  : 직진/좌회전 중 하나라도 황색
  ←   : 좌회전 보호 신호 (G)
  녹  : 직진 진행 (G/g)
  우회전은 양보하며 진행하는 경우가 많아 신호등에 따로 표시하지 않는다.
"""
import math
import os
import xml.etree.ElementTree as ET

MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sumo_roundabout")
OUT = os.path.join(MAP_DIR, "roundabout.signals.add.xml")

LAMP = 1.1            # 등 하나 칸 크기 (m). 실제보다 크게 그려 ego 추적 화면에서도 보이게
BOX_W = 1.2           # 신호등 몸체 두께 (m, 진행 방향)
GAP = 0.8             # 차로 바깥 경계에서 신호등까지 거리 (m)
BACK = 1.0            # 차로 끝(정지 위치)에서 뒤로 (정지선 위치)
OFF = (45, 45, 45, 255)
ON = {"R": (255, 30, 30, 255), "Y": (255, 200, 0, 255),
      "L": (40, 230, 70, 255), "G": (40, 230, 70, 255)}
LAYER = "60"


def _read(v):
    return [tuple(map(float, p.split(","))) for p in v.split()]


def _write(pts):
    return " ".join(f"{x:.3f},{y:.3f}" for x, y in pts)


def _approaches(root):
    """{(tl, 진입 edge): {"s": [링크], "l": [...], "r": [...]}}"""
    out = {}
    for c in root.findall("connection"):
        if not c.get("tl"):
            continue
        d = c.get("dir").lower()
        d = {"t": "l"}.get(d, d)
        g = out.setdefault((c.get("tl"), c.get("from")), {"s": [], "l": [], "r": []})
        g.setdefault(d, []).append(int(c.get("linkIndex")))
    return out


def build(net_path=None, out=OUT):
    net_path = net_path or os.path.join(MAP_DIR, "roundabout.net.xml")
    root = ET.parse(net_path).getroot()
    lanes = {}
    for e in root.findall("edge"):
        for l in e.findall("lane"):
            lanes[(e.get("id"), int(l.get("index")))] = (_read(l.get("shape")), float(l.get("width", 3.2)))
    lines = ["<?xml version='1.0' encoding='utf-8'?>",
             "<!-- env/roundabout_signal_heads.py 가 생성. 신호등 그림 (주행에는 영향 없음) -->",
             "<additional>"]
    off = ",".join(map(str, OFF))
    for (tl, edge), g in sorted(_approaches(root).items()):
        pts, w = lanes[(edge, 0)]                     # 맨 오른쪽 차로
        (x0, y0), (x1, y1) = pts[-2], pts[-1]
        L = math.hypot(x1 - x0, y1 - y0)
        hx, hy = (x1 - x0) / L, (y1 - y0) / L          # 진행 방향
        rx, ry = hy, -hx                               # 오른쪽
        bx = x1 - hx * BACK + rx * (w / 2 + GAP)       # 신호등 안쪽 끝 (도로 쪽)
        by = y1 - hy * BACK + ry * (w / 2 + GAP)

        def P(a, b):                                   # a: 바깥쪽(오른쪽) 거리, b: 진행 방향 거리
            return (bx + rx * a + hx * b, by + ry * a + hy * b)

        n = 4
        pid = f"sig_{tl}__{edge}"
        box = [P(-0.1, -BOX_W / 2), P(n * LAMP + 0.1, -BOX_W / 2),
               P(n * LAMP + 0.1, BOX_W / 2), P(-0.1, BOX_W / 2)]
        lines.append(f'    <poly id="{pid}__box" type="signal" color="15,15,15,255" fill="1" '
                     f'layer="{float(LAYER) - 1}" shape="{_write(box)}"/>')
        # 운전자가 보는 왼쪽(도로 쪽)부터 적·황·←·녹
        for k, lamp in enumerate("RYLG"):
            cx = (k + 0.5) * LAMP
            if lamp == "L":                            # 왼쪽 화살표 (진행 방향 기준 왼쪽 = 도로 쪽)
                s = LAMP * 0.38
                shape = [P(cx - s, 0), P(cx - s * 0.1, -s * 0.75), P(cx - s * 0.1, -s * 0.3),
                         P(cx + s, -s * 0.3), P(cx + s, s * 0.3), P(cx - s * 0.1, s * 0.3),
                         P(cx - s * 0.1, s * 0.75)]
            else:
                rad = LAMP * 0.4
                shape = [P(cx + rad * math.cos(t), rad * math.sin(t))
                         for t in (i / 16 * 2 * math.pi for i in range(17))]
            lines.append(
                f'    <poly id="{pid}__{lamp}" type="signal" color="{off}" fill="1" layer="{LAYER}" '
                f'shape="{_write(shape)}">\n'
                f'        <param key="tl" value="{tl}"/>\n'
                f'        <param key="s" value="{" ".join(map(str, g["s"]))}"/>\n'
                f'        <param key="l" value="{" ".join(map(str, g["l"]))}"/>\n'
                f'    </poly>')
    lines.append("</additional>")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return out


def _lamp_on(lamp, state, s, l):
    st = [state[k] for k in s]
    lt = [state[k] for k in l]
    if lamp == "R":
        main = st or lt
        return bool(main) and all(c in "rs" for c in main)
    if lamp == "Y":
        return any(c in "yY" for c in st + lt)
    if lamp == "L":
        return any(c == "G" for c in lt)
    if lamp == "G":
        return any(c in "Gg" for c in st)
    return False


class SignalHeads:
    """TraCI 로 신호 상태를 읽어 신호등 polygon 색을 바꾼다 (바뀐 등만)."""

    def __init__(self, path=OUT):
        self.lamps = []                                # (poly id, tl, 등 종류, 직진 링크, 좌회전 링크)
        for p in ET.parse(path).getroot().iter("poly"):
            prm = {x.get("key"): x.get("value") for x in p.findall("param")}
            if "tl" in prm:
                ints = lambda v: [int(t) for t in v.split()]   # noqa: E731
                self.lamps.append((p.get("id"), prm["tl"], p.get("id").rsplit("__", 1)[1],
                                   ints(prm.get("s", "")), ints(prm.get("l", ""))))
        self.reset()

    def reset(self):
        self._shown = {}

    def update(self):
        import traci
        states = {}
        for pid, tl, lamp, s, l in self.lamps:
            if tl not in states:
                states[tl] = traci.trafficlight.getRedYellowGreenState(tl)
            on = _lamp_on(lamp, states[tl], s, l)
            if self._shown.get(pid) != on:
                try:
                    traci.polygon.setColor(pid, ON[lamp] if on else OFF)
                except traci.TraCIException:
                    continue
                self._shown[pid] = on


if __name__ == "__main__":
    print(build())
