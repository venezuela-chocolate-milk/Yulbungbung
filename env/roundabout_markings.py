"""One physical dash/gap pattern for complete roads, junctions and the ring."""
import math
import xml.etree.ElementTree as ET

from env.roundabout_geometry import _read, _write, _offset, _length, _slice


def crossing_clearance(net_path, source, target, margin=0.5):
    """Swept pavement of the two left-turn lanes crossing the main road."""
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    root = ET.parse(net_path).getroot()
    lanes = {l.get('id'): l for e in root.findall('edge') for l in e.findall('lane')}
    follow = {(c.get('from'), c.get('fromLane')): c for c in root.findall('connection')
              if c.get('from').startswith(':')}
    areas = []
    for c in root.findall('connection'):
        if c.get('from') != source or c.get('to') != target:
            continue
        current = c.get('via')
        while current:
            lane = lanes[current]
            areas.append(LineString(_read(lane.get('shape'))).buffer(
                float(lane.get('width', '3.2'))/2+margin, cap_style=2))
            edge, index = current.rsplit('_', 1)
            current = follow[(edge, index)].get('via')
    return unary_union(areas)


def dash_style(radius, desired_dash=3.0, desired_gap=3.0):
    # A closed circle cannot fit arbitrary full periods. Scale the shared style
    # slightly (about 8mm for a 20m ring) and use it on every straight/curve too.
    circumference = 2*math.pi*radius
    count = max(1, round(circumference/(desired_dash+desired_gap)))
    scale = circumference/(count*(desired_dash+desired_gap))
    return desired_dash*scale, desired_gap*scale, count


def divider_routes(net_path, separated_edge=None, special_divider=None, width=3.2,
                   junction_guides=()):
    """Join separators across straight-through internal junction connections."""
    root = ET.parse(net_path).getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    lane_shapes = {l.get('id'): _read(l.get('shape')) for e in edges.values() for l in e.findall('lane')}
    paths = {}
    for eid, edge in edges.items():
        if edge.get('function') == 'internal' or eid.startswith('ring'):
            continue
        count = len(edge.findall('lane'))
        for i in range(count-1):
            if eid == separated_edge:
                if i == 1 and special_divider is not None:
                    paths[(eid, i)] = special_divider
                continue
            paths[(eid, i)] = _offset(lane_shapes[f'{eid}_{i}'], width/2)
    connections = root.findall('connection')
    external = {(c.get('from'), c.get('to'), int(c.get('fromLane'))): c
                for c in connections if not c.get('from').startswith(':')}
    follow = {(c.get('from'), c.get('fromLane')): c for c in connections
              if c.get('from').startswith(':')}
    next_route, previous = {}, {}
    for (source, index) in paths:
        if source == separated_edge:
            continue
        for (fr, to, lane), lower in external.items():
            if fr != source or lane != index or lower.get('dir') != 's':
                continue
            upper = external.get((fr, to, index+1))
            target_index = int(lower.get('toLane'))
            target = (to, target_index)
            if upper is None or int(upper.get('toLane')) != target_index+1 or target not in paths:
                continue
            bridge = []
            current = lower.get('via')
            while current:
                bridge.extend(lane_shapes[current][1 if bridge else 0:])
                edge, lane_index = current.rsplit('_', 1)
                current = follow[(edge, lane_index)].get('via')
            if _length(bridge) > 1e-6:
                bridge = _offset(bridge, width/2)
            else:
                bridge = []
            next_route[(source, index)] = target, bridge
            previous[target] = (source, index)
            break
    visited, routes = set(), {}
    roots = [key for key in paths if key not in previous]
    for first in roots+list(paths):
        if first in visited:
            continue
        points, current = [], first
        if first[0] in junction_guides:
            entering = next((c for c in connections if c.get('from', '').startswith('ring')
                             and c.get('to') == first[0]
                             and int(c.get('toLane')) == first[1]), None)
            if entering is not None:
                bridge, lane_id = [], entering.get('via')
                while lane_id:
                    bridge.extend(lane_shapes[lane_id][1 if bridge else 0:])
                    edge, index = lane_id.rsplit('_', 1)
                    lane_id = follow[(edge, index)].get('via')
                points = _offset(bridge, width/2)
                points[-1] = paths[first][0]
        while current not in visited:
            visited.add(current)
            for p in paths[current]:
                if not points or math.dist(points[-1], p) > 1e-8:
                    points.append(p)
            if current not in next_route:
                break
            target, bridge = next_route[current]
            for p in bridge:
                if not points or math.dist(points[-1], p) > 1e-8:
                    points.append(p)
            current = target
        routes[f'{first[0]}_{first[1]}'] = points
    return routes


def write_dashes(lines, net_path, radius, desired_dash=3.0, desired_gap=3.0,
                 separated_edge=None, special_divider=None, ring=True, width=3.2,
                 junction_guides=(), omit_dashes=(), clear_crossing=None):
    dash, gap, count = dash_style(radius, desired_dash, desired_gap)
    routes = divider_routes(net_path, separated_edge, special_divider, width, junction_guides)
    strokes = 0
    for name, path in routes.items():
        total = _length(path)
        for k in range(max(0, math.floor((total-dash+1e-8)/(dash+gap))+1)):
            if f'lane_dash_{name}_{k}' in omit_dashes:
                continue
            start = k*(dash+gap)
            piece = _slice(path, start, start+dash)
            if clear_crossing is not None and name.startswith('W_'):
                from shapely.geometry import LineString
                if LineString(piece).buffer(0.075).intersects(clear_crossing):
                    continue
            lines.append(f'    <poly id="lane_dash_{name}_{k}" color="255,255,255" fill="0" '
                         f'lineWidth="0.15" layer="6" shape="{_write(piece)}"/>')
            strokes += 1
    if ring:
        for k in range(count):
            angle = k*(dash+gap)/radius
            sweep = dash/radius
            samples = max(2, math.ceil(dash/0.08))
            points = [(radius*math.cos(angle+sweep*i/samples), radius*math.sin(angle+sweep*i/samples))
                      for i in range(samples+1)]
            lines.append(f'    <poly id="lane_dash_ring_{k}" color="255,255,255" fill="0" '
                         f'lineWidth="0.15" layer="6" shape="{_write(points)}"/>')
            strokes += 1
    return {'white_dash_length_m': dash, 'white_dash_gap_m': gap,
            'white_dash_route_count': len(routes)+int(ring), 'white_dash_count': strokes}
