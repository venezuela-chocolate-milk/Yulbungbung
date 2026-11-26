"""Through-lane dividers following real SUMO connections."""
import xml.etree.ElementTree as ET
from shapely.geometry import LineString
from shapely.ops import unary_union
from env.roundabout_geometry import _read, _write, _offset, _slice, _length
from env.roundabout_builder import _movement_path

# Guide each west straight lane, including user lanes 2 and 3 (SUMO 3 and 2).
MOVEMENTS = [('west', 'SW_slip_main', 'SW_new_cross_out', (0, 1, 2, 3)),
             ('east', 'SW_end_T_extension_in', 'SW_new_cross_in', (1, 2, 3, 4)),
             ('lower', 'SW_new_lower_in', 'SW_new_upper_out', (1, 2, 3, 4)),
             ('upper', 'SW_new_upper_in', 'SW_new_lower_near', (1, 2, 3, 4))]
GUIDE_LANES = {'west': (0, 1, 2, 3), 'east': (1, 2, 3), 'lower': (1, 2, 3), 'upper': (1, 2, 3)}
OPPOSITE = {'west': 'east', 'east': 'west', 'lower': 'upper', 'upper': 'lower'}


def add_guides(root, net_path):
    net = ET.parse(net_path).getroot()
    edges = {e.get('id'): e for e in net.findall('edge')}
    stops = {arm: _read(root.find(f"poly[@id='{name}']").get('shape'))
             for arm, name in [('west', 'SW_end_T_extension_stop'),
                               ('east', 'SW_new_east_stop'),
                               ('lower', 'SW_lower_approach_stop'),
                               ('upper', 'SW_upper_approach_stop')]}

    def plane(points):
        a, b = points
        return LineString([(a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t)
                           for t in (-20, 20)])

    for p in list(root):
        if p.get('id', '').startswith(('SW_horizontal_straight_guide_', 'SW_lower5_straight_guide_')):
            root.remove(p)
    paths = {}
    marks = []
    for arm, source, target, lanes in MOVEMENTS:
        for index in GUIDE_LANES[arm]:
            src = edges[source].find(f"lane[@index='{index}']")
            connection = next(c for c in net.findall('connection')
                              if c.get('from') == source and c.get('to') == target
                              and c.get('fromLane') == str(index))
            dst = edges[target].find(f"lane[@index='{connection.get('toLane')}']")
            # Straight guides separate lanes; never extend a median boundary.
            source_inner = max(int(l.get('index')) for l in edges[source].findall('lane'))
            target_inner = max(int(l.get('index')) for l in edges[target].findall('lane'))
            if index == source_inner or int(connection.get('toLane')) == target_inner:
                continue
            center = _read(src.get('shape'))
            if arm == 'west':
                upstream = edges['SW_end_T_extension_out'].find(f"lane[@index='{index}']")
                center = _read(upstream.get('shape')) + center[1:]
            center += _movement_path(net_path, source, target, index)[1:]
            center += _read(dst.get('shape'))[1:]
            boundary = _offset(center, float(src.get('width', '3.2')) / 2)
            line = LineString(boundary)
            stations = []
            for end in (arm, OPPOSITE[arm]):
                hit = line.intersection(plane(stops[end]))
                if hit.geom_type != 'Point':
                    raise ValueError(f'Stop plane misses actual boundary: {arm}, {index}')
                stations.append(line.project(hit))
            path = _slice(boundary, *stations)
            paths[f'{arm}_{index}'] = path
            # Match the surrounding 3 m dashes and 3 m gaps.
            length = _length(path)
            count = max(2, round((length + 3) / 6))
            gap = (length - count * 3) / (count - 1)
            for i in range(count):
                segment = _slice(path, i * (3 + gap), i * (3 + gap) + 3)
                marks.append(dict(id=f'SW_horizontal_straight_guide_{arm}_{index}_{i}',
                                  color='255,255,255', fill='0', lineWidth='.15', layer='6',
                                  shape=_write(segment)))

    # Replace only existing thin white dashes that physically overlap the new paint.
    # Preserve all four left-turn guides even at crossing points.
    paint = unary_union([LineString(_read(m['shape'])).buffer(.075, cap_style=2) for m in marks])
    removed = []
    for p in list(root):
        name = p.get('id', '')
        if (p.get('color') != '255,255,255' or p.get('fill') != '0'
                or float(p.get('lineWidth', '.15')) > .2
                or name.startswith('SW_four_left_guide_')
                or 'left_guide' in name or 'arrow' in name or 'stop' in name):
            continue
        shape = _read(p.get('shape'))
        if len(shape) >= 2 and LineString(shape).buffer(float(p.get('lineWidth', '.15'))/2,
                                                       cap_style=2).intersects(paint):
            removed.append(name)
            root.remove(p)
    for m in marks:
        ET.SubElement(root, 'poly', **m)
    return paths, removed


def fit_east_receiving_lanes(root):
    """Send east user lanes 2/3/4/5 to west user lanes 1/2/3/4."""
    from env.roundabout_geometry import _curve, _unit
    edges = {e.get('id'): e for e in root.findall('edge')}
    lanes = {l.get('id'): l for e in edges.values() for l in e.findall('lane')}
    follow = {(c.get('from'), c.get('fromLane')): c for c in root.findall('connection')
              if c.get('from').startswith(':')}
    for index in (1, 2, 3, 4):
        connection = next(c for c in root.findall('connection')
                          if c.get('from') == 'SW_end_T_extension_in'
                          and c.get('to') == 'SW_new_cross_in'
                          and c.get('fromLane') == str(index))
        a = _read(lanes[f'SW_end_T_extension_in_{index}'].get('shape'))
        b = _read(lanes[f'SW_new_cross_in_{index}'].get('shape'))
        path = _curve(a[-1], b[0], _unit(a[-2], a[-1]), _unit(b[0], b[1]))
        chain = []
        via = connection.get('via')
        while via:
            chain.append(via)
            edge, lane = via.rsplit('_', 1)
            continuation = follow[(edge, lane)]
            via = continuation.get('via')
        connection.set('toLane', str(index))
        continuation.set('toLane', str(index))
        sizes = [float(lanes[key].get('length')) for key in chain]
        total = sum(sizes)
        length, start = _length(path), 0
        for i, (key, size) in enumerate(zip(chain, sizes)):
            end = length if i == len(chain)-1 else start + length*size/total
            piece = _slice(path, start, end)
            lanes[key].set('shape', _write(piece))
            lanes[key].set('length', str(_length(piece)))
            start = end
