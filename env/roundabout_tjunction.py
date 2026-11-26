"""Configurable northeast T junction; all coordinates are in metres."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from env.roundabout_geometry import _read, _write, _offset, _slice, _length


def finish_geometry(net_path, config):
    """Start the unconnected turn lane after the painted island's taper."""
    tree = ET.parse(net_path)
    lane = next(l for e in tree.getroot().findall('edge') if e.get('id') == 'E_out0_near'
                for l in e.findall('lane') if l.get('index') == '2')
    points = _read(lane.get('shape'))
    points = _slice(points, config.get('safety_tip_length', 16.0), _length(points))
    lane.set('shape', _write(points))
    lane.set('length', f'{_length(points):.6f}')
    # Match the actual traffic-light stopping position to the relocated paint.
    angle = math.radians(config['axis'])
    u = math.cos(angle), math.sin(angle)
    approach = next(e for e in tree.getroot().findall('edge') if e.get('id') == 'E_out0_near')
    for approach_lane in approach.findall('lane'):
        end = _read(approach_lane.get('shape'))[-1]
        distance = end[0]*u[0]+end[1]*u[1]-config['roundabout_stop_station']
        ET.SubElement(approach_lane, 'stopOffset', value=f'{max(0.0, distance):.6f}')
    main_in = next(e for e in tree.getroot().findall('edge') if e.get('id') == 'T_main_in')
    for main_lane in main_in.findall('lane'):
        ET.SubElement(main_lane, 'stopOffset',
                      value=str(1.5+config.get('main_stop_shift', 0.0)))
    end_near = next((e for e in tree.getroot().findall('edge')
                     if e.get('id') == 'T_end_main_near'), None)
    if end_near is not None:
        turn_lane = next(l for l in end_near.findall('lane') if l.get('index') == '2')
        shape = _read(turn_lane.get('shape'))
        tip_length = min(8.0, max(0.0, _length(shape)-6.5))
        shape = _slice(shape, tip_length, _length(shape))
        turn_lane.set('shape', _write(shape))
        turn_lane.set('length', f'{_length(shape):.6f}')
    from env.roundabout_end_widening import finish_geometry as finish_widened_end
    finish_widened_end(tree.getroot())
    from env.roundabout_sw_outer_lane import finish_geometry as finish_SW_outer_lane
    finish_SW_outer_lane(tree.getroot())
    extension = next((e for e in tree.getroot().findall('edge') if e.get('id') == 'SW_end_T_extension_out'), None)
    if extension is not None:
        inner = next(l for l in extension.findall('lane') if l.get('index') == '4')
        shape = _read(inner.get('shape'))
        shape = _slice(shape, 14.0, _length(shape))
        inner.set('shape', _write(shape))
        inner.set('length', f'{_length(shape):.6f}')
    from env.roundabout_road_first_corners import connect_lower5_to_upper4
    connect_lower5_to_upper4(tree.getroot())
    from env.roundabout_four_left_guides import fit_receiving_lanes
    fit_receiving_lanes(tree.getroot())
    from env.roundabout_horizontal_guides import fit_east_receiving_lanes
    fit_east_receiving_lanes(tree.getroot())
    tree.write(net_path, encoding='utf-8', xml_declaration=True)


def add_network(out_dir, config):
    base = Path(out_dir)
    nodes = ET.parse(base/'roundabout.nod.xml')
    edges = ET.parse(base/'roundabout.edg.xml')
    cons = ET.parse(base/'roundabout.con.xml')
    nr, er, cr = nodes.getroot(), edges.getroot(), cons.getroot()
    angle = math.radians(config['axis'])
    u, n = (math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle))
    station = config['distance']

    def point(a, lateral=0):
        return (u[0]*a+n[0]*lateral, u[1]*a+n[1]*lateral)

    center = point(station)
    branch_angle = math.radians(config['branch_axis'])
    v = (math.cos(branch_angle), math.sin(branch_angle))
    bn = (-v[1], v[0])

    def side(d, lateral):
        return (center[0]+v[0]*d+bn[0]*lateral,
                center[1]+v[1]*d+bn[1]*lateral)

    def node(name, xy, kind='priority'):
        ET.SubElement(nr, 'node', id=name, x=str(xy[0]), y=str(xy[1]), type=kind)

    for name in ('E_end0', 'E_end1'):
        nr.remove(next(e for e in nr if e.get('id') == name))
    node('T_junction', center, 'traffic_light')
    bay_start = station-config['turn_bay_length']
    taper_start = bay_start-config.get('taper_length', 45.0)
    node('T_bay', point(bay_start, -6.9))
    node('T_main_end', point(station+config['main_extension']))
    node('T_side_end', side(config['branch_length'], 0))
    outgoing = next(e for e in er if e.get('id') == 'E_out0')
    incoming = next(e for e in er if e.get('id') == 'E_in1')
    shape = _read(outgoing.get('shape'))
    # Move both existing through lanes outward before opening the inner turn
    # lane. A smooth taper replaces the previous square step in the road edge.
    shape = shape[:-1] + [point(taper_start+(bay_start-taper_start)*k/90,
                               -3.7-3.2*(3*(k/90)**2-2*(k/90)**3))
                         for k in range(91)]
    outgoing.set('to', 'T_bay')
    outgoing.set('shape', _write(shape))
    incoming.set('from', 'T_junction')

    def edge(name, fr, to, lanes, shape, speed=16.67, priority=3):
        ET.SubElement(er, 'edge', id=name, **{'from': fr, 'to': to,
            'numLanes': str(lanes), 'width': '3.2', 'speed': str(speed),
            'priority': str(priority), 'spreadType': 'center', 'shape': _write(shape)})

    # Extra inner lane is reserved for the left turn. The opposing approach
    # has two through lanes, with a shared right turn from the outer lane.
    edge('E_out0_near', 'T_bay', 'T_junction', 3,
         [point(station-config['turn_bay_length'], -5.3), point(station, -5.3)])
    edge('T_main_out', 'T_junction', 'T_main_end', 2,
         [point(station, -3.7), point(station+config['main_extension'], -3.7)])
    edge('T_main_in', 'T_main_end', 'T_junction', 2,
         [point(station+config['main_extension'], 3.7), point(station, 3.7)])
    edge('T_side_out', 'T_junction', 'T_side_end', 2,
         [side(0, -3.45), side(config['branch_length'], -3.45)], 11.11, 2)
    edge('T_side_in', 'T_side_end', 'T_junction', 4,
         [side(config['branch_length'], 6.65), side(0, 6.65)], 11.11, 2)
    movements = [('E_out0', 'E_out0_near', 0, 0), ('E_out0', 'E_out0_near', 1, 1),
                 ('E_out0_near', 'T_main_out', 0, 0),
                 ('E_out0_near', 'T_main_out', 1, 1),
                 ('E_out0_near', 'T_side_out', 2, 1),
                 ('T_main_in', 'E_in1', 0, 0), ('T_main_in', 'E_in1', 1, 1),
                 ('T_main_in', 'T_side_out', 0, 0),
                 ('T_side_in', 'T_main_out', 0, 0),
                 ('T_side_in', 'T_main_out', 1, 1),
                 ('T_side_in', 'E_in1', 2, 0), ('T_side_in', 'E_in1', 3, 1)]
    for fr, to, fl, tl in movements:
        ET.SubElement(cr, 'connection', **{'from': fr, 'to': to,
                      'fromLane': str(fl), 'toLane': str(tl)})
    from env.roundabout_config import END_T_JUNCTION
    if END_T_JUNCTION:
        # Extend the original endpoint into an exact right-angle T junction.
        next(e for e in nr if e.get('id') == 'T_main_end').set(
            'radius', str(END_T_JUNCTION.get('junction_radius', 12.0)))
        end_station = station+config['main_extension']
        side_u = n
        side_n = (-side_u[1], side_u[0])
        end_center = point(end_station)
        def end_side(distance, lateral):
            return (end_center[0]+side_u[0]*distance+side_n[0]*lateral,
                    end_center[1]+side_u[1]*distance+side_n[1]*lateral)
        main_length = END_T_JUNCTION['main_extension']
        side_length = END_T_JUNCTION['branch_length']
        bay = end_station-END_T_JUNCTION['turn_bay_length']
        taper = bay-END_T_JUNCTION['taper_length']
        node('T_end_bay', point(bay, -6.9))
        previous = next(e for e in er if e.get('id') == 'T_main_out')
        previous.set('to', 'T_end_bay')
        previous.set('shape', _write([point(station, -3.7), point(taper, -3.7)]
            + [point(taper+(bay-taper)*k/90, -3.7-3.2*(3*(k/90)**2-2*(k/90)**3))
               for k in range(1, 91)]))
        edge('T_end_main_near', 'T_end_bay', 'T_main_end', 3,
             [point(bay, -5.3), point(end_station, -5.3)])
        node('T_end_main_end', point(end_station+main_length))
        node('T_end_side_end', end_side(side_length, 0))
        edge('T_end_main_out', 'T_main_end', 'T_end_main_end', 3,
             [point(end_station, -5.3), point(end_station+main_length, -5.3)])
        edge('T_end_main_in', 'T_end_main_end', 'T_main_end', 2,
             [point(end_station+main_length, 3.7), point(end_station, 3.7)])
        edge('T_end_side_out', 'T_main_end', 'T_end_side_end', 2,
             [end_side(0, -3.7), end_side(side_length, -3.7)], 11.11, 1)
        edge('T_end_side_in', 'T_end_side_end', 'T_main_end', 2,
             [end_side(side_length, 3.7), end_side(0, 3.7)], 11.11, 1)
        for fr, to, pairs in [
                ('T_main_out', 'T_end_main_near', [(0, 0), (1, 1)]),
                ('T_end_main_near', 'T_end_main_out', [(0, 0), (1, 1), (2, 2)]),
                ('T_end_main_in', 'T_main_in', [(0, 0), (1, 1)]),
                ('T_end_main_in', 'T_end_side_out', [(0, 0)]),
                ('T_end_side_in', 'T_end_main_out', [(0, 0)]),
                ('T_end_side_in', 'T_main_in', [(1, 1)])]:
            for fl, tl in pairs:
                ET.SubElement(cr, 'connection', **{'from': fr, 'to': to,
                              'fromLane': str(fl), 'toLane': str(tl)})
    from env.roundabout_stop_tjunction import add_network as add_stop_network
    add_stop_network(nr, er, cr)
    from env.roundabout_crossroads import add_network as add_crossroads_network
    add_crossroads_network(nr,er,cr)
    for tree, suffix in ((nodes, 'nod'), (edges, 'edg'), (cons, 'con')):
        tree.write(base/f'roundabout.{suffix}.xml', encoding='utf-8', xml_declaration=True)


def add_routes(out_dir):
    path = Path(out_dir)/'roundabout.rou.xml'
    tree = ET.parse(path)
    root = tree.getroot()
    for flow in root.findall('flow'):
        if flow.get('from') == 'E_in1':
            flow.set('from', 'T_main_in')
        if flow.get('to') == 'E_out0':
            flow.set('to', 'T_main_out')
    for i, (fr, to) in enumerate([('E_out0', 'T_side_out'), ('T_side_in', 'W_out0_far'),
                                  ('T_main_in', 'T_side_out'), ('T_side_in', 'T_main_out')]):
        ET.SubElement(root, 'flow', id=f'tjunction_{i}', type='car',
                      **{'from': fr, 'to': to, 'begin': '0', 'end': '100000',
                         'vehsPerHour': '90', 'departLane': 'best', 'departSpeed': 'max'})
    from env.roundabout_config import END_T_JUNCTION
    if END_T_JUNCTION:
        for flow in root.findall('flow'):
            if flow.get('from') == 'T_main_in':
                flow.set('from', 'T_end_main_in')
            if flow.get('to') == 'T_main_out':
                flow.set('to', 'T_end_main_out')
        for i, (fr, to) in enumerate([('T_end_side_in', 'W_out0_far'),
                                      ('T_end_main_in', 'T_end_side_out'),
                                      ('T_end_side_in', 'T_end_main_out')]):
            ET.SubElement(root, 'flow', id=f'end_tjunction_{i}', type='car',
                          **{'from': fr, 'to': to, 'begin': '0', 'end': '100000',
                             'vehsPerHour': '60', 'departLane': 'best', 'departSpeed': 'max'})
    from env.roundabout_stop_tjunction import add_routes as add_stop_routes
    add_stop_routes(root)
    from env.roundabout_crossroads import add_routes as add_crossroads_routes
    add_crossroads_routes(root)
    tree.write(path, encoding='utf-8', xml_declaration=True)


def add_markings(out_dir):
    from env.roundabout_builder import (_arrow, _stop_line, _center_line,
                                       _read_lane_shapes, _hatch, _point_from_end)
    from env.roundabout_config import T_JUNCTION, END_T_JUNCTION
    base = Path(out_dir)
    net = ET.parse(base/'roundabout.net.xml').getroot()
    shapes = {e.get('id'): [_read(l.get('shape')) for l in e.findall('lane')]
              for e in net.findall('edge') if e.get('function') != 'internal'}
    lane_shapes = _read_lane_shapes(base/'roundabout.net.xml')
    lines = []
    main_shift = T_JUNCTION.get('main_stop_shift', 0.0)
    for edge, arrows in [('E_out0_near', ['straight', 'straight', 'left']),
                         ('T_main_in', ['straight_right', 'straight']),
                         ('T_end_main_in', ['straight_right', 'straight']),
                         ('T_end_main_near', ['straight', 'straight', 'straight']),
                         ('T_side_in', ['right', 'right', 'left', 'left'])]:
        if edge not in shapes:
            continue
        stop_distance = 1.5
        if edge == 'T_main_in':
            stop_distance += main_shift
        if edge == 'E_out0_near':
            angle = math.radians(T_JUNCTION['axis'])
            end = lane_shapes[(edge, 0)][-1]
            stop_distance = (end[0]*math.cos(angle)+end[1]*math.sin(angle)
                             -T_JUNCTION['roundabout_stop_station'])
        if edge not in ('T_end_main_in', 'T_end_main_near'):
            _stop_line(lines, f't_stop_{edge}', lane_shapes, edge, dist=stop_distance)
        for i, kind in enumerate(arrows):
            arrow_distance = 14.0+(main_shift if edge == 'T_main_in' else 0.0)
            if edge == 'T_end_main_in':
                arrow_distance = END_T_JUNCTION.get('main_arrow_distance', 2.25)
            elif edge == 'T_end_main_near':
                arrow_distance = min(14.0, min(_length(p) for p in shapes[edge])-4.0)
            _arrow(lines, f't_arrow_{edge}_{i}', lane_shapes, edge, i, kind, arrow_distance)
            if edge not in ('T_side_in', 'T_main_in', 'T_end_main_in', 'T_end_main_near'):
                _arrow(lines, f't_arrow_repeat_{edge}_{i}', lane_shapes, edge, i, kind, 32.0)
    if 'T_end_main_out' in shapes and END_T_JUNCTION.get('outgoing_stop_from_junction') is not None:
        angle = math.radians(T_JUNCTION['axis'])
        target = (T_JUNCTION['distance']+T_JUNCTION['main_extension']
                  +END_T_JUNCTION['outgoing_stop_from_junction'])
        end = lane_shapes[('T_end_main_out', 0)][-1]
        distance = end[0]*math.cos(angle)+end[1]*math.sin(angle)-target
        _stop_line(lines, 't_stop_end_main_out', lane_shapes, 'T_end_main_out', dist=distance)
        # Straight tips sit 2 m behind the stop line; the shared right-turn
        # marking uses the same stem and origin as the other two arrows.
        for lane, kind in enumerate(('straight_right', 'straight', 'straight')):
            _arrow(lines, f't_arrow_end_stop_{lane}', lane_shapes,
                   'T_end_main_out', lane, kind, distance+4.1)
    # Side road has two outbound and four inbound lanes; offset their inner
    # lane boundaries to locate the center marking without widening the lanes.
    center_paths = {}
    for name, outward, inward in [('main', 'T_main_out', 'T_main_in'),
                                   ('side', 'T_side_out', 'T_side_in'),
                                   ('end_main', 'T_end_main_out', 'T_end_main_in'),
                                   ('end_side', 'T_end_side_out', 'T_end_side_in'),
                                   ('stop_main', 'T_stop_main_out', 'T_stop_main_in'),
                                   ('stop_side', 'T_stop_side_out', 'T_stop_side_in')]:
        if outward not in shapes:
            continue
        a = _offset(shapes[outward][-1], 1.6)
        b = list(reversed(_offset(shapes[inward][-1], 1.6)))
        points = [((p[0]+q[0])/2, (p[1]+q[1])/2) for p, q in zip(a, b)]
        if name == 'main':
            # The median starts at the relocated stop line, rather than running
            # through the longer turning-guide area.
            points = _slice(points, main_shift+1.5, _length(points))
            if 'T_end_main_near' in shapes:
                angle = math.radians(T_JUNCTION['axis'])
                start = points[0][0]*math.cos(angle)+points[0][1]*math.sin(angle)
                end = T_JUNCTION['distance']+T_JUNCTION['main_extension']
                points = [(a*math.cos(angle), a*math.sin(angle)) for a in (start, end)]
        center_paths[name] = points
        lines.append(f'    <poly id="t_median_surface_{name}" color="84,84,90" '
                     f'fill="0" lineWidth="1.0" layer="5" shape="{_write(points)}"/>')
        _center_line(lines, f't_center_{name}', points)
    if 'end_main' in center_paths:
        # Continue the main-road median across the new T junction and bring
        # the side-road median up to it, including the unpaved narrow tips.
        end_station = T_JUNCTION['distance']+T_JUNCTION['main_extension']
        angle = math.radians(T_JUNCTION['axis'])
        junction_center = (end_station*math.cos(angle), end_station*math.sin(angle))
        for name, points in [('end_main_link', [center_paths['main'][-1],
                                               center_paths['end_main'][0]]),
                             ('end_side_link', [junction_center, center_paths['end_side'][0]])]:
            lines.append(f'    <poly id="t_median_surface_{name}" color="84,84,90" '
                         f'fill="0" lineWidth="1.0" layer="5" shape="{_write(points)}"/>')
            _center_line(lines, f't_center_{name}', points)
    if 'stop_main' in center_paths:
        from env.roundabout_stop_tjunction import station as stop_station
        angle = math.radians(T_JUNCTION['axis'])
        stop_center = (stop_station()*math.cos(angle), stop_station()*math.sin(angle))
        for name, points in [('stop_main_link', [center_paths['end_main'][-1],center_paths['stop_main'][0]]),
                             ('stop_side_link', [stop_center,center_paths['stop_side'][0]])]:
            lines.append(f'    <poly id="t_median_surface_{name}" color="84,84,90" '
                         f'fill="0" lineWidth="1.0" layer="5" shape="{_write(points)}"/>')
            _center_line(lines,f't_center_{name}',points)
    tree = ET.parse(base/'roundabout.deco.xml')
    root = tree.getroot()
    # Connect the full road cross-sections at the three stop lines. This is
    # junction pavement only; painted islands upstream retain their geometry.
    from shapely.geometry import MultiPoint
    main_angle = math.radians(T_JUNCTION['axis'])
    u = math.cos(main_angle), math.sin(main_angle)
    n = -u[1], u[0]
    side_angle = math.radians(T_JUNCTION['branch_axis'])
    v = math.cos(side_angle), math.sin(side_angle)
    bn = -v[1], v[0]
    center = (u[0]*T_JUNCTION['distance'], u[1]*T_JUNCTION['distance'])
    side_end, _ = _point_from_end(lane_shapes[('T_side_in', 3)], 1.5)
    side_station = ((side_end[0]-center[0])*v[0]+(side_end[1]-center[1])*v[1])
    main_end, _ = _point_from_end(lane_shapes[('T_main_in', 0)], 1.5+main_shift)
    main_station = main_end[0]*u[0]+main_end[1]*u[1]
    def main_point(a, lateral):
        return (u[0]*a+n[0]*lateral, u[1]*a+n[1]*lateral)
    def side_point(a, lateral):
        return (center[0]+v[0]*a+bn[0]*lateral,
                center[1]+v[1]*a+bn[1]*lateral)
    corners = [main_point(T_JUNCTION['roundabout_stop_station'], offset)
               for offset in (-10.1, 6.9)]
    corners += [main_point(main_station, offset) for offset in (-6.9, 6.9)]
    corners += [side_point(side_station, offset) for offset in (-6.65, 13.05)]
    junction_surface = MultiPoint(corners).convex_hull
    # Match the two sketched curb returns, with tangents parallel to each road.
    # Keep the lower main-road boundary and all lane centerlines unchanged.
    def curb_curve(a, b, ta, tb):
        reach = math.dist(a, b)*0.42
        c = (a[0]+ta[0]*reach, a[1]+ta[1]*reach)
        d = (b[0]-tb[0]*reach, b[1]-tb[1]*reach)
        return [tuple((1-t)**3*a[j]+3*(1-t)**2*t*c[j]
                      +3*(1-t)*t*t*d[j]+t**3*b[j] for j in (0, 1))
                for t in (k/120 for k in range(121))]
    from shapely.geometry import Polygon
    sw_station = T_JUNCTION['roundabout_stop_station']-10.0
    ne_station = main_station-main_shift+8.0
    branch_station = side_station+8.0
    right_curb = curb_curve(main_point(ne_station, 6.9),
                            side_point(branch_station, -6.65),
                            (-u[0], -u[1]), v)
    left_curb = curb_curve(side_point(branch_station, 13.05),
                           main_point(sw_station, 6.9),
                           (-v[0], -v[1]), (-u[0], -u[1]))
    curb_surface = Polygon([main_point(sw_station, -10.1),
                            main_point(main_station, -6.9),
                            *right_curb, side_point(branch_station, 13.05),
                            *left_curb])
    if not curb_surface.is_valid:
        raise ValueError('The custom T-junction curb outline is invalid')
    # Every approach retains its lane separators. Only the intersection core
    # uses the two manually specified curved guides instead of divider dashes.
    from shapely.geometry import LineString
    for element in list(root):
        if element.get('id', '').startswith(('lane_dash_E_', 'lane_dash_E_out',
                                             'lane_dash_E_in', 'lane_dash_T_')):
            if LineString(_read(element.get('shape'))).intersects(junction_surface.buffer(0.2)):
                root.remove(element)
    ET.SubElement(root, 'poly', id='t_full_junction_surface', color='84,84,90',
                  fill='1', layer='4.8', shape=_write(list(curb_surface.exterior.coords)))

    # Two smooth dashed curves follow the user's sketch between median ends.
    from env.roundabout_markings import dash_style
    dash, gap, _ = dash_style(20.0)
    def emit_guide(name, path):
        total = _length(path)
        for k in range(max(0, math.floor((total-dash)/(dash+gap))+1)):
            piece = _slice(path, k*(dash+gap), k*(dash+gap)+dash)
            ET.SubElement(root, 'poly', id=f't_user_guide_{name}_{k}',
                          color='255,255,255', fill='0', lineWidth='0.15',
                          layer='6', shape=_write(piece))
    def guide(name, a, b, tangent_a, tangent_b):
        reach = math.dist(a, b)*0.45
        c = (a[0]+tangent_a[0]*reach, a[1]+tangent_a[1]*reach)
        d = (b[0]-tangent_b[0]*reach, b[1]-tangent_b[1]*reach)
        path = []
        for k in range(201):
            t = k/200
            path.append(tuple((1-t)**3*a[j]+3*(1-t)**2*t*c[j]
                              +3*(1-t)*t*t*d[j]+t**3*b[j] for j in (0, 1)))
        emit_guide(name, path)
        return path
    branch_median = side_point(side_station, 0)
    guide('roundabout_to_side', main_point(T_JUNCTION['roundabout_stop_station'], 0),
          branch_median, u, v)
    guide('side_to_main', branch_median, main_point(main_station, 0),
          (-v[0], -v[1]), u)
    # Join the two through-lane boundaries without changing any road geometry.
    # Derive the second curve by a true 3.2 m normal offset, rather than drawing
    # separate curves whose separation could change through the intersection.
    through = guide('main_inner',
                    main_point(T_JUNCTION['roundabout_stop_station'], -3.7),
                    main_point(main_station, -0.5), u, u)
    # Continue one dash phase from the curved guide onto the outgoing road.
    # Separate patterns and removed boundary-straddling dashes left a long gap.
    for element in list(root):
        if not element.get('id', '').startswith('lane_dash_'):
            continue
        points = _read(element.get('shape'))
        a = sum(p[0]*u[0]+p[1]*u[1] for p in points)/len(points)
        if a > main_station-0.2 and element.get('id', '').startswith('lane_dash_E_out0_0_'):
            root.remove(element)
    outer = _offset(through, -3.2)
    from env.roundabout_markings import divider_routes
    divider_paths = divider_routes(base/'roundabout.net.xml')
    downstream = divider_paths['E_out0_0']
    outer.extend(p for p in downstream if p[0]*u[0]+p[1]*u[1] > main_station+0.001)
    emit_guide('main_outer', outer)
    if 'T_end_main_near_1' in divider_paths:
        from shapely.geometry import Point
        inner_path = divider_paths['T_end_main_near_1']
        start_point = _offset(shapes['T_end_main_near'][2], -1.6)[0]
        start_distance = LineString(inner_path).project(Point(start_point))
        inner_path = _slice(inner_path, start_distance, _length(inner_path))
        for element in list(root):
            if element.get('id', '').startswith('lane_dash_T_end_main_near_1_'):
                root.remove(element)
        emit_guide('end_inner_continuous', inner_path)
    # Opposing through movement: retain a single dash phase on the existing
    # lane divider and add its parallel outer guide across the junction.
    opposite_name = next(name for name in ('T_main_in_0','T_end_main_in_0','T_stop_main_in_0','X_main_in_0')
                         if name in divider_paths)
    opposite = divider_paths[opposite_name]
    for element in list(root):
        if element.get('id', '').startswith('lane_dash_'+opposite_name+'_'):
            root.remove(element)
    emit_guide('opposite_divider', opposite)
    guide('opposite_outer', main_point(main_station, 6.9),
          main_point(T_JUNCTION['roundabout_stop_station'], 6.9),
          (-u[0], -u[1]), (-u[0], -u[1]))
    # The space released beside the centerline is a painted safety island,
    # bounded by the final lane geometry rather than by a wider road surface.
    from env.roundabout_surfaces import _network, _polygons
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    _, _, strips = _network(base/'roundabout.net.xml')
    road = unary_union(list(strips.values()))
    angle = math.radians(T_JUNCTION['axis'])
    ux, uy = math.cos(angle), math.sin(angle)
    nx, ny = -uy, ux
    bay_start = T_JUNCTION['distance']-T_JUNCTION['turn_bay_length']
    taper_start = bay_start-T_JUNCTION.get('taper_length', 45.0)
    def point(a, lateral):
        return (ux*a+nx*lateral, uy*a+ny*lateral)
    path = list(shapes['E_out0'][-1])
    connections = net.findall('connection')
    turn = next(c for c in connections if c.get('from') == 'E_out0'
                and c.get('to') == 'E_out0_near' and c.get('fromLane') == '1')
    all_lanes = {l.get('id'): _read(l.get('shape'))
                 for e in net.findall('edge') for l in e.findall('lane')}
    follow = {(c.get('from'), c.get('fromLane')): c for c in connections
              if c.get('from', '').startswith(':')}
    current = turn.get('via')
    while current:
        path.extend(all_lanes[current][1:])
        edge_id, lane_id = current.rsplit('_', 1)
        current = follow[(edge_id, lane_id)].get('via')
    border = _offset(path, 1.6)
    border = [p for p in border if p[0]*ux+p[1]*uy >= taper_start-0.01]
    if len(border) >= 2:
        finish = border[-1][0]*ux+border[-1][1]*uy
        last_lateral = border[-1][0]*nx+border[-1][1]*ny
        tip_length = T_JUNCTION.get('safety_tip_length', 16.0)
        # Round off the broad end into a long nose rather than leaving a
        # perpendicular end or a jagged outline at the new lane's entrance.
        nose = [point(finish+tip_length*k/64,
                      last_lateral+(-0.35-last_lateral)*(3*(k/64)**2-2*(k/64)**3))
                for k in range(1, 65)]
        seed = Polygon([point(taper_start, -0.35), *border, *nose])
        # Neutral painted shoulder beneath the tapered nose; no lane is added.
        shoulder = Polygon([border[-1], point(finish+tip_length, last_lateral),
                            point(finish+tip_length, -0.35), point(finish, -0.35)])
        for i, polygon in enumerate(_polygons(shoulder.difference(road))):
            ET.SubElement(root, 'poly', id=f't_safety_shoulder_{i}', color='84,84,90',
                          fill='1', layer='5', shape=_write(list(polygon.exterior.coords)))
        region = seed.buffer(0).difference(road.buffer(0.14, quad_segs=24))
        for i, polygon in enumerate(_polygons(region)):
            if polygon.area < 0.05:
                continue
            points = list(polygon.exterior.coords)[:-1]
            name = f't_taper_safety_{i}'
            ET.SubElement(root, 'poly', id=name+'_surface', color='84,84,90',
                          fill='1', layer='5', shape=_write(points))
            _hatch(lines, name, points, T_JUNCTION['axis'], layer=6, pattern='diagonal')
    for line in lines:
        root.append(ET.fromstring(line))
    if 'T_end_main_out' in shapes:
        refine_end_junction(root, net, base/'roundabout.net.xml', T_JUNCTION)
    if 'T_stop_main_out' in shapes:
        from env.roundabout_stop_tjunction import clear_markings, align_side_dashes, add_side_stop_line, add_turn_guide, extend_exit_dashes, align_exit_dividers
        clear_markings(root)
        align_side_dashes(root)
        add_side_stop_line(root,base/'roundabout.net.xml')
        add_turn_guide(root)
        extend_exit_dashes(root)
        align_exit_dividers(root)
    if 'X_main_out' in shapes:
        from env.roundabout_crossroads import add_markings as add_crossroads_markings
        add_crossroads_markings(root,base/'roundabout.net.xml')
    tree.write(base/'roundabout.deco.xml', encoding='utf-8', xml_declaration=True)
    # Open on the new intersection so its first draft can be reviewed directly.
    settings = ET.parse(base/'roundabout.gui.xml')
    viewport = settings.getroot().find('viewport')
    angle = math.radians(T_JUNCTION['axis'])
    from env.roundabout_config import END_T_JUNCTION
    focus_station = T_JUNCTION['distance']+(T_JUNCTION['main_extension'] if END_T_JUNCTION else 0)
    if 'T_stop_main_out' in shapes:
        from env.roundabout_stop_tjunction import station as stop_station
        focus_station = stop_station()
    if 'X_main_out' in shapes:
        from env.roundabout_crossroads import center_station
        focus_station=center_station()
    viewport.set('x', str(focus_station*math.cos(angle)))
    viewport.set('y', str(focus_station*math.sin(angle)))
    viewport.set('zoom', str(float(viewport.get('zoom'))*140/230))
    if ('SE_triangle_forward_out',0) in shapes:
        from env.roundabout_end_widening import layout
        point,_,_,road_end=layout()
        focus=point(road_end+51-8,-20)
        net=ET.parse(base/'roundabout.net.xml').getroot()
        bounds=list(map(float,net.find('location').get('convBoundary').split(',')))
        viewport.set('x',str(focus[0]))
        viewport.set('y',str(focus[1]))
        viewport.set('zoom',str((bounds[2]-bounds[0])/150.0*100))
        viewport.set('angle','0')
    if ('SW_final_forward_out',0) in shapes:
        from env.roundabout_config import END_CROSSROADS
        point,_,_,road_end=layout()
        focus=point(road_end+51,-END_CROSSROADS['sw_arm_length']-8)
        viewport.set('x',str(focus[0]))
        viewport.set('y',str(focus[1]))
        viewport.set('zoom',str((bounds[2]-bounds[0])/170.0*100))
    if ('SW_end_T_left_out',0) in shapes:
        focus=point(road_end+51,-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']+12)
        viewport.set('x',str(focus[0]))
        viewport.set('y',str(focus[1]))
        view_width=max(170.0,2*END_CROSSROADS['sw_end_t_arm_length']+30)
        viewport.set('zoom',str((bounds[2]-bounds[0])/view_width*100))
    settings.write(base/'roundabout.gui.xml', encoding='utf-8', xml_declaration=True)


def refine_end_junction(root, net, net_path, config):
    """Clear the sketched markings and soften the local curb return only."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    from env.roundabout_surfaces import _network, _polygons, _lines
    from env.roundabout_config import END_T_JUNCTION
    angle = math.radians(config['axis'])
    u = math.cos(angle), math.sin(angle)
    v = -u[1], u[0]
    vn = -v[1], v[0]
    station = config['distance']+config['main_extension']
    center = u[0]*station, u[1]*station
    def side(d, lateral):
        return (center[0]+v[0]*d+vn[0]*lateral,
                center[1]+v[1]*d+vn[1]*lateral)
    branch_clearance = END_T_JUNCTION.get('branch_marking_clearance', 15.0)
    main_clearance = END_T_JUNCTION.get('main_guide_clearance', 8.0)
    clear_median = Polygon([side(-1, -1), side(branch_clearance, -1),
                            side(branch_clearance, 1), side(-1, 1)])
    for element in list(root):
        name = element.get('id', '')
        if name.startswith(('t_center_end_side_', 't_center_end_side_link_')):
            pieces = _lines(LineString(_read(element.get('shape'))).difference(clear_median))
            root.remove(element)
            for i, piece in enumerate(pieces):
                if piece.length < 0.05:
                    continue
                clone = ET.Element('poly', element.attrib)
                clone.set('id', name if i == 0 else name+f'_part{i}')
                clone.set('shape', _write(list(piece.coords)))
                root.append(clone)
        elif name.startswith(('lane_dash_T_end_side_in_', 'lane_dash_T_end_side_out_')):
            points = _read(element.get('shape'))
            if any((p[0]-center[0])*v[0]+(p[1]-center[1])*v[1] < branch_clearance for p in points):
                root.remove(element)
        elif name.startswith('t_user_guide_opposite_divider_'):
            points = _read(element.get('shape'))
            if any(abs(p[0]*u[0]+p[1]*u[1]-station) < main_clearance for p in points):
                root.remove(element)
    node = next(n for n in net.findall('junction') if n.get('id') == 'T_main_end')
    junction = Polygon(_read(node.get('shape'))).buffer(0)
    _, _, strips = _network(net_path)
    nearby = unary_union(list(strips.values())).intersection(junction.buffer(25))
    smooth = junction.union(nearby).buffer(8, quad_segs=32).buffer(-8, quad_segs=32)
    smooth = smooth.simplify(0.005, preserve_topology=True).buffer(0)
    for i, polygon in enumerate(_polygons(smooth)):
        ET.SubElement(root, 'poly', id=f'end_t_smooth_corner_{i}', color='84,84,90',
                      fill='1', layer='4.8', shape=_write(list(polygon.exterior.coords)))
    # Painted taper beside the two outward-shifting through lanes. The new
    # inner turn lane starts after its rounded nose, outside the painted area.
    from env.roundabout_builder import _hatch
    from env.roundabout_config import END_T_JUNCTION
    road = unary_union(list(strips.values()))
    lanes = {l.get('id'): _read(l.get('shape')) for e in net.findall('edge')
             for l in e.findall('lane')}
    path = list(lanes['T_main_out_1'])
    connections = net.findall('connection')
    connection = next(c for c in connections if c.get('from') == 'T_main_out'
                      and c.get('to') == 'T_end_main_near' and c.get('fromLane') == '1')
    follow = {(c.get('from'), c.get('fromLane')): c for c in connections
              if c.get('from', '').startswith(':')}
    current = connection.get('via')
    while current:
        path.extend(lanes[current][1:])
        edge, index = current.rsplit('_', 1)
        current = follow[(edge, index)].get('via')
    start = station-END_T_JUNCTION['turn_bay_length']-END_T_JUNCTION['taper_length']
    border = [p for p in _offset(path, 1.6) if p[0]*u[0]+p[1]*u[1] >= start-0.01]
    def main(a, lateral):
        return (u[0]*a+v[0]*lateral, u[1]*a+v[1]*lateral)
    finish = border[-1][0]*u[0]+border[-1][1]*u[1]
    last = border[-1][0]*v[0]+border[-1][1]*v[1]
    turn_start = lanes['T_end_main_near_2'][0]
    tip_length = max(0.0, turn_start[0]*u[0]+turn_start[1]*u[1]-finish)
    nose = [main(finish+tip_length*t, last+(-0.35-last)*(3*t*t-2*t*t*t))
            for t in (k/64 for k in range(1,65))]
    seed = Polygon([main(start,-0.35), *border, *nose]).buffer(0)
    shoulder = Polygon([border[-1], main(finish+tip_length,last), main(finish+tip_length,-0.35),
                        main(finish,-0.35)])
    for i, p in enumerate(_polygons(shoulder.difference(road))):
        ET.SubElement(root,'poly',id=f'end_t_safety_shoulder_{i}',color='84,84,90',
                      fill='1',layer='5',shape=_write(list(p.exterior.coords)))
    for i,p in enumerate(_polygons(seed.difference(road.buffer(0.14)))):
        if p.area < 0.05:
            continue
        points = list(p.exterior.coords)[:-1]
        name = f'end_t_safety_{i}'
        ET.SubElement(root,'poly',id=name+'_surface',color='84,84,90',fill='1',
                      layer='5',shape=_write(points))
        markings=[]
        _hatch(markings,name,points,config['axis'],layer=6,pattern='diagonal')
        for marking in markings:
            root.append(ET.fromstring(marking))
