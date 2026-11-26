"""Complete the outside receiving-lane divider beside the southwest island."""
import xml.etree.ElementTree as ET
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from env.roundabout_geometry import _read, _write, _offset, _slice
from env.roundabout_builder import _movement_path
from env.roundabout_markings import dash_style

PREFIX = 'SW_lower_outer_divider_extension_'


def add_divider(root, net_path):
    net = ET.parse(net_path).getroot()
    def lane(edge):
        return _read(net.find(f"edge[@id='{edge}']/lane[@index='0']").get('shape'))
    # The outer receiving lane is reached by upper approach lane 1.
    center = _movement_path(net_path, 'SW_new_upper_in', 'SW_new_lower_near', 1)
    center += lane('SW_new_lower_near')[1:]
    center += _movement_path(net_path, 'SW_new_lower_near', 'SW_new_lower_out', 0)[1:]
    center += lane('SW_new_lower_out')[1:]
    width = float(net.find("edge[@id='SW_new_lower_near']/lane[@index='0']").get('width', '3.2'))
    boundary = _offset(center, width/2)
    line = LineString(boundary)
    island = Polygon(_read(root.find("poly[@id='SW_drawn_island_border_3']").get('shape')))
    nose = max(island.exterior.coords, key=lambda q: q[0])
    start = line.project(Point(nose))
    anchors = []
    for p in list(root):
        if p.get('id', '').startswith(PREFIX):
            root.remove(p)
            continue
        if not p.get('id', '').startswith('SW_new_straight_dash_lower_7_'):
            continue
        points = _read(p.get('shape'))
        if all(line.distance(Point(q)) < .05 for q in points):
            anchors.append(min(line.project(Point(q)) for q in points))
    if not anchors:
        raise ValueError('Cannot find existing outside divider phase')
    anchor = min(a for a in anchors if a > start)
    dash, gap, _ = dash_style(20)
    end = anchor-gap
    segments = []
    # New upper through guides may already cover the same outside divider.
    existing = unary_union([
        LineString(_read(p.get('shape'))).buffer(.075, cap_style=2)
        for p in root if p.get('id', '').startswith('SW_horizontal_straight_guide_upper_1_')
    ])
    while end > start:
        begin = max(start, end-dash)
        if end-begin > .5:
            piece = _slice(boundary, begin, end)
            if LineString(piece).buffer(.075, cap_style=2).intersects(island):
                raise ValueError('Outside divider overlaps the safety island')
            if LineString(piece).buffer(.075, cap_style=2).intersects(existing):
                end -= dash+gap
                continue
            ET.SubElement(root, 'poly', id=f'{PREFIX}{len(segments)}', color='255,255,255',
                          fill='0', lineWidth='.15', layer='6', shape=_write(piece))
            segments.append(piece)
        end -= dash+gap
    return segments


def add_receiving_continuations(root, net_path):
    """Bridge the two remaining receiving dividers to their existing dash phase."""
    prefix = 'SW_lower_receiving_divider_'
    for p in list(root):
        if p.get('id', '').startswith(prefix):
            root.remove(p)
    net = ET.parse(net_path).getroot()
    dash, gap, _ = dash_style(20)
    pieces = []
    for index in (1, 2):
        def shape(edge):
            return _read(net.find(f"edge[@id='{edge}']/lane[@index='{index}']").get('shape'))
        center = _movement_path(net_path, 'SW_new_upper_in', 'SW_new_lower_near', index+1)
        center += shape('SW_new_lower_near')[1:]
        center += _movement_path(net_path, 'SW_new_lower_near', 'SW_new_lower_out', index)[1:]
        center += shape('SW_new_lower_out')[1:]
        width = float(net.find(f"edge[@id='SW_new_lower_near']/lane[@index='{index}']").get('width', '3.2'))
        boundary = _offset(center, width/2)
        line = LineString(boundary)
        upstream, downstream = [], []
        for p in root:
            name = p.get('id', '')
            if name.startswith(f'SW_horizontal_straight_guide_upper_{index+1}_'):
                upstream.extend(line.project(Point(q)) for q in _read(p.get('shape')))
            elif name.startswith(f'SW_new_straight_dash_lower_{7-index}_'):
                downstream.extend(line.project(Point(q)) for q in _read(p.get('shape')))
        if not upstream or not downstream:
            continue  # The upper through guides have not been added yet.
        start, end = max(upstream), min(downstream)-gap
        count = 0
        while end-dash > start+.5:
            piece = _slice(boundary, end-dash, end)
            ET.SubElement(root, 'poly', id=f'{prefix}{index}_{count}',
                          color='255,255,255', fill='0', lineWidth='.15',
                          layer='6', shape=_write(piece))
            pieces.append(piece)
            count += 1
            end -= dash+gap
    return pieces
