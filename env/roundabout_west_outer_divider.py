"""Join the west receiving road's fourth/fifth-lane separator at the island."""
import xml.etree.ElementTree as ET
from shapely.geometry import LineString, Point, Polygon
from env.roundabout_geometry import _read, _write, _offset, _slice
from env.roundabout_builder import _movement_path
from env.roundabout_markings import dash_style

PREFIX = 'SW_west_outer_divider_extension_'


def add_divider(root, net_path):
    net = ET.parse(net_path).getroot()
    lane = net.find("edge[@id='SW_new_cross_in']/lane[@index='0']")
    center = _movement_path(net_path, 'SW_new_upper_in', 'SW_new_cross_in', 0)
    center += _read(lane.get('shape'))[1:]
    boundary = _offset(center, float(lane.get('width', '3.2'))/2)
    line = LineString(boundary)
    island = Polygon(_read(root.find("poly[@id='SW_road_first_island_border_0']").get('shape')))
    tip = min(island.exterior.coords, key=lambda p: p[0])
    start = line.project(Point(tip))
    anchors = []
    for p in list(root):
        name = p.get('id', '')
        if name.startswith(PREFIX):
            root.remove(p)
        elif (p.get('color') == '255,255,255' and p.get('fill') == '0'
              and float(p.get('lineWidth', '1')) <= .2):
            points = _read(p.get('shape'))
            if all(line.distance(Point(q)) < .05 for q in points):
                station = min(line.project(Point(q)) for q in points)
                if station > start:
                    anchors.append(station)
    if not anchors:
        raise ValueError('Missing existing west outside divider')
    dash, gap, _ = dash_style(20)
    end = min(anchors)-gap
    pieces = []
    while end > start:
        begin = max(start, end-dash)
        if end-begin > .5:
            piece = _slice(boundary, begin, end)
            if LineString(piece).buffer(.075, cap_style=2).intersects(island):
                raise ValueError('West outside divider overlaps safety island')
            ET.SubElement(root, 'poly', id=f'{PREFIX}{len(pieces)}', color='255,255,255',
                          fill='0', lineWidth='.15', layer='6', shape=_write(piece))
            pieces.append(piece)
        end -= dash+gap
    return pieces
