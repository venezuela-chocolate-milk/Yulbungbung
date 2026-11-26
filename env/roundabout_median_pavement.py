"""Pave the exposed median remnant beyond the west approach stop line."""
import xml.etree.ElementTree as ET
from shapely import make_valid
from shapely.geometry import Polygon
from shapely.ops import unary_union
from env.roundabout_geometry import _read, _write, _unit

PREFIX = 'SW_west_stop_median_pavement_'


def fill_west_stop_gap(root, net_path):
    for p in list(root):
        if p.get('id', '').startswith(PREFIX):
            root.remove(p)
    net = ET.parse(net_path).getroot()
    median = _read(root.find("poly[@id='SW_end_T_extension_center']").get('shape'))
    tip = median[-1]
    forward = _unit(median[-2], tip)
    normal = (-forward[1], forward[0])
    # The 1 m median is exposed only beyond the stop, where its yellow line ends.
    strip = Polygon([(tip[0]+d*forward[0]+s*normal[0],
                      tip[1]+d*forward[1]+s*normal[1])
                     for d, s in [(0, -.53), (25, -.53), (25, .53), (0, .53)]])
    road = [make_valid(Polygon(_read(net.find("junction[@id='SW_new_cross']").get('shape'))))]
    road += [make_valid(Polygon(_read(p.get('shape')))) for p in root
             if p.get('fill') == '1' and p.get('color') == '84,84,90']
    missing = strip.difference(unary_union(road))
    polygons = [missing] if missing.geom_type == 'Polygon' else list(missing.geoms)
    for i, polygon in enumerate(polygons):
        if polygon.geom_type != 'Polygon' or polygon.area < .001:
            continue
        ET.SubElement(root, 'poly', id=f'{PREFIX}{i}', color='84,84,90', fill='1',
                      layer='4.8', shape=_write(polygon.exterior.coords))
    return missing.area
