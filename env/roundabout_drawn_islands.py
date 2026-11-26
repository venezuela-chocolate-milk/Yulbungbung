"""Three safety islands traced from the user's road-edge sketch in metres."""
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon,LineString
from shapely.ops import unary_union
from env.roundabout_config import END_CROSSROADS as C
from env.roundabout_end_widening import layout
from env.roundabout_geometry import _read,_write
from env.roundabout_surfaces import _network,_polygons,_lines
OUTLINES = [[(83.346, 13.9095), (87.2119, 15.9385), (90.9064, 18.3526), (94.2438, 21.2232), (97.824, 23.8941), (94.477, 13.7884)], [(131.1256, 32.7208), (132.2988, 28.0123), (134.1994, 24.0891), (136.785, 20.0095), (139.3557, 17.0001), (141.9685, 14.9326), (144.3949, 14.3205), (147.3349, 13.9371), (125.8722, 13.7661)], [(121.866, -13.8642), (134.0673, -13.9704), (129.5022, -16.1568), (126.892, -18.242), (124.1113, -21.3262), (121.787, -24.0534), (119.4627, -26.7806)]]

def add_markings(root,net_path):
    for p in list(root):
        if p.get('id','').startswith('SW_drawn_island_'):root.remove(p)
    point,_,_,end=layout()
    start=end+51+C['sw_end_t_left_length']; s=-C['sw_arm_length']-C['sw_forward_length']
    _,_,strips=_network(net_path)
    # Preserve the full 3.2 m approach/exit lanes at every corner.
    road=unary_union([v for k,v in strips.items() if not k.startswith(':')])
    for i,outline in enumerate(OUTLINES):
        candidate=Polygon([point(start+d,s+l) for d,l in outline])
        pieces=list(_polygons(candidate.difference(road.buffer(.15))))
        if not pieces:continue
        island=max(pieces,key=lambda p:p.area)
        for p in list(root):
            if p.get('color')!='255,255,255' or p.get('fill')!='0':continue
            coords=_read(p.get('shape',''))
            if len(coords)<2:continue
            line=LineString(coords)
            if not line.intersects(island):continue
            root.remove(p)
            for j,piece in enumerate(_lines(line.difference(island.buffer(.05)))):
                if piece.length<.05:continue
                attrs=dict(p.attrib);attrs.update(id=f"cropped_drawn_{p.get('id')}_{j}",shape=_write(list(piece.coords)))
                ET.SubElement(root,'poly',attrs)
        ET.SubElement(root,'poly',id=f'SW_drawn_island_fill_{i}',color='84,84,90',fill='1',layer='5',shape=_write(list(island.exterior.coords)))
        ET.SubElement(root,'poly',id=f'SW_drawn_island_border_{i}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(island.exterior.coords)))
        for j,d in enumerate(range(40,170,4)):
            stripe=LineString([point(start+d,s-100),point(start+d+70,s+100)])
            for k,piece in enumerate(_lines(stripe.intersection(island.buffer(-.25)))):
                if piece.length>.3:
                    ET.SubElement(root,'poly',id=f'SW_drawn_island_hatch_{i}_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))
