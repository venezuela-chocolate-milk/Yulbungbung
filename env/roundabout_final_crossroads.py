"""Road markings for the final four-arm junction."""
import math
import xml.etree.ElementTree as ET

from shapely.geometry import LineString
from env.roundabout_geometry import _read,_write,_offset,_slice
from env.roundabout_end_widening import layout


def add_markings(root,net_path,shapes):
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_builder import _center_line,_stop_line,_arrow,_movement_path
    from env.roundabout_markings import dash_style
    net=ET.parse(net_path).getroot()
    point,_,_,end=layout()
    corner=end+51
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    normal=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    def local(p):
        return [sum((p[j]-origin[j])*axis[j] for j in (0,1)) for axis in (along,normal)]
    incoming=[('SE_triangle_main_out',('straight','straight','left'),'NW'),
              ('SE_triangle_forward_in',('straight_right','straight_left'),'SE'),
              ('SE_triangle_NE_in',('straight_right','straight','left'),'NE'),
              ('SE_triangle_cross_in',('right','straight_left','left'),'SW')]
    marks=[]
    for edge,kinds,label in incoming:
        _stop_line(marks,f'se_final_stop_{label}',shapes,edge,dist=7)
        for lane,kind in enumerate(kinds):
            _arrow(marks,f'se_final_arrow_{label}_{lane}',shapes,edge,lane,kind,14)
    nw_limit=local(shapes[('SE_triangle_main_out',0)][-1])[0]-7
    se_limit=local(shapes[('SE_triangle_forward_in',0)][-1])[0]+7
    ne_limit=local(shapes[('SE_triangle_NE_in',0)][-1])[1]+7
    sw_limit=local(shapes[('SE_triangle_cross_in',0)][-1])[1]-7
    # Cover the narrow terrain wedge between the upper approach trajectories.
    ne_pavement=[point(corner-9.6,0),point(corner,0),
                 point(corner,ne_limit),point(corner-9.6,ne_limit)]
    ET.SubElement(root,'poly',id='se_final_NE_approach_pavement',color='84,84,90',
                  fill='1',layer='4.9',shape=_write(ne_pavement))
    for label,pts in [
            ('NW',[point(end,0),point(nw_limit,0)]),
            ('SE',[point(se_limit,0),point(corner+120,0)]),
            ('NE',[point(corner,ne_limit),point(corner,80)]),
            ('SW',[point(corner,-END_CROSSROADS['sw_arm_length']),point(corner,sw_limit)])]:
        _center_line(marks,f'se_final_center_{label}',pts)
    for mark in marks:
        root.append(ET.fromstring(mark))
    # Extend yellow medians over the unpaved gaps up to the turning lanes,
    # following their actual edges rather than drawing across a traffic path.
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from env.roundabout_surfaces import _network,_polygons
    _,_,strips=_network(net_path)
    road=unary_union(list(strips.values()))
    for label,start,finish in [('NW',nw_limit-0.1,corner),('SE',corner,se_limit+0.1)]:
        median=Polygon([point(start,-0.5),point(finish,-0.5),
                        point(finish,0.5),point(start,0.5)])
        pieces=_polygons(median.difference(road.buffer(0.03)))
        if not pieces:
            continue
        median=max(pieces,key=lambda part:part.area)
        ET.SubElement(root,'poly',id=f'se_final_yellow_median_gap_{label}',
                      color='235,190,40',fill='1',layer='6',
                      shape=_write(list(median.exterior.coords)))
    median=Polygon([point(corner-0.5,sw_limit-0.1),point(corner+0.5,sw_limit-0.1),
                    point(corner+0.5,0),point(corner-0.5,0)])
    pieces=_polygons(median.difference(road.buffer(0.03)))
    if pieces:
        median=max(pieces,key=lambda part:part.area)
        ET.SubElement(root,'poly',id='se_final_yellow_median_gap_SW',
                      color='235,190,40',fill='1',layer='6',
                      shape=_write(list(median.exterior.coords)))
    # Align the new main-road separators across all three lanes.
    for p in list(root):
        name=p.get('id','')
        if name.startswith('lane_dash_SE_triangle_forward_out_'):
            root.remove(p)
    dash,gap,_=dash_style(20)
    boundary=local(shapes[('SE_triangle_forward_out',0)][0])[0]
    for lane,lateral in enumerate((-3.7,)):
        for i in range(max(0,math.floor((corner+120-boundary-dash)/(dash+gap))+1)):
            d=boundary+i*(dash+gap)
            ET.SubElement(root,'poly',id=f'se_final_forward_dash_{lane}_{i}',color='255,255,255',
                          fill='0',lineWidth='0.15',layer='6',
                          shape=_write([point(d,lateral),point(d+dash,lateral)]))
    # The two straight guides specified by the user connect the approach
    # dividers to the inner boundary and divider of the two-lane exit.
    for guide,source_lateral,target_lateral in [(0,-3.7,-0.5),(1,-6.9,-3.7)]:
        path=[point(nw_limit,source_lateral),point(boundary,target_lateral)]
        length=LineString(path).length
        count=max(0,math.floor((length-gap-dash)/(dash+gap))+1)
        for i in range(count):
            start=length-gap-dash-i*(dash+gap)
            finish=start+dash
            start=max(0.3,start)
            if finish-start<0.1:
                continue
            ET.SubElement(root,'poly',id=f'se_final_straight_guide_{guide}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,finish)))
    # Guides from the upper three-lane approach to the lower two-lane exit.
    ne_boundary=local(shapes[('SE_triangle_NE_in',0)][-1])[1]
    approach_dashes=[p for p in root if p.get('id','').startswith('lane_dash_SE_triangle_NE_in_0_')]
    phase=gap
    if approach_dashes:
        last_end=min(min(local(q)[1] for q in _read(p.get('shape'))) for p in approach_dashes)
        phase=(gap-(last_end-ne_boundary))%(dash+gap)
    for guide,source_d,target_d in [(0,corner-6.4,corner-3.7),
                                    (1,corner-3.2,corner-0.5)]:
        path=[point(source_d,ne_boundary),point(target_d,sw_limit)]
        length=LineString(path).length
        for i in range(max(0,math.floor((length-phase-dash)/(dash+gap))+1)):
            start=phase+i*(dash+gap)
            ET.SubElement(root,'poly',id=f'se_final_cross_guide_{guide}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,start+dash)))
    # The marked line joins the lower approach's inner divider to the upper
    # median endpoint; it is offset from the intersection's center axis.
    opposite_path=[point(corner+3.7,sw_limit+0.3),point(corner,ne_limit-0.3)]
    opposite_length=LineString(opposite_path).length
    for i in range(max(0,math.floor((opposite_length-gap-dash)/(dash+gap))+1)):
        start=gap+i*(dash+gap)
        ET.SubElement(root,'poly',id=f'se_final_opposite_cross_guide_{i}',
                      color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                      shape=_write(_slice(opposite_path,start,start+dash)))
    approaches={edge for edge,_,_ in incoming}
    first=next(p for p in root if p.get('id')=='lane_dash_SE_triangle_main_in_0_0')
    a,b=_read(first.get('shape'))
    tangent=tuple((b[j]-a[j])/math.dist(a,b) for j in (0,1))
    start=tuple(a[j]-(dash+gap)*tangent[j] for j in (0,1))
    finish=tuple(start[j]+dash*tangent[j] for j in (0,1))
    ET.SubElement(root,'poly',id='se_final_NW_missing_dash',color='255,255,255',
                  fill='0',lineWidth='0.15',layer='6',shape=_write([start,finish]))
    for turn in net.findall('connection'):
        if turn.get('from') not in approaches or turn.get('dir')!='l':
            continue
        path=_offset(_movement_path(net_path,turn.get('from'),turn.get('to'),int(turn.get('fromLane'))),1.6)
        length=LineString(path).length
        for i in range(max(0,math.floor((length-dash)/(dash+gap))+1)):
            d=i*(dash+gap)
            ET.SubElement(root,'poly',id=f'se_final_turn_guide_{turn.get("from")}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,d,d+dash)))
    # Remove the three visible strokes marked by the user. The first stroke
    # has an overlapping automatic divider and manually drawn guide.
    removed_marks={'se_final_cross_guide_0_7','se_final_cross_guide_1_7',
                   'se_final_turn_guide_SE_triangle_cross_in_0'}
    for p in list(root):
        if p.get('id') in removed_marks:
            root.remove(p)
