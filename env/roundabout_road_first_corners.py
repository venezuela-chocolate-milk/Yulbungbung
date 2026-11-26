"""Construct three corner roads at lane width before painting their islands."""
import math
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon,LineString,Point
from shapely.ops import unary_union
from shapely import set_precision
from env.roundabout_config import END_CROSSROADS as C
from env.roundabout_end_widening import layout
from env.roundabout_geometry import _read,_write,_offset,_slice
from env.roundabout_surfaces import _network,_polygons,_lines
from env.roundabout_builder import _movement_path


def clear_four_selected_dash_areas(root):
    """Clear only the four blue-selected groups, preserving stops and arrows."""
    selected={
        'lane_dash_SW_end_T_extension_in_2_16',
        'lane_dash_SW_end_T_extension_in_2_17',
        'lane_dash_SW_end_T_extension_in_3_16',
        'lane_dash_SW_end_T_extension_in_3_17',
        'cropped_core_lane_dash_SW_end_T_extension_in_2_15_0',
        'cropped_core_lane_dash_SW_end_T_extension_in_3_15_0',
        'lane_dash_SW_end_T_extension_in_4_15',
        'lane_dash_SW_end_T_extension_in_4_16',
        'lane_dash_SW_end_T_extension_in_4_17',
        'lane_dash_SW_end_T_extension_out_3_5',
        'lane_dash_SW_end_T_extension_out_3_6',
        'lane_dash_SW_end_T_extension_out_3_7',
        'lane_dash_SW_end_T_extension_out_3_8',
        'lane_dash_SW_end_T_left_out_2_112',
        'lane_dash_SW_end_T_left_out_2_113',
        'lane_dash_SW_end_T_left_out_2_114',
        'lane_dash_SW_end_T_right_in_0_132',
        'lane_dash_SW_end_T_right_in_0_133',
        'lane_dash_SW_end_T_right_in_0_134',
        'lane_dash_SW_end_T_right_in_1_132',
        'lane_dash_SW_end_T_right_in_1_133',
        'lane_dash_SW_end_T_right_in_1_134',
        'lane_dash_SW_end_T_right_in_1_135',
        'cropped_core_lane_dash_SW_end_T_left_out_2_115_0',
        'cropped_core_lane_dash_SW_end_T_right_in_0_135_0',
        'SW_new_straight_dash_upper_0_6',
        'SW_new_straight_dash_upper_0_7',
        'SW_new_straight_dash_upper_0_8',
        'SW_new_straight_dash_upper_0_9',
        'SW_new_straight_dash_upper_1_6',
        'SW_new_straight_dash_upper_1_7',
        'SW_new_straight_dash_upper_1_8',
        'SW_new_straight_dash_upper_1_9',
        'SW_new_straight_dash_upper_2_6',
        'SW_new_straight_dash_upper_2_7',
        'SW_new_straight_dash_upper_2_8',
        'SW_new_straight_dash_upper_2_9',
        'SW_new_straight_dash_upper_3_6',
        'SW_new_straight_dash_upper_3_7',
        'SW_new_straight_dash_upper_3_8',
        'SW_new_straight_dash_upper_3_9',
        'lane_dash_SW_end_T_extension_out_3_14',
        'lane_dash_SW_end_T_extension_out_3_15',
        'lane_dash_SW_end_T_extension_out_3_16',
        'lane_dash_SW_end_T_extension_out_3_17',
        'lane_dash_SW_end_T_left_out_2_121',
        'lane_dash_SW_end_T_left_out_2_122',
        'lane_dash_SW_end_T_left_out_2_123',
        'lane_dash_SW_end_T_left_out_2_124',
        'lane_dash_SW_end_T_right_in_1_142',
        'lane_dash_SW_end_T_right_in_1_143',
        'lane_dash_SW_end_T_right_in_1_144',
        'cropped_core_lane_dash_SW_end_T_right_in_1_141_0',
        'cropped_lower_expanded_lane_dash_SW_end_T_right_in_0_144_0',
        'SW_main_outer_divider_link_0',
        'SW_main_outer_divider_link_1',
        'SW_main_outer_divider_link_2',
    }
    for poly in list(root):
        if poly.get('id') in selected:root.remove(poly)


def fit_upper4_island_boundary(root,net_path):
    """Replace the complete traffic-facing side by the exact fourth-lane edge."""
    fill=root.find("poly[@id='SW_road_first_island_fill_1']")
    border=root.find("poly[@id='SW_road_first_island_border_1']")
    coords=_read(fill.get('shape'))[:-1]
    top=max(range(len(coords)),key=lambda i:coords[i][1])
    left=min(range(len(coords)),key=lambda i:coords[i][0])
    def chain(step):
        result=[coords[top]];i=top
        while i!=left:
            i=(i+step)%len(coords);result.append(coords[i])
        return result
    # Retain the opposite curved perimeter and bottom edge, replacing only the
    # full side facing the observed fifth-to-fourth straight movement.
    opposite=max([chain(1),chain(-1)],key=lambda q:max(p[0] for p in q))
    lane=ET.parse(net_path).getroot().find("edge[@id='SW_new_upper_out']/lane[@index='1']")
    half_width=float(lane.get('width','3.2'))/2
    points=_offset(_movement_path(net_path,'SW_new_lower_in','SW_new_upper_out',1),-half_width)
    boundary=LineString(points)
    start=boundary.project(Point(coords[left]));finish=boundary.length
    side=_slice(points,start,finish)
    island=Polygon(side+opposite[1:-1])
    if not island.is_valid:raise ValueError('Fourth-lane safety boundary is not a simple polygon')
    shape=_write(list(island.exterior.coords));fill.set('shape',shape);border.set('shape',shape)
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_road_first_hatch_1_'):root.remove(p)
        elif p.get('color')=='255,255,255' and p.get('fill')=='0' and float(p.get('lineWidth','1'))<=.3:
            coords=_read(p.get('shape',''))
            if len(coords)<2:continue
            line=LineString(coords)
            if not line.intersects(island.buffer(.3)):continue
            root.remove(p)
            for j,piece in enumerate(_lines(line.difference(island.buffer(.3)))):
                if piece.length<1:continue
                attrs=dict(p.attrib);attrs.update(id=f'cropped_upper4_exact_{name}_{j}',shape=_write(list(piece.coords)));ET.SubElement(root,'poly',attrs)
    point,_,_,end=layout();start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    for j,d in enumerate(range(20,190,4)):
        stroke=LineString([point(start+d,s-100),point(start+d+70,s+100)])
        for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.25)))):
            if piece.length>.3:ET.SubElement(root,'poly',id=f'SW_road_first_hatch_1_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))


def connect_lower5_to_upper4(root):
    """Connect user lanes 2/3/4/5 to opposite lanes 1/2/3/4 respectively."""
    from env.roundabout_geometry import _curve,_unit,_length
    edges={e.get('id'):e for e in root.findall('edge')}
    lanes={l.get('id'):l for e in edges.values() for l in e.findall('lane')}
    for source_index in (1,2,3,4):
        connection=next(c for c in root.findall('connection') if c.get('from')=='SW_new_lower_in'
                        and c.get('to')=='SW_new_upper_out' and c.get('fromLane')==str(source_index))
        a=_read(lanes[f'SW_new_lower_in_{source_index}'].get('shape'))
        b=_read(lanes[f'SW_new_upper_out_{source_index}'].get('shape'))
        path=_curve(a[-1],b[0],_unit(a[-2],a[-1]),_unit(b[0],b[1]))
        follow={(c.get('from'),c.get('fromLane')):c for c in root.findall('connection') if c.get('from').startswith(':')}
        chain=[];via=connection.get('via')
        while via:
            chain.append(via);edge,index=via.rsplit('_',1);continuation=follow[(edge,index)];via=continuation.get('via')
        continuation.set('toLane',str(source_index));connection.set('toLane',str(source_index))
        sizes=[float(lanes[k].get('length')) for k in chain];total=sum(sizes);length=_length(path);start=0
        for i,(key,size) in enumerate(zip(chain,sizes)):
            finish=length if i==len(chain)-1 else start+length*size/total
            piece=_slice(path,start,finish);lanes[key].set('shape',_write(piece));lanes[key].set('length',str(_length(piece)));start=finish

def add_lower5_straight_guide(root,net_path):
    """Guide the left boundary of the fifth incoming lane across the junction."""
    from env.roundabout_markings import dash_style
    net=ET.parse(net_path).getroot()
    lane=net.find("edge[@id='SW_new_lower_in']/lane[@index='1']")
    half_width=float(lane.get('width','3.2'))/2
    approach=_offset(_read(lane.get('shape')),half_width)
    movement=_offset(_movement_path(net_path,'SW_new_lower_in','SW_new_upper_out',1),half_width)
    stop=root.find("poly[@id='SW_lower_approach_stop']")
    stop_line=LineString(_read(stop.get('shape')))
    path=LineString(approach)
    crossing=path.intersection(stop_line)
    start=path.project(crossing) if crossing.geom_type=='Point' else path.project(Point(_read(stop.get('shape'))[-1]))
    points=_slice(approach,start,path.length)+movement[1:]
    length=LineString(points).length
    for p in list(root):
        if p.get('id','').startswith('SW_lower5_straight_guide_'):root.remove(p)
    dash,gap,_=dash_style(20)
    position=1.0;index=0
    while position+dash<length-1:
        ET.SubElement(root,'poly',id=f'SW_lower5_straight_guide_{index}',color='255,255,255',
                      fill='0',lineWidth='.15',layer='6',shape=_write(_slice(points,position,position+dash)))
        position+=dash+gap;index+=1


def extend_upper_right_island_along_straight(root,net_path):
    """Extend toward the upper exit while reserving actual vehicle corridors."""
    fill=root.find("poly[@id='SW_road_first_island_fill_1']")
    border=root.find("poly[@id='SW_road_first_island_border_1']")
    original=Polygon(_read(fill.get('shape')))
    tip=max(original.exterior.coords,key=lambda q:q[1])
    net=ET.parse(net_path).getroot()
    lane=net.find("edge[@id='SW_new_upper_out']/lane[@index='0']")
    points=_read(lane.get('shape'))
    length=math.dist(points[0],points[-1]);direction=tuple((points[-1][j]-points[0][j])/length for j in (0,1))
    right=(direction[1],-direction[0])
    ahead=tuple(tip[j]+direction[j]*12 for j in (0,1))
    shoulder=tuple(ahead[j]+right[j]*1.6 for j in (0,1))
    left=min(original.exterior.coords,key=lambda q:q[0])
    extended_left=tuple(left[j]-right[j]*6.4 for j in (0,1))
    candidate=unary_union([original,Polygon([extended_left,ahead,shoulder,max(original.exterior.coords,key=lambda q:q[0])])]).convex_hull
    # Supply enough paint area along the entire observed through movement so
    # its real lane boundary, rather than a chord of the hull, defines the side.
    through_points=_movement_path(net_path,'SW_new_lower_in','SW_new_upper_out',1)
    through_boundary=_offset(through_points,-1.75)
    boundary=LineString(through_boundary)
    start=max(0,boundary.project(Point(left))-3)
    finish=min(boundary.length,boundary.project(Point(tip))+6)
    side=_slice(through_boundary,start,finish)
    paint_side=Polygon(side+[max(original.exterior.coords,key=lambda q:q[0])])
    candidate=unary_union([candidate,paint_side]).convex_hull
    reserves=[]
    for source,index,radius in [('SW_new_lower_in',1,1.75),('SW_end_T_extension_in',0,1.15),('SW_slip_main',4,1.15)]:
        reserves.append(LineString(_movement_path(net_path,source,'SW_new_upper_out',index)).buffer(radius,quad_segs=32))
    reserves.append(LineString(_movement_path(net_path,'SW_end_T_extension_in','SW_new_cross_in',1)).buffer(1.25,quad_segs=32))
    fitted=candidate.difference(unary_union(reserves))
    island=max(_polygons(fitted),key=lambda p:p.intersection(original).area)
    shape=_write(list(island.exterior.coords));fill.set('shape',shape);border.set('shape',shape)
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_road_first_hatch_1_'):root.remove(p)
        elif p.get('color')=='255,255,255' and p.get('fill')=='0' and float(p.get('lineWidth','1'))<=.3:
            coords=_read(p.get('shape',''))
            if len(coords)<2:continue
            line=LineString(coords)
            if not line.intersects(island.buffer(.3)):continue
            root.remove(p)
            for j,piece in enumerate(_lines(line.difference(island.buffer(.3)))):
                if piece.length<1:continue
                attrs=dict(p.attrib);attrs.update(id=f'cropped_upper_right_{name}_{j}',shape=_write(list(piece.coords)));ET.SubElement(root,'poly',attrs)
    point,_,_,end=layout();start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    for j,d in enumerate(range(20,190,4)):
        stroke=LineString([point(start+d,s-100),point(start+d+70,s+100)])
        for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.25)))):
            if piece.length>.3:ET.SubElement(root,'poly',id=f'SW_road_first_hatch_1_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))


def connect_main_outer_divider(root,net_path):
    """Continue the fourth/fifth-lane separator with the receiving road's phase."""
    from env.roundabout_markings import dash_style
    points=_offset(_movement_path(net_path,'SW_slip_main','SW_new_cross_out',0),1.6)
    boundary=LineString(points)
    anchors=[]
    for p in list(root):
        if p.get('id','').startswith('SW_main_outer_divider_link_'):
            root.remove(p);continue
        if p.get('color')!='255,255,255' or p.get('fill')!='0' or float(p.get('lineWidth','1'))>.3:continue
        coords=_read(p.get('shape',''))
        if len(coords)<2:continue
        if any(boundary.distance(Point(q))>.01 for q in (coords[0],coords[-1])):continue
        start=min(boundary.project(Point(q)) for q in (coords[0],coords[-1]))
        if start>boundary.length-10:anchors.append(start)
    anchor=min(anchors) if anchors else boundary.length-5.17
    dash,gap,_=dash_style(20)
    for i in range(3):
        finish=anchor-gap-i*(dash+gap)
        ET.SubElement(root,'poly',id=f'SW_main_outer_divider_link_{i}',color='255,255,255',
                      fill='0',lineWidth='.15',layer='6',shape=_write(_slice(points,finish-dash,finish)))


def align_lower_outer_dash(root,net_path):
    """Fit the dash to the inner boundary of the actual outer right-turn lane."""
    dash=root.find("poly[@id='SW_new_straight_dash_lower_4_10']")
    if dash is None:return
    line=LineString(_read(dash.get('shape')))
    center=line.interpolate(.5,normalized=True)
    net=ET.parse(net_path).getroot()
    lane=net.find("edge[@id='SW_new_lower_in']/lane[@index='0']")
    half_width=float(lane.get('width','3.2'))/2
    points=_offset(_movement_path(net_path,'SW_new_lower_in','SW_new_cross_out',0),half_width)
    boundary=LineString(points)
    station=boundary.project(center)
    start=max(0,min(station-line.length/2,boundary.length-line.length))
    dash.set('shape',_write(_slice(points,start,start+line.length)))
    previous=root.find("poly[@id='SW_new_straight_dash_lower_4_9']")
    if previous is not None:
        from env.roundabout_markings import dash_style
        _,gap,_=dash_style(20)
        length=LineString(_read(previous.get('shape'))).length
        previous_start=max(0,start-gap-length)
        previous.set('shape',_write(_slice(points,previous_start,previous_start+length)))


def extend_lower_island_along_turn(root,net_path):
    """Straight outer side and a curved side derived from the actual turn lane."""
    fill=root.find("poly[@id='SW_road_first_island_fill_2']")
    border=root.find("poly[@id='SW_road_first_island_border_2']")
    original=Polygon(_read(fill.get('shape')))
    top=max(original.exterior.coords,key=lambda q:q[1])
    corner=min(original.exterior.coords,key=lambda q:q[0])
    turn=_movement_path(net_path,'SW_new_lower_in','SW_new_cross_out',0)
    boundary_points=_offset(turn,1.25)
    boundary=LineString(boundary_points)
    finish=boundary.project(Point(corner))
    curve=_slice(boundary_points,max(0,finish-11),finish)
    cap=(top[0]-1.5,top[1]+.6)
    extension=Polygon([cap,top,corner]+list(reversed(curve)))
    through=LineString(_movement_path(net_path,'SW_new_lower_in','SW_new_upper_out',1))
    main_through=LineString(_movement_path(net_path,'SW_slip_main','SW_new_cross_out',0))
    fitted=unary_union([original,extension]).convex_hull.difference(unary_union([
        LineString(turn).buffer(1.25,quad_segs=32),through.buffer(1.15,quad_segs=32),
        main_through.buffer(1.75,quad_segs=32)]))
    island=max(_polygons(fitted),key=lambda q:q.intersection(original).area)
    shape=_write(list(island.exterior.coords));fill.set('shape',shape);border.set('shape',shape)
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_road_first_hatch_2_'):root.remove(p)
        elif p.get('color')=='255,255,255' and p.get('fill')=='0' and float(p.get('lineWidth','1'))<=.3:
            coords=_read(p.get('shape',''))
            if len(coords)<2:continue
            line=LineString(coords)
            if not line.intersects(island.buffer(.3)):continue
            root.remove(p)
            for j,piece in enumerate(_lines(line.difference(island.buffer(.3)))):
                if piece.length<1:continue
                attrs=dict(p.attrib);attrs.update(id=f'cropped_lower_turn_{name}_{j}',shape=_write(list(piece.coords)))
                ET.SubElement(root,'poly',attrs)
    point,_,_,end=layout();start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    for j,d in enumerate(range(20,190,4)):
        stroke=LineString([point(start+d,s-100),point(start+d+70,s+100)])
        for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.25)))):
            if piece.length>.3:
                ET.SubElement(root,'poly',id=f'SW_road_first_hatch_2_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))


def expand_lower_island_to_sketch(root):
    """Extend the lower-right safety paint along the user's blue outline."""
    fill=root.find("poly[@id='SW_road_first_island_fill_2']")
    border=root.find("poly[@id='SW_road_first_island_border_2']")
    original=Polygon(_read(fill.get('shape')))
    top=max(original.exterior.coords,key=lambda p:p[1])
    # Sketch coordinates relative to its original upper corner, in metres.
    offsets=[(0,0),(1.26,1.98),(3.15,1.08),(5.95,.45),(8.74,-.72),
             (11.71,-2.07),(15.05,-3.24),(18.56,-4.95),(15.05,-4.14),
             (11.62,-3.87),(8.98,-4.0)]
    extension=Polygon([(top[0]+x,top[1]+y) for x,y in offsets])
    island=unary_union([original,extension]).buffer(0)
    shape=_write(list(island.exterior.coords));fill.set('shape',shape);border.set('shape',shape)
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_road_first_hatch_2_'):root.remove(p)
        elif p.get('color')=='255,255,255' and p.get('fill')=='0' and float(p.get('lineWidth','1'))<=.3:
            coords=_read(p.get('shape',''))
            if len(coords)<2:continue
            line=LineString(coords)
            if not line.intersects(island.buffer(.3)):continue
            root.remove(p)
            for j,piece in enumerate(_lines(line.difference(island.buffer(.3)))):
                if piece.length<1.0:continue
                attrs=dict(p.attrib);attrs.update(id=f'cropped_lower_expanded_{name}_{j}',shape=_write(list(piece.coords)))
                ET.SubElement(root,'poly',attrs)
    point,_,_,end=layout();start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    for j,d in enumerate(range(20,190,4)):
        stroke=LineString([point(start+d,s-100),point(start+d+70,s+100)])
        for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.25)))):
            if piece.length>.3:
                ET.SubElement(root,'poly',id=f'SW_road_first_hatch_2_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))


def expand_upper_island_to_sketch(root):
    """Extend the lower edge of the upper splitter to the sketched outline."""
    fill=root.find("poly[@id='SW_road_first_island_fill_0']")
    border=root.find("poly[@id='SW_road_first_island_border_0']")
    points=_read(fill.get('shape'))
    left,bottom=points[0],points[-2]
    blue_left=(left[0]-6.52,left[1]-.47)
    blue_bottom=(bottom[0]-1.63,bottom[1]-2.56)
    island=unary_union([Polygon(points),Polygon([left,blue_left,blue_bottom,bottom])]).buffer(0)
    shape=_write(list(island.exterior.coords))
    fill.set('shape',shape);border.set('shape',shape)
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_road_first_hatch_0_'):
            root.remove(p)
        elif p.get('color')=='255,255,255' and p.get('fill')=='0' and float(p.get('lineWidth','1'))<=.3:
            coords=_read(p.get('shape',''))
            if len(coords)<2:continue
            line=LineString(coords)
            # Remove complete dashes running along the yellow border, including
            # their stroke widths; centre-line intersection alone misses these.
            if line.intersection(island.buffer(.18)).length>1.0:
                root.remove(p)
                continue
            if not line.intersects(island):continue
            root.remove(p)
            for j,piece in enumerate(_lines(line.difference(island.buffer(.05)))):
                if piece.length<.05:continue
                attrs=dict(p.attrib);attrs.update(id=f"cropped_upper_expanded_{name}_{j}",shape=_write(list(piece.coords)))
                ET.SubElement(root,'poly',attrs)
    point,_,_,end=layout();start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    for j,d in enumerate(range(20,190,4)):
        stroke=LineString([point(start+d,s-100),point(start+d+70,s+100)])
        for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.25)))):
            if piece.length>.3:
                ET.SubElement(root,'poly',id=f'SW_road_first_hatch_0_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))


def finish(root,net_path):
    net=ET.parse(net_path).getroot();node=net.find("junction[@id='SW_new_cross']")
    center=(float(node.get('x')),float(node.get('y')))
    point,_,_,end=layout();origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1));across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    cd=sum((center[j]-origin[j])*along[j] for j in (0,1));cl=sum((center[j]-origin[j])*across[j] for j in (0,1))
    def quadrant(shape):
        q=shape.centroid;v=(q.x-center[0],q.y-center[1])
        return (1 if sum(v[j]*along[j] for j in (0,1))>0 else -1,1 if sum(v[j]*across[j] for j in (0,1))>0 else -1)
    _,_,strips=_network(net_path)
    external=unary_union([v for k,v in strips.items() if not k.startswith(':')])
    start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    main=Polygon([point(start+30,s-16.5),point(start+185,s-16.5),point(start+185,s+16.5),point(start+30,s+16.5)])
    transverse=unary_union([Polygon(_read(p.get('shape'))) for p in root if p.get('id','').startswith('SW_new_straight_pavement_')])
    straight=unary_union([main,transverse])
    originals={}
    surfaces=list(strips.values())+[Polygon(_read(node.get('shape'))),straight]
    for p in root:
        if p.get('fill')=='1' and p.get('id','').startswith(('SW_new_corner_pavement_','SW_drawn_island_fill_')):
            q=Polygon(_read(p.get('shape'))).buffer(0);surfaces.append(q)
            if p.get('id','').startswith('SW_drawn_island_fill_'):originals[quadrant(q)]=q
    pavement=unary_union(surfaces).intersection(Point(center).buffer(65))
    movements=[((-1,1),'SW_new_upper_in','SW_new_cross_in'),((1,1),'SW_end_T_extension_in','SW_new_upper_out'),((1,-1),'SW_new_lower_in','SW_new_cross_out')]
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_road_first_'):root.remove(p);continue
        if name.startswith(('SW_drawn_island_fill_','SW_drawn_island_border_','SW_drawn_island_hatch_')):
            index=name.split('_')[4] if name.startswith('SW_drawn_island_hatch_') else name.rsplit('_',1)[1]
            fill=root.find(f"poly[@id='SW_drawn_island_fill_{index}']")
            # Fill nodes may already have been removed: resolve from their saved quadrant.
            if name.startswith('SW_drawn_island_fill_'):
                q=quadrant(Polygon(_read(p.get('shape'))));p.set('_corner_remove','1' if q!=(-1,-1) else '0')
    remove_indices={p.get('id').rsplit('_',1)[1] for p in root if p.get('_corner_remove')=='1'}
    for p in list(root):
        name=p.get('id','');p.attrib.pop('_corner_remove',None)
        if any(name in (f'SW_drawn_island_fill_{i}',f'SW_drawn_island_border_{i}') or name.startswith(f'SW_drawn_island_hatch_{i}_') for i in remove_indices):root.remove(p)
    all_clear=[];results=[]
    for i,(quad,source,target) in enumerate(movements):
        path=LineString(_movement_path(net_path,source,target,0))
        lane=path.buffer(1.6,quad_segs=32)
        sx,sy=quad
        sector=Polygon([point(cd,cl),point(cd+sx*100,cl),point(cd+sx*100,cl+sy*100),point(cd,cl+sy*100)])
        corner_straight=straight
        through_lane=Polygon()
        if source=='SW_new_upper_in':
            # The six incoming lanes include lane 0's right turn. Continue the
            # splitter toward the first divider, rather than leaving a seventh
            # partial-width pocket beside the six-lane straight pavement.
            edge=net.find("edge[@id='SW_new_upper_in']")
            width=float(edge.find('lane').get('width','3.2'))
            count=len(edge.findall('lane'))
            endpoint=net.find("junction[@id='SW_new_upper']")
            end_xy=(float(endpoint.get('x')),float(endpoint.get('y')))
            direction=tuple((end_xy[j]-center[j])/math.dist(center,end_xy) for j in (0,1))
            right=(direction[1],-direction[0])
            def side(q,offset):
                return tuple(q[j]+right[j]*offset for j in (0,1))
            outer_strip=Polygon([side(center,-count*width),side(end_xy,-count*width),
                                 side(end_xy,-(count-1)*width),side(center,-(count-1)*width)])
            corner_straight=unary_union([main,transverse.difference(outer_strip)])
            through_path=LineString(_movement_path(net_path,source,'SW_new_lower_near',1))
            through_lane=through_path.buffer(width/2,quad_segs=32)
        pocket=pavement.intersection(sector).difference(corner_straight)
        available=pocket.difference(unary_union([external.buffer(.15),path.buffer(1.75,quad_segs=32),
                                                through_lane.buffer(.15) if not through_lane.is_empty else through_lane]))
        if source=='SW_new_upper_in':
            # Let the right-turn and through-lane boundaries meet naturally at
            # the upstream tip instead of rounding off the splitter early.
            # Snap the shared strip edge so numerical slivers cannot split the
            # original island from its extension toward the first divider.
            available=set_precision(available,.0001)
        pieces=[q for q in _polygons(available) if q.area>.5]
        if not pieces:raise ValueError(f'No space for road-first island {quad}')
        original=originals[quad]
        island=max(pieces,key=lambda p:(p.intersection(original).area,-p.distance(Point(center))))
        island=set_precision(island.simplify(.005,preserve_topology=True),.0001)
        grass_pocket=pavement.intersection(sector).difference(straight) if source=='SW_new_upper_in' else pocket
        outside=grass_pocket.difference(unary_union([lane.buffer(.03),through_lane,external,island]))
        for j,q in enumerate(_polygons(outside)):
            if q.area<.1:continue
            ET.SubElement(root,'poly',id=f'SW_road_first_grass_{i}_{j}',color='112,140,96',fill='1',layer='5.1',shape=_write(list(q.exterior.coords)))
        for j,q in enumerate(_polygons(lane)):
            ET.SubElement(root,'poly',id=f'SW_road_first_lane_{i}_{j}',color='84,84,90',fill='1',layer='5.2',shape=_write(list(q.exterior.coords)))
        ET.SubElement(root,'poly',id=f'SW_road_first_island_fill_{i}',color='84,84,90',fill='1',layer='5.3',shape=_write(list(island.exterior.coords)))
        ET.SubElement(root,'poly',id=f'SW_road_first_island_border_{i}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(island.exterior.coords)))
        for j,d in enumerate(range(20,190,4)):
            stroke=LineString([point(start+d,s-100),point(start+d+70,s+100)])
            for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.25)))):
                if piece.length>.3:ET.SubElement(root,'poly',id=f'SW_road_first_hatch_{i}_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))
        all_clear.extend([island.buffer(.18),outside]);results.append((quad,island.area))
    clear=unary_union(all_clear)
    for p in list(root):
        if p.get('color')!='255,255,255' or p.get('fill')!='0' or float(p.get('lineWidth','1'))>.3:continue
        stroke=LineString(_read(p.get('shape')))
        if not stroke.intersects(clear):continue
        root.remove(p)
        for j,piece in enumerate(_lines(stroke.difference(clear))):
            if piece.length<.5:continue
            attrs=dict(p.attrib);attrs.update(id=f"cropped_road_first_{p.get('id')}_{j}",shape=_write(list(piece.coords)));ET.SubElement(root,'poly',attrs)
    return results
