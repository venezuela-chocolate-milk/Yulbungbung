"""Basic T junction at the end of the 150 m southwest extension."""
import xml.etree.ElementTree as ET
from env.roundabout_end_widening import layout
from env.roundabout_geometry import _write


def add_network(nodes,edges,connections):
    from env.roundabout_config import END_CROSSROADS
    arm_length=END_CROSSROADS['sw_end_t_arm_length']
    left_length=END_CROSSROADS.get('sw_end_t_left_length',arm_length)
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']
    node=next(n for n in nodes if n.get('id')=='SW_final_forward')
    node.set('type','priority');node.set('radius','22')
    for label,offset in [('left',left_length),('right',-arm_length)]:
        xy=point(d+offset,s)
        name=f'SW_end_T_{label}'
        ET.SubElement(nodes,'node',id=name,x=str(xy[0]),y=str(xy[1]),type='dead_end')
        sign=1 if offset>0 else -1
        for suffix,reverse in [('out',False),('in',True)]:
            lanes=5 if label=='right' else (5 if reverse else 4)
            lateral=s+(lanes*1.6+0.5)*sign*(1 if reverse else -1)
            pts=[point(d,lateral),point(d+offset,lateral)]
            source,target='SW_final_forward',name
            if reverse:
                source,target=target,source
                pts.reverse()
            ET.SubElement(edges,'edge',id=f'{name}_{suffix}',**{'from':source,'to':target,
                          'numLanes':str(lanes),'width':'3.2','speed':'8.33','priority':'2',
                          'spreadType':'center','shape':_write(pts)})
    # Continue the long arm beyond its former endpoint, opening an inner lane.
    start=d+left_length
    import math
    # Translate the transverse road 10 m south (screen down). Intersect its
    # translated axis with the unchanged main-road centerline.
    base=point(0,0)
    along=tuple(point(1,0)[j]-base[j] for j in (0,1))
    across=tuple(point(0,1)[j]-base[j] for j in (0,1))
    cross_shift=-10*along[1]+10*across[1]*math.tan(math.radians(15))-12.8/math.cos(math.radians(15))
    for name,station in [('SW_end_T_widen',start+40),('SW_end_T_extension',start+180),('SW_new_cross',start+114+cross_shift),('SW_slip_start',start+60)]:
        xy=point(station,s)
        ET.SubElement(nodes,'node',id=name,x=str(xy[0]),y=str(xy[1]),
                      type='dead_end' if name.endswith('extension') else 'priority',radius='18' if name=='SW_new_cross' else ('4' if name=='SW_slip_start' else '0'))
    old_node=next(n for n in nodes if n.get('id')=='SW_end_T_left')
    old_node.set('type','priority');old_node.set('radius','0')
    curve=[point(start+40*i/40,s-6.9-3.2*(3*(i/40)**2-2*(i/40)**3)) for i in range(41)]
    for edge_id,source,target,lanes,shape in [
        ('SW_end_T_taper_out','SW_end_T_left','SW_end_T_widen',4,curve),
        ('SW_end_T_extension_out','SW_end_T_widen','SW_slip_start',5,[point(start+40,s-8.5),point(start+60,s-8.5)]),
        ('SW_slip_main','SW_slip_start','SW_new_cross',5,[point(start+60,s-8.5),point(start+114+cross_shift,s-8.5)]),
        ('SW_new_cross_out','SW_new_cross','SW_end_T_extension',5,[point(start+114+cross_shift,s-8.5),point(start+180,s-8.5)]),
        # Widen outwards: lanes 1..5 retain the old five lane centre lines.
        # Lane 0 is the additional right-turn lane; the median stays fixed.
        ('SW_end_T_extension_in','SW_end_T_extension','SW_new_cross',6,[point(start+180,s+10.1),point(start+114+cross_shift,s+10.1)]),
        ('SW_new_cross_in','SW_new_cross','SW_end_T_left',5,[point(start+114+cross_shift,s+8.5),point(start,s+8.5)])]:
        ET.SubElement(edges,'edge',id=edge_id,**{'from':source,'to':target,'numLanes':str(lanes),
                      'width':'3.2','speed':'8.33','priority':'2','spreadType':'center','shape':_write(shape)})
    for lane in range(4):
        ET.SubElement(connections,'connection',**{'from':'SW_end_T_left_out','to':'SW_end_T_taper_out','fromLane':str(lane),'toLane':str(lane)})
        ET.SubElement(connections,'connection',**{'from':'SW_end_T_taper_out','to':'SW_end_T_extension_out','fromLane':str(lane),'toLane':str(lane)})
    for lane in range(5):
        ET.SubElement(connections,'connection',**{'from':'SW_new_cross_in','to':'SW_end_T_left_in','fromLane':str(lane),'toLane':str(lane)})
    # Cross street is tilted 15 degrees from the perpendicular to the main road.
    import math
    center=start+114
    for sign,label in [(1,'upper'),(-1,'lower')]:
        endpoint=point(center+sign*80*math.sin(math.radians(15))-12.8*math.cos(math.radians(15)),s+sign*80*math.cos(math.radians(15))+12.8*math.sin(math.radians(15)))
        endpoint=(endpoint[0],endpoint[1]-10.0)
        ET.SubElement(nodes,'node',id='SW_new_'+label,x=str(endpoint[0]),y=str(endpoint[1]),type='dead_end')
        for suffix,source,target in [('in','SW_new_'+label,'SW_new_cross'),('out','SW_new_cross','SW_new_'+label)]:
            ET.SubElement(edges,'edge',id='SW_new_'+label+'_'+suffix,**{'from':source,'to':target,'numLanes':str(6 if suffix=='in' else (5 if label=='upper' else 4)),'width':'3.2','speed':'8.33','priority':'1','spreadType':'right'})
    # A one-lane right-turn bypass separates before the stop line and merges
    # into the lower branch, surrounding the triangular painted island.
    theta=math.radians(15)
    md=center-50*math.sin(theta)-12.8*math.cos(theta)
    ml=s-50*math.cos(theta)+12.8*math.sin(theta)
    xy=point(md,ml)
    xy=(xy[0],xy[1]-10.0)
    ET.SubElement(nodes,'node',id='SW_slip_merge',x=str(xy[0]),y=str(xy[1]),type='priority',radius='4')
    lower=next(e for e in edges if e.get('id')=='SW_new_lower_out')
    lower.set('id','SW_new_lower_near'); lower.set('to','SW_slip_merge')
    ET.SubElement(edges,'edge',id='SW_new_lower_out',**{'from':'SW_slip_merge','to':'SW_new_lower','numLanes':'4','width':'3.2','speed':'8.33','priority':'1','spreadType':'right'})
    for lane in range(5):
        ET.SubElement(connections,'connection',**{'from':'SW_end_T_extension_out','to':'SW_slip_main','fromLane':str(lane),'toLane':str(lane)})
    for fr,to in [('SW_new_lower_near','SW_new_lower_out')]:
        ET.SubElement(connections,'connection',**{'from':fr,'to':to,'fromLane':'0','toLane':'0'})
    ET.SubElement(connections,'connection',**{'from':'SW_new_lower_near','to':'SW_new_lower_out','fromLane':'1','toLane':'1'})
    for lane in range(5):
        for source,target in [('SW_slip_main','SW_new_cross_out'),('SW_end_T_extension_in','SW_new_cross_in')]:
            ET.SubElement(connections,'connection',**{'from':source,'to':target,'fromLane':str(lane),'toLane':str(lane)})
    for source,target,src,dst in [
        ('SW_slip_main','SW_new_lower_near',0,0),('SW_slip_main','SW_new_upper_out',4,0),
        ('SW_end_T_extension_in','SW_new_upper_out',0,0),('SW_end_T_extension_in','SW_new_lower_near',4,0),
        ('SW_new_upper_in','SW_new_lower_near',0,0),('SW_new_upper_in','SW_new_cross_out',0,0),('SW_new_upper_in','SW_new_cross_in',0,4),
        ('SW_new_lower_in','SW_new_upper_out',0,0),('SW_new_lower_in','SW_new_cross_in',0,0),('SW_new_lower_in','SW_new_cross_out',0,4)]:
        ET.SubElement(connections,'connection',**{'from':source,'to':target,'fromLane':str(src),'toLane':str(dst)})
    for source,target in [('SW_new_upper_in','SW_new_lower_near'),('SW_new_lower_in','SW_new_upper_out')]:
        ET.SubElement(connections,'connection',**{'from':source,'to':target,'fromLane':'1','toLane':'1'})
    for source,target,count in [('SW_new_upper_in','SW_new_lower_near',5),('SW_new_lower_in','SW_new_upper_out',6),('SW_new_lower_near','SW_new_lower_out',5)]:
        for lane in range(2,count):
            ET.SubElement(connections,'connection',**{'from':source,'to':target,'fromLane':str(lane),'toLane':str(lane)})
    side_pairs={('SW_new_upper_in','SW_new_lower_near'),('SW_new_lower_in','SW_new_upper_out')}
    for connection in list(connections):
        if (connection.get('from'),connection.get('to')) in side_pairs:
            connections.remove(connection)
    for source,target in side_pairs:
        for lane in range(5):
            ET.SubElement(connections,'connection',**{'from':source,'to':target,'fromLane':str(lane+1),'toLane':str(lane)})
    for connection in connections:
        if (connection.get('from'),connection.get('to')) in {('SW_new_upper_in','SW_new_cross_in'),('SW_new_lower_in','SW_new_cross_out')}:
            connection.set('fromLane','5')
    # Four receiving lanes on the branch beside the original slip island.
    for connection in list(connections):
        if connection.get('to') in ('SW_new_lower_near','SW_new_lower_out') and int(connection.get('toLane','0'))>=4:
            connections.remove(connection)
    # The outermost (sixth from the median) lower approach lane turns right.
    for connection in connections:
        if connection.get('from')=='SW_new_lower_in':
            if connection.get('to')=='SW_new_cross_out':
                connection.set('fromLane','0');connection.set('toLane','0')
            elif connection.get('to')=='SW_new_cross_in':
                connection.set('fromLane','5');connection.set('toLane','4')
    for connection in connections:
        if connection.get('from')=='SW_new_upper_in':
            if connection.get('to')=='SW_new_cross_in':
                connection.set('fromLane','0');connection.set('toLane','0')
            elif connection.get('to')=='SW_new_cross_out':
                connection.set('fromLane','5');connection.set('toLane','4')
    # Keep the five existing straight/left lane alignments and open the new
    # outside lane into the existing right-turn exit.
    for connection in connections:
        if connection.get('from')=='SW_end_T_extension_in' and connection.get('to')!='SW_new_upper_out':
            connection.set('fromLane',str(int(connection.get('fromLane'))+1))
    def connect(source,target,source_lane=0,target_lane=0):
        ET.SubElement(connections,'connection',**{'from':source,'to':target,
                      'fromLane':str(source_lane),'toLane':str(target_lane),'contPos':'0'})
    for lane in range(3):
        connect('SW_final_forward_out','SW_end_T_right_out',lane,lane)
    connect('SW_final_forward_out','SW_end_T_left_out',3,3)
    for lane in range(5):
        connect('SW_end_T_left_in','SW_end_T_right_out',lane,lane)
    connect('SW_end_T_left_in','SW_final_forward_in',0,0)
    for lane in range(3):
        connect('SW_end_T_right_in','SW_end_T_left_out',lane,lane)
    for lane in range(2):
        connect('SW_end_T_right_in','SW_final_forward_in',lane+3,lane+1)

    from env.roundabout_long_right import add_network as add_long_right
    add_long_right(nodes,edges,connections)


def add_routes(root):
    incoming=outgoing=0
    for flow in root.findall('flow'):
        if flow.get('from')=='SW_final_forward_in':
            flow.set('from','SW_end_T_left_in' if incoming%2==0 else 'SW_end_T_right_in')
            incoming+=1
        if flow.get('to')=='SW_final_forward_out':
            flow.set('to','SW_end_T_left_out' if outgoing%2==0 else 'SW_end_T_right_out')
            outgoing+=1
        if flow.get('from')=='SW_end_T_left_in':
            flow.set('from','SW_end_T_extension_in')
        if flow.get('to')=='SW_end_T_left_out':
            flow.set('to','SW_new_cross_out')


def add_markings(root,net_path):
    from shapely.geometry import Polygon,LineString
    from shapely.ops import unary_union
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_surfaces import _network,_polygons,_lines
    arm_length=END_CROSSROADS['sw_end_t_arm_length']
    left_length=END_CROSSROADS.get('sw_end_t_left_length',arm_length)
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']
    _,lanes,strips=_network(net_path)
    road=unary_union(list(strips.values()))
    # Keep both carriageways on one straight alignment across the T junction.
    pavement=[point(d-arm_length,s-16.5),point(d-30,s-16.5),point(d,s-13.3),point(d+left_length,s-13.3),
              point(d+left_length,s+16.5),point(d-arm_length,s+16.5)]
    ET.SubElement(root,'poly',id='SW_end_T_straight_pavement',color='84,84,90',
                  fill='1',layer='4.9',shape=_write(pavement))
    for label,start,finish in [('left',d,d+left_length),('right',d-arm_length,d)]:
        median=Polygon([point(start,s-0.5),point(finish,s-0.5),
                        point(finish,s+0.5),point(start,s+0.5)]).difference(road)
        for i,part in enumerate(_polygons(median)):
            ET.SubElement(root,'poly',id=f'SW_end_T_median_pavement_{label}_{i}',
                          color='84,84,90',fill='1',layer='4.9',shape=_write(list(part.exterior.coords)))
        center=LineString([point(start,s),point(finish,s)]).difference(road.buffer(0.03))
        for i,line in enumerate(_lines(center)):
            if line.length<3.0:
                continue
            ET.SubElement(root,'poly',id=f'SW_end_T_center_{label}_{i}',color='235,190,40',
                          fill='0',lineWidth='0.2',layer='6',shape=_write(list(line.coords)))
    from env.roundabout_geometry import _read
    from env.roundabout_markings import dash_style
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    def local(q):
        return tuple(sum((q[j]-origin[j])*axis[j] for j in (0,1)) for axis in (along,across))
    rows=[s+x for x in (-10.1,-6.9,-3.7,3.7,6.9,10.1,13.3)]
    all_rows=rows+[s-13.3]
    for p in list(root):
        if not p.get('id','').startswith('lane_dash_'):
            continue
        coords=[local(q) for q in _read(p.get('shape'))]
        if any(all(abs(q[1]-row)<0.01 for q in coords) for row in all_rows):
            if min(q[0] for q in coords)>=d-arm_length-0.01 and max(q[0] for q in coords)<=d+left_length+0.01:
                root.remove(p)
    dash,gap,_=dash_style(20)
    count=max(0,int((arm_length+left_length-dash)//(dash+gap))+1)
    for row,lateral in enumerate(rows):
        for i in range(count):
            start=d-arm_length+i*(dash+gap)
            ET.SubElement(root,'poly',id=f'SW_end_T_aligned_dash_{row}_{i}',color='255,255,255',
                          fill='0',lineWidth='0.15',layer='6',
                          shape=_write([point(start,lateral),point(start+dash,lateral)]))
    # Solid white stop lines at the user-indicated approach positions.
    def station(q):
        return sum((q[j]-origin[j])*along[j] for j in (0,1))
    right_stop=station(lanes['SW_end_T_right_in_0'][-1])-1.5
    left_stop=station(lanes['SW_end_T_left_in_0'][-1])+1.5
    extra_count=max(0,int((right_stop-(d-arm_length)-dash)//(dash+gap))+1)
    for i in range(extra_count):
        start=d-arm_length+i*(dash+gap)
        ET.SubElement(root,'poly',id=f'SW_end_T_extra_outer_dash_{i}',color='255,255,255',
                      fill='0',lineWidth='0.15',layer='6',
                      shape=_write([point(start,s-13.3),point(start+dash,s-13.3)]))
    for label,stop,low,high in [('right',right_stop,s-16.5,s),('left',left_stop,s,s+16.5)]:
        ET.SubElement(root,'poly',id=f'SW_end_T_stop_{label}',
                      color='255,255,255',fill='0',lineWidth='0.45',layer='6',
                      shape=_write([point(stop,low),point(stop,high)]))
    add_right_approach_arrows(root,net_path)
    add_left_approach_arrows(root,net_path)
    add_extension_markings(root,point,d+left_length,s)
    from env.roundabout_long_right import add_markings as mark_long_right
    mark_long_right(root)
    add_transverse_center(root,net_path)
    add_straight_transverse_pavement(root,net_path)
    add_east_approach_pavement(root,net_path)
    add_straight_transverse_dashes(root,net_path)
    add_remaining_corner_pavement(root,net_path)
    from env.roundabout_drawn_islands import add_markings as mark_drawn_islands
    mark_drawn_islands(root,net_path)
    from env.roundabout_long_right import add_entry_stop_and_island
    add_entry_stop_and_island(root,net_path)
    from env.roundabout_crossing_cleanup import refine
    refine(root,net_path)
    from env.roundabout_crossing_cleanup import extend_upper_island_to_divider
    extend_upper_island_to_divider(root,net_path)
    from env.roundabout_crossing_cleanup import fit_right_island_curves
    fit_right_island_curves(root,net_path)
    from env.roundabout_road_first_corners import finish as finish_road_first
    finish_road_first(root,net_path)
    from env.roundabout_road_first_corners import align_lower_outer_dash
    align_lower_outer_dash(root,net_path)
    from env.roundabout_road_first_corners import expand_upper_island_to_sketch
    expand_upper_island_to_sketch(root)
    from env.roundabout_road_first_corners import expand_lower_island_to_sketch
    expand_lower_island_to_sketch(root)
    from env.roundabout_road_first_corners import extend_lower_island_along_turn
    extend_lower_island_along_turn(root,net_path)
    from env.roundabout_road_first_corners import extend_upper_right_island_along_straight
    extend_upper_right_island_along_straight(root,net_path)
    from env.roundabout_road_first_corners import fit_upper4_island_boundary
    fit_upper4_island_boundary(root,net_path)
    from env.roundabout_long_right import add_six_approach_arrows
    add_six_approach_arrows(root,net_path)
    add_east_approach_stop(root,net_path)
    clear_east_stop_dash_remnants(root)
    add_east_outer_lead_dash(root,net_path)
    add_east_approach_arrows(root,net_path)
    add_upper_approach_stop_and_arrows(root,net_path)
    from env.roundabout_road_first_corners import connect_main_outer_divider
    connect_main_outer_divider(root,net_path)
    add_lower_approach_stop_and_arrows(root,net_path)
    # Clear only the twelve lower-approach dashes selected in the reference.
    selected={f'SW_new_straight_dash_lower_{row}_{index}'
              for row in range(4) for index in range(9,12)}
    for poly in list(root):
        if poly.get('id') in selected:root.remove(poly)
    from env.roundabout_road_first_corners import add_lower5_straight_guide
    add_lower5_straight_guide(root,net_path)
    for poly in list(root):
        if poly.get('id')=='cropped_upper4_exact_SW_new_straight_dash_upper_8_5_0':
            root.remove(poly)
    selected_upper={f'SW_new_straight_dash_upper_{row}_{index}'
                    for row in (5,6) for index in (6,7,8)}
    selected_upper.update(f'cropped_core_SW_new_straight_dash_upper_{row}_9_0'
                          for row in (5,6))
    # Retain the incoming movement's guide, removing its three parallel native dashes.
    selected_upper.update(f'SW_new_straight_dash_upper_7_{index}' for index in (6,7,8))
    for poly in list(root):
        if poly.get('id') in selected_upper:root.remove(poly)
    selected_lower_opposite={f'SW_new_straight_dash_lower_{row}_{index}'
                             for row in (5,6,7) for index in (7,8,9,10)}
    selected_lower_opposite.update(f'cropped_core_SW_new_straight_dash_lower_{row}_11_0'
                                   for row in (5,6))
    for poly in list(root):
        if poly.get('id') in selected_lower_opposite:root.remove(poly)
    from env.roundabout_road_first_corners import clear_four_selected_dash_areas
    clear_four_selected_dash_areas(root)
    from env.roundabout_four_left_guides import add_guides as add_four_left_guides
    add_four_left_guides(root,net_path)
    from env.roundabout_horizontal_guides import add_guides as add_horizontal_guides
    add_horizontal_guides(root,net_path)
    from env.roundabout_lower_outer_divider import add_divider as add_lower_outer_divider
    add_lower_outer_divider(root,net_path)
    from env.roundabout_lower_outer_divider import add_receiving_continuations
    add_receiving_continuations(root,net_path)
    from env.roundabout_west_outer_divider import add_divider as add_west_outer_divider
    add_west_outer_divider(root,net_path)
    from env.roundabout_median_pavement import fill_west_stop_gap
    fill_west_stop_gap(root,net_path)
    # Lower approach stop removed; align the slip stop with the main stop instead.
    # Triangular safety-zone markings removed at user request.


def add_extension_markings(root,point,start,s):
    """Paint the median and striped safety island at the new inner lane."""
    import math
    from shapely.geometry import Polygon,LineString
    from env.roundabout_surfaces import _lines
    inner=[point(start+40*i/80,s-0.5-3.2*(3*(i/80)**2-2*(i/80)**3)) for i in range(81)]
    inner += [point(start+40+14*i/40,s-3.7+3.2*(3*(i/40)**2-2*(i/40)**3)) for i in range(1,41)]
    pocket=Polygon([point(start,s-0.5),point(start+54,s-0.5)]+list(reversed(inner)))
    for name,shape,fill,width,layer in [
        ('pavement',list(pocket.exterior.coords),'1','0.2','4.9'),
        ('border',inner,'0','0.2','6'),
        ('center',[point(start,s),point(start+90,s)],'0','0.2','6'),
        ('center_far',[point(start+138,s),point(start+180,s)],'0','0.2','6'),
        ('stop',[point(start+68,s-16.5),point(start+68,s)],'0','0.45','6')]:
        ET.SubElement(root,'poly',id='SW_end_T_extension_'+name,color='84,84,90' if name=='pavement' else ('255,255,255' if name=='stop' else '235,190,40'),
                      fill=fill,lineWidth=width,layer=layer,shape=_write(shape))
    # Pave the full new inner-lane opening beneath the safety-zone nose.
    ET.SubElement(root,'poly',id='SW_end_T_extension_opening_pavement',color='84,84,90',
                  fill='1',layer='4.9',shape=_write([point(start+40,s-3.7),point(start+54,s-3.7),point(start+54,s-.5),point(start+40,s-.5)]))
    for i in range(3,54,4):
        for k,line in enumerate(_lines(LineString([point(start+i-2,s-0.5),point(start+i+2,s-4)]).intersection(pocket.buffer(-0.18)))):
            if line.length>.15:
                ET.SubElement(root,'poly',id=f'SW_end_T_extension_hatch_{i}_{k}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(line.coords)))


def add_right_approach_arrows(root,net_path):
    from env.roundabout_builder import _read_lane_shapes,_arrow
    from env.roundabout_markings import dash_style
    for p in list(root):
        if p.get('id','').startswith('SW_end_T_right_arrow_'):
            root.remove(p)
    shapes=_read_lane_shapes(net_path)
    dash,_,_=dash_style(20)
    arrows=[]
    for lane,kind in enumerate(('straight','straight','straight','left','left')):
        _arrow(arrows,f'SW_end_T_right_arrow_{lane}',shapes,'SW_end_T_right_in',lane,kind,14.0-dash)
    for arrow in arrows:
        root.append(ET.fromstring(arrow))


def add_left_approach_arrows(root,net_path):
    from env.roundabout_builder import _read_lane_shapes,_arrow
    from env.roundabout_markings import dash_style
    for p in list(root):
        if p.get('id','').startswith('SW_end_T_left_arrow_'):
            root.remove(p)
    shapes=_read_lane_shapes(net_path)
    dash,_,_=dash_style(20)
    arrows=[]
    for lane,kind in enumerate(('straight_right','straight','straight','straight','straight')):
        _arrow(arrows,f'SW_end_T_left_arrow_{lane}',shapes,'SW_end_T_left_in',lane,kind,14.0-dash)
    for arrow in arrows:
        root.append(ET.fromstring(arrow))


def clear_marked_dashes(root):
    """Remove only white dashed strokes inside the user's black outline."""
    from shapely.geometry import Polygon,LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _read
    from env.roundabout_surfaces import _lines
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']
    outline=[(-30.144,13.708),(-26.734,14.723),(-21.270,17.156),(-17.081,17.174),
             (-11.708,17.551),(-5.479,17.135),(0.143,16.952),(6.606,17.143),
             (12.773,16.867),(19.500,16.840),(24.951,15.909),(29.686,14.324),
             (33.082,13.485),(34.564,9.780),(34.099,5.544),(32.840,1.961),
             (30.725,-2.338),(28.671,-5.267),(28.098,-8.881),(27.492,-11.669),
             (25.764,-11.935),(22.649,-10.971),(18.787,-11.347),(13.119,-12.192),
             (7.716,-13.254),(1.721,-13.741),(-4.648,-14.898),(-10.643,-15.384),
             (-16.685,-15.388),(-21.295,-15.594),(-25.733,-15.052),(-30.234,-14.369),
             (-34.565,-12.940),(-36.215,-13.003),(-36.622,-9.826),(-36.951,-7.958),
             (-36.392,-4.687),(-36.346,-3.659),(-34.899,-2.008),(-33.545,-0.901),
             (-31.989,1.638),(-30.199,3.275),(-30.934,6.809),(-30.313,9.940)]
    area=Polygon([point(d+x,s+y) for x,y in outline])
    for p in list(root):
        name=p.get('id','')
        if p.get('color')!='255,255,255' or p.get('fill')!='0':
            continue
        if 'stop' in name or 'arrow' in name or float(p.get('lineWidth','0.15'))>0.2:
            continue
        path=LineString(_read(p.get('shape')))
        if not path.intersects(area):
            continue
        root.remove(p)
        for i,line in enumerate(_lines(path.difference(area))):
            if line.length<0.05:
                continue
            clone=ET.Element('poly',p.attrib)
            clone.set('id',f'cropped_{name}_{i}')
            clone.set('shape',_write(list(line.coords)))
            root.append(clone)


def clear_center_fragment(root):
    """Remove the isolated yellow median fragment in the junction center."""
    from shapely.geometry import Point,LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _read
    point,_,_,end=layout()
    center=Point(point(end+51,-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']))
    for p in list(root):
        if p.get('id','').startswith(('SW_end_T_center_','SW_forward_center_line_')):
            if LineString(_read(p.get('shape'))).distance(center)<0.1:
                root.remove(p)


def add_median_guides(root,net_path):
    """Join median tips along the actual inside boundaries of both left turns."""
    import math
    from shapely.geometry import LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _read,_slice
    from env.roundabout_markings import dash_style
    for p in list(root):
        if p.get('id','').startswith('SW_end_T_median_guide_'):
            root.remove(p)
    point,_,_,end=layout()
    center=point(end+51,-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length'])
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    tips={}
    for label,prefix in [('stem','SW_forward_center_line_'),
                         ('left','SW_end_T_center_right_'),('right','SW_end_T_center_left_')]:
        candidates=[q for p in root if p.get('id','').startswith(prefix)
                    for q in (_read(p.get('shape'))[0],_read(p.get('shape'))[-1])]
        tips[label]=min(candidates,key=lambda q:math.dist(q,center))
    from env.roundabout_left_guides import median_boundary_path
    paths={}
    for label,source,target,fr,to in [
            ('left_to_stem','left','stem','SW_end_T_right_in','SW_final_forward_in'),
            ('stem_to_right','stem','right','SW_final_forward_out','SW_end_T_left_out')]:
        paths[label]=median_boundary_path(net_path,fr,to,tips[source],tips[target])
    dash,standard_gap,_=dash_style(20)
    for label,path in paths.items():
        length=LineString(path).length
        count=max(2,round((length+standard_gap)/(dash+standard_gap)))
        gap=(length-count*dash)/(count-1)
        for i in range(count):
            start=i*(dash+gap)
            ET.SubElement(root,'poly',id=f'SW_end_T_median_guide_{label}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,start+dash)))


def add_through_guides(root,net_path):
    """Three smooth separators following the actual through-lane connections."""
    from shapely.geometry import LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _read,_offset,_slice
    from env.roundabout_markings import dash_style
    from env.roundabout_builder import _movement_path
    point,_,_,_=layout()
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    stop=next(p for p in root if p.get('id')=='SW_end_T_stop_right')
    endpoint=_read(stop.get('shape'))[-1]
    stop_d=sum((endpoint[j]-origin[j])*along[j] for j in (0,1))
    s=-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']
    for p in list(root):
        if p.get('id','').startswith('SW_end_T_through_guide_'):
            root.remove(p)
    dash,gap,_=dash_style(20)
    for lane in range(3):
        curve=_offset(_movement_path(net_path,'SW_end_T_right_in','SW_end_T_left_out',lane),1.6)
        path=[point(stop_d,s-13.3+lane*3.2)]+curve
        length=LineString(path).length
        lead=0.5
        count=max(0,int((length-lead-dash)//(dash+gap))+1)
        for i in range(count):
            start=lead+i*(dash+gap)
            ET.SubElement(root,'poly',id=f'SW_end_T_through_guide_{lane}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,start+dash)))


def add_reverse_through_guides(root,net_path):
    """Connect all four dividers of the five-lane through carriageway."""
    import math
    from shapely.geometry import Polygon,LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _read
    from env.roundabout_markings import dash_style
    from env.roundabout_builder import _read_lane_shapes
    from env.roundabout_surfaces import _lines
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']-END_CROSSROADS['sw_forward_length']
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    def local(q):
        return tuple(sum((q[j]-origin[j])*axis[j] for j in (0,1)) for axis in (along,across))
    stop=next(p for p in root if p.get('id')=='SW_end_T_stop_left')
    finish=local(_read(stop.get('shape'))[0])[0]
    shapes=_read_lane_shapes(net_path)
    start=local(shapes[('SW_end_T_right_out',0)][0])[0]
    rows=[s+x for x in (3.7,6.9,10.1,13.3)]
    for p in list(root):
        name=p.get('id','')
        if name.startswith('SW_end_T_reverse_through_guide_'):
            root.remove(p)
            continue
        if p.get('color')!='255,255,255' or p.get('fill')!='0' or 'stop' in name or 'arrow' in name:
            continue
        pts=_read(p.get('shape'))
        coords=[local(q) for q in pts]
        if not any(all(abs(q[1]-row)<0.01 for q in coords) for row in rows):
            continue
        area=Polygon([point(start,s+3.5),point(finish,s+3.5),
                      point(finish,s+13.5),point(start,s+13.5)])
        path=LineString(pts)
        if not path.intersects(area):
            continue
        root.remove(p)
        for i,line in enumerate(_lines(path.difference(area))):
            if line.length<0.05:
                continue
            clone=ET.Element('poly',p.attrib)
            clone.set('id',f'outside_{name}_{i}')
            clone.set('shape',_write(list(line.coords)))
            root.append(clone)
    dash,gap,_=dash_style(20)
    base=d-END_CROSSROADS['sw_end_t_arm_length']
    first=math.floor((start-base)/(dash+gap))
    last=math.ceil((finish-base)/(dash+gap))
    for row,lateral in enumerate(rows):
        for i in range(first,last+1):
            a=max(start,base+i*(dash+gap))
            b=min(finish,base+i*(dash+gap)+dash)
            if b-a<0.1:
                continue
            ET.SubElement(root,'poly',id=f'SW_end_T_reverse_through_guide_{row}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write([point(a,lateral),point(b,lateral)]))



def add_slip_island(root,net_path,point,start,s):
    """Fill and stripe the enclosed island between the bypass and junction."""
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    from env.roundabout_surfaces import _network, _polygons, _lines
    import math
    original_point=point
    def point(d,lateral):
        q=original_point(d,lateral)
        return q[0],q[1]-10.0
    _,_,strips=_network(net_path)
    road=unary_union([v for k,v in strips.items() if k.startswith(('SW_new_slip_','SW_slip_main_','SW_new_lower_near_','SW_new_lower_out_'))])
    triangle=Polygon([point(start+62,s-16.5),point(start+108,s-16.5),point(start+114-50*math.sin(math.radians(15)),s-50*math.cos(math.radians(15)))])
    # The painted island is only on the junction side of the slip lane.
    # Extend the lane centerline through the triangle so the narrow outside
    # sliver cannot reconnect around an end of the lane strip.
    from env.roundabout_geometry import _read
    import xml.etree.ElementTree as ET
    net=ET.parse(net_path).getroot()
    lane=net.find("edge[@id='SW_new_slip']/lane")
    curve=_read(lane.get('shape'))
    def extend(a,b,distance):
        length=math.hypot(a[0]-b[0],a[1]-b[1])
        return (a[0]+distance*(a[0]-b[0])/length,a[1]+distance*(a[1]-b[1])/length)
    cutter=LineString([extend(curve[0],curve[1],25)]+curve+[extend(curve[-1],curve[-2],25)]).buffer(2.0)
    pieces=list(_polygons(triangle.difference(road.buffer(.3)).difference(cutter)))
    island=max(pieces,key=lambda x:x.area) if pieces else Polygon()
    for j,part in enumerate(_polygons(island)):
        if part.area<1: continue
        ET.SubElement(root,'poly',id=f'SW_new_slip_island_fill_{j}',color='84,84,90',fill='1',layer='5',shape=_write(list(part.exterior.coords)))
        ET.SubElement(root,'poly',id=f'SW_new_slip_island_border_{j}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(part.exterior.coords)))
        for i in range(60,115,4):
            line=LineString([point(start+i,s-16),point(start+i+28,s-60)])
            for k,piece in enumerate(_lines(line.intersection(part.buffer(-.2)))):
                if piece.length>.2:
                    ET.SubElement(root,'poly',id=f'SW_new_slip_hatch_{j}_{i}_{k}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(piece.coords)))


def add_transverse_center(root,net_path):
    net=ET.parse(net_path).getroot()
    center=net.find("junction[@id='SW_new_cross']")
    a=(float(center.get('x')),float(center.get('y')))
    import math
    for label in ('upper','lower'):
        node=net.find(f"junction[@id='SW_new_{label}']")
        b=(float(node.get('x')),float(node.get('y')))
        distance=math.dist(a,b)
        first=tuple(a[j]+(b[j]-a[j])*32/distance for j in (0,1))
        ET.SubElement(root,'poly',id=f'SW_new_center_{label}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write([first,b]))


def add_straight_transverse_pavement(root,net_path):
    """Carry each approach's full road width straight through to the crossing."""
    import math
    net=ET.parse(net_path).getroot()
    center=net.find("junction[@id='SW_new_cross']")
    a=(float(center.get('x')),float(center.get('y')))
    for label,incoming,outgoing in [('upper',6,5),('lower',6,4)]:
        node=net.find(f"junction[@id='SW_new_{label}']")
        b=(float(node.get('x')),float(node.get('y')))
        length=math.dist(a,b)
        direction=tuple((b[j]-a[j])/length for j in (0,1))
        right=(direction[1],-direction[0])
        def side(q,offset): return tuple(q[j]+right[j]*offset for j in (0,1))
        # Looking out from the junction, incoming lanes are on the left.
        shape=[side(a,-incoming*3.2),side(b,-incoming*3.2),side(b,outgoing*3.2),side(a,outgoing*3.2)]
        ET.SubElement(root,'poly',id=f'SW_new_straight_pavement_{label}',color='84,84,90',fill='1',layer='4.9',shape=_write(shape))


def add_east_approach_pavement(root,net_path):
    """Cover native SUMO markings under all six lanes with uniform asphalt."""
    from env.roundabout_builder import _read_lane_shapes
    from env.roundabout_geometry import _offset
    net=ET.parse(net_path).getroot()
    edge=net.find("edge[@id='SW_end_T_extension_in']")
    lanes=edge.findall('lane')
    shapes=_read_lane_shapes(net_path)
    outer=_offset(shapes[('SW_end_T_extension_in',0)],-float(lanes[0].get('width'))/2)
    inner=_offset(shapes[('SW_end_T_extension_in',len(lanes)-1)],float(lanes[-1].get('width'))/2)
    for poly in list(root):
        if poly.get('id')=='SW_new_east_pavement':root.remove(poly)
    ET.SubElement(root,'poly',id='SW_new_east_pavement',color='84,84,90',fill='1',
                  layer='4.9',shape=_write(outer+inner[::-1]))


def add_straight_transverse_dashes(root,net_path):
    """Continue the transverse road's lane dividers at constant 3.2 m spacing."""
    import math
    from env.roundabout_markings import dash_style
    prefixes=('lane_dash_SW_new_upper_in_', 'lane_dash_SW_new_upper_out_',
              'lane_dash_SW_new_lower_in_', 'lane_dash_SW_new_lower_near_',
              'lane_dash_SW_new_lower_out_', 'SW_new_straight_dash_')
    for poly in list(root):
        if poly.get('id','').startswith(prefixes) or 'SW_new_straight_dash_' in poly.get('id',''): root.remove(poly)
    net=ET.parse(net_path).getroot()
    center=net.find("junction[@id='SW_new_cross']")
    a=(float(center.get('x')),float(center.get('y')))
    dash,gap,_=dash_style(20)
    for label in ('upper','lower'):
        node=net.find(f"junction[@id='SW_new_{label}']")
        b=(float(node.get('x')),float(node.get('y')))
        length=math.dist(a,b)
        direction=tuple((b[j]-a[j])/length for j in (0,1))
        right=(direction[1],-direction[0])
        def point(station,offset): return tuple(a[j]+direction[j]*station+right[j]*offset for j in (0,1))
        rows=[-3.2*i for i in range(1,6)]+[3.2*i for i in range(1,5 if label=='upper' else 4)]
        for row,offset in enumerate(rows):
            # Work inward from the endpoint so a full dash reaches the road end.
            station=length-dash
            index=0
            while station>=0:
                ET.SubElement(root,'poly',id=f'SW_new_straight_dash_{label}_{row}_{index}',color='255,255,255',fill='0',lineWidth='0.15',layer='6',shape=_write([point(station,offset),point(station+dash,offset)]))
                station-=dash+gap
                index+=1


def add_lower_wait_stop(root,net_path):
    """Place the stop line at the actual minor-road junction waiting boundary."""
    from env.roundabout_builder import _read_lane_shapes,_stop_line
    for poly in list(root):
        if poly.get('id')=='SW_new_lower_wait_stop':root.remove(poly)
    shapes=_read_lane_shapes(net_path)
    marks=[]
    _stop_line(marks,'SW_new_lower_wait_stop',shapes,'SW_new_lower_in',dist=.4)
    for mark in marks:root.append(ET.fromstring(mark))


def add_remaining_corner_pavement(root,net_path):
    """Round the other three road-edge joins without changing lane widths."""
    from shapely.geometry import Polygon,Point
    from shapely.ops import unary_union
    from env.roundabout_geometry import _read
    from env.roundabout_surfaces import _network,_polygons
    for poly in list(root):
        if poly.get('id','').startswith('SW_new_corner_pavement_'):root.remove(poly)
    net=ET.parse(net_path).getroot()
    node=net.find("junction[@id='SW_new_cross']")
    center=(float(node.get('x')),float(node.get('y')))
    _,_,strips=_network(net_path)
    surfaces=list(strips.values())
    surfaces += [Polygon(_read(p.get('shape'))) for p in root if p.get('id','').startswith('SW_new_straight_pavement_')]
    surfaces.append(Polygon(_read(node.get('shape'))))
    road=unary_union(surfaces).intersection(Point(center).buffer(85))
    additions=road.buffer(20,quad_segs=24).buffer(-20,quad_segs=24).difference(road)
    point,_,_,_=layout()
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    for i,part in enumerate(_polygons(additions)):
        if part.area<.5 or part.centroid.distance(Point(center))>50:continue
        delta=(part.centroid.x-center[0],part.centroid.y-center[1])
        if sum(delta[j]*along[j] for j in (0,1))<0 and sum(delta[j]*across[j] for j in (0,1))<0:continue
        ET.SubElement(root,'poly',id=f'SW_new_corner_pavement_{i}',color='84,84,90',fill='1',layer='4.9',shape=_write(list(part.exterior.coords)))


def add_remaining_corner_islands(root):
    """Stripe the three corner pockets outside the full-width vehicle lanes."""
    import math
    from shapely.geometry import Polygon,LineString
    from env.roundabout_geometry import _read
    from env.roundabout_surfaces import _polygons,_lines
    for poly in list(root):
        if poly.get('id','').startswith('SW_new_corner_island_'):root.remove(poly)
    patches=[p for p in root if p.get('id','').startswith('SW_new_corner_pavement_')]
    for i,patch in enumerate(patches):
        # The pavement patches were cut against actual lane strips. A small
        # inset keeps the paint off those 3.2 m lane boundaries.
        parts=list(_polygons(Polygon(_read(patch.get('shape'))).buffer(-.2)))
        if not parts:continue
        island=max(parts,key=lambda p:p.area)
        ET.SubElement(root,'poly',id=f'SW_new_corner_island_border_{i}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(island.exterior.coords)))
        lowx,lowy,highx,highy=island.bounds
        span=max(highx-lowx,highy-lowy)+10
        for j,x in enumerate(range(math.floor(lowx-span),math.ceil(highx+span),4)):
            stroke=LineString([(x,lowy-2),(x+(highy-lowy)+4,highy+2)])
            for k,piece in enumerate(_lines(stroke.intersection(island.buffer(-.2)))):
                if piece.length<.3:continue
                ET.SubElement(root,'poly',id=f'SW_new_corner_island_hatch_{i}_{j}_{k}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(piece.coords)))


def add_east_approach_stop(root,net_path):
    from env.roundabout_geometry import _read
    from env.roundabout_config import END_CROSSROADS as C
    for p in list(root):
        if p.get('id')=='SW_new_east_stop':root.remove(p)
    point,_,_,_=layout();origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    border=root.find("poly[@id='SW_road_first_island_border_1']")
    if border is None:raise ValueError('Stop line needs the finished island tip')
    coords=_read(border.get('shape'))
    local=[(sum((q[j]-origin[j])*along[j] for j in (0,1)),sum((q[j]-origin[j])*across[j] for j in (0,1))) for q in coords]
    low=min(q[1] for q in local)
    tip=max((q for q in local if q[1]<=low+.01),key=lambda q:q[0])
    median=-C['sw_arm_length']-C['sw_forward_length']
    ET.SubElement(root,'poly',id='SW_new_east_stop',color='255,255,255',fill='0',lineWidth='.45',layer='6',shape=_write([point(tip[0],median),point(tip[0],tip[1])]))


def add_lower_approach_stop_and_arrows(root,net_path):
    """Place the sketched lower stop line and six lane-direction arrows."""
    import math
    from env.roundabout_builder import _read_lane_shapes,_arrow
    from env.roundabout_geometry import _read
    net=ET.parse(net_path).getroot()
    junction=net.find("junction[@id='SW_new_cross']")
    lower=net.find("junction[@id='SW_new_lower']")
    center=tuple(float(junction.get(k)) for k in ('x','y'))
    endpoint=tuple(float(lower.get(k)) for k in ('x','y'))
    direction=tuple((endpoint[j]-center[j])/math.dist(center,endpoint) for j in (0,1))
    def station(q):return sum((q[j]-center[j])*direction[j] for j in (0,1))
    island=root.find("poly[@id='SW_road_first_island_border_2']")
    tip=min(_read(island.get('shape')),key=lambda q:q[0])
    stop_station=station(tip)
    median=tuple(center[j]+direction[j]*stop_station for j in (0,1))
    for poly in list(root):
        if poly.get('id','').startswith('SW_lower_approach_'):root.remove(poly)
    ET.SubElement(root,'poly',id='SW_lower_approach_stop',color='255,255,255',
                  fill='0',lineWidth='.45',layer='6',shape=_write([median,tip]))
    shapes=_read_lane_shapes(net_path)
    for lane,kind in enumerate(('right','straight','straight','straight_left','straight_left','left')):
        path=shapes[('SW_new_lower_in',lane)]
        marks=[]
        _arrow(marks,f'SW_lower_approach_arrow_{lane}_{kind}',shapes,
               'SW_new_lower_in',lane,kind,stop_station+8.0-station(path[-1]))
        for mark in marks:root.append(ET.fromstring(mark))


def add_upper_approach_stop_and_arrows(root,net_path):
    """Follow the sketched stop line from the splitter tip to the median."""
    import math
    from env.roundabout_builder import _read_lane_shapes,_arrow
    from env.roundabout_geometry import _read
    net=ET.parse(net_path).getroot()
    junction=net.find("junction[@id='SW_new_cross']")
    upper=net.find("junction[@id='SW_new_upper']")
    center=tuple(float(junction.get(k)) for k in ('x','y'))
    endpoint=tuple(float(upper.get(k)) for k in ('x','y'))
    direction=tuple((endpoint[j]-center[j])/math.dist(center,endpoint) for j in (0,1))
    right=(direction[1],-direction[0])
    def station(q):return sum((q[j]-center[j])*direction[j] for j in (0,1))
    def lateral(q):return sum((q[j]-center[j])*right[j] for j in (0,1))
    island=root.find("poly[@id='SW_road_first_island_border_0']")
    tip=max(_read(island.get('shape')),key=station)
    tip_station,tip_offset=station(tip),lateral(tip)
    # Both ends share one road station, perpendicular to the approach axis.
    median_station=tip_station
    median=tuple(center[j]+direction[j]*median_station for j in (0,1))
    for poly in list(root):
        if poly.get('id','').startswith('SW_upper_approach_'):root.remove(poly)
    ET.SubElement(root,'poly',id='SW_upper_approach_stop',color='255,255,255',
                  fill='0',lineWidth='.45',layer='6',shape=_write([tip,median]))
    shapes=_read_lane_shapes(net_path)
    for lane,kind in enumerate(('right','straight','straight','straight','straight_left','left')):
        path=shapes[('SW_new_upper_in',lane)]
        offset=lateral(path[-1])
        stop_station=median_station+(tip_station-median_station)*offset/tip_offset
        distance=stop_station+8.25-station(path[-1])
        marks=[]
        _arrow(marks,f'SW_upper_approach_arrow_{lane}_{kind}',shapes,
               'SW_new_upper_in',lane,kind,distance)
        for mark in marks:root.append(ET.fromstring(mark))


def add_east_approach_arrows(root,net_path):
    """Align six direction arrows five metres behind the actual stop line."""
    import math
    from env.roundabout_builder import _read_lane_shapes,_arrow
    from env.roundabout_geometry import _read
    stop=root.find("poly[@id='SW_new_east_stop']")
    anchor=_read(stop.get('shape'))[0]
    shapes=_read_lane_shapes(net_path)
    for poly in list(root):
        if poly.get('id','').startswith('SW_east_approach_arrow_'):root.remove(poly)
    for lane,kind in enumerate(('right','straight','straight','straight','straight_left','left')):
        path=shapes[('SW_end_T_extension_in',lane)]
        before,end=path[-2:]
        forward=tuple((end[j]-before[j])/math.dist(before,end) for j in (0,1))
        distance=5.0+sum((end[j]-anchor[j])*forward[j] for j in (0,1))
        marks=[]
        _arrow(marks,f'SW_east_approach_arrow_{lane}_{kind}',shapes,
               'SW_end_T_extension_in',lane,kind,distance)
        for mark in marks:root.append(ET.fromstring(mark))


def add_east_outer_lead_dash(root,net_path):
    """Complete the outside divider's last dash alongside the stop line."""
    from env.roundabout_builder import _read_lane_shapes
    from env.roundabout_geometry import _read
    reference=root.find("poly[@id='lane_dash_SW_end_T_extension_in_1_5']")
    if reference is None:return
    shapes=_read_lane_shapes(net_path)
    outer=shapes[('SW_end_T_extension_in',0)][0]
    adjacent=shapes[('SW_end_T_extension_in',1)][0]
    shift=tuple(outer[j]-adjacent[j] for j in (0,1))
    for poly in list(root):
        if poly.get('id')=='SW_new_east_outer_lead_dash':root.remove(poly)
    attrs=dict(reference.attrib)
    attrs.update(id='SW_new_east_outer_lead_dash',
                 shape=_write([tuple(q[j]+shift[j] for j in (0,1)) for q in _read(reference.get('shape'))]))
    ET.SubElement(root,'poly',attrs)


def clear_east_stop_dash_remnants(root):
    from env.roundabout_geometry import _read
    import math
    stop=root.find("poly[@id='SW_new_east_stop']")
    if stop is None:return
    point,_,_,_=layout();o=point(0,0);along=tuple(point(1,0)[j]-o[j] for j in (0,1))
    def station(q):return sum((q[j]-o[j])*along[j] for j in (0,1))
    position=station(_read(stop.get('shape'))[0])
    removed=[]
    for p in list(root):
        if 'lane_dash_SW_end_T_extension_in_' not in p.get('id',''):continue
        coords=_read(p.get('shape',''))
        if len(coords)<2:continue
        stations=[station(q) for q in coords]
        high=max(stations)
        length=sum(math.dist(a,b) for a,b in zip(coords,coords[1:]))
        # Clear the four divider tails and the short remaining fragment inside
        # the crossing, as well as the old fragments around the stop line.
        if (position-7.1<=high<=position+2.2
                or position-16<=high<=position-12
                or (length<1.2 and (position-12<=high<=position-8 or position-20<=high<=position-18))):
            removed.append(p.get('id'));root.remove(p)
    return removed
