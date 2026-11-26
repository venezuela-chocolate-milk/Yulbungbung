"""Skew four-way intersection at the 135 m northeast endpoint."""
import math
import xml.etree.ElementTree as ET
from env.roundabout_geometry import _read, _write, _offset


def _round_marking_corners(points,cutback=3.0):
    """Round only paint corners while preserving the adjoining straight runs."""
    result=[points[0]]
    for a,b,c in zip(points,points[1:],points[2:]):
        before,after=math.dist(a,b),math.dist(b,c)
        cut=min(cutback,before*0.3,after*0.3)
        entry=tuple(b[j]+(a[j]-b[j])*cut/before for j in (0,1))
        leave=tuple(b[j]+(c[j]-b[j])*cut/after for j in (0,1))
        result.append(entry)
        for k in range(1,25):
            t=k/24
            result.append(tuple((1-t)**2*entry[j]+2*(1-t)*t*b[j]+t*t*leave[j] for j in (0,1)))
    result.append(points[-1])
    return result


def center_station():
    from env.roundabout_config import T_JUNCTION, END_T_JUNCTION
    return T_JUNCTION['distance']+T_JUNCTION['main_extension']+END_T_JUNCTION['main_extension']


def add_network(nodes, edges, connections):
    from env.roundabout_config import T_JUNCTION, END_CROSSROADS
    if not END_CROSSROADS:
        return
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    n=-u[1],u[0]
    cross=math.radians(END_CROSSROADS['cross_axis'])
    v=math.cos(cross),math.sin(cross)
    vn=-v[1],v[0]
    center=u[0]*center_station(),u[1]*center_station()
    node=next(p for p in nodes if p.get('id')=='T_end_main_end')
    node.set('type','traffic_light');node.set('radius','12.0')
    length=END_CROSSROADS['arm_length']
    approach=next(e for e in edges if e.get('id')=='T_stop_main_out')
    reference=_read(approach.get('shape'))[-1]
    reference_lateral=(reference[0]-center[0])*n[0]+(reference[1]-center[1])*n[1]
    # Anchor the right arm to the two incoming through lanes, so their divider
    # and inner boundary continue along straight, parallel lines.
    main_median=reference_lateral+2.1
    def point(axis,normal,d,lateral):
        return (center[0]+axis[0]*d+normal[0]*lateral,
                center[1]+axis[1]*d+normal[1]*lateral)
    for label,axis,normal,out_lanes,in_lanes in [
            ('main',u,n,2,3), ('nw',v,vn,2,4),
            ('se',(-v[0],-v[1]),(-vn[0],-vn[1]),2,3)]:
        length=END_CROSSROADS.get(f'{label}_arm_length',END_CROSSROADS['arm_length'])
        end=point(axis,normal,length,0)
        ET.SubElement(nodes,'node',id=f'X_{label}_end',x=str(end[0]),y=str(end[1]),type='dead_end')
        for suffix,lanes,reverse in [('out',out_lanes,False),('in',in_lanes,True)]:
            lateral=(lanes*1.6+0.5)*(1 if reverse else -1)
            if label=='main':
                # Transfer one lane across the median while keeping both
                # outside road edges at their existing positions.
                lateral+=main_median
            points=[point(axis,normal,0,lateral),point(axis,normal,length,lateral)]
            fr,to='T_end_main_end',f'X_{label}_end'
            if reverse:
                fr,to=to,fr;points.reverse()
            ET.SubElement(edges,'edge',id=f'X_{label}_{suffix}',**{'from':fr,'to':to,
                          'numLanes':str(lanes),'width':'3.2','speed':'13.89',
                          'priority':'3' if label=='main' else '2',
                          'spreadType':'center','shape':_write(points)})
    moves=[('T_stop_main_out','X_main_out',[(0,0),(1,1)]),
           ('T_stop_main_out','X_se_out',[(0,0)]),
           ('T_stop_main_out','X_nw_out',[(2,1)]),
           ('X_main_in','T_stop_main_in',[(0,0),(1,1)]),
           ('X_main_in','X_nw_out',[(0,0)]),('X_main_in','X_se_out',[(2,1)]),
           ('X_nw_in','X_se_out',[(1,0),(2,1)]),
           ('X_nw_in','T_stop_main_in',[(0,0)]),('X_nw_in','X_main_out',[(3,1)]),
           ('X_se_in','X_nw_out',[(0,0),(1,1)]),
           ('X_se_in','X_main_out',[(0,0)]),('X_se_in','T_stop_main_in',[(2,1)])]
    for fr,to,pairs in moves:
        for fl,tl in pairs:
            ET.SubElement(connections,'connection',**{'from':fr,'to':to,
                          'fromLane':str(fl),'toLane':str(tl)})
    from env.roundabout_end_widening import add_network as widen_end
    widen_end(nodes,edges,connections)
    from env.roundabout_end_triangle import add_network as add_end_triangle
    add_end_triangle(nodes,edges,connections)
    from env.roundabout_sw_outer_lane import add_network as add_SW_outer_lane
    add_SW_outer_lane(nodes,edges,connections)
    from env.roundabout_sw_crossroads import add_network as add_SW_crossroads
    add_SW_crossroads(nodes,edges,connections)
    from env.roundabout_sw_end_t import add_network as add_SW_end_T
    add_SW_end_T(nodes,edges,connections)


def add_routes(root):
    for flow in root.findall('flow'):
        if flow.get('from')=='T_stop_main_in':flow.set('from','X_main_in')
        if flow.get('to')=='T_stop_main_out':flow.set('to','X_main_out')
    for i,(fr,to) in enumerate([('X_nw_in','W_out0_far'),('X_se_in','W_out0_far'),
                                ('E_out0','X_nw_out'),('E_out0','X_se_out'),
                                ('X_nw_in','X_se_out'),('X_se_in','X_nw_out')]):
        ET.SubElement(root,'flow',id=f'end_crossroads_{i}',type='car',
                      **{'from':fr,'to':to,'begin':'0','end':'100000','vehsPerHour':'60',
                         'departLane':'best','departSpeed':'max'})
    for flow in root.findall('flow'):
        if flow.get('to')=='X_se_out':
            flow.set('to','X_se_out_far')
    from env.roundabout_end_triangle import add_routes as route_end_triangle
    route_end_triangle(root)
    from env.roundabout_sw_outer_lane import add_routes as route_SW_outer_lane
    route_SW_outer_lane(root)
    from env.roundabout_sw_crossroads import add_routes as route_SW_crossroads
    route_SW_crossroads(root)
    from env.roundabout_sw_end_t import add_routes as route_SW_end_T
    route_SW_end_T(root)


def add_markings(root,net_path):
    from env.roundabout_config import T_JUNCTION
    from env.roundabout_builder import _read_lane_shapes,_arrow,_stop_line,_center_line,_point_from_end
    from env.roundabout_surfaces import _network,_polygons
    from shapely.geometry import Polygon,LineString
    from shapely.ops import unary_union
    net=ET.parse(net_path).getroot()
    shapes=_read_lane_shapes(net_path)
    lines=[]
    for edge,kinds in [('T_stop_main_out',['straight_right','straight','left']),
                        ('X_main_in',['straight_right','straight','left']),
                        ('X_nw_in',['right','straight','straight','left']),
                        ('X_se_in',['straight_right','straight','left'])]:
        _stop_line(lines,f'x_stop_{edge}',shapes,edge,dist=1.5)
        for lane,kind in enumerate(kinds):
            _arrow(lines,f'x_arrow_{edge}_{lane}',shapes,edge,lane,kind,12.0)
    for label,out_count,in_count in [('main',2,3),('nw',2,4),('se',2,3)]:
        a=_offset(shapes[(f'X_{label}_out',out_count-1)],1.6)
        b=list(reversed(_offset(shapes[(f'X_{label}_in',in_count-1)],1.6)))
        center=[((p[0]+q[0])/2,(p[1]+q[1])/2) for p,q in zip(a,b)]
        if label=='se':
            from env.roundabout_end_widening import layout
            end_point,_,_,end_length=layout()
            center=[center[0],end_point(end_length,0)]
        lines.append(f'<poly id="x_median_{label}" color="84,84,90" fill="0" '
                     f'lineWidth="1.0" layer="5" shape="{_write(center)}"/>')
        if label=='nw':
            # Paint-only edit: follow the user's three-segment sketch without
            # changing road boundaries, lane shapes, lane counts or routing.
            from env.roundabout_config import END_CROSSROADS
            angle=math.radians(END_CROSSROADS['cross_axis'])
            axis=math.cos(angle),math.sin(angle)
            normal=-axis[1],axis[0]
            main_angle=math.radians(T_JUNCTION['axis'])
            origin=(center_station()*math.cos(main_angle),center_station()*math.sin(main_angle))
            start=(center[0][0]-origin[0])*axis[0]+(center[0][1]-origin[1])*axis[1]
            points=[(origin[0]+axis[0]*d+normal[0]*offset,
                     origin[1]+axis[1]*d+normal[1]*offset)
                    for d,offset in [(start,0.0),(49.0,0.0),(61.0,3.9),
                                     (END_CROSSROADS.get('nw_arm_length',100.0),3.9)]]
            _center_line(lines,f'x_center_{label}',_round_marking_corners(points))
        else:
            _center_line(lines,f'x_center_{label}',center)
    node=next(n for n in net.findall('junction') if n.get('id')=='T_end_main_end')
    core=Polygon(_read(node.get('shape'))).buffer(0)
    _,_,strips=_network(net_path)
    local=unary_union(list(strips.values())).intersection(core.buffer(20))
    pavement=core.union(local).buffer(4,quad_segs=24).buffer(-4,quad_segs=24).simplify(.005)
    for i,p in enumerate(_polygons(pavement)):
        ET.SubElement(root,'poly',id=f'x_junction_surface_{i}',color='84,84,90',fill='1',
                      layer='4.8',shape=_write(list(p.exterior.coords)))
    # Only the new intersection core is cleared; the established markings stay.
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    for p in list(root):
        name=p.get('id','')
        if name.startswith('t_aligned_exit_inner_'):
            if min(x*u[0]+y*u[1] for x,y in _read(p.get('shape'))) > center_station():
                root.remove(p)
                continue
        if name.startswith(('lane_dash_','t_user_guide_','t_aligned_exit_')):
            # Keep this straight divider's original dash phase across the core.
            if not name.startswith('t_aligned_exit_opposite_') and LineString(_read(p.get('shape'))).intersects(core):
                root.remove(p)
                continue
    # The user-drawn pair: one inner boundary and one through-lane divider.
    # Both are straight and keep exactly one lane width between them.
    from env.roundabout_markings import dash_style
    start,_=_point_from_end(shapes[('T_stop_main_out',0)],1.5)
    finish,_=_point_from_end(shapes[('X_main_in',0)],1.5)
    start_station=start[0]*u[0]+start[1]*u[1]
    end_station=finish[0]*u[0]+finish[1]*u[1]
    n=-u[1],u[0]
    road_end=shapes[('X_main_out',0)][-1]
    far_station=road_end[0]*u[0]+road_end[1]*u[1]
    for p in list(root):
        if not p.get('id','').startswith(('lane_dash_','t_user_guide_','t_aligned_exit_')):
            continue
        points=_read(p.get('shape'))
        lateral=sum(x*n[0]+y*n[1] for x,y in points)/len(points)
        if abs(lateral+6.9)<.01 and min(x*u[0]+y*u[1] for x,y in points)>center_station():
            root.remove(p)
    dash,gap,_=dash_style(20.0)
    for label,lateral,stop in [('inner',-3.7,end_station),('outer',-6.9,far_station)]:
        count=max(0,math.floor((stop-start_station-dash)/(dash+gap))+1)
        for i in range(count):
            a=start_station+i*(dash+gap)
            points=[(u[0]*s+n[0]*lateral,u[1]*s+n[1]*lateral) for s in (a,a+dash)]
            ET.SubElement(root,'poly',id=f'x_straight_guide_{label}_{i}',color='255,255,255',
                          fill='0',lineWidth='0.15',layer='6',shape=_write(points))
    for line in lines:root.append(ET.fromstring(line))
    yellow=unary_union([LineString(_read(p.get('shape'))).buffer(
        float(p.get('lineWidth','0.15'))/2+0.10,cap_style=2)
        for p in root if p.get('id','').startswith('x_center_nw_')])
    for p in list(root):
        if not p.get('id','').startswith(('lane_dash_','t_user_guide_','t_aligned_exit_','x_straight_guide_')):
            continue
        white=LineString(_read(p.get('shape'))).buffer(
            float(p.get('lineWidth','0.15'))/2,cap_style=2)
        if white.intersects(yellow):
            root.remove(p)
    # The user's added divider follows the original straight median alignment
    # from the outer end to the start of the shifted yellow median.
    from env.roundabout_config import END_CROSSROADS
    angle=math.radians(END_CROSSROADS['cross_axis'])
    axis=math.cos(angle),math.sin(angle)
    main_angle=math.radians(T_JUNCTION['axis'])
    origin=(center_station()*math.cos(main_angle),center_station()*math.sin(main_angle))
    # Continue the two user-sketched cross-road guides from the existing
    # northwest dash phase, including the southeast divider beyond the core.
    normal=-axis[1],axis[0]
    def cross_point(station,lateral):
        return (origin[0]+axis[0]*station+normal[0]*lateral,
                origin[1]+axis[1]*station+normal[1]*lateral)
    nw_stop,_=_point_from_end(shapes[('X_nw_in',0)],1.5)
    se_stop,_=_point_from_end(shapes[('X_se_in',0)],1.5)
    def cross_station(point):
        return sum((point[j]-origin[j])*axis[j] for j in (0,1))
    nw_station,se_station=cross_station(nw_stop),cross_station(se_stop)
    from env.roundabout_geometry import _slice
    for lane,target_lateral in [(1,3.7),(2,0.5)]:
        prefix=f'lane_dash_X_nw_in_{lane}_'
        existing=[p for p in root if p.get('id','').startswith(prefix)]
        approach=[p for p in existing
                  if min(cross_station(q) for q in _read(p.get('shape')))>nw_station]
        anchor=min(approach,key=lambda p:min(cross_station(q) for q in _read(p.get('shape'))))
        anchor_points=_read(anchor.get('shape'))
        anchor_end=min(anchor_points,key=cross_station)
        anchor_index=int(anchor.get('id').rsplit('_',1)[1])
        for p in existing:
            if int(p.get('id').rsplit('_',1)[1])>anchor_index:
                root.remove(p)
        source_lateral=6.9 if lane==1 else 3.7
        path=[anchor_end,cross_point(nw_station,source_lateral),
              cross_point(se_station,target_lateral)]
        if lane==1:
            # Follow the widened through-lane divider instead of cutting
            # straight across the safety zone at the remote endpoint.
            boundary=_offset(shapes[('X_se_out',0)],1.6)
            path.extend(boundary[1:])
            if ('X_se_out_far',0) in shapes:
                path.extend(_offset(shapes[('X_se_out_far',0)],1.6))
            if ('SE_triangle_main_out',0) in shapes:
                boundary=_offset(shapes[('SE_triangle_main_out',0)],1.6)
                stop,_=_point_from_end(boundary,7.0)
                path.extend(boundary[:-1]+[stop])
        length=LineString(path).length
        dash,gap,_=dash_style(20.0)
        for i in range(max(0,math.floor((length-gap-dash)/(dash+gap))+1)):
            start=gap+i*(dash+gap)
            # Junction guides must not extend back across the approach stop.
            end=start+dash
            start=max(start,math.dist(anchor_end,cross_point(nw_station,source_lateral))+0.3)
            if end-start<0.05:
                continue
            ET.SubElement(root,'poly',id=f'x_cross_guide_{lane}_{i}',color='255,255,255',
                          fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,end)))
    # The opposite pair follows the newly sketched straight crossing paths.
    for lane,source_lateral,target_lateral in [(0,-6.9,-3.7),(1,-3.7,0.0)]:
        prefix=f'lane_dash_X_se_in_{lane}_'
        existing=[p for p in root if p.get('id','').startswith(prefix)]
        if not existing:
            prefix=f'lane_dash_SE_triangle_main_in_{lane}_'
            existing=[p for p in root if p.get('id','').startswith(prefix)]
        if not existing:
            prefix=f'lane_dash_SE_triangle_forward_in_0_'
            existing=[p for p in root if p.get('id','').startswith(prefix)]
        approach=[p for p in existing
                  if max(cross_station(q) for q in _read(p.get('shape')))<se_station]
        anchor=max(approach,key=lambda p:max(cross_station(q) for q in _read(p.get('shape'))))
        anchor_end=max(_read(anchor.get('shape')),key=cross_station)
        anchor_index=int(anchor.get('id').rsplit('_',1)[1])
        for p in existing:
            if int(p.get('id').rsplit('_',1)[1])>anchor_index:
                root.remove(p)
        target_station=(cross_station(shapes[('X_nw_in',0)][-1])
                        if lane==1 else nw_station)
        path=[anchor_end,cross_point(se_station,source_lateral),
              cross_point(target_station,target_lateral)]
        if lane==0:
            path.append(cross_point(cross_station(shapes[('X_nw_out',0)][-1]),target_lateral))
        length=LineString(path).length
        dash,gap,_=dash_style(20.0)
        for i in range(max(0,math.floor((length-gap-dash)/(dash+gap))+1)):
            # Anchor the median guide at the yellow centerline's exact end.
            start=(length-dash-i*(dash+gap)) if lane==1 else gap+i*(dash+gap)
            end=start+dash
            if lane==1:
                start=max(start,math.dist(anchor_end,cross_point(se_station,source_lateral))+0.3)
                if end-start<0.05:
                    continue
            ET.SubElement(root,'poly',id=f'x_cross_opposite_guide_{lane}_{i}',color='255,255,255',
                          fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,end)))
    dash,gap,_=dash_style(20.0)
    length=END_CROSSROADS.get('nw_arm_length',100.0)
    first=math.ceil((100.0-length)/(dash+gap))
    for i in range(first,math.floor((100.0-49.0-dash)/(dash+gap))+1):
        end=100.0-i*(dash+gap)
        points=[(origin[0]+axis[0]*d,origin[1]+axis[1]*d) for d in (end,end-dash)]
        ET.SubElement(root,'poly',id=f'x_nw_sketched_divider_{i}',color='255,255,255',
                      fill='0',lineWidth='0.15',layer='6',shape=_write(points))
    # Keep the stop line at its established cross-section.
    normal=-axis[1],axis[0]
    points=[(origin[0]+axis[0]*90+normal[0]*lateral,
             origin[1]+axis[1]*90+normal[1]*lateral) for lateral in (-6.9,3.675)]
    ET.SubElement(root,'poly',id='x_nw_outer_stop',color='255,255,255',fill='0',
                  lineWidth='0.45',layer='6',shape=_write(points))
    # Mark the three displayed lanes above the shifted yellow median.
    paint_shapes={('nw_outer_markings',i):[(origin[0]+axis[0]*d+normal[0]*offset,
                                          origin[1]+axis[1]*d+normal[1]*offset)
                                         for d in (0.0,100.0)]
                  for i,offset in enumerate((-5.3,-1.85,1.95))}
    arrows=[]
    for lane,kind in enumerate(('straight','straight','straight')):
        _arrow(arrows,f'x_nw_outer_arrow_{lane}',paint_shapes,'nw_outer_markings',lane,kind,14.1)
    for arrow in arrows:root.append(ET.fromstring(arrow))
    # Guide the inside boundary of each actual left-turn lane through the core.
    from env.roundabout_builder import _movement_path
    approaches={'T_stop_main_out','X_main_in','X_nw_in','X_se_in'}
    turns=[c for c in net.findall('connection')
           if c.get('from') in approaches and c.get('dir')=='l']
    dash,gap,_=dash_style(20.0)
    for turn in turns:
        source,target=turn.get('from'),turn.get('to')
        lane=int(turn.get('fromLane'))
        movement=_movement_path(net_path,source,target,lane)
        path=_offset(movement,1.6)
        length=LineString(path).length
        count=max(0,math.floor((length-dash)/(dash+gap))+1)
        margin=(length-(count*dash+max(0,count-1)*gap))/2
        for i in range(count):
            start=margin+i*(dash+gap)
            ET.SubElement(root,'poly',id=f'x_left_turn_guide_{source}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,start+dash)))
    from env.roundabout_end_widening import add_markings as mark_widened_end
    mark_widened_end(root)
    from env.roundabout_end_triangle import add_markings as mark_end_triangle
    mark_end_triangle(root,net_path)
    from env.roundabout_sw_outer_lane import add_markings as mark_SW_outer_lane
    mark_SW_outer_lane(root,net_path)
    from env.roundabout_sw_crossroads import add_markings as mark_SW_crossroads
    mark_SW_crossroads(root,net_path)
    from env.roundabout_sw_crossroads import clear_marked_area as clear_SW_core
    clear_SW_core(root)
    from env.roundabout_sw_crossroads import add_median_left_guides as guide_SW_medians
    guide_SW_medians(root,net_path)
    from env.roundabout_sw_end_t import add_markings as mark_SW_end_T
    mark_SW_end_T(root,net_path)
    from env.roundabout_sw_end_t import clear_marked_dashes as clear_SW_end_T_dashes
    clear_SW_end_T_dashes(root)
    from env.roundabout_sw_end_t import clear_center_fragment as clear_SW_end_T_center
    clear_SW_end_T_center(root)
    from env.roundabout_sw_end_t import add_median_guides as guide_SW_end_T_medians
    guide_SW_end_T_medians(root,net_path)
    from env.roundabout_sw_end_t import add_through_guides as guide_SW_end_T_through
    guide_SW_end_T_through(root,net_path)
    from env.roundabout_sw_end_t import add_reverse_through_guides as guide_SW_end_T_reverse
    guide_SW_end_T_reverse(root,net_path)
