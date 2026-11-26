"""Right-angle T junction at a configured distance beyond the stop line."""
import math
import xml.etree.ElementTree as ET
from env.roundabout_geometry import _write


def station():
    from env.roundabout_config import T_JUNCTION, END_T_JUNCTION, STOP_T_JUNCTION
    return (T_JUNCTION['distance']+T_JUNCTION['main_extension']
            +END_T_JUNCTION['outgoing_stop_from_junction']+STOP_T_JUNCTION['ahead_of_stop'])


def add_network(nodes, edges, connections):
    from env.roundabout_config import T_JUNCTION, END_T_JUNCTION, STOP_T_JUNCTION
    if not STOP_T_JUNCTION:
        return
    angle = math.radians(T_JUNCTION['axis'])
    u = math.cos(angle), math.sin(angle)
    n = -u[1], u[0]
    location = station()
    far = T_JUNCTION['distance']+T_JUNCTION['main_extension']+END_T_JUNCTION['main_extension']
    def point(a, lateral):
        return u[0]*a+n[0]*lateral, u[1]*a+n[1]*lateral
    center = point(location, 0)
    def side(d, lateral):
        return center[0]-n[0]*d+u[0]*lateral, center[1]-n[1]*d+u[1]*lateral
    ET.SubElement(nodes, 'node', id='T_stop_junction', x=str(center[0]),
                  y=str(center[1]), type='priority', radius='6.0')
    end = side(STOP_T_JUNCTION['branch_length'], 0)
    ET.SubElement(nodes, 'node', id='T_stop_side_end', x=str(end[0]), y=str(end[1]), type='dead_end')
    old_out = next(e for e in edges if e.get('id') == 'T_end_main_out')
    old_in = next(e for e in edges if e.get('id') == 'T_end_main_in')
    old_start = T_JUNCTION['distance']+T_JUNCTION['main_extension']
    old_out.set('to', 'T_stop_junction')
    old_out.set('shape', _write([point(old_start, -5.3), point(location, -5.3)]))
    old_in.set('from', 'T_stop_junction')
    old_in.set('shape', _write([point(location, 3.7), point(old_start, 3.7)]))
    def edge(name, fr, to, lanes, points, priority=3):
        ET.SubElement(edges, 'edge', id=name, **{'from':fr, 'to':to, 'numLanes':str(lanes),
                      'width':'3.2', 'speed':'16.67' if priority==3 else '11.11',
                      'priority':str(priority), 'spreadType':'center', 'shape':_write(points)})
    edge('T_stop_main_out', 'T_stop_junction', 'T_end_main_end', 3,
         [point(location,-5.3), point(far,-5.3)])
    edge('T_stop_main_in', 'T_end_main_end', 'T_stop_junction', 2,
         [point(far,3.7), point(location,3.7)])
    length = STOP_T_JUNCTION['branch_length']
    edge('T_stop_side_out', 'T_stop_junction', 'T_stop_side_end', 1,
         [side(0,-2.1), side(length,-2.1)], 1)
    edge('T_stop_side_in', 'T_stop_side_end', 'T_stop_junction', 1,
         [side(length,2.1), side(0,2.1)], 1)
    for fr,to,pairs in [
            ('T_end_main_out','T_stop_main_out',[(0,0),(1,1),(2,2)]),
            ('T_end_main_out','T_stop_side_out',[(0,0)]),
            ('T_stop_main_in','T_end_main_in',[(0,0),(1,1)]),
            ('T_stop_main_in','T_stop_side_out',[(1,0)]),
            ('T_stop_side_in','T_stop_main_out',[(0,0)]),
            ('T_stop_side_in','T_end_main_in',[(0,1)])]:
        for fl,tl in pairs:
            ET.SubElement(connections,'connection',**{'from':fr,'to':to,
                          'fromLane':str(fl),'toLane':str(tl)})


def add_routes(root):
    for flow in root.findall('flow'):
        if flow.get('from')=='T_end_main_in':
            flow.set('from','T_stop_main_in')
        if flow.get('to')=='T_end_main_out':
            flow.set('to','T_stop_main_out')
    for i,(fr,to) in enumerate([('T_stop_side_in','W_out0_far'),
                                ('E_out0','T_stop_side_out'),
                                ('T_stop_main_in','T_stop_side_out'),
                                ('T_stop_side_in','T_stop_main_out')]):
        ET.SubElement(root,'flow',id=f'stop_tjunction_{i}',type='car',
                      **{'from':fr,'to':to,'begin':'0','end':'100000','vehsPerHour':'45',
                         'departLane':'best','departSpeed':'max'})


def clear_markings(root):
    """Remove only stop/center lines inside the user-marked junction area."""
    from env.roundabout_config import T_JUNCTION, STOP_T_JUNCTION, END_T_JUNCTION
    from env.roundabout_geometry import _read
    from env.roundabout_surfaces import _lines
    from shapely.geometry import LineString, MultiPoint
    if not STOP_T_JUNCTION.get('clear_junction_markings'):
        return
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    n=-u[1],u[0]
    junction=station()
    stop=(T_JUNCTION['distance']+T_JUNCTION['main_extension']
          +END_T_JUNCTION['outgoing_stop_from_junction'])
    def point(a,lateral):
        return u[0]*a+n[0]*lateral,u[1]*a+n[1]*lateral
    far_boundary=junction+STOP_T_JUNCTION.get('main_clear_after',15.0)
    region=MultiPoint([point(a,lateral) for a in (stop,far_boundary)
                       for lateral in (1.0,-10.5)]
                      +[point(junction+side,-15.5) for side in (-6.9,6.9)]).convex_hull
    for element in list(root):
        name=element.get('id','')
        if name=='t_stop_end_main_out':
            if not STOP_T_JUNCTION.get('keep_stop_line'):
                root.remove(element)
        elif STOP_T_JUNCTION.get('clear_opposite_divider') and name.startswith('t_user_guide_opposite_divider_'):
            positions=[x*u[0]+y*u[1] for x,y in _read(element.get('shape'))]
            if min(positions)<junction+15 and max(positions)>stop:
                root.remove(element)
        elif (STOP_T_JUNCTION.get('clear_white_dashes')
              and name.startswith(('lane_dash_', 't_user_guide_'))):
            if LineString(_read(element.get('shape'))).intersects(region):
                root.remove(element)
        elif name.startswith('t_center_'):
            if STOP_T_JUNCTION.get('keep_main_centerline') and name.startswith(
                    ('t_center_end_main_', 't_center_stop_main_')):
                continue
            line=LineString(_read(element.get('shape')))
            if not line.intersects(region):
                continue
            pieces=_lines(line.difference(region))
            root.remove(element)
            for i,piece in enumerate(pieces):
                if piece.length<0.05:
                    continue
                clone=ET.Element('poly',element.attrib)
                clone.set('id',name if i==0 else name+f'_outside{i}')
                clone.set('shape',_write(list(piece.coords)))
                root.append(clone)


def align_side_dashes(root):
    """Mirror the screenshot's left divider across the side-road centerline."""
    from env.roundabout_config import T_JUNCTION, STOP_T_JUNCTION
    from env.roundabout_geometry import _read
    if not STOP_T_JUNCTION.get('symmetric_side_dashes'):
        return
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    center=u[0]*station(),u[1]*station()
    axis=u[1],-u[0]
    source_prefix='lane_dash_T_stop_side_out_0_'
    target_prefix='lane_dash_T_stop_side_in_0_'
    source=[p for p in root if p.get('id','').startswith(source_prefix)]
    if not source:
        raise ValueError('Missing reference divider on the stop-line side road')
    for p in list(root):
        if p.get('id','').startswith(target_prefix):
            root.remove(p)
    for p in source:
        mirrored=[]
        for x,y in _read(p.get('shape')):
            dx,dy=x-center[0],y-center[1]
            along=dx*axis[0]+dy*axis[1]
            mirrored.append((center[0]+2*along*axis[0]-dx,
                             center[1]+2*along*axis[1]-dy))
        clone=ET.Element('poly',p.attrib)
        clone.set('id',p.get('id').replace(source_prefix,target_prefix,1))
        clone.set('shape',_write(mirrored))
        root.append(clone)


def add_side_stop_line(root, net_path):
    """Place the side-road approach stop line at the sketched cross-section."""
    from env.roundabout_config import T_JUNCTION, STOP_T_JUNCTION
    from env.roundabout_builder import _read_lane_shapes, _stop_line, _arrow
    distance=STOP_T_JUNCTION.get('side_stop_distance')
    if distance is None:
        return
    shapes=_read_lane_shapes(net_path)
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    branch=u[1],-u[0]
    center=u[0]*station(),u[1]*station()
    end=shapes[('T_stop_side_in',0)][-1]
    end_station=(end[0]-center[0])*branch[0]+(end[1]-center[1])*branch[1]
    from env.roundabout_geometry import _read
    from env.roundabout_surfaces import _lines
    from shapely.geometry import LineString, Polygon
    def point(d,lateral):
        return (center[0]+branch[0]*d+u[0]*lateral,
                center[1]+branch[1]*d+u[1]*lateral)
    stop_width=0.45
    median_end=distance-stop_width/2.0-0.10
    trim=Polygon([point(-1,-1),point(median_end,-1),
                  point(median_end,1),point(-1,1)])
    for p in list(root):
        if not p.get('id','').startswith(('t_center_stop_side_', 't_center_stop_side_link_')):
            continue
        pieces=_lines(LineString(_read(p.get('shape'))).difference(trim))
        root.remove(p)
        for i,piece in enumerate(pieces):
            if piece.length<0.05:
                continue
            clone=ET.Element('poly',p.attrib)
            clone.set('id',p.get('id') if i==0 else p.get('id')+f'_trim{i}')
            clone.set('shape',_write(list(piece.coords)))
            root.append(clone)
    lines=[]
    _stop_line(lines,'t_stop_side_approach',shapes,'T_stop_side_in',dist=distance-end_station)
    for kind in ('right','left'):
        _arrow(lines,f't_arrow_stop_side_shared_{kind}',shapes,'T_stop_side_in',0,kind,
               dist=distance-end_station+3.0)
    for line in lines:
        element=ET.fromstring(line)
        if element.get('id')=='t_stop_side_approach':
            endpoints=_read(element.get('shape'))
            # Touch the yellow stripe's outer edge without overlapping it.
            endpoints[-1]=point(distance,0.225)
            element.set('shape',_write(endpoints))
        root.append(element)


def add_turn_guide(root):
    """Dashed right-turn guide between the two marked stop-line endpoints."""
    from env.roundabout_config import T_JUNCTION, END_T_JUNCTION, STOP_T_JUNCTION
    from env.roundabout_geometry import _length, _slice
    from env.roundabout_markings import dash_style
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    branch=u[1],-u[0]
    center=u[0]*station(),u[1]*station()
    main_stop=(T_JUNCTION['distance']+T_JUNCTION['main_extension']
               +END_T_JUNCTION['outgoing_stop_from_junction'])
    a=u[0]*main_stop,u[1]*main_stop
    side_stop=STOP_T_JUNCTION['side_stop_distance']
    b=(center[0]+branch[0]*side_stop+u[0]*0.225,
       center[1]+branch[1]*side_stop+u[1]*0.225)
    reach=math.dist(a,b)*0.5
    c=a[0]+u[0]*reach,a[1]+u[1]*reach
    d=b[0]-branch[0]*reach,b[1]-branch[1]*reach
    points=[tuple((1-t)**3*a[j]+3*(1-t)**2*t*c[j]
                  +3*(1-t)*t*t*d[j]+t**3*b[j] for j in (0,1))
            for t in (k/240 for k in range(241))]
    dash,gap,_=dash_style(20.0)
    total=_length(points)
    for i in range(max(0,math.floor((total-dash)/(dash+gap))+1)):
        piece=_slice(points,i*(dash+gap),i*(dash+gap)+dash)
        ET.SubElement(root,'poly',id=f't_stop_right_turn_guide_{i}',color='255,255,255',
                      fill='0',lineWidth='0.15',layer='6',shape=_write(piece))


def extend_exit_dashes(root):
    """Add one full dash toward the junction on each outgoing divider."""
    from env.roundabout_config import T_JUNCTION
    from env.roundabout_geometry import _read
    from env.roundabout_markings import dash_style
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    dash,gap,_=dash_style(20.0)
    for i,prefix in enumerate(('t_user_guide_main_outer_', 't_user_guide_end_inner_continuous_')):
        candidates=[p for p in root if p.get('id','').startswith(prefix)
                    and min(x*u[0]+y*u[1] for x,y in _read(p.get('shape'))) >= station()+15-0.001]
        if not candidates:
            raise ValueError('No outgoing divider dash available to extend')
        source=min(candidates,key=lambda p:min(x*u[0]+y*u[1] for x,y in _read(p.get('shape'))))
        points=[(x-u[0]*(dash+gap),y-u[1]*(dash+gap)) for x,y in _read(source.get('shape'))]
        if any(p.get('id','').startswith(prefix)
               and math.dist(_read(p.get('shape'))[0],points[0])<0.001 for p in root):
            continue
        clone=ET.Element('poly',source.attrib)
        clone.set('id',f't_stop_exit_dash_extension_{i}')
        clone.set('shape',_write(points))
        root.append(clone)


def align_exit_dividers(root):
    """Use the outgoing outer divider's phase for all three parallel dividers."""
    from env.roundabout_config import T_JUNCTION, STOP_T_JUNCTION
    from env.roundabout_geometry import _read
    if not STOP_T_JUNCTION.get('align_exit_dividers'):
        return
    angle=math.radians(T_JUNCTION['axis'])
    u=math.cos(angle),math.sin(angle)
    n=-u[1],u[0]
    start=station()+STOP_T_JUNCTION.get('main_clear_after',12.0)
    def start_station(p):
        return min(x*u[0]+y*u[1] for x,y in _read(p.get('shape')))
    source=[p for p in root if (p.get('id','').startswith('t_user_guide_main_outer_')
                               or p.get('id')=='t_stop_exit_dash_extension_0')
            and start_station(p)>=start-0.01]
    if source:
        first=min(start_station(p) for p in source)
        for p in root:
            if not p.get('id','').startswith(('t_center_end_main_','t_center_stop_main_')):
                continue
            points=[]
            for x,y in _read(p.get('shape')):
                a=x*u[0]+y*u[1]
                delta=first-a if abs(a-start)<0.01 else 0.0
                points.append((x+u[0]*delta,y+u[1]*delta))
            p.set('shape',_write(points))
    for p in list(root):
        name=p.get('id','')
        if (name.startswith(('t_user_guide_opposite_divider_','t_user_guide_end_inner_continuous_'))
                or name=='t_stop_exit_dash_extension_1') and start_station(p)>=start-0.01:
            root.remove(p)
    for i,p in enumerate(sorted(source,key=start_station)):
        points=_read(p.get('shape'))
        lateral=sum(x*n[0]+y*n[1] for x,y in points)/len(points)
        for label,target in [('inner',-3.7),('opposite',3.7)]:
            shifted=[(x+n[0]*(target-lateral),y+n[1]*(target-lateral)) for x,y in points]
            clone=ET.Element('poly',p.attrib)
            clone.set('id',f't_aligned_exit_{label}_{i}')
            clone.set('shape',_write(shifted))
            root.append(clone)
