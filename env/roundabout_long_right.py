"""Long outer right-turn lane following the user's drawn road boundary."""
import math
import xml.etree.ElementTree as ET
from env.roundabout_end_widening import layout
from env.roundabout_config import END_CROSSROADS as C
from env.roundabout_geometry import _write


def add_network(nodes,edges,connections):
    point,_,_,end=layout()
    start=end+51+C['sw_end_t_left_length']
    s=-C['sw_arm_length']-C['sw_forward_length']
    station=start-100
    xy=point(station,s)
    ET.SubElement(nodes,'node',id='SW_long_right_start',x=str(xy[0]),y=str(xy[1]),type='priority',radius='4')
    main=next(e for e in edges if e.get('id')=='SW_end_T_left_out')
    destination=main.get('to')
    main.set('to','SW_long_right_start')
    main.set('shape',_write([point(end+51,s-6.9),point(station,s-6.9)]))
    attrs=dict(main.attrib)
    attrs.update(id='SW_long_main_continue',**{'from':'SW_long_right_start','to':destination,'shape':_write([point(station,s-6.9),point(start,s-6.9)])})
    ET.SubElement(edges,'edge',attrs)
    for c in connections:
        if c.get('from')=='SW_end_T_left_out': c.set('from','SW_long_main_continue')
    for lane in range(4):
        ET.SubElement(connections,'connection',**{'from':'SW_end_T_left_out','to':'SW_long_main_continue','fromLane':str(lane),'toLane':str(lane)})
    # Open outside the four existing lanes, then parallel the main carriageway.
    curve=[]
    for i in range(61):
        u=i/60
        curve.append(point(station+25*u,s-11.7-3.2*(3*u*u-2*u**3)))
    curve.append(point(start,s-14.9))
    for i in range(1,81):
        u=i/80
        curve.append(point(start+40*u,s-14.9-3.2*(3*u*u-2*u**3)))
    curve.append(point(start+62,s-18.1))
    theta=math.radians(15)
    md=start+114-50*math.sin(theta)
    ml=s-50*math.cos(theta)
    q=point(md-24.0*math.cos(theta),ml+24.0*math.sin(theta))
    p3=(q[0],q[1]-10)
    base=point(0,0); v=point(-math.sin(theta),-math.cos(theta))
    p0=curve[-1]; p1=point(start+84,s-18.1)
    p2=tuple(p3[j]-12*(v[j]-base[j]) for j in (0,1))
    for i in range(1,101):
        u=i/100
        curve.append(tuple((1-u)**3*p0[j]+3*(1-u)**2*u*p1[j]+3*(1-u)*u*u*p2[j]+u**3*p3[j] for j in (0,1)))
    ET.SubElement(edges,'edge',id='SW_long_right',**{'from':'SW_long_right_start','to':'SW_slip_merge','numLanes':'1','width':'3.2','speed':'8.33','priority':'1','spreadType':'center','shape':_write(curve)})
    for source,target in [('SW_end_T_left_out','SW_long_right'),('SW_long_right','SW_new_lower_out')]:
        ET.SubElement(connections,'connection',**{'from':source,'to':target,'fromLane':'0','toLane':'0'})


def add_markings(root):
    point,_,_,end=layout()
    start=end+51+C['sw_end_t_left_length']
    s=-C['sw_arm_length']-C['sw_forward_length']
    # Divider lies halfway between the existing outer lane and the added
    # 3.2 m lane, following the same taper as the existing carriageway.
    from env.roundabout_markings import dash_style
    dash,gap,_=dash_style(20)
    def boundary(station):
        if station < start-75:
            u=max(0.0,min(1.0,(station-(start-100))/25))
            return point(station,s-11.7-1.6*(3*u*u-2*u**3))
        u=max(0.0,min(1.0,(station-start)/40))
        return point(station,s-13.3-3.2*(3*u*u-2*u**3))
    # 2026-11-12 사용자 요청: 차로가 덜 벌어진 테이퍼 안 점선 4개를 뺀다 (차로가 온전한 폭이 되는 start-75 부터)
    station=start-75
    index=0
    while station+dash<=start+62:
        shape=[boundary(station+dash*i/8) for i in range(9)]
        ET.SubElement(root,'poly',id=f'SW_long_right_divider_{index}',color='255,255,255',fill='0',lineWidth='0.15',layer='6',shape=_write(shape))
        station+=dash+gap
        index+=1


def add_entry_stop_and_island(root,net_path):
    """Fit the user's stop line and safety paint to actual road boundaries."""
    from shapely.geometry import LineString,Polygon
    from shapely.ops import unary_union
    from env.roundabout_geometry import _read,_offset,_slice,_length
    from env.roundabout_surfaces import _network,_polygons,_lines
    for poly in list(root):
        if poly.get('id','').startswith('SW_long_safety_'): root.remove(poly)
    point,_,_,end=layout()
    start=end+51+C['sw_end_t_left_length']
    s=-C['sw_arm_length']-C['sw_forward_length']
    net=ET.parse(net_path).getroot()
    lane=net.find("edge[@id='SW_long_right']/lane")
    path=_read(lane.get('shape'))
    line=LineString(path)
    from shapely.geometry import Point
    station=line.project(Point(point(start+62,s-18.1)))
    # Share the exact station and transverse direction of the main stop line.
    plane=LineString([point(start+68,s+5),point(start+68,s-70)])
    outer=LineString(_offset(path,-1.6)).intersection(plane)
    if outer.geom_type=='MultiPoint': outer=min(outer.geoms,key=lambda q:q.distance(Point(point(start+68,s-18.1))))
    if outer.geom_type!='Point':raise ValueError('Slip stop plane does not intersect the outer lane boundary')
    anchor=point(start+68,s-16.5)
    midpoint=((anchor[0]+outer.x)/2,(anchor[1]+outer.y)/2)
    at=line.project(Point(midpoint))
    before=line.interpolate(max(0,at-.2));after=line.interpolate(min(line.length,at+.2))
    dx,dy=after.x-before.x,after.y-before.y
    norm=math.hypot(dx,dy)
    normal=(-dy/norm,dx/norm)
    stop=[tuple(midpoint[j]+sign*1.6*normal[j] for j in (0,1)) for sign in (-1,1)]
    # Right-turn stop line removed at user request.
    inner=_offset(_slice(path,station,_length(path)),1.6)
    # Intersection of the main road outer edge and widened cross-road edge.
    base=point(0,0);along=tuple(point(1,0)[j]-base[j] for j in (0,1));across=tuple(point(0,1)[j]-base[j] for j in (0,1))
    theta=math.radians(15)
    shift=-10*along[1]+10*across[1]*math.tan(theta)-12.8/math.cos(theta)
    corner=point(start+114+shift-16.5*math.tan(theta)-16/math.cos(theta),s-16.5)
    candidate=Polygon(inner+[corner]).buffer(0)
    _,_,strips=_network(net_path)
    boundary_edges=('SW_long_right_', 'SW_slip_main_', 'SW_new_lower_near_', 'SW_new_lower_out_', 'SW_end_T_extension_out_')
    road=unary_union([strip for name,strip in strips.items() if name.startswith(boundary_edges)])
    parts=list(_polygons(candidate.difference(road.buffer(.15))))
    if not parts:return
    island=max(parts,key=lambda p:p.area)
    # The user-marked island must have no white lane strokes through its fill.
    for poly in list(root):
        if poly.get('color')!='255,255,255' or poly.get('fill')!='0' or poly.get('id')=='SW_long_safety_stop':
            continue
        coords=_read(poly.get('shape',''))
        if len(coords)<2:continue
        stroke=LineString(coords)
        if not stroke.intersects(island):continue
        root.remove(poly)
        for index,piece in enumerate(_lines(stroke.difference(island.buffer(.05)))):
            if piece.length<.05:continue
            attrs=dict(poly.attrib)
            attrs.update(id=f"cropped_safety_{poly.get('id')}_{index}",shape=_write(list(piece.coords)))
            ET.SubElement(root,'poly',attrs)
    ET.SubElement(root,'poly',id='SW_long_safety_fill',color='84,84,90',fill='1',layer='5',shape=_write(list(island.exterior.coords)))
    ET.SubElement(root,'poly',id='SW_long_safety_border',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(island.exterior.coords)))
    for i in range(45,115,4):
        stripe=LineString([point(start+i,s-15),point(start+i+35,s-80)])
        for k,piece in enumerate(_lines(stripe.intersection(island.buffer(-.2)))):
            if piece.length>.2:
                ET.SubElement(root,'poly',id=f'SW_long_safety_hatch_{i}_{k}',color='235,190,40',fill='0',lineWidth='0.2',layer='6',shape=_write(list(piece.coords)))


def add_six_approach_arrows(root,net_path):
    """One aligned row: left, straight-left, straight, straight, straight, right."""
    from shapely.geometry import LineString
    from env.roundabout_builder import _read_lane_shapes,_arrow
    from env.roundabout_geometry import _read
    for poly in list(root):
        if poly.get('id','').startswith('SW_six_approach_arrow_'):root.remove(poly)
    shapes=_read_lane_shapes(net_path)
    point,_,_,end=layout()
    start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    base=point(0,0);forward=tuple(point(1,0)[j]-base[j] for j in (0,1));normal=(-forward[1],forward[0])
    # All six arrows share one transverse plane, ten metres before the stop.
    plane=LineString([point(start+58,s+2),point(start+58,s-30)])
    origins=[]
    for order,lane,kind in [(0,4,'left'),(1,3,'straight_left'),(2,2,'straight'),(3,1,'straight'),(4,0,'straight'),(5,0,'right')]:
        if order<5:
            path=shapes[('SW_end_T_extension_out',lane)]+shapes[('SW_slip_main',lane)]
        else:path=shapes[('SW_long_right',0)]
        intersection=LineString(path).intersection(plane)
        if intersection.geom_type!='Point':raise ValueError('Cannot align arrow to its actual lane')
        origin=(intersection.x,intersection.y);origins.append(origin)
        virtual={('row',0):[tuple(origin[j]-10*forward[j] for j in (0,1)),tuple(origin[j]+10*forward[j] for j in (0,1))]}
        marks=[]
        _arrow(marks,f'SW_six_approach_arrow_{order}_{kind}',virtual,'row',0,kind,10)
        for mark in marks:
            poly=ET.fromstring(mark)
            if kind in ('left','right') or (kind == 'straight_left' and poly.get('id','').endswith('_left')):
                # Keep the turn head comfortably inside a 3.2 m lane.
                fitted=[]
                for q in _read(poly.get('shape')):
                    delta=tuple(q[j]-origin[j] for j in (0,1))
                    along=sum(delta[j]*forward[j] for j in (0,1));across=sum(delta[j]*normal[j] for j in (0,1))*.65
                    fitted.append(tuple(origin[j]+along*forward[j]+across*normal[j] for j in (0,1)))
                poly.set('shape',_write(fitted))
            root.append(poly)
    return origins
