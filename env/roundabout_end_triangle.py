"""Triangular painted island between an endpoint extension and a slip road."""
import math
import xml.etree.ElementTree as ET

from env.roundabout_geometry import _read,_write,_offset
from env.roundabout_end_widening import layout


def add_network(nodes,edges,connections):
    from env.roundabout_config import END_CROSSROADS
    sw_length=END_CROSSROADS['sw_arm_length']
    point,_,_,end=layout()
    corner=end+51
    old=next(n for n in nodes if n.get('id')=='X_se_end')
    old.set('type','priority');old.set('radius','8')
    def node(name,d,lateral,radius=6):
        xy=point(d,lateral)
        ET.SubElement(nodes,'node',id=name,x=str(xy[0]),y=str(xy[1]),
                      type='priority' if radius else 'dead_end',radius=str(radius))
    node('SE_triangle_corner',corner,0,12)
    next(n for n in nodes if n.get('id')=='SE_triangle_corner').set('type','traffic_light')
    node('SE_triangle_merge',corner,-54,8)
    node('SE_triangle_NE',corner,80,0)
    node('SE_triangle_SW',corner,-sw_length,0)
    node('SE_triangle_forward',corner+120,0,0)
    def edge(name,fr,to,lanes,pts):
        ET.SubElement(edges,'edge',id=name,**{'from':fr,'to':to,'numLanes':str(lanes),
                      'width':'3.2','speed':'8.33','priority':'2','spreadType':'center',
                      'shape':_write([point(d,lateral) for d,lateral in pts])})
    edge('SE_triangle_main_out','X_se_end','SE_triangle_corner',3,[(end,-5.3),(corner,-5.3)])
    edge('SE_triangle_main_in','SE_triangle_corner','X_se_end',3,[(corner,5.3),(end,5.3)])
    edge('SE_triangle_NE_out','SE_triangle_corner','SE_triangle_NE',1,[(corner+1.6,0),(corner+1.6,80)])
    edge('SE_triangle_NE_in','SE_triangle_NE','SE_triangle_corner',3,[(corner-4.8,80),(corner-4.8,0)])
    edge('SE_triangle_forward_out','SE_triangle_corner','SE_triangle_forward',2,[(corner,-3.7),(corner+120,-3.7)])
    edge('SE_triangle_forward_in','SE_triangle_forward','SE_triangle_corner',2,[(corner+120,3.7),(corner,3.7)])
    edge('SE_triangle_cross_out','SE_triangle_corner','SE_triangle_merge',2,[(corner-3.7,0),(corner-3.7,-54)])
    edge('SE_triangle_cross_in','SE_triangle_merge','SE_triangle_corner',3,[(corner+5.3,-54),(corner+5.3,0)])
    edge('SE_triangle_SW_out','SE_triangle_merge','SE_triangle_SW',3,[(corner-5.3,-54),(corner-5.3,-sw_length)])
    edge('SE_triangle_SW_in','SE_triangle_SW','SE_triangle_merge',3,[(corner+5.3,-sw_length),(corner+5.3,-54)])
    slip_control=[(end,-11.7),(end+6,-28),(corner-8.5,-40),(corner-8.5,-54)]
    slip_curve=[]
    for i in range(151):
        t=i/150
        slip_curve.append(tuple((1-t)**3*slip_control[0][j]
                                +3*(1-t)**2*t*slip_control[1][j]
                                +3*(1-t)*t*t*slip_control[2][j]
                                +t**3*slip_control[3][j] for j in (0,1)))
    edge('SE_triangle_slip','X_se_end','SE_triangle_merge',1,slip_curve)
    def connect(fr,to,source=0,target=0):
        ET.SubElement(connections,'connection',**{'from':fr,'to':to,
                      'fromLane':str(source),'toLane':str(target)})
    for lane in range(3):
        connect('X_se_out_far','SE_triangle_main_out',lane,lane)
        connect('SE_triangle_main_in','X_se_in',lane,lane)
    for lane in range(2):
        connect('SE_triangle_main_out','SE_triangle_forward_out',lane+1,lane)
        connect('SE_triangle_forward_in','SE_triangle_main_in',lane,lane+1)
    connect('X_se_out_far','SE_triangle_slip',0)
    connect('SE_triangle_main_out','SE_triangle_cross_out',0)
    connect('SE_triangle_main_out','SE_triangle_NE_out',2)
    connect('SE_triangle_NE_in','SE_triangle_main_in',0,0)
    connect('SE_triangle_cross_in','SE_triangle_main_in',2,2)
    connect('SE_triangle_NE_in','SE_triangle_cross_out',1)
    connect('SE_triangle_cross_in','SE_triangle_NE_out',1)
    connect('SE_triangle_forward_in','SE_triangle_NE_out',0)
    connect('SE_triangle_forward_in','SE_triangle_cross_out',1,1)
    connect('SE_triangle_NE_in','SE_triangle_forward_out',2,1)
    connect('SE_triangle_cross_in','SE_triangle_forward_out',0,0)
    for lane in range(2):
        connect('SE_triangle_cross_out','SE_triangle_SW_out',lane,lane+1)
    for lane in range(3):
        connect('SE_triangle_SW_in','SE_triangle_cross_in',lane,lane)
    connect('SE_triangle_slip','SE_triangle_SW_out',0,0)
    final_approaches={'SE_triangle_main_out','SE_triangle_forward_in',
                      'SE_triangle_NE_in','SE_triangle_cross_in'}
    for connection in connections:
        if connection.get('from') in final_approaches:
            connection.set('contPos','0')


def add_routes(root):
    incoming=0
    for flow in root.findall('flow'):
        if flow.get('from')=='X_se_in':
            flow.set('from','SE_triangle_NE_in' if incoming%2==0 else 'SE_triangle_SW_in')
            incoming+=1
        if flow.get('to')=='X_se_out_far':
            flow.set('to','SE_triangle_forward_out')
    for i,(fr,to) in enumerate([
            ('SE_triangle_forward_in','SE_triangle_main_in'),
            ('SE_triangle_NE_in','SE_triangle_forward_out'),
            ('SE_triangle_SW_in','SE_triangle_forward_out'),
            ('SE_triangle_NE_in','SE_triangle_SW_out'),
            ('SE_triangle_SW_in','SE_triangle_NE_out')]):
        ET.SubElement(root,'flow',id=f'se_final_crossroads_{i}',type='car',
                      **{'from':fr,'to':to,'begin':'0','end':'100000',
                         'vehsPerHour':'40','departLane':'best','departSpeed':'max'})


def add_markings(root,net_path):
    from shapely.geometry import Polygon,LineString,Point
    from shapely.ops import unary_union
    from env.roundabout_surfaces import _network,_polygons,_lines
    from env.roundabout_builder import _center_line,_stop_line,_arrow
    _,lanes,strips=_network(net_path)
    point,_,_,end=layout()
    corner=end+51
    # Pavement must cover the lane corridor and the spaces between merging
    # trajectories; terrain masks otherwise leave green holes between strips.
    slip_surface=strips['SE_triangle_slip_0']
    merge_strips=[strip for name,strip in strips.items()
                  if name.startswith(('SE_triangle_cross_','SE_triangle_SW_',
                                      'SE_triangle_slip_',':SE_triangle_merge_'))]
    merge_surface=unary_union(merge_strips).intersection(
        Point(point(corner,-54)).buffer(20))
    # Follow the actual single-lane slip corridor instead of paving a broad
    # hull across its curved outer edge. Close only tiny seams between strips.
    merge_surface=merge_surface.buffer(0.12,quad_segs=24).buffer(-0.12,quad_segs=24)
    cross_surface=Polygon([point(corner-6.9,-54),point(corner+10.1,-54),
                           point(corner+10.1,0),point(corner-6.9,0)])
    for label,surface in [('slip',slip_surface),('merge',merge_surface),('cross',cross_surface)]:
        for i,part in enumerate(_polygons(surface)):
            ET.SubElement(root,'poly',id=f'se_triangle_filled_{label}_{i}',color='84,84,90',
                          fill='1',layer='4.9',shape=_write(list(part.exterior.coords)))
    incoming_surface=[point(end-20,0.5),point(corner-12,0.5),
                      point(corner-12,10.1),point(end-20,10.1)]
    ET.SubElement(root,'poly',id='se_triangle_incoming_pavement',color='84,84,90',
                  fill='1',layer='4.9',shape=_write(incoming_surface))
    # Fill the full junction mouth, including gaps between individual turns.
    mouth=[point(corner-12,-10.1),point(corner+3.2,-10.1),
           point(corner+3.2,10.1),point(corner-12,10.1)]
    ET.SubElement(root,'poly',id='se_triangle_junction_pavement',color='84,84,90',
                  fill='1',layer='4.9',shape=_write(mouth))
    triangle=Polygon([point(end+0.8,-10.1),point(corner-3.2,-10.1),
                      point(corner-6.9,-54.0)])
    road=unary_union([strip for name,strip in strips.items()
                     if name.startswith(('SE_triangle_',':SE_triangle_','X_se_out_far_'))])
    pocket=triangle.difference(road.buffer(0.3))
    # Keep the main safety island; remove the small detached triangular remnant.
    parts=_polygons(pocket)
    if parts:
        pocket=max(parts,key=lambda part:part.area)
    # Use the complete space enclosed by the actual lane corridors, including
    # the narrow wedges along the slip-road edge and the merging tip.
    all_road=unary_union(list(strips.values()))
    enclosed=[Polygon(ring) for part in _polygons(all_road.buffer(0.01))
              for ring in part.interiors]
    candidates=[part for part in enclosed if part.intersection(pocket).area>pocket.area*0.5]
    if candidates:
        pocket=max(candidates,key=lambda part:part.intersection(pocket).area).buffer(-0.12)
    for i,part in enumerate(_polygons(pocket)):
        if part.area<0.5:
            continue
        ET.SubElement(root,'poly',id=f'se_triangle_pavement_{i}',color='84,84,90',
                      fill='1',layer='4.9',shape=_write(list(part.exterior.coords)))
        ET.SubElement(root,'poly',id=f'se_triangle_border_{i}',color='235,190,40',
                      fill='0',lineWidth='0.2',layer='6',shape=_write(list(part.exterior.coords)))
    for i,d in enumerate(range(math.ceil(end)+4,math.floor(corner),4)):
        stroke=LineString([point(d-10,-8),point(d+20,-74)])
        for k,piece in enumerate(_lines(stroke.intersection(pocket.buffer(-0.3)))):
            if piece.length<0.3:
                continue
            ET.SubElement(root,'poly',id=f'se_triangle_hatch_{i}_{k}',color='235,190,40',
                          fill='0',lineWidth='0.25',layer='6',shape=_write(list(piece.coords)))
    shapes={(edge,int(index)):pts for name,pts in lanes.items()
            for edge,index in [name.rsplit('_',1)]}
    marks=[]
    from env.roundabout_final_crossroads import add_markings as mark_final_crossroads
    mark_final_crossroads(root,net_path,shapes)
    for mark in marks:
        root.append(ET.fromstring(mark))
    # Give both outbound dividers the same dash phase through the endpoint.
    base=point(0,0)
    along=tuple(point(1,0)[j]-base[j] for j in (0,1))
    def station(p):
        return sum((p[j]-base[j])*along[j] for j in (0,1))
    start=station(lanes['X_se_out_far_0'][-1])
    for p in list(root):
        if p.get('id','').startswith('lane_dash_X_se_out_far_1_'):
            if max(station(q) for q in _read(p.get('shape')))>start:
                root.remove(p)
    shift=tuple(point(0,3.2)[j]-base[j] for j in (0,1))
    for p in list(root):
        if not p.get('id','').startswith('x_cross_guide_1_'):
            continue
        pts=_read(p.get('shape'))
        if min(station(q) for q in pts)<start-0.001:
            continue
        ET.SubElement(root,'poly',id=p.get('id').replace('x_cross_guide_1_','se_triangle_aligned_dash_'),
                      color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                      shape=_write([tuple(q[j]+shift[j] for j in (0,1)) for q in pts]))
    # Restore the single missing dash immediately before the outer divider.
    first=next(p for p in root if p.get('id')=='lane_dash_SE_triangle_SW_out_0_0')
    a,b=_read(first.get('shape'))
    from env.roundabout_markings import dash_style
    dash,gap,_=dash_style(20)
    tangent=tuple((b[j]-a[j])/math.dist(a,b) for j in (0,1))
    start=tuple(a[j]-(dash+gap)*tangent[j] for j in (0,1))
    finish=tuple(start[j]+dash*tangent[j] for j in (0,1))
    ET.SubElement(root,'poly',id='se_triangle_SW_missing_dash',color='255,255,255',
                  fill='0',lineWidth='0.15',layer='6',shape=_write([start,finish]))
