"""Basic four-arm junction at the southwest endpoint, ready for later detailing."""
import xml.etree.ElementTree as ET
from env.roundabout_end_widening import layout
from env.roundabout_geometry import _read,_write


def add_network(nodes,edges,connections):
    from env.roundabout_config import END_CROSSROADS
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']
    forward_length=END_CROSSROADS['sw_forward_length']
    center=next(n for n in nodes if n.get('id')=='SE_triangle_SW')
    center.set('type','priority');center.set('radius','14')
    def node(name,station,lateral):
        xy=point(station,lateral)
        ET.SubElement(nodes,'node',id=name,x=str(xy[0]),y=str(xy[1]),type='dead_end')
    node('SW_final_forward',d,s-forward_length)
    node('SW_final_left',d+60,s)
    node('SW_final_right',d-60,s)
    def edge(name,fr,to,count,coords):
        ET.SubElement(edges,'edge',id=name,**{'from':fr,'to':to,'numLanes':str(count),
                      'width':'3.2','speed':'8.33','priority':'2','spreadType':'center',
                      'shape':_write([point(a,b) for a,b in coords])})
    edge('SW_final_forward_out','SE_triangle_SW','SW_final_forward',4,[(d-6.9,s),(d-6.9,s-forward_length)])
    edge('SW_final_forward_in','SW_final_forward','SE_triangle_SW',3,[(d+5.3,s-forward_length),(d+5.3,s)])
    edge('SW_final_left_out','SE_triangle_SW','SW_final_left',1,[(d,s-2.1),(d+60,s-2.1)])
    edge('SW_final_left_in','SW_final_left','SE_triangle_SW',1,[(d+60,s+2.1),(d,s+2.1)])
    edge('SW_final_right_out','SE_triangle_SW','SW_final_right',1,[(d,s+2.1),(d-60,s+2.1)])
    edge('SW_final_right_in','SW_final_right','SE_triangle_SW',1,[(d-60,s-2.1),(d,s-2.1)])
    def connect(fr,to,source=0,target=0):
        ET.SubElement(connections,'connection',**{'from':fr,'to':to,
                      'fromLane':str(source),'toLane':str(target),'contPos':'0'})
    for lane in range(4):
        connect('SE_triangle_SW_out_far','SW_final_forward_out',lane,lane)
    for lane in range(3):
        connect('SW_final_forward_in','SE_triangle_SW_in',lane,lane)
    connect('SE_triangle_SW_out_far','SW_final_right_out',0)
    connect('SE_triangle_SW_out_far','SW_final_left_out',3)
    connect('SW_final_forward_in','SW_final_left_out',0)
    connect('SW_final_forward_in','SW_final_right_out',2)
    connect('SW_final_left_in','SW_final_right_out')
    connect('SW_final_left_in','SE_triangle_SW_in',0,0)
    connect('SW_final_left_in','SW_final_forward_out',0,3)
    connect('SW_final_right_in','SW_final_left_out')
    connect('SW_final_right_in','SW_final_forward_out',0,0)
    connect('SW_final_right_in','SE_triangle_SW_in',0,2)


def add_routes(root):
    for flow in root.findall('flow'):
        if flow.get('to')=='SE_triangle_SW_out_far':
            flow.set('to','SW_final_forward_out')
        if flow.get('from')=='SE_triangle_SW_in':
            flow.set('from','SW_final_forward_in')
    # Frequent, alternating turns from both single-lane side approaches.
    for side,source,left_target,right_target,offset in [
            ('west','SW_final_right_in','SE_triangle_SW_in','SW_final_forward_out',0),
            ('east','SW_final_left_in','SW_final_forward_out','SE_triangle_SW_in',3)]:
        for turn,target,begin,color in [('left',left_target,offset,'200,80,255'),
                                       ('right',right_target,offset+6,'255,140,0')]:
            ET.SubElement(root,'flow',id=f'SW_side_{side}_{turn}',type='car',
                          **{'from':source,'to':target,'begin':str(begin),'end':'100000',
                             'period':'12','departLane':'0','departSpeed':'0','color':color})
    flows=root.findall('flow')
    for flow in flows:
        root.remove(flow)
    for flow in sorted(flows,key=lambda item:float(item.get('begin','0'))):
        root.append(flow)


def add_markings(root,net_path):
    from shapely.geometry import Polygon,LineString
    from shapely.ops import unary_union
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_surfaces import _network,_polygons,_lines
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']
    forward_length=END_CROSSROADS['sw_forward_length']
    _,_,strips=_network(net_path)
    road=unary_union(list(strips.values()))
    right_stop=d-25.0
    left_stop=d+21.0
    for label,start,finish in [('right',d-60,d),('left',d,d+60)]:
        median=Polygon([point(start,s-0.5),point(finish,s-0.5),
                        point(finish,s+0.5),point(start,s+0.5)]).difference(road)
        for i,part in enumerate(_polygons(median)):
            ET.SubElement(root,'poly',id=f'SW_{label}_median_pavement_{i}',color='84,84,90',
                          fill='1',layer='4.9',shape=_write(list(part.exterior.coords)))
        center=LineString([point(start,s),point(finish,s)]).difference(road.buffer(0.03))
        if label=='right':
            center=LineString([point(start,s),point(right_stop,s)])
        else:
            center=LineString([point(left_stop,s),point(finish,s)])
        for i,line in enumerate(_lines(center)):
            if line.length<0.3:
                continue
            ET.SubElement(root,'poly',id=f'SW_{label}_center_line_{i}',color='235,190,40',
                          fill='0',lineWidth='0.2',layer='6',shape=_write(list(line.coords)))
    median=Polygon([point(d-0.5,s-forward_length),point(d+0.5,s-forward_length),
                    point(d+0.5,s),point(d-0.5,s)]).difference(road)
    for i,part in enumerate(_polygons(median)):
        ET.SubElement(root,'poly',id=f'SW_forward_median_pavement_{i}',color='84,84,90',
                      fill='1',layer='4.9',shape=_write(list(part.exterior.coords)))
    center=LineString([point(d,s-forward_length),point(d,s)]).difference(road.buffer(0.03))
    for i,line in enumerate(_lines(center)):
        if line.length<0.3:
            continue
        ET.SubElement(root,'poly',id=f'SW_forward_center_line_{i}',color='235,190,40',
                      fill='0',lineWidth='0.2',layer='6',shape=_write(list(line.coords)))
    ET.SubElement(root,'poly',id='SW_right_stop',color='255,255,255',fill='0',
                  lineWidth='0.45',layer='6',
                  shape=_write([point(right_stop,s),point(right_stop,s-3.7)]))
    ET.SubElement(root,'poly',id='SW_left_stop',color='255,255,255',fill='0',
                  lineWidth='0.45',layer='6',
                  shape=_write([point(left_stop,s),point(left_stop,s+3.7)]))
    from env.roundabout_builder import _read_lane_shapes,_arrow
    shapes=_read_lane_shapes(net_path)
    # Keep all three separators on this four-lane arm in the same dash phase.
    import math
    from env.roundabout_markings import dash_style
    origin=point(0,0)
    axis=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    def local(q):
        return [sum((q[j]-origin[j])*a[j] for j in (0,1)) for a in (axis,across)]
    arm_start=local(shapes[('SW_final_forward_out',0)][0])[1]
    arm_end=local(shapes[('SW_final_forward_out',0)][-1])[1]
    divider_d=[d-10.1,d-6.9,d-3.7]
    dash,gap,_=dash_style(20)
    for p in list(root):
        if not p.get('id','').startswith('lane_dash_'):
            continue
        coords=[local(q) for q in _read(p.get('shape'))]
        if any(all(abs(q[0]-divider)<0.01 for q in coords) for divider in divider_d):
            if min(q[1] for q in coords)<arm_start+dash and max(q[1] for q in coords)>arm_end:
                root.remove(p)
    dash,gap,_=dash_style(20)
    count=max(0,math.floor((arm_start-arm_end-dash)/(dash+gap))+1)
    for lane,divider in enumerate(divider_d):
        for i in range(count):
            start=arm_start-i*(dash+gap)
            ET.SubElement(root,'poly',id=f'SW_forward_aligned_dash_{lane}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write([point(divider,start),point(divider,start-dash)]))
    marks=[]
    origin=point(0,0)
    normal=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    forward_end=shapes[('SW_final_forward_in',0)][-1]
    forward_stop=sum((forward_end[j]-origin[j])*normal[j] for j in (0,1))-1.5
    ET.SubElement(root,'poly',id='SW_forward_stop',color='255,255,255',fill='0',
                  lineWidth='0.45',layer='6',
                  shape=_write([point(d,forward_stop),point(d+10.1,forward_stop)]))
    for lane,kind in enumerate(('straight_right','straight','left')):
        _arrow(marks,f'SW_forward_arrow_{lane}',shapes,'SW_final_forward_in',lane,kind,12.0)
    _arrow(marks,'SW_right_near_arrow',shapes,'SW_final_right_in',0,'right',8.0)
    _arrow(marks,'SW_right_far_arrow',shapes,'SW_final_right_in',0,'left',20.0)
    _arrow(marks,'SW_left_near_arrow',shapes,'SW_final_left_in',0,'right',8.0)
    _arrow(marks,'SW_left_far_arrow',shapes,'SW_final_left_in',0,'left',20.0)
    for mark in marks:
        root.append(ET.fromstring(mark))
    for p in list(root):
        if p.get('id','').startswith(('lane_dash_SW_final_forward_in_0_',
                                     'lane_dash_SW_final_forward_in_1_')):
            positions=[local(q)[1] for q in _read(p.get('shape'))]
            if max(positions)>forward_stop and min(positions)<local(forward_end)[1]+0.01:
                root.remove(p)


def clear_marked_area(root):
    """Clear only lane dashes and medians inside the user's red outline."""
    from shapely.geometry import Polygon,LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_surfaces import _polygons,_lines
    from env.roundabout_geometry import _read
    point,_,_,end=layout()
    d=end+51
    s=-END_CROSSROADS['sw_arm_length']
    outline=[(-13.080,16.382),(-10.310,16.833),(-3.812,17.694),(2.352,18.249),
             (7.516,17.890),(10.258,17.703),(11.305,13.947),(11.815,9.639),
             (11.746,5.229),(11.140,0.268),(10.548,-4.374),(10.508,-9.552),
             (10.017,-13.367),(9.133,-14.542),(6.885,-14.762),(4.593,-14.531),
             (1.677,-13.953),(-1.470,-14.259),(-4.080,-15.421),(-7.083,-15.351),
             (-10.071,-14.961),(-12.435,-16.326),(-13.117,-15.848),(-14.845,-11.614),
             (-16.444,-7.321),(-16.185,-4.391),(-15.099,-1.562),(-14.376,1.731),
             (-14.058,4.531),(-14.728,8.142),(-14.454,11.391),(-14.355,15.032)]
    area=Polygon([point(d+x,s+y) for x,y in outline])
    for p in list(root):
        name=p.get('id','')
        if any(word in name for word in ('stop','arrow')):
            continue
        white=p.get('color')=='255,255,255' and p.get('fill')=='0'
        yellow=p.get('color')=='235,190,40' and ('center' in name or 'median_gap' in name)
        if not (white or yellow):
            continue
        pts=_read(p.get('shape'))
        filled=p.get('fill')=='1'
        shape=Polygon(pts) if filled else LineString(pts)
        if not shape.intersects(area):
            continue
        pieces=_polygons(shape.difference(area)) if filled else _lines(shape.difference(area))
        root.remove(p)
        for i,piece in enumerate(pieces):
            if (piece.area if filled else piece.length)<0.05:
                continue
            clone=ET.Element('poly',p.attrib)
            # These are intentionally clipped border fragments rather than
            # full standard lane dashes.
            base_name=f'cropped_{name}' if name.startswith('lane_dash_') else name
            clone.set('id',base_name if i==0 else f'{base_name}_outside_{i}')
            clone.set('shape',_write(list(piece.exterior.coords) if filled else list(piece.coords)))
            root.append(clone)


def add_median_left_guides(root,net_path):
    """Connect the four painted median tips with smooth left-turn guides."""
    import math
    from shapely.geometry import LineString
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _read,_slice
    from env.roundabout_markings import dash_style
    from env.roundabout_left_guides import median_boundary_path
    for p in list(root):
        if p.get('id','').startswith('SW_median_left_guide_'):
            root.remove(p)
    point,_,_,end=layout()
    center=point(end+51,-END_CROSSROADS['sw_arm_length'])
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    normal=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    tips={}
    for arm,prefix in [('upper','se_final_center_SW_'),('lower','SW_forward_center_line_'),
                       ('left','SW_right_center_line_'),('right','SW_left_center_line_')]:
        ends=[q for p in root if p.get('id','').startswith(prefix)
              for q in (_read(p.get('shape'))[0],_read(p.get('shape'))[-1])]
        nearest=min(ends,key=lambda q:math.dist(q,center))
        cluster=[q for q in ends if math.dist(q,nearest)<1.0]
        tips[arm]=tuple(sum(q[j] for q in cluster)/len(cluster) for j in (0,1))
    turns=[('upper','right',tuple(-x for x in normal),along),
           ('right','lower',tuple(-x for x in along),tuple(-x for x in normal)),
           ('lower','left',normal,tuple(-x for x in along)),
           ('left','upper',along,normal)]
    dash,standard_gap,_=dash_style(20)
    for source,target,incoming,outgoing in turns:
        a,b=tips[source],tips[target]
        handle=math.dist(a,b)*0.4
        controls=[a,tuple(a[j]+incoming[j]*handle for j in (0,1)),
                  tuple(b[j]-outgoing[j]*handle for j in (0,1)),b]
        path=[]
        for i in range(201):
            t=i/200
            path.append(tuple((1-t)**3*controls[0][j]+3*(1-t)**2*t*controls[1][j]
                              +3*(1-t)*t*t*controls[2][j]+t**3*controls[3][j]
                              for j in (0,1)))
        repaired_movements={'lower':('SW_final_forward_in','SW_final_right_out'),
                            'right':('SW_final_left_in','SW_final_forward_out')}
        if source in repaired_movements:
            fr,to=repaired_movements[source]
            path=median_boundary_path(net_path,fr,to,a,b)
        length=LineString(path).length
        count=max(2,round((length+standard_gap)/(dash+standard_gap)))
        gap=(length-count*dash)/(count-1)
        for i in range(count):
            start=i*(dash+gap)
            ET.SubElement(root,'poly',id=f'SW_median_left_guide_{source}_{i}',
                          color='255,255,255',fill='0',lineWidth='0.15',layer='6',
                          shape=_write(_slice(path,start,start+dash)))

