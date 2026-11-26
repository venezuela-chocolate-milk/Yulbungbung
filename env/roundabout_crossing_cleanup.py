"""Finish the four painted islands and clear only the central crossing dashes."""
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon,LineString
from shapely.ops import unary_union
from env.roundabout_config import END_CROSSROADS as C
from env.roundabout_end_widening import layout
from env.roundabout_geometry import _read,_write
from env.roundabout_surfaces import _network,_polygons,_lines


def refine(root,net_path):
    point,_,_,end=layout()
    start=end+51+C['sw_end_t_left_length']
    s=-C['sw_arm_length']-C['sw_forward_length']
    _,_,strips=_network(net_path)
    road=unary_union([v for k,v in strips.items() if not k.startswith(':')])
    main=Polygon([point(start+35,s-16.5),point(start+180,s-16.5),point(start+180,s+16.5),point(start+35,s+16.5)])
    transverse=unary_union([Polygon(_read(p.get('shape'))) for p in root if p.get('id','').startswith('SW_new_straight_pavement_')])
    straight_lanes=unary_union([main,transverse])
    net=ET.parse(net_path).getroot()
    junction=net.find("junction[@id='SW_new_cross']")
    center=(float(junction.get('x')),float(junction.get('y')))
    origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    from env.roundabout_builder import _movement_path
    right_path=LineString(_movement_path(net_path,'SW_new_lower_in','SW_new_cross_out',0))
    right_corridor=right_path.buffer(1.9)
    fills=[p for p in root if p.get('id','').startswith('SW_drawn_island_fill_') or p.get('id')=='SW_long_safety_fill']
    if len(fills)!=4:raise ValueError('Expected all four safety islands before cleanup')
    islands=[]
    for p in fills:
        candidate=Polygon(_read(p.get('shape'))).buffer(0)
        delta=(candidate.centroid.x-center[0],candidate.centroid.y-center[1])
        lower_right=(sum(delta[j]*along[j] for j in (0,1))>0 and
                     sum(delta[j]*across[j] for j in (0,1))<0)
        lower_left=(sum(delta[j]*along[j] for j in (0,1))<0 and
                    sum(delta[j]*across[j] for j in (0,1))<0)
        if lower_left:
            # Grow only toward the freed outer-lane strip, then fit against
            # the four complete lanes and the existing curved slip lane.
            from shapely.affinity import translate
            side=net.find("junction[@id='SW_new_lower']")
            direction=(float(side.get('x'))-center[0],float(side.get('y'))-center[1])
            length=(direction[0]**2+direction[1]**2)**.5
            right=(direction[1]/length,-direction[0]/length)
            candidate=unary_union([candidate,translate(candidate,xoff=-right[0]*3.2,yoff=-right[1]*3.2)]).convex_hull
            candidate=candidate.difference(straight_lanes.buffer(.25))
        if lower_right:
            # Approach lanes end at SUMO's junction boundary. Preserve their
            # full straight continuation too, rather than treating that gap
            # as available island space and hiding the outermost lane.
            candidate=candidate.difference(unary_union([straight_lanes.buffer(.25),right_corridor]))
        parts=list(_polygons(candidate.difference(road.buffer(.2))))
        if not parts:raise ValueError('Safety island has no space outside the lane boundaries')
        island=max(parts,key=lambda q:q.area)
        island=island.simplify(.08,preserve_topology=True).buffer(-.08).buffer(.08)
        islands.append(island)
    for p in list(root):
        name=p.get('id','')
        if name.startswith(('SW_drawn_island_','SW_long_safety_hatch_')) or name in ('SW_long_safety_fill','SW_long_safety_border'):
            root.remove(p)
    for i,island in enumerate(islands):
        ET.SubElement(root,'poly',id=f'SW_drawn_island_fill_{i}',color='84,84,90',fill='1',layer='5',shape=_write(list(island.exterior.coords)))
        ET.SubElement(root,'poly',id=f'SW_drawn_island_border_{i}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(island.exterior.coords)))
        for j,d in enumerate(range(25,180,4)):
            stripe=LineString([point(start+d,s-100),point(start+d+70,s+100)])
            for k,piece in enumerate(_lines(stripe.intersection(island.buffer(-.25)))):
                if piece.length>.3:
                    ET.SubElement(root,'poly',id=f'SW_drawn_island_hatch_{i}_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))
    core=main.intersection(transverse).buffer(.5)
    clear=unary_union([core]+[q.buffer(.35) for q in islands])
    removed=0
    for p in list(root):
        if p.get('color')!='255,255,255' or p.get('fill')!='0' or float(p.get('lineWidth','1'))>.3:continue
        coords=_read(p.get('shape',''))
        if len(coords)<2:continue
        stroke=LineString(coords)
        if not stroke.intersects(clear):continue
        root.remove(p);removed+=1
        for j,piece in enumerate(_lines(stroke.difference(clear))):
            if piece.length<.5:continue
            attrs=dict(p.attrib);attrs.update(id=f"cropped_core_{p.get('id')}_{j}",shape=_write(list(piece.coords)))
            ET.SubElement(root,'poly',attrs)
    return islands,core,removed


def extend_upper_island_to_divider(root,net_path):
    """Align the photographed island's straight edge with the nearest divider."""
    from shapely.geometry import Point
    net=ET.parse(net_path).getroot()
    node=net.find("junction[@id='SW_new_cross']")
    center=(float(node.get('x')),float(node.get('y')))
    upper=net.find("junction[@id='SW_new_upper']")
    delta=(float(upper.get('x'))-center[0],float(upper.get('y'))-center[1])
    length=(delta[0]**2+delta[1]**2)**.5
    direction=(delta[0]/length,delta[1]/length)
    right=(direction[1],-direction[0])
    point,_,_,end=layout();origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1))
    across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    chosen=None
    for p in root:
        if not p.get('id','').startswith('SW_drawn_island_fill_'):continue
        shape=Polygon(_read(p.get('shape')))
        d=(shape.centroid.x-center[0],shape.centroid.y-center[1])
        if sum(d[j]*along[j] for j in (0,1))>0 and sum(d[j]*across[j] for j in (0,1))>0:
            chosen=p;break
    if chosen is None:raise ValueError('Photographed upper-right island not found')
    index=chosen.get('id').rsplit('_',1)[1]
    coords=[]
    for q in _read(chosen.get('shape')):
        offset=sum((q[j]-center[j])*right[j] for j in (0,1))
        # The fourth divider is 12.8 m from the median. Keep the yellow and
        # white stroke edges distinct while extending to that boundary.
        if offset<17.0:
            q=tuple(q[j]+right[j]*(13.0-offset) for j in (0,1))
        coords.append(q)
    island=Polygon(coords).buffer(0)
    parts=list(_polygons(island));island=max(parts,key=lambda q:q.area)
    chosen.set('shape',_write(list(island.exterior.coords)))
    root.find(f"poly[@id='SW_drawn_island_border_{index}']").set('shape',_write(list(island.exterior.coords)))
    for p in list(root):
        if p.get('id','').startswith(f'SW_drawn_island_hatch_{index}_'):root.remove(p)
    start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
    for j,d in enumerate(range(25,180,4)):
        stripe=LineString([point(start+d,s-100),point(start+d+70,s+100)])
        for k,piece in enumerate(_lines(stripe.intersection(island.buffer(-.25)))):
            if piece.length>.3:
                ET.SubElement(root,'poly',id=f'SW_drawn_island_hatch_{index}_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))
    for p in list(root):
        if p.get('color')!='255,255,255' or p.get('fill')!='0' or float(p.get('lineWidth','1'))>.3:continue
        stroke=LineString(_read(p.get('shape')))
        if not stroke.intersects(island.buffer(.18)):continue
        root.remove(p)
        for j,piece in enumerate(_lines(stroke.difference(island.buffer(.18)))):
            if piece.length<.5:continue
            attrs=dict(p.attrib);attrs.update(id=f"cropped_extended_{p.get('id')}_{j}",shape=_write(list(piece.coords)))
            ET.SubElement(root,'poly',attrs)
    return island


def fit_right_island_curves(root,net_path):
    """Align both right-side islands to dividers and real turning-lane edges."""
    from env.roundabout_builder import _movement_path
    net=ET.parse(net_path).getroot()
    node=net.find("junction[@id='SW_new_cross']")
    center=(float(node.get('x')),float(node.get('y')))
    point,_,_,end=layout();origin=point(0,0)
    along=tuple(point(1,0)[j]-origin[j] for j in (0,1));across=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    _,_,strips=_network(net_path)
    external=unary_union([v for k,v in strips.items() if not k.startswith(':')]).buffer(.1)
    changed=[]
    for label,source,target,divider,threshold in [('upper','SW_end_T_extension_in','SW_new_upper_out',12.8,17.0),('lower','SW_new_lower_in','SW_new_cross_out',-16.0,-20.2)]:
        endpoint=net.find(f"junction[@id='SW_new_{label}']")
        direction=(float(endpoint.get('x'))-center[0],float(endpoint.get('y'))-center[1]);length=(direction[0]**2+direction[1]**2)**.5
        right=(direction[1]/length,-direction[0]/length)
        chosen=None
        for p in root:
            if not p.get('id','').startswith('SW_drawn_island_fill_'):continue
            poly=Polygon(_read(p.get('shape')));delta=(poly.centroid.x-center[0],poly.centroid.y-center[1])
            if sum(delta[j]*along[j] for j in (0,1))<=0:continue
            if (sum(delta[j]*across[j] for j in (0,1))>0)==(label=='upper'):chosen=p;break
        if chosen is None:raise ValueError(f'Missing {label} island')
        coords=[]
        original=_read(chosen.get('shape'))
        if label=='lower':
            from env.roundabout_drawn_islands import OUTLINES
            start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
            # Fit the two straight sides to their nearest outside dividers;
            # the turning-lane envelope below then supplies the curved side.
            original=[point(start+d,s+(-13.3 if l>-17 else l)) for d,l in OUTLINES[2]]
        for q in original:
            offset=sum((q[j]-center[j])*right[j] for j in (0,1))
            if (label=='upper' and offset<threshold) or (label=='lower' and offset>threshold):
                q=tuple(q[j]+right[j]*(divider-offset) for j in (0,1))
            coords.append(q)
        turn=LineString(_movement_path(net_path,source,target,0))
        # The 3.2 m turn lane plus paint clearance determines the curved edge.
        reserve=unary_union([turn.buffer(1.7,quad_segs=32),external])
        parts=list(_polygons(Polygon(coords).buffer(0).difference(reserve)))
        if not parts:raise ValueError(f'No island space remains beside {label} right-turn lane')
        from shapely import set_precision
        island=set_precision(max(parts,key=lambda p:p.area).simplify(.005,preserve_topology=True),.0001)
        index=chosen.get('id').rsplit('_',1)[1]
        chosen.set('shape',_write(list(island.exterior.coords)))
        root.find(f"poly[@id='SW_drawn_island_border_{index}']").set('shape',_write(list(island.exterior.coords)))
        for p in list(root):
            if p.get('id','').startswith(f'SW_drawn_island_hatch_{index}_'):root.remove(p)
        start=end+51+C['sw_end_t_left_length'];s=-C['sw_arm_length']-C['sw_forward_length']
        for j,d in enumerate(range(25,180,4)):
            stripe=LineString([point(start+d,s-100),point(start+d+70,s+100)])
            for k,piece in enumerate(_lines(stripe.intersection(island.buffer(-.25)))):
                if piece.length>.3:ET.SubElement(root,'poly',id=f'SW_drawn_island_hatch_{index}_{j}_{k}',color='235,190,40',fill='0',lineWidth='.2',layer='6',shape=_write(list(piece.coords)))
        changed.append(island)
    clear=unary_union([q.buffer(.18) for q in changed])
    for p in list(root):
        if p.get('color')!='255,255,255' or p.get('fill')!='0' or float(p.get('lineWidth','1'))>.3:continue
        stroke=LineString(_read(p.get('shape')))
        if not stroke.intersects(clear):continue
        root.remove(p)
        for j,piece in enumerate(_lines(stroke.difference(clear))):
            if piece.length<.5:continue
            attrs=dict(p.attrib);attrs.update(id=f"cropped_curve_{p.get('id')}_{j}",shape=_write(list(piece.coords)))
            ET.SubElement(root,'poly',attrs)
    return changed
