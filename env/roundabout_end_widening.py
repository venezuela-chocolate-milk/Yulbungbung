"""Open one inside lane after a painted safety-zone taper at the SE endpoint."""
import math
import xml.etree.ElementTree as ET

from env.roundabout_geometry import _read, _write


def layout():
    from env.roundabout_config import END_CROSSROADS, T_JUNCTION
    from env.roundabout_crossroads import center_station
    angle=math.radians(END_CROSSROADS['cross_axis'])
    axis=(-math.cos(angle),-math.sin(angle))
    normal=(-axis[1],axis[0])
    main=math.radians(T_JUNCTION['axis'])
    origin=(center_station()*math.cos(main),center_station()*math.sin(main))
    length=END_CROSSROADS['se_arm_length']
    start=length-END_CROSSROADS['se_end_widening']
    finish=start+END_CROSSROADS['se_end_taper']
    def point(d,lateral):
        return tuple(origin[j]+axis[j]*d+normal[j]*lateral for j in (0,1))
    return point,start,finish,length


def add_network(nodes,edges,connections):
    point,start,finish,length=layout()
    edge=next(e for e in edges if e.get('id')=='X_se_out')
    original=_read(edge.get('shape'))
    ET.SubElement(nodes,'node',id='X_se_widen',x=str(point(finish,0)[0]),
                  y=str(point(finish,0)[1]),type='priority',radius='0')
    edge.set('to','X_se_widen')
    curve=[point(start+(finish-start)*i/100,-3.7-3.2*(3*(i/100)**2-2*(i/100)**3))
           for i in range(101)]
    edge.set('shape',_write([original[0]]+curve))
    attributes=dict(edge.attrib)
    attributes.update(id='X_se_out_far',**{'from':'X_se_widen','to':'X_se_end',
                      'numLanes':'3','shape':_write([point(finish,-5.3),point(length,-5.3)])})
    ET.SubElement(edges,'edge',attributes)
    for lane in (0,1):
        ET.SubElement(connections,'connection',**{'from':'X_se_out','to':'X_se_out_far',
                      'fromLane':str(lane),'toLane':str(lane)})


def add_markings(root):
    from shapely.geometry import Polygon,LineString
    from env.roundabout_surfaces import _lines
    point,start,finish,length=layout()
    inner=[point(start+(finish-start)*i/100,-0.5-3.2*(3*(i/100)**2-2*(i/100)**3))
           for i in range(101)]
    from env.roundabout_config import END_CROSSROADS
    entry_length=END_CROSSROADS['se_end_entry_taper']
    # Close the island with a tangent-continuous narrowing nose, rather than
    # a crosswise bar at the mouth of the new inside lane.
    inner.extend(point(finish+entry_length*i/80,
                       -3.7+3.2*(3*(i/80)**2-2*(i/80)**3))
                 for i in range(1,81))
    tip=finish+entry_length
    # Keep the whole opening paved, including the part before the new lane's
    # traversable centerline begins beyond the safety-zone nose.
    opening=[point(finish,-3.7),point(length,-3.7),
             point(length,-0.5),point(finish,-0.5)]
    ET.SubElement(root,'poly',id='x_se_end_entry_pavement',color='84,84,90',
                  fill='1',layer='4.8',shape=_write(opening))
    # This pocket is pavement outside the two traversable taper lanes.
    pocket=Polygon([point(start,-0.5),point(tip,-0.5)]+list(reversed(inner)))
    ET.SubElement(root,'poly',id='x_se_end_safety_pavement',color='84,84,90',
                  fill='1',layer='4.8',shape=_write(list(pocket.exterior.coords)))
    ET.SubElement(root,'poly',id='x_se_end_safety_border',color='235,190,40',
                  fill='0',lineWidth='0.15',layer='6',shape=_write(inner))
    for i,d in enumerate(range(math.ceil(start)+3,math.floor(tip),4)):
        stroke=LineString([point(d-2,-0.5),point(d+2,-4.0)])
        for k,piece in enumerate(_lines(stroke.intersection(pocket.buffer(-0.18)))):
            if piece.length<0.15:
                continue
            ET.SubElement(root,'poly',id=f'x_se_end_safety_hatch_{i}_{k}',
                          color='235,190,40',fill='0',lineWidth='0.2',layer='6',
                          shape=_write(list(piece.coords)))


def finish_geometry(root):
    """Keep vehicles out of the painted nose until the new lane is full width."""
    from env.roundabout_config import END_CROSSROADS
    from env.roundabout_geometry import _slice,_length
    edge=next((e for e in root.findall('edge') if e.get('id')=='X_se_out_far'),None)
    if edge is None:
        return
    lane=next(l for l in edge.findall('lane') if l.get('index')=='2')
    points=_read(lane.get('shape'))
    points=_slice(points,END_CROSSROADS['se_end_entry_taper'],_length(points))
    lane.set('shape',_write(points))
    lane.set('length',f'{_length(points):.6f}')
