"""Add one outside lane downstream of the marked southwest merge."""
import xml.etree.ElementTree as ET
import math
from env.roundabout_geometry import _read,_write,_slice,_length
from env.roundabout_end_widening import layout

START=-70.0
TAPER=12.0


def add_network(nodes,edges,connections):
    from env.roundabout_config import END_CROSSROADS
    sw_length=END_CROSSROADS['sw_arm_length']
    point,_,_,end=layout()
    corner=end+51
    edge=next(e for e in edges if e.get('id')=='SE_triangle_SW_out')
    pts=_read(edge.get('shape'))
    xy=point(corner,START)
    ET.SubElement(nodes,'node',id='SE_SW_outer_lane_start',x=str(xy[0]),y=str(xy[1]),
                  type='priority',radius='2')
    edge.set('to','SE_SW_outer_lane_start')
    edge.set('shape',_write([pts[0],point(corner-5.3,START)]))
    attributes=dict(edge.attrib)
    attributes.update(id='SE_triangle_SW_out_far',**{'from':'SE_SW_outer_lane_start',
                      'to':'SE_triangle_SW','numLanes':'4',
                      'shape':_write([point(corner-6.9,START),point(corner-6.9,-sw_length)])})
    ET.SubElement(edges,'edge',attributes)
    for lane in range(3):
        ET.SubElement(connections,'connection',**{'from':'SE_triangle_SW_out',
                      'to':'SE_triangle_SW_out_far','fromLane':str(lane),'toLane':str(lane+1)})


def add_routes(root):
    for flow in root.findall('flow'):
        if flow.get('to')=='SE_triangle_SW_out':
            flow.set('to','SE_triangle_SW_out_far')


def finish_geometry(root):
    edge=next(e for e in root.findall('edge') if e.get('id')=='SE_triangle_SW_out_far')
    lane=next(l for l in edge.findall('lane') if l.get('index')=='0')
    pts=_read(lane.get('shape'))
    point,_,_,_=layout()
    origin=point(0,0)
    normal=tuple(point(0,1)[j]-origin[j] for j in (0,1))
    lateral_station=sum((pts[0][j]-origin[j])*normal[j] for j in (0,1))
    trim=max(0,lateral_station-(START-TAPER))
    pts=_slice(pts,trim,_length(pts))
    lane.set('shape',_write(pts))
    lane.set('length',f'{_length(pts):.6f}')


def add_markings(root,net_path):
    point,_,_,end=layout()
    corner=end+51
    curb=[]
    for i in range(101):
        t=i/100
        curb.append(point(corner-10.1-3.2*t*t*(3-2*t),START-TAPER*t))
    pavement=[point(corner-10.1,START),point(corner-10.1,START-TAPER)]+list(reversed(curb))
    ET.SubElement(root,'poly',id='se_SW_outer_lane_taper_pavement',color='84,84,90',
                  fill='1',layer='4.9',shape=_write(pavement))
    from env.roundabout_markings import dash_style
    first=next(p for p in root if p.get('id')=='lane_dash_SE_triangle_SW_out_far_0_0')
    a,b=_read(first.get('shape'))
    dash,gap,_=dash_style(20)
    tangent=tuple((b[j]-a[j])/math.dist(a,b) for j in (0,1))
    start=tuple(a[j]-(dash+gap)*tangent[j] for j in (0,1))
    finish=tuple(start[j]+dash*tangent[j] for j in (0,1))
    ET.SubElement(root,'poly',id='se_SW_outer_lane_entry_dash',color='255,255,255',
                  fill='0',lineWidth='0.15',layer='6',shape=_write([start,finish]))
    from env.roundabout_builder import _read_lane_shapes,_stop_line,_arrow
    shapes=_read_lane_shapes(net_path)
    marks=[]
    edge='SE_triangle_SW_out_far'
    _stop_line(marks,'se_SW_end_stop',shapes,edge,dist=9.0)
    for lane,kind in enumerate(('straight_right','straight','straight','left')):
        _arrow(marks,f'se_SW_end_arrow_{lane}',shapes,edge,lane,kind,18.0)
    for mark in marks:
        root.append(ET.fromstring(mark))
