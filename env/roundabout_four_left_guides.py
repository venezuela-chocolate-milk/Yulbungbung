"""Median-to-median guides derived from the four real innermost left turns."""
import xml.etree.ElementTree as ET
from shapely.geometry import LineString,Point
from env.roundabout_geometry import _read,_write,_offset,_slice,_length,_curve,_unit
from env.roundabout_builder import _movement_path

MOVEMENTS=[('lower','west','SW_new_lower_in','SW_new_cross_in',5),
           ('upper','east','SW_new_upper_in','SW_new_cross_out',5),
           ('west','upper','SW_slip_main','SW_new_upper_out',4),
           ('east','lower','SW_end_T_extension_in','SW_new_lower_near',5)]

def fit_receiving_lanes(root):
    edges={e.get('id'):e for e in root.findall('edge')}
    lanes={l.get('id'):l for e in edges.values() for l in e.findall('lane')}
    follow={(c.get('from'),c.get('fromLane')):c for c in root.findall('connection') if c.get('from').startswith(':')}
    for _,_,source,target,index in MOVEMENTS:
        c=next(c for c in root.findall('connection') if c.get('from')==source and c.get('to')==target and c.get('fromLane')==str(index))
        destination=max(edges[target].findall('lane'),key=lambda l:int(l.get('index')))
        if c.get('toLane')==destination.get('index'):continue
        a=_read(lanes[f'{source}_{index}'].get('shape'));b=_read(destination.get('shape'))
        path=_curve(a[-1],b[0],_unit(a[-2],a[-1]),_unit(b[0],b[1]))
        chain=[];via=c.get('via')
        while via:
            chain.append(via);edge,i=via.rsplit('_',1);continuation=follow[(edge,i)];via=continuation.get('via')
        c.set('toLane',destination.get('index'));continuation.set('toLane',destination.get('index'))
        sizes=[float(lanes[k].get('length')) for k in chain];total=sum(sizes);length=_length(path);start=0
        for i,(key,size) in enumerate(zip(chain,sizes)):
            finish=length if i==len(chain)-1 else start+length*size/total
            piece=_slice(path,start,finish);lanes[key].set('shape',_write(piece));lanes[key].set('length',str(_length(piece)));start=finish

def add_guides(root,net_path):
    from env.roundabout_markings import dash_style
    net=ET.parse(net_path).getroot();edges={e.get('id'):e for e in net.findall('edge')}
    names={'lower':('SW_new_center_lower','SW_lower_approach_stop',0),
           'upper':('SW_new_center_upper','SW_upper_approach_stop',-1),
           'west':('SW_end_T_extension_center','SW_end_T_extension_stop',-1),
           'east':('SW_end_T_extension_center_far','SW_new_east_stop',0)}
    tips={}
    for arm,(name,stop_name,endpoint) in names.items():
        line=root.find(f"poly[@id='{name}']");stop=root.find(f"poly[@id='{stop_name}']")
        tip=_read(stop.get('shape'))[endpoint];points=_read(line.get('shape'))
        points[-1 if arm=='west' else 0]=tip;line.set('shape',_write(points));tips[arm]=tip
    for p in list(root):
        if p.get('id','').startswith('SW_four_left_guide_'):root.remove(p)
    paths={}
    for a,b,source,target,index in MOVEMENTS:
        src=edges[source].find(f"lane[@index='{index}']")
        dst=max(edges[target].findall('lane'),key=lambda l:int(l.get('index')))
        center=_read(src.get('shape'))+_movement_path(net_path,source,target,index)[1:]+_read(dst.get('shape'))[1:]
        boundary=_offset(center,float(src.get('width','3.2'))/2);line=LineString(boundary)
        start=line.project(Point(tips[a]));finish=line.project(Point(tips[b]))
        if finish<=start:raise ValueError(f'Reversed left guide: {source}')
        path=_slice(boundary,start,finish)
        for i,tip in [(0,tips[a]),(-1,tips[b])]:
            if Point(path[i]).distance(Point(tip))>1:raise ValueError(f'Median too far from actual lane: {source}: {tip}')
            path[i]=tip
        length=_length(path);dash,standard_gap,_=dash_style(20)
        count=max(2,round((length+standard_gap)/(dash+standard_gap)));gap=(length-count*dash)/(count-1)
        for i in range(count):
            at=i*(dash+gap)
            ET.SubElement(root,'poly',id=f'SW_four_left_guide_{a}_{i}',color='255,255,255',fill='0',lineWidth='.15',layer='6',shape=_write(_slice(path,at,at+dash)))
        paths[a]=path
    return paths
