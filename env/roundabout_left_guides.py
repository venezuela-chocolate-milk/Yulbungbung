"""Associate left-turn markings with the real SUMO movements they guide."""
GUIDE_PREFIXES={
    'E_out0_near':'t_user_guide_roundabout_to_side_',
    'T_side_in':'t_user_guide_side_to_main_',
    'T_stop_main_out':'x_left_turn_guide_T_stop_main_out_',
    'X_main_in':'x_left_turn_guide_X_main_in_',
    'X_nw_in':'x_left_turn_guide_X_nw_in_',
    'X_se_in':'x_left_turn_guide_X_se_in_',
    'SE_triangle_NE_in':'se_final_turn_guide_SE_triangle_NE_in_',
    'SE_triangle_cross_in':'se_final_turn_guide_SE_triangle_cross_in_',
    'SE_triangle_forward_in':'se_final_turn_guide_SE_triangle_forward_in_',
    'SE_triangle_main_out':'se_final_turn_guide_SE_triangle_main_out_',
    'SE_triangle_SW_out_far':'SW_median_left_guide_upper_',
    'SW_final_forward_in':'SW_median_left_guide_lower_',
    'SW_final_left_in':'SW_median_left_guide_right_',
    'SW_final_right_in':'SW_median_left_guide_left_',
    'SW_end_T_right_in':'SW_end_T_median_guide_left_to_stem_',
    'SW_final_forward_out':'SW_end_T_median_guide_stem_to_right_',
}


def median_boundary_path(net_path,source,target,start_tip,end_tip):
    """Fit a median guide to the inside boundary of the innermost left lane."""
    import xml.etree.ElementTree as ET
    from shapely.geometry import LineString,Point
    from env.roundabout_builder import _movement_path
    from env.roundabout_geometry import _offset,_slice
    net=ET.parse(net_path).getroot()
    connection=max((c for c in net.findall('connection')
                    if c.get('from')==source and c.get('to')==target and c.get('dir') in ('l','L')),
                   key=lambda c:int(c.get('fromLane')))
    index=int(connection.get('fromLane'))
    edge=next(e for e in net.findall('edge') if e.get('id')==source)
    lane=next(l for l in edge.findall('lane') if int(l.get('index'))==index)
    width=float(lane.get('width','3.2'))
    center=_movement_path(net_path,source,target,index)
    boundary=_offset(center,width/2)
    line=LineString(boundary)
    start=line.project(Point(start_tip));finish=line.project(Point(end_tip))
    if finish<=start:
        raise ValueError(f'Median endpoints run against left-turn movement {source} -> {target}')
    path=_slice(boundary,start,finish)
    center_line=LineString(center)
    # Preserve existing tips only when their small connection remains outside
    # the vehicle corridor; otherwise end the paint on the real lane boundary.
    for tip,position in [(start_tip,0),(end_tip,-1)]:
        if line.distance(Point(tip))<=0.6 and center_line.distance(Point(tip))>=width/2-0.05:
            path[position]=tip
    return path
