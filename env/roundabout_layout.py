"""Road-first channelization: fit markings into the space between lane strips."""
import math
import xml.etree.ElementTree as ET

from env.roundabout_geometry import _read, _offset, _unit, _check_bundle


def _cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def _cut_at_line(points, baseline):
    a, b = baseline
    direction = (b[0]-a[0], b[1]-a[1])
    side = _cross(direction, (points[0][0]-a[0], points[0][1]-a[1]))
    sign = 1.0 if side >= 0 else -1.0
    out = [points[0]]
    for p, q in zip(points, points[1:]):
        dp = sign*_cross(direction, (p[0]-a[0], p[1]-a[1]))
        dq = sign*_cross(direction, (q[0]-a[0], q[1]-a[1]))
        if dp > 0 and dq <= 0:
            t = dp/(dp-dq)
            out.append((p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1])))
            return out
        out.append(q)
    raise ValueError('The lane corridor does not reach the island baseline')


def channel_layout(net_path, source, baseline, width=3.2, clearance=0.15):
    """Return the two road corridors and an island fitted between them.

    The 1-lane and 2-lane strips are fixed first. Each island side is offset
    outside the corresponding drivable strip, and its base is clipped to the
    large island's extended painted edge. No arbitrary triangle width is used.
    """
    root = ET.parse(net_path).getroot()
    lanes = {l.get('id'): _read(l.get('shape'))
             for edge in root.findall('edge') for l in edge.findall('lane')}
    connections = root.findall('connection')
    follow = {(c.get('from'), c.get('fromLane')): c for c in connections
              if c.get('from').startswith(':')}

    def complete_path(index):
        result = list(lanes[f'{source}_{index}'])
        turn = next(c for c in connections if c.get('from') == source
                    and c.get('fromLane') == str(index))
        current = turn.get('via')
        while current:
            result.extend(lanes[current][1:])
            edge_id, lane_index = current.rsplit('_', 1)
            current = follow[(edge_id, lane_index)].get('via')
        return result

    paths = [complete_path(index) for index in range(3)]
    from shapely.geometry import LineString
    right_strip = LineString(_cut_at_line(paths[0], baseline)).buffer(width/2, cap_style=2)
    left_strips = [LineString(_cut_at_line(p, baseline)).buffer(width/2, cap_style=2)
                   for p in paths[1:]]
    overlap = sum(right_strip.intersection(strip).area for strip in left_strips)
    if overlap > 0.002:
        raise ValueError(f'The independent slip lane overlaps the left-turn lanes: {overlap:.4f} m2')
    # A complete drivable strip must keep its width on both sides of every
    # centerline, and the two left-turn lanes must remain parallel through joins.
    strip_error = max(_check_bundle([_offset(path, -width/2), path,
                                     _offset(path, width/2)], width/2) for path in paths)
    spacing_error = _check_bundle(paths[1:], width)
    # White road edges locate the actual 3.2m strip; yellow paint stays outside it.
    right_outer = _cut_at_line(_offset(paths[0], -width/2), baseline)
    right_inner = _cut_at_line(_offset(paths[0], width/2+clearance), baseline)
    left_inner = _cut_at_line(_offset(paths[1], -width/2-clearance), baseline)
    left_outer = _cut_at_line(_offset(paths[2], width/2), baseline)
    divider = _cut_at_line(_offset(paths[1], width/2), baseline)

    # The independent slip curve has its own sampling/length. Find the opening
    # by intersecting actual cross-sections, not by matching vertex indices.
    left_source = lanes[f'{source}_1']
    opening = None
    for k in range(min(len(left_source), len(left_inner))-1):
        tangent = _unit(left_source[max(0, k-1)], left_source[min(len(left_source)-1, k+1)])
        normal = (-tangent[1], tangent[0])
        p = left_inner[k]
        hits = []
        for j, (a, b) in enumerate(zip(right_inner, right_inner[1:])):
            segment = (b[0]-a[0], b[1]-a[1])
            denominator = _cross(normal, segment)
            if abs(denominator) < 1e-10:
                continue
            delta = (a[0]-p[0], a[1]-p[1])
            d, u = _cross(delta, segment)/denominator, _cross(delta, normal)/denominator
            if d < -0.12 and -1e-8 <= u <= 1+1e-8:
                hits.append((abs(d), j, (p[0]+d*normal[0], p[1]+d*normal[1])))
        if hits:
            _, j, right_point = min(hits)
            opening = k, j, right_point
            break
    if opening is None:
        raise ValueError('No free space remains for a channelizing island')
    k, j, right_point = opening
    nose = ((right_point[0]+left_inner[k][0])/2, (right_point[1]+left_inner[k][1])/2)
    island = [nose, right_point]+right_inner[j+1:]+list(reversed(left_inner[k:]))

    # Verify the entire island, not just its nominal base, against all three
    # actual lane centerlines. Interpolation also covers the long base segment.
    samples = []
    for a, b in zip(island, island[1:]+island[:1]):
        count = max(1, math.ceil(math.dist(a, b)/0.1))
        samples.extend((a[0]+(b[0]-a[0])*k/count, a[1]+(b[1]-a[1])*k/count)
                       for k in range(count))
    def distance_to_segment(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        t = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2)) if length2 else 0.0
        return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)
    min_clearance = min(distance_to_segment(p, a, b)-width/2
                        for path in paths for a, b in zip(path, path[1:]) for p in samples)
    if min_clearance < clearance-0.015:
        raise ValueError(f'Safety island intrudes into the road corridor: {min_clearance:.3f} m')
    return {'island': island, 'right_outer': right_outer, 'right_inner': right_inner,
            'left_inner': left_inner, 'left_outer': left_outer, 'divider': divider,
            'min_clearance': min_clearance, 'lane_width': width,
            'max_strip_error': strip_error, 'max_spacing_error': spacing_error}
