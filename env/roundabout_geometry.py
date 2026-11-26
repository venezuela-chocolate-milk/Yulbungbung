"""Keep adjacent lanes parallel through curves, including junction internals.

SUMO's width attribute does not constrain the spacing of independently generated
turn trajectories. Build each movement from one curve and offset its other lanes.
The safety-island slip lane is a separate corridor and retains its separation.
"""
import math
import xml.etree.ElementTree as ET


def _read(value):
    return [tuple(map(float, p.split(','))) for p in value.split()]


def _write(points):
    return ' '.join(f'{x:.6f},{y:.6f}' for x, y in points)


def _unit(a, b):
    length = math.dist(a, b)
    if length < 1e-10:
        return 1.0, 0.0
    return (b[0]-a[0])/length, (b[1]-a[1])/length


def _length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _circle_arc(start, end, radius):
    a = math.atan2(start[1], start[0])
    sweep = (math.atan2(end[1], end[0])-a) % (2*math.pi)
    count = max(2, math.ceil(radius*sweep/0.08))
    return [(radius*math.cos(a+sweep*k/count), radius*math.sin(a+sweep*k/count))
            for k in range(count+1)]


def _circle_tangent(point):
    radius = math.hypot(*point)
    return -point[1]/radius, point[0]/radius


def _offset(points, distance, first_tangent=None, last_tangent=None):
    """Intersect parallel segment lines: spacing is measured normal to the road."""
    if abs(distance) < 1e-12:
        return list(points)
    tangents = [_unit(a, b) for a, b in zip(points, points[1:])]
    out = []
    for i, (x, y) in enumerate(points):
        if i == 0:
            tx, ty = first_tangent or tangents[0]
            nx, ny = -ty, tx
        elif i == len(points)-1:
            tx, ty = last_tangent or tangents[-1]
            nx, ny = -ty, tx
        else:
            a, b = tangents[i-1], tangents[i]
            denominator = 1.0 + a[0]*b[0] + a[1]*b[1]
            if denominator < 0.05:
                raise ValueError('A reversing curve cannot carry parallel lanes')
            nx, ny = (-a[1]-b[1])/denominator, (a[0]+b[0])/denominator
        out.append((x+nx*distance, y+ny*distance))
    return out


def _curve(start, end, incoming, outgoing, max_offset=0.0, lane_half_width=1.6, clockwise=False):
    distance = math.dist(start, end)
    dot = max(-1.0, min(1.0, sum(a*b for a, b in zip(incoming, outgoing))))
    turn = math.acos(dot)
    handle = distance / (3.0 * math.cos(turn/4.0)**2)
    a = (start[0]+incoming[0]*handle, start[1]+incoming[1]*handle)
    b = (end[0]-outgoing[0]*handle, end[1]-outgoing[1]*handle)

    def curvature_limits(c, d):
        positive, negative, absolute = 0.0, 0.0, 0.0
        for k in range(101):
            t = k/100
            v = tuple(3*(1-t)**2*(c[j]-start[j])+6*(1-t)*t*(d[j]-c[j])
                      +3*t*t*(end[j]-d[j]) for j in (0, 1))
            w = tuple(6*(1-t)*(d[j]-2*c[j]+start[j])
                      +6*t*(end[j]-2*d[j]+c[j]) for j in (0, 1))
            speed = math.hypot(*v)
            if speed < 1e-9:
                continue
            curvature = (v[0]*w[1]-v[1]*w[0])/speed**3
            positive = max(positive, curvature)
            negative = min(negative, curvature)
            absolute = max(absolute, abs(curvature))
        return positive, negative, absolute

    # An inward offset must stay outside the curvature center. Fit asymmetric
    # handles when closely spaced exit nodes would otherwise fold an inner lane.
    if distance > 0.5:
        limit = 0.8/(max_offset+lane_half_width)
        if clockwise:
            limit = min(limit, 1e-8)
        negative_limit = -0.8/lane_half_width
        positive, negative, _ = curvature_limits(a, b)
        if positive > limit or negative < negative_limit:
            candidates = []
            for sa in (0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0, 1.3, 1.6, 2.0):
                for sb in (0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0, 1.3, 1.6, 2.0):
                    c = (start[0]+incoming[0]*distance*sa, start[1]+incoming[1]*distance*sa)
                    d = (end[0]-outgoing[0]*distance*sb, end[1]-outgoing[1]*distance*sb)
                    positive, negative, absolute = curvature_limits(c, d)
                    if positive < limit and negative > negative_limit:
                        candidates.append((absolute, sa+sb, c, d))
            if not candidates:
                raise ValueError('Not enough space for constant-width turn lanes')
            _, _, a, b = min(candidates)
    count = max(8, math.ceil((distance+2*handle)/0.1))
    points = []
    for k in range(count+1):
        t = k/count
        points.append(tuple((1-t)**3*start[j]+3*(1-t)**2*t*a[j]
                            +3*(1-t)*t*t*b[j]+t**3*end[j] for j in (0, 1)))
    points[0], points[-1] = start, end
    return points


def _slice(points, start, end):
    """Slice by physical distance, preserving all intervening geometry points."""
    out = []
    position = 0.0
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if length < 1e-12:
            continue
        lo, hi = max(start, position), min(end, position+length)
        if lo <= hi and hi >= position and lo <= position+length:
            for s in (lo, hi):
                t = (s-position)/length
                p = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                if not out or math.dist(out[-1], p) > 1e-9:
                    out.append(p)
        position += length
        if position >= end:
            break
    if len(out) < 2:
        out = [points[0], points[-1]]
    return out


def _resample(points, spacing=0.08):
    """Avoid microscopic final segments when cutting a smooth lane at a junction."""
    import bisect
    stations = [0.0]
    for a, b in zip(points, points[1:]):
        stations.append(stations[-1]+math.dist(a, b))
    count = max(2, math.ceil(stations[-1]/spacing))
    result = []
    for k in range(count+1):
        distance = stations[-1]*k/count
        i = min(max(bisect.bisect_right(stations, distance)-1, 0), len(points)-2)
        length = stations[i+1]-stations[i]
        t = (distance-stations[i])/length if length > 1e-12 else 0.0
        a, b = points[i], points[i+1]
        result.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
    result[0], result[-1] = points[0], points[-1]
    return result


def _check_bundle(paths, width):
    """Check every parallel segment, including sub-meter junction fragments."""
    worst = 0.0
    for right, left in zip(paths, paths[1:]):
        if len(right) != len(left):
            raise ValueError('Parallel lanes must share cross-sections')
        for a, b, c, d in zip(right, right[1:], left, left[1:]):
            length = math.dist(a, b)
            other_length = math.dist(c, d)
            if min(length, other_length) < 1e-9:
                continue
            vx, vy = (b[0]-a[0])/length, (b[1]-a[1])/length
            if vx*(d[0]-c[0])+vy*(d[1]-c[1]) < 0:
                raise ValueError('A lane offset folded back on itself')
            for p in (c, d):
                separation = vx*(p[1]-a[1])-vy*(p[0]-a[0])
                worst = max(worst, abs(separation-width))
    if worst > 0.005:
        raise ValueError(f'Lane-spacing error exceeds 5 mm: {worst:.6f} m')
    return worst


def enforce_lane_spacing(net_path, width=3.2, separated_edge=None, ring_radius=20.0, ring_lanes=2,
                         right_turn_lead=3.5, right_turn_merge=8.0):
    """Normalize compiled lane geometry while retaining all connection identities.

    Keeping the internal lane IDs and their split points preserves junction
    request/foe references. All changed lengths and internal-node positions follow
    the new geometry, and adjacent external/internal endpoints remain identical.
    """
    tree = ET.parse(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    lanes = {l.get('id'): l for e in edges.values() for l in e.findall('lane')}
    original = {name: _read(l.get('shape')) for name, l in lanes.items()}
    shapes = {name: list(p) for name, p in original.items()}
    connections = root.findall('connection')
    follow = {(c.get('from'), c.get('fromLane')): c for c in connections
              if c.get('from').startswith(':')}

    def slip_path(base):
        normals = _offset(base, 1.0)
        result, run = [], 0.0
        for k, p in enumerate(base):
            if k:
                run += math.dist(base[k-1], p)
            t = min(run/12.0, 1.0)
            gap = width*(6*t**5-15*t**4+10*t**3) if k > 1 else 0.0
            result.append((p[0]-(normals[k][0]-p[0])*(width+gap),
                           p[1]-(normals[k][1]-p[1])*(width+gap)))
        return result

    # A common normal section is also used at the ends of very short ring edges.
    checked_normal = []
    for eid, edge in edges.items():
        if edge.get('function') == 'internal':
            continue
        group = sorted(edge.findall('lane'), key=lambda l: int(l.get('index')))
        if eid == separated_edge:
            group = group[1:]
        if not group:
            continue
        base = original[group[0].get('id')]
        if eid.startswith('ring'):
            radius = ring_radius+(ring_lanes-1)*width/2
            base = _circle_arc(base[0], base[-1], radius)
        for i, lane in enumerate(group):
            shapes[lane.get('id')] = _offset(base, i*width)
            if eid.startswith('ring'):
                shapes[lane.get('id')] = _circle_arc(base[0], base[-1], radius-i*width)
        if eid.startswith('ring'):
            # Share angular stations even though the lane arc lengths differ.
            for i, lane in enumerate(group):
                scale = (radius-i*width)/radius
                shapes[lane.get('id')] = [(x*scale, y*scale) for x, y in base]
        checked_normal.append([shapes[l.get('id')] for l in group])
        if eid == separated_edge:
            # Use the same entrance cross-section before opening the island gap.
            shapes[f'{eid}_0'] = slip_path(base)

    movements = {}
    for c in connections:
        if c.get('from').startswith(':') or not c.get('via'):
            continue
        key = c.get('from'), c.get('to')
        movements.setdefault(key, []).append(c)

    # Remove tiny artificial S bends between two continuous road sections.
    # Align the downstream cross-section first, then join it over a 2m blend.
    for (source, dest), group in movements.items():
        if source.startswith('ring') or dest.startswith('ring') or len(group) < 2:
            continue
        if any(c.get('fromLane') != c.get('toLane') or c.get('dir') != 's' for c in group):
            continue
        index = 1 if dest == separated_edge else 0
        src = shapes[f'{source}_{index}']
        dst = shapes[f'{dest}_{index}']
        gap = math.dist(src[-1], dst[0])
        if not 1e-6 < gap < 0.75 or _length(dst) < 3.0:
            continue
        head, tail = _slice(dst, 0, 2.0), _slice(dst, 2.0, _length(dst))
        blend = _curve(src[-1], head[-1], _unit(src[-2], src[-1]),
                       _unit(head[-2], head[-1]), width if dest == separated_edge else (len(group)-1)*width)
        base = blend+tail[1:]
        for lane in edges[dest].findall('lane'):
            j = int(lane.get('index'))
            if dest == separated_edge and j == 0:
                shapes[lane.get('id')] = slip_path(base)
            else:
                shapes[lane.get('id')] = _offset(base, (j-index)*width)
        for c in group:
            shapes[f'{dest}_{c.get("toLane")}'][0] = shapes[f'{source}_{c.get("fromLane")}'][-1]

    # Design the slip road as one independent, long bend. Its lane-width strip
    # is fixed before any safety island is fitted; no varying island-gap offset
    # controls the lower bend or its main-road merge.
    gentle_turn_tail = None
    gentle_target = None
    if separated_edge:
        turn = next(c for c in connections if c.get('from') == separated_edge
                    and c.get('fromLane') == '0')
        gentle_target = turn.get('to')
        old_out = shapes[f'{gentle_target}_0']
        new_out = _slice(old_out, right_turn_merge, _length(old_out))
        for lane in edges[gentle_target].findall('lane'):
            shapes[lane.get('id')] = _offset(new_out, int(lane.get('index'))*width)
        old_slip = shapes[f'{separated_edge}_0']
        lead = _slice(old_slip, 0, right_turn_lead)
        upstream = next(c.get('from') for c in connections if c.get('to') == separated_edge
                        and not c.get('from').startswith(':') and c.get('toLane') == '0')
        approach = shapes[f'{upstream}_0']
        lead_tangent = _unit(lead[-2], lead[-1])
        lead = _curve(lead[0], lead[-1], _unit(approach[-2], approach[-1]),
                      lead_tangent, clockwise=True)
        bend = _curve(lead[-1], new_out[0], lead_tangent,
                      _unit(new_out[0], new_out[1]), clockwise=True)
        whole = lead+bend[1:]
        left = shapes[f'{separated_edge}_1']
        station, tangent = left[-1], _unit(left[-2], left[-1])
        for k, (a, b) in enumerate(zip(whole, whole[1:])):
            da = sum((a[j]-station[j])*tangent[j] for j in (0, 1))
            db = sum((b[j]-station[j])*tangent[j] for j in (0, 1))
            if da <= 0 < db:
                t = -da/(db-da)
                cut = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                shapes[f'{separated_edge}_0'] = _resample(whole[:k+1]+[cut])
                gentle_turn_tail = _resample([cut]+whole[k+1:])
                break
        if gentle_turn_tail is None:
            raise ValueError('Gentle slip road did not cross the stop-line reference section')

    checked_bundles = []
    crossing_lanes = set()   # inner-to-outer ring lanes: a lane change, not a parallel strip
    for (source, dest), group in movements.items():
        group.sort(key=lambda c: int(c.get('fromLane')))
        paths = []
        for c in group:
            current = c.get('via')
            chain = []
            while current:
                if current in chain:
                    raise ValueError(f'Internal connection cycle: {current}')
                chain.append(current)
                edge_id, index = current.rsplit('_', 1)
                continuation = follow[(edge_id, index)]
                current = continuation.get('via')
            paths.append(chain)

        def assign(c, chain, path):
            # Split one movement path over its internal lanes in original length ratio.
            old_lengths = [_length(original[name]) for name in chain]
            old_total = sum(old_lengths)
            total = _length(path)
            position = 0.0
            last_point = path[0]
            for k, name in enumerate(chain):
                finish = total if k == len(chain)-1 else position+total*old_lengths[k]/max(old_total, 1e-12)
                piece = _slice(path, position, finish)
                if c.get('dir') == 't':
                    piece = _resample(piece)
                piece[0] = last_point
                shapes[name] = piece
                last_point = piece[-1]
                position = finish

        if source.startswith('ring') and dest.startswith('ring'):
            # Inner-to-outer ring connections deliberately cross lanes. Draw them as
            # a smooth radius change along the circle and keep them out of the
            # parallel-bundle checks, which only apply to same-lane movements.
            kept = []
            for c, chain in zip(group, paths):
                if c.get('fromLane') == c.get('toLane'):
                    kept.append((c, chain))
                    continue
                p0 = shapes[f'{source}_{c.get("fromLane")}'][-1]
                p1 = shapes[f'{dest}_{c.get("toLane")}'][0]
                r0, r1 = math.hypot(*p0), math.hypot(*p1)
                a0, a1 = math.atan2(p0[1], p0[0]), math.atan2(p1[1], p1[0])
                a1 = a0+((a1-a0+math.pi) % (2*math.pi)-math.pi)
                n = max(int(abs(a1-a0)*max(r0, r1)/0.25), 8)
                path = []
                for k in range(n+1):
                    t = k/n
                    s = t*t*(3-2*t)
                    a, r = a0+(a1-a0)*t, r0+(r1-r0)*s
                    path.append((r*math.cos(a), r*math.sin(a)))
                path[0], path[-1] = p0, p1
                assign(c, chain, path)
                crossing_lanes.update(chain)
            group = [c for c, _ in kept]
            paths = [chain for _, chain in kept]
            if not group:
                continue

        first = group[0]
        src = shapes[f'{source}_{first.get("fromLane")}']
        dst = shapes[f'{dest}_{first.get("toLane")}']
        incoming, outgoing = _unit(src[-2], src[-1]), _unit(dst[0], dst[1])
        if source.startswith('ring'):
            incoming = _circle_tangent(src[-1])
        if dest.startswith('ring'):
            outgoing = _circle_tangent(dst[0])
        circular = source.startswith('ring') and dest.startswith('ring')
        if source == separated_edge and dest == gentle_target:
            reference = gentle_turn_tail
        elif circular:
            reference = _circle_arc(src[-1], dst[0], math.hypot(*src[-1]))
        else:
            try:
                reference = _curve(src[-1], dst[0], incoming, outgoing, (len(group)-1)*width)
            except ValueError:
                if not source.startswith('ring') or dest.startswith('ring'):
                    raise ValueError(f'Insufficient turn space: {source} -> {dest}')
                # Extend the exit's turning area when a full-width inner lane
                # cannot fit in the original automatically trimmed junction.
                old_base = shapes[f'{dest}_0']
                for trim in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0):
                    if _length(old_base) <= trim+3:
                        continue
                    candidate = _slice(old_base, trim, _length(old_base))
                    try:
                        reference = _curve(src[-1], candidate[0], incoming,
                                           _unit(candidate[0], candidate[1]), (len(group)-1)*width)
                    except ValueError:
                        continue
                    for lane in edges[dest].findall('lane'):
                        shapes[lane.get('id')] = _offset(candidate, int(lane.get('index'))*width)
                    dst = shapes[f'{dest}_{first.get("toLane")}']
                    outgoing = _unit(dst[0], dst[1])
                    break
                else:
                    raise ValueError(f'Unable to fit full-width exit: {source} -> {dest}')
        bundle = []
        for i, (c, chain) in enumerate(zip(group, paths)):
            src_lane = shapes[f'{source}_{c.get("fromLane")}']
            dst_lane = shapes[f'{dest}_{c.get("toLane")}']
            if math.dist(src_lane[-1], dst_lane[0]) < 1e-6:
                path = [src_lane[-1], dst_lane[0]]
            else:
                try:
                    path = _offset(reference, i*width, incoming, outgoing)
                except ValueError as error:
                    raise ValueError(f'{source} -> {dest}, lane {i}: {error}') from error
            if circular:
                radius = math.hypot(*reference[0])
                path = [(x*(radius-i*width)/radius, y*(radius-i*width)/radius)
                        for x, y in reference]
            if max(math.dist(path[0], src_lane[-1]), math.dist(path[-1], dst_lane[0])) > 0.02:
                # Distinct lane groups must not be silently forced into a strip.
                raise ValueError(f'Nonparallel movement endpoints: {source} -> {dest}')
            path[0], path[-1] = src_lane[-1], dst_lane[0]
            bundle.append(path)
            assign(c, chain, path)
        if len(bundle) > 1:
            checked_bundles.append((source, dest, bundle))

    errors = [_check_bundle(p, width) for p in checked_normal]
    for source, dest, paths in checked_bundles:
        try:
            errors.append(_check_bundle(paths, width))
        except ValueError as error:
            raise ValueError(f'{source} -> {dest}: {error}') from error
    spacing_error = max(errors)
    strip_error = 0.0
    for name, path in shapes.items():
        if name in crossing_lanes:
            # Shifts one lane width inside a short junction; it lies wholly on the
            # junction surface, so its full-width strip is never drawn on its own.
            continue
        try:
            error = _check_bundle([_offset(path, -width/2), path,
                                   _offset(path, width/2)], width/2)
        except ValueError as error:
            raise ValueError(f'Invalid full-width lane strip {name}: {error}') from error
        strip_error = max(strip_error, error)
    for name, lane in lanes.items():
        lane.set('width', f'{width:.6f}')
        lane.set('shape', _write(shapes[name]))
        lane.set('length', f'{max(_length(shapes[name]), 0.001):.6f}')
    for junction in root.findall('junction'):
        if junction.get('type') != 'internal':
            continue
        incoming_ids = junction.get('incLanes', '').split()
        if incoming_ids and incoming_ids[0] in shapes:
            x, y = shapes[incoming_ids[0]][-1]
            junction.set('x', f'{x:.6f}')
            junction.set('y', f'{y:.6f}')

    # Join the exposed shoulder to the diagonal road along the actual turn,
    # replacing the old junction outline's bulge and angular corner.
    if separated_edge:
        for c in connections:
            if c.get('from') != separated_edge or c.get('fromLane') != '0':
                continue
            junction_id = edges[c.get('to')].get('from')
            junction = next((j for j in root.findall('junction')
                             if j.get('id') == junction_id), None)
            if junction is None or not junction.get('shape'):
                continue
            path = []
            current = c.get('via')
            while current:
                path.extend(shapes[current][1 if path else 0:])
                edge_id, index = current.rsplit('_', 1)
                current = follow[(edge_id, index)].get('via')
            incoming = shapes[f'{separated_edge}_0']
            outgoing = shapes[f'{c.get("to")}_{c.get("toLane")}']
            outer = _offset(path, -width/2,
                            _unit(incoming[-2], incoming[-1]),
                            _unit(outgoing[0], outgoing[1]))
            outline = _read(junction.get('shape'))
            start = min(range(len(outline)), key=lambda k: math.dist(outline[k], outer[-1]))
            end = min(range(len(outline)), key=lambda k: math.dist(outline[k], outer[0]))
            if start < end:
                junction.set('shape', _write(outline[:start]+outer[::-1]+outline[end+1:]))

    # Validate topology without advancing SUMO or moving any vehicles.
    max_gap = 0.0
    for c in connections:
        source = f'{c.get("from")}_{c.get("fromLane")}'
        dest = c.get('via') or f'{c.get("to")}_{c.get("toLane")}'
        max_gap = max(max_gap, math.dist(shapes[source][-1], shapes[dest][0]))
    if max_gap > 1e-5:
        raise ValueError(f'Disconnected lane geometry: {max_gap:.6f} m')
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return {'parallel_movements': len(checked_bundles), 'max_endpoint_gap': max_gap,
            'max_spacing_error': spacing_error, 'max_full_width_strip_error': strip_error}
