"""Derive junction pavement and terrain from the final 3.2m lane corridors."""
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union, triangulate

from env.roundabout_geometry import _read, _write, _slice, _length


def _polygons(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == 'Polygon':
        return [geometry]
    return [p for part in getattr(geometry, 'geoms', ()) for p in _polygons(part)]


def _lines(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type in ('LineString', 'LinearRing'):
        return [geometry]
    return [line for part in getattr(geometry, 'geoms', ()) for line in _lines(part)]


def _polygon(points):
    p = Polygon(points)
    return p if p.is_valid else p.buffer(0)


def _network(net_path):
    tree = ET.parse(net_path)
    root = tree.getroot()
    lanes = {}
    strips = {}
    for edge in root.findall('edge'):
        for lane in edge.findall('lane'):
            points = _read(lane.get('shape'))
            lanes[lane.get('id')] = points
            line = LineString(points)
            half = float(lane.get('width', '3.2'))/2
            strips[lane.get('id')] = (line.buffer(half, cap_style=2, quad_segs=24)
                                     if line.length > 1e-8 else Point(points[0]).buffer(half))
    return tree, lanes, strips


def fit_junction_pavement(net_path):
    """Replace oversized junction outlines with unions of real lane-width strips."""
    tree, lanes, strips = _network(net_path)
    root = tree.getroot()
    edges = {e.get('id'): e for e in root.findall('edge')}
    connections = root.findall('connection')
    follow = {(c.get('from'), c.get('fromLane')): c for c in connections
              if c.get('from').startswith(':')}
    surfaces = {}
    for c in connections:
        if c.get('from').startswith(':') or not c.get('via'):
            continue
        node = edges[c.get('from')].get('to')
        pieces = surfaces.setdefault(node, [])
        for edge_name, index, at_end in ((c.get('from'), c.get('fromLane'), True),
                                          (c.get('to'), c.get('toLane'), False)):
            points = lanes[f'{edge_name}_{index}']
            length = _length(points)
            stub = _slice(points, max(0.0, length-0.05), length) if at_end else _slice(points, 0, min(0.05, length))
            pieces.append(LineString(stub).buffer(1.6, cap_style=2, quad_segs=24))
        current = c.get('via')
        while current:
            pieces.append(strips[current])
            edge, index = current.rsplit('_', 1)
            current = follow[(edge, index)].get('via')
    count = 0
    for node in root.findall('junction'):
        if node.get('id') not in surfaces:
            continue
        union = unary_union(surfaces[node.get('id')])
        polygons = _polygons(union)
        if polygons:
            # Intersecting movements connect the main junction polygon. Other
            # disconnected lane strips retain their own native lane geometry.
            polygon = max(polygons, key=lambda p: p.area)
            node.set('shape', _write(list(polygon.exterior.coords)[:-1]))
            count += 1
    tree.write(net_path, encoding='utf-8', xml_declaration=True)
    return count


def fit_terrain(net_path, deco_path, ring_radius, width, ring_lanes, geometry_checks=None):
    """Keep all islands/curbs/hatching outside roads and above junction drawing."""
    network, lane_points, strips = _network(net_path)
    road = unary_union(list(strips.values()))
    # The 0.25 m outline reaches the lane edge without painting inside it.
    protected = road.buffer(0.125, quad_segs=16)
    tree = ET.parse(deco_path)
    root = tree.getroot()
    original = list(root)
    faces = {}
    face_ids = {}
    before_overlap = 0.0
    for p in original:
        name = p.get('id', '')
        if p.tag != 'poly' or p.get('fill') != '1' or not name.startswith('isl_'):
            continue
        geometry = _polygon(_read(p.get('shape')))
        before_overlap += geometry.intersection(road).area
        fitted = geometry.difference(road.buffer(0.18, quad_segs=16))
        faces[name] = fitted
        if name.endswith('_land'):
            face_ids[name[:-5]+'_curb'] = fitted
        elif name.endswith('_island'):
            face_ids[name[:-7]+'_curb'] = fitted
        else:
            face_ids[name+'_curb'] = fitted
    solid_terrain = unary_union(list(faces.values())) if faces else Polygon()
    hatch_regions = {}
    hatch_seeds = {}
    # Closed pockets between lane corridors belong to the adjacent safety zone.
    # A tiny buffer joins numerical seams without changing the network itself.
    pockets = [_polygon(list(hole.coords))
               for polygon in _polygons(road.buffer(0.01, quad_segs=16))
               for hole in polygon.interiors]
    for p in original:
        name = p.get('id', '')
        if p.tag == 'poly' and name.endswith('_edge'):
            seed = _polygon(_read(p.get('shape')))
            hatch_seeds[name[:-5]] = seed
            adjacent = [pocket for pocket in pockets
                        if pocket.intersection(seed).area > min(seed.area, pocket.area) * 0.2
                        and pocket.area < seed.area * 4]
            # Open-ended medians only need their existing edge clearance filled.
            expanded = unary_union([seed, *adjacent]) if adjacent else seed.buffer(0.5, join_style=2)
            hatch_regions[name[:-5]] = expanded.difference(protected)

    def emit_like(element, geometry, fill):
        pieces = _polygons(geometry) if fill else _lines(geometry)
        emitted = 0
        for piece in pieces:
            if (piece.area if fill else piece.length) < (0.01 if fill else 0.025):
                continue
            if fill and piece.interiors:
                # SUMO polygons cannot encode holes; fill only interior triangles.
                triangles = [t for t in triangulate(piece) if piece.covers(t)]
            else:
                triangles = [piece]
            for shape in triangles:
                clone = ET.Element('poly', element.attrib)
                if emitted:
                    clone.set('id', f'{element.get("id")}_fit{emitted}')
                coords = list(shape.exterior.coords) if fill else list(shape.coords)
                clone.set('shape', _write(coords))
                root.append(clone)
                emitted += 1

    for p in original:
        root.remove(p)
    # Safety zones are painted asphalt, not holes exposing background grass.
    # These are neutral areas outside the actual drivable lane corridors.
    for name, region in hatch_regions.items():
        pavement = ET.Element('poly', {'id': name+'_pavement', 'color': '84,84,90',
                                      'fill': '1', 'layer': '5'})
        emit_like(pavement, region, True)
    central_layers = {'center_apron': '5', 'center_grass': '5.1',
                      'center_flower': '5.2', 'center_bush': '5.3', 'center_curb': '6'}
    clipped_elements = 0
    for p in original:
        name = p.get('id', '')
        if name in central_layers:
            p.set('layer', central_layers[name])
            root.append(p)
        elif name in faces:
            p.set('layer', '5')
            emit_like(p, faces[name], True)
            clipped_elements += 1
        elif name in face_ids:
            p.set('layer', '6.2')
            boundaries = unary_union([poly.boundary for poly in _polygons(face_ids[name])])
            emit_like(p, boundaries, False)
            clipped_elements += 1
        elif name.endswith('_edge') and name[:-5] in hatch_regions:
            p.set('layer', '6')
            outline = unary_union([LineString(poly.exterior.coords)
                                   for poly in _polygons(hatch_regions[name[:-5]])])
            emit_like(p, outline, False)
            clipped_elements += 1
        else:
            match = re.fullmatch(r'(.+)_h\d+', name)
            if match and match.group(1) in hatch_regions:
                p.set('layer', '6')
                allowed = hatch_regions[match.group(1)].difference(solid_terrain.buffer(0.15))
                points = _read(p.get('shape'))
                seed = hatch_seeds[match.group(1)]
                # Continue each stripe into the newly included gaps. Keep V tips
                # inside the original zone fixed so chevrons retain their shape.
                start, end = points[0], points[-1]
                dx, dy = end[0]-start[0], end[1]-start[1]
                length = math.hypot(dx, dy)
                if length > 1e-8:
                    reach = max(10.0, math.sqrt(allowed.area))
                    ux, uy = dx/length, dy/length
                    if seed.boundary.distance(Point(start)) < 0.35:
                        points[0] = (start[0]-ux*reach, start[1]-uy*reach)
                    if seed.boundary.distance(Point(end)) < 0.35:
                        points[-1] = (end[0]+ux*reach, end[1]+uy*reach)
                emit_like(p, LineString(points).intersection(allowed), False)
                clipped_elements += 1
            else:
                root.append(p)

    # Fill the central island as an exact concentric disk. It is above native
    # junction pavement so no junction can visually bite into the lower-left arc.
    inner_radius = ring_radius-ring_lanes*width/2
    for p in root.findall('poly'):
        if p.get('id') in ('center_apron', 'center_curb'):
            radius = inner_radius-0.125
            circle = [(radius*math.cos(2*math.pi*k/720), radius*math.sin(2*math.pi*k/720))
                      for k in range(721)]
            p.set('shape', _write(circle))

    terrain_overlap = sum(g.intersection(road).area for g in faces.values())
    if terrain_overlap > 1e-6:
        raise ValueError(f'Terrain still overlaps lane corridors: {terrain_overlap:.6f} m2')
    widths = sorted({float(l.get('width', '3.2')) for e in network.getroot().findall('edge')
                     for l in e.findall('lane')})
    report = {'lane_count': len(lane_points), 'lane_widths_m': widths,
              'terrain_elements_fitted': clipped_elements,
              'terrain_overlap_before_m2': before_overlap,
              'terrain_overlap_after_m2': terrain_overlap,
              'ring_center_radius_m': ring_radius, 'ring_inner_radius_m': inner_radius,
              'ring_outer_radius_m': ring_radius+ring_lanes*width/2,
              'simulation_run': False}
    report.update(geometry_checks or {})
    tree.write(deco_path, encoding='utf-8', xml_declaration=True)
    Path(deco_path).with_name('roundabout.geometry-report.json').write_text(
        json.dumps(report, indent=2), encoding='utf-8')
    return report
