"""Static checks of generated roads; never start or advance a simulation."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from shapely.geometry import LineString, Polygon, Point
from shapely.ops import unary_union

from env.roundabout_geometry import _read, _offset, _check_bundle, _unit
from env.roundabout_markings import dash_style, divider_routes


BASE = Path(__file__).resolve().parents[1]/'env'/'sumo_roundabout'


class GeneratedRoadGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.net = ET.parse(BASE/'roundabout.net.xml').getroot()
        cls.deco = ET.parse(BASE/'roundabout.deco.xml').getroot()
        cls.lanes = {lane.get('id'): lane for edge in cls.net.findall('edge')
                     for lane in edge.findall('lane')}
        cls.points = {name: _read(lane.get('shape')) for name, lane in cls.lanes.items()}
        cls.road = unary_union([LineString(p).buffer(1.6, cap_style=2, quad_segs=24)
                               for p in cls.points.values()])

    def test_every_lane_keeps_its_full_3_2m_strip_without_folding(self):
        # Inner-to-outer ring connections change lane inside a short junction; they
        # lie wholly on the junction surface and are not parallel lane strips.
        crossing = {c.get('via') for c in self.net.findall('connection')
                    if c.get('from', '').startswith('ring') and c.get('to', '').startswith('ring')
                    and c.get('fromLane') != c.get('toLane') and c.get('via')}
        for name, lane in self.lanes.items():
            with self.subTest(lane=name):
                self.assertAlmostEqual(float(lane.get('width')), 3.2, places=6)
                if name in crossing:
                    continue
                path = self.points[name]
                error = _check_bundle([_offset(path, -1.6), path, _offset(path, 1.6)], 1.6)
                self.assertLess(error, 0.005)

    def test_all_connection_endpoints_meet(self):
        for c in self.net.findall('connection'):
            source = f'{c.get("from")}_{c.get("fromLane")}'
            target = c.get('via') or f'{c.get("to")}_{c.get("toLane")}'
            with self.subTest(source=source, target=target):
                self.assertLess(math.dist(self.points[source][-1], self.points[target][0]), 1e-5)

    def test_central_island_remains_a_complete_circle_above_junctions(self):
        for name in ('center_apron', 'center_curb'):
            p = next(p for p in self.deco.findall('poly') if p.get('id') == name)
            radii = [math.hypot(x, y) for x, y in _read(p.get('shape'))]
            self.assertLess(max(radii)-min(radii), 2e-6)
            self.assertAlmostEqual(min(radii), 16.675, places=5)
            self.assertGreater(float(p.get('layer')), 4)
            self.assertGreater(len(radii), 360)

    def test_ring_road_has_no_missing_sector_including_the_lower_left(self):
        for radius in (16.81, 20.0, 23.19):
            for k in range(720):
                a = 2*math.pi*k/720
                with self.subTest(radius=radius, angle=k/2):
                    self.assertTrue(self.road.covers(Point(radius*math.cos(a), radius*math.sin(a))))

    def test_outer_slip_lane_turns_one_way_without_an_s_bend(self):
        connections = self.net.findall('connection')
        following = {(c.get('from'), c.get('fromLane')): c for c in connections
                     if c.get('from').startswith(':')}
        path = list(self.points['DANBONG_diag_end_0'])
        current = next(c.get('via') for c in connections
                       if c.get('from') == 'DANBONG_diag_end' and c.get('fromLane') == '0')
        while current:
            path.extend(self.points[current][1:])
            edge, index = current.rsplit('_', 1)
            current = following[(edge, index)].get('via')
        reverse_turn = 0.0
        for a, b, c in zip(path, path[1:], path[2:]):
            if min(math.dist(a, b), math.dist(b, c)) < 1e-8:
                continue
            u, v = _unit(a, b), _unit(b, c)
            angle = math.degrees(math.atan2(u[0]*v[1]-u[1]*v[0], u[0]*v[0]+u[1]*v[1]))
            reverse_turn += max(0.0, angle)
        self.assertLess(reverse_turn, 0.25)

    def test_all_white_lane_dashes_have_identical_physical_length(self):
        target, _, _ = dash_style(20.0)
        marks = [p for p in self.deco.findall('poly') if p.get('id', '').startswith('lane_dash_')]
        self.assertGreater(len(marks), 200)
        for p in marks:
            length = LineString(_read(p.get('shape'))).length
            with self.subTest(mark=p.get('id')):
                self.assertAlmostEqual(length, target, delta=0.0001)

    def test_main_road_dash_gaps_continue_through_the_junction(self):
        dash, gap, _ = dash_style(20.0)
        from env.roundabout_config import MARKINGS
        routes = divider_routes(BASE/'roundabout.net.xml', junction_guides=MARKINGS['junction_guides'])
        for name in ('W_out0_0', 'W_in1_far_0'):
            route = LineString(routes[name])
            prefix = f'lane_dash_{name}_'
            marks = sorted([p for p in self.deco.findall('poly') if p.get('id', '').startswith(prefix)],
                           key=lambda p: int(p.get('id').rsplit('_', 1)[1]))
            self.assertGreater(len(marks), 20)
            for a, b in zip(marks, marks[1:]):
                end = Point(_read(a.get('shape'))[-1])
                start = Point(_read(b.get('shape'))[0])
                skipped = int(b.get('id').rsplit('_', 1)[1])-int(a.get('id').rsplit('_', 1)[1])-1
                self.assertAlmostEqual(route.project(start)-route.project(end),
                                       gap+skipped*(dash+gap), delta=0.0001)

    def test_islands_and_safety_paint_stay_outside_drivable_lanes(self):
        for p in self.deco.findall('poly'):
            name = p.get('id', '')
            if not (name.startswith('isl_') or name.startswith('right_island')):
                continue
            if '_pavement' in name:
                continue
            points = _read(p.get('shape'))
            if p.get('fill') == '1':
                area = Polygon(points).buffer(0)
            else:
                area = LineString(points).buffer(float(p.get('lineWidth', '0.1'))/2, cap_style=2)
            with self.subTest(element=name):
                self.assertLess(area.intersection(self.road).area, 0.001)

    def test_south_uturn_connects_inner_lanes_below_the_island(self):
        connection = next(c for c in self.net.findall('connection')
                          if c.get('from') == 'S_in1_far' and c.get('to') == 'S_out0_far')
        self.assertEqual((connection.get('fromLane'), connection.get('toLane')), ('1', '1'))
        self.assertEqual(connection.get('dir'), 't')
        follow = {(c.get('from'), c.get('fromLane')): c for c in self.net.findall('connection')
                  if c.get('from').startswith(':')}
        path, current = [], connection.get('via')
        while current:
            path.extend(self.points[current][1 if path else 0:])
            edge, index = current.rsplit('_', 1)
            current = follow[(edge, index)].get('via')
        self.assertGreater(LineString(path).length, 15)
        island = next(p for p in self.deco.findall('poly') if p.get('id') == 'isl_S_0_island')
        self.assertFalse(LineString(path).buffer(1.6).intersects(Polygon(_read(island.get('shape')))))

    def test_northeast_t_junction_has_all_six_movements_without_crosswalks(self):
        from env.roundabout_config import T_JUNCTION
        if not T_JUNCTION:
            self.skipTest('Northeast T junction disabled')
        node = next(n for n in self.net.findall('junction') if n.get('id') == 'T_junction')
        self.assertEqual(node.get('type'), 'traffic_light')
        self.assertAlmostEqual(math.hypot(float(node.get('x')), float(node.get('y'))), 350, places=3)
        movements = {(c.get('from'), c.get('to')) for c in self.net.findall('connection')}
        for pair in [('E_out0_near', 'T_main_out'), ('E_out0_near', 'T_side_out'),
                     ('T_main_in', 'E_in1'), ('T_main_in', 'T_side_out'),
                     ('T_side_in', 'T_main_out'), ('T_side_in', 'E_in1')]:
            self.assertIn(pair, movements)
        self.assertFalse(any(e.get('function') == 'crossing' for e in self.net.findall('edge')))

    def test_far_endpoint_t_junction_is_perpendicular_and_connected(self):
        from env.roundabout_config import END_T_JUNCTION
        if not END_T_JUNCTION:
            self.skipTest('Far endpoint junction disabled')
        nodes = {n.get('id'): n for n in self.net.findall('junction')}
        def vector(destination):
            a, b = nodes['T_main_end'], nodes[destination]
            return (float(b.get('x'))-float(a.get('x')),
                    float(b.get('y'))-float(a.get('y')))
        main, side = vector('T_end_main_end'), vector('T_end_side_end')
        self.assertAlmostEqual(sum(a*b for a, b in zip(main, side)), 0, delta=0.1)
        moves = {(c.get('from'), c.get('to')) for c in self.net.findall('connection')}
        for fr, to in [('T_end_main_near', 'T_end_main_out'),
                       ('T_end_main_in', 'T_main_in'), ('T_end_main_in', 'T_end_side_out'),
                       ('T_end_side_in', 'T_main_in'), ('T_end_side_in', 'T_end_main_out')]:
            self.assertIn((fr, to), moves)
        outgoing = next(e for e in self.net.findall('edge') if e.get('id') == 'T_end_main_out')
        self.assertEqual(len(outgoing.findall('lane')), 3)
        straight = {(c.get('fromLane'), c.get('toLane')) for c in self.net.findall('connection')
                    if c.get('from') == 'T_end_main_near' and c.get('to') == 'T_end_main_out'}
        self.assertEqual(straight, {('0', '0'), ('1', '1'), ('2', '2')})

    def test_stop_line_spacing_matches_new_t_junction_setting(self):
        from env.roundabout_config import T_JUNCTION, STOP_T_JUNCTION
        if not STOP_T_JUNCTION:
            self.skipTest('Stop-line T junction disabled')
        node = next(n for n in self.net.findall('junction') if n.get('id') == 'T_stop_junction')
        angle = math.radians(T_JUNCTION['axis'])
        u = math.cos(angle), math.sin(angle)
        station = float(node.get('x'))*u[0]+float(node.get('y'))*u[1]
        if STOP_T_JUNCTION.get('clear_junction_markings') and not STOP_T_JUNCTION.get('keep_stop_line'):
            self.assertFalse(any(p.get('id') == 't_stop_end_main_out' for p in self.deco))
            stop_station = T_JUNCTION['distance']+T_JUNCTION['main_extension']+50.0
            self.assertAlmostEqual(station-stop_station, STOP_T_JUNCTION['ahead_of_stop'], places=3)
        else:
            stop = next(p for p in self.deco if p.get('id') == 't_stop_end_main_out')
            for x, y in _read(stop.get('shape')):
                self.assertAlmostEqual(station-(x*u[0]+y*u[1]), STOP_T_JUNCTION['ahead_of_stop'], places=3)
        moves = {(c.get('from'), c.get('to')) for c in self.net.findall('connection')}
        for pair in [('T_end_main_out','T_stop_main_out'), ('T_end_main_out','T_stop_side_out'),
                     ('T_stop_main_in','T_end_main_in'), ('T_stop_main_in','T_stop_side_out'),
                     ('T_stop_side_in','T_stop_main_out'), ('T_stop_side_in','T_end_main_in')]:
            self.assertIn(pair, moves)

    def test_end_crossroads_has_four_arms_and_all_twelve_turn_movements(self):
        node=next(n for n in self.net.findall('junction') if n.get('id')=='T_end_main_end')
        self.assertEqual(node.get('type'),'traffic_light')
        approaches={'T_stop_main_out','X_main_in','X_nw_in','X_se_in'}
        destinations={'T_stop_main_in','X_main_out','X_nw_out','X_se_out'}
        connections={(c.get('from'),c.get('to')) for c in self.net.findall('connection')
                     if c.get('from') in approaches and c.get('to') in destinations}
        self.assertEqual(len(connections),12)
        for approach in approaches:
            self.assertEqual(sum(fr==approach for fr,to in connections),3)
        for edge,count in [('T_stop_main_out',3),('X_main_out',2),
                           ('T_stop_main_in',2),('X_main_in',3)]:
            item=next(e for e in self.net.findall('edge') if e.get('id')==edge)
            self.assertEqual(len(item.findall('lane')),count)

    def test_crossroads_straight_guides_align_with_right_arm(self):
        for label,index in [('outer',0),('inner',1)]:
            boundary=LineString(_offset(self.points[f'X_main_out_{index}'],1.6))
            a,b=boundary.coords[0],boundary.coords[-1]
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(dx,dy)
            marks=[p for p in self.deco if p.get('id','').startswith(f'x_straight_guide_{label}_')]
            self.assertGreater(len(marks),5)
            for mark in marks:
                for x,y in _read(mark.get('shape')):
                    self.assertLess(abs(dx*(y-a[1])-dy*(x-a[0]))/length,0.00001)



    def test_final_crossroads_connects_every_arm(self):
        incoming={'SE_triangle_main_out','SE_triangle_forward_in',
                  'SE_triangle_NE_in','SE_triangle_cross_in'}
        outgoing={'SE_triangle_main_in','SE_triangle_forward_out',
                  'SE_triangle_NE_out','SE_triangle_cross_out'}
        moves=[c for c in self.net.findall('connection')
               if c.get('from') in incoming and c.get('to') in outgoing]
        self.assertEqual(len({(c.get('from'),c.get('to')) for c in moves}),12)
        for edge in incoming:
            self.assertEqual(len({c.get('to') for c in moves if c.get('from')==edge}),3)
        self.assertTrue(all(c.get('tl')=='SE_triangle_corner' for c in moves))


if __name__ == '__main__':
    unittest.main()
