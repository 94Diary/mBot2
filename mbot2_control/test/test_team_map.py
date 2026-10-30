"""Shared team map: nodes, branch links and shortest routes, without ROS."""

import math
import unittest

from mbot2_control.team_coordinator import direction_key, TeamLedger

EAST, NORTH, WEST = direction_key(0.0), direction_key(math.pi / 2), direction_key(math.pi)


class TeamMapTest(unittest.TestCase):
    def setUp(self):
        self.ledger = TeamLedger(match_radius=0.25)
        for robot in ('mbot1', 'mbot2'):
            self.ledger.update_pose({'robot': robot, 'x': 0, 'y': 0}, 10.0)
        self.tokens = 0

    def node(self, robot, kind, x, y, approach, **extra):
        return self.ledger.report_node(
            {'robot': robot, 'kind': kind, 'x': x, 'y': y,
             'approach_heading': approach, **extra}, 10.0)

    def junction(self, robot, x, y, approach, exit_heading):
        self.tokens += 1
        result = self.ledger.request_branch({
            'robot': robot, 'request_id': f't{self.tokens}', 'x': x, 'y': y,
            'approach_heading': approach,
            'choices': [{'name': 'go', 'heading': exit_heading}]}, 10.0)
        return self.ledger.junctions[result['junction_id']], result

    def leads(self, node, key):
        return node['branches'][key]['leads_to']

    def test_straight_walk_links_both_ends(self):
        start = self.node('mbot1', 'START', -1.0, 0.0, 0.0, heading=0.0)
        j1, _ = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        j2, _ = self.junction('mbot1', 0.5, 0.0, 0.0, 0.0)
        self.assertEqual({'node': j1['id'], 'enter_dir': WEST}, self.leads(start, EAST))
        self.assertEqual({'node': j2['id'], 'enter_dir': WEST}, self.leads(j1, EAST))
        self.assertEqual({'node': j1['id'], 'enter_dir': EAST}, self.leads(j2, WEST))

    def test_corner_sits_between_junctions(self):
        j1, _ = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        corner = self.node('mbot1', 'CORNER', 0.5, 0.0, 0.0, exit_heading=math.pi / 2)
        j2, _ = self.junction('mbot1', 0.5, 0.5, math.pi / 2, math.pi / 2)
        self.assertEqual('CORNER', corner['kind'])
        self.assertEqual(corner['id'], self.leads(j1, EAST)['node'])
        self.assertEqual(j2['id'], self.leads(corner, NORTH)['node'])
        self.assertEqual(corner['id'], self.leads(j2, direction_key(-math.pi / 2))['node'])

    def test_dead_end_node_and_blocked_branch(self):
        j1, result = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        self.ledger.block_branch({'robot': 'mbot1', 'junction_id': result['junction_id'],
                                  'direction': result['direction']})
        dead = self.node('mbot1', 'DEAD_END', 0.4, 0.0, 0.0)
        self.assertEqual('DEAD_END', dead['kind'])
        self.assertEqual('BLOCKED', j1['branches'][EAST]['status'])
        self.assertEqual(dead['id'], self.leads(j1, EAST)['node'])

    def test_two_robots_reach_same_junction_from_different_sides(self):
        west = self.node('mbot1', 'CORNER', -0.5, 0.0, math.pi / 2, exit_heading=0.0)
        east = self.node('mbot2', 'CORNER', 0.6, 0.0, math.pi / 2, exit_heading=math.pi)
        a, _ = self.junction('mbot1', 0.0, 0.0, 0.0, math.pi / 2)
        b, _ = self.junction('mbot2', 0.1, 0.0, math.pi, math.pi / 2)
        self.assertEqual(a['id'], b['id'])
        self.assertEqual(west['id'], self.leads(a, WEST)['node'])
        self.assertEqual(east['id'], self.leads(a, EAST)['node'])

    def test_conflicting_link_keeps_first(self):
        j1, _ = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        j2, _ = self.junction('mbot1', 0.5, 0.0, 0.0, 0.0)
        self.junction('mbot2', 0.0, 0.0, 0.0, 0.0)
        j3, _ = self.junction('mbot2', 1.5, 0.0, 0.0, 0.0)
        self.assertEqual(j2['id'], self.leads(j1, EAST)['node'])
        self.assertEqual(1, len(self.ledger.link_conflicts))

    def test_all_start_reports_share_one_node(self):
        ids = {self.node(robot, 'START', x, -1.25, 0.0, heading=0.0)['id']
               for robot, x in (('mbot1', -1.85), ('mbot2', -2.10), ('mbot3', -2.38))}
        self.assertEqual(1, len(ids))
        start = self.ledger.junctions[ids.pop()]
        self.assertAlmostEqual(-1.85, start['x'])

    def test_corner_upgrades_to_junction(self):
        corner = self.node('mbot1', 'CORNER', 0.0, 0.0, 0.0, exit_heading=math.pi / 2)
        node, _ = self.junction('mbot2', 0.05, 0.0, 0.0, 0.0)
        self.assertEqual(corner['id'], node['id'])
        self.assertEqual('JUNCTION', node['kind'])
        self.node('mbot1', 'CORNER', 0.0, 0.0, 0.0, exit_heading=math.pi / 2)
        self.assertEqual('JUNCTION', node['kind'])

    def test_goal_becomes_node(self):
        j1, _ = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        self.ledger.set_goal({'robot': 'mbot1', 'x': 0.5, 'y': 0.0,
                              'approach_heading': 0.0})
        goal = self.ledger.junctions[self.leads(j1, EAST)['node']]
        self.assertEqual('GOAL', goal['kind'])

    def test_missed_node_splits_the_edge(self):
        # mbot1 drove P -> C without noticing J between them; mbot2 sees J.
        p, _ = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        c = self.node('mbot1', 'CORNER', 1.0, 0.0, 0.0, exit_heading=math.pi / 2)
        self.junction('mbot2', 0.0, 0.0, 0.0, 0.0)
        j, _ = self.junction('mbot2', 0.5, 0.0, 0.0, 0.0)
        self.assertEqual(j['id'], self.leads(p, EAST)['node'])
        self.assertEqual(c['id'], self.leads(j, EAST)['node'])
        self.assertEqual(p['id'], self.leads(j, WEST)['node'])
        self.assertEqual(j['id'], self.leads(c, WEST)['node'])
        self.assertEqual([], self.ledger.link_conflicts)

    def test_cancelled_request_restores_trail(self):
        j1, _ = self.junction('mbot1', 0.0, 0.0, 0.0, 0.0)
        before = self.ledger.trails['mbot1']
        self.tokens += 1
        self.ledger.request_branch({
            'robot': 'mbot1', 'request_id': 'ahead', 'x': 0.5, 'y': 0.0,
            'approach_heading': 0.0,
            'choices': [{'name': 'go', 'heading': 0.0}]}, 10.0)
        self.assertNotEqual(before, self.ledger.trails['mbot1'])
        self.ledger.cancel_branch({'robot': 'mbot1', 'request_id': 'ahead'})
        self.assertEqual(before, self.ledger.trails['mbot1'])

    def square(self):
        """J1(0,0)-J2(1,0)-J3(1,1)-J4(0,1)-J1, plus a detour J2-J5-J6-J3."""
        ids = {}
        for name, x, y in (('J1', 0, 0), ('J2', 1, 0), ('J3', 1, 1),
                           ('J4', 0, 1), ('J5', 2, 0), ('J6', 2, 1)):
            ids[name] = self.ledger._junction(float(x), float(y))['id']

        def join(a, b):
            na, nb = self.ledger.junctions[ids[a]], self.ledger.junctions[ids[b]]
            heading = math.atan2(nb['y'] - na['y'], nb['x'] - na['x'])
            ka, kb = direction_key(heading), direction_key(heading + math.pi)
            self.ledger._branch(na, ka, heading)['leads_to'] = {
                'node': nb['id'], 'enter_dir': kb}
            self.ledger._branch(nb, kb, heading + math.pi)['leads_to'] = {
                'node': na['id'], 'enter_dir': ka}
        for a, b in (('J1', 'J2'), ('J2', 'J3'), ('J3', 'J4'), ('J4', 'J1'),
                     ('J2', 'J5'), ('J5', 'J6'), ('J6', 'J3')):
            join(a, b)
        return ids

    def test_route_prefers_shorter_side_of_loop(self):
        ids = self.square()
        route = self.ledger.route(ids['J1'], ids['J3'])
        self.assertEqual(2, len(route))
        self.assertIn(route[1]['node'], (ids['J2'], ids['J4']))
        self.assertEqual(ids['J1'], route[0]['node'])

    def test_route_avoids_blocked_branch(self):
        ids = self.square()
        j1 = self.ledger.junctions[ids['J1']]
        j1['branches'][NORTH]['status'] = 'BLOCKED'
        route = self.ledger.route(ids['J1'], ids['J4'])
        self.assertEqual([ids['J1'], ids['J2'], ids['J3']], [s['node'] for s in route])
        self.assertEqual(EAST, route[0]['exit_dir'])
        self.assertAlmostEqual(0.0, route[0]['heading'])

    def test_route_unreachable_is_none(self):
        ids = self.square()
        lone = self.ledger._junction(5.0, 5.0)['id']
        self.assertIsNone(self.ledger.route(ids['J1'], lone))

    def test_route_to_self_is_empty(self):
        ids = self.square()
        self.assertEqual([], self.ledger.route(ids['J1'], ids['J1']))


if __name__ == '__main__':
    unittest.main()
