"""Released corridors and steering toward unexplored branches or the goal."""

import math
import unittest

from mbot2_control.team_coordinator import direction_key, TeamLedger

EAST, NORTH, WEST, SOUTH = (direction_key(a) for a in
                            (0.0, math.pi / 2, math.pi, -math.pi / 2))
HEADING = {EAST: 0.0, NORTH: math.pi / 2, WEST: math.pi, SOUTH: -math.pi / 2}


class TeamFrontierTest(unittest.TestCase):
    def setUp(self):
        self.ledger = TeamLedger(match_radius=0.25)
        for robot in ('mbot1', 'mbot2'):
            self.ledger.update_pose({'robot': robot, 'x': 0.0, 'y': 0.0}, 10.0)
        self.tokens = 0

    def node(self, x, y, kind='JUNCTION'):
        return self.ledger._junction(float(x), float(y), kind)

    def arm(self, node, key, visited=(), status='OPEN'):
        branch = self.ledger._branch(node, key, HEADING[key])
        branch['visited_by'] = list(visited)
        branch['status'] = status
        return branch

    def join(self, a, ka, b, kb):
        self.arm(a, ka)['leads_to'] = {'node': b['id'], 'enter_dir': kb}
        self.arm(b, kb)['leads_to'] = {'node': a['id'], 'enter_dir': ka}

    def ask(self, robot, node, approach, keys, now=10.0):
        self.tokens += 1
        return self.ledger.request_branch({
            'robot': robot, 'request_id': f't{self.tokens}',
            'x': node['x'], 'y': node['y'], 'approach_heading': approach,
            'choices': [{'name': f'k{k}', 'heading': HEADING[k]} for k in keys]}, now)

    # Task 1 ---------------------------------------------------------------

    def release(self, robot, node, key, peer='mbot2', now=10.0):
        self.ledger.release_branch({'robot': robot, 'junction_id': node['id'],
                                    'direction': key, 'peer': peer}, now)

    def test_release_keeps_visited(self):
        j = self.node(0, 0)
        first = self.ask('mbot1', j, 0.0, [EAST, NORTH])
        self.release('mbot1', j, first['direction'])
        branch = j['branches'][first['direction']]
        self.assertIsNone(branch['owner'])
        self.assertEqual('OPEN', branch['status'])
        self.assertIn('mbot1', branch['visited_by'])

    def test_released_branch_avoided(self):
        j = self.node(0, 0)
        self.arm(j, NORTH, visited=['mbot2'])
        self.release('mbot1', j, EAST)
        result = self.ask('mbot1', j, math.pi, [EAST, NORTH])
        self.assertEqual(NORTH, result['direction'])

    def test_avoided_branch_used_when_nothing_else(self):
        j = self.node(0, 0)
        self.release('mbot1', j, EAST)
        result = self.ask('mbot1', j, math.pi, [EAST])
        self.assertEqual(EAST, result['direction'])

    def test_release_keeps_teammates_claim(self):
        j = self.node(0, 0)
        branch = self.arm(j, EAST, visited=['mbot2'])
        branch['owner'], branch['status'] = 'mbot2', 'CLAIMED'
        self.release('mbot1', j, EAST)
        self.assertEqual('mbot2', branch['owner'])
        self.assertEqual('CLAIMED', branch['status'])

    def test_avoid_cleared_when_peer_moves_on(self):
        j, k = self.node(0, 0), self.node(1, 0)
        self.ledger.trails['mbot2'] = (k['id'], WEST)
        self.arm(j, NORTH, visited=['mbot2'])
        self.release('mbot1', j, EAST)
        self.ledger.trails['mbot2'] = (j['id'], SOUTH)  # mbot2 left the corridor
        result = self.ask('mbot1', j, math.pi, [EAST, NORTH])
        self.assertEqual(EAST, result['direction'])

    # Task 2 ---------------------------------------------------------------

    def test_likely_known_ranks_below_new(self):
        # J's east arm points straight at K, whose west arm is still unlinked.
        j, k = self.node(0, 0), self.node(0.5, 0)
        self.arm(k, WEST)
        result = self.ask('mbot1', j, math.pi / 2, [EAST, NORTH])  # from the south
        self.assertEqual(NORTH, result['direction'])
        self.assertEqual('UNEXPLORED_BRANCH', result['reason'])
        alone = self.ask('mbot2', j, math.pi / 2, [EAST])
        self.assertEqual('LIKELY_KNOWN_BRANCH', alone['reason'])

    def visited_cross(self):
        # J(0,0): east -> A(1,0) dead-ish explored; north -> B(0,1) which still
        # has a new arm; both of J's arms already driven.
        j, a, b = self.node(0, 0), self.node(1, 0), self.node(0, 1)
        self.join(j, EAST, a, WEST)
        self.join(j, NORTH, b, SOUTH)
        for node, key in ((j, EAST), (j, NORTH), (a, WEST), (b, SOUTH)):
            node['branches'][key]['visited_by'] = ['mbot2']
        self.arm(b, NORTH)              # new arm beyond B
        self.arm(a, EAST, visited=['mbot2'])
        return j, a, b

    def test_all_visited_heads_to_nearest_frontier(self):
        j, a, b = self.visited_cross()
        result = self.ask('mbot1', j, math.pi / 2 + math.pi, [EAST, NORTH])
        self.assertEqual(NORTH, result['direction'])
        self.assertEqual('TOWARD_UNEXPLORED', result['reason'])

    def test_frontier_route_skips_blocked(self):
        j, a, b = self.visited_cross()
        j['branches'][NORTH]['status'] = 'BLOCKED'
        c = self.node(2, 0)
        self.join(a, EAST, c, WEST)
        a['branches'][EAST]['visited_by'] = ['mbot2']
        self.arm(c, NORTH)
        result = self.ask('mbot1', j, -math.pi / 2, [EAST, NORTH])
        self.assertEqual(EAST, result['direction'])

    def test_goal_by_map_distance(self):
        j, a, b = self.visited_cross()
        goal = self.node(0, 2, 'GOAL')
        self.join(b, NORTH, goal, SOUTH)
        self.ledger.goal = {'robot': 'mbot2', 'x': 0.0, 'y': 2.0}
        result = self.ask('mbot1', j, -math.pi / 2, [EAST, NORTH])
        self.assertEqual(NORTH, result['direction'])
        self.assertEqual('ROUTE_TO_GOAL', result['reason'])
        self.release('mbot1', j, NORTH)
        result = self.ask('mbot1', j, -math.pi / 2, [EAST, NORTH])
        self.assertEqual(EAST, result['direction'])

    def test_goal_not_on_map_uses_heading(self):
        j, a, b = self.visited_cross()
        self.ledger.goal = {'robot': 'mbot2', 'x': 3.0, 'y': 0.0}
        result = self.ask('mbot1', j, -math.pi / 2, [EAST, NORTH])
        self.assertEqual(EAST, result['direction'])
        self.assertEqual('TOWARD_GOAL', result['reason'])

    # Task C3: goal routes ------------------------------------------------

    def goal_map(self):
        # P(0,0): north -> goal G(0,1); east arm is where mbot1 explores.
        p, g = self.node(0, 0), self.node(0, 1, 'GOAL')
        self.join(p, NORTH, g, SOUTH)
        self.ledger.goal = {'robot': 'mbot2', 'x': 0.0, 'y': 1.0}
        self.ledger.update_pose({'robot': 'mbot1', 'x': 0.3, 'y': 0.0}, 10.0)
        self.ledger.trails['mbot1'] = (p['id'], EAST)
        return p, g

    def route_for(self, robot, orders):
        return next(o for o in orders if o['robot'] == robot)

    def test_goal_route_turn_back_when_branch_unknown(self):
        self.goal_map()
        orders = self.ledger.goal_routes(10.0)
        self.assertTrue(self.route_for('mbot1', orders)['turn_back'])
        self.assertEqual('GOAL_ROUTE', orders[0]['type'])

    def test_goal_route_keeps_going_when_ahead_is_shorter(self):
        p, g = self.goal_map()
        n, g2 = self.node(0.5, 0), self.node(0.5, 0.3)
        self.join(p, EAST, n, WEST)
        self.join(n, NORTH, g2, SOUTH)
        self.join(g2, NORTH, g, EAST)
        # Robot near N: ahead 0.05 + 0.3 + 0.86 m beats back 0.45 + 1.0 m.
        self.ledger.update_pose({'robot': 'mbot1', 'x': 0.45, 'y': 0.0}, 10.0)
        orders = self.ledger.goal_routes(10.0)
        self.assertFalse(self.route_for('mbot1', orders)['turn_back'])

    def test_goal_routes_skip_finder_and_returning(self):
        self.goal_map()
        self.ledger.update_pose({'robot': 'mbot3', 'x': 0.3, 'y': 0.0,
                                 'returning': True}, 10.0)
        self.ledger.trails['mbot3'] = self.ledger.trails['mbot1']
        orders = self.ledger.goal_routes(10.0)
        self.assertNotIn('mbot2', [o['robot'] for o in orders])
        self.assertFalse(self.route_for('mbot3', orders)['turn_back'])

    def test_goal_next_to_a_junction_gets_its_own_node(self):
        # First red seen 7 cm past a junction: still a separate GOAL node.
        j = self.node(-1.25, 0.75)
        self.ask('mbot1', j, -math.pi / 2, [SOUTH])
        self.ledger.set_goal({'robot': 'mbot1', 'x': -1.25, 'y': 0.68,
                              'approach_heading': -math.pi / 2})
        goals = [n for n in self.ledger.junctions.values() if n['kind'] == 'GOAL']
        self.assertEqual(1, len(goals))
        self.assertEqual('JUNCTION', j['kind'])
        self.assertEqual(goals[0]['id'], j['branches'][SOUTH]['leads_to']['node'])

    def test_two_free_robots_head_for_different_frontiers(self):
        # From J both arms are explored; B (north, 1 m) and C (east, 1.5 m)
        # each still have a new arm. The second robot must not chase B too.
        j, a, b = self.visited_cross()
        c = self.node(1.5, 0)
        self.join(a, EAST, c, WEST)
        a['branches'][EAST]['visited_by'] = ['mbot2']
        c['branches'][WEST]['visited_by'] = ['mbot2']
        self.arm(c, EAST)
        first = self.ask('mbot1', j, -math.pi / 2, [EAST, NORTH])
        second = self.ask('mbot2', j, -math.pi / 2, [EAST, NORTH])
        self.assertEqual(NORTH, first['direction'])
        self.assertEqual(EAST, second['direction'])
        self.assertEqual('TOWARD_UNEXPLORED', second['reason'])


if __name__ == '__main__':
    unittest.main()
