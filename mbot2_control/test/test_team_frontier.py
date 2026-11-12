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

    def test_idle_moves_to_newer_known_junction_then_waits(self):
        start = self.node(-1, 0, 'START')
        near, far = self.node(0, 0), self.node(1, 0)
        self.join(start, EAST, near, WEST)
        self.join(near, EAST, far, WEST)
        result = self.ask('mbot1', near, 0.0, [WEST, EAST])
        self.assertEqual('TOWARD_WAITING_JUNCTION', result['reason'])
        self.assertEqual(EAST, result['direction'])
        result = self.ask('mbot1', far, 0.0, [WEST])
        self.assertEqual('WAIT_FOR_FRONTIER', result['reason'])
        self.assertIsNone(result['choice'])

    def test_idle_does_not_route_back_via_start(self):
        near = self.node(0, 0)
        start = self.node(-1, 0, 'START')
        far = self.node(-2, 0)
        self.join(start, EAST, near, WEST)
        self.join(start, WEST, far, EAST)
        self.assertIsNone(self.ask('mbot1', near, 0.0, [WEST])['choice'])

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

    def test_waits_if_only_branch_is_temporarily_avoided(self):
        j = self.node(0, 0)
        self.release('mbot1', j, EAST)
        result = self.ask('mbot1', j, math.pi, [EAST])
        self.assertIsNone(result['choice'])

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
        result = self.ask('mbot1', j, math.pi / 2, [EAST, NORTH])
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

    def test_waits_until_the_only_unknown_branch_is_released(self):
        j, k = self.node(0, 0), self.node(0, 1)
        self.join(j, NORTH, k, SOUTH)
        self.arm(j, EAST, status='BLOCKED')
        self.arm(k, NORTH)
        first = self.ask('mbot2', k, math.pi / 2, [NORTH])
        self.assertEqual(NORTH, first['direction'])
        self.assertIsNone(self.ask('mbot1', j, math.pi / 2, [NORTH, EAST])['choice'])
        self.release('mbot2', k, NORTH, peer='mbot1')
        self.assertEqual(NORTH, self.ask('mbot1', j, math.pi / 2, [NORTH, EAST])['direction'])

    def test_does_not_leave_via_a_corridor_that_only_loops_back_to_the_task(self):
        j, a, b = self.node(0, 0), self.node(1, 0), self.node(0, 1)
        self.join(j, EAST, a, WEST)
        self.join(j, NORTH, b, SOUTH)
        self.arm(b, NORTH)
        self.assertIsNone(self.ask('mbot1', j, math.pi / 2, [EAST])['choice'])

    def test_unfinished_exit_stays_available_after_explorer_returns(self):
        j = self.node(0, 0)
        first = self.ask('mbot1', j, math.pi / 2, [EAST])
        self.assertEqual(EAST, first['direction'])
        self.release('mbot1', j, EAST, peer='mbot3')

        # The first robot came back along the unfinished leg without linking it.
        self.assertIsNone(self.ask('mbot1', j, math.pi, [EAST])['choice'])

        second = self.ask('mbot2', j, math.pi / 2, [EAST])
        self.assertEqual(EAST, second.get('direction'))
        self.assertFalse(second['branches'][0]['endpoint_known'])

    def test_back_is_valid_transit_to_reachable_work(self):
        j, a = self.node(0, 0), self.node(-1, 0)
        self.join(j, WEST, a, EAST)
        self.arm(a, NORTH)
        self.arm(j, NORTH, status='BLOCKED')
        result = self.ledger.request_branch({
            'robot': 'mbot1', 'request_id': 'back', 'x': 0.0, 'y': 0.0,
            'approach_heading': 0.0,
            'choices': [{'name': 'back', 'heading': math.pi},
                        {'name': 'left', 'heading': math.pi / 2}],
        }, 10.0)
        self.assertEqual('back', result['choice'])
        self.assertEqual('TOWARD_UNEXPLORED', result['reason'])

    def test_front_robot_takes_work_reserved_by_the_fourth_robot_in_queue(self):
        j, k = self.node(0, 0), self.node(0, 1)
        self.join(j, NORTH, k, SOUTH)
        self.arm(k, EAST)
        for name, ahead in (('mbot2', 'mbot1'), ('mbot3', 'mbot2'), ('mbot4', 'mbot3')):
            self.ledger.update_pose({'robot': name, 'x': 0.0, 'y': 0.0,
                                     'waiting_for': ahead}, 10.0)
        self.ledger.frontier_claims['mbot4'] = (k['id'], EAST)

        result = self.ask('mbot1', j, math.pi / 2, [NORTH])

        self.assertEqual(NORTH, result.get('direction'))
        self.assertEqual('TAKE_QUEUED_TASK', result['reason'])
        self.assertNotIn('mbot4', self.ledger.frontier_claims)
        self.assertEqual(EAST, self.ledger.snapshot(10.0)['assignments']['mbot1']['direction'])

    def test_front_robot_cannot_take_work_already_being_explored(self):
        j, k = self.node(0, 0), self.node(0, 1)
        self.join(j, NORTH, k, SOUTH)
        self.arm(k, EAST)
        self.assertEqual(EAST, self.ask('mbot2', k, math.pi / 2, [EAST])['direction'])
        self.ledger.update_pose({'robot': 'mbot2', 'x': 0.0, 'y': 1.0,
                                 'waiting_for': 'mbot1'}, 10.0)

        self.assertIsNone(self.ask('mbot1', j, math.pi / 2, [NORTH])['choice'])

    def test_front_robot_takes_assignment_from_blocked_follower(self):
        j, k = self.node(0, 0), self.node(0, 1)
        self.join(j, NORTH, k, SOUTH)
        self.arm(k, EAST)
        self.ledger.trails['mbot2'] = (j['id'], NORTH)
        self.ledger.update_pose({'robot': 'mbot2', 'x': 0.0, 'y': 0.0,
                                 'waiting_for': 'mbot1'}, 10.0)
        self.assertIn('mbot2', self.ledger.snapshot(10.0)['assignments'])

        result = self.ask('mbot1', j, math.pi / 2, [NORTH])

        self.assertEqual('TAKE_QUEUED_TASK', result['reason'])
        assignments = self.ledger.snapshot(10.0)['assignments']
        self.assertNotIn('mbot2', assignments)
        self.assertEqual({'junction_id': k['id'], 'direction': EAST}, assignments['mbot1'])

    def test_robot_at_branch_can_take_en_route_reservation_but_not_active_claim(self):
        j, k = self.node(0, 0), self.node(0, 1)
        self.join(j, NORTH, k, SOUTH)
        self.arm(k, EAST)
        self.assertEqual(NORTH, self.ask('mbot2', j, math.pi / 2, [NORTH])['direction'])
        self.assertEqual(EAST, self.ask('mbot1', k, -math.pi / 2, [EAST])['direction'])
        self.assertIsNone(self.ask('mbot2', k, math.pi / 2, [EAST])['choice'])

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
        self.assertEqual('N', result['exit_cardinal'])
        self.assertEqual(b['id'], result['next_node_id'])
        self.release('mbot1', j, NORTH)
        result = self.ask('mbot1', j, -math.pi / 2, [EAST, NORTH])
        self.assertEqual(EAST, result['direction'])

    def test_goal_waits_when_only_mapped_dead_end_points_toward_it(self):
        j, dead = self.node(0, 0), self.node(1, 0, 'DEAD_END')
        self.join(j, EAST, dead, WEST)
        self.ledger.set_goal({'robot': 'mbot2', 'x': 2.0, 'y': 0.0})
        result = self.ask('mbot1', j, math.pi / 2, [EAST])
        self.assertIsNone(result['choice'])
        self.assertEqual('WAIT_FOR_GOAL_ROUTE', result['reason'])

    def test_goal_route_can_use_back_without_entering_dead_end(self):
        j, dead, west = self.node(0, 0), self.node(1, 0, 'DEAD_END'), self.node(-1, 0)
        self.ledger.set_goal({'robot': 'mbot2', 'x': 2.0, 'y': 0.0})
        goal = next(n for n in self.ledger.junctions.values() if n['kind'] == 'GOAL')
        self.join(j, EAST, dead, WEST)
        self.join(j, WEST, west, EAST)
        self.join(west, NORTH, goal, WEST)
        result = self.ledger.request_branch({
            'robot': 'mbot1', 'request_id': 'goal-back', 'x': 0.0, 'y': 0.0,
            'approach_heading': 0.0,
            'choices': [{'name': 'straight', 'heading': 0.0},
                        {'name': 'back', 'heading': math.pi}],
        }, 10.0)
        self.assertEqual('back', result['choice'])
        self.assertEqual('ROUTE_TO_GOAL', result['reason'])

    def test_goal_route_respects_blocked_edge_in_direction_of_travel(self):
        j, k = self.node(0, 0), self.node(1, 0)
        self.ledger.set_goal({'robot': 'mbot2', 'x': 2.0, 'y': 0.0})
        goal = next(n for n in self.ledger.junctions.values() if n['kind'] == 'GOAL')
        self.join(j, EAST, k, WEST)
        self.join(k, EAST, goal, WEST)
        k['branches'][EAST]['status'] = 'BLOCKED'
        self.assertIsNone(self.ask('mbot1', j, math.pi / 2, [EAST])['choice'])
        k['branches'][EAST]['status'] = 'OPEN'
        self.assertEqual('ROUTE_TO_GOAL',
                         self.ask('mbot1', j, math.pi / 2, [EAST])['reason'])

    def test_shortcut_reported_after_goal_changes_next_junction_choice(self):
        b, c, turn = self.node(0, 0), self.node(0, 2), self.node(1, 2)
        goal = self.node(1, 0, 'GOAL')
        self.join(b, NORTH, c, SOUTH)
        self.join(c, EAST, turn, WEST)
        self.ledger.trails['mbot2'] = (turn['id'], SOUTH)
        self.ledger.set_goal({'robot': 'mbot2', 'x': 1.0, 'y': 0.0,
                              'approach_heading': -math.pi / 2})
        before = self.ask('mbot1', b, math.pi / 2, [EAST, NORTH])
        self.assertEqual(NORTH, before['direction'])

        self.ledger.trails['mbot3'] = (b['id'], EAST)
        self.ledger.set_goal({'robot': 'mbot3', 'x': 1.0, 'y': 0.0,
                              'approach_heading': 0.0})

        after = self.ask('mbot1', b, math.pi / 2, [EAST, NORTH])
        self.assertEqual(goal['id'], b['branches'][EAST]['leads_to']['node'])
        self.assertEqual(EAST, after['direction'])
        self.assertGreater(after['routing_revision'], before['routing_revision'])

    def test_shortcut_already_mapped_when_goal_is_first_found(self):
        b, c, d = self.node(0, 0), self.node(0, 2), self.node(1, 0)
        self.join(b, NORTH, c, SOUTH)
        self.join(c, EAST, d, NORTH)
        self.join(b, EAST, d, WEST)
        self.ledger.trails['mbot2'] = (d['id'], EAST)
        self.ledger.set_goal({'robot': 'mbot2', 'x': 2.0, 'y': 0.0,
                              'approach_heading': 0.0})

        result = self.ask('mbot1', b, math.pi / 2, [NORTH, EAST])

        self.assertEqual(EAST, result['direction'])
        self.assertEqual('ROUTE_TO_GOAL', result['reason'])

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
