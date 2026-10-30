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


if __name__ == '__main__':
    unittest.main()
