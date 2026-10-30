"""Team assignment: idle robots get the nearest unexplored branch."""

import math
import unittest

from mbot2_control.team_coordinator import direction_key, TeamLedger

EAST, NORTH, WEST, SOUTH = (direction_key(a) for a in
                            (0.0, math.pi / 2, math.pi, -math.pi / 2))
HEADING = {EAST: 0.0, NORTH: math.pi / 2, WEST: math.pi, SOUTH: -math.pi / 2}


class TeamDispatchTest(unittest.TestCase):
    """J(0,0) -E- A(1,0) with a new east arm; J -N- B(0,1) with a new north arm."""

    def setUp(self):
        self.ledger = TeamLedger(match_radius=0.25)
        self.j = self.ledger._junction(0.0, 0.0)
        self.a = self.ledger._junction(1.0, 0.0)
        self.b = self.ledger._junction(0.0, 1.0)
        self.join(self.j, EAST, self.a, WEST)
        self.join(self.j, NORTH, self.b, SOUTH)
        self.arm(self.a, EAST)      # task A
        self.arm(self.b, NORTH)     # task B
        self.tokens = 0

    def arm(self, node, key):
        branch = self.ledger._branch(node, key, HEADING[key])
        return branch

    def join(self, a, ka, b, kb):
        for node, key, other, okey in ((a, ka, b, kb), (b, kb, a, ka)):
            branch = self.arm(node, key)
            branch['leads_to'] = {'node': other['id'], 'enter_dir': okey}
            branch['visited_by'] = ['mbot9']

    def place(self, robot, x, y, trail, returning=False, now=10.0):
        self.ledger.update_pose({'robot': robot, 'x': x, 'y': y,
                                 'returning': returning}, now)
        self.ledger.trails[robot] = (trail[0]['id'], trail[1])

    def task_of(self, robot):
        a = self.ledger.assignments.get(robot)
        return None if a is None else (a['junction_id'], a['direction'])

    def ask(self, robot, node, approach, keys, now=10.0):
        self.tokens += 1
        return self.ledger.request_branch({
            'robot': robot, 'request_id': f't{self.tokens}',
            'x': node['x'], 'y': node['y'], 'approach_heading': approach,
            'choices': [{'name': f'k{k}', 'heading': HEADING[k]} for k in keys]}, now)

    def test_single_idle_robot_gets_nearest_task(self):
        self.place('mbot1', 0.8, 0.0, (self.j, EAST))
        self.ledger._update_assignments(10.0)
        self.assertEqual((self.a['id'], EAST), self.task_of('mbot1'))

    def test_two_idle_robots_get_different_tasks(self):
        self.place('mbot1', 0.9, 0.0, (self.j, EAST))
        self.place('mbot2', 0.6, 0.0, (self.j, EAST))
        self.ledger._update_assignments(10.0)
        self.assertEqual((self.a['id'], EAST), self.task_of('mbot1'))
        self.assertEqual((self.b['id'], NORTH), self.task_of('mbot2'))

    def test_busy_robot_not_assigned(self):
        branch = self.a['branches'][EAST]
        branch['owner'], branch['status'] = 'mbot1', 'CLAIMED'
        self.place('mbot1', 1.2, 0.0, (self.a, EAST))
        self.ledger._update_assignments(10.0)
        self.assertIsNone(self.task_of('mbot1'))

    def test_reassigned_to_much_closer_idle_robot(self):
        branch = self.a['branches'][EAST]
        self.place('mbot2', 0.0, 0.8, (self.j, NORTH))
        branch['owner'] = 'mbot1'    # mbot1 busy exploring A-east... not a task
        self.ledger._update_assignments(10.0)
        branch['owner'] = None
        self.ledger.assignments['mbot2'] = {'junction_id': self.a['id'], 'direction': EAST}
        self.place('mbot1', 0.9, 0.0, (self.j, EAST))   # idle, right next to A
        self.ledger._update_assignments(10.0)
        self.assertEqual((self.a['id'], EAST), self.task_of('mbot1'))
        self.assertEqual((self.b['id'], NORTH), self.task_of('mbot2'))

    def test_no_reassignment_below_margin(self):
        self.ledger.update_pose({'robot': 'mbot2', 'x': 0.5, 'y': 0.0}, 10.0)
        self.ledger.trails['mbot2'] = (self.j['id'], EAST)
        self.ledger.assignments['mbot2'] = {'junction_id': self.a['id'], 'direction': EAST}
        self.place('mbot1', 0.7, 0.0, (self.j, EAST))   # only 0.2 m closer
        self.ledger._update_assignments(10.0)
        self.assertEqual((self.a['id'], EAST), self.task_of('mbot2'))

    def test_request_steers_toward_assigned_task(self):
        self.place('mbot2', 0.0, -0.3, (self.j, SOUTH))
        self.ledger.assignments['mbot2'] = {'junction_id': self.b['id'], 'direction': NORTH}
        self.ledger.update_pose({'robot': 'mbot1', 'x': 5, 'y': 5}, 10.0)
        result = self.ask('mbot2', self.j, math.pi / 2, [EAST, NORTH])
        self.assertEqual(NORTH, result['direction'])
        self.assertEqual('TOWARD_ASSIGNED', result['reason'])

    def test_assigned_branch_given_at_task_junction(self):
        self.place('mbot2', 0.0, 0.8, (self.j, NORTH))
        self.ledger.assignments['mbot2'] = {'junction_id': self.b['id'], 'direction': NORTH}
        result = self.ask('mbot2', self.b, math.pi / 2, [NORTH])
        self.assertEqual('ASSIGNED_BRANCH', result['reason'])
        self.assertIsNone(self.task_of('mbot2'))

    def test_unreachable_task_uses_normal_ranking(self):
        far = self.ledger._junction(5.0, 5.0)
        self.arm(far, EAST)
        for node, key in ((self.a, EAST), (self.b, NORTH)):  # the only task is far
            node['branches'][key]['visited_by'] = ['mbot9']
        self.place('mbot2', 0.0, -0.3, (self.j, SOUTH))
        self.ledger.assignments['mbot2'] = {'junction_id': far['id'], 'direction': EAST}
        result = self.ask('mbot2', self.j, math.pi / 2, [EAST, NORTH])
        self.assertNotIn(result['reason'], ('TOWARD_ASSIGNED', 'ASSIGNED_BRANCH'))

    def test_state_lists_assignments(self):
        self.place('mbot1', 0.8, 0.0, (self.j, EAST))
        self.ledger._update_assignments(10.0)
        self.assertIn('mbot1', self.ledger.snapshot(10.0)['assignments'])

    def test_no_assignments_after_goal(self):
        self.place('mbot1', 0.8, 0.0, (self.j, EAST))
        self.ledger._update_assignments(10.0)
        self.ledger.set_goal({'robot': 'mbot3', 'x': 3.0, 'y': 3.0})
        self.ledger._update_assignments(10.0)
        self.assertEqual({}, self.ledger.assignments)

    def test_avoided_only_route_falls_back_to_normal_ranking(self):
        # Only way to task B is J's north arm, which mbot2 just gave up.
        self.place('mbot2', 0.0, -0.3, (self.j, SOUTH))
        self.ledger.assignments['mbot2'] = {'junction_id': self.b['id'], 'direction': NORTH}
        self.ledger.avoid['mbot2'] = {(self.j['id'], NORTH): {
            'peer': 'mbot3', 'peer_trail': None, 'until': 99.0}}
        new_arm = self.arm(self.j, WEST)  # a fresh, unexplored arm at J
        self.assertFalse(new_arm['visited_by'])
        result = self.ask('mbot2', self.j, math.pi / 2, [EAST, NORTH, WEST])
        self.assertNotEqual('TOWARD_ASSIGNED', result['reason'])
        self.assertEqual(WEST, result['direction'])

    def test_assigned_robot_switches_to_much_closer_free_task(self):
        self.place('mbot2', 0.9, 0.0, (self.j, EAST))   # right next to A
        self.ledger.assignments['mbot2'] = {'junction_id': self.b['id'], 'direction': NORTH}
        self.ledger._update_assignments(10.0)
        self.assertEqual((self.a['id'], EAST), self.task_of('mbot2'))


if __name__ == '__main__':
    unittest.main()
