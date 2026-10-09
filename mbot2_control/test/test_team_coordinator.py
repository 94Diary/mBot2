"""Small deterministic checks for the multi-robot branch ledger."""

import math
import unittest

from mbot2_control.team_coordinator import TeamLedger


def request(robot, token, x=-2.0, y=0.0):
    return {
        'robot': robot, 'request_id': token, 'x': x, 'y': y,
        'choices': [
            {'name': 'straight', 'heading': 0.0},
            {'name': 'left', 'heading': math.pi / 2},
        ],
    }


class TeamLedgerTest(unittest.TestCase):
    def setUp(self):
        self.ledger = TeamLedger()
        self.ledger.update_pose({'robot': 'mbot1', 'x': -2, 'y': 0}, 10.0)
        self.ledger.update_pose({'robot': 'mbot2', 'x': -2, 'y': 0}, 10.0)

    def test_two_robots_choose_distinct_branches(self):
        first = self.ledger.request_branch(request('mbot1', 'a'), 10.0)
        second = self.ledger.request_branch(request('mbot2', 'b', -2.1), 10.0)
        self.assertEqual('straight', first['choice'])
        self.assertEqual('left', second['choice'])
        self.assertEqual(first['junction_id'], second['junction_id'])
        self.assertEqual(first['junction_x'], second['junction_x'])
        self.assertEqual(first['junction_y'], second['junction_y'])

    def test_duplicate_request_is_idempotent(self):
        first = self.ledger.request_branch(request('mbot1', 'a'), 10.0)
        repeat = self.ledger.request_branch(request('mbot1', 'a'), 11.0)
        self.assertEqual(first, repeat)

    def test_late_camera_detection_still_matches_same_junction(self):
        first = self.ledger.request_branch(
            request('mbot1', 'near', -2.0, 3.05), 10.0)
        second = self.ledger.request_branch(
            request('mbot2', 'late', -2.0, 3.82), 10.0)
        self.assertEqual(first['junction_id'], second['junction_id'])

    def test_blocked_branch_is_not_assigned_again(self):
        first = self.ledger.request_branch(request('mbot1', 'a'), 10.0)
        self.ledger.block_branch({'robot': 'mbot1',
                                  'junction_id': first['junction_id'],
                                  'direction': first['direction']})
        second = self.ledger.request_branch(request('mbot2', 'b'), 10.0)
        self.assertEqual('left', second['choice'])

    def test_goal_biases_new_branch(self):
        self.ledger.set_goal({'robot': 'mbot3', 'x': -2, 'y': 5})
        result = self.ledger.request_branch(request('mbot1', 'a'), 10.0)
        self.assertEqual('left', result['choice'])

    def test_stale_claim_can_be_reused(self):
        self.ledger.request_branch(request('mbot1', 'a'), 10.0)
        result = self.ledger.request_branch(request('mbot2', 'b'), 20.0)
        self.assertEqual('straight', result['choice'])

    def test_peer_state_is_shared_for_yield_priority(self):
        self.ledger.update_pose({
            'robot': 'mbot1', 'x': -2.0, 'y': 2.5,
            'heading': math.pi / 2, 'state': 'YIELDING'}, 12.0)
        peer = self.ledger.snapshot(12.0)['robots']['mbot1']
        self.assertEqual('YIELDING', peer['state'])
        self.assertAlmostEqual(math.pi / 2, peer['heading'])


if __name__ == '__main__':
    unittest.main()
