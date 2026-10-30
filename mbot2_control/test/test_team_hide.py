"""Head-on encounters: who hides where, and when the hider may come out."""

import math
import unittest

from mbot2_control.team_coordinator import direction_key, TeamLedger

EAST, NORTH, WEST, SOUTH = (direction_key(a) for a in
                            (0.0, math.pi / 2, math.pi, -math.pi / 2))


class TeamHideTest(unittest.TestCase):
    """Map: H(0,0.5) -S- C(0,0) corner -E- K(1,0); H also has E and N arms."""

    def setUp(self):
        self.ledger = TeamLedger(match_radius=0.25)
        self.h = self.ledger._junction(0.0, 0.5)
        self.c = self.ledger._junction(0.0, 0.0, 'CORNER')
        self.k = self.ledger._junction(1.0, 0.0)
        self.join(self.h, SOUTH, self.c, NORTH)
        self.join(self.c, EAST, self.k, WEST)
        for key in (EAST, NORTH):
            self.ledger._branch(self.h, key, key * math.pi / 2)
        self.place('mbot1', 0.2, 0.0, (self.c, EAST))
        self.place('mbot2', 0.6, 0.0, (self.k, WEST))

    def join(self, a, ka, b, kb):
        self.ledger._branch(a, ka, ka * math.pi / 2)['leads_to'] = {
            'node': b['id'], 'enter_dir': kb}
        self.ledger._branch(b, kb, kb * math.pi / 2)['leads_to'] = {
            'node': a['id'], 'enter_dir': ka}

    def place(self, robot, x, y, trail, now=10.0, order=None):
        report = {'robot': robot, 'x': x, 'y': y}
        if order is not None:
            report['route_order'] = order
        self.ledger.update_pose(report, now)
        self.ledger.trails[robot] = (trail[0]['id'], trail[1])

    def orders(self, kind, result):
        return [o for o in result if o['type'] == kind]

    def test_closer_robot_hides(self):
        # mbot1 is 0.1 + 0.5 m from H; mbot2 is 0.7 m from K's free arm.
        self.ledger._branch(self.k, SOUTH, -math.pi / 2)
        self.place('mbot1', 0.1, 0.0, (self.c, EAST))
        self.place('mbot2', 0.3, 0.0, (self.k, WEST))
        result = self.ledger.head_on('mbot1', 'mbot2', 10.0)
        yields = self.orders('YIELD', result)
        self.assertEqual(['mbot1'], [o['robot'] for o in yields])
        self.assertEqual(['mbot2'], [o['robot'] for o in self.orders('HOLD', result)])
        self.assertEqual(self.h['id'], yields[0]['hide_node'])

    def test_closer_robot_hides_other_way_round(self):
        self.ledger._branch(self.k, SOUTH, -math.pi / 2)  # K has a free arm
        self.place('mbot2', 0.9, 0.0, (self.k, WEST))
        result = self.ledger.head_on('mbot1', 'mbot2', 10.0)
        self.assertEqual(['mbot2'], [o['robot'] for o in self.orders('YIELD', result)])

    def test_equal_distance_uses_route_order(self):
        self.ledger._branch(self.k, SOUTH, -math.pi / 2)
        self.ledger.junctions[self.h['id']]['y'] = 0.3  # H 0.3 m above C
        self.place('mbot1', 0.2, 0.0, (self.c, EAST), order=1)
        self.place('mbot2', 0.5, 0.0, (self.k, WEST), order=2)
        result = self.ledger.head_on('mbot1', 'mbot2', 10.0)
        self.assertEqual(['mbot2'], [o['robot'] for o in self.orders('YIELD', result)])

    def test_only_robot_with_option_hides(self):
        result = self.ledger.head_on('mbot2', 'mbot1', 10.0)
        self.assertEqual(['mbot1'], [o['robot'] for o in self.orders('YIELD', result)])

    def test_no_option_no_orders(self):
        for key in (EAST, NORTH):
            del self.h['branches'][key]
        self.assertEqual([], self.ledger.head_on('mbot1', 'mbot2', 10.0))

    def test_hide_branch_order(self):
        self.h['branches'][NORTH]['status'] = 'BLOCKED'
        option = self.ledger._hide_option('mbot1', 'mbot2')
        self.assertEqual(NORTH, option['hide_dir'])
        self.h['branches'][NORTH]['status'] = 'OPEN'
        self.ledger.update_pose({'robot': 'mbot3', 'x': 0.0, 'y': 0.8}, 10.0)
        option = self.ledger._hide_option('mbot1', 'mbot2')
        self.assertEqual(EAST, option['hide_dir'])  # north arm is occupied
        self.assertNotEqual(SOUTH, option['hide_dir'])

    def test_route_goes_back_through_the_corner(self):
        option = self.ledger._hide_option('mbot1', 'mbot2')
        self.assertEqual([self.c['id'], self.h['id']], [s['node'] for s in option['route']])
        self.assertEqual(NORTH, option['route'][0]['exit_dir'])
        self.assertAlmostEqual(0.2 + 0.5, option['distance'])

    def test_hide_at_the_node_just_left(self):
        self.place('mbot1', 0.0, 0.3, (self.h, SOUTH))
        option = self.ledger._hide_option('mbot1', 'mbot2')
        self.assertEqual([self.h['id']], [s['node'] for s in option['route']])

    def test_duplicate_head_on_gives_no_new_orders(self):
        self.assertTrue(self.ledger.head_on('mbot1', 'mbot2', 10.0))
        self.assertEqual([], self.ledger.head_on('mbot2', 'mbot1', 10.1))

    def hide_mbot1(self):
        self.ledger.head_on('mbot1', 'mbot2', 10.0)
        return self.ledger.hidden('mbot1')

    def test_hidden_sends_go_and_marks_branch(self):
        result = self.hide_mbot1()
        self.assertEqual(['mbot2'], [o['robot'] for o in self.orders('GO', result)])
        hidden = [b for b in self.h['branches'].values() if b.get('hidden_by')]
        self.assertEqual(['mbot1'], [b['hidden_by'] for b in hidden])

    def test_hidden_sets_trail_to_hide_branch(self):
        self.hide_mbot1()
        node, key = self.ledger.trails['mbot1']
        self.assertEqual(self.h['id'], node)
        self.assertEqual('mbot1', self.h['branches'][key]['hidden_by'])

    def test_no_clear_before_holder_passes(self):
        self.hide_mbot1()
        self.place('mbot2', 0.1, 0.0, (self.k, WEST), now=12.0)
        self.assertEqual([], self.ledger.check_clear(12.0))

    def test_clear_after_holder_passes(self):
        self.hide_mbot1()
        self.place('mbot2', 0.0, 0.9, (self.h, NORTH), now=14.0)
        result = self.ledger.check_clear(14.0)
        self.assertEqual(['mbot1'], [o['robot'] for o in self.orders('CLEAR', result)])
        self.assertFalse(any(b.get('hidden_by') for b in self.h['branches'].values()))
        self.assertEqual([], self.ledger.check_clear(14.5))

    def test_request_branch_skips_hidden_branch(self):
        self.hide_mbot1()
        hide_key = self.ledger.trails['mbot1'][1]
        other = (NORTH if hide_key == EAST else EAST)
        result = self.ledger.request_branch({
            'robot': 'mbot2', 'request_id': 'r', 'x': 0.0, 'y': 0.5,
            'approach_heading': math.pi / 2,
            'choices': [{'name': 'a', 'heading': hide_key * math.pi / 2},
                        {'name': 'b', 'heading': other * math.pi / 2}]}, 11.0)
        self.assertEqual(other, result['direction'])

    def test_escaping_robot_is_not_asked_to_yield_again(self):
        self.ledger.head_on('mbot1', 'mbot2', 10.0)  # mbot1 escapes
        self.ledger._branch(self.k, SOUTH, -math.pi / 2)
        self.place('mbot3', 0.5, 0.0, (self.k, WEST))
        result = self.ledger.head_on('mbot1', 'mbot3', 10.5)
        self.assertNotIn('mbot1', [o['robot'] for o in self.orders('YIELD', result)])

    def test_two_escaping_robots_get_no_orders(self):
        # mbot1 hides from mbot2; mbot3 hides from mbot2 too; then mbot3 and
        # mbot1 meet while both are on their way to hide.
        self.ledger.head_on('mbot1', 'mbot2', 10.0)
        self.ledger._start_encounter(
            'mbot3', 'mbot2', {'route': [], 'hide_node': self.k['id'],
                               'hide_dir': SOUTH, 'hide_heading': -math.pi / 2})
        before = {k: dict(v) for k, v in self.ledger.encounters.items()}
        self.assertEqual([], self.ledger.head_on('mbot3', 'mbot1', 10.4))
        self.assertEqual(before, self.ledger.encounters)


if __name__ == '__main__':
    unittest.main()
