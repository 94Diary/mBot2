"""State-machine checks for junction reading and line recovery, without Gazebo."""

import math
import time
import unittest

import rclpy

from mbot2_control import maze_solver
from mbot2_control.maze_solver import MazeSolver


class SolverHarness:
    """Drive a real MazeSolver with fake sensor readings and simple kinematics."""

    def __init__(self, node):
        self.node = node
        self.sent = []
        node.cmd_pub.publish = self.sent.append
        node.ultrasonic.ready = True
        node.ultrasonic.distance_m = None
        node.sensors.seen = [True] * 4

    def set_colours(self, black=(False,) * 4, junction=(False,) * 4):
        self.node.sensors.black = list(black)
        self.node.sensors.yellow = list(junction)

    def tick(self, dt=0.1):
        now = time.monotonic()
        self.node.sensors.last_seen_at = [now] * 4
        self.node.ultrasonic.last_seen_at = now
        self.node.last_odom_at = now
        self.node.last_team_state_at = now
        self.node.tick()
        cmd = self.sent[-1]
        x, y, yaw = self.node.pose
        # Differential drive pivots on the wheel axle, behind base_link.
        axle = maze_solver.WHEEL_X_OFFSET_M if hasattr(
            maze_solver, 'WHEEL_X_OFFSET_M') else 0.0
        ax, ay = x + axle * math.cos(yaw), y + axle * math.sin(yaw)
        yaw += cmd.angular.z * dt
        ax += cmd.linear.x * dt * math.cos(yaw)
        ay += cmd.linear.x * dt * math.sin(yaw)
        self.node._update_pose(ax - axle * math.cos(yaw),
                               ay - axle * math.sin(yaw), yaw)
        return cmd


class LineRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MazeSolver()
        self.node._update_pose(0.0, 0.0, 0.0)
        self.robot = SolverHarness(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_outer_sensor_on_junction_colour_confirms_side_branch(self):
        # New map: side arms near a junction are painted junction colour,
        # so the outer sensor sees that colour instead of black.
        self.node._change_state(maze_solver.APPROACH_JUNCTION)
        self.node.approach_start_distance = self.node.distance_total
        self.robot.set_colours(junction=(True, True, True, False))
        for _ in range(3):
            self.robot.tick()
        self.assertEqual(1, len(self.node.junctions))
        self.assertTrue(self.node.junctions[-1]['left_open'])
        self.assertFalse(self.node.junctions[-1]['right_open'])

    def use_team(self):
        self.reports = []
        self.node.team_enabled = True
        self.node._team_report = self.reports.append

    def confirm_junction_with_left_branch(self):
        self.node._change_state(maze_solver.APPROACH_JUNCTION)
        self.node.approach_start_distance = self.node.distance_total
        self.robot.set_colours(junction=(True, True, True, False))
        for _ in range(3):
            self.robot.tick()

    def drive_to_centre(self, ahead=(False,) * 4):
        """Wheels reach the junction centre while RGB reads the floor ahead."""
        self.robot.set_colours(junction=ahead)
        for _ in range(100):
            if self.node.state != maze_solver.CENTERING:
                return
            self.robot.tick()

    def branch_requests(self):
        return [r for r in self.reports if r.get('type') == 'BRANCH_REQUEST']

    def test_junction_request_waits_until_wheels_reach_centre(self):
        self.use_team()
        self.confirm_junction_with_left_branch()
        self.assertEqual(maze_solver.CENTERING, self.node.state)
        self.assertEqual([], self.branch_requests())

    def test_no_straight_offered_when_floor_ahead_is_blank(self):
        self.use_team()
        self.confirm_junction_with_left_branch()
        self.drive_to_centre()
        self.assertEqual(maze_solver.JUNCTION_PAUSE, self.node.state)
        names = [c['name'] for c in self.branch_requests()[-1]['choices']]
        self.assertNotIn('straight', names)
        self.assertIn('left', names)
        self.assertFalse(self.node.junctions[-1]['straight_open'])

    def test_straight_offered_when_junction_colour_ahead(self):
        self.use_team()
        self.confirm_junction_with_left_branch()
        self.drive_to_centre(ahead=(False, True, True, False))
        names = [c['name'] for c in self.branch_requests()[-1]['choices']]
        self.assertIn('straight', names)
        self.assertTrue(self.node.junctions[-1]['straight_open'])

    def test_return_trip_omits_missing_straight(self):
        self.use_team()
        self.node.junctions = [{
            'id': 1, 'position': (0.0, 0.0), 'world_position': (0.0, 0.0),
            'heading': 0.0, 'world_heading': 0.0, 'left_open': True,
            'right_open': False, 'straight_open': False}]
        self.node._update_pose(0.3, 0.0, math.pi)
        self.node._request_team_branch(True, False, True, (0.0, 0.0))
        forward = [c for c in self.branch_requests()[-1]['choices']
                   if not c['fallback'] and abs(maze_solver.angle_difference(
                       c['heading'], 0.0)) < 0.1]
        self.assertEqual([], forward)

    def test_yield_during_centering_sends_no_cancel(self):
        self.use_team()
        self.confirm_junction_with_left_branch()
        self.node.team_peers = {'mbot0': {
            'x': 0.5, 'y': 0.0, 'heading': math.pi, 'state': 'FOLLOW',
            'route_order': 0}}
        self.node._begin_yield('mbot0')
        self.assertNotIn('CANCEL_BRANCH', [r.get('type') for r in self.reports])
        self.assertFalse(self.node.junction_needs_request)

    def test_solo_robot_turns_when_straight_closed(self):
        self.node.junction_choice = 'straight_first'
        self.confirm_junction_with_left_branch()
        self.drive_to_centre()
        self.assertEqual(maze_solver.TURNING, self.node.state)
        self.assertGreater(self.node.branch_angle, 0.0)

    def lose_line_here(self):
        self.node.last_centered_pose = self.node.pose
        self.node._begin_line_recovery(maze_solver.FOLLOW)
        self.robot.set_colours()

    def run_recovery(self, max_ticks=1500, nudge_every=None, black_behind=False,
                     stop_when_done=True):
        """Tick a lost robot; black_behind puts the line only behind the anchor."""
        rotation = 0.0
        scan_ticks = 0
        for _ in range(max_ticks):
            if stop_when_done and self.node.state != maze_solver.LINE_LOST:
                break
            if self.node.line_lost_phase == 'SCAN_AT_ANCHOR':
                scan_ticks += 1
                if nudge_every and scan_ticks % nudge_every == 0:
                    # Wheel slip while pivoting moves the robot off its anchor.
                    x, y, yaw = self.node.pose
                    self.node._update_pose(x, y + 0.03, yaw)
            behind = black_behind and abs(maze_solver.angle_difference(
                self.node.pose[2], math.pi)) < 0.3
            self.robot.set_colours(black=(behind,) * 4)
            before = self.node.pose[2]
            self.robot.tick()
            rotation += abs(maze_solver.angle_difference(self.node.pose[2], before))
        return rotation

    def test_dead_end_scan_finishes_within_one_revolution(self):
        self.lose_line_here()
        rotation = self.run_recovery(black_behind=True)
        self.assertEqual(maze_solver.TURNING, self.node.state)
        self.assertEqual('UTURN', self.node.turn_kind)
        self.assertLess(rotation, 2 * math.pi + 0.5)

    def test_scan_count_survives_drifting_off_the_anchor(self):
        self.lose_line_here()
        self.run_recovery(nudge_every=50, black_behind=True)
        self.assertEqual(maze_solver.TURNING, self.node.state)
        self.assertEqual('UTURN', self.node.turn_kind)

    def test_lost_line_reports_retry_not_blocked(self):
        self.use_team()
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'r1', 'previous_route_order': None}
        self.node.active_team_branch = dict(branch)
        self.lose_line_here()
        self.run_recovery(black_behind=True)
        types = [r.get('type') for r in self.reports]
        self.assertIn({'type': 'RETRY_BRANCH', **branch}, self.reports)
        self.assertNotIn('BLOCKED', types)
        self.assertIsNone(self.node.active_team_branch)

    def test_scan_without_any_black_stops_instead_of_spinning(self):
        self.lose_line_here()
        rotation = self.run_recovery(stop_when_done=False)
        self.assertLess(rotation, 2 * math.pi + 0.5)
        self.assertEqual(0.0, self.robot.sent[-1].angular.z)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)

    def test_scan_failure_while_returning_does_not_turn_back(self):
        # Heading back to the junction: the line behind leads to the dead end.
        self.node.last_centered_pose = self.node.pose
        self.node._begin_line_recovery(maze_solver.RETURNING)
        rotation = self.run_recovery(black_behind=True, stop_when_done=False)
        self.assertNotEqual('UTURN', self.node.turn_kind)
        self.assertLess(rotation, 2 * math.pi + 0.5)
        self.assertEqual(0.0, self.robot.sent[-1].angular.z)

    def test_inbound_arm_off_centre_is_not_a_side_branch(self):
        # Offset >= 1.75 cm on the 2.5 cm inbound green arm: one outer sensor
        # touches the arm while an inner sensor is already off it.
        self.node._change_state(maze_solver.APPROACH_JUNCTION)
        self.node.approach_start_distance = self.node.distance_total
        self.robot.set_colours(junction=(True, True, False, False))
        for _ in range(3):
            self.robot.tick()
        self.assertEqual([], self.node.junctions)


if __name__ == '__main__':
    unittest.main()
