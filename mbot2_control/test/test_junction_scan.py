"""All-black junctions: outer-sensor trigger, in-place scan and exit choice."""

import math
import time
import unittest
from unittest import mock

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from mbot2_control import maze_solver
from mbot2_control.maze_solver import MazeSolver
from mbot2_control.modules.sensors.quad_rgb_sensor import QuadRGBArray

BLACK = (10, 10, 10)
WHITE = (240, 240, 240)
YELLOW = (100, 87, 0)


def image(rgb):
    msg = Image()
    msg.data = bytes(rgb)
    return msg


class SensorNode(Node):
    def __init__(self, junction_color):
        super().__init__('sensor_test')
        self.declare_parameter('junction_color', junction_color)
        self.declare_parameter('goal_color', 'red')


class BlackModeSensorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SensorNode('black')
        self.sensors = QuadRGBArray(self.node)
        self.callbacks = [self.sensors._make_callback(i) for i in range(4)]

    def tearDown(self):
        self.node.destroy_node()

    def show(self, *colours):
        for callback, rgb in zip(self.callbacks, colours):
            callback(image(rgb))

    def test_black_mode_reports_no_junction_colour(self):
        self.show(YELLOW, YELLOW, YELLOW, YELLOW)
        self.assertEqual([False] * 4, self.sensors.yellow)

    def test_outer_hit_callback_fires_between_ticks(self):
        hits = []
        self.sensors.on_outer_hit = lambda: hits.append(True)
        self.show(WHITE, BLACK, BLACK, WHITE)
        self.assertEqual([], hits)
        self.show(BLACK, BLACK, BLACK, WHITE)  # side branch passes under sensor 1
        self.show(WHITE, BLACK, BLACK, WHITE)  # gone again before the next tick
        self.assertTrue(hits)

    def test_outer_black_without_inner_line_is_not_a_hit(self):
        hits = []
        self.sensors.on_outer_hit = lambda: hits.append(True)
        self.show(BLACK, WHITE, WHITE, WHITE)  # drifted off the line
        self.assertEqual([], hits)


SENSOR_Y = (0.030, 0.010, -0.010, -0.030)
HALF_LINE = 0.0125  # 2.5 cm lines as in mbot2_seed7_maze

INBOUND = (-1.0, 0.0, 0.5, 0.0)  # robot drives +x toward a junction at (0.5, 0)
STRAIGHT = (0.5, 0.0, 1.0, 0.0)
LEFT = (0.5, 0.0, 0.5, 0.5)
RIGHT = (0.5, 0.0, 0.5, -0.5)


def on_segment(px, py, segment):
    x1, y1, x2, y2 = segment
    dx, dy = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) <= HALF_LINE


class FloorHarness:
    """Real MazeSolver over a drawn black floor; sensors fire as callbacks."""

    SUBSTEPS = 5

    def __init__(self, node, segments, clock=None):
        self.node = node
        self.clock = clock
        self.segments = segments
        self.sent = []
        node.cmd_pub.publish = self.sent.append
        node.ultrasonic.ready = True
        node.ultrasonic.distance_m = None
        self.callbacks = [node.sensors._make_callback(i) for i in range(4)]
        self.read_floor()

    def read_floor(self):
        x, y, yaw = self.node.pose
        for callback, offset in zip(self.callbacks, SENSOR_Y):
            sx = x + 0.085 * math.cos(yaw) - offset * math.sin(yaw)
            sy = y + 0.085 * math.sin(yaw) + offset * math.cos(yaw)
            black = any(on_segment(sx, sy, s) for s in self.segments)
            callback(image(BLACK if black else WHITE))

    def tick(self, dt=0.1):
        now = time.monotonic()
        self.node.ultrasonic.last_seen_at = now
        self.node.last_odom_at = now
        self.node.last_team_state_at = now
        self.node.tick()
        cmd = self.sent[-1]
        if self.clock is not None:
            self.clock[0] += dt
        axle = maze_solver.WHEEL_X_OFFSET_M
        for _ in range(self.SUBSTEPS):
            x, y, yaw = self.node.pose
            ax, ay = x + axle * math.cos(yaw), y + axle * math.sin(yaw)
            step = dt / self.SUBSTEPS
            yaw += cmd.angular.z * step
            ax += cmd.linear.x * step * math.cos(yaw)
            ay += cmd.linear.x * step * math.sin(yaw)
            self.node._update_pose(ax - axle * math.cos(yaw),
                                   ay - axle * math.sin(yaw), yaw)
            self.read_floor()
        return cmd


class BlackJunctionScanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Simulated clock: speed ramps and pauses must not depend on how fast
        # the test machine runs.
        self.clock = [1000.0]
        patcher = mock.patch.object(time, 'monotonic', lambda: self.clock[0])
        patcher.start()
        self.addCleanup(patcher.stop)
        self.node = MazeSolver(parameter_overrides=[
            Parameter('junction_color', value='black')])
        self.node._update_pose(0.0, 0.0, 0.0)
        self.reports = []

    def tearDown(self):
        self.node.destroy_node()

    def use_team(self):
        self.node.team_enabled = True
        self.node._team_report = self.reports.append

    def drive(self, segments, until, max_ticks=600, state=maze_solver.FOLLOW):
        self.robot = FloorHarness(self.node, segments, self.clock)
        self.node._change_state(state)
        self.scan_rotation = 0.0
        for _ in range(max_ticks):
            if until():
                return
            before = self.node.pose[2]
            scanning = self.node.state == maze_solver.JUNCTION_SCAN
            self.robot.tick()
            if scanning:
                self.scan_rotation += abs(maze_solver.angle_difference(
                    self.node.pose[2], before))
        self.fail(f'never reached the expected state; now {self.node.state}')

    def branch_requests(self):
        return [r for r in self.reports if r.get('type') == 'BRANCH_REQUEST']

    def back_on_line_after(self, x_min=None, y_min=None):
        def done():
            x, y, _ = self.node.pose
            return (self.node.state == maze_solver.FOLLOW
                    and (x_min is None or x > x_min)
                    and (y_min is None or y > y_min))
        return done

    def test_t_junction_records_left_and_straight(self):
        self.use_team()
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        node = self.node.junctions[-1]
        self.assertTrue(node['left_open'])
        self.assertTrue(node['straight_open'])
        self.assertFalse(node['right_open'])

    def test_team_is_offered_only_scanned_exits(self):
        self.use_team()
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        choices = {c['name']: c['heading']
                   for c in self.branch_requests()[-1]['choices']}
        self.assertEqual({'straight', 'left', 'back'}, set(choices))
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            choices['straight'], 0.0), delta=0.15)
        # Measured, not snapped: the hit point is the near edge of the arm and
        # the axle stops up to 2 cm short, so the arm reads nearer 73 deg.
        self.assertAlmostEqual(math.pi / 2, choices['left'], delta=0.35)
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            choices['back'], math.pi), delta=0.15)

    def test_cross_scans_both_sides_within_400_degrees(self):
        self.drive([INBOUND, STRAIGHT, LEFT, RIGHT],
                   until=lambda: bool(self.node.junctions))
        node = self.node.junctions[-1]
        self.assertTrue(node['left_open'] and node['right_open'])
        self.assertLess(self.scan_rotation, math.radians(400))
        self.assertGreater(self.scan_rotation, math.radians(360))

    def test_left_hit_checks_only_the_left_side(self):
        # Sensor 1 found the arm; sensor 4 saw nothing, so no right sweep.
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: bool(self.node.junctions))
        node = self.node.junctions[-1]
        self.assertTrue(node['left_open'] and node['straight_open'])
        self.assertFalse(node['right_open'])
        self.assertLess(self.scan_rotation, math.radians(150))

    def test_right_hit_checks_only_the_right_side(self):
        self.drive([INBOUND, STRAIGHT, RIGHT],
                   until=lambda: bool(self.node.junctions))
        node = self.node.junctions[-1]
        self.assertTrue(node['right_open'] and node['straight_open'])
        self.assertFalse(node['left_open'])
        self.assertLess(self.scan_rotation, math.radians(150))

    def test_corner_turns_without_asking_the_team(self):
        self.use_team()
        self.drive([INBOUND, LEFT], until=self.back_on_line_after(y_min=0.2))
        self.assertEqual([], self.node.junctions)
        self.assertEqual([], self.branch_requests())
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            self.node.pose[2], math.pi / 2), delta=0.3)

    def test_false_trigger_keeps_following_the_line(self):
        speck = (0.5, 0.025, 0.5, 0.035)  # touches only the left outer sensor
        self.drive([INBOUND, STRAIGHT, speck],
                   until=self.back_on_line_after(x_min=0.75))
        self.assertEqual([], self.node.junctions)
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            self.node.pose[2], 0.0), delta=0.3)

    def test_line_end_with_crossbar_turns_back(self):
        crossbar = (0.5, -0.04, 0.5, 0.04)
        self.drive([INBOUND, crossbar],
                   until=lambda: self.node.state == maze_solver.TURNING
                   and self.node.turn_kind == 'UTURN')
        self.assertEqual([], self.node.junctions)

    def test_dead_end_reports_blocked_branch(self):
        self.use_team()
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'r1', 'previous_route_order': None}
        self.node.active_team_branch = dict(branch)
        crossbar = (0.5, -0.04, 0.5, 0.04)
        self.drive([INBOUND, crossbar],
                   until=lambda: self.node.state == maze_solver.TURNING)
        self.assertIn({'type': 'BLOCKED', **branch}, self.reports)
        self.assertIsNone(self.node.active_team_branch)

    def test_wheels_past_centre_still_find_side_branch(self):
        def overshoot():
            if self.node.junctions:
                return True
            if (self.node.state == maze_solver.CENTERING
                    and not getattr(self, 'shifted', False)):
                x, y = self.node.center_target
                self.node.center_target = (x + 0.05, y)
                self.shifted = True
            return False
        self.drive([INBOUND, STRAIGHT, LEFT], until=overshoot)
        self.assertTrue(self.shifted)
        self.assertTrue(self.node.junctions[-1]['left_open'])

    def test_solo_robot_takes_measured_side_branch(self):
        self.drive([INBOUND, STRAIGHT, RIGHT],
                   until=lambda: self.node.state == maze_solver.FOLLOW
                   and self.node.pose[1] < -0.2)
        self.assertTrue(self.node.junctions[-1]['right_open'])
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            self.node.pose[2], -math.pi / 2), delta=0.3)

    def test_yield_while_centring_drops_the_scan(self):
        self.use_team()
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: self.node.state == maze_solver.CENTERING)
        self.node.team_peers = {'mbot0': {
            'x': 0.9, 'y': 0.0, 'heading': math.pi, 'state': 'FOLLOW',
            'route_order': 0}}
        self.node._begin_yield('mbot0')
        self.assertFalse(self.node.scan_pending)
        self.assertIsNone(self.node.outer_hit_position)

    def test_return_request_uses_measured_headings(self):
        self.use_team()
        self.node.junctions = [{
            'id': 1, 'position': (0.0, 0.0), 'world_position': (0.0, 0.0),
            'heading': 0.0, 'world_heading': 0.0, 'left_open': True,
            'right_open': False, 'straight_open': True,
            'exit_headings': {'left': 1.40, 'straight': 0.05}}]
        self.node._update_pose(0.3, 0.0, math.pi)
        self.node._request_team_branch(True, False, True, (0.0, 0.0))
        headings = sorted(round(c['heading'], 2)
                          for c in self.branch_requests()[-1]['choices']
                          if not c['fallback'])
        self.assertEqual([0.05, 1.40], headings)

    def test_corner_while_returning_keeps_returning(self):
        self.node.returning = True
        self.drive([INBOUND, LEFT], state=maze_solver.RETURNING,
                   until=lambda: self.node.state == maze_solver.RETURNING
                   and self.node.pose[1] > 0.2)
        self.assertTrue(self.node.returning)

    def test_missed_junction_while_returning_asks_as_return(self):
        # Returning from a dead end (behind, -x) and meeting an unsaved T.
        self.use_team()
        self.node.returning = True
        self.drive([INBOUND, STRAIGHT, LEFT], state=maze_solver.RETURNING,
                   until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        node = self.node.junctions[-1]
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            node['heading'], math.pi), delta=0.1)  # stored as the outward pass
        self.assertFalse(node['straight_open'])  # the arm it came back from
        choices = self.branch_requests()[-1]['choices']
        new = [c for c in choices if not c['fallback']]
        self.assertEqual(['left'], [c['name'] for c in new])
        # Axle stops up to 2 cm short of the hit point, so the arm is seen
        # nearer 75° than 90°; the team still files it under the 90° sector.
        self.assertAlmostEqual(math.pi / 2, new[0]['heading'], delta=0.35)
        home = [c for c in choices if c['fallback']]
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            home[0]['heading'], 0.0), delta=0.1)

    def test_return_pivots_where_the_scan_pivoted(self):
        # Outward straight into a walled dead end, back, then into the left arm.
        saved = (maze_solver.REVERSE_SECONDS, maze_solver.JUNCTION_PAUSE_SECONDS)
        maze_solver.REVERSE_SECONDS = maze_solver.JUNCTION_PAUSE_SECONDS = 0.0
        self.addCleanup(setattr, maze_solver, 'REVERSE_SECONDS', saved[0])
        self.addCleanup(setattr, maze_solver, 'JUNCTION_PAUSE_SECONDS', saved[1])
        self.node.junction_choice = 'straight_first'
        pivots = {}

        def watch():
            node = self.node
            x, y, yaw = node.pose
            node.ultrasonic.distance_m = (
                1.23 - (x + 0.05) if abs(maze_solver.angle_difference(yaw, 0.0))
                < 0.3 and x > 0.3 else None)
            if node.state == maze_solver.JUNCTION_SCAN and 'scan' not in pivots:
                pivots['scan'] = node._axle_point(node.pose)
            if (node.state == maze_solver.TURNING and node.turn_kind == 'BRANCH'
                    and node.returning):
                pivots['return'] = node._axle_point(node.pose)
                return True
            return False
        self.drive([INBOUND, (0.5, 0.0, 1.2, 0.0), LEFT], until=watch)
        self.assertLessEqual(math.dist(pivots['scan'], pivots['return']),
                             maze_solver.CENTER_TOLERANCE_M)

    def test_yield_while_centring_keeps_current_branch(self):
        self.use_team()
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'r1', 'previous_route_order': None}
        self.node.active_team_branch = dict(branch)
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: self.node.state == maze_solver.CENTERING)
        self.node.team_peers = {'mbot0': {
            'x': 0.9, 'y': 0.0, 'heading': math.pi, 'state': 'FOLLOW',
            'route_order': 0}}
        self.node._begin_yield('mbot0')
        self.assertNotIn('CANCEL_BRANCH', [r.get('type') for r in self.reports])
        self.assertEqual(branch, self.node.active_team_branch)

    def test_yield_while_centring_resumes_before_the_side_arm(self):
        # Yield begins once the sensors are already well past the side arm.
        self.use_team()
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: self.node.state == maze_solver.CENTERING
                   and self.node.pose[0] > 0.47)
        self.node.team_peers = {'mbot0': {
            'x': 0.9, 'y': 0.0, 'heading': math.pi, 'state': 'FOLLOW',
            'route_order': 0}}
        self.node._begin_yield('mbot0')
        # YIELD_RESUME ends with base_link 6 cm short of its target.
        sensor_x = (self.node.yield_resume_target[0] - 0.06 +
                    maze_solver.RGB_FRONT_OFFSET_M)
        self.assertLess(sensor_x, 0.5 - 0.02)  # arm still ahead: hit again

    def peer(self, x, heading, state):
        self.node.team_peers = {'mbot0': {
            'x': x, 'y': 0.0, 'heading': heading, 'state': state,
            'route_order': 0}}

    def test_scanning_peer_is_not_taken_for_head_on(self):
        # A robot sweeping to +-130 deg at a junction ahead is not coming back.
        self.use_team()
        self.peer(0.6, math.radians(130), maze_solver.JUNCTION_SCAN)
        self.assertIsNone(self.node._head_on_peer())
        self.peer(0.6, math.pi, maze_solver.RETURNING)
        self.assertIsNotNone(self.node._head_on_peer())

    def test_peer_waiting_at_junction_after_scan_is_not_head_on(self):
        # The sweep ends at -130 deg; the robot then waits there for the team.
        self.use_team()
        for state in (maze_solver.JUNCTION_PAUSE, maze_solver.CENTERING):
            self.peer(0.6, math.radians(-130), state)
            self.assertIsNone(self.node._head_on_peer(), state)

    def test_waits_behind_scanning_peer_instead_of_yielding(self):
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.node.ultrasonic.distance_m = 0.2
        for _ in range(20):
            self.peer(self.node.pose[0] + 0.3, math.radians(130),
                      maze_solver.JUNCTION_SCAN)
            self.robot.tick()
            self.assertNotEqual(maze_solver.YIELDING, self.node.state)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)

    def test_yield_stops_at_the_end_of_the_line(self):
        # Last robot in the start queue: the line ends just behind it.
        self.use_team()
        self.robot = FloorHarness(self.node, [(-0.10, 0.0, 1.0, 0.0)], self.clock)
        self.peer(0.6, math.pi, maze_solver.RETURNING)
        self.node._begin_yield('mbot0')
        for _ in range(100):
            # The returning robot keeps pressing in, so the yield never clears.
            self.peer(self.node.pose[0] + 0.5, math.pi, maze_solver.RETURNING)
            before = self.node.pose[2]
            self.robot.tick()
            self.assertLess(abs(maze_solver.angle_difference(
                self.node.pose[2], before)), 0.05)  # no spinning on the spot
            # Black maze: stop as soon as the RGB row leaves the line end.
            self.assertGreater(self.node._sensor_position()[0],
                               -0.10 - HALF_LINE - 0.03)
        self.assertEqual(maze_solver.YIELDING, self.node.state)
        # Black maze: stop as soon as the RGB row leaves the line end.
        sensor_x = self.node._sensor_position()[0]
        self.assertGreater(sensor_x, -0.10 - HALF_LINE - 0.02)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        # The returning robot turns off; this one drives back without a search.
        for _ in range(200):
            self.peer(self.node.pose[0] - 1.0, math.pi, maze_solver.FOLLOW)
            self.robot.tick()
            self.assertNotEqual(maze_solver.LINE_LOST, self.node.state)
            if self.node.state == maze_solver.FOLLOW:
                break
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    def test_waits_behind_peer_aligning_after_its_turn(self):
        # The leader has just turned out of a junction and is squaring up on
        # its new line; it is driving away, so the follower only waits.
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.node.ultrasonic.distance_m = 0.2
        for _ in range(20):
            self.peer(self.node.pose[0] + 0.35, -0.1, maze_solver.ALIGN_LINE)
            self.robot.tick()
            self.assertNotEqual(maze_solver.YIELDING, self.node.state)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)

    def test_resume_drives_back_onto_line_from_just_past_its_end(self):
        # A yield left the RGB row 6 cm past the start of the line.
        self.use_team()
        self.robot = FloorHarness(self.node, [(-0.10, 0.0, 1.0, 0.0)], self.clock)
        self.node._update_pose(0.2, 0.0, 0.0)
        self.peer(0.8, math.pi, maze_solver.RETURNING)
        self.node._begin_yield('mbot0')
        self.node._update_pose(-0.16 - maze_solver.RGB_FRONT_OFFSET_M, 0.0, 0.0)
        self.robot.read_floor()
        self.node.last_path_color_distance = self.node.distance_total - 0.06
        for _ in range(200):
            self.peer(self.node.pose[0] - 1.0, math.pi, maze_solver.FOLLOW)
            self.robot.tick()
            self.assertNotEqual(maze_solver.LINE_LOST, self.node.state)
            if self.node.state == maze_solver.FOLLOW:
                break
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    def wait_at_junction_facing(self, heading):
        # Scan finished: the robot waits for the team, still turned to where
        # the sweep stopped, which is not the way it will drive.
        self.use_team()
        self.robot = FloorHarness(self.node, [INBOUND, STRAIGHT, LEFT], self.clock)
        self.node._update_pose(0.5, 0.0, heading)
        self.node.junctions = [{
            'id': 1, 'position': (0.5, 0.0), 'world_position': (0.5, 0.0),
            'heading': 0.0, 'world_heading': 0.0, 'left_open': True,
            'right_open': False, 'straight_open': True}]
        self.node._change_state(maze_solver.JUNCTION_PAUSE)

    def test_waiting_at_junction_does_not_yield_along_scan_heading(self):
        heading = math.radians(-130)
        self.wait_at_junction_facing(heading)
        # A peer on another line happens to lie along the leftover heading.
        self.node.team_peers = {'mbot0': {
            'x': 0.5 + 0.6 * math.cos(heading), 'y': 0.6 * math.sin(heading),
            'heading': heading + math.pi, 'state': maze_solver.FOLLOW,
            'route_order': 0}}
        self.robot.tick()
        self.assertNotEqual(maze_solver.YIELDING, self.node.state)

    def test_waiting_at_junction_turns_away_despite_ultrasonic(self):
        self.wait_at_junction_facing(math.radians(-130))
        self.node.ultrasonic.distance_m = 0.2  # a robot parked on another line
        self.node.team_choice = {
            'choice': 'left', 'junction_id': 1, 'direction': 1,
            'request_id': 'r1', 'junction_x': 0.5, 'junction_y': 0.0,
            'heading': math.pi / 2}
        self.clock[0] += maze_solver.JUNCTION_PAUSE_SECONDS + 0.1
        self.robot.tick()
        self.assertEqual(maze_solver.TURNING, self.node.state)


if __name__ == '__main__':
    unittest.main()
