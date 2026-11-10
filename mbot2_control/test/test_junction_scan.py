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
RED = (200, 20, 20)
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
    team_state_age = 0.0  # seconds since the last /maze_team/state

    def __init__(self, node, segments, clock=None, red_boxes=()):
        self.node = node
        self.clock = clock
        self.red_boxes = red_boxes  # (xmin, xmax, ymin, ymax) under the line
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
            red = any(a <= sx <= b and c <= sy <= d for a, b, c, d in self.red_boxes)
            callback(image(BLACK if black else RED if red else WHITE))

    def tick(self, dt=0.1):
        now = time.monotonic()
        self.node.ultrasonic.last_seen_at = now
        self.node.last_odom_at = now
        self.node.last_team_state_at = now - self.team_state_age
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

    def drive(self, segments, until, max_ticks=600, state=maze_solver.FOLLOW,
              team_state_age=0.0):
        self.robot = FloorHarness(self.node, segments, self.clock)
        self.robot.team_state_age = team_state_age
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

    def test_line_end_without_wall_turns_back_and_reports_blocked(self):
        # The branch just stops: no wall for the ultrasonic, no crossbar.
        self.use_team()
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'r1', 'previous_route_order': None}
        self.node.active_team_branch = dict(branch)
        self.drive([INBOUND],
                   until=lambda: self.node.state == maze_solver.RETURNING)
        self.assertTrue(self.node.returning)
        self.assertIn({'type': 'BLOCKED', **branch}, self.reports)
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            self.node.pose[2], math.pi), delta=0.3)

    def test_drift_off_the_line_finds_it_again_without_turning_back(self):
        # The line runs 6 cm to the left: every sensor misses it at once.
        self.robot = FloorHarness(self.node, [(-1.0, 0.06, 1.0, 0.06)], self.clock)
        self.node.last_centered_pose = self.node.pose
        self.node._change_state(maze_solver.FOLLOW)
        for _ in range(300):
            self.robot.tick()
            self.assertNotEqual('UTURN', self.node.turn_kind)
            if (self.node.state == maze_solver.FOLLOW
                    and self.node.pose[0] > 0.4):
                break
        self.assertEqual(maze_solver.FOLLOW, self.node.state)
        self.assertEqual([], self.node.junctions)

    def test_yield_right_after_a_turn_still_backs_along_the_line(self):
        # The yield starts before the RGB row has settled back on the line
        # (e.g. mid branch capture); that is not the end of the line.
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._update_pose(0.5, 0.0, 0.0)
        self.node.last_path_color_distance = self.node.distance_total - 0.05
        self.node.sensors.black = [False] * 4
        start_x = self.node.pose[0]
        for _ in range(40):
            self.peer(self.node.pose[0] + 0.5, math.pi, maze_solver.RETURNING)
            if self.node.state != maze_solver.YIELDING:
                self.node._begin_yield('mbot0')
            self.robot.tick()
        self.assertLess(self.node.pose[0], start_x - 0.15)

    def node_reports(self, kind):
        return [r for r in self.reports
                if r.get('type') == 'NODE' and r.get('kind') == kind]

    def test_new_corner_is_reported(self):
        self.use_team()
        self.drive([INBOUND, LEFT], until=self.back_on_line_after(y_min=0.2))
        corners = self.node_reports('CORNER')
        self.assertEqual(1, len(corners))
        self.assertAlmostEqual(math.pi / 2, corners[0]['exit_heading'], delta=0.35)
        self.assertAlmostEqual(0.0, corners[0]['approach_heading'], delta=0.1)
        self.assertAlmostEqual(0.5, corners[0]['x'], delta=0.03)

    def test_false_trigger_is_not_reported(self):
        self.use_team()
        speck = (0.5, 0.025, 0.5, 0.035)
        self.drive([INBOUND, STRAIGHT, speck],
                   until=self.back_on_line_after(x_min=0.75))
        self.assertEqual([], self.node_reports('CORNER'))

    def test_line_end_reports_dead_end(self):
        self.use_team()
        self.drive([INBOUND],
                   until=lambda: self.node.state == maze_solver.RETURNING)
        dead = self.node_reports('DEAD_END')
        self.assertEqual(1, len(dead))
        self.assertAlmostEqual(0.0, dead[0]['approach_heading'], delta=0.1)

    def test_leaving_start_box_reports_start(self):
        self.use_team()
        self.drive([INBOUND], state=maze_solver.SEARCH,
                   until=lambda: self.node.state == maze_solver.FOLLOW)
        start = self.node_reports('START')
        self.assertEqual(1, len(start))
        self.assertAlmostEqual(0.0, start[0]['heading'], delta=0.1)

    def team_node(self, kind, *headings):
        self.node.team_nodes = [{
            'id': 7, 'kind': kind, 'x': 0.5, 'y': 0.0,
            'branches': {str(round(h / (math.pi / 2)) % 4): {
                'status': 'OPEN', 'heading': h, 'leads_to': None}
                for h in headings}}]

    def test_known_junction_is_not_scanned(self):
        self.use_team()
        self.team_node('JUNCTION', math.pi, 0.0, math.pi / 2)
        self.drive([INBOUND, STRAIGHT, LEFT],
                   until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        self.assertLess(self.scan_rotation, 0.1)
        names = {c['name'] for c in self.branch_requests()[-1]['choices']}
        self.assertEqual({'straight', 'left', 'back'}, names)

    def test_known_junction_from_other_side_offers_other_branches(self):
        # Same junction, now reached down its north arm (heading south).
        self.use_team()
        self.team_node('JUNCTION', math.pi, 0.0, math.pi / 2)
        self.node._update_pose(0.5, 0.7, -math.pi / 2)
        self.drive([(0.5, 1.2, 0.5, 0.0), (-1.0, 0.0, 1.0, 0.0)],
                   until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        self.assertLess(self.scan_rotation, 0.1)
        choices = {c['name']: c['heading']
                   for c in self.branch_requests()[-1]['choices']}
        self.assertEqual({'left', 'right', 'back'}, set(choices))
        self.assertAlmostEqual(0.0, choices['left'], delta=0.05)

    def test_known_corner_turns_without_scan(self):
        self.use_team()
        self.team_node('CORNER', math.pi, math.pi / 2)
        self.drive([INBOUND, LEFT], until=self.back_on_line_after(y_min=0.2))
        self.assertLess(self.scan_rotation, 0.1)
        self.assertEqual(1, len(self.node_reports('CORNER')))
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            self.node.pose[2], math.pi / 2), delta=0.3)

    def test_stale_team_map_scans(self):
        self.use_team()
        self.team_node('JUNCTION', math.pi, 0.0, math.pi / 2)
        self.drive([INBOUND, STRAIGHT, LEFT], team_state_age=2.0,
                   until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        self.assertGreater(self.scan_rotation, 1.0)

    def test_known_node_needs_the_robot_at_the_node(self):
        self.use_team()
        self.team_node('CORNER', math.pi, math.pi / 2)
        self.node.last_team_state_at = time.monotonic()
        self.assertIsNone(self.node._known_node((0.3, 0.0)))
        self.assertIsNotNone(self.node._known_node((0.47, 0.0)))

    def test_line_end_near_known_corner_still_sweeps(self):
        # The line ends 18 cm short of a mapped corner: a dead end here,
        # not the corner, so the robot must not turn onto the corner's exit.
        self.use_team()
        self.team_node('CORNER', math.pi, math.pi / 2)
        self.drive([(-1.0, 0.0, 0.32, 0.0)],
                   until=lambda: self.node.state == maze_solver.RETURNING)
        self.assertEqual([], self.node_reports('CORNER'))
        self.assertEqual(1, len(self.node_reports('DEAD_END')))

    def head_on_setup(self):
        self.use_team()
        self.node.head_on_hiding = True  # C4 path
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._change_state(maze_solver.FOLLOW)

    def tick_with_peer_ahead(self, ticks=1):
        for _ in range(ticks):
            self.peer(self.node.pose[0] + 0.6, math.pi, maze_solver.RETURNING)
            self.robot.tick()

    def test_head_on_reports_and_stops(self):
        self.head_on_setup()
        self.tick_with_peer_ahead(3)
        heads = [r for r in self.reports if r.get('type') == 'HEAD_ON']
        self.assertEqual([{'type': 'HEAD_ON', 'peer': 'mbot0'}], heads)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        self.assertNotEqual(maze_solver.YIELDING, self.node.state)

    def test_no_answer_falls_back_to_old_yield(self):
        self.head_on_setup()
        self.tick_with_peer_ahead(25)  # 2.5 s without an order
        self.assertEqual(maze_solver.YIELDING, self.node.state)

    def order(self, kind, **extra):
        self.node._on_team_order({'type': kind, 'robot': self.node.robot_id,
                                  'encounter': 1, **extra})

    def test_hold_then_go_resumes(self):
        self.head_on_setup()
        self.tick_with_peer_ahead()
        self.order('HOLD')
        self.tick_with_peer_ahead(5)
        self.assertEqual(maze_solver.HOLDING, self.node.state)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        self.order('GO')
        self.node.team_peers = {}
        self.robot.tick()
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    def test_hold_times_out(self):
        self.head_on_setup()
        self.order('HOLD')
        self.robot.tick()
        self.clock[0] += maze_solver.HOLD_TIMEOUT_SECONDS + 0.5
        self.robot.read_floor()  # fresh camera frames after the jump
        self.node.team_peers = {}
        self.robot.tick()
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    # Corridor y=0 from corner C(0,0) east; C turns north to junction H(0,0.5)
    # whose east arm is free to hide in and whose north arm goes on.
    CORRIDOR = (0.0, 0.0, 1.0, 0.0)
    C_TO_H = (0.0, 0.0, 0.0, 0.5)
    H_EAST = (0.0, 0.5, 0.6, 0.5)
    H_NORTH = (0.0, 0.5, 0.0, 1.1)

    def escape_setup(self):
        self.use_team()
        self.node.head_on_hiding = True  # C4 path
        self.node.team_nodes = [
            {'id': 1, 'kind': 'CORNER', 'x': 0.0, 'y': 0.0, 'branches': {
                '0': {'status': 'OPEN', 'heading': 0.0, 'leads_to': None},
                '1': {'status': 'OPEN', 'heading': math.pi / 2,
                      'leads_to': {'node': 2, 'enter_dir': 3}}}},
            {'id': 2, 'kind': 'JUNCTION', 'x': 0.0, 'y': 0.5, 'branches': {
                '3': {'status': 'OPEN', 'heading': -math.pi / 2,
                      'leads_to': {'node': 1, 'enter_dir': 1}},
                '0': {'status': 'BLOCKED', 'heading': 0.0, 'leads_to': None},
                '1': {'status': 'OPEN', 'heading': math.pi / 2, 'leads_to': None}}}]
        self.node._update_pose(0.3, 0.0, 0.0)
        self.robot = FloorHarness(
            self.node, [self.CORRIDOR, self.C_TO_H, self.H_EAST, self.H_NORTH],
            self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.robot.tick()
        self.order('YIELD', route=[
            {'node': 1, 'exit_dir': 1, 'heading': math.pi / 2},
            {'node': 2, 'exit_dir': 0, 'heading': 0.0}],
            hide_node=2, hide_dir=0, hide_heading=0.0)

    def run_until(self, done, ticks=800):
        for _ in range(ticks):
            if done():
                return
            self.robot.tick()
        self.fail(f'stuck in {self.node.state}')

    def test_yield_turns_round_follows_route_and_hides(self):
        self.escape_setup()
        self.run_until(lambda: self.node.state == maze_solver.YIELD_WAIT)
        types = [r.get('type') for r in self.reports]
        self.assertEqual(1, types.count('HIDDEN'))
        self.assertNotIn('BRANCH_REQUEST', types)
        self.assertNotIn('NODE', types)
        ax, ay = self.node._axle_point(self.node.pose)
        self.assertAlmostEqual(maze_solver.HIDE_DEPTH_M, ax, delta=0.03)
        self.assertAlmostEqual(0.5, ay, delta=0.03)

    def test_clear_backs_out_and_asks_without_hide_branch(self):
        self.escape_setup()
        self.run_until(lambda: self.node.state == maze_solver.YIELD_WAIT)
        self.order('CLEAR')
        self.run_until(lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        ax, ay = self.node._axle_point(self.node.pose)
        self.assertLess(math.hypot(ax, ay - 0.5), 0.03)
        headings = [c['heading'] for c in self.branch_requests()[-1]['choices']]
        self.assertFalse(any(abs(maze_solver.angle_difference(h, 0.0)) < 0.3
                             for h in headings))

    def turn_back_setup(self, peer_state=maze_solver.RETURNING, peer_heading=math.pi,
                        gap=0.6):
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.gap, self.peer_state, self.peer_heading = gap, peer_state, peer_heading

    def tick_turn_back(self, ticks):
        start_x = self.node.pose[0]
        for _ in range(ticks):
            self.peer(start_x + self.gap, self.peer_heading, self.peer_state)
            self.robot.tick()

    def test_later_robot_turns_back(self):
        self.turn_back_setup()
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'r1', 'previous_route_order': None}
        self.node.active_team_branch = dict(branch)
        start_x = self.node.pose[0]
        for _ in range(80):  # until the turn is done and it drives back
            self.peer(start_x + self.gap, self.peer_heading, self.peer_state)
            self.robot.tick()
            if self.node.state == maze_solver.RETURNING:
                break
        self.assertIn({'type': 'RELEASE_BRANCH', **branch, 'peer': 'mbot0'}, self.reports)
        self.assertNotIn('BLOCKED', [r.get('type') for r in self.reports])
        self.assertTrue(self.node.returning)
        self.assertAlmostEqual(0.0, maze_solver.angle_difference(
            self.node.pose[2], math.pi), delta=0.3)
        self.assertFalse(any(c.linear.x < -0.01 for c in self.robot.sent))

    # Start gate (ก): wait in the start box until the robot in front has
    # reached its first junction.
    def start_setup(self, leader=None):
        self.use_team()
        self.robot = FloorHarness(self.node, [], self.clock)
        self.node._change_state(maze_solver.SEARCH)
        self.node.team_peers = {} if leader is None else {'mbot0': leader}

    def leader(self, reached):
        return {'x': 0.25, 'y': 0.0, 'heading': 0.0, 'state': maze_solver.FOLLOW,
                'route_order': 0, 'junction_reached': reached}

    def test_start_waits_until_leader_reaches_junction(self):
        self.start_setup(self.leader(False))
        for _ in range(20):
            self.robot.tick()
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        self.assertEqual(0.0, self.node.pose[0])
        self.node.team_peers = {'mbot0': {**self.leader(True), 'x': 1.5}}
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_start_goes_with_no_robot_in_front(self):
        self.start_setup()
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_start_goes_when_leader_drops_out(self):
        self.start_setup(self.leader(False))
        self.robot.tick()
        self.node.team_peers = {}
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_start_gate_does_not_hold_a_solo_robot(self):
        self.robot = FloorHarness(self.node, [], self.clock)
        self.node._change_state(maze_solver.SEARCH)
        self.node.team_peers = {'mbot0': self.leader(False)}
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    # Clearance (ข): do not drive into a teammate just ahead, e.g. one still
    # turning off a junction where the ultrasonic beam cannot see it.
    def clearance_setup(self, peer_x, peer_y, heading, state=maze_solver.FOLLOW, order=0):
        self.turn_back_setup()
        self.node.team_route_order = 5
        self.node.team_peers = {'mbot0': {
            'x': peer_x, 'y': peer_y, 'heading': heading, 'state': state,
            'route_order': order}}

    def test_waits_for_peer_turning_off_ahead(self):
        x = self.node.pose[0]
        self.clearance_setup(x + 0.17, 0.09, math.pi / 2)
        for _ in range(10):
            self.robot.tick()
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    def test_goes_once_peer_has_cleared(self):
        x = self.node.pose[0]
        self.clearance_setup(x + 0.17, 0.30, math.pi / 2)
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_parked_peer_ahead_does_not_stop_clearance(self):
        x = self.node.pose[0]
        self.clearance_setup(x + 0.30, 0.0, 0.0, state=maze_solver.GOAL_REACHED)
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_crossing_robots_only_the_later_one_waits(self):
        # Each sees the other ahead; mbot0 (order 0) entered first and goes.
        x = self.node.pose[0]
        self.clearance_setup(x + 0.20, -0.20, math.pi / 2, order=0)
        self.robot.tick()
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        self.node.team_route_order = 0
        self.node.team_peers['mbot0']['route_order'] = 5
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_waits_for_pivoting_peer_even_with_priority(self):
        # mbot0 got its branch last (order 9) but is still turning toward us.
        x = self.node.pose[0]
        self.clearance_setup(x + 0.25, 0.0, math.pi, state=maze_solver.TURNING, order=9)
        self.robot.tick()
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)

    def test_centering_robots_meeting_at_a_junction_do_not_both_wait(self):
        # Seen in sim: two robots centring on one junction from two arms each
        # treated the other as pivoting and both waited for good.
        x = self.node.pose[0]
        self.clearance_setup(x + 0.19, -0.08, math.pi / 2,
                             state=maze_solver.CENTERING, order=9)
        self.node.team_route_order = 0  # we entered first: we go
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_busy_peer_does_not_run_down_the_turn_back_timer(self):
        x = self.node.pose[0]
        self.clearance_setup(x + 0.25, 0.0, 0.0, state=maze_solver.JUNCTION_PAUSE)
        for _ in range(int(maze_solver.PEER_WAIT_TURN_BACK_SECONDS * 10) + 20):
            self.robot.tick()
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    def test_stuck_peer_ahead_clearance_turns_back(self):
        x = self.node.pose[0]
        self.clearance_setup(x + 0.25, 0.0, 0.0, state=maze_solver.LINE_LOST)
        for _ in range(int(maze_solver.PEER_WAIT_TURN_BACK_SECONDS * 10) + 20):
            self.robot.tick()
        self.assertNotEqual(maze_solver.FOLLOW, self.node.state)

    def test_returning_robot_waits_instead_of_turning_back(self):
        x = self.node.pose[0]
        self.clearance_setup(x + 0.25, 0.0, 0.0, state=maze_solver.LINE_LOST)
        self.node.returning = True
        self.node._change_state(maze_solver.RETURNING)
        for _ in range(int(maze_solver.PEER_WAIT_TURN_BACK_SECONDS * 10) + 20):
            self.robot.tick()
        self.assertEqual(maze_solver.RETURNING, self.node.state)
        self.assertNotIn('RELEASE_BRANCH', [r.get('type') for r in self.reports])

    def test_start_waits_for_far_leader_that_missed_a_junction(self):
        # mbot0 drove past the first junction without stopping: still wait.
        self.start_setup(self.leader(False))
        self.robot.tick()
        self.node.team_peers = {'mbot0': {**self.leader(False), 'x': 1.3}}
        self.robot.tick()
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)

    def test_start_goes_after_gate_timeout(self):
        self.start_setup(self.leader(False))
        self.robot.tick()
        self.node.team_peers = {'mbot0': {**self.leader(False), 'x': 0.6}}  # stalled
        self.clock[0] += maze_solver.START_GATE_TIMEOUT_SECONDS + 1.0
        self.robot.read_floor()
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_start_goes_when_leader_is_at_goal(self):
        self.start_setup({**self.leader(False), 'x': 0.6,
                          'state': maze_solver.GOAL_REACHED})
        self.robot.tick()
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_earlier_robot_waits(self):
        self.turn_back_setup(gap=0.3)
        self.node.team_route_order = 1  # entered first: mbot0 has order 0? no
        self.node.team_peers = {}
        self.node.ultrasonic.distance_m = 0.2
        for _ in range(20):
            self.node.team_peers = {'mbot0': {
                'x': self.node.pose[0] + 0.3, 'y': 0.0, 'heading': math.pi,
                'state': maze_solver.RETURNING, 'route_order': 5}}
            self.robot.tick()
        self.assertEqual(maze_solver.FOLLOW, self.node.state)
        self.assertEqual(0.0, self.robot.sent[-1].linear.x)
        self.assertNotIn('RELEASE_BRANCH', [r.get('type') for r in self.reports])

    def test_lost_peer_ahead_waits(self):
        self.turn_back_setup(peer_state=maze_solver.LINE_LOST, peer_heading=0.0, gap=0.3)
        self.node.ultrasonic.distance_m = 0.2
        self.tick_turn_back(20)
        self.assertNotIn(self.node.state, (maze_solver.YIELDING, maze_solver.TURNING))
        self.assertFalse(any(c.linear.x < -0.01 for c in self.robot.sent))

    def test_returning_robot_turns_back_into_follow(self):
        self.turn_back_setup()
        # Both on their way back: the later one turns and explores on.
        self.node.returning = True
        self.node._change_state(maze_solver.RETURNING)
        start_x = self.node.pose[0]
        for _ in range(80):
            self.peer(start_x + self.gap, self.peer_heading, self.peer_state)
            self.node.team_peers['mbot0']['returning'] = True
            self.robot.tick()
            if (self.node.state == maze_solver.FOLLOW
                    and abs(maze_solver.angle_difference(self.node.pose[2], math.pi)) < 0.3):
                break
        self.assertFalse(self.node.returning)
        self.assertEqual(maze_solver.FOLLOW, self.node.state)

    def test_robot_parked_at_goal_is_not_head_on(self):
        # A teammate waiting at the goal faces any way; walk up and stop.
        self.use_team()
        self.peer(0.6, math.pi, maze_solver.GOAL_REACHED)
        self.assertIsNone(self.node._head_on_peer())

    def test_stuck_peer_ahead_makes_robot_turn_back(self):
        # The teammate ahead lost its line and never moves again.
        self.turn_back_setup(peer_state=maze_solver.LINE_LOST, peer_heading=0.0, gap=0.3)
        self.node.ultrasonic.distance_m = 0.2
        ticks = int(maze_solver.PEER_WAIT_TURN_BACK_SECONDS * 10) + 20
        self.tick_turn_back(ticks)
        self.assertIn(self.node.state, (maze_solver.TURNING, maze_solver.REACQUIRE,
                                        maze_solver.ALIGN_LINE, maze_solver.RETURNING))
        self.assertFalse(any(c.linear.x < -0.01 for c in self.robot.sent))

    def test_returning_robot_never_turns_back(self):
        # mbot0 has priority by order, but this robot is on its way back
        # from a dead end: the one heading into the corridor turns instead.
        self.turn_back_setup()
        self.node.returning = True
        self.node._change_state(maze_solver.RETURNING)
        self.tick_turn_back(5)
        self.assertNotIn('RELEASE_BRANCH', [r.get('type') for r in self.reports])
        self.assertNotEqual(maze_solver.TURNING, self.node.state)

    def test_robot_turns_back_for_returning_peer_without_priority(self):
        self.turn_back_setup()
        self.node.team_route_order = 0
        for _ in range(5):
            self.node.team_peers = {'mbot0': {
                'x': self.node.pose[0] + 0.6, 'y': 0.0, 'heading': math.pi,
                'state': maze_solver.RETURNING, 'route_order': 9, 'returning': True}}
            self.robot.tick()
        self.assertIn(self.node.state, (maze_solver.TURNING,))

    def test_no_turn_back_with_teammate_close_behind(self):
        self.turn_back_setup()
        for _ in range(5):
            self.node.team_peers = {
                'mbot0': {'x': self.node.pose[0] + 0.6, 'y': 0.0, 'heading': math.pi,
                          'state': maze_solver.RETURNING, 'route_order': 0},
                'mbot9': {'x': self.node.pose[0] - 0.3, 'y': 0.0, 'heading': 0.0,
                          'state': maze_solver.FOLLOW, 'route_order': 9}}
            self.robot.tick()
        self.assertNotEqual(maze_solver.TURNING, self.node.state)
        self.assertTrue(self.reports[-1].get('blocked_behind', False) or any(
            r.get('blocked_behind') for r in self.reports if r.get('type') == 'POSE'))

    def test_turns_back_for_peer_blocked_behind(self):
        self.turn_back_setup()
        self.node.team_route_order = 0
        for _ in range(5):
            self.node.team_peers = {'mbot0': {
                'x': self.node.pose[0] + 0.6, 'y': 0.0, 'heading': math.pi,
                'state': maze_solver.FOLLOW, 'route_order': 9, 'blocked_behind': True}}
            self.robot.tick()
        self.assertEqual(maze_solver.TURNING, self.node.state)

    def test_wall_with_teammate_behind_turns_without_reversing(self):
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.node.ultrasonic.distance_m = 0.2
        self.node.team_peers = {'mbot9': {
            'x': -0.3, 'y': 0.0, 'heading': 0.0, 'state': maze_solver.FOLLOW,
            'route_order': 9}}
        self.robot.tick()
        self.assertNotEqual(maze_solver.REVERSING, self.node.state)
        self.assertFalse(any(c.linear.x < -0.01 for c in self.robot.sent))

    def test_turn_back_while_centring_cancels_the_fresh_choice(self):
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node.active_team_branch = {
            'junction_id': 4, 'direction': 1, 'choice': 'left', 'request_id': 'r9',
            'previous_route_order': 3}
        self.node.center_start = self.node.pose[:2]
        self.node.center_heading = 0.0
        self.node._change_state(maze_solver.CENTERING)
        self.node.junction_needs_request = True
        self.node._begin_turn_back('mbot0')  # not asked yet: still in its corridor
        self.assertFalse(self.node.junction_needs_request)
        self.node.junction_needs_request = False
        self.node.active_team_branch = {
            'junction_id': 4, 'direction': 1, 'choice': 'left', 'request_id': 'r9',
            'previous_route_order': 3}
        self.node._change_state(maze_solver.CENTERING)
        self.node._begin_turn_back('mbot0')
        self.assertIn({'type': 'CANCEL_BRANCH', 'request_id': 'r9'}, self.reports)
        self.assertEqual(3, self.node.team_route_order)

    def test_robot_parked_at_goal_never_triggers_turn_back(self):
        self.turn_back_setup(peer_state=maze_solver.GOAL_REACHED, gap=0.3)
        self.node.ultrasonic.distance_m = 0.2
        self.tick_turn_back(5)
        self.assertNotEqual(maze_solver.TURNING, self.node.state)

    # Goal area: red box x 0.05..0.95, line runs through it and ends at 0.8.
    GOAL_BOX = (0.05, 0.95, -0.15, 0.15)

    def goal_setup(self):
        self.use_team()
        self.node.sensors.goal_color = 'red'
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 0.8, 0.0)], self.clock,
                                  red_boxes=[self.GOAL_BOX])
        self.node._change_state(maze_solver.FOLLOW)

    def goals(self):
        return [r for r in self.reports if r.get('type') == 'GOAL']

    def test_parks_at_line_end_inside_area(self):
        self.goal_setup()
        self.run_until(lambda: self.node.state == maze_solver.GOAL_REACHED)
        self.assertEqual(1, len(self.goals()))
        self.assertTrue(self.node.sensors.any_green())
        self.assertGreater(self.node.pose[0], 0.55)

    def test_parks_behind_teammate(self):
        self.goal_setup()
        rear = 0.55  # back of a teammate parked further in
        for _ in range(300):
            if self.node.state == maze_solver.GOAL_REACHED:
                break
            self.node.team_peers = {'mbot0': {
                'x': rear + 0.1, 'y': 0.0, 'heading': 0.0,
                'state': maze_solver.GOAL_REACHED, 'route_order': 0}}
            gap = rear - (self.node.pose[0] + 0.1)
            self.node.ultrasonic.distance_m = gap if gap < 0.4 else None
            self.robot.tick()
        self.assertEqual(maze_solver.GOAL_REACHED, self.node.state)
        self.assertLess(self.node.pose[0] + 0.1, rear)  # never touched it

    def test_parking_ignores_peer_ahead_head_on(self):
        self.goal_setup()
        self.run_until(lambda: self.node.state == maze_solver.GOAL_PARKING)
        for _ in range(10):
            self.peer(self.node.pose[0] + 0.5, math.pi, maze_solver.RETURNING)
            self.robot.tick()
            self.assertNotEqual(maze_solver.TURNING, self.node.state)
        self.assertNotIn('RELEASE_BRANCH', [r.get('type') for r in self.reports])

    def test_near_goal_without_red_is_not_arrived(self):
        self.use_team()
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 1.0, 0.0)], self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.node.team_goal = {'robot': 'mbot0', 'x': 0.3, 'y': 0.0}
        for _ in range(5):
            self.robot.tick()
        self.assertNotEqual(maze_solver.GOAL_REACHED, self.node.state)

    def test_goal_route_turns_robot_back(self):
        self.turn_back_setup()
        self.node.team_peers = {}
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'r1', 'previous_route_order': None}
        self.node.active_team_branch = dict(branch)
        self.robot.tick()
        self.order('GOAL_ROUTE', turn_back=True)
        self.assertEqual(maze_solver.TURNING, self.node.state)
        self.assertEqual('UTURN', self.node.turn_kind)
        self.assertIn('RELEASE_BRANCH', [r.get('type') for r in self.reports])

    def test_goal_route_ignored_when_returning(self):
        self.turn_back_setup()
        self.node.team_peers = {}
        self.node.returning = True
        self.node._change_state(maze_solver.RETURNING)
        self.robot.tick()
        self.order('GOAL_ROUTE', turn_back=True)
        self.assertEqual(maze_solver.RETURNING, self.node.state)

    def test_goal_reported_once(self):
        self.goal_setup()
        self.run_until(lambda: self.node.state == maze_solver.GOAL_REACHED)
        self.node._change_state(maze_solver.FOLLOW)  # e.g. knocked off and back
        self.robot.tick()
        self.assertEqual(1, len(self.goals()))

    def test_parking_entered_off_line_creeps_in(self):
        # Entering the area 5 cm beside the line: no sensor sees black yet.
        self.goal_setup()
        self.node._update_pose(0.1, 0.05, 0.0)
        self.robot.read_floor()
        self.robot.tick()
        self.assertEqual(maze_solver.GOAL_PARKING, self.node.state)
        self.assertGreater(self.robot.sent[-1].linear.x, 0.0)

    def test_parking_without_red_or_line_recovers(self):
        self.goal_setup()
        self.node._update_pose(0.1, 0.05, 0.0)
        self.robot.read_floor()
        self.robot.tick()  # parking, creeping
        self.node._update_pose(1.2, 0.5, 0.0)  # pushed off everything
        self.robot.read_floor()
        for _ in range(30):
            self.robot.tick()
            if self.node.state != maze_solver.GOAL_PARKING:
                break
        self.assertNotEqual(maze_solver.GOAL_PARKING, self.node.state)
        self.assertNotEqual(maze_solver.GOAL_REACHED, self.node.state)

    def test_queued_outside_area_creeps_up_and_arrives(self):
        # The area is full: the last robot queues on the line before it.
        self.use_team()
        self.node.sensors.goal_color = 'red'
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 0.8, 0.0)], self.clock,
                                  red_boxes=[(0.5, 0.95, -0.15, 0.15)])
        self.node._change_state(maze_solver.FOLLOW)
        rear = 0.37
        for _ in range(300):
            if self.node.state == maze_solver.GOAL_REACHED:
                break
            self.node.team_peers = {'mbot0': {
                'x': rear + 0.08, 'y': 0.0, 'heading': 0.0,
                'state': maze_solver.GOAL_REACHED, 'route_order': 0}}
            gap = rear - (self.node.pose[0] + 0.1)
            self.node.ultrasonic.distance_m = gap if gap < 0.4 else None
            self.robot.tick()
        self.assertEqual(maze_solver.GOAL_REACHED, self.node.state)
        self.assertLess(self.node.pose[0] + 0.1, rear)

    def test_leaves_junction_as_soon_as_team_answers(self):
        self.wait_at_junction_facing(0.0)
        self.node.team_choice = {
            'choice': 'left', 'junction_id': 1, 'direction': 1,
            'request_id': 'r1', 'junction_x': 0.5, 'junction_y': 0.0,
            'heading': math.pi / 2}
        self.robot.tick()  # answer already in: no fixed one-second pause
        self.assertEqual(maze_solver.TURNING, self.node.state)

    # C2 stage 1: ask ahead at known junctions -------------------------------

    def ahead_setup(self):
        self.use_team()
        self.team_node('JUNCTION', math.pi, 0.0, math.pi / 2)
        self.robot = FloorHarness(self.node, [INBOUND, STRAIGHT, LEFT], self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.states = []

    def tick_ahead(self, ticks, answer=None, until=None):
        for _ in range(ticks):
            if until is not None and until():
                return
            pending = self.node.pending_team_request
            if answer and pending is not None and self.node.team_choice is None:
                name, heading, key = answer
                self.node.team_choice = {
                    'choice': name, 'junction_id': 7, 'direction': key,
                    'request_id': pending['request_id'], 'junction_x': 0.5,
                    'junction_y': 0.0, 'heading': heading}
            self.robot.tick()
            self.states.append(self.node.state)

    def test_asks_ahead_for_known_junction(self):
        self.ahead_setup()
        self.tick_ahead(100, until=lambda: bool(self.branch_requests()))
        self.assertEqual(1, len(self.branch_requests()))
        self.assertEqual(maze_solver.FOLLOW, self.node.state)
        self.assertLess(self.node._sensor_position()[0], 0.5)

    def test_passes_straight_without_stopping(self):
        self.ahead_setup()
        self.tick_ahead(200, answer=('straight', 0.0, 0),
                        until=lambda: self.node.pose[0] > 0.75)
        for state in (maze_solver.CENTERING, maze_solver.JUNCTION_PAUSE,
                      maze_solver.JUNCTION_SCAN, maze_solver.TURNING):
            self.assertNotIn(state, self.states)
        self.assertEqual(1, len(self.branch_requests()))
        self.assertAlmostEqual(0.5, self.node.junctions[-1]['world_position'][0], delta=0.05)
        self.assertIsNotNone(self.node.active_team_branch)

    def test_turns_at_known_junction_without_scan_or_wait(self):
        self.ahead_setup()
        self.tick_ahead(400, answer=('left', math.pi / 2, 1),
                        until=lambda: self.node.state == maze_solver.FOLLOW
                        and self.node.pose[1] > 0.2)
        self.assertIn(maze_solver.TURNING, self.states)
        self.assertNotIn(maze_solver.JUNCTION_SCAN, self.states)
        self.assertNotIn(maze_solver.JUNCTION_PAUSE, self.states)

    def test_waits_when_answer_late_without_second_request(self):
        self.ahead_setup()
        self.tick_ahead(200, until=lambda: self.node.state == maze_solver.JUNCTION_PAUSE)
        self.assertEqual(maze_solver.JUNCTION_PAUSE, self.node.state)
        ids = {r['request_id'] for r in self.branch_requests()}
        self.assertEqual(1, len(ids))

    def test_turn_back_cancels_ahead_request(self):
        self.ahead_setup()
        self.tick_ahead(100, until=lambda: bool(self.branch_requests()))
        rid = self.branch_requests()[-1]['request_id']
        self.node._begin_turn_back('mbot0')
        self.assertIn({'type': 'CANCEL_BRANCH', 'request_id': rid}, self.reports)
        self.assertIsNone(self.node.ahead_request)

    def test_stale_ahead_request_dropped(self):
        self.ahead_setup()
        self.tick_ahead(100, until=lambda: bool(self.branch_requests()))
        rid = self.branch_requests()[-1]['request_id']
        self.node._update_pose(1.0, 0.0, 0.0)  # e.g. relocated past the junction
        self.robot.read_floor()
        self.robot.tick()
        self.assertIsNone(self.node.ahead_request)
        self.assertIn({'type': 'CANCEL_BRANCH', 'request_id': rid}, self.reports)

    def test_hit_short_of_asked_junction_cancels_ahead_request(self):
        # A corner the map does not know sits 0.2 m before the junction asked about.
        self.use_team()
        self.team_node('JUNCTION', math.pi, 0.0, math.pi / 2)
        self.robot = FloorHarness(self.node, [(-1.0, 0.0, 0.3, 0.0), (0.3, 0.0, 0.3, 0.5)],
                                  self.clock)
        self.node._change_state(maze_solver.FOLLOW)
        self.states = []
        self.tick_ahead(100, until=lambda: bool(self.branch_requests()))
        rid = self.branch_requests()[-1]['request_id']
        self.tick_ahead(100, until=lambda: self.node.state == maze_solver.CENTERING)
        self.assertIn({'type': 'CANCEL_BRANCH', 'request_id': rid}, self.reports)
        self.assertIsNone(self.node.ahead_request)

    def test_turn_back_while_waiting_for_ahead_answer_releases_current_branch(self):
        self.ahead_setup()
        branch = {'junction_id': 3, 'direction': 0, 'choice': 'straight',
                  'request_id': 'rA', 'previous_route_order': 2}
        self.node.active_team_branch = dict(branch)
        self.tick_ahead(200, until=lambda: self.node.state == maze_solver.CENTERING)
        rid = self.node.pending_team_request['request_id']
        self.node._begin_turn_back('mbot0')
        self.assertIn({'type': 'RELEASE_BRANCH', **branch, 'peer': 'mbot0'}, self.reports)
        self.assertNotIn({'type': 'CANCEL_BRANCH', 'request_id': 'rA'}, self.reports)
        self.assertIn({'type': 'CANCEL_BRANCH', 'request_id': rid}, self.reports)


if __name__ == '__main__':
    unittest.main()
