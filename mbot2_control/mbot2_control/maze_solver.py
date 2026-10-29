"""Experiment 1: follow a line, backtrack from walls, and leave digital trails."""

import json
import math
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from mbot2_control.modules import QuadRGBArray, UltrasonicSensor, WheelEncoder

# Keep the 0.5 m/s straight-line cruise speed; speed up the slow recovery modes.
FORWARD_SPEED = 0.5
FOLLOW_RAMP_START_SPEED = 0.20
FOLLOW_RAMP_SECONDS = 0.8
APPROACH_SPEED = 0.14
RETURN_SPEED = 0.22
JUNCTION_RETURN_SPEED = 0.10
SEARCH_SPEED = 0.12
SEARCH_MAX_M = 0.45  # enough to leave the blue start area and reach the line
REVERSE_SPEED = 0.15
REVERSE_SECONDS = 1.5
OBSTACLE_STOP_DISTANCE_M = 0.25
YIELD_REVERSE_M = 1.20  # fallback when no junction position is known
YIELD_CLEARANCE_M = 0.65
YIELD_REAR_CLEARANCE_M = 0.45
YIELD_JUNCTION_CLEARANCE_M = 0.70
YIELD_BRANCH_CLEARANCE_M = 0.55
HEAD_ON_DETECT_M = 1.20
HEAD_ON_HEADING_RAD = 2.2
YIELD_LINE_TURN_GAIN = 0.35
YIELD_HEADING_GAIN = 0.8
TEAM_GOAL_RADIUS_M = 0.55
TEAM_ASSEMBLY_RADIUS_M = 1.45

TURN_GAIN = 0.15
MAX_TURN = 0.5
LINE_CORRECTION_SPEED = 0.18
LINE_CORRECTION_GAIN = 0.65
LINE_CORRECTION_MAX_TURN = 1.2
ROTATE_SPEED = 0.85
TURN_TOLERANCE_RAD = 0.07
ALIGN_SPEED = 0.07              # เดินช้าเฉพาะเมื่อเซนเซอร์ในเริ่มเห็นเส้นแล้ว
ALIGN_TURN_GAIN = 0.50
ALIGN_MAX_TURN = 0.75
ALIGN_LOST_GRACE_SECONDS = 0.6
BRANCH_CAPTURE_SPEED = 0.16
BRANCH_CAPTURE_IGNORE_M = 0.03
BRANCH_CAPTURE_MAX_M = 0.24
PATH_COLOR_GRACE_M = 0.08
YIELD_PATH_GAP_M = 0.35  # front RGB can cross a broad junction while reversing
ANCHOR_ARRIVAL_M = 0.015  # RGB footprint is only centimetres wide
ANCHOR_TRAVEL_SPEED = 0.10
ANCHOR_TURN_SPEED = 0.65
ANCHOR_SWEEP_RAD = 0.65
INNER_CONFIRM_TICKS = 2        # สองตัวในต้องเห็นดำติดต่อกันก่อนวิ่งปกติ
SIGNAL_STALE_SECONDS = 2.0     # ข้อมูลหายจริงจึงพักคำสั่งขับ และกลับมาทำต่อเมื่อข้อมูลมา
SEARCH_LOG_SECONDS = 10.0
JUNCTION_PAUSE_SECONDS = 1.0

# Front RGB sensor is ~8.5 cm ahead of the base centre in the robot description.
RGB_FRONT_OFFSET_M = 0.085
WHEEL_X_OFFSET_M = -0.015  # wheel axle behind base_link: the pivot when turning in place
JUNCTION_SLOW_RADIUS_M = 0.55
JUNCTION_LATERAL_TOLERANCE_M = 0.15
JUNCTION_ESCAPE_DISTANCE_M = 0.16  # พ้นปื้นเหลืองตรงกลางก่อนตรวจแยกใหม่
JUNCTION_APPROACH_MAX_M = 0.40  # yellow warning stripe starts 0.25 m before J
JUNCTION_APPROACH_ABORT_M = 0.48
RETURN_MARKER_RADIUS_M = 0.10
TRAIL_UPDATE_SECONDS = 1.0
# junction_color='black': stop on an outer-sensor hit, then sweep in place.
SCAN_LIMIT_RAD = math.radians(130)
SCAN_STRAIGHT_MAX_RAD = math.radians(40)  # |θ| ≤ 40° is the straight arm
SCAN_SIDE_MIN_RAD = math.radians(50)      # 50°..130° is a side arm
CENTER_TOLERANCE_M = 0.02

SEARCH = 'SEARCH'
FOLLOW = 'FOLLOW'
APPROACH_JUNCTION = 'APPROACH_JUNCTION'
JUNCTION_PAUSE = 'JUNCTION_PAUSE'
JUNCTION_SCAN = 'JUNCTION_SCAN'
CENTERING = 'CENTERING'
BRANCH_CAPTURE = 'BRANCH_CAPTURE'
TURNING = 'TURNING'
REACQUIRE = 'REACQUIRE'
ALIGN_LINE = 'ALIGN_LINE'
RETURNING = 'RETURNING'
LINE_LOST = 'LINE_LOST'
REVERSING = 'REVERSING'
YIELDING = 'YIELDING'
YIELD_RESUME = 'YIELD_RESUME'
YIELD_TURN = 'YIELD_TURN'
YIELD_ESCAPE = 'YIELD_ESCAPE'
YIELD_WAIT = 'YIELD_WAIT'
GOAL_REACHED = 'GOAL_REACHED'
# A peer stopped or turning on the spot at a junction points anywhere (a
# scan ends at -130 deg); its heading says nothing about whether it is
# coming back down this corridor.
PIVOTING_STATES = (CENTERING, JUNCTION_SCAN, JUNCTION_PAUSE, TURNING)


def angle_difference(target, current):
    """Shortest signed angle from current to target."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class MazeSolver(Node):

    def __init__(self, **kwargs):
        super().__init__('maze_solver', **kwargs)
        self.declare_parameter('junction_choice', 'side_first')
        self.declare_parameter('team_enabled', False)
        self.declare_parameter('use_ground_truth', False)
        self.declare_parameter('world_name', 'simple_branch_maze')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('junction_color', 'yellow')
        self.declare_parameter('goal_color', 'green')
        self.team_enabled = bool(self.get_parameter('team_enabled').value)
        self.use_ground_truth = bool(self.get_parameter('use_ground_truth').value)
        self.spawn_pose = (
            float(self.get_parameter('spawn_x').value),
            float(self.get_parameter('spawn_y').value),
            float(self.get_parameter('spawn_yaw').value),
        )
        self.odom_origin = None
        self.robot_id = self.get_namespace().strip('/') or 'robot'
        self.run_id = uuid.uuid4().hex[:8]
        self.request_sequence = 0
        self.pending_team_request = None
        self.team_choice = None
        self.active_team_branch = None
        self.active_junction_world = None
        self.team_goal = None
        self.team_peers = {}
        self.team_route_order = None
        self.team_route_order_local = False
        self.last_team_pose_at = 0.0
        self.last_team_request_at = 0.0
        self.last_team_state_at = 0.0
        self.team_waiting_logged = False
        self.no_route_logged = False
        self.yield_start_pose = None
        self.yield_junction_world = None
        self.yield_heading = None
        self.yield_escape_heading = None
        self.yield_peer_name = None
        self.yield_resume_state = FOLLOW
        self.yield_resume_target = None
        self.yield_peer_was_yielding = False
        self.yield_rear_blocked = False
        if self.team_enabled:
            qos = QoSProfile(depth=10,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.team_pub = self.create_publisher(String, '/maze_team/report', 50)
            self.create_subscription(
                String, '/maze_team/decision', self._on_team_decision, qos)
            self.create_subscription(
                String, '/maze_team/state', self._on_team_state, qos)
        self.junction_choice = self.get_parameter('junction_choice').value
        if self.junction_choice not in ('straight_first', 'side_first'):
            self.get_logger().warning(
                f'junction_choice={self.junction_choice!r} ไม่รู้จัก; '
                'ใช้ side_first')
            self.junction_choice = 'side_first'
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.trail_pub = self.create_publisher(String, 'trail', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        if self.use_ground_truth:
            world_name = str(self.get_parameter('world_name').value)
            self.create_subscription(
                TFMessage, f'/world/{world_name}/dynamic_pose/info',
                self._on_ground_truth, 10)
        self.timer = self.create_timer(0.1, self.tick)

        self.sensors = QuadRGBArray(self)
        self.black_mode = self.sensors.junction_color == 'black'
        if self.black_mode:
            self.sensors.on_outer_hit = self._on_outer_hit
        self.ultrasonic = UltrasonicSensor(self)
        self.encoder = WheelEncoder(self)

        self.state = SEARCH
        self.state_start_time = time.monotonic()
        self.search_start_pose = None
        self.pose = None  # x, y, yaw in this robot's local odom frame
        self.last_odom_at = 0.0
        self.distance_total = 0.0
        self.junctions = []  # local breadcrumb stack; NOT a shared global map
        self.returning = False
        self.approach_start_distance = 0.0
        self.wait_for_yellow_clear = False
        self.junction_count = 0
        self.branch_angle = 0.0
        self.center_start = None
        self.center_heading = None
        self.turn_target = None
        self.turn_after = None
        self.turn_kind = None
        self.branch_capture_start_distance = 0.0
        self.branch_capture_heading = None
        self.branch_recovery_anchor = None
        self.junction_entry_pose = None
        self.junction_candidate = None
        self.junction_candidate_ticks = 0
        self.junction_needs_request = False
        self.outer_hit_position = None  # local sensor point of a black-mode hit
        self.outer_hit_world = None
        self.outer_hit_sides = set()  # 'left' (sensor 1) / 'right' (sensor 4)
        self.scan_sides = set()
        self.scan_hit_world = None
        self.trigger_resume_distance = 0.0
        self.scan_pending = False
        self.center_on_axle = False  # black mode: pivot where the scan pivoted
        self.scan_pivot = None
        self.scan_returning = False
        self.center_target = None
        self.scan_heading = None
        self.scan_phases = []
        self.scan_hits = {}
        self.line_search_after = FOLLOW
        self.last_search_log_at = 0.0
        self.align_last_black_at = 0.0
        self.align_last_side = 1
        self.inner_confirm_ticks = 0
        self.last_centered_pose = None
        self.last_path_color_distance = 0.0
        self.yield_line_missing_logged = False
        self.yield_at_line_end = False
        self.line_lost_anchor = None
        self.line_lost_phase = None
        self.line_lost_turn_direction = 1
        self.scan_rotation = 0.0
        self.scan_last_yaw = 0.0
        self.scan_saw_black = False
        self.line_search_given_up = False
        self.trail_id = 0
        self.trail_start_distance = 0.0
        self.last_trail_update = 0.0
        self.waiting_logged = False
        self.get_logger().debug(
            f'เริ่มทดลองตามเส้น ย้อนทางตัน และสร้างร่องรอยดิจิทัล; '
            f'junction_choice={self.junction_choice}')

    def _on_odom(self, msg):
        if self.use_ground_truth:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self._update_pose(p.x, p.y, yaw)

    def _on_ground_truth(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != self.robot_id:
                continue
            p = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self._update_pose(p.x, p.y, yaw)
            return

    def _update_pose(self, x, y, yaw):
        if self.pose is not None:
            step = math.hypot(x - self.pose[0], y - self.pose[1])
            if step < 0.5:  # ignore odom resets/teleports
                self.distance_total += step
        self.pose = (x, y, yaw)
        if self.odom_origin is None:
            self.odom_origin = self.pose
        self.last_odom_at = time.monotonic()

    def _world_pose(self):
        """Map this robot's independent odom origin to its Gazebo spawn pose."""
        if self.use_ground_truth:
            return self.pose
        if self.pose is None or self.odom_origin is None:
            return None
        x0, y0, yaw0 = self.odom_origin
        sx, sy, syaw = self.spawn_pose
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        rotation = syaw - yaw0
        return (sx + dx * math.cos(rotation) - dy * math.sin(rotation),
                sy + dx * math.sin(rotation) + dy * math.cos(rotation),
                syaw + angle_difference(self.pose[2], yaw0))

    def _world_sensor_position(self):
        pose = self._world_pose()
        if pose is None:
            return None
        return (pose[0] + RGB_FRONT_OFFSET_M * math.cos(pose[2]),
                pose[1] + RGB_FRONT_OFFSET_M * math.sin(pose[2]))

    def _team_report(self, payload):
        if not self.team_enabled:
            return
        msg = String()
        msg.data = json.dumps({'robot': self.robot_id, **payload})
        self.team_pub.publish(msg)

    def _on_team_decision(self, msg):
        try:
            decision = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        pending = self.pending_team_request
        if (pending is not None and decision.get('robot') == self.robot_id
                and decision.get('request_id') == pending['request_id']):
            self.team_choice = decision
            self.get_logger().debug(
                f"ทีมจัดทางแยก J{decision.get('junction_id')}: "
                f"{decision.get('choice')} (ร่วมทาง={decision.get('shared', False)})")

    def _on_team_state(self, msg):
        try:
            data = json.loads(msg.data)
            self.team_goal = data.get('goal')
            self.team_peers = {
                name: position for name, position in data.get('robots', {}).items()
                if name != self.robot_id
            }
            own = data.get('robots', {}).get(self.robot_id)
            if (not self.team_route_order_local and own is not None
                    and own.get('route_order') is not None):
                self.team_route_order = int(own['route_order'])
            self.last_team_state_at = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            return

    def _peer_ahead(self):
        pose = self._world_pose()
        if pose is None:
            return None
        closest = None
        for name, peer in self.team_peers.items():
            dx, dy = peer['x'] - pose[0], peer['y'] - pose[1]
            forward = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
            side = abs(-dx * math.sin(pose[2]) + dy * math.cos(pose[2]))
            if 0.0 < forward < 0.7 and side < 0.32:
                if closest is None or forward < closest[2]:
                    closest = (name, peer, forward)
        return closest

    def _peer_has_priority(self, name, peer):
        """The robot that entered the current route earlier has right of way."""
        root = self._yield_root(name)
        if root == self.robot_id:
            return False  # never yield to a robot already making room for us
        name = root
        peer = self.team_peers.get(root, peer)
        peer_order = peer.get('route_order')
        if self.team_route_order is not None and peer_order is not None:
            return (self.team_route_order, self.robot_id) > (int(peer_order), name)
        # Both robots must make the same choice before the first team update.
        return self.robot_id > name

    def _yield_root(self, name):
        seen = set()
        while name != self.robot_id and name not in seen:
            seen.add(name)
            target = self.team_peers.get(name, {}).get('yield_to')
            if not target:
                return name
            name = target
        return name

    def _yield_has_clearance(self):
        """Only retreat as the passing robot/queue approaches, not forever."""
        pose = self._world_pose()
        for name, peer in self.team_peers.items():
            if name != self.yield_peer_name and peer.get('yield_to') != self.yield_peer_name:
                continue
            dx, dy = peer['x'] - pose[0], peer['y'] - pose[1]
            along = dx * math.cos(self.yield_heading) + dy * math.sin(self.yield_heading)
            side = abs(-dx * math.sin(self.yield_heading) + dy * math.cos(self.yield_heading))
            if 0 <= along < 0.9 and side < 0.4:
                return False
        return True

    def _head_on_peer(self):
        pose = self._world_pose()
        if pose is None:
            return None
        closest = None
        for name, peer in self.team_peers.items():
            dx, dy = peer['x'] - pose[0], peer['y'] - pose[1]
            forward = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
            side = abs(-dx * math.sin(pose[2]) + dy * math.cos(pose[2]))
            opposite = abs(angle_difference(peer['heading'], pose[2]))
            if peer.get('state') in PIVOTING_STATES:
                continue
            if (0.0 < forward < HEAD_ON_DETECT_M and side < 0.32
                    and opposite > HEAD_ON_HEADING_RAD):
                if closest is None or forward < closest[2]:
                    closest = (name, peer, forward)
        return closest

    def _peer_behind_close(self):
        pose = self._world_pose()
        if pose is None:
            return False
        for peer in self.team_peers.values():
            dx, dy = peer['x'] - pose[0], peer['y'] - pose[1]
            forward = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
            side = abs(-dx * math.sin(pose[2]) + dy * math.cos(pose[2]))
            if -YIELD_REAR_CLEARANCE_M < forward < 0.0 and side < 0.32:
                return True
        return False

    def _yield_clear(self):
        pose = self._world_pose()
        if pose is None:
            return False
        peer = self.team_peers.get(self.yield_peer_name)
        if peer is None:
            return False
        chain = {self.robot_id: self.team_route_order or 0}
        cursor = self.yield_peer_name
        while cursor in self.team_peers and cursor not in chain:
            item = self.team_peers[cursor]
            chain[cursor] = item.get('route_order') or 0
            cursor = item.get('yield_to')
        if cursor == self.robot_id:
            # Exactly one winner also for a three-node circular wait.
            return min(chain, key=lambda name: (chain[name], name)) == self.robot_id
        dx, dy = peer['x'] - pose[0], peer['y'] - pose[1]
        lateral = abs(-dx * math.sin(self.yield_heading) +
                      dy * math.cos(self.yield_heading))
        along = dx * math.cos(self.yield_heading) + dy * math.sin(self.yield_heading)
        if along < -YIELD_CLEARANCE_M:
            return True
        if self.yield_peer_was_yielding:
            ahead = (dx * math.cos(self.yield_heading) +
                     dy * math.sin(self.yield_heading))
            return (peer.get('state') not in (YIELDING, YIELD_TURN,
                                               YIELD_ESCAPE, YIELD_WAIT)
                    and (ahead >= YIELD_CLEARANCE_M or
                         lateral >= YIELD_BRANCH_CLEARANCE_M))
        # Keep backing up through yellow / saved junctions. Only resume once
        # the earlier robot has actually left this corridor on a side branch.
        return (lateral >= YIELD_BRANCH_CLEARANCE_M and
                math.hypot(dx, dy) >= YIELD_CLEARANCE_M and
                peer.get('state') not in (YIELDING, YIELD_TURN,
                                          YIELD_ESCAPE, YIELD_WAIT))

    def _begin_yield(self, peer_name):
        peer_name = self._yield_root(peer_name)
        if peer_name == self.robot_id:
            return
        world = self._world_pose()
        if world is None:
            return
        if (self.state == CENTERING and not self.junction_needs_request
                and not self.scan_pending
                and self.active_team_branch is not None):
            self._team_report({
                'type': 'CANCEL_BRANCH',
                'request_id': self.active_team_branch['request_id'],
            })
            self.team_route_order = self.active_team_branch[
                'previous_route_order']
            self.team_route_order_local = self.team_route_order is not None
            self.active_team_branch = None
        if self.pending_team_request is not None:
            self._team_report({
                'type': 'CANCEL_BRANCH',
                'request_id': self.pending_team_request['request_id'],
            })
            self.pending_team_request = None
            self.team_choice = None
        # Not yet asked: the junction is seen again after the yield.
        self.junction_needs_request = False
        rescan = self.scan_pending
        self.scan_pending = False
        self.center_on_axle = False
        self.yield_start_pose = world[:2]
        self.yield_heading = world[2]
        self.yield_peer_name = peer_name
        if self.state != YIELD_RESUME:
            self.yield_resume_state = RETURNING if self.returning else FOLLOW
            self.yield_resume_target = world[:2]
            if rescan and self.scan_hit_world is not None:
                # Come back with the sensors short of the arm so it is hit again.
                hx, hy = self.scan_hit_world
                self.yield_resume_target = (
                    hx - RGB_FRONT_OFFSET_M * math.cos(world[2]),
                    hy - RGB_FRONT_OFFSET_M * math.sin(world[2]))
        self.yield_peer_was_yielding = self.team_peers.get(
            peer_name, {}).get('state') in (
                YIELDING, YIELD_TURN, YIELD_ESCAPE, YIELD_WAIT)
        self.yield_junction_world = self.active_junction_world
        if self.yield_junction_world is None and self.junctions:
            self.yield_junction_world = self.junctions[-1]['world_position']
        if self.yield_junction_world is None:
            peer_junction = self.team_peers.get(peer_name, {}).get('yield_junction')
            if peer_junction is not None:
                self.yield_junction_world = (peer_junction['x'], peer_junction['y'])
        self.yield_escape_heading = None
        self.yield_rear_blocked = False
        self.yield_at_line_end = False
        self._change_state(YIELDING)
        self.get_logger().debug(
            f'ถอยตามเส้นให้ {peer_name} จนกว่าเขาจะเลี้ยวพ้นทางร่วม')

    def _request_team_branch(self, returning, left_open, right_open,
                             position_override=None, straight_open=True,
                             approach_heading=None, exit_headings=None):
        world = self._world_pose()
        position = position_override or self._world_sensor_position()
        if world is None or position is None:
            return
        if approach_heading is not None:
            # A black-mode scan ends rotated; headings come from its start.
            world = (world[0], world[1], approach_heading)
        exit_headings = exit_headings or {}
        self.request_sequence += 1
        choices = []
        if not returning and straight_open:
            choices.append({'name': 'straight',
                            'heading': exit_headings.get('straight', world[2])})
        if left_open:
            choices.append({'name': 'left', 'heading': exit_headings.get(
                'left', world[2] + math.pi / 2)})
        if right_open:
            choices.append({'name': 'right', 'heading': exit_headings.get(
                'right', world[2] - math.pi / 2)})
        choices.append({'name': 'back', 'heading': world[2] + math.pi})
        if returning and self.junctions:
            node = self.junctions[-1]
            original = node.get('world_heading', world[2] + math.pi)
            measured = node.get('exit_headings', {})  # black-mode scan
            headings = [(original + math.pi, True)]
            if node.get('straight_open', True):
                headings.insert(0, (measured.get('straight', original), False))
            if node['left_open']:
                headings.append((measured.get('left', original + math.pi / 2), False))
            if node['right_open']:
                headings.append((measured.get('right', original - math.pi / 2), False))
            choices = []
            for heading, fallback in headings:
                turn = angle_difference(heading, world[2])
                name = ('straight' if abs(turn) < math.pi / 4 else
                        'back' if abs(turn) > 3 * math.pi / 4 else
                        'left' if turn > 0 else 'right')
                choices.append({'name': name, 'heading': heading, 'fallback': fallback})
        self.pending_team_request = {
            'type': 'BRANCH_REQUEST',
            'request_id': f'{self.run_id}-{self.request_sequence}',
            'x': position[0], 'y': position[1],
            'approach_heading': world[2],
            'choices': choices,
        }
        available = {option['name'] for option in choices}
        self.get_logger().info(
            f"แยก ({'ขากลับ' if returning else 'ขาไป'}; ทิศเทียบกับหน้ารถตอนนี้): "
            f"ซ้ายเปิด={'left' in available} หน้าเปิด={'straight' in available} "
            f"ขวาเปิด={'right' in available} "
            f"[{'จากความจำกิ่งเดิม' if returning else 'กิ่งข้างจาก RGB; หน้าตรงยังต้องสำรวจ'}]")
        self.team_choice = None
        self._team_report(self.pending_team_request)
        self.last_team_request_at = time.monotonic()

    def _apply_team_choice(self):
        choice = self.team_choice
        if choice is None or choice.get('choice') is None:
            return False
        branch = choice['choice']
        self.active_team_branch = {
            'junction_id': choice['junction_id'],
            'direction': choice['direction'],
            'choice': branch,
            'request_id': choice['request_id'],
            'previous_route_order': self.team_route_order,
        }
        self.active_junction_world = (
            float(choice['junction_x']), float(choice['junction_y']))
        # Going straight keeps the robot in the same corridor. Its original
        # entry order must survive a junction, especially when it later
        # returns from the dead end beyond that junction.
        if branch != 'straight' and choice.get('route_order') is not None:
            self.team_route_order = int(choice['route_order'])
            self.team_route_order_local = True
        self.branch_angle = (
            math.pi / 2 if branch == 'left' else
            -math.pi / 2 if branch == 'right' else
            math.pi if branch == 'back' else 0.0)
        if 'heading' in choice:
            self.branch_angle = angle_difference(float(choice['heading']), self._world_pose()[2])
        label = {'left': 'ซ้าย', 'right': 'ขวา', 'straight': 'ตรง', 'back': 'กลับทางเดิม'}[branch]
        labels = {'left': 'ซ้าย', 'right': 'ขวา', 'straight': 'หน้า', 'back': 'หลัง'}
        details = []
        for item in choice.get('branches', []):
            status = ('ทางตัน' if item['status'] == 'BLOCKED' else
                      'เคยผ่าน/เลือกแล้ว:' + ','.join(item['visited_by']) if item['visited_by'] else
                      'ยังไม่สำรวจ')
            if item.get('failed_attempts'):
                status += f"/จับเส้นพลาด{item['failed_attempts']}ครั้ง"
            details.append(f"{labels[item['name']]}={status}")
        if details:
            self.get_logger().info(f"J{choice['junction_id']} ทีมจำไว้: " + ' | '.join(details))
        reason = {'UNEXPLORED_BRANCH': 'สำรวจทางที่ยังไม่เคยไป',
                  'KNOWN_RETURN_ROUTE': 'ไม่เหลือกิ่งอื่นที่เปิด จึงย้อนทางเข้าเดิม',
                  'REVISIT_OPEN_BRANCH': 'กิ่งที่เปิดเคยถูกเลือกแล้ว จึงใช้ทางเดิมต่อ',
                  'TOWARD_GOAL': 'เดินไปทางเป้าหมายที่ทีมพบ'}.get(choice.get('reason'), 'ทีมเลือกทาง')
        self.get_logger().info(f'เลือก{label}ที่ J{choice["junction_id"]}: {reason}')
        self.pending_team_request = None
        self.team_choice = None
        self.no_route_logged = False
        return True

    def _ask_at_junction_centre(self):
        """Wheels are on the junction; RGB now looks past it at the straight arm."""
        self.junction_needs_request = False
        node = self.junctions[-1]
        node['straight_open'] = self.sensors.any_black() or self.sensors.any_yellow()
        self.get_logger().info(
            f"J{node['id']} ทางตรง{'เปิด' if node['straight_open'] else 'ปิด'} "
            '(อ่านจากพื้นเลยกลางแยก)')
        if self.team_enabled:
            self._request_team_branch(
                False, node['left_open'], node['right_open'],
                node['world_position'], straight_open=node['straight_open'])
            self._change_state(JUNCTION_PAUSE)
            return
        if self.branch_angle == 0.0 and not node['straight_open']:
            self.branch_angle = math.pi / 2 if node['left_open'] else -math.pi / 2
        self._leave_junction_center()

    def _leave_junction_center(self):
        if self.branch_angle == 0.0:
            # Cross the marker without treating its yellow centre as
            # a lost line; the new straight segment begins ahead.
            self.returning = False
            self.wait_for_yellow_clear = True
            self._new_segment()
            self.branch_capture_heading = self.pose[2]
            self.branch_capture_start_distance = self.distance_total
            self._change_state(BRANCH_CAPTURE)
        else:
            self._start_turn(self.branch_angle, FOLLOW, 'BRANCH')

    def _sensor_position(self):
        x, y, yaw = self.pose
        return (x + RGB_FRONT_OFFSET_M * math.cos(yaw),
                y + RGB_FRONT_OFFSET_M * math.sin(yaw))

    def _distance_to_junction(self):
        if not self.junctions or self.pose is None:
            return None
        sx, sy = self._sensor_position()
        jx, jy = self.junctions[-1]['position']
        return math.hypot(sx - jx, sy - jy)

    def _return_reached_junction(self):
        """Detect crossing the saved junction plane, not one exact RGB frame."""
        if not self.junctions or self.pose is None:
            return False
        node = self.junctions[-1]
        sx, sy = self._sensor_position()
        jx, jy = node['position']
        heading = node['heading']
        dx, dy = sx - jx, sy - jy
        along_original_heading = dx * math.cos(heading) + dy * math.sin(heading)
        sideways = -dx * math.sin(heading) + dy * math.cos(heading)
        return (along_original_heading <= 0.02
                and abs(sideways) <= JUNCTION_LATERAL_TOLERANCE_M)

    def _trail_event(self, status):
        """Virtual scent: report the current segment even before a junction."""
        if not self.trail_id:
            return
        data = {
            'robot': self.get_namespace().strip('/') or 'robot',
            'segment_id': self.trail_id,
            'status': status,
            'distance_m': round(self.distance_total - self.trail_start_distance, 3),
        }
        if self.pose is not None:
            data['odom_xy'] = [round(self.pose[0], 3), round(self.pose[1], 3)]
        if status in ('JUNCTION', 'BACK_AT_JUNCTION') and self.junctions:
            node = self.junctions[-1]
            data['junction'] = {
                'id': node['id'],
                'left_open': node['left_open'],
                'right_open': node['right_open'],
                'odom_xy': [round(value, 3) for value in node['position']],
            }
        message = String()
        message.data = json.dumps(data, ensure_ascii=False)
        self.trail_pub.publish(message)
        self.last_trail_update = time.monotonic()

    def _new_segment(self):
        self.trail_id += 1
        self.trail_start_distance = self.distance_total
        if self.sensors.any_black() or self.sensors.any_yellow():
            self._trail_event('IN_PROGRESS')

    def _change_state(self, state):
        labels = {RETURNING: 'ย้อนกลับไปเลือกทางที่แยก',
                  YIELDING: 'ถอยให้หุ่นที่มาก่อนผ่าน',
                  YIELD_RESUME: 'เพื่อนผ่านแล้ว กลับไปเดินต่อ',
                  LINE_LOST: 'เส้นหาย กำลังกลับไปหาจุดที่เคยเห็นเส้น',
                  GOAL_REACHED: 'ถึงเป้าหมายแล้ว'}
        if state != self.state and state in labels:
            self.get_logger().info(labels[state])
        if self.state != state:
            self.get_logger().debug(f'{self.state} -> {state}')
        self.state = state
        self.state_start_time = time.monotonic()
        # A latched hit belongs to the drive that saw it.
        self.outer_hit_position = None
        self.outer_hit_world = None
        self.outer_hit_sides = set()

    def _start_turn(self, angle, after, kind):
        if self.pose is None:
            self.get_logger().warning('ยังไม่มี odom: รอข้อมูลก่อนเริ่มหมุน')
            return
        self.turn_target = self.pose[2] + angle
        self.turn_after = after
        self.turn_kind = kind
        self._change_state(TURNING)

    def _follow_line(self, cmd, speed):
        if not self.sensors.any_black():
            if self.sensors.any_yellow():
                # Yellow paint may hide all of the narrow black line for a
                # moment. Keep the last centered heading until black returns.
                cmd.linear.x = min(speed, APPROACH_SPEED)
                if self.last_centered_pose is not None:
                    cmd.angular.z = max(-MAX_TURN, min(
                        MAX_TURN, angle_difference(
                            self.last_centered_pose[2], self.pose[2])))
                return
            self._begin_line_recovery(self.state)
            self.get_logger().warning(
                'เส้นหายระหว่างขับ: กลับไปตำแหน่งล่าสุดที่เห็นเส้นชัด')
            return
        if (not self.sensors.black[1] and not self.sensors.black[2]
                and (self.sensors.black[0] or self.sensors.black[3])):
            # Outer-only is a line edge, not a centered path to drive along.
            self.line_search_after = self.state
            self.align_last_black_at = time.monotonic()
            self.inner_confirm_ticks = 0
            self.align_last_side = 1 if self.sensors.black[0] else -1
            self._change_state(ALIGN_LINE)
            return
        error = self._line_error()
        if error:
            self.align_last_side = 1 if error > 0 else -1
        if self.sensors.black[1] and self.sensors.black[2]:
            # Keep the requested cruising speed only while centred.
            cmd.linear.x = speed
            cmd.angular.z = self._line_follow_turn()
        else:
            # On a 5 cm line, 0.5 m/s with a shallow turn loses the line.
            cmd.linear.x = min(speed, LINE_CORRECTION_SPEED)
            turn = LINE_CORRECTION_GAIN * self._line_error()
            cmd.angular.z = max(-LINE_CORRECTION_MAX_TURN,
                                min(LINE_CORRECTION_MAX_TURN, turn))

    def _yield_reverse_command(self, cmd):
        """Back along the same line without evaluating yellow junctions."""
        world = self._world_pose()
        heading_error = angle_difference(self.yield_heading, world[2])
        # The side arm at a yellow junction is not a lateral tracking error.
        line_error = (self._line_error() if self.sensors.any_black()
                      and not self.sensors.any_yellow() else 0.0)
        dx = world[0] - self.yield_start_pose[0]
        dy = world[1] - self.yield_start_pose[1]
        lateral = -dx * math.sin(self.yield_heading) + dy * math.cos(self.yield_heading)
        cmd.linear.x = -REVERSE_SPEED
        cmd.angular.z = max(-ALIGN_MAX_TURN, min(
            ALIGN_MAX_TURN,
            YIELD_HEADING_GAIN * heading_error + 2.0 * lateral -
            YIELD_LINE_TURN_GAIN * line_error))

    def _begin_line_recovery(self, after_state, anchor=None):
        self.line_search_after = after_state
        anchor = anchor or self.last_centered_pose or self.pose
        previous = self.line_lost_anchor
        # Losing the line again at the same spot continues the same search.
        if (previous is None or
                math.hypot(anchor[0] - previous[0], anchor[1] - previous[1]) > 0.05):
            self.scan_rotation = 0.0
            self.scan_saw_black = False
            self.line_search_given_up = False
        self.line_lost_anchor = anchor
        self.line_lost_phase = 'RETURN_TO_ANCHOR'
        self.line_lost_turn_direction = self.align_last_side
        self._change_state(LINE_LOST)

    def _reject_unseen_branch(self):
        """Undo a turn that never reached black, using an observed inbound pose."""
        if self.team_enabled and self.active_team_branch is not None:
            self._team_report({'type': 'RETRY_BRANCH', **self.active_team_branch})
            self.active_team_branch = None
        self.branch_recovery_anchor = None
        self.wait_for_yellow_clear = False
        self.returning = False
        self._begin_line_recovery(FOLLOW, self.junction_entry_pose)

    def _recover_lost_line(self, cmd, now):
        """Return to a recently observed line position, then sweep in place."""
        if self.line_search_given_up:
            return  # stay stopped rather than spin; see the warning below
        if self.sensors.any_black():
            self.scan_saw_black = True
        anchor_yaw = self.line_lost_anchor[2]
        # Compare axle points: turning in place moves base_link, not the axle.
        anchor_x, anchor_y = self._axle_point(self.line_lost_anchor)
        axle_x, axle_y = self._axle_point(self.pose)
        dx = anchor_x - axle_x
        dy = anchor_y - axle_y
        distance = math.hypot(dx, dy)
        if distance > ANCHOR_ARRIVAL_M:
            self.line_lost_phase = 'RETURN_TO_ANCHOR'
            toward = math.atan2(dy, dx)
            forward_error = angle_difference(toward, self.pose[2])
            reverse_error = angle_difference(toward + math.pi, self.pose[2])
            backwards = abs(reverse_error) < abs(forward_error)
            heading_error = reverse_error if backwards else forward_error
            if abs(heading_error) > 0.25:
                cmd.angular.z = (ANCHOR_TURN_SPEED if heading_error > 0
                                 else -ANCHOR_TURN_SPEED)
                return
            if backwards:
                if self.team_enabled and self._peer_behind_close():
                    return
                cmd.linear.x = -ANCHOR_TRAVEL_SPEED
            else:
                if self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M):
                    return
                cmd.linear.x = ANCHOR_TRAVEL_SPEED
            cmd.angular.z = max(-MAX_TURN, min(MAX_TURN,
                                              1.5 * heading_error))
            return

        if self.line_lost_phase != 'SCAN_AT_ANCHOR':
            heading_error = angle_difference(anchor_yaw, self.pose[2])
            if abs(heading_error) > 0.15:
                self.line_lost_phase = 'ALIGN_ANCHOR'
                cmd.angular.z = (ANCHOR_TURN_SPEED if heading_error > 0
                                 else -ANCHOR_TURN_SPEED)
                return
            self.line_lost_phase = 'SCAN_AT_ANCHOR'
            self.scan_last_yaw = self.pose[2]

        # A narrow left/right sweep can miss a line after drifting sideways.
        # Rotate all the way around until RGB finds black, but only once.
        self.scan_rotation += abs(angle_difference(self.pose[2], self.scan_last_yaw))
        self.scan_last_yaw = self.pose[2]
        if self.scan_rotation >= 2 * math.pi:
            # With walls at dead ends, a lost line is not proof of one.
            if self.team_enabled and self.active_team_branch is not None:
                self._team_report({'type': 'RETRY_BRANCH', **self.active_team_branch})
                self.active_team_branch = None
            # A U-turn only helps on the way out with the line seen behind;
            # returning, it would face the dead end again.
            if self.scan_saw_black and self.line_search_after == FOLLOW:
                self.get_logger().warning(
                    'หมุนหาเส้นครบรอบแล้วไม่เจอ: กลับรถย้อนไปแยกล่าสุด')
                self.line_lost_anchor = None
                self._start_turn(math.pi, RETURNING, 'UTURN')
                return
            self.line_search_given_up = True
            self.get_logger().error(
                'หมุนหาเส้นครบรอบแล้วไม่เจอทางที่ไปต่อได้: หยุดรอ (ต้องช่วยวางหุ่นกลับบนเส้น)')
            return
        cmd.angular.z = self.line_lost_turn_direction * ANCHOR_TURN_SPEED

    def _axle_point(self, pose):
        x, y, yaw = pose
        return (x + WHEEL_X_OFFSET_M * math.cos(yaw),
                y + WHEEL_X_OFFSET_M * math.sin(yaw))

    def _line_error(self):
        weights = [2, 1, -1, -2]  # leftmost to rightmost
        active = [weight for weight, black in zip(weights, self.sensors.black)
                  if black]
        return sum(active) / len(active) if active else 0.0

    def _line_follow_turn(self):
        turn = TURN_GAIN * self._line_error()
        return max(-MAX_TURN, min(MAX_TURN, turn))

    def _remember_junction(self, position, world_position, heading,
                           world_heading, left_open, right_open):
        """Push a new outward junction, or reuse the saved one at this spot."""
        known_index = next((index for index in
                            range(len(self.junctions) - 1, -1, -1)
                            if (math.hypot(
                                world_position[0] - self.junctions[index][
                                    'world_position'][0],
                                world_position[1] - self.junctions[index][
                                    'world_position'][1]) < 0.25
                                and abs(angle_difference(
                                    heading, self.junctions[index][
                                        'heading'])) < 0.6)), None)
        if known_index is None:
            self.junction_count += 1
            self.junctions.append({
                'id': self.junction_count,
                'position': position,
                'world_position': world_position,
                'heading': heading,
                'world_heading': world_heading,
                'left_open': left_open,
                'right_open': right_open,
                'straight_open': True,
            })
        else:
            self.junctions = self.junctions[:known_index + 1]
            node = self.junctions[-1]
            node['position'] = position
            node['world_position'] = world_position
            node['left_open'] |= left_open
            node['right_open'] |= right_open
            self.get_logger().debug(f"กลับถึงแยกที่เคยบันทึก J{node['id']}")
        return self.junctions[-1]

    def _on_outer_hit(self):
        """Image callback: latch where an outer sensor first touched black."""
        sides = {side for side, index in (('left', 0), ('right', 3))
                 if self.sensors.black[index]}
        if self.state == CENTERING and self.scan_pending:
            # The other arm may pass under its sensor a frame later.
            self.scan_sides |= sides
            return
        if (self.pose is None or self.state not in (FOLLOW, RETURNING)
                or self.distance_total < self.trigger_resume_distance):
            return
        if self.outer_hit_position is None:
            self.outer_hit_position = self._sensor_position()
            self.outer_hit_world = self._world_sensor_position()
        self.outer_hit_sides |= sides

    def _begin_black_junction(self):
        """Bring the wheel axle onto the hit point; the sweep pivots there."""
        self.center_target = self.outer_hit_position
        self.scan_hit_world = self.outer_hit_world
        self.scan_sides = set(self.outer_hit_sides)
        self.center_heading = self.pose[2]
        self.scan_returning = self.returning
        self.junction_entry_pose = self.last_centered_pose or self.pose
        self.scan_pending = True
        self.center_on_axle = True
        self.get_logger().debug(
            f'ตัวนอกเห็นเส้น: หยุดแล้วเข้ากลางจุดเพื่อหมุนสแกน; '
            f'black={self.sensors.black}')
        self._change_state(CENTERING)

    def _start_junction_scan(self):
        self.scan_pending = False
        self.center_on_axle = False
        self.scan_pivot = self._axle_point(self.pose)
        self.scan_heading = self.pose[2]
        self.scan_hits = {'straight': [], 'left': [], 'right': []}
        # Wheels on the centre: the inner sensors already sit over the
        # straight arm, so only the sides an outer sensor touched are swept.
        if self.sensors.black[1] or self.sensors.black[2]:
            self.scan_hits['straight'].append(0.0)
        self.scan_phases = [side for side in ('left', 'right')
                            if side in self.scan_sides]
        self._change_state(JUNCTION_SCAN)

    def _scan_junction(self, cmd):
        """Sweep to ±130° on each touched side, noting inner-sensor black."""
        theta = angle_difference(self.pose[2], self.scan_heading)
        if self.sensors.black[1] or self.sensors.black[2]:
            if abs(theta) <= SCAN_STRAIGHT_MAX_RAD:
                self.scan_hits['straight'].append(theta)
            elif SCAN_SIDE_MIN_RAD <= theta <= SCAN_LIMIT_RAD:
                self.scan_hits['left'].append(theta)
            elif -SCAN_LIMIT_RAD <= theta <= -SCAN_SIDE_MIN_RAD:
                self.scan_hits['right'].append(theta)
        while self.scan_phases and (
                (self.scan_phases[0] == 'left' and theta >= SCAN_LIMIT_RAD) or
                (self.scan_phases[0] == 'right' and theta <= -SCAN_LIMIT_RAD)):
            self.scan_phases.pop(0)
        if not self.scan_phases:
            self._finish_junction_scan()
        else:
            cmd.angular.z = (ROTATE_SPEED if self.scan_phases[0] == 'left'
                             else -ROTATE_SPEED)

    def _finish_junction_scan(self):
        exits = {name: sum(angles) / len(angles)
                 for name, angles in self.scan_hits.items() if angles}
        h0 = self.scan_heading
        world_h0 = h0 + angle_difference(self._world_pose()[2], self.pose[2])
        labels = {'left': 'ซ้าย', 'straight': 'ตรง', 'right': 'ขวา'}
        self.get_logger().info(
            'สแกนแยก: ' + ' '.join(
                f"{labels[name]}={'มี' if name in exits else 'ไม่มี'}"
                for name in ('left', 'straight', 'right')))
        self.trigger_resume_distance = (self.distance_total +
                                        JUNCTION_ESCAPE_DISTANCE_M)
        if not exits:
            if self.team_enabled and self.active_team_branch is not None:
                self._team_report({'type': 'BLOCKED', **self.active_team_branch})
                self.active_team_branch = None
            self._trail_event('BLOCKED_BRANCH')
            self.get_logger().warning('สแกนแล้วไม่มีทางไปต่อ: กลับรถ')
            self._start_turn(angle_difference(h0 + math.pi, self.pose[2]),
                             RETURNING, 'UTURN')
            return
        if len(exits) == 1:
            # A corner, or a wobble that touched the line edge: no choice here.
            (name, theta), = exits.items()
            self.get_logger().info(f'ทางเดียว ({labels[name]}): ไปต่อโดยไม่ถามทีม')
            self._start_turn(angle_difference(h0 + theta, self.pose[2]),
                             RETURNING if self.scan_returning else FOLLOW,
                             'CORNER')
            return
        if self.scan_returning:
            self._ask_at_missed_junction(exits, h0, world_h0)
            return
        node = self._remember_junction(
            self.scan_pivot, self.scan_hit_world, h0, world_h0,
            'left' in exits, 'right' in exits)
        node['straight_open'] = 'straight' in exits
        node['exit_headings'] = {name: world_h0 + theta
                                 for name, theta in exits.items()}
        self.active_junction_world = node['world_position']
        self.returning = False
        self._trail_event('JUNCTION')
        if self.team_enabled:
            self._request_team_branch(
                False, node['left_open'], node['right_open'],
                node['world_position'], straight_open=node['straight_open'],
                approach_heading=world_h0, exit_headings=node['exit_headings'])
            self._change_state(JUNCTION_PAUSE)
            return
        order = (('left', 'right', 'straight')
                 if self.junction_choice == 'side_first'
                 else ('straight', 'left', 'right'))
        name = next(n for n in order if n in exits)
        self.branch_angle = angle_difference(h0 + exits[name], self.pose[2])
        self._leave_junction_center()

    def _ask_at_missed_junction(self, exits, h0, world_h0):
        # Returning through a junction the outward pass never saved. Store it
        # as the outward pass saw it (facing h0 + pi): robot-left is
        # outward-right, and the arm behind is the dead end just explored.
        self.junction_count += 1
        node = {
            'id': self.junction_count,
            'position': self.scan_pivot,
            'world_position': self.scan_hit_world,
            'heading': h0 + math.pi,
            'world_heading': world_h0 + math.pi,
            'left_open': 'right' in exits,
            'right_open': 'left' in exits,
            'straight_open': False,
            'exit_headings': {
                {'left': 'right', 'right': 'left'}[name]: world_h0 + theta
                for name, theta in exits.items() if name != 'straight'},
        }
        self.junctions.append(node)
        self.active_junction_world = node['world_position']
        self.get_logger().warning('ขาไปพลาดการบันทึกแยก; สแกนพบตอนขากลับ')
        self._trail_event('BACK_AT_JUNCTION')
        # Wheels are already on the centre: leave from here once chosen.
        self.returning = False
        if self.team_enabled:
            self._request_team_branch(
                True, node['right_open'], node['left_open'],
                node['world_position'], approach_heading=world_h0)
            self._change_state(JUNCTION_PAUSE)
            return
        name = next((n for n in ('left', 'right', 'straight') if n in exits))
        self.branch_angle = angle_difference(h0 + exits[name], self.pose[2])
        self._leave_junction_center()

    def _at_junction(self, returning, left_open=False, right_open=False):
        self.junction_entry_pose = self.last_centered_pose or self.pose
        self.get_logger().info(
            f'เห็นแยก: black={self.sensors.black}, yellow={self.sensors.yellow}')
        if not returning:
            self._remember_junction(
                self._sensor_position(), self._world_sensor_position(),
                self.pose[2], self._world_pose()[2], left_open, right_open)
            self._trail_event('JUNCTION')
            if self.junction_choice == 'side_first':
                self.branch_angle = math.pi / 2 if left_open else -math.pi / 2
                self.get_logger().debug(
                    f'ขาไปเลือกกิ่ง{"ซ้าย" if left_open else "ขวา"}ทันที')
        else:
            self._trail_event('BACK_AT_JUNCTION')
            node = self.junctions[-1]
            # After a U-turn, original left is now robot-right and vice versa.
            if node['left_open']:
                self.branch_angle = -math.pi / 2
                chosen = 'ขวา (กิ่งซ้ายเดิม)'
            elif node['right_open']:
                self.branch_angle = math.pi / 2
                chosen = 'ซ้าย (กิ่งขวาเดิม)'
            else:
                self.get_logger().warning('แยกที่บันทึกไว้ยังไม่มีกิ่งข้างให้เลือก')
                if not self.team_enabled:
                    return False
                chosen = 'รอตัวกลางเลือกจากกิ่งที่จำได้'
            self.get_logger().debug(f'ขากลับเลือกเลี้ยว{chosen}')
        if not returning:
            # Bring the wheels onto the junction first; the RGB row then looks
            # past it and shows whether a straight branch exists.
            self.active_junction_world = self.junctions[-1]['world_position']
            self.junction_needs_request = True
            self.center_start = self.pose[:2]
            self.center_heading = self.pose[2]
            self._change_state(CENTERING)
            return True
        if self.team_enabled:
            # Left/right swap when the robot approaches from the other side.
            self._request_team_branch(
                True, self.junctions[-1]['right_open'],
                self.junctions[-1]['left_open'],
                self.junctions[-1].get('world_position'))
        self._change_state(JUNCTION_PAUSE)
        self.get_logger().debug(
            f"แยก {self.junctions[-1]['id']} "
            f"ซ้ายเปิด={self.junctions[-1]['left_open']} "
            f"ขวาเปิด={self.junctions[-1]['right_open']} "
            f"({'ย้อนมาเลือกกิ่งข้าง' if returning else 'ขาไปลองทางตรง'})")
        return True

    def tick(self):
        now = time.monotonic()
        cmd = Twist()

        if self.team_enabled and self.pose is not None:
            if now - self.last_team_pose_at >= 0.5:
                world = self._world_pose()
                self._team_report({'type': 'POSE', 'x': world[0], 'y': world[1],
                                   'heading': world[2], 'state': self.state,
                                   'route_order': self.team_route_order,
                                   'returning': self.returning,
                                   'yield_to': self.yield_peer_name if self.state == YIELDING else None,
                                   'yield_junction': (
                                       {'x': self.yield_junction_world[0],
                                        'y': self.yield_junction_world[1]}
                                       if self.yield_junction_world is not None
                                       else None),
                                   'yield_escape_heading': self.yield_escape_heading})
                self.last_team_pose_at = now
            if (self.pending_team_request is not None and self.team_choice is None
                    and now - self.last_team_request_at >= 1.0):
                self._team_report(self.pending_team_request)
                self.last_team_request_at = now

        if self.team_enabled and self.team_goal is not None and self.pose is not None:
            world = self._world_pose()
            goal_distance = math.hypot(world[0] - self.team_goal['x'],
                                       world[1] - self.team_goal['y'])
            peer_ahead = self._peer_ahead()
            queued_behind_teammate = (
                goal_distance < TEAM_ASSEMBLY_RADIUS_M
                and peer_ahead is not None
                and peer_ahead[1].get('state') == GOAL_REACHED)
            if goal_distance < TEAM_GOAL_RADIUS_M or queued_behind_teammate:
                if self.state != GOAL_REACHED:
                    if queued_behind_teammate and goal_distance >= TEAM_GOAL_RADIUS_M:
                        self.get_logger().debug(
                            f'รวมพลหลังเพื่อนใกล้เป้าหมาย ระยะ {goal_distance:.2f} m')
                        self._trail_event('TEAM_ASSEMBLED_NEAR_GOAL')
                    else:
                        self.get_logger().debug('ถึงจุดรวมพลใกล้เป้าหมายของทีม')
                        self._trail_event('TEAM_GOAL_REACHED')
                    self._change_state(GOAL_REACHED)

        if self.state == GOAL_REACHED:
            self.cmd_pub.publish(cmd)
            return

        if (not self.sensors.fresh(SIGNAL_STALE_SECONDS)
                or not self.ultrasonic.fresh(SIGNAL_STALE_SECONDS)
                or self.pose is None
                or now - self.last_odom_at > SIGNAL_STALE_SECONDS):
            if not self.waiting_logged:
                self.get_logger().warning(
                    'รอข้อมูล: '
                    f'RGB={self.sensors.fresh(SIGNAL_STALE_SECONDS)}, '
                    f'Ultrasonic={self.ultrasonic.fresh(SIGNAL_STALE_SECONDS)}, '
                    f'ตำแหน่ง={self.pose is not None and now - self.last_odom_at <= SIGNAL_STALE_SECONDS} '
                    '(True=ได้รับแล้ว, False=ยังไม่มีหรือเก่า)')
                self.waiting_logged = True
            self.cmd_pub.publish(cmd)
            return
        if self.waiting_logged:
            self.get_logger().info('ข้อมูลเซนเซอร์และตำแหน่งครบแล้ว: เริ่มทำงานต่อ')
            self.waiting_logged = False
        if self.team_enabled and now - self.last_team_state_at > 3.0:
            if not self.team_waiting_logged:
                self.get_logger().warning('ตัวกลางทีมไม่ตอบ: หยุดรอการสื่อสาร')
                self.team_waiting_logged = True
            self.cmd_pub.publish(cmd)
            return
        if self.team_waiting_logged:
            self.get_logger().debug('การสื่อสารกับทีมกลับมาแล้ว')
            self.team_waiting_logged = False

        if self.sensors.any_black() or self.sensors.any_yellow():
            self.last_path_color_distance = self.distance_total
            self.yield_line_missing_logged = False
        if self.sensors.black[1] and self.sensors.black[2]:
            self.last_centered_pose = self.pose

        if (self.team_enabled and self.state in
                (SEARCH, FOLLOW, APPROACH_JUNCTION, JUNCTION_PAUSE,
                 CENTERING, BRANCH_CAPTURE, RETURNING,
                 REACQUIRE, ALIGN_LINE, YIELD_RESUME)):
            head_on = self._head_on_peer()
            if (head_on is not None and
                    self._peer_has_priority(head_on[0], head_on[1])):
                self._begin_yield(head_on[0])
                self.cmd_pub.publish(cmd)
                return

        if self.state == YIELDING:
            if self._yield_clear():
                self.wait_for_yellow_clear = False
                self._change_state(YIELD_RESUME)
                self.yield_junction_world = None
                self.yield_peer_name = None
                self.yield_peer_was_yielding = False
                self.yield_rear_blocked = False
                self.get_logger().debug(
                    'หุ่นที่มาก่อนเลี้ยวพ้นแล้ว ย้อนตามเส้นกลับไปตำแหน่งเดิม '
                    'โดยไม่เลือกแยกระหว่างทาง')
            elif self._yield_has_clearance():
                pass  # enough room: let the leader move; keep the yield grant
            elif self._peer_behind_close():
                if not self.yield_rear_blocked:
                    self.get_logger().warning('มีหุ่นด้านหลัง หยุดก่อนถอยหลีกทาง')
                    self.yield_rear_blocked = True
            elif (self.yield_at_line_end or
                  self.distance_total - self.last_path_color_distance >
                  (PATH_COLOR_GRACE_M if self.black_mode else YIELD_PATH_GAP_M)):
                # Backed off the end of the line (e.g. last in the start
                # queue): creep forward until the RGB sees it again, then
                # hold there for the rest of this yield; spinning would lose
                # the line for good.
                if not self.yield_at_line_end:
                    self.get_logger().warning(
                        'ถอยจนสุดเส้นแล้ว: หยุดรอตรงนี้โดยยังให้ทางอยู่')
                    self.yield_at_line_end = True
                if not (self.sensors.any_black() or self.sensors.any_yellow()):
                    cmd.linear.x = ALIGN_SPEED
            else:
                self.yield_rear_blocked = False
                heading_error = angle_difference(
                    self.yield_heading, self._world_pose()[2])
                if abs(heading_error) > 0.25:
                    cmd.angular.z = (ANCHOR_TURN_SPEED if heading_error > 0
                                     else -ANCHOR_TURN_SPEED)
                else:
                    self._yield_reverse_command(cmd)
            self.cmd_pub.publish(cmd)
            return

        if self.state == YIELD_RESUME:
            world = self._world_pose()
            dx = world[0] - self.yield_resume_target[0]
            dy = world[1] - self.yield_resume_target[1]
            remaining = dx * math.cos(self.yield_heading) + dy * math.sin(
                self.yield_heading)
            if remaining >= -0.06:
                self._change_state(self.yield_resume_state)
                self.yield_start_pose = None
                self.yield_resume_target = None
                self.yield_heading = None
                self.get_logger().debug('กลับถึงแนวทางเดิมแล้ว ทำงานต่อ')
            elif not self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M):
                if self.sensors.any_black():
                    self._follow_line(cmd, RETURN_SPEED)
                elif (not self.sensors.any_yellow() and
                      self.distance_total - self.last_path_color_distance >
                      PATH_COLOR_GRACE_M):
                    self._begin_line_recovery(YIELD_RESUME)
                    self.get_logger().warning(
                        'เดินกลับแล้วไม่เห็นเส้น: กลับหาจุดเส้นล่าสุดก่อน')
                else:
                    # Cross a short yellow marker / camera gap, but never
                    # travel indefinitely without a visible path colour.
                    cmd.linear.x = JUNCTION_RETURN_SPEED
                    cmd.angular.z = max(-MAX_TURN, min(
                        MAX_TURN, YIELD_HEADING_GAIN * angle_difference(
                            self.yield_heading, world[2])))
            self.cmd_pub.publish(cmd)
            return

        # Back away just enough to turn; no rear sensor, so never reverse far.
        if self.state == REVERSING:
            if now - self.state_start_time < REVERSE_SECONDS:
                cmd.linear.x = -REVERSE_SPEED
            else:
                self._start_turn(math.pi, RETURNING, 'UTURN')
            self.cmd_pub.publish(cmd)
            return

        if self.state == TURNING:
            error = (angle_difference(self.turn_target, self.pose[2])
                     if self.pose is not None else math.inf)
            if abs(error) <= TURN_TOLERANCE_RAD:
                if self.turn_kind == 'UTURN':
                    self.returning = True
                    self._new_segment()
                elif self.turn_kind == 'CORNER':
                    pass  # same corridor: keep the direction and trail
                else:
                    self.returning = False
                    self.wait_for_yellow_clear = True
                    self._new_segment()
                self.line_search_after = self.turn_after
                if self.turn_kind == 'BRANCH':
                    self.branch_capture_heading = self.turn_target
                    self.branch_capture_start_distance = self.distance_total
                    self.branch_recovery_anchor = (
                        self.pose[0], self.pose[1], self.turn_target)
                    self._change_state(BRANCH_CAPTURE)
                else:
                    self._change_state(REACQUIRE)
            else:
                # Continue aiming for the odom angle; stale odom is handled above.
                cmd.angular.z = ROTATE_SPEED if error > 0 else -ROTATE_SPEED
            self.cmd_pub.publish(cmd)
            return

        if self.state == CENTERING:
            if self.center_start is None:
                self.center_start = self.pose[:2]
            world = self._world_pose()
            if self.center_on_axle:
                ax, ay = self._axle_point(self.pose)
                heading = self.center_heading
                remaining = ((self.center_target[0] - ax) * math.cos(heading) +
                             (self.center_target[1] - ay) * math.sin(heading))
                if abs(remaining) <= CENTER_TOLERANCE_M:
                    if self.scan_pending:
                        self._start_junction_scan()
                    else:
                        self.center_on_axle = False
                        self._leave_junction_center()
                else:
                    # Passed the hit point: back up the few centimetres.
                    cmd.linear.x = SEARCH_SPEED if remaining > 0 else -SEARCH_SPEED
                    cmd.angular.z = max(-MAX_TURN, min(
                        MAX_TURN, 2.0 * angle_difference(heading, self.pose[2])))
                self.cmd_pub.publish(cmd)
                return
            if (self.team_enabled and self.use_ground_truth
                    and self.active_junction_world is not None):
                dx = self.active_junction_world[0] - world[0]
                dy = self.active_junction_world[1] - world[1]
                remaining = dx * math.cos(world[2]) + dy * math.sin(world[2])
                centered = remaining <= 0.02
            else:
                centered = (math.hypot(self.pose[0] - self.center_start[0],
                                       self.pose[1] - self.center_start[1])
                            >= RGB_FRONT_OFFSET_M)
            if centered:
                if self.junction_needs_request:
                    self._ask_at_junction_centre()
                else:
                    self._leave_junction_center()
            else:
                cmd.linear.x = SEARCH_SPEED
                cmd.angular.z = max(-MAX_TURN, min(
                    MAX_TURN,
                    2.0 * angle_difference(self.center_heading, self.pose[2])))
            self.cmd_pub.publish(cmd)
            return

        if self.state == JUNCTION_SCAN:
            self._scan_junction(cmd)
            self.cmd_pub.publish(cmd)
            return

        if self.sensors.any_green():
            if self.team_enabled:
                world = self._world_pose()
                self._team_report({'type': 'GOAL', 'x': world[0], 'y': world[1]})
            self._trail_event('GOAL')
            self._change_state(GOAL_REACHED)
            self.cmd_pub.publish(cmd)
            return

        # Let followers back up too, before the front ultrasonic would see
        # the yielding robot as an obstacle. This opens space for a queue.
        if (self.team_enabled and not self.returning
                and self.state in (SEARCH, FOLLOW, APPROACH_JUNCTION,
                                   BRANCH_CAPTURE, REACQUIRE,
                                   ALIGN_LINE)):
            peer = self._peer_ahead()
            if (peer is not None and peer[1].get('state') in
                    (YIELDING, YIELD_TURN, YIELD_ESCAPE, YIELD_WAIT, LINE_LOST)
                    and peer[2] < YIELD_CLEARANCE_M
                    and self._peer_has_priority(peer[0], peer[1])):
                self._begin_yield(peer[0])
                self.cmd_pub.publish(cmd)
                return

        # Stop for one control tick before reversing.
        if (self.state not in (REACQUIRE, LINE_LOST)
                and self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M)):
            peer = self._peer_ahead() if self.team_enabled else None
            if peer is not None:
                peer_name, peer_pose, peer_distance = peer
                opposite = (abs(angle_difference(peer_pose['heading'],
                                                 self._world_pose()[2]))
                            > HEAD_ON_HEADING_RAD
                            and peer_pose.get('state') not in PIVOTING_STATES)
                recovering = peer_pose.get('state') in (LINE_LOST, ALIGN_LINE,
                                                       REACQUIRE)
                if ((opposite or recovering) and
                        self._peer_has_priority(peer_name, peer_pose)):
                    self._begin_yield(peer_name)
                    self.cmd_pub.publish(cmd)
                    return
                if not (not self._peer_has_priority(peer_name, peer_pose)
                        and peer_pose.get('state') in
                        (YIELDING, YIELD_TURN, YIELD_ESCAPE, YIELD_WAIT)
                        and peer_distance > 0.40):
                    # Wait for the later robot to back away; do not mark its
                    # occupied corridor as a wall or a blocked branch.
                    self.cmd_pub.publish(cmd)
                    return
            else:
                if self.team_enabled and self.active_team_branch is not None:
                    self._team_report({'type': 'BLOCKED', **self.active_team_branch})
                    self.active_team_branch = None
                self._trail_event('BLOCKED_BRANCH')
                self._change_state(REVERSING)
                self.get_logger().warning('เจอกำแพง: ถอยสั้น หมุนกลับ แล้วตามเส้นย้อน')
                self.cmd_pub.publish(cmd)
                return

        if self.state == BRANCH_CAPTURE:
            # The centered yellow marker separates crossing from branch.
            # Accept black almost immediately after leaving that marker.
            progress = self.distance_total - self.branch_capture_start_distance
            if (progress >= BRANCH_CAPTURE_IGNORE_M
                    and not self.sensors.any_yellow()
                    and self.sensors.any_black()):
                self.get_logger().debug(
                    f'จับกิ่งที่เลือกได้หลัง {progress:.2f} m; '
                    f'pose={self.pose[:2]}, black={self.sensors.black}')
                self.align_last_black_at = now
                self.inner_confirm_ticks = 0
                self.line_search_after = FOLLOW
                self._change_state(ALIGN_LINE)
            elif progress >= BRANCH_CAPTURE_MAX_M:
                self.get_logger().warning(
                    'เลี้ยวแล้วจับเส้นไม่ได้ กลับมาตั้งหลักและให้ทีมลองทางนี้ใหม่')
                self._reject_unseen_branch()
            else:
                node = self.junctions[-1]
                target = (self.active_junction_world
                          if self.team_enabled and self.use_ground_truth
                          and self.active_junction_world is not None
                          else node['position'])
                sensor_x, sensor_y = self._sensor_position()
                dx = target[0] - sensor_x
                dy = target[1] - sensor_y
                heading = self.branch_capture_heading
                lateral_error = -dx * math.sin(heading) + dy * math.cos(heading)
                yaw_error = angle_difference(heading, self.pose[2])
                cmd.linear.x = BRANCH_CAPTURE_SPEED
                cmd.angular.z = max(-0.6, min(0.6,
                                             3.0 * lateral_error +
                                             1.5 * yaw_error))
                self.cmd_pub.publish(cmd)
                return

        if self.state in (REACQUIRE, LINE_LOST):
            if self.sensors.any_black():
                # A black pixel while facing backwards is not yet a recovered
                # route. Reorient to the recorded line direction first.
                if (self.state == LINE_LOST and
                        abs(angle_difference(self.line_lost_anchor[2],
                                             self.pose[2])) > 0.4):
                    self._recover_lost_line(cmd, now)
                    self.cmd_pub.publish(cmd)
                    return
                self.align_last_black_at = now
                self.inner_confirm_ticks = 0
                error = self._line_error()
                if error:
                    self.align_last_side = 1 if error > 0 else -1
                self._change_state(ALIGN_LINE)
                self.get_logger().debug(
                    f'เห็นเส้นดำ เริ่มจัดแนว: black={self.sensors.black}')
            elif self.state == LINE_LOST:
                self._recover_lost_line(cmd, now)
                if now - self.last_search_log_at >= SEARCH_LOG_SECONDS:
                    self.last_search_log_at = now
                    self.get_logger().debug(
                        f'กำลังหาเส้นดำ ({self.line_lost_phase}): '
                        f'black={self.sensors.black}, RGB={self.sensors.rgb}')
                self.cmd_pub.publish(cmd)
                return
            else:
                # Keep rotating until a sensor actually sees black; no fixed timeout.
                cmd.angular.z = self.align_last_side * ROTATE_SPEED
                if now - self.last_search_log_at >= SEARCH_LOG_SECONDS:
                    self.last_search_log_at = now
                    self.get_logger().debug(
                        f'ยังหาเส้นดำอยู่: black={self.sensors.black}, '
                        f'RGB={self.sensors.rgb}')
                self.cmd_pub.publish(cmd)
                return

        if self.state == ALIGN_LINE:
            if self.sensors.any_black():
                self.align_last_black_at = now
                error = self._line_error()
                if error:
                    self.align_last_side = 1 if error > 0 else -1
                inner_pair = self.sensors.black[1] and self.sensors.black[2]
                self.inner_confirm_ticks = (
                    self.inner_confirm_ticks + 1 if inner_pair else 0)
                if self.inner_confirm_ticks >= INNER_CONFIRM_TICKS:
                    self._change_state(self.line_search_after)
                    self.get_logger().debug(
                        f'สองตัวในอยู่บนเส้นแล้ว: black={self.sensors.black}')
                else:
                    # With only an outer sensor on black, pivot the front RGB
                    # array toward the line. Do not drive along its edge.
                    inner_seen = self.sensors.black[1] or self.sensors.black[2]
                    cmd.linear.x = ALIGN_SPEED if inner_seen else 0.0
                    cmd.angular.z = max(-ALIGN_MAX_TURN, min(
                        ALIGN_MAX_TURN, ALIGN_TURN_GAIN * error))
                    if not inner_seen and abs(cmd.angular.z) < ALIGN_MAX_TURN:
                        cmd.angular.z = self.align_last_side * ALIGN_MAX_TURN
                    self.cmd_pub.publish(cmd)
                    return
            elif now - self.align_last_black_at < ALIGN_LOST_GRACE_SECONDS:
                # Brief colour dropout: search toward the last detected side.
                self.inner_confirm_ticks = 0
                cmd.angular.z = self.align_last_side * ALIGN_MAX_TURN
                self.cmd_pub.publish(cmd)
                return
            else:
                self._begin_line_recovery(self.line_search_after)
                self.inner_confirm_ticks = 0
                self.cmd_pub.publish(cmd)
                return

        if self.state == SEARCH:
            if self.sensors.any_black():
                self.get_logger().info('ออกจากจุดเริ่มสีฟ้าและพบเส้นดำแล้ว')
                self._new_segment()
                self._change_state(FOLLOW)
            else:
                if self.search_start_pose is None:
                    self.search_start_pose = self.pose
                distance = math.hypot(
                    self.pose[0] - self.search_start_pose[0],
                    self.pose[1] - self.search_start_pose[1])
                if distance >= SEARCH_MAX_M:
                    self._begin_line_recovery(
                        SEARCH, anchor=self.search_start_pose)
                else:
                    cmd.linear.x = SEARCH_SPEED

        if (self.black_mode and self.outer_hit_position is not None
                and self.state in (FOLLOW, RETURNING)):
            if (self.returning and self.junctions
                    and math.hypot(
                        self.outer_hit_position[0] - self.junctions[-1]['position'][0],
                        self.outer_hit_position[1] - self.junctions[-1]['position'][1])
                    <= RETURN_MARKER_RADIUS_M):
                # The saved junction itself: RETURNING handles it from memory.
                self.outer_hit_position = None
            else:
                self._begin_black_junction()
                self.cmd_pub.publish(Twist())
                return

        if self.state == RETURNING:
            distance = self._distance_to_junction()
            yellow_now = self.sensors.any_yellow()
            at_saved_junction = (distance is not None and
                                 ((yellow_now and distance <= RETURN_MARKER_RADIUS_M)
                                  or self._return_reached_junction()))
            if at_saved_junction and self._at_junction(returning=True):
                cmd = Twist()
            elif distance is None and yellow_now:
                # If the outward pass missed this marker, recover a side branch
                # only from actual outer-sensor evidence at the yellow centre.
                left, right = self.sensors.junction_sides()
                if left or right:
                    self.junction_count += 1
                    self.junctions.append({
                        'id': self.junction_count,
                        'position': self._sensor_position(),
                        'world_position': self._world_sensor_position(),
                        'heading': self.pose[2] + math.pi,
                        'world_heading': self._world_pose()[2] + math.pi,
                        'left_open': right,
                        'right_open': left,
                    })
                    self.get_logger().warning(
                        'ขาไปพลาดการบันทึกแยก; ตรวจพบกิ่งตอนขากลับ')
                    self._at_junction(returning=True)
                    cmd = Twist()
                else:
                    cmd.linear.x = JUNCTION_RETURN_SPEED
            else:
                speed = (JUNCTION_RETURN_SPEED if (yellow_now or
                         (distance is not None and distance < JUNCTION_SLOW_RADIUS_M))
                         else RETURN_SPEED)
                if distance is not None and distance < JUNCTION_SLOW_RADIUS_M and not self.sensors.any_black():
                    # Paint or a short camera dropout can hide the black line
                    # next to a junction. Use this robot's saved junction
                    # position to cross the marker instead of oscillating
                    # between RETURNING and LINE_LOST.
                    sx, sy = self._sensor_position()
                    jx, jy = self.junctions[-1]['position']
                    desired = math.atan2(jy - sy, jx - sx)
                    cmd.linear.x = JUNCTION_RETURN_SPEED
                    cmd.angular.z = max(-MAX_TURN, min(MAX_TURN,
                        1.5 * angle_difference(desired, self.pose[2])))
                elif yellow_now and not self.sensors.any_black():
                    # The warning stripe temporarily covers the inner black
                    # line; keep heading toward J instead of declaring loss.
                    cmd.linear.x = speed
                else:
                    self._follow_line(cmd, speed)

        if self.state == FOLLOW:
            distance = self._distance_to_junction()
            yellow_now = self.sensors.any_yellow()
            if (self.wait_for_yellow_clear and
                    not yellow_now and
                    (distance is None or distance > JUNCTION_ESCAPE_DISTANCE_M)):
                self.wait_for_yellow_clear = False
            if not self.wait_for_yellow_clear and yellow_now:
                self.approach_start_distance = self.distance_total
                self._change_state(APPROACH_JUNCTION)
                self.get_logger().debug(
                    f'เห็นจุดเหลืองตรงแยก: ตรวจตัวนอก; '
                    f'black={self.sensors.black}, yellow={self.sensors.yellow}')
            else:
                ramp = min(1.0, (now - self.state_start_time) /
                           FOLLOW_RAMP_SECONDS)
                cruise_speed = (FOLLOW_RAMP_START_SPEED +
                                (FORWARD_SPEED - FOLLOW_RAMP_START_SPEED) * ramp)
                self._follow_line(
                    cmd, APPROACH_SPEED if yellow_now else cruise_speed)

        if self.state == APPROACH_JUNCTION:
            left, right = self.sensors.junction_sides()
            since_marker = self.distance_total - self.approach_start_distance
            if ((left or right) and self.sensors.any_yellow()
                    and since_marker <= JUNCTION_APPROACH_MAX_M):
                candidate = (left, right)
                if candidate == self.junction_candidate:
                    self.junction_candidate_ticks += 1
                else:
                    self.junction_candidate = candidate
                    self.junction_candidate_ticks = 1
                if self.junction_candidate_ticks < 3:
                    self.cmd_pub.publish(Twist())
                    return
                self.junction_candidate = None
                self.junction_candidate_ticks = 0
                # A side branch seen by an outer sensor at the marker is the
                # junction. Stop on this control tick, before driving past it.
                self.wait_for_yellow_clear = True
                self._at_junction(returning=False, left_open=left, right_open=right)
                cmd = Twist()
            else:
                # The narrow yellow centre may be seen one frame before the
                # outer sensors reach the side line. Creep only to that point.
                self.junction_candidate = None
                self.junction_candidate_ticks = 0
                if (since_marker > JUNCTION_APPROACH_ABORT_M or
                        (not self.sensors.any_black() and
                         not self.sensors.any_yellow() and
                         self.distance_total - self.last_path_color_distance >
                         PATH_COLOR_GRACE_M)):
                    self.get_logger().warning(
                        'ยังยืนยันทางแยกไม่ได้ กลับไปหาเส้นก่อน')
                    self.wait_for_yellow_clear = True
                    if self.sensors.any_black():
                        self._change_state(FOLLOW)
                    else:
                        self._begin_line_recovery(FOLLOW)
                else:
                    cmd.linear.x = APPROACH_SPEED
                    if self.sensors.black[1] or self.sensors.black[2]:
                        cmd.angular.z = self._line_follow_turn()

        if self.state == JUNCTION_PAUSE:
            cmd = Twist()
            if now - self.state_start_time >= JUNCTION_PAUSE_SECONDS:
                if self.team_enabled:
                    if (self.team_choice is not None and
                            self.team_choice.get('choice') is None):
                        if not self.no_route_logged:
                            self.get_logger().warning(
                                'ทีมไม่พบกิ่งที่ยังเปิด: หยุดรอข้อมูลใหม่')
                            self.no_route_logged = True
                        if now - self.last_team_request_at >= 5.0:
                            self.request_sequence += 1
                            self.pending_team_request['request_id'] = (
                                f'{self.run_id}-{self.request_sequence}')
                            self.team_choice = None
                            self._team_report(self.pending_team_request)
                            self.last_team_request_at = now
                        self.cmd_pub.publish(cmd)
                        return
                    if not self._apply_team_choice():
                        self.cmd_pub.publish(cmd)
                        return
                if not self.returning:
                    # The outward pass asked from the centre already.
                    self._leave_junction_center()
                    self.cmd_pub.publish(cmd)
                    return
                # RGB sits 8.5 cm ahead of the wheel centre. The outer sensor
                # stops at the actual junction; move the wheels to its centre
                # before rotating or crossing straight.
                self.center_start = self.pose[:2]
                self.center_heading = self.pose[2]
                if self.black_mode:
                    # Exit headings were measured from the scan pivot.
                    self.center_target = self.junctions[-1]['position']
                    self.center_on_axle = True
                self._change_state(CENTERING)

        if (self.trail_id and (self.sensors.any_black() or self.sensors.any_yellow())
                and now - self.last_trail_update >= TRAIL_UPDATE_SECONDS
                and self.state in (FOLLOW, APPROACH_JUNCTION, RETURNING)):
            self._trail_event('IN_PROGRESS')
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass  # launch may already have invalidated the ROS context
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
