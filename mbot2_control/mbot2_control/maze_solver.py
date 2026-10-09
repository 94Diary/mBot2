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

# Keep the forward speeds already chosen for this project.
FORWARD_SPEED = 0.5
APPROACH_SPEED = 0.10
RETURN_SPEED = 0.12
JUNCTION_RETURN_SPEED = 0.06
SEARCH_SPEED = 0.07
REVERSE_SPEED = 0.15
REVERSE_SECONDS = 1.5
OBSTACLE_STOP_DISTANCE_M = 0.25
YIELD_REVERSE_M = 0.35
YIELD_CLEARANCE_M = 0.65
YIELD_REAR_CLEARANCE_M = 0.45
TEAM_GOAL_RADIUS_M = 0.55
TEAM_ASSEMBLY_RADIUS_M = 1.45

TURN_GAIN = 0.15
MAX_TURN = 0.5
LINE_CORRECTION_SPEED = 0.12
LINE_CORRECTION_GAIN = 0.45
LINE_CORRECTION_MAX_TURN = 0.8
ROTATE_SPEED = 0.55
TURN_TOLERANCE_RAD = 0.07
ALIGN_SPEED = 0.06              # เจอดำแค่ตัวนอก: คืบเข้าหาเส้น ไม่เร่งเต็มทันที
ALIGN_TURN_GAIN = 0.3
ALIGN_MAX_TURN = 0.45
ALIGN_LOST_GRACE_SECONDS = 0.6
BRANCH_CAPTURE_SPEED = 0.12
BRANCH_CAPTURE_IGNORE_M = 0.65
BRANCH_CAPTURE_MAX_M = 1.60
BRANCH_SWEEP_SPEED = 0.07
BRANCH_SWEEP_PERIOD_M = 1.20
BRANCH_SWEEP_ANGLE_RAD = 0.75
INNER_CONFIRM_TICKS = 2        # สองตัวในต้องเห็นดำติดต่อกันก่อนวิ่งปกติ
SIGNAL_STALE_SECONDS = 2.0     # ข้อมูลหายจริงจึงพักคำสั่งขับ และกลับมาทำต่อเมื่อข้อมูลมา
SEARCH_LOG_SECONDS = 10.0
JUNCTION_PAUSE_SECONDS = 1.0
JUNCTION_CONFIRM_TICKS = 2
JUNCTION_MIN_AFTER_YELLOW_M = 0.25

# Front RGB sensor is ~8.5 cm ahead of the base centre in the robot description.
RGB_FRONT_OFFSET_M = 0.085
JUNCTION_SLOW_RADIUS_M = 0.55
JUNCTION_LATERAL_TOLERANCE_M = 0.15
JUNCTION_ESCAPE_DISTANCE_M = 0.55  # leave all 0.4 m warning stripes first
RETURN_YELLOW_SIDE_MIN_M = 0.25  # avoid a false side detection at stripe entrance
RETURN_YELLOW_TO_JUNCTION_M = 0.38  # warning starts ~0.4 m before junction
TRAIL_UPDATE_SECONDS = 1.0

SEARCH = 'SEARCH'
FOLLOW = 'FOLLOW'
APPROACH_JUNCTION = 'APPROACH_JUNCTION'
JUNCTION_PAUSE = 'JUNCTION_PAUSE'
CENTERING = 'CENTERING'
BRANCH_CAPTURE = 'BRANCH_CAPTURE'
BRANCH_SWEEP = 'BRANCH_SWEEP'
TURNING = 'TURNING'
REACQUIRE = 'REACQUIRE'
ALIGN_LINE = 'ALIGN_LINE'
RETURNING = 'RETURNING'
LINE_LOST = 'LINE_LOST'
REVERSING = 'REVERSING'
YIELDING = 'YIELDING'
YIELD_WAIT = 'YIELD_WAIT'
GOAL_REACHED = 'GOAL_REACHED'


def angle_difference(target, current):
    """Shortest signed angle from current to target."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')
        self.declare_parameter('junction_choice', 'straight_first')
        self.declare_parameter('team_enabled', False)
        self.declare_parameter('use_ground_truth', False)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
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
        self.last_team_pose_at = 0.0
        self.last_team_request_at = 0.0
        self.last_team_state_at = 0.0
        self.team_waiting_logged = False
        self.no_route_logged = False
        self.yield_start_pose = None
        self.yield_resume_state = None
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
                'ใช้ straight_first')
            self.junction_choice = 'straight_first'
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.trail_pub = self.create_publisher(String, 'trail', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        if self.use_ground_truth:
            self.create_subscription(
                TFMessage, '/world/simple_branch_maze/dynamic_pose/info',
                self._on_ground_truth, 10)
        self.timer = self.create_timer(0.1, self.tick)

        self.sensors = QuadRGBArray(self)
        self.ultrasonic = UltrasonicSensor(self)
        self.encoder = WheelEncoder(self)

        self.state = SEARCH
        self.state_start_time = time.monotonic()
        self.pose = None  # x, y, yaw in this robot's local odom frame
        self.last_odom_at = 0.0
        self.distance_total = 0.0
        self.junctions = []  # local breadcrumb stack; NOT a shared global map
        self.returning = False
        self.junction_candidate = None
        self.junction_candidate_ticks = 0
        self.approach_start_distance = 0.0
        self.wait_for_yellow_clear = False
        self.return_yellow_armed = False
        self.return_yellow_start_distance = 0.0
        self.junction_count = 0
        self.branch_angle = 0.0
        self.center_start = None
        self.turn_target = None
        self.turn_after = None
        self.turn_kind = None
        self.branch_capture_start_distance = 0.0
        self.branch_capture_heading = None
        self.branch_sweep_start_distance = 0.0
        self.line_search_after = FOLLOW
        self.last_search_log_at = 0.0
        self.align_last_black_at = 0.0
        self.align_last_side = 1
        self.inner_confirm_ticks = 0
        self.trail_id = 0
        self.trail_start_distance = 0.0
        self.last_trail_update = 0.0
        self.waiting_logged = False
        self.get_logger().info(
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
            self.get_logger().info(
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
        for peer in self.team_peers.values():
            dx, dy = peer['x'] - pose[0], peer['y'] - pose[1]
            forward = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
            side = abs(-dx * math.sin(pose[2]) + dy * math.cos(pose[2]))
            if 0.0 < forward < YIELD_CLEARANCE_M and side < 0.42:
                return False
        return True

    def _request_team_branch(self, returning, left_open, right_open,
                             position_override=None):
        world = self._world_pose()
        position = position_override or self._world_sensor_position()
        if world is None or position is None:
            return
        self.request_sequence += 1
        choices = []
        if not returning:
            choices.append({'name': 'straight', 'heading': world[2]})
        if left_open:
            choices.append({'name': 'left', 'heading': world[2] + math.pi / 2})
        if right_open:
            choices.append({'name': 'right', 'heading': world[2] - math.pi / 2})
        self.pending_team_request = {
            'type': 'BRANCH_REQUEST',
            'request_id': f'{self.run_id}-{self.request_sequence}',
            'x': position[0], 'y': position[1],
            'choices': choices,
        }
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
        }
        self.active_junction_world = (
            float(choice['junction_x']), float(choice['junction_y']))
        self.branch_angle = (
            math.pi / 2 if branch == 'left' else
            -math.pi / 2 if branch == 'right' else 0.0)
        self.pending_team_request = None
        self.team_choice = None
        self.no_route_logged = False
        return True

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
        self._trail_event('IN_PROGRESS')

    def _change_state(self, state):
        if self.state != state:
            self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.state_start_time = time.monotonic()

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
            self.line_search_after = self.state
            self._change_state(LINE_LOST)
            return
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

    def _line_error(self):
        weights = [2, 1, -1, -2]  # leftmost to rightmost
        active = [weight for weight, black in zip(weights, self.sensors.black)
                  if black]
        return sum(active) / len(active) if active else 0.0

    def _line_follow_turn(self):
        turn = TURN_GAIN * self._line_error()
        return max(-MAX_TURN, min(MAX_TURN, turn))

    def _at_junction(self, returning, left_open=False, right_open=False):
        if not returning:
            self.junction_count += 1
            self.junctions.append({
                'id': self.junction_count,
                'position': self._sensor_position(),
                'world_position': self._world_sensor_position(),
                'heading': self.pose[2],
                'left_open': left_open,
                'right_open': right_open,
            })
            self._trail_event('JUNCTION')
            if self.junction_choice == 'side_first':
                self.branch_angle = math.pi / 2 if left_open else -math.pi / 2
                self.get_logger().info(
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
                return False
            self.get_logger().info(f'ขากลับเลือกเลี้ยว{chosen}')
        if self.team_enabled:
            if returning:
                # Left/right swap when the robot approaches from the other side.
                self._request_team_branch(
                    True, self.junctions[-1]['right_open'],
                    self.junctions[-1]['left_open'],
                    self.junctions[-1].get('world_position'))
            else:
                self._request_team_branch(False, left_open, right_open)
        self._change_state(JUNCTION_PAUSE)
        self.get_logger().info(
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
                                   'heading': world[2], 'state': self.state})
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
                        self.get_logger().info(
                            f'รวมพลหลังเพื่อนใกล้เป้าหมาย ระยะ {goal_distance:.2f} m')
                        self._trail_event('TEAM_ASSEMBLED_NEAR_GOAL')
                    else:
                        self.get_logger().info('ถึงจุดรวมพลใกล้เป้าหมายของทีม')
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
                    'ข้อมูล RGB/Ultrasonic/odom หายหรือยังไม่มา: พักการขับจนสัญญาณกลับ')
                self.waiting_logged = True
            self.cmd_pub.publish(cmd)
            return
        if self.waiting_logged:
            self.get_logger().info('ข้อมูลเซนเซอร์กลับมาแล้ว: ทำงานต่อ')
            self.waiting_logged = False
        if self.team_enabled and now - self.last_team_state_at > 3.0:
            if not self.team_waiting_logged:
                self.get_logger().warning('ตัวกลางทีมไม่ตอบ: หยุดรอการสื่อสาร')
                self.team_waiting_logged = True
            self.cmd_pub.publish(cmd)
            return
        if self.team_waiting_logged:
            self.get_logger().info('การสื่อสารกับทีมกลับมาแล้ว')
            self.team_waiting_logged = False

        if self.state == YIELDING:
            world = self._world_pose()
            backed = math.hypot(world[0] - self.yield_start_pose[0],
                                world[1] - self.yield_start_pose[1])
            if backed >= YIELD_REVERSE_M:
                self._change_state(YIELD_WAIT)
                self.get_logger().info('ถอยหลีกทางแล้ว รอหุ่นขากลับพ้นทางแยก')
            elif self._peer_behind_close():
                self.get_logger().warning('มีหุ่นด้านหลัง หยุดก่อนถอยหลีกทาง')
            else:
                cmd.linear.x = -REVERSE_SPEED
            self.cmd_pub.publish(cmd)
            return

        if self.state == YIELD_WAIT:
            if self._yield_clear():
                self._change_state(self.yield_resume_state)
                self.yield_start_pose = None
                self.yield_resume_state = None
                self.get_logger().info('หุ่นอีกตัวพ้นทางแล้ว กลับมาทำงานต่อ')
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
                    self.return_yellow_armed = False
                else:
                    self.returning = False
                    self.wait_for_yellow_clear = True
                    self.return_yellow_armed = False
                self._new_segment()
                self.line_search_after = self.turn_after
                if self.turn_kind == 'BRANCH':
                    self.branch_capture_heading = self.turn_target
                    self.branch_capture_start_distance = self.distance_total
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
                self._start_turn(self.branch_angle, FOLLOW, 'BRANCH')
            else:
                cmd.linear.x = SEARCH_SPEED
                cmd.angular.z = self._line_follow_turn()
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

        # Stop for one control tick before reversing.
        if (self.state not in (REACQUIRE, LINE_LOST)
                and self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M)):
            peer = self._peer_ahead() if self.team_enabled else None
            if peer is not None:
                peer_name, peer_pose, peer_distance = peer
                peer_returning = peer_pose.get('state') in (
                    RETURNING, REVERSING, REACQUIRE, ALIGN_LINE)
                own_priority = self.state == RETURNING
                if peer_returning and not own_priority:
                    self.yield_start_pose = self._world_pose()[:2]
                    self.yield_resume_state = self.state
                    self._change_state(YIELDING)
                    self.get_logger().info(
                        f'หลีกทางให้ {peer_name} ที่กำลังย้อนทาง')
                    self.cmd_pub.publish(cmd)
                    return
                if not (own_priority and peer_pose.get('state') in
                        (YIELDING, YIELD_WAIT) and peer_distance > 0.40):
                    # Another robot is in front: wait without marking the
                    # branch BLOCKED. A returning robot has right of way.
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
            # At a crossing the old straight line is still underneath the
            # cameras. Move away in the selected branch direction before
            # accepting black as the new line.
            progress = self.distance_total - self.branch_capture_start_distance
            if progress >= BRANCH_CAPTURE_IGNORE_M and self.sensors.any_black():
                self.get_logger().info(
                    f'จับกิ่งที่เลือกได้หลัง {progress:.2f} m; '
                    f'pose={self.pose[:2]}, black={self.sensors.black}')
                self.align_last_black_at = now
                self.inner_confirm_ticks = 0
                self.line_search_after = FOLLOW
                self._change_state(ALIGN_LINE)
            elif progress >= BRANCH_CAPTURE_MAX_M:
                self.get_logger().warning(
                    f'ยังไม่เห็นเส้นบนกิ่งที่เลือกหลัง {progress:.2f} m; '
                    f'pose={self.pose[:2]}: เริ่มกวาดซ้าย-ขวาบนกิ่ง')
                self.line_search_after = FOLLOW
                self.branch_sweep_start_distance = self.distance_total
                self._change_state(BRANCH_SWEEP)
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

        if self.state == BRANCH_SWEEP:
            if self.sensors.any_black():
                self.align_last_black_at = now
                self.inner_confirm_ticks = 0
                self.line_search_after = FOLLOW
                self._change_state(ALIGN_LINE)
            else:
                progress = self.distance_total - self.branch_sweep_start_distance
                desired = (self.branch_capture_heading +
                           BRANCH_SWEEP_ANGLE_RAD * math.sin(
                               2 * math.pi * progress / BRANCH_SWEEP_PERIOD_M))
                cmd.linear.x = BRANCH_SWEEP_SPEED
                cmd.angular.z = max(-0.7, min(0.7,
                    1.5 * angle_difference(desired, self.pose[2])))
                self.cmd_pub.publish(cmd)
                return

        if self.state in (REACQUIRE, LINE_LOST):
            if self.sensors.any_black():
                # One black sensor is useful evidence: stop sweeping and align.
                self.align_last_black_at = now
                self.inner_confirm_ticks = 0
                error = self._line_error()
                if error:
                    self.align_last_side = 1 if error > 0 else -1
                self._change_state(ALIGN_LINE)
                self.get_logger().info(
                    f'เห็นเส้นดำ เริ่มจัดแนว: black={self.sensors.black}')
            else:
                # Keep rotating until a sensor actually sees black; no fixed timeout.
                cmd.angular.z = -ROTATE_SPEED
                if now - self.last_search_log_at >= SEARCH_LOG_SECONDS:
                    self.last_search_log_at = now
                    self.get_logger().info(
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
                    self.get_logger().info(
                        f'สองตัวในอยู่บนเส้นแล้ว: black={self.sensors.black}')
                else:
                    # Edge sensor first: turn toward it while creeping forward.
                    cmd.linear.x = ALIGN_SPEED
                    cmd.angular.z = max(
                        -ALIGN_MAX_TURN,
                        min(ALIGN_MAX_TURN, ALIGN_TURN_GAIN * error))
                    self.cmd_pub.publish(cmd)
                    return
            elif now - self.align_last_black_at < ALIGN_LOST_GRACE_SECONDS:
                # Brief colour dropout: search toward the last detected side.
                self.inner_confirm_ticks = 0
                cmd.angular.z = self.align_last_side * ALIGN_MAX_TURN
                self.cmd_pub.publish(cmd)
                return
            else:
                self._change_state(REACQUIRE)
                self.inner_confirm_ticks = 0
                self.cmd_pub.publish(cmd)
                return

        if self.state == SEARCH:
            if self.sensors.any_black():
                self._new_segment()
                self._change_state(FOLLOW)
            else:
                cmd.linear.x = SEARCH_SPEED

        if self.state == RETURNING:
            distance = self._distance_to_junction()
            yellow_now = self.sensors.any_yellow() and self.sensors.any_black()
            if yellow_now and not self.return_yellow_armed:
                self.return_yellow_armed = True
                self.return_yellow_start_distance = self.distance_total
                self.get_logger().info(
                    'ขากลับเห็นแถบเหลือง: ชะลอเพื่อเลือกกิ่งที่เคยบันทึก')
            if distance is None:
                # Recovery if the short branch pattern was missed on the first pass:
                # keep returning SLOWLY and read the live side at the junction.
                self._follow_line(cmd, JUNCTION_RETURN_SPEED)
                if self.state == RETURNING:
                    left, right = self.sensors.junction_sides()
                    if left != right:
                        self.junction_count += 1
                        self.junctions.append({
                            'id': self.junction_count,
                            'position': self._sensor_position(),
                            'world_position': self._world_sensor_position(),
                            'heading': self.pose[2] + math.pi,
                            'left_open': right,
                            'right_open': left,
                        })
                        self.get_logger().warning(
                            'ขาไปพลาดการบันทึกแยก; ตรวจพบกิ่งตอนขากลับ')
                        self._at_junction(returning=True)
                        cmd = Twist()
            else:
                speed = (JUNCTION_RETURN_SPEED if (self.return_yellow_armed
                                                   or distance < JUNCTION_SLOW_RADIUS_M)
                         else RETURN_SPEED)
                self._follow_line(cmd, speed)
                if self.state == RETURNING:
                    left, right = self.sensors.junction_sides()
                    yellow_progress = self.distance_total - self.return_yellow_start_distance
                    at_warning_junction = (
                        self.return_yellow_armed
                        and ((yellow_progress >= RETURN_YELLOW_SIDE_MIN_M
                              and left != right)
                             or yellow_progress >= RETURN_YELLOW_TO_JUNCTION_M))
                    if (self._return_reached_junction() or at_warning_junction) and (
                            self._at_junction(returning=True)):
                        self.return_yellow_armed = False
                        cmd = Twist()

        if self.state == FOLLOW:
            distance = self._distance_to_junction()
            yellow_now = self.sensors.any_yellow()
            if (self.wait_for_yellow_clear and
                    not yellow_now and
                    (distance is None or distance > JUNCTION_ESCAPE_DISTANCE_M)):
                self.wait_for_yellow_clear = False
            if (not self.wait_for_yellow_clear and yellow_now
                    and self.sensors.any_black()):
                self.approach_start_distance = self.distance_total
                self.junction_candidate = None
                self.junction_candidate_ticks = 0
                self._change_state(APPROACH_JUNCTION)
                self.get_logger().info(
                    f'เห็นแถบเหลือง: ชะลอ; '
                    f'black={self.sensors.black}, yellow={self.sensors.yellow}')
            else:
                self._follow_line(
                    cmd, APPROACH_SPEED if yellow_now else FORWARD_SPEED)

        if self.state == APPROACH_JUNCTION:
            self._follow_line(cmd, APPROACH_SPEED)
            if self.state == APPROACH_JUNCTION:
                left, right = self.sensors.junction_sides()
                past_warning = (self.distance_total - self.approach_start_distance
                                >= JUNCTION_MIN_AFTER_YELLOW_M)
                candidate = ((left, right) if past_warning and left != right
                             else None)
                if candidate is None:
                    self.junction_candidate = None
                    self.junction_candidate_ticks = 0
                elif candidate == self.junction_candidate:
                    self.junction_candidate_ticks += 1
                else:
                    self.junction_candidate = candidate
                    self.junction_candidate_ticks = 1
                if self.junction_candidate_ticks >= JUNCTION_CONFIRM_TICKS:
                    self.wait_for_yellow_clear = True
                    self._at_junction(returning=False, left_open=left, right_open=right)
                    cmd = Twist()

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
                if (self.branch_angle != 0.0 if self.team_enabled else
                        self.returning or self.junction_choice == 'side_first'):
                    self.center_start = self.pose[:2]
                    self._change_state(CENTERING)
                else:
                    # First visit: try straight; a wall will trigger backtracking.
                    self._new_segment()
                    self._change_state(FOLLOW)

        if (self.trail_id and now - self.last_trail_update >= TRAIL_UPDATE_SECONDS
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
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
