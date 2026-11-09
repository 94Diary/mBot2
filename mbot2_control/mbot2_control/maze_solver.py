"""Experiment 1: follow a line, backtrack from walls, and leave digital trails."""

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

# Balanced speeds for 10 Hz control and 15 Hz line sensors.
from .solver_parts.config import (  # noqa: F401 - public constants for callers
    FORWARD_SPEED,
    FOLLOW_RAMP_START_SPEED,
    FOLLOW_RAMP_SECONDS,
    APPROACH_SPEED,
    RETURN_SPEED,
    JUNCTION_RETURN_SPEED,
    SEARCH_SPEED,
    SEARCH_MAX_M,
    REVERSE_SPEED,
    REVERSE_SECONDS,
    OBSTACLE_STOP_DISTANCE_M,
    YIELD_REVERSE_M,
    YIELD_CLEARANCE_M,
    YIELD_REAR_CLEARANCE_M,
    YIELD_JUNCTION_CLEARANCE_M,
    YIELD_BRANCH_CLEARANCE_M,
    HEAD_ON_DETECT_M,
    HEAD_ON_HEADING_RAD,
    YIELD_LINE_TURN_GAIN,
    YIELD_HEADING_GAIN,
    GOAL_PARK_GAP_M,
    GOAL_PARK_ENTRY_M,
    TURN_GAIN,
    MAX_TURN,
    LINE_CORRECTION_SPEED,
    LINE_CORRECTION_GAIN,
    LINE_CORRECTION_MAX_TURN,
    ROTATE_SPEED,
    TURN_TOLERANCE_RAD,
    ALIGN_SPEED,
    ALIGN_TURN_GAIN,
    ALIGN_MAX_TURN,
    ALIGN_LOST_GRACE_SECONDS,
    BRANCH_CAPTURE_SPEED,
    BRANCH_CAPTURE_IGNORE_M,
    BRANCH_CAPTURE_MAX_M,
    PATH_COLOR_GRACE_M,
    YIELD_PATH_GAP_M,
    YIELD_LINE_END_M,
    ANCHOR_ARRIVAL_M,
    ANCHOR_TRAVEL_SPEED,
    ANCHOR_TURN_SPEED,
    ANCHOR_SWEEP_RAD,
    INNER_CONFIRM_TICKS,
    SIGNAL_STALE_SECONDS,
    SEARCH_LOG_SECONDS,
    JUNCTION_PAUSE_SECONDS,
    RGB_FRONT_OFFSET_M,
    WHEEL_X_OFFSET_M,
    JUNCTION_SLOW_RADIUS_M,
    JUNCTION_LATERAL_TOLERANCE_M,
    JUNCTION_ESCAPE_DISTANCE_M,
    JUNCTION_APPROACH_MAX_M,
    JUNCTION_APPROACH_ABORT_M,
    RETURN_MARKER_RADIUS_M,
    TRAIL_UPDATE_SECONDS,
    SCAN_LIMIT_RAD,
    SCAN_STRAIGHT_MAX_RAD,
    SCAN_SIDE_MIN_RAD,
    CENTER_TOLERANCE_M,
    LOOKAHEAD_M,
    KNOWN_NODE_RADIUS_M,
    TEAM_MAP_FRESH_SECONDS,
    HEAD_ON_DECISION_SECONDS,
    HEAD_ON_FALLBACK_SECONDS,
    START_LEADER_AHEAD_M,
    START_LEADER_SIDE_M,
    START_GATE_TIMEOUT_SECONDS,
    CLEAR_AHEAD_M,
    CLEAR_SIDE_M,
    PEER_WAIT_TURN_BACK_SECONDS,
    HOLD_TIMEOUT_SECONDS,
    HIDE_DEPTH_M,
    SEARCH,
    FOLLOW,
    APPROACH_JUNCTION,
    JUNCTION_PAUSE,
    JUNCTION_SCAN,
    CENTERING,
    BRANCH_CAPTURE,
    TURNING,
    REACQUIRE,
    ALIGN_LINE,
    RETURNING,
    LINE_LOST,
    REVERSING,
    YIELDING,
    YIELD_RESUME,
    YIELD_TURN,
    YIELD_ESCAPE,
    YIELD_WAIT,
    GOAL_REACHED,
    HOLDING,
    GOAL_PARKING,
    PIVOTING_STATES,
    angle_difference,
)


from .solver_parts.team_motion import TeamMotionMixin
from .solver_parts.line_navigation import LineNavigationMixin
from .solver_parts.junction_mapping import JunctionMappingMixin


from .solver_parts.control_loop import ControlLoopMixin


class MazeSolver(TeamMotionMixin, LineNavigationMixin, JunctionMappingMixin,
                 ControlLoopMixin, Node):

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
        # True: C4 (team arbitrates, loser hides in a side branch).
        # False: the robot without priority turns round and drives back.
        self.declare_parameter('head_on_hiding', False)
        self.team_enabled = bool(self.get_parameter('team_enabled').value)
        self.use_ground_truth = bool(self.get_parameter('use_ground_truth').value)
        self.head_on_hiding = bool(self.get_parameter('head_on_hiding').value)
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
        self.team_nodes = []  # shared map from /maze_team/state
        self.head_on_wait = None  # {'peer', 'since'} while the team decides
        self.head_on_fallback_until = {}
        self.peer_wait = None  # {'peer', 'since'} while stopped behind a teammate
        self.start_leader = None  # robot in front in the start box
        self.start_gate_open = False
        self.start_gate_since = 0.0
        self.clearance_peer = None  # {'peer', 'since'} while stopped for a teammate
        self.goal_reported = False
        self.ahead_request = None  # {'node', 'request_id'}: asked before arriving
        self.await_ahead_choice = False
        self.park_start_distance = 0.0
        self.park_heading = 0.0
        self.team_encounter = None
        self.hold_resume_state = FOLLOW
        self.route_steps = []         # [{'node', 'exit_dir', 'heading'}, ...]
        self.route_after = FOLLOW     # state to drive in between route steps
        self.escape = None            # the YIELD order being carried out
        self.escape_cleared = False
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
        self.scan_from_line_end = False
        self.scan_hit_world = None
        self.trigger_resume_distance = 0.0
        self.scan_pending = False
        self.center_on_axle = False  # black mode: pivot where the scan pivoted
        self.scan_pivot = None
        self.scan_pivot_world = None
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

    def tick(self):
        """Run one control step with the current public timing settings."""
        self.run_control_tick(REVERSE_SECONDS, JUNCTION_PAUSE_SECONDS)


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
