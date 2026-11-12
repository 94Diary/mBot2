"""Shared settings and angle helpers."""

import math

FORWARD_SPEED = 0.55
FOLLOW_RAMP_START_SPEED = 0.22
FOLLOW_RAMP_SECONDS = 0.8
APPROACH_SPEED = 0.20
RETURN_SPEED = 0.50
JUNCTION_RETURN_SPEED = 0.20
SEARCH_SPEED = 0.18
SEARCH_MAX_M = 0.45  # enough to leave the blue start area and reach the line
REVERSE_SPEED = 0.22
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
GOAL_PARK_GAP_M = 0.10  # queue this close behind a teammate inside the goal area
GOAL_PARK_ENTRY_M = 0.15  # drive at least this far in before a missing line means "end"

TURN_GAIN = 0.5
MAX_TURN = 0.5
LINE_CORRECTION_SPEED = 0.18
LINE_CORRECTION_GAIN = 0.65
LINE_CORRECTION_MAX_TURN = 1.2
ROTATE_SPEED = 1.2  # ~7 deg per 10 Hz tick: still lands inside the ±4 deg turn window
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
YIELD_LINE_END_M = 0.03  # black maze: two reversing ticks without black
REACQUIRE_TIMEOUT_SECONDS = 2.0
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
LOOKAHEAD_M = 0.40             # ask the team this far before a known junction
KNOWN_NODE_RADIUS_M = 0.10      # pivot is within ~2 cm of a real node
TEAM_MAP_FRESH_SECONDS = 1.0    # state arrives every 0.5 s
HEAD_ON_DECISION_SECONDS = 2.0  # wait this long for the team's hide order
HEAD_ON_FALLBACK_SECONDS = 5.0  # then use the old rule for this peer
START_LEADER_AHEAD_M = 1.0     # robot in front in the start box
START_LEADER_SIDE_M = 0.15
START_GATE_TIMEOUT_SECONDS = 60.0  # wall clock: the sim runs at ~1/3 speed
CLEAR_AHEAD_M = 0.40            # stop for a teammate this close in front
CLEAR_SIDE_M = 0.25
PEER_WAIT_TURN_BACK_SECONDS = 15.0  # stuck behind a teammate: go elsewhere
HOLD_TIMEOUT_SECONDS = 60.0     # wall clock: the sim runs at ~1/3 speed
HIDE_DEPTH_M = 0.25             # wheels this far into the hiding branch

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
HOLDING = 'HOLDING'  # stopped by the team while a teammate hides
GOAL_PARKING = 'GOAL_PARKING'  # inside the red goal area, driving in to park
# A peer stopped or turning on the spot at a junction points anywhere (a
# scan ends at -130 deg); its heading says nothing about whether it is
# coming back down this corridor.
PIVOTING_STATES = (CENTERING, JUNCTION_SCAN, JUNCTION_PAUSE, TURNING)


def angle_difference(target, current):
    """Shortest signed angle from current to target."""
    return math.atan2(math.sin(target - current), math.cos(target - current))
