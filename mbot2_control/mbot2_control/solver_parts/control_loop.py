"""State machine loop for line following and team behavior."""

import math
import time

from geometry_msgs.msg import Twist

from .config import (
    FORWARD_SPEED,
    FOLLOW_RAMP_START_SPEED,
    FOLLOW_RAMP_SECONDS,
    APPROACH_SPEED,
    RETURN_SPEED,
    JUNCTION_RETURN_SPEED,
    SEARCH_SPEED,
    SEARCH_MAX_M,
    REVERSE_SPEED,
    OBSTACLE_STOP_DISTANCE_M,
    YIELD_CLEARANCE_M,
    HEAD_ON_HEADING_RAD,
    YIELD_HEADING_GAIN,
    GOAL_PARK_GAP_M,
    GOAL_PARK_ENTRY_M,
    MAX_TURN,
    ROTATE_SPEED,
    TURN_TOLERANCE_RAD,
    ALIGN_SPEED,
    ALIGN_TURN_GAIN,
    ALIGN_MAX_TURN,
    ALIGN_LOST_GRACE_SECONDS,
    REACQUIRE_TIMEOUT_SECONDS,
    BRANCH_CAPTURE_SPEED,
    BRANCH_CAPTURE_IGNORE_M,
    BRANCH_CAPTURE_MAX_M,
    PATH_COLOR_GRACE_M,
    YIELD_PATH_GAP_M,
    YIELD_LINE_END_M,
    ANCHOR_TURN_SPEED,
    INNER_CONFIRM_TICKS,
    SIGNAL_STALE_SECONDS,
    SEARCH_LOG_SECONDS,
    RGB_FRONT_OFFSET_M,
    JUNCTION_SLOW_RADIUS_M,
    JUNCTION_ESCAPE_DISTANCE_M,
    JUNCTION_APPROACH_MAX_M,
    JUNCTION_APPROACH_ABORT_M,
    RETURN_MARKER_RADIUS_M,
    TRAIL_UPDATE_SECONDS,
    CENTER_TOLERANCE_M,
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


class ControlLoopMixin:
    def run_control_tick(self, reverse_seconds, junction_pause_seconds):
        now = time.monotonic()
        cmd = Twist()

        if self.team_enabled and self.pose is not None:
            if now - self.last_team_pose_at >= 0.5:
                world = self._world_pose()
                self._team_report({'type': 'POSE', 'x': world[0], 'y': world[1],
                                   'heading': world[2], 'state': self.state,
                                   'route_order': self.team_route_order,
                                   'returning': self.returning,
                                   'blocked_behind': self._peer_behind_close(),
                                   'route_waiting': (self.state == JUNCTION_PAUSE
                                                     and self.no_route_logged),
                                   'waiting_for': (self.clearance_peer['peer']
                                                   if self.clearance_peer is not None
                                                   and self.state in (
                                                       SEARCH, FOLLOW, APPROACH_JUNCTION,
                                                       CENTERING, RETURNING, BRANCH_CAPTURE,
                                                       YIELD_RESUME)
                                                   else None),
                                   'yield_to': (self.yield_peer_name
                                                if self.state == YIELDING else None),
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
                    f'ตำแหน่ง={self.pose is not None and now - self.last_odom_at <= SIGNAL_STALE_SECONDS} '  # noqa: E501
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
                (SEARCH, FOLLOW, APPROACH_JUNCTION,
                 CENTERING, BRANCH_CAPTURE, RETURNING,
                 REACQUIRE, ALIGN_LINE, YIELD_RESUME)):
            # Not while waiting at a junction: after a scan the robot faces
            # wherever the sweep stopped, not the way it will drive.
            head_on = self._head_on_peer()
            if head_on is None:
                self.head_on_wait = None
            elif self.head_on_hiding:
                if self._on_head_on(head_on[0], head_on[1]):
                    self.cmd_pub.publish(cmd)
                    return
            elif self._should_turn_back(head_on[0], head_on[1]):
                self._begin_turn_back(head_on[0])
                self.cmd_pub.publish(cmd)
                return

        if self.state == SEARCH and self._start_gate_closed():
            self.cmd_pub.publish(cmd)
            return

        if (self.team_enabled and self.state in
                (SEARCH, FOLLOW, APPROACH_JUNCTION, CENTERING, RETURNING, BRANCH_CAPTURE,
                 YIELD_RESUME)):
            blocker = self._clearance_peer()
            if blocker is not None:
                peer = self.team_peers[blocker]
                waiting_for_route = (peer.get('state') == JUNCTION_PAUSE
                                     and peer.get('route_waiting', False))
                if self.clearance_peer is None or self.clearance_peer['peer'] != blocker:
                    self.get_logger().info(f'รอ {blocker} ที่อยู่ข้างหน้าออกไปก่อน')
                    self.clearance_peer = {'peer': blocker, 'since': now}
                elif peer.get('state') in PIVOTING_STATES and not waiting_for_route:
                    self.clearance_peer['since'] = now  # busy at a junction, not stuck
                elif (self.state != SEARCH and not self.head_on_hiding
                      and not self.returning
                      and now - self.clearance_peer['since'] > PEER_WAIT_TURN_BACK_SECONDS
                      and not self._peer_behind_close()):
                    # It is not moving on: leave this corridor to it.
                    self.clearance_peer = None
                    self._begin_turn_back(blocker)
                self.cmd_pub.publish(cmd)
                return
            self.clearance_peer = None

        if self.state == YIELD_TURN:
            error = angle_difference(self.turn_target, self.pose[2])
            if abs(error) <= TURN_TOLERANCE_RAD:
                self._change_state(YIELD_ESCAPE)
            else:
                cmd.angular.z = ROTATE_SPEED if error > 0 else -ROTATE_SPEED
            self.cmd_pub.publish(cmd)
            return

        if self.state == YIELD_WAIT:
            if self.escape_cleared:
                if self._hide_depth() > CENTER_TOLERANCE_M:
                    # Straight back out of the branch to the junction centre.
                    cmd.linear.x = -SEARCH_SPEED
                    cmd.angular.z = max(-MAX_TURN, min(MAX_TURN, 2.0 * angle_difference(
                        self.escape['hide_heading'], self._world_pose()[2])))
                else:
                    self._resume_after_hiding()
                    cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        if self.state == HOLDING:
            if now - self.state_start_time >= HOLD_TIMEOUT_SECONDS:
                self.get_logger().warning('รอนานเกินไปแล้วไม่ได้รับ GO: ไปต่อเอง')
                self.team_encounter = None
                self._change_state(self.hold_resume_state)
            self.cmd_pub.publish(cmd)
            return

        if self.state == YIELDING:
            if self._yield_clear():
                self.wait_for_yellow_clear = False
                # Count line-free travel from here: reversing off the line
                # end must not use up the grace for driving back onto it.
                self.last_path_color_distance = self.distance_total
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
                  # Black maze: no paint gaps on the line, so stop almost at once.
                  (YIELD_LINE_END_M if self.black_mode else YIELD_PATH_GAP_M)):
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
            if now - self.state_start_time < reverse_seconds:
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
                elif self.turn_kind in ('CORNER', 'ROUTE'):
                    pass  # same corridor or a planned route: keep the trail
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
                    elif self.await_ahead_choice:
                        # Asked ahead but no answer yet: wait for that answer.
                        self.center_on_axle = False
                        self.await_ahead_choice = False
                        self.returning = False
                        self._change_state(JUNCTION_PAUSE)
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

        if self.state not in (GOAL_PARKING, GOAL_REACHED) and self.sensors.any_green():
            if not self.goal_reported:
                self.goal_reported = True
                if self.team_enabled:
                    world = self._world_pose()
                    self._team_report({'type': 'GOAL', 'x': world[0], 'y': world[1],
                                       'approach_heading': world[2]})
                self._trail_event('GOAL')
            self.get_logger().info('เห็นพื้นที่เป้าหมาย: เดินตามเส้นเข้าไปจอด')
            self.park_start_distance = self.distance_total
            self.park_heading = self.pose[2]
            self._change_state(GOAL_PARKING)

        if self.state == GOAL_PARKING:
            # Drive in along the line; stop at its end or close behind a
            # teammate already parked. Nothing else matters in here.
            driven = self.distance_total - self.park_start_distance
            if self.ultrasonic.obstacle_ahead(GOAL_PARK_GAP_M):
                if self.sensors.any_green():
                    self.get_logger().info('จอดในพื้นที่เป้าหมายแล้ว')
                    self._change_state(GOAL_REACHED)
            elif self.sensors.any_black():
                cmd.linear.x = APPROACH_SPEED
                cmd.angular.z = self._line_follow_turn()
            elif driven < GOAL_PARK_ENTRY_M and self.sensors.any_green():
                # Came in beside the line (e.g. mid-turn): creep straight in.
                cmd.linear.x = APPROACH_SPEED
                cmd.angular.z = max(-MAX_TURN, min(MAX_TURN, angle_difference(
                    self.park_heading, self.pose[2])))
            elif self.sensors.any_green():
                self.get_logger().info('จอดในพื้นที่เป้าหมายแล้ว')
                self._change_state(GOAL_REACHED)
            else:
                # Neither line nor red: find the line again, then park.
                self._begin_line_recovery(FOLLOW)
            self.cmd_pub.publish(cmd)
            return

        # Let followers back up too, before the front ultrasonic would see
        # the yielding robot as an obstacle. This opens space for a queue.
        if (self.team_enabled and self.head_on_hiding and not self.returning
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

        if not self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M):
            self.peer_wait = None  # nobody in front any more
        # Stop for one control tick before reversing.
        # Standing or pivoting at a junction cannot hit anything ahead, and
        # the ultrasonic may be pointing down another line.
        if (self.state not in (REACQUIRE, LINE_LOST, JUNCTION_PAUSE, TURNING,
                               HOLDING, YIELD_TURN, YIELD_WAIT, YIELD_ESCAPE)
                and self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M)):
            peer = self._peer_ahead() if self.team_enabled else None
            if peer is not None:
                peer_name, peer_pose, peer_distance = peer
                opposite = (abs(angle_difference(peer_pose['heading'],
                                                 self._world_pose()[2]))
                            > HEAD_ON_HEADING_RAD
                            and peer_pose.get('state') not in
                            PIVOTING_STATES + (GOAL_REACHED, GOAL_PARKING))
                # Only a lost peer may back into us while it searches; one
                # squaring up after a turn is driving away, so just wait.
                recovering = peer_pose.get('state') == LINE_LOST
                if opposite:
                    if self.head_on_hiding:
                        if self._on_head_on(peer_name, peer_pose):
                            self.cmd_pub.publish(cmd)
                            return
                    elif self._should_turn_back(peer_name, peer_pose):
                        self._begin_turn_back(peer_name)
                        self.cmd_pub.publish(cmd)
                        return
                if (recovering and self.head_on_hiding
                        and self._peer_has_priority(peer_name, peer_pose)):
                    self._begin_yield(peer_name)
                    self.cmd_pub.publish(cmd)
                    return
                if not (not self._peer_has_priority(peer_name, peer_pose)
                        and peer_pose.get('state') in
                        (YIELDING, YIELD_TURN, YIELD_ESCAPE, YIELD_WAIT)
                        and peer_distance > 0.40):
                    # Wait for the later robot to back away; do not mark its
                    # occupied corridor as a wall or a blocked branch.
                    if peer_pose.get('state') in (GOAL_REACHED, GOAL_PARKING):
                        # Queue for the goal: close up behind the parked
                        # teammate; at the parking gap this robot is parked.
                        if not self.ultrasonic.obstacle_ahead(GOAL_PARK_GAP_M):
                            if self.sensors.any_black():
                                self._follow_line(cmd, APPROACH_SPEED)
                        elif peer_pose.get('state') == GOAL_REACHED:
                            self.get_logger().info('จอดต่อท้ายเพื่อนในแถวเป้าหมายแล้ว')
                            self._change_state(GOAL_REACHED)
                            cmd = Twist()
                    elif not self.head_on_hiding:
                        if self.peer_wait is None or self.peer_wait['peer'] != peer_name:
                            self.peer_wait = {'peer': peer_name, 'since': now}
                        elif now - self.peer_wait['since'] > PEER_WAIT_TURN_BACK_SECONDS:
                            # It is not moving on: leave this corridor to it.
                            self.peer_wait = None
                            self._begin_turn_back(peer_name)
                    self.cmd_pub.publish(cmd)
                    return
            else:
                if self.team_enabled and self.active_team_branch is not None:
                    self._team_report({'type': 'BLOCKED', **self.active_team_branch})
                    self.active_team_branch = None
                world = self._world_pose()
                self._report_node('DEAD_END', world[:2], world[2])
                self._trail_event('BLOCKED_BRANCH')
                if self.team_enabled and not self.head_on_hiding and self._peer_behind_close():
                    # A teammate right behind: turn on the spot, never back into it.
                    self._start_turn(math.pi, RETURNING, 'UTURN')
                else:
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
                cmd.angular.z = max(-0.6, min(
                    0.6, 3.0 * lateral_error + 1.5 * yaw_error))
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
                # If the expected exit is not under the sensors, recover
                # from the last known line pose instead of spinning here.
                if now - self.state_start_time >= REACQUIRE_TIMEOUT_SECONDS:
                    self._begin_line_recovery(self.line_search_after)
                    self.cmd_pub.publish(cmd)
                    return
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
                world = self._world_pose()
                self._report_node('START', self._world_sensor_position(),
                                  world[2], heading=world[2])
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
                and self.state in (FOLLOW, RETURNING, YIELD_ESCAPE)):
            if (self.state == RETURNING and self.junctions
                    and math.hypot(
                        self.outer_hit_position[0] - self.junctions[-1]['position'][0],
                        self.outer_hit_position[1] - self.junctions[-1]['position'][1])
                    <= RETURN_MARKER_RADIUS_M):
                # The saved junction itself: RETURNING handles it from memory.
                self.outer_hit_position = None
            elif self.state == FOLLOW and self._arrive_with_ahead_request():
                pass  # passed straight through: keep following this tick
            else:
                self._begin_black_junction()
                self.cmd_pub.publish(Twist())
                return

        if self.state == YIELD_ESCAPE:
            if self.escape is not None and self.escape['hiding']:
                if self._hide_depth() >= HIDE_DEPTH_M:
                    self._team_report({'type': 'HIDDEN'})
                    self.get_logger().info('หลบเข้ากิ่งข้างแล้ว: รอเพื่อนผ่าน')
                    self._change_state(YIELD_WAIT)
                    self.cmd_pub.publish(Twist())
                    return
            if self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M):
                self.cmd_pub.publish(Twist())  # a robot, never a wall: wait
                return
            self._follow_line(cmd, RETURN_SPEED)

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
                if (distance is not None and distance < JUNCTION_SLOW_RADIUS_M
                        and not self.sensors.any_black()):
                    # Paint or a short camera dropout can hide the black line
                    # next to a junction. Use this robot's saved junction
                    # position to cross the marker instead of oscillating
                    # between RETURNING and LINE_LOST.
                    sx, sy = self._sensor_position()
                    jx, jy = self.junctions[-1]['position']
                    desired = math.atan2(jy - sy, jx - sx)
                    cmd.linear.x = JUNCTION_RETURN_SPEED
                    cmd.angular.z = max(-MAX_TURN, min(
                        MAX_TURN, 1.5 * angle_difference(desired, self.pose[2])))
                elif yellow_now and not self.sensors.any_black():
                    # The warning stripe temporarily covers the inner black
                    # line; keep heading toward J instead of declaring loss.
                    cmd.linear.x = speed
                else:
                    self._follow_line(cmd, speed)

        if self.state == FOLLOW and self.black_mode and self.team_enabled:
            self._ask_ahead()

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
            # Leave as soon as the team has answered; the pause is only the
            # longest wait before re-checking an unanswered request.
            answered = self.team_enabled and self.team_choice is not None
            if answered or now - self.state_start_time >= junction_pause_seconds:
                if self.team_enabled:
                    if (self.team_choice is not None and
                            self.team_choice.get('choice') is None):
                        if not self.no_route_logged:
                            message = ('ไม่มีกิ่งสำรวจที่ว่างและไปถึงได้: หยุดรอข้อมูลใหม่'
                                       if self.team_choice.get('reason') == 'WAIT_FOR_FRONTIER'
                                       else 'ยังไม่มีทางยืนยันถึงเป้าหมาย: หยุดรอข้อมูลใหม่')
                            self.get_logger().warning(message)
                            self.no_route_logged = True
                        if (self._team_choice_is_stale()
                                or now - self.last_team_request_at >= 5.0):
                            self._refresh_team_request()
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
