"""Methods moved from maze_solver.py; behavior is unchanged."""

import json
import math
import time
from std_msgs.msg import String
from .config import (
    RETURN_MARKER_RADIUS_M,
    APPROACH_SPEED,
    REVERSE_SPEED,
    OBSTACLE_STOP_DISTANCE_M,
    YIELD_LINE_TURN_GAIN,
    YIELD_HEADING_GAIN,
    TURN_GAIN,
    MAX_TURN,
    LINE_CORRECTION_SPEED,
    LINE_CORRECTION_GAIN,
    LINE_CORRECTION_MAX_TURN,
    ALIGN_MAX_TURN,
    ANCHOR_ARRIVAL_M,
    ANCHOR_TRAVEL_SPEED,
    ANCHOR_TURN_SPEED,
    RGB_FRONT_OFFSET_M,
    WHEEL_X_OFFSET_M,
    JUNCTION_LATERAL_TOLERANCE_M,
    FOLLOW,
    JUNCTION_PAUSE,
    BRANCH_CAPTURE,
    TURNING,
    ALIGN_LINE,
    TURN_TOLERANCE_RAD,
    RETURNING,
    LINE_LOST,
    YIELDING,
    YIELD_RESUME,
    GOAL_REACHED,
    angle_difference,
)


class LineNavigationMixin:
    def _request_team_branch(self, returning, left_open, right_open,
                             position_override=None, straight_open=True,
                             approach_heading=None, exit_headings=None,
                             offer_back=True):
        world = self._world_pose()
        position = position_override or self._world_sensor_position()
        if world is None or position is None:
            return
        if approach_heading is not None:
            # A black-mode scan ends rotated; headings come from its start.
            world = (world[0], world[1], approach_heading)
        exit_headings = exit_headings or {}
        self.request_sequence += 1
        self.no_route_logged = False
        choices = []
        if straight_open and (not returning or 'straight' in exit_headings):
            choices.append({'name': 'straight',
                            'heading': exit_headings.get('straight', world[2])})
        if left_open:
            choices.append({'name': 'left', 'heading': exit_headings.get(
                'left', world[2] + math.pi / 2)})
        if right_open:
            choices.append({'name': 'right', 'heading': exit_headings.get(
                'right', world[2] - math.pi / 2)})
        if offer_back:
            choices.append({'name': 'back', 'heading': world[2] + math.pi})
        if returning and exit_headings:
            # A fresh scan is already relative to the current approach.
            for option in choices:
                option['fallback'] = option['name'] == 'back'
        elif returning and self.junctions:
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
        source = ('จากสแกนขากลับ' if returning and exit_headings else
                  'จากความจำกิ่งเดิม' if returning else
                  'กิ่งข้างจาก RGB; หน้าตรงยังต้องสำรวจ')
        self.get_logger().info(
            f"แยก ({'ขากลับ' if returning else 'ขาไป'}; ทิศเทียบกับหน้ารถตอนนี้): "
            f"ซ้ายเปิด={'left' in available} หน้าเปิด={'straight' in available} "
            f"ขวาเปิด={'right' in available} "
            f'[{source}]')
        self.team_choice = None
        self._team_report(self.pending_team_request)
        self.last_team_request_at = time.monotonic()

    def _team_choice_is_stale(self):
        choice = self.team_choice
        if choice is None or self.team_routing_revision is None:
            return False
        revision = choice.get('routing_revision')
        return revision is not None and revision < self.team_routing_revision

    def _refresh_team_request(self):
        """Ask again at the same junction with any newly mapped exits."""
        previous = self.pending_team_request
        choice = self.team_choice
        choices = list(previous['choices'])
        node = next((n for n in self.team_nodes if n['id'] == choice['junction_id']), None)
        if node is not None:
            approach = previous['approach_heading']
            for key, branch in node['branches'].items():
                if branch.get('leads_to') is None:
                    continue
                heading = branch.get('heading', int(key) * math.pi / 2)
                if any(abs(angle_difference(heading, c['heading'])) < math.pi / 4
                       for c in choices):
                    continue
                turn = angle_difference(heading, approach)
                name = ('straight' if abs(turn) < math.pi / 4 else
                        'back' if abs(turn) > 3 * math.pi / 4 else
                        'left' if turn > 0 else 'right')
                choices.append({'name': name, 'heading': heading})
        if choice.get('choice') is not None:
            self._team_report({'type': 'CANCEL_BRANCH', 'request_id': previous['request_id']})
        self.request_sequence += 1
        self.pending_team_request = {**previous, 'choices': choices,
                                     'request_id': f'{self.run_id}-{self.request_sequence}'}
        self.team_choice = None
        self._team_report(self.pending_team_request)
        self.last_team_request_at = time.monotonic()

    def _apply_team_choice(self):
        choice = self.team_choice
        if choice is None or choice.get('choice') is None:
            return False
        if self._team_choice_is_stale():
            self.get_logger().info('แผนที่เปลี่ยน: ขอคำนวณเส้นทางใหม่ที่แยก')
            self._refresh_team_request()
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
        self.branch_target_yaw = None
        if 'heading' in choice:
            self.branch_angle = angle_difference(float(choice['heading']), self._world_pose()[2])
            self.branch_target_yaw = self.pose[2] + self.branch_angle
        label = {'left': 'ซ้าย', 'right': 'ขวา', 'straight': 'ตรง', 'back': 'กลับทางเดิม'}[branch]
        labels = {'left': 'ซ้าย', 'right': 'ขวา', 'straight': 'หน้า', 'back': 'หลัง'}
        details = []
        for item in choice.get('branches', []):
            if item['status'] == 'BLOCKED':
                status = 'ทางตัน'
            elif item.get('endpoint_known'):
                status = 'รู้ปลายทางแล้ว'
            elif item['visited_by']:
                status = 'เคยเลือก/ผ่าน แต่ยังไม่รู้ปลายทาง'
            else:
                status = 'ยังไม่สำรวจ'
            if item.get('failed_attempts'):
                status += f"/จับเส้นพลาด{item['failed_attempts']}ครั้ง"
            details.append(f"{labels[item['name']]}={status}")
        if details:
            self.get_logger().info(f"J{choice['junction_id']} ทีมจำไว้: " + ' | '.join(details))
        reason = {'UNEXPLORED_BRANCH': 'สำรวจกิ่งที่ยังไม่รู้ปลายทาง',
                  'EXPLORE_GOAL_CONNECTION': 'สำรวจทางเชื่อม: ยังไม่มีเส้นทางยืนยันถึงเป้าหมาย',
                  'ROUTE_TO_GOAL': 'ทางที่สั้นที่สุดไปเป้าหมายตามแผนที่',
                  'LIKELY_KNOWN_BRANCH': 'กิ่งที่น่าจะพาไปที่ที่ทีมรู้แล้ว',
                  'TOWARD_UNEXPLORED': 'ไปหาแยกที่ยังมีทางไม่ได้สำรวจที่ใกล้ที่สุด',
                  'ASSIGNED_BRANCH': 'สำรวจกิ่งที่ทีมจองให้',
                  'TOWARD_ASSIGNED': 'ไปหางานที่ทีมจองให้',
                  'TAKE_QUEUED_TASK': 'รับงานต่อจากหุ่นที่รออยู่ด้านหลัง',
                  }.get(choice.get('reason'), 'ทีมเลือกทาง')
        exit_cardinal = choice.get('exit_cardinal')
        compass = ''
        if exit_cardinal is not None:
            approach = choice.get('approach_cardinal') or '?'
            next_id = choice.get('next_node_id')
            next_label = f'J{next_id}' if next_id is not None else 'ยังไม่รู้ปลายกิ่ง'
            compass = f' [ทิศโลก เข้า {approach} → ออก {exit_cardinal}, ถัดไป {next_label}]'
        self.get_logger().info(f'เลือก{label}ที่ J{choice["junction_id"]}: {reason}{compass}')
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
        target = self.branch_target_yaw
        self.branch_target_yaw = None
        if target is not None:
            self.branch_angle = angle_difference(target, self.pose[2])
            if abs(self.branch_angle) <= TURN_TOLERANCE_RAD:
                self.branch_angle = 0.0
        if self.branch_angle == 0.0:
            # Cross the marker without treating its yellow centre as
            # a lost line; the new straight segment begins ahead.
            self.returning = False
            self.wait_for_yellow_clear = True
            self._new_segment()
            self.branch_capture_heading = self.pose[2] if target is None else target
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
        heading = self.pose[2]
        dx, dy = sx - jx, sy - jy
        along = dx * math.cos(heading) + dy * math.sin(heading)
        sideways = -dx * math.sin(heading) + dy * math.cos(heading)
        return (-0.02 <= along <= RETURN_MARKER_RADIUS_M
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
        self.branch_target_yaw = None
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
            if (self.black_mode and self.state == FOLLOW
                    and self.last_centered_pose is not None):
                self._begin_line_end_check()
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
        self._drop_ahead_request()
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
            cmd.angular.z = max(-MAX_TURN, min(
                MAX_TURN, 1.5 * heading_error))
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
