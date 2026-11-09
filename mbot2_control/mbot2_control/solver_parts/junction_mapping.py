"""Methods moved from maze_solver.py; behavior is unchanged."""

import math
import time
from geometry_msgs.msg import Twist
from .config import (
    ROTATE_SPEED,
    RGB_FRONT_OFFSET_M,
    JUNCTION_ESCAPE_DISTANCE_M,
    SCAN_LIMIT_RAD,
    SCAN_STRAIGHT_MAX_RAD,
    SCAN_SIDE_MIN_RAD,
    LOOKAHEAD_M,
    KNOWN_NODE_RADIUS_M,
    TEAM_MAP_FRESH_SECONDS,
    FOLLOW,
    JUNCTION_PAUSE,
    JUNCTION_SCAN,
    CENTERING,
    RETURNING,
    YIELD_ESCAPE,
    angle_difference,
)


class JunctionMappingMixin:
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
        hiding = self.escape is not None and self.escape['hiding']
        if (self.pose is None or hiding
                or self.state not in (FOLLOW, RETURNING, YIELD_ESCAPE)
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
        self.scan_from_line_end = False
        self.center_heading = self.pose[2]
        self.scan_returning = self.returning
        self.junction_entry_pose = self.last_centered_pose or self.pose
        self.scan_pending = True
        self.center_on_axle = True
        self.get_logger().debug(
            f'ตัวนอกเห็นเส้น: หยุดแล้วเข้ากลางจุดเพื่อหมุนสแกน; '
            f'black={self.sensors.black}')
        self._change_state(CENTERING)

    def _report_node(self, kind, position, approach_heading, **extra):
        """Tell the team map about a start, corner or dead end."""
        self._team_report({'type': 'NODE', 'kind': kind,
                           'x': position[0], 'y': position[1],
                           'approach_heading': approach_heading, **extra})

    def _ask_ahead(self):
        """Ask about a mapped junction before reaching it, once per junction."""
        if self.ahead_request is not None:
            node = self.ahead_request['node']
            sx, sy = self._world_sensor_position()
            yaw = self._world_pose()[2]
            dx, dy = node['x'] - sx, node['y'] - sy
            along = dx * math.cos(yaw) + dy * math.sin(yaw)
            lateral = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
            if along < -0.20 or lateral > 0.10:
                self._drop_ahead_request()  # behind us, or no longer on our line
            return
        if (self.pending_team_request is not None or self.returning or
                time.monotonic() - self.last_team_state_at > TEAM_MAP_FRESH_SECONDS):
            return
        sx, sy = self._world_sensor_position()
        yaw = self._world_pose()[2]
        for node in self.team_nodes:
            if node.get('kind') != 'JUNCTION':
                continue
            dx, dy = node['x'] - sx, node['y'] - sy
            along = dx * math.cos(yaw) + dy * math.sin(yaw)
            lateral = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
            if 0.05 <= along <= LOOKAHEAD_M and lateral <= 0.05:
                self.scan_hits = {'straight': [], 'left': [], 'right': []}
                for key, branch in node['branches'].items():
                    heading = branch.get('heading', int(key) * math.pi / 2)
                    self._bin_scan_angle(angle_difference(heading, yaw))
                exits = {name: yaw + sum(a) / len(a)
                         for name, a in self.scan_hits.items() if a}
                self._request_team_branch(
                    False, 'left' in exits, 'right' in exits, (node['x'], node['y']),
                    straight_open='straight' in exits, approach_heading=yaw,
                    exit_headings=exits)
                self.ahead_request = {'node': node, 'exits': exits,
                                      'request_id': self.pending_team_request['request_id']}
                return

    def _drop_ahead_request(self):
        if self.ahead_request is None:
            return
        if (self.pending_team_request is not None and self.pending_team_request[
                'request_id'] == self.ahead_request['request_id']):
            self._team_report({'type': 'CANCEL_BRANCH',
                               'request_id': self.pending_team_request['request_id']})
            self.pending_team_request = None
            self.team_choice = None
        self.ahead_request = None
        self.await_ahead_choice = False

    def _arrive_with_ahead_request(self):
        """At the junction asked about: pass straight, or centre to turn/wait."""
        ahead = self.ahead_request
        if ahead is None:
            return False
        node = ahead['node']
        hx, hy = self.outer_hit_world
        if math.hypot(node['x'] - hx, node['y'] - hy) > 0.15:
            # Something else came first (e.g. an unmapped corner): withdraw.
            self._drop_ahead_request()
            return False
        exits = ahead['exits']
        world = self._world_pose()
        local = self._remember_junction(
            self._sensor_position(), (node['x'], node['y']), self.pose[2], world[2],
            'left' in exits, 'right' in exits)
        local['straight_open'] = 'straight' in exits
        local['exit_headings'] = exits
        self.active_junction_world = (node['x'], node['y'])
        self.junction_entry_pose = self.last_centered_pose or self.pose
        self.ahead_request = None
        self.trigger_resume_distance = self.distance_total + JUNCTION_ESCAPE_DISTANCE_M
        center = self.outer_hit_position
        if self.team_choice is not None and self._apply_team_choice():
            if abs(self.branch_angle) < 0.35:
                self.get_logger().info(f"ผ่าน J{node['id']} ตรงไปโดยไม่หยุด")
                self.outer_hit_position = None
                return True
            self.await_ahead_choice = False
        else:
            self.await_ahead_choice = True
        # Turn (or wait for the answer) with the wheels on the junction.
        self.center_target = center
        self.center_heading = self.pose[2]
        self.scan_pending = False
        self.center_on_axle = True
        self._change_state(CENTERING)
        self.cmd_pub.publish(Twist())
        return True

    def _begin_line_end_check(self):
        # Black maze: the line vanished, so pivot where it was last centred.
        # Both sides are swept: nothing found is a dead end (BLOCKED, U-turn),
        # a single exit is a drift or a bend the outer sensors missed.
        self._drop_ahead_request()
        x, y, yaw = self.last_centered_pose
        self.center_target = (x + RGB_FRONT_OFFSET_M * math.cos(yaw),
                              y + RGB_FRONT_OFFSET_M * math.sin(yaw))
        self.scan_hit_world = self._world_sensor_position()
        self.scan_sides = {'left', 'right'}
        self.scan_from_line_end = True  # not at a node: never trust the map
        self.center_heading = yaw
        self.scan_returning = self.returning
        self.junction_entry_pose = self.last_centered_pose
        self.scan_pending = True
        self.center_on_axle = True
        self.get_logger().info('เส้นหาย: กลับไปจุดสุดท้ายที่เห็นเส้นแล้วหมุนสแกน')
        self._change_state(CENTERING)

    def _start_junction_scan(self):
        self.scan_pending = False
        self.center_on_axle = False
        self.scan_pivot = self._axle_point(self.pose)
        self.scan_pivot_world = self._axle_point(self._world_pose())
        self.scan_heading = self.pose[2]
        self.scan_hits = {'straight': [], 'left': [], 'right': []}
        # Wheels on the centre: the inner sensors already sit over the
        # straight arm, so only the sides an outer sensor touched are swept.
        if self.sensors.black[1] or self.sensors.black[2]:
            self.scan_hits['straight'].append(0.0)
        self.scan_phases = [side for side in ('left', 'right')
                            if side in self.scan_sides]
        if self.escape is not None and not self.escape['hiding']:
            node = self._known_node(self.scan_pivot_world)
            if node is not None and self._follow_route_step(node):
                return
            px, py = self.scan_pivot_world
            near = [(n['id'], n['kind'], round(n['x'], 2), round(n['y'], 2))
                    for n in self.team_nodes
                    if math.hypot(n['x'] - px, n['y'] - py) < 0.4]
            self.get_logger().warning(
                'ทางหลบไม่ตรงกับแผนที่: เลิกหลบ ใช้การสแกนปกติ; '
                f'pivot=({px:.2f},{py:.2f}) matched={node and node["id"]} '
                f'next={self.route_steps[:1]} near={near}')
            self.escape = None
            self.route_steps = []
        known = (None if self.scan_from_line_end
                 else self._known_node(self.scan_pivot_world))
        if known is not None:
            # The team already mapped this place: use its exits, no sweep.
            world_h0 = self._world_pose()[2]
            self.scan_hits = {'straight': [], 'left': [], 'right': []}
            for key, branch in known['branches'].items():
                heading = branch.get('heading', int(key) * math.pi / 2)
                self._bin_scan_angle(angle_difference(heading, world_h0))
            self.get_logger().info(f"ใช้ข้อมูลทีมที่ J{known['id']}: ไม่ต้องหมุนสแกน")
            self._finish_junction_scan()
            return
        self._change_state(JUNCTION_SCAN)

    def _scan_junction(self, cmd):
        """Sweep to ±130° on each touched side, noting inner-sensor black."""
        theta = angle_difference(self.pose[2], self.scan_heading)
        if self.sensors.black[1] or self.sensors.black[2]:
            self._bin_scan_angle(theta)
        while self.scan_phases and (
                (self.scan_phases[0] == 'left' and theta >= SCAN_LIMIT_RAD) or
                (self.scan_phases[0] == 'right' and theta <= -SCAN_LIMIT_RAD)):
            self.scan_phases.pop(0)
        if not self.scan_phases:
            self._finish_junction_scan()
        else:
            cmd.angular.z = (ROTATE_SPEED if self.scan_phases[0] == 'left'
                             else -ROTATE_SPEED)

    def _bin_scan_angle(self, theta):
        """File an exit angle; the way in (about 180 deg) fits no window."""
        if abs(theta) <= SCAN_STRAIGHT_MAX_RAD:
            self.scan_hits['straight'].append(theta)
        elif SCAN_SIDE_MIN_RAD <= theta <= SCAN_LIMIT_RAD:
            self.scan_hits['left'].append(theta)
        elif -SCAN_LIMIT_RAD <= theta <= -SCAN_SIDE_MIN_RAD:
            self.scan_hits['right'].append(theta)

    def _known_node(self, world_xy):
        """Nearest mapped junction or corner, while the team map is fresh."""
        if (not self.team_enabled or world_xy is None or
                time.monotonic() - self.last_team_state_at > TEAM_MAP_FRESH_SECONDS):
            return None
        best = None
        for node in self.team_nodes:
            if node.get('kind') not in ('JUNCTION', 'CORNER'):
                continue
            distance = math.hypot(node['x'] - world_xy[0], node['y'] - world_xy[1])
            if distance <= KNOWN_NODE_RADIUS_M and (best is None or distance < best[0]):
                best = (distance, node)
        return best[1] if best else None

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
            self._report_node('DEAD_END', self.scan_pivot_world, world_h0)
            self._trail_event('BLOCKED_BRANCH')
            self.get_logger().warning('สแกนแล้วไม่มีทางไปต่อ: กลับรถ')
            self._start_turn(angle_difference(h0 + math.pi, self.pose[2]),
                             RETURNING, 'UTURN')
            return
        if len(exits) == 1:
            # A corner, or a wobble that touched the line edge: no choice here.
            (name, theta), = exits.items()
            self.get_logger().info(f'ทางเดียว ({labels[name]}): ไปต่อโดยไม่ถามทีม')
            if name != 'straight':  # straight only: a wobble, not a place
                self._report_node('CORNER', self.scan_pivot_world, world_h0,
                                  exit_heading=world_h0 + theta)
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
