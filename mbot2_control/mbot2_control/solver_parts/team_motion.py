"""Methods moved from maze_solver.py; behavior is unchanged."""

import json
import math
import time
from std_msgs.msg import String
from .config import (
    YIELD_CLEARANCE_M,
    YIELD_REAR_CLEARANCE_M,
    YIELD_BRANCH_CLEARANCE_M,
    HEAD_ON_DETECT_M,
    HEAD_ON_HEADING_RAD,
    RGB_FRONT_OFFSET_M,
    JUNCTION_ESCAPE_DISTANCE_M,
    HEAD_ON_DECISION_SECONDS,
    HEAD_ON_FALLBACK_SECONDS,
    START_LEADER_AHEAD_M,
    START_LEADER_SIDE_M,
    START_GATE_TIMEOUT_SECONDS,
    CLEAR_AHEAD_M,
    CLEAR_SIDE_M,
    FOLLOW,
    APPROACH_JUNCTION,
    JUNCTION_PAUSE,
    JUNCTION_SCAN,
    CENTERING,
    BRANCH_CAPTURE,
    TURNING,
    ALIGN_LINE,
    RETURNING,
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


class TeamMotionMixin:
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
        if decision.get('type') is not None:
            if decision.get('robot') == self.robot_id:
                self._on_team_order(decision)
            return
        pending = self.pending_team_request
        if (pending is not None and decision.get('robot') == self.robot_id
                and decision.get('request_id') == pending['request_id']):
            self.team_choice = decision
            self.get_logger().debug(
                f"ทีมจัดทางแยก J{decision.get('junction_id')}: "
                f"{decision.get('choice')} (ร่วมทาง={decision.get('shared', False)})")

    def _on_team_order(self, order):
        """Head-on orders from the team: YIELD, HOLD, GO or CLEAR."""
        kind = order['type']
        self.head_on_wait = None
        if kind == 'YIELD':
            self._begin_escape(order)
        elif kind == 'HOLD':
            self.team_encounter = order
            self.hold_resume_state = RETURNING if self.returning else FOLLOW
            self.get_logger().info('ทีมสั่งให้รอ: เพื่อนกำลังหลบเข้ากิ่งข้าง')
            self._change_state(HOLDING)
        elif kind == 'GO' and self.state == HOLDING:
            self.get_logger().info('ทีมให้ไปต่อได้')
            self.team_encounter = None
            self._change_state(self.hold_resume_state)
        elif kind == 'CLEAR':
            self.escape_cleared = True
        elif kind == 'GOAL_ROUTE':
            if (order.get('turn_back') and not self.returning and self.state in
                    (FOLLOW, APPROACH_JUNCTION, BRANCH_CAPTURE, ALIGN_LINE)):
                self.get_logger().info('ทีมเจอเป้าหมายแล้ว และทางกลับใกล้กว่า: กลับตัวไปเป้าหมาย')
                self._begin_turn_back(None)

    def _on_head_on(self, name, peer):
        """Ask the team who hides; True while this robot must stay put."""
        now = time.monotonic()
        if self.head_on_fallback_until.get(name, 0.0) <= now:
            if self.head_on_wait is None or self.head_on_wait['peer'] != name:
                self.head_on_wait = {'peer': name, 'since': now}
                self._team_report({'type': 'HEAD_ON', 'peer': name})
            if now - self.head_on_wait['since'] < HEAD_ON_DECISION_SECONDS:
                return True
            self.get_logger().warning('ทีมไม่ตอบเรื่องหุ่นสวนกัน: ใช้กฎลำดับเดิม')
            self.head_on_wait = None
            self.head_on_fallback_until[name] = now + HEAD_ON_FALLBACK_SECONDS
        if self._peer_has_priority(name, peer):
            self._begin_yield(name)
            return True
        return False

    def _should_turn_back(self, name, peer):
        """Decide which of two robots meeting head-on turns round (both agree)."""
        mine = self._peer_behind_close()
        theirs = bool(peer.get('blocked_behind', False))
        if mine != theirs:
            return theirs  # the one with a teammate on its tail cannot turn
        mine, theirs = self.returning, bool(peer.get('returning', False))
        if mine != theirs:
            return theirs  # one is leaving a dead end: the other gives way
        return self._peer_has_priority(name, peer)

    def _begin_turn_back(self, peer):
        """Meeting a teammate head-on: it already explores this corridor."""
        if self._peer_behind_close():
            return  # turning would swing into the teammate behind; wait
        if (self.state == CENTERING and not self.junction_needs_request
                and not self.scan_pending and not self.await_ahead_choice
                and self.active_team_branch is not None):
            # The branch was chosen at this junction but not entered yet.
            self._team_report({'type': 'CANCEL_BRANCH',
                               'request_id': self.active_team_branch['request_id']})
            self.team_route_order = self.active_team_branch['previous_route_order']
            self.team_route_order_local = self.team_route_order is not None
            self.active_team_branch = None
        elif self.active_team_branch is not None:
            self._team_report({'type': 'RELEASE_BRANCH', **self.active_team_branch,
                               'peer': peer})
            self.active_team_branch = None
        if self.pending_team_request is not None:
            self._team_report({'type': 'CANCEL_BRANCH',
                               'request_id': self.pending_team_request['request_id']})
            self.pending_team_request = None
            self.team_choice = None
        self.ahead_request = None
        self.await_ahead_choice = False
        self.junction_needs_request = False
        self.scan_pending = False
        self.center_on_axle = False
        if peer is not None:
            self.get_logger().info(f'กลับตัวให้ {peer}: ทางนี้มีเพื่อนสำรวจอยู่แล้ว')
        if self.returning:
            # Heading back already: turn round and explore on, asking anew.
            self.returning = False
            self._start_turn(math.pi, FOLLOW, 'CORNER')
        else:
            self._start_turn(math.pi, RETURNING, 'UTURN')

    def _begin_escape(self, order):
        """Turn round and drive the team's route to a branch to hide in."""
        self.team_encounter = order
        self.escape = {'hide_node': order['hide_node'], 'hide_dir': order['hide_dir'],
                       'hide_heading': float(order['hide_heading']),
                       'hiding': False, 'hide_center': None}
        self.escape_cleared = False
        self.route_steps = list(order['route'])
        self.route_after = YIELD_ESCAPE
        self.turn_target = self.pose[2] + math.pi
        self.get_logger().info(
            f"ทีมให้หลบ: กลับหัวแล้วไปหลบที่ J{order['hide_node']}")
        self._change_state(YIELD_TURN)

    def _follow_route_step(self, node):
        """At the next node of the route, turn onto its exit; False otherwise."""
        if not self.route_steps or self.route_steps[0]['node'] != node['id']:
            return False
        step = self.route_steps.pop(0)
        world_yaw = self._world_pose()[2]
        self.trigger_resume_distance = self.distance_total + JUNCTION_ESCAPE_DISTANCE_M
        if not self.route_steps and self.escape is not None:
            self.escape['hiding'] = True
            self.escape['hide_center'] = self._axle_point(self._world_pose())
        self._start_turn(angle_difference(float(step['heading']), world_yaw),
                         self.route_after, 'ROUTE')
        return True

    def _hide_depth(self):
        """How far the axle is along the hiding branch from its junction."""
        ax, ay = self._axle_point(self._world_pose())
        hx, hy = self.escape['hide_center']
        heading = self.escape['hide_heading']
        return (ax - hx) * math.cos(heading) + (ay - hy) * math.sin(heading)

    def _resume_after_hiding(self):
        """Back at the hiding junction's centre: ask the team as usual."""
        escape = self.escape
        hx, hy = escape['hide_center']
        node = next((n for n in self.team_nodes if n['id'] == escape['hide_node']), None)
        approach = escape['hide_heading'] + math.pi
        self.scan_hits = {'straight': [], 'left': [], 'right': []}
        for key, branch in (node or {'branches': {}})['branches'].items():
            if int(key) == escape['hide_dir']:
                continue
            heading = branch.get('heading', int(key) * math.pi / 2)
            self._bin_scan_angle(angle_difference(heading, approach))
        exits = {name: approach + sum(angles) / len(angles)
                 for name, angles in self.scan_hits.items() if angles}
        local_approach = self.pose[2] + angle_difference(approach, self._world_pose()[2])
        index = next((i for i in range(len(self.junctions) - 1, -1, -1)
                      if math.hypot(self.junctions[i]['world_position'][0] - hx,
                                    self.junctions[i]['world_position'][1] - hy) <= 0.25),
                     None)
        if index is None:
            self.junction_count += 1
            self.junctions.append({'id': self.junction_count})
        else:
            self.junctions = self.junctions[:index + 1]
        self.junctions[-1].update({
            'position': self._axle_point(self.pose), 'world_position': (hx, hy),
            'heading': local_approach, 'world_heading': approach,
            'left_open': 'left' in exits, 'right_open': 'right' in exits,
            'straight_open': 'straight' in exits, 'exit_headings': exits})
        self.active_junction_world = (hx, hy)
        self.returning = False
        self.escape = None
        self.team_encounter = None
        self.escape_cleared = False
        self.route_steps = []
        self.get_logger().info('เพื่อนผ่านแล้ว: ออกจากที่หลบและขอทางจากทีม')
        self._request_team_branch(
            False, 'left' in exits, 'right' in exits, (hx, hy),
            straight_open='straight' in exits, approach_heading=approach,
            exit_headings=exits, offer_back=False)  # back = the hiding branch
        self._change_state(JUNCTION_PAUSE)

    def _on_team_state(self, msg):
        try:
            data = json.loads(msg.data)
            self.team_goal = data.get('goal')
            self.team_nodes = data.get('junctions', [])
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

    @staticmethod
    def _in_front(pose, x, y, ahead, side):
        dx, dy = x - pose[0], y - pose[1]
        forward = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
        lateral = abs(-dx * math.sin(pose[2]) + dy * math.cos(pose[2]))
        return 0.0 < forward < ahead and lateral < side

    def _start_gate_closed(self):
        """Leave the start box only once the robot in front reached a junction."""
        if not self.team_enabled or self.start_gate_open:
            return False
        pose = self._world_pose()
        if pose is None:
            return True
        if self.start_leader is None:
            ahead = [(math.hypot(p['x'] - pose[0], p['y'] - pose[1]), name)
                     for name, p in self.team_peers.items()
                     if self._in_front(pose, p['x'], p['y'],
                                       START_LEADER_AHEAD_M, START_LEADER_SIDE_M)]
            if not ahead:
                self.start_gate_open = True
                return False
            self.start_leader = min(ahead)[1]
            self.start_gate_since = time.monotonic()
            self.get_logger().info(
                f'รอ {self.start_leader} ถึงแยกก่อน แล้วค่อยออกจากจุดเริ่ม')
        leader = self.team_peers.get(self.start_leader)
        if (leader is None or leader.get('junction_reached')
                or leader.get('state') in (GOAL_PARKING, GOAL_REACHED)
                or time.monotonic() - self.start_gate_since > START_GATE_TIMEOUT_SECONDS):
            self.start_gate_open = True
            self.get_logger().info(f'{self.start_leader} ถึงแยกแล้ว: ออกจากจุดเริ่ม')
            return False
        return True

    def _clearance_peer(self):
        """Name a teammate just in front that we must not drive into."""
        pose = self._world_pose()
        if pose is None:
            return None
        for name, peer in self.team_peers.items():
            if peer.get('state') in (GOAL_PARKING, GOAL_REACHED):
                continue  # the parking queue handles itself
            if not self._in_front(pose, peer['x'], peer['y'], CLEAR_AHEAD_M, CLEAR_SIDE_M):
                continue
            peer_pose = (peer['x'], peer['y'], peer.get('heading', 0.0))
            mutual = self._in_front(peer_pose, pose[0], pose[1],
                                    CLEAR_AHEAD_M, CLEAR_SIDE_M)
            # When both see each other only the later robot waits; a robot
            # turning on the spot cannot give way, so always wait for it.
            # CENTERING still drives, and we may be centring too: no exception.
            if (mutual and peer.get('state') not in (JUNCTION_SCAN, JUNCTION_PAUSE, TURNING)
                    and not self._peer_has_priority(name, peer)):
                continue
            return name
        return None

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
            if peer.get('state') in PIVOTING_STATES + (GOAL_REACHED, GOAL_PARKING):
                continue  # stopped where it is: walk up and wait behind it
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
            active_yield = (YIELDING, YIELD_TURN, YIELD_ESCAPE, YIELD_WAIT)
            return (peer.get('state') not in active_yield
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
                and not self.scan_pending and not self.await_ahead_choice
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
        self.ahead_request = None
        self.await_ahead_choice = False
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
        # Judge the line end by this yield's own reversing only; the RGB row
        # may be briefly off the line right after a turn.
        self.last_path_color_distance = self.distance_total
        self._change_state(YIELDING)
        self.get_logger().debug(
            f'ถอยตามเส้นให้ {peer_name} จนกว่าเขาจะเลี้ยวพ้นทางร่วม')
