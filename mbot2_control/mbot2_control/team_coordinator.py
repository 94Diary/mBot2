"""Coordinate branch reservations and goal announcements for a small robot team.

The wire format is JSON in std_msgs/String. The ledger is deliberately free of
ROS imports so its decisions can be unit-tested without Gazebo or ROS running.
"""

import json
import math
import time
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


MATCH_RADIUS_M = 1.00
PEER_TIMEOUT_SECONDS = 4.0
DIRECTION_SECTORS = 4  # the current line maze has orthogonal branches


def direction_key(heading):
    """Turn a world-frame heading into a cardinal branch key."""
    return round((heading % (2 * math.pi)) * DIRECTION_SECTORS /
                 (2 * math.pi)) % DIRECTION_SECTORS


class TeamLedger:
    """Shared, observation-based junction/branch ledger; no preloaded maze."""

    def __init__(self, match_radius=MATCH_RADIUS_M):
        self.match_radius = match_radius
        self.junctions = {}
        self.next_junction_id = 1
        self.next_route_order = 1
        self.robots = {}
        self.goal = None
        self.decisions = {}

    def update_pose(self, report, now):
        robot = report['robot']
        previous = self.robots.get(robot)
        if previous is None:
            route_order = self.next_route_order
            self.next_route_order += 1
        else:
            route_order = previous['route_order']
        if report.get('route_order') is not None:
            route_order = int(report['route_order'])
        self.robots[robot] = {
            'x': float(report['x']),
            'y': float(report['y']),
            'heading': float(report.get('heading', 0.0)),
            'state': str(report.get('state', 'UNKNOWN')),
            'returning': bool(report.get('returning', False)),
            'yield_to': report.get('yield_to'),
            'yield_junction': report.get('yield_junction'),
            'yield_escape_heading': report.get('yield_escape_heading'),
            'route_order': route_order,
            'seen_at': now,
        }

    def _junction(self, x, y):
        for node in self.junctions.values():
            if math.hypot(node['x'] - x, node['y'] - y) <= self.match_radius:
                return node
        node_id = self.next_junction_id
        self.next_junction_id += 1
        node = {'id': node_id, 'x': x, 'y': y, 'branches': {}}
        self.junctions[node_id] = node
        return node

    def request_branch(self, report, now):
        robot = report['robot']
        request_id = report['request_id']
        token = (robot, request_id)
        if token in self.decisions:
            return self.decisions[token]

        node = self._junction(float(report['x']), float(report['y']))
        # Arriving along a corridor is evidence that it has already been used.
        # Without this, a later approach from the opposite side can mistake
        # the original entrance for an unexplored straight-ahead branch.
        if report.get('approach_heading') is not None:
            inbound_key = direction_key(float(report['approach_heading']) + math.pi)
            inbound = node['branches'].setdefault(
                inbound_key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
            arrived = inbound.setdefault('arrived_by', [])
            if robot not in arrived:
                arrived.append(robot)
            if robot not in inbound['visited_by']:
                inbound['visited_by'].append(robot)
        candidates = []
        branch_details = []
        for option in report['choices']:
            key = direction_key(float(option['heading']))
            branch = node['branches'].setdefault(
                key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
            branch_details.append({
                'name': option['name'], 'status': branch['status'],
                'visited_by': list(branch['visited_by']),
                'fallback': bool(option.get('fallback', option['name'] == 'back')),
                'failed_attempts': branch.get('failed_by', {}).get(robot, 0),
            })
            if option.get('fallback', option['name'] == 'back'):
                continue  # use the known inbound route only as a fallback
            heading = float(option['heading'])
            if branch['status'] == 'BLOCKED':
                continue
            # A robot that already explored this branch must try another one.
            # Teammates may still share it when every unclaimed path is busy.
            # Reservations and failed line capture do not prove a dead end.
            owner = branch['owner']
            owner_live = (owner in self.robots and
                          now - self.robots[owner]['seen_at'] < PEER_TIMEOUT_SECONDS)
            unclaimed = owner is None or owner == robot or not owner_live
            # The current order of choices is the robot's local preference.
            # A failed alignment is not exploration: never let its retry
            # penalty make a known corridor beat an unexplored side branch.
            score = (not bool(branch['visited_by']),
                     -branch.get('failed_by', {}).get(robot, 0), unclaimed)
            if self.goal is not None:
                dx = self.goal['x'] - node['x']
                dy = self.goal['y'] - node['y']
                toward_goal = (dx * math.cos(heading) +
                               dy * math.sin(heading))
            else:
                toward_goal = 0.0
            # Once a goal is known, reaching it matters more than keeping
            # robots on separate branches. Before that, spread exploration.
            priority = ((toward_goal, score) if self.goal is not None
                        else (score, toward_goal))
            candidates.append((*priority, -len(candidates), option, key))

        if not candidates:
            for option in report['choices']:
                if not option.get('fallback', option['name'] == 'back'):
                    continue
                key = direction_key(float(option['heading']))
                branch = node['branches'].setdefault(
                    key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
                if branch['status'] != 'BLOCKED':
                    candidates.append((0, 0, 0, option, key))
        if not candidates:
            result = {'robot': robot, 'request_id': request_id,
                      'choice': None, 'junction_id': node['id'],
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'reason': 'NO_OPEN_BRANCH'}
        else:
            _, _, _, option, key = max(candidates)
            branch = node['branches'][key]
            owner = branch['owner']
            owner_live = (owner in self.robots and
                          now - self.robots[owner]['seen_at'] < PEER_TIMEOUT_SECONDS)
            shared = owner not in (None, robot) and owner_live
            reason = ('TOWARD_GOAL' if self.goal is not None else
                      'KNOWN_RETURN_ROUTE' if option.get('fallback', option['name'] == 'back') else
                      'UNEXPLORED_BRANCH' if not branch['visited_by'] else
                      'REVISIT_OPEN_BRANCH')
            if not shared:
                branch['owner'] = robot
                branch['status'] = 'CLAIMED'
            if robot not in branch['visited_by']:
                branch['visited_by'].append(robot)
            route_order = self.next_route_order
            self.next_route_order += 1
            result = {'robot': robot, 'request_id': request_id,
                      'choice': option['name'], 'junction_id': node['id'],
                      'heading': float(option['heading']),
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'direction': key, 'shared': shared,
                      'reason': reason,
                      'route_order': route_order}
        result['branches'] = branch_details
        self.decisions[token] = result
        return result

    def cancel_branch(self, report):
        """Release a junction choice when its robot has to yield before moving."""
        token = (report['robot'], report['request_id'])
        decision = self.decisions.pop(token, None)
        if decision is None or decision.get('direction') is None:
            return
        node = self.junctions.get(decision['junction_id'])
        if node is None:
            return
        branch = node['branches'].get(decision['direction'])
        if branch is None:
            return
        if branch['owner'] == report['robot'] and branch['status'] != 'BLOCKED':
            branch['owner'] = None
            branch['status'] = 'OPEN'
        if (report['robot'] in branch['visited_by'] and
                report['robot'] not in branch.get('arrived_by', [])):
            branch['visited_by'].remove(report['robot'])

    def block_branch(self, report):
        node = self.junctions.get(int(report['junction_id']))
        if node is None:
            return
        branch = node['branches'].get(int(report['direction']))
        if branch is not None:
            branch['status'] = 'BLOCKED'
            branch['owner'] = None

    def retry_branch(self, report):
        self.cancel_branch(report)
        node = self.junctions.get(int(report['junction_id']))
        if node is None:
            return
        branch = node['branches'].get(int(report['direction']))
        if branch is not None:
            failed = branch.setdefault('failed_by', {})
            failed[report['robot']] = failed.get(report['robot'], 0) + 1

    def set_goal(self, report):
        if self.goal is None:
            self.goal = {'robot': report['robot'],
                         'x': float(report['x']), 'y': float(report['y'])}

    def snapshot(self, now):
        return {
            'goal': self.goal,
            'robots': {
                name: {'x': p['x'], 'y': p['y'],
                       'heading': p['heading'], 'state': p['state'],
                       'route_order': p['route_order'],
                       'returning': p['returning'],
                       'yield_to': p['yield_to'],
                       'yield_junction': p['yield_junction'],
                       'yield_escape_heading': p['yield_escape_heading']}
                for name, p in self.robots.items()
                if now - p['seen_at'] < PEER_TIMEOUT_SECONDS
            },
            'junctions': list(self.junctions.values()),
        }


class TeamCoordinator(Node):
    def __init__(self):
        super().__init__('maze_team_coordinator')
        self.run_id = uuid.uuid4().hex
        self.run_started_at = datetime.now(timezone.utc).isoformat()
        self.declare_parameter('junction_match_radius', MATCH_RADIUS_M)
        self.ledger = TeamLedger(
            float(self.get_parameter('junction_match_radius').value))
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.decision_pub = self.create_publisher(String, '/maze_team/decision', qos)
        self.state_pub = self.create_publisher(String, '/maze_team/state', qos)
        self.create_subscription(String, '/maze_team/report', self._on_report, 50)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info('Team coordinator ready: branch reservations and shared goal')

    def _publish(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)

    def _on_report(self, msg):
        try:
            report = json.loads(msg.data)
            kind = report['type']
            now = time.monotonic()
            if kind == 'POSE':
                self.ledger.update_pose(report, now)
            elif kind == 'BRANCH_REQUEST':
                result = self.ledger.request_branch(report, now)
                self._publish(self.decision_pub, {
                    **result,
                    'run_id': self.run_id,
                    'run_started_at': self.run_started_at,
                })
                self.get_logger().debug(
                    f"{result['robot']} at J{result['junction_id']}: "
                    f"{result['choice']} (shared={result.get('shared', False)})")
                self._publish_state()
            elif kind == 'BLOCKED':
                self.ledger.block_branch(report)
                self._publish_state()
            elif kind == 'RETRY_BRANCH':
                self.ledger.retry_branch(report)
                self._publish_state()
            elif kind == 'CANCEL_BRANCH':
                self.ledger.cancel_branch(report)
                self._publish_state()
            elif kind == 'GOAL':
                self.ledger.set_goal(report)
                self._publish_state()
                self.get_logger().info(f"Goal reported by {report['robot']}")
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring malformed team report: {exc}')

    def _publish_state(self):
        self._publish(self.state_pub, {
            **self.ledger.snapshot(time.monotonic()),
            'run_id': self.run_id,
            'run_started_at': self.run_started_at,
        })


def main(args=None):
    rclpy.init(args=args)
    node = TeamCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
