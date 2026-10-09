"""Coordinate branch reservations and goal announcements for a small robot team.

The wire format is JSON in std_msgs/String. The ledger is deliberately free of
ROS imports so its decisions can be unit-tested without Gazebo or ROS running.
"""

import json
import math
import time

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

    def __init__(self):
        self.junctions = {}
        self.next_junction_id = 1
        self.robots = {}
        self.goal = None
        self.decisions = {}

    def update_pose(self, report, now):
        robot = report['robot']
        self.robots[robot] = {
            'x': float(report['x']),
            'y': float(report['y']),
            'heading': float(report.get('heading', 0.0)),
            'state': str(report.get('state', 'UNKNOWN')),
            'seen_at': now,
        }

    def _junction(self, x, y):
        for node in self.junctions.values():
            if math.hypot(node['x'] - x, node['y'] - y) <= MATCH_RADIUS_M:
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
        candidates = []
        for option in report['choices']:
            heading = float(option['heading'])
            key = direction_key(heading)
            branch = node['branches'].setdefault(
                key, {'status': 'OPEN', 'owner': None})
            if branch['status'] == 'BLOCKED':
                continue
            owner = branch['owner']
            owner_live = (owner in self.robots and
                          now - self.robots[owner]['seen_at'] < PEER_TIMEOUT_SECONDS)
            unclaimed = owner is None or owner == robot or not owner_live
            # The current order of choices is the robot's local preference.
            score = 1 if unclaimed else 0
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
            result = {'robot': robot, 'request_id': request_id,
                      'choice': None, 'junction_id': node['id'],
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'reason': 'NO_OPEN_BRANCH'}
        else:
            _, _, _, option, key = max(candidates)
            branch = node['branches'][key]
            shared = branch['owner'] not in (None, robot)
            if not shared:
                branch['owner'] = robot
                branch['status'] = 'CLAIMED'
            result = {'robot': robot, 'request_id': request_id,
                      'choice': option['name'], 'junction_id': node['id'],
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'direction': key, 'shared': shared}
        self.decisions[token] = result
        return result

    def block_branch(self, report):
        node = self.junctions.get(int(report['junction_id']))
        if node is None:
            return
        branch = node['branches'].get(int(report['direction']))
        if branch is not None and branch['owner'] == report['robot']:
            branch['status'] = 'BLOCKED'
            branch['owner'] = None

    def set_goal(self, report):
        if self.goal is None:
            self.goal = {'robot': report['robot'],
                         'x': float(report['x']), 'y': float(report['y'])}

    def snapshot(self, now):
        return {
            'goal': self.goal,
            'robots': {
                name: {'x': p['x'], 'y': p['y'],
                       'heading': p['heading'], 'state': p['state']}
                for name, p in self.robots.items()
                if now - p['seen_at'] < PEER_TIMEOUT_SECONDS
            },
            'junctions': list(self.junctions.values()),
        }


class TeamCoordinator(Node):
    def __init__(self):
        super().__init__('maze_team_coordinator')
        self.ledger = TeamLedger()
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
                self._publish(self.decision_pub, result)
                self.get_logger().info(
                    f"{result['robot']} at J{result['junction_id']}: "
                    f"{result['choice']} (shared={result.get('shared', False)})")
                self._publish_state()
            elif kind == 'BLOCKED':
                self.ledger.block_branch(report)
                self._publish_state()
            elif kind == 'GOAL':
                self.ledger.set_goal(report)
                self._publish_state()
                self.get_logger().info(f"Goal reported by {report['robot']}")
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring malformed team report: {exc}')

    def _publish_state(self):
        self._publish(self.state_pub, self.ledger.snapshot(time.monotonic()))


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
