"""Coordinate branch reservations and goal announcements for a small robot team.

The wire format is JSON in std_msgs/String. The ledger is deliberately free of
ROS imports so its decisions can be unit-tested without Gazebo or ROS running.
"""

import json
import time
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


from .coordinator_parts.config import (  # noqa: F401 - public constants for callers
    MATCH_RADIUS_M,
    PEER_TIMEOUT_SECONDS,
    JUNCTION_REACHED_M,
    JUNCTION_PASSED_M,
    REASSIGN_MARGIN_M,
    DIRECTION_SECTORS,
    direction_key,
)


from .coordinator_parts.map_assignments import MapAssignmentsMixin
from .coordinator_parts.branch_routes import BranchRoutesMixin
from .coordinator_parts.encounters import EncountersMixin
from .coordinator_parts.goal_state import GoalStateMixin


class TeamLedger(MapAssignmentsMixin, BranchRoutesMixin, EncountersMixin, GoalStateMixin):
    """Shared, observation-based junction/branch ledger; no preloaded maze."""

    def __init__(self, match_radius=MATCH_RADIUS_M):
        self.match_radius = match_radius
        self.junctions = {}
        self.next_junction_id = 1
        self.next_route_order = 1
        self.robots = {}
        self.goal = None
        self._routing_revision = 0
        self._routing_signature = None
        self.decisions = {}
        # Shared map: where each robot left its last node, and links that
        # contradicted an earlier one (kept out of the map, reported).
        self.trails = {}
        self.link_conflicts = []
        # Head-on encounters: who hides where, and who waits for whom.
        self.encounters = {}
        self.next_encounter_id = 1
        # Corridors a robot gave up to a teammate it met head-on.
        self.avoid = {}
        # (Junction ID, direction) of the unknown branch a robot is heading toward.
        self.frontier_claims = {}
        # Trail before each granted request, restored if it is cancelled.
        self.request_trails = {}
        # C2: unexplored branch the team reserved for each idle robot.
        self.assignments = {}
        # Robots that have stood at a junction: the next one may leave START.
        self.junction_reached = set()


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

    def _publish_orders(self, orders):
        for order in orders:
            self._publish(self.decision_pub, order)
            self.get_logger().info(
                f"{order['type']} -> {order['robot']} (encounter {order['encounter']})")
        if orders:
            self._publish_state()

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
                self._publish_orders(self.ledger.check_clear(now))
            elif kind == 'HEAD_ON':
                self._publish_orders(
                    self.ledger.head_on(report['robot'], report['peer'], now))
            elif kind == 'HIDDEN':
                self._publish_orders(self.ledger.hidden(report['robot']))
            elif kind == 'RELEASE_BRANCH':
                self.ledger.release_branch(report, now)
                self._publish_state()
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
            elif kind == 'NODE':
                conflicts = len(self.ledger.link_conflicts)
                self.ledger.report_node(report, now)
                for line in self.ledger.link_conflicts[conflicts:]:
                    self.get_logger().warning(f'map link ignored: {line}')
                self._publish_state()
            elif kind == 'GOAL':
                first = self.ledger.goal is None
                self.ledger.set_goal(report)
                if first:
                    for order in self.ledger.goal_routes(now):
                        self._publish(self.decision_pub, order)
                self._publish_state()
                self.get_logger().info(f"Goal reported by {report['robot']}")
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring malformed team report: {exc}')

    def _publish_state(self):
        self._publish(self.state_pub, {
            **self.ledger.snapshot(time.monotonic()),
            'map_frame': {'east': '+x', 'north': '+y', 'distance_unit': 'm'},
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
