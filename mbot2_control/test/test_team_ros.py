"""ROS topic smoke test without Gazebo hardware or movement."""

import json
import math
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mbot2_control.team_coordinator import TeamCoordinator


class TeamRosTest(unittest.TestCase):
    def test_branch_replies_cross_ros_topics(self):
        rclpy.init()
        coordinator = TeamCoordinator()
        client = Node('team_protocol_smoke_test')
        reports = client.create_publisher(String, '/maze_team/report', 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        decisions = []
        subscription = client.create_subscription(
            String, '/maze_team/decision',
            lambda msg: decisions.append(json.loads(msg.data)), qos)
        executor = SingleThreadedExecutor()
        executor.add_node(coordinator)
        executor.add_node(client)
        try:
            deadline = time.monotonic() + 3.0
            while reports.get_subscription_count() == 0 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertGreater(reports.get_subscription_count(), 0)

            def send(payload):
                msg = String()
                msg.data = json.dumps(payload)
                reports.publish(msg)

            for robot in ('mbot1', 'mbot2'):
                send({'type': 'POSE', 'robot': robot, 'x': -2.0, 'y': 0.0})
            options = [{'name': 'straight', 'heading': 0.0},
                       {'name': 'left', 'heading': math.pi / 2}]
            send({'type': 'BRANCH_REQUEST', 'robot': 'mbot1',
                  'request_id': 'test-1', 'x': -2.0, 'y': 0.0,
                  'choices': options})
            send({'type': 'BRANCH_REQUEST', 'robot': 'mbot2',
                  'request_id': 'test-2', 'x': -2.05, 'y': 0.0,
                  'choices': options})
            deadline = time.monotonic() + 5.0
            while len(decisions) < 2 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            by_robot = {item['robot']: item['choice'] for item in decisions}
            self.assertEqual({'mbot1': 'straight', 'mbot2': 'left'}, by_robot)
        finally:
            executor.remove_node(client)
            executor.remove_node(coordinator)
            client.destroy_subscription(subscription)
            client.destroy_node()
            coordinator.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
