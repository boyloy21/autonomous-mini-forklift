#!/usr/bin/env python3
"""
route_follow_test.py

Manual one-off test bridging route_server's computed path to
controller_server's FollowPath action -- confirms the "compute route,
then separately drive it" two-step pattern actually works, before
building it properly into task_manager_node.cpp.

Usage:
  python3 route_follow_test.py forklift1

Does NOT call compute_and_track_route itself -- run that separately
first (in another terminal, as before), THEN run this script while it's
still tracking, to pick up the live /plan it's publishing.
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path


class RouteFollowTest(Node):
    def __init__(self, robot_namespace: str):
        super().__init__("route_follow_test")
        self.ns = robot_namespace
        self.plan_received = False

        self.sub = self.create_subscription(
            Path, f"/{self.ns}/route_plan", self.plan_callback, 10)
        self.follow_path_client = ActionClient(
            self, FollowPath, f"/{self.ns}/follow_path")

    def plan_callback(self, msg: Path):
        if self.plan_received:
            return  # only act on the first one
        self.plan_received = True

        if len(msg.poses) == 0:
            self.get_logger().error("Received an empty path -- nothing to follow.")
            return

        self.get_logger().info(
            f"Got path with {len(msg.poses)} poses, sending FollowPath goal...")

        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowPath action server not available.")
            return

        goal = FollowPath.Goal()
        goal.path = msg
        goal.controller_id = ""  # use default

        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("FollowPath goal was rejected.")
            return
        self.get_logger().info("FollowPath goal accepted, robot should now be moving.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"FollowPath finished: {result.result}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 route_follow_test.py <robot_namespace>")
        sys.exit(1)

    robot_namespace = sys.argv[1]

    rclpy.init()
    node = RouteFollowTest(robot_namespace)
    print(f"Listening on /{robot_namespace}/plan for the first published path...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()