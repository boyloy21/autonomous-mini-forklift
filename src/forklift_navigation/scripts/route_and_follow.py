#!/usr/bin/env python3
"""
route_and_follow.py

Bypasses bt_navigator / default_nav_to_pose_bt_xml entirely for testing.
Calls route_server's ComputeRoute action directly, then feeds the
resulting nav_msgs/Path straight into controller_server's FollowPath
action. No BT involved -- this isolates whether route_server + the
controller work correctly on their own, independent of any BT XML
config or port-mismatch issues.

Usage:
  ros2 run <your_pkg> route_and_follow.py --start-id 10 --goal-id 1
  ros2 run <your_pkg> route_and_follow.py --goal-x 1.5 --goal-y 4.5
    (goal by pose; start always taken from TF unless --start-id given)
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeRoute, FollowPath
from geometry_msgs.msg import PoseStamped


class RouteAndFollow(Node):
    def __init__(self):
        super().__init__('route_and_follow')
        self.compute_route_client = ActionClient(self, ComputeRoute, '/compute_route')
        self.follow_path_client = ActionClient(self, FollowPath, '/follow_path')

    def run(self, start_id=None, goal_id=None, goal_x=None, goal_y=None):
        goal_msg = ComputeRoute.Goal()

        if start_id is not None:
            goal_msg.use_start = True
            goal_msg.use_poses = False
            goal_msg.start_id = start_id
        else:
            goal_msg.use_start = False  # take current pose from TF

        if goal_id is not None:
            goal_msg.use_poses = False
            goal_msg.goal_id = goal_id
        elif goal_x is not None and goal_y is not None:
            goal_msg.use_poses = True
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.w = 1.0
            goal_msg.goal = goal
        else:
            self.get_logger().error('Must supply --goal-id or --goal-x/--goal-y')
            sys.exit(1)

        self.get_logger().info('Waiting for /compute_route action server...')
        self.compute_route_client.wait_for_server()

        self.get_logger().info('Sending ComputeRoute goal...')
        future = self.compute_route_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('ComputeRoute goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        path = result.path
        num_poses = len(path.poses)
        self.get_logger().info(f'ComputeRoute returned a path with {num_poses} poses')

        if num_poses < 2:
            self.get_logger().error(
                'Path has fewer than 2 poses -- start and goal likely snapped '
                'to the same graph node. Not sending to FollowPath.')
            for i, p in enumerate(path.poses):
                self.get_logger().info(
                    f'  pose[{i}] = ({p.pose.position.x:.2f}, {p.pose.position.y:.2f})')
            return

        for i, p in enumerate(path.poses):
            self.get_logger().info(
                f'  pose[{i}] = ({p.pose.position.x:.2f}, {p.pose.position.y:.2f})')

        self.get_logger().info('Waiting for /follow_path action server...')
        self.follow_path_client.wait_for_server()

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'

        self.get_logger().info('Sending path to FollowPath...')
        follow_future = self.follow_path_client.send_goal_async(follow_goal)
        rclpy.spin_until_future_complete(self, follow_future)
        follow_handle = follow_future.result()

        if not follow_handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            return

        follow_result_future = follow_handle.get_result_async()
        rclpy.spin_until_future_complete(self, follow_result_future)
        self.get_logger().info(f'FollowPath finished: {follow_result_future.result().status}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-id', type=int, default=None)
    parser.add_argument('--goal-id', type=int, default=None)
    parser.add_argument('--goal-x', type=float, default=None)
    parser.add_argument('--goal-y', type=float, default=None)
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = RouteAndFollow()
    node.run(start_id=args.start_id, goal_id=args.goal_id,
              goal_x=args.goal_x, goal_y=args.goal_y)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()