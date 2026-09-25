#!/usr/bin/env python3
"""
navigation_bridge_node.py

Thin translation layer exposing Nav2's NavigateToPose action to the web
dashboard as simple JSON commands over a topic, relaying its feedback/
result the same way. No navigation algorithm here -- forwards a single-
point "drive here" command to Nav2 and reports back what Nav2 says.

Multi-waypoint PATROL is intentionally NOT handled here -- that
sequencing logic already exists as a mission type in
forklift_mission_manager (and a task type in forklift_task_manager).
Route patrol requests through mission_bridge_node instead, to avoid
duplicating waypoint-sequencing logic in a second place.

Command topic (sub): web_bridge/nav_command  (std_msgs/String, JSON)
    {"command": "navigate", "x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map"}
    {"command": "cancel"}

Status topic (pub): web_bridge/navigation_status  (std_msgs/String, JSON)
    {"state": "IDLE"|"EXECUTING"|"SUCCEEDED"|"FAILED"|"CANCELED",
     "message": "...", "goal_active": true|false}
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NavigationBridgeNode(Node):
    def __init__(self):
        super().__init__("navigation_bridge_node")

        self.declare_parameter("nav_command_topic", "web_bridge/nav_command")
        self.declare_parameter("nav_status_topic", "web_bridge/navigation_status")
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("default_frame_id", "map")

        cmd_topic = self.get_parameter("nav_command_topic").value
        status_topic = self.get_parameter("nav_status_topic").value
        action_name = self.get_parameter("navigate_action_name").value
        self.default_frame_id = self.get_parameter("default_frame_id").value

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.cmd_sub = self.create_subscription(String, cmd_topic, self._on_command, 10)

        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self._goal_handle = None

        self._publish_status("IDLE", "navigation_bridge_node ready", goal_active=False)
        self.get_logger().info(f"navigation_bridge_node ready, listening on '{cmd_topic}'")

    def _publish_status(self, state, message, goal_active):
        msg = String()
        msg.data = json.dumps({"state": state, "message": message, "goal_active": goal_active})
        self.status_pub.publish(msg)

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON on nav_command: {msg.data}")
            return

        action = cmd.get("command")
        if action == "navigate":
            self._send_navigate_goal(cmd)
        elif action == "cancel":
            self._cancel_goal()
        else:
            self.get_logger().warn(f"Unknown nav command: {action}")

    def _send_navigate_goal(self, cmd):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._publish_status("FAILED", "NavigateToPose action server not available", False)
            return

        pose = PoseStamped()
        pose.header.frame_id = cmd.get("frame_id", self.default_frame_id)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(cmd.get("x", 0.0))
        pose.pose.position.y = float(cmd.get("y", 0.0))
        qx, qy, qz, qw = yaw_to_quaternion(float(cmd.get("yaw", 0.0)))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._publish_status("EXECUTING", "Sending navigation goal", True)

        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status("FAILED", "Navigation goal rejected", False)
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        distance = getattr(fb, "distance_remaining", None)
        message = f"distance_remaining={distance:.2f}m" if distance is not None else "navigating"
        self._publish_status("EXECUTING", message, True)

    def _on_result(self, future):
        result = future.result()
        self._goal_handle = None
        code = result.status
        # action_msgs/msg/GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if code == 4:
            self._publish_status("SUCCEEDED", "Navigation goal reached", False)
        elif code == 5:
            self._publish_status("CANCELED", "Navigation canceled", False)
        else:
            self._publish_status("FAILED", f"Navigation failed (status={code})", False)

    def _cancel_goal(self):
        if self._goal_handle is None:
            self._publish_status("IDLE", "No active navigation goal to cancel", False)
            return
        self._goal_handle.cancel_goal_async()
        self._publish_status("EXECUTING", "Cancel requested", True)


def main():
    rclpy.init()
    node = NavigationBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()