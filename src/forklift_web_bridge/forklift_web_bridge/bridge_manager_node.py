#!/usr/bin/env python3
"""
bridge_manager_node.py

Lightweight supervisor: periodically checks whether the ROS2 endpoints
this bridge package depends on are actually present on the graph
(Nav2's NavigateToPose action, forklift_mission_manager's ExecuteMission
action, forklift_task_manager's ExecuteTask action, /diagnostics, /odom),
and publishes a single readiness summary the web dashboard can check
once instead of separately probing every dependency itself.

Does NOT start or manage the other bridge nodes' process lifecycle
(that's the launch file's job) -- "manage" here means monitoring
connectivity, not process supervision.

Publishes: web_bridge/status (std_msgs/String, JSON)
    {"bridges_ready": true|false,
     "checks": {"navigate_to_pose": true, "execute_mission": true, ...},
     "timestamp": ..}
"""

import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BridgeManagerNode(Node):
    def __init__(self):
        super().__init__("bridge_manager_node")

        self.declare_parameter("status_topic", "web_bridge/status")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("execute_mission_action_name", "execute_mission")
        self.declare_parameter("execute_task_action_name", "execute_task")
        self.declare_parameter("diagnostics_topic", "diagnostics")
        self.declare_parameter("odom_topic", "odom")

        status_topic = self.get_parameter("status_topic").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value

        self.status_pub = self.create_publisher(String, status_topic, 10)

        period_sec = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 1.0
        self.timer = self.create_timer(period_sec, self._check_and_publish)

        self.get_logger().info("bridge_manager_node ready")

    def _action_server_present(self, action_name: str) -> bool:
        # Action servers register a family of services under the action
        # name (e.g. "<name>/_action/send_goal") -- checking the service
        # list avoids creating a throwaway ActionClient per check.
        service_names = [name for name, _ in self.get_service_names_and_types()]
        # Graph names are fully qualified; expand relative names against
        # this node's namespace so multi-robot ("/forklift1/...") works.
        action_name = self.resolve_topic_name(action_name)
        return any(name.startswith(action_name + "/_action/") for name in service_names)

    def _topic_present(self, topic_name: str) -> bool:
        topic_names = [name for name, _ in self.get_topic_names_and_types()]
        return self.resolve_topic_name(topic_name) in topic_names

    def _check_and_publish(self):
        checks = {
            "navigate_to_pose": self._action_server_present(
                self.get_parameter("navigate_action_name").value),
            "execute_mission": self._action_server_present(
                self.get_parameter("execute_mission_action_name").value),
            "execute_task": self._action_server_present(
                self.get_parameter("execute_task_action_name").value),
            "diagnostics": self._topic_present(self.get_parameter("diagnostics_topic").value),
            "odom": self._topic_present(self.get_parameter("odom_topic").value),
        }

        status = {
            "bridges_ready": all(checks.values()),
            "checks": checks,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = BridgeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()