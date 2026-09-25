#!/usr/bin/env python3
"""
robot_state_node.py

Aggregates robot pose/velocity (/odom) and battery state (/battery_state)
into a single, web-friendly JSON topic -- rather than requiring the
dashboard to manage several separate roslibjs subscriptions and merge
them client-side. Pure aggregation/format-simplification; computes
nothing new, interprets nothing.

Publishes: web_bridge/robot_state (std_msgs/String, JSON), at
publish_rate_hz.
    {"pose": {"x":.., "y":.., "yaw":..} | null,
     "velocity": {"linear":.., "angular":..} | null,
     "battery": {"percentage":.., "current":.., "voltage":..} | null,
     "timestamp": ..}
"""

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotStateNode(Node):
    def __init__(self):
        super().__init__("robot_state_node")

        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("robot_state_topic", "web_bridge/robot_state")
        self.declare_parameter("publish_rate_hz", 5.0)

        odom_topic = self.get_parameter("odom_topic").value
        battery_topic = self.get_parameter("battery_topic").value
        state_topic = self.get_parameter("robot_state_topic").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value

        self._latest_odom = None
        self._latest_battery = None

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, battery_topic, self._on_battery, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)

        period_sec = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 0.2
        self.timer = self.create_timer(period_sec, self._publish_state)

        self.get_logger().info(
            f"robot_state_node aggregating '{odom_topic}' + '{battery_topic}' -> '{state_topic}'")

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _on_battery(self, msg: BatteryState):
        self._latest_battery = msg

    def _publish_state(self):
        state = {"timestamp": self.get_clock().now().nanoseconds / 1e9}

        if self._latest_odom is not None:
            p = self._latest_odom.pose.pose.position
            q = self._latest_odom.pose.pose.orientation
            v = self._latest_odom.twist.twist
            state["pose"] = {"x": p.x, "y": p.y, "yaw": yaw_from_quaternion(q)}
            state["velocity"] = {"linear": v.linear.x, "angular": v.angular.z}
        else:
            state["pose"] = None
            state["velocity"] = None

        if self._latest_battery is not None:
            b = self._latest_battery
            percentage = b.percentage if b.percentage == b.percentage else None  # filters NaN
            state["battery"] = {
                "percentage": percentage,
                "current": b.current,
                "voltage": b.voltage,
            }
        else:
            state["battery"] = None

        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()