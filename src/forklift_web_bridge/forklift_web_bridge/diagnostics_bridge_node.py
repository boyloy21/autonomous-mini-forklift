#!/usr/bin/env python3
"""
diagnostics_bridge_node.py

Subscribes to /diagnostics (diagnostic_msgs/DiagnosticArray, published by
forklift_diagnostics) and republishes a simplified, web-friendly JSON
summary -- overall level plus a compact per-component list -- instead of
the verbose DiagnosticStatus/KeyValue structure, which is awkward to
consume directly in JS. Pure format simplification; does not compute any
new diagnostic judgment itself -- forklift_diagnostics already decided
every OK/WARN/ERROR level.

Publishes: web_bridge/diagnostics_summary (std_msgs/String, JSON)
    {"overall_level": "OK"|"WARNING"|"ERROR"|"STALE",
     "components": [{"name":.., "level":.., "message":..}, ...],
     "timestamp": ..}
"""

import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


_LEVEL_NAMES = {
    DiagnosticStatus.OK: "OK",
    DiagnosticStatus.WARN: "WARNING",
    DiagnosticStatus.ERROR: "ERROR",
    DiagnosticStatus.STALE: "STALE",
}


class DiagnosticsBridgeNode(Node):
    def __init__(self):
        super().__init__("diagnostics_bridge_node")

        self.declare_parameter("diagnostics_topic", "diagnostics")
        self.declare_parameter("summary_topic", "web_bridge/diagnostics_summary")

        diag_topic = self.get_parameter("diagnostics_topic").value
        summary_topic = self.get_parameter("summary_topic").value

        self.summary_pub = self.create_publisher(String, summary_topic, 10)
        self.diag_sub = self.create_subscription(
            DiagnosticArray, diag_topic, self._on_diagnostics, 10)

        self.get_logger().info(
            f"diagnostics_bridge_node relaying '{diag_topic}' -> '{summary_topic}'")

    def _on_diagnostics(self, msg: DiagnosticArray):
        components = []
        overall_level = DiagnosticStatus.OK

        for status in msg.status:
            components.append({
                "name": status.name,
                "level": _LEVEL_NAMES.get(status.level, "UNKNOWN"),
                "message": status.message,
            })
            if status.level > overall_level:
                overall_level = status.level

        summary = {
            "overall_level": _LEVEL_NAMES.get(overall_level, "UNKNOWN"),
            "components": components,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }

        out = String()
        out.data = json.dumps(summary)
        self.summary_pub.publish(out)


def main():
    rclpy.init()
    node = DiagnosticsBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()