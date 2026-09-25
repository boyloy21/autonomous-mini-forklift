#!/usr/bin/env python3
"""
sensor_monitor_node.py

Watches LiDAR, Camera, and IMU topic activity -- message timeout and
frequency -- and publishes standard ROS2 diagnostics on /diagnostics.
Does NOT interpret or process sensor data itself; only observes whether
messages are arriving, how fast, and how long since the last one.

Status logic per sensor, evaluated on every publish tick:
    ERROR - no message ever received, OR last message older than timeout
    WARN  - receiving, but below the configured minimum frequency
    OK    - receiving at or above the configured minimum frequency

Parameters (see config/diagnostics.yaml):
    lidar_topic, camera_topic, imu_topic
    lidar_timeout_sec, camera_timeout_sec, imu_timeout_sec
    lidar_min_freq_hz, camera_min_freq_hz, imu_min_freq_hz
    publish_rate_hz
    diagnostics_topic
"""

import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SensorWatcher:
    """Tracks message arrival timing for one sensor topic and derives a
    DiagnosticStatus from it. Keeps only timestamps (a small rolling
    window) -- never touches the message payload, since this node has no
    business interpreting sensor data, only observing whether it's
    flowing."""

    def __init__(self, name: str, timeout_sec: float, min_freq_hz: float, window: int = 20):
        self.name = name
        self.timeout_sec = timeout_sec
        self.min_freq_hz = min_freq_hz
        self.last_stamp = None  # float seconds, node clock
        self.msg_times = collections.deque(maxlen=window)

    def on_message(self, now_sec: float):
        self.last_stamp = now_sec
        self.msg_times.append(now_sec)

    def current_frequency_hz(self):
        if len(self.msg_times) < 2:
            return None
        span = self.msg_times[-1] - self.msg_times[0]
        if span <= 0.0:
            return None
        return (len(self.msg_times) - 1) / span

    def get_status(self, now_sec: float) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f"sensor_monitor: {self.name}"
        status.hardware_id = self.name.lower()

        if self.last_stamp is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "No messages received yet"
            status.values.append(KeyValue(key="frequency_hz", value="0.0"))
            status.values.append(KeyValue(key="seconds_since_last_msg", value="n/a"))
            return status

        elapsed = now_sec - self.last_stamp
        freq = self.current_frequency_hz()
        freq_str = f"{freq:.2f}" if freq is not None else "n/a"
        status.values.append(KeyValue(key="frequency_hz", value=freq_str))
        status.values.append(KeyValue(key="seconds_since_last_msg", value=f"{elapsed:.2f}"))
        status.values.append(KeyValue(key="min_expected_freq_hz", value=f"{self.min_freq_hz:.2f}"))
        status.values.append(KeyValue(key="timeout_sec", value=f"{self.timeout_sec:.2f}"))

        if elapsed > self.timeout_sec:
            status.level = DiagnosticStatus.ERROR
            status.message = f"No data for {elapsed:.1f}s (timeout {self.timeout_sec:.1f}s)"
        elif freq is not None and freq < self.min_freq_hz:
            status.level = DiagnosticStatus.WARN
            status.message = f"Low frequency: {freq:.1f}Hz (expected >= {self.min_freq_hz:.1f}Hz)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"Receiving at {freq_str}Hz" if freq is not None else "Receiving (warming up)"

        return status


class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__("sensor_monitor_node")

        self.declare_parameter("lidar_topic", "scan")
        self.declare_parameter("camera_topic", "camera/image_raw")
        self.declare_parameter("imu_topic", "imu")
        self.declare_parameter("lidar_timeout_sec", 2.0)
        self.declare_parameter("camera_timeout_sec", 2.0)
        self.declare_parameter("imu_timeout_sec", 1.0)
        self.declare_parameter("lidar_min_freq_hz", 5.0)
        self.declare_parameter("camera_min_freq_hz", 10.0)
        self.declare_parameter("imu_min_freq_hz", 20.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("diagnostics_topic", "diagnostics")

        lidar_topic = self.get_parameter("lidar_topic").value
        camera_topic = self.get_parameter("camera_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        diagnostics_topic = self.get_parameter("diagnostics_topic").value

        self.lidar_watcher = SensorWatcher(
            "LiDAR",
            self.get_parameter("lidar_timeout_sec").value,
            self.get_parameter("lidar_min_freq_hz").value,
        )
        self.camera_watcher = SensorWatcher(
            "Camera",
            self.get_parameter("camera_timeout_sec").value,
            self.get_parameter("camera_min_freq_hz").value,
        )
        self.imu_watcher = SensorWatcher(
            "IMU",
            self.get_parameter("imu_timeout_sec").value,
            self.get_parameter("imu_min_freq_hz").value,
        )

        self.lidar_sub = self.create_subscription(
            LaserScan, lidar_topic, self._make_callback(self.lidar_watcher),
            qos_profile_sensor_data)
        self.camera_sub = self.create_subscription(
            Image, camera_topic, self._make_callback(self.camera_watcher),
            qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._make_callback(self.imu_watcher),
            qos_profile_sensor_data)

        self.diag_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)

        period_sec = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 1.0
        self.timer = self.create_timer(period_sec, self._publish_diagnostics)

        self.get_logger().info(
            f"sensor_monitor_node watching: lidar='{lidar_topic}', "
            f"camera='{camera_topic}', imu='{imu_topic}'")

    def _make_callback(self, watcher: SensorWatcher):
        def callback(_msg):
            watcher.on_message(self.get_clock().now().nanoseconds / 1e9)
        return callback

    def _publish_diagnostics(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(self.lidar_watcher.get_status(now_sec))
        array.status.append(self.camera_watcher.get_status(now_sec))
        array.status.append(self.imu_watcher.get_status(now_sec))

        self.diag_pub.publish(array)


def main():
    rclpy.init()
    node = SensorMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()