#!/usr/bin/env python3
"""
system_monitor_node.py

Monitors host system resource usage -- CPU, GPU, RAM, Temperature,
Network -- and publishes standard ROS2 diagnostics on /diagnostics.
Read-only observation only; no control or safety-stop action of any kind.

Status logic per metric, evaluated on every publish tick:
    ERROR - value >= configured error threshold
    WARN  - value >= configured warn threshold (and < error threshold)
    OK    - below warn threshold

GPU monitoring uses `nvidia-smi` via subprocess -- if it's not present
(no NVIDIA GPU, or not installed), GPU status degrades gracefully to
WARN with an explanatory message rather than crashing the node, so this
package stays usable on non-NVIDIA machines too.

Parameters (see config/diagnostics.yaml):
    cpu_warn_percent, cpu_error_percent
    ram_warn_percent, ram_error_percent
    temp_warn_c, temp_error_c
    gpu_enabled
    publish_rate_hz
    diagnostics_topic
"""

import shutil
import subprocess

import psutil

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def level_for(value: float, warn: float, error: float) -> int:
    if value >= error:
        return DiagnosticStatus.ERROR
    if value >= warn:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.OK


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__("system_monitor_node")

        self.declare_parameter("cpu_warn_percent", 80.0)
        self.declare_parameter("cpu_error_percent", 95.0)
        self.declare_parameter("ram_warn_percent", 80.0)
        self.declare_parameter("ram_error_percent", 95.0)
        self.declare_parameter("temp_warn_c", 75.0)
        self.declare_parameter("temp_error_c", 90.0)
        self.declare_parameter("gpu_enabled", True)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("diagnostics_topic", "diagnostics")

        self.cpu_warn = self.get_parameter("cpu_warn_percent").value
        self.cpu_error = self.get_parameter("cpu_error_percent").value
        self.ram_warn = self.get_parameter("ram_warn_percent").value
        self.ram_error = self.get_parameter("ram_error_percent").value
        self.temp_warn = self.get_parameter("temp_warn_c").value
        self.temp_error = self.get_parameter("temp_error_c").value
        self.gpu_enabled = self.get_parameter("gpu_enabled").value

        diagnostics_topic = self.get_parameter("diagnostics_topic").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value

        self._nvidia_smi_path = shutil.which("nvidia-smi") if self.gpu_enabled else None
        if self.gpu_enabled and self._nvidia_smi_path is None:
            self.get_logger().warn(
                "gpu_enabled=true but nvidia-smi not found on PATH -- GPU status will "
                "report WARN with an explanatory message instead of crashing.")

        # First psutil.cpu_percent() call with no interval always returns
        # 0.0 (needs a baseline) -- call it once here so the first real
        # publish tick already has a meaningful comparison window.
        psutil.cpu_percent(interval=None)

        self.diag_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)

        period_sec = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 1.0
        self.timer = self.create_timer(period_sec, self._publish_diagnostics)

        self.get_logger().info("system_monitor_node started")

    # ------------------------------------------------------------------
    def _cpu_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "system_monitor: CPU"
        status.hardware_id = "cpu"

        percent = psutil.cpu_percent(interval=None)
        per_core = psutil.cpu_percent(interval=None, percpu=True)

        status.level = level_for(percent, self.cpu_warn, self.cpu_error)
        status.message = f"CPU usage {percent:.1f}%"
        status.values.append(KeyValue(key="usage_percent", value=f"{percent:.1f}"))
        status.values.append(KeyValue(key="core_count", value=str(psutil.cpu_count())))
        status.values.append(
            KeyValue(key="per_core_percent", value=",".join(f"{c:.0f}" for c in per_core)))
        return status

    def _ram_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "system_monitor: RAM"
        status.hardware_id = "ram"

        mem = psutil.virtual_memory()
        status.level = level_for(mem.percent, self.ram_warn, self.ram_error)
        status.message = f"RAM usage {mem.percent:.1f}%"
        status.values.append(KeyValue(key="usage_percent", value=f"{mem.percent:.1f}"))
        status.values.append(KeyValue(key="used_gb", value=f"{mem.used / 1e9:.2f}"))
        status.values.append(KeyValue(key="total_gb", value=f"{mem.total / 1e9:.2f}"))
        return status

    def _temperature_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "system_monitor: Temperature"
        status.hardware_id = "temperature"

        try:
            temps = psutil.sensors_temperatures()
        except AttributeError:
            temps = {}

        if not temps:
            status.level = DiagnosticStatus.WARN
            status.message = "No temperature sensors available on this platform"
            return status

        max_temp = None
        max_label = "unknown"
        for chip_name, entries in temps.items():
            for entry in entries:
                if entry.current is None:
                    continue
                if max_temp is None or entry.current > max_temp:
                    max_temp = entry.current
                    max_label = entry.label or chip_name

        if max_temp is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Temperature sensors present but no readable values"
            return status

        status.level = level_for(max_temp, self.temp_warn, self.temp_error)
        status.message = f"Max temperature {max_temp:.1f}C ({max_label})"
        status.values.append(KeyValue(key="max_temp_c", value=f"{max_temp:.1f}"))
        status.values.append(KeyValue(key="max_temp_sensor", value=max_label))
        return status

    def _gpu_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "system_monitor: GPU"
        status.hardware_id = "gpu"

        if not self.gpu_enabled:
            status.level = DiagnosticStatus.OK
            status.message = "GPU monitoring disabled (gpu_enabled=false)"
            return status

        if self._nvidia_smi_path is None:
            status.level = DiagnosticStatus.WARN
            status.message = "nvidia-smi not found -- GPU monitoring unavailable"
            return status

        try:
            output = subprocess.check_output(
                [self._nvidia_smi_path,
                 "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
                 "--format=csv,noheader,nounits"],
                timeout=2.0,
            ).decode().strip()
            util_str, mem_used_str, mem_total_str, temp_str = [
                v.strip() for v in output.split(",")]
            util = float(util_str)
            mem_used = float(mem_used_str)
            mem_total = float(mem_total_str)
            gpu_temp = float(temp_str)
        except (subprocess.SubprocessError, ValueError, OSError) as e:
            status.level = DiagnosticStatus.WARN
            status.message = f"Failed to query nvidia-smi: {e}"
            return status

        # Same warn/error thresholds as CPU/temperature for simplicity --
        # separate GPU-specific thresholds can be added to
        # diagnostics.yaml later if needed.
        level_from_util = level_for(util, self.cpu_warn, self.cpu_error)
        level_from_temp = level_for(gpu_temp, self.temp_warn, self.temp_error)
        status.level = max(level_from_util, level_from_temp)
        status.message = f"GPU util {util:.0f}%, {gpu_temp:.0f}C"
        status.values.append(KeyValue(key="utilization_percent", value=f"{util:.0f}"))
        status.values.append(KeyValue(key="memory_used_mb", value=f"{mem_used:.0f}"))
        status.values.append(KeyValue(key="memory_total_mb", value=f"{mem_total:.0f}"))
        status.values.append(KeyValue(key="temperature_c", value=f"{gpu_temp:.0f}"))
        return status

    def _network_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "system_monitor: Network"
        status.hardware_id = "network"

        try:
            stats = psutil.net_if_stats()
        except Exception as e:  # noqa: BLE001 - platform-dependent psutil failures
            status.level = DiagnosticStatus.WARN
            status.message = f"Could not read network interfaces: {e}"
            return status

        up_interfaces = [
            name for name, s in stats.items() if s.isup and name != "lo"
        ]

        if not up_interfaces:
            status.level = DiagnosticStatus.ERROR
            status.message = "No active (non-loopback) network interfaces"
            return status

        counters = psutil.net_io_counters()
        status.level = DiagnosticStatus.OK
        status.message = f"{len(up_interfaces)} interface(s) up: {', '.join(up_interfaces)}"
        status.values.append(KeyValue(key="up_interfaces", value=",".join(up_interfaces)))
        status.values.append(KeyValue(key="bytes_sent", value=str(counters.bytes_sent)))
        status.values.append(KeyValue(key="bytes_recv", value=str(counters.bytes_recv)))
        return status

    # ------------------------------------------------------------------
    def _publish_diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(self._cpu_status())
        array.status.append(self._ram_status())
        array.status.append(self._temperature_status())
        array.status.append(self._gpu_status())
        array.status.append(self._network_status())

        self.diag_pub.publish(array)


def main():
    rclpy.init()
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()