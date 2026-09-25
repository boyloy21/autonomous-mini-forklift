"""Collects hardware/system/sensor/communication/maintenance status and
publishes one consolidated /robot_health message for the dashboard.

battery_percent is read from maintenance_monitor_node's battery_health
item (the only "percent" figure this project currently tracks for the
battery - it's a health estimate, not a live state-of-charge reading;
swap in a real SoC source once the BMS bridge exists).
"""

import rclpy
from rclpy.node import Node
from forklift_interfaces.msg import (
    HardwareStatus, SystemStatus, SensorDiagnostics, CommunicationStatus,
    MaintenanceStatus, RobotHealth,
)

from forklift_diagnostics import utils


def _find(items, name, attr='name'):
    for item in items:
        if getattr(item, attr) == name:
            return item
    return None


class DiagnosticsPublisherNode(Node):

    def __init__(self):
        super().__init__('diagnostics_publisher_node')
        rate = self.declare_parameter('publish_rate_hz', 1.0).value

        self.hardware = None
        self.system = None
        self.sensor = None
        self.communication = None
        self.maintenance = None

        self.create_subscription(HardwareStatus, '/hardware/status', self._set('hardware'), 10)
        self.create_subscription(SystemStatus, '/system/status', self._set('system'), 10)
        self.create_subscription(SensorDiagnostics, '/sensor/status', self._set('sensor'), 10)
        self.create_subscription(
            CommunicationStatus, '/communication/status', self._set('communication'), 10)
        self.create_subscription(
            MaintenanceStatus, '/maintenance/status', self._set('maintenance'), 10)

        self.health_pub = self.create_publisher(RobotHealth, '/robot_health', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('diagnostics_publisher_node ready')

    def _set(self, attr):
        def cb(msg):
            setattr(self, attr, msg)
        return cb

    def _temperature_summary(self):
        hot = []
        if self.hardware:
            bt = _find(self.hardware.devices, 'battery_temperature')
            if bt and bt.level != utils.OK:
                hot.append(f'battery {bt.value:.0f}C')
        if self.system:
            if self.system.cpu_temperature and self.system.overall_level != utils.OK:
                hot.append(f'cpu {self.system.cpu_temperature:.0f}C')
        return 'Normal' if not hot else 'High: ' + ', '.join(hot)

    def _maintenance_summary(self):
        if not self.maintenance:
            return 'Unknown'
        flagged = [i.component for i in self.maintenance.items if i.level != 0]
        return 'None' if not flagged else ', '.join(flagged)

    def _tick(self):
        health = RobotHealth()

        if self.maintenance:
            bh = _find(self.maintenance.items, 'battery_health', attr='component')
            health.battery_percent = bh.value if bh else 0.0
        if self.system:
            health.cpu_usage = self.system.cpu_usage
            health.gpu_usage = self.system.gpu_usage
            health.ram_usage = self.system.ram_usage

        if self.sensor:
            for field, name in (('lidar_status', 'lidar'), ('camera_status', 'camera'),
                                 ('imu_status', 'imu')):
                d = _find(self.sensor.devices, name)
                setattr(health, field, d.level if d else utils.ERROR)
        if self.communication:
            for field, name in (('can_status', 'can'), ('uart_status', 'uart'),
                                 ('rs485_status', 'rs485')):
                d = _find(self.communication.links, name)
                setattr(health, field, d.level if d else utils.ERROR)

        health.temperature_summary = self._temperature_summary()
        health.maintenance_summary = self._maintenance_summary()

        levels = [
            s.overall_level for s in
            (self.hardware, self.system, self.sensor, self.communication, self.maintenance)
            if s is not None
        ]
        health.overall_status = max(levels) if levels else utils.WARNING
        health.message = f'Robot health: {utils.level_name(health.overall_status)}'
        self.health_pub.publish(health)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()