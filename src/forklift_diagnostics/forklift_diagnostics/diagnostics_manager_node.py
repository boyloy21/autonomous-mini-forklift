"""Aggrgates all 4 subsystems statuses -> /diagnostics/status, and emits edge-triggered alerts on 
/diagnostics/error and /diagnostics/warning.

Alerts are edge-triggered (only sent when a device's level gets worse
than it was last tick) rather than resent every cycle, so a persistently
degraded device doesn't spam the alert topics.
"""

import rclpy 
from rclpy.node import Node
from forklift_interfaces.msg import (
    HardwareStatus, SystemStatus, SensorDiagnostics, CommunicationStatus,
    DiagnosticsStatus, DiagnosticsAlert, MaintenanceStatus
)

from forklift_diagnostics import utils

class DiagnosticsManagerNode(Node):
    
    def __init__(self):
        super().__init__('diagnostics_manager_node')
        rate = self.declare_parameter('check_rate_hz', 1.0).value
        
        self.hardware = None
        self.system = None
        self.sensor = None
        self.communication = None
        self.maintenance = None
        self._last_levels = {}
        
        self.create_subscription(HardwareStatus, '/hardware/status', self._set('hardware'), 10)
        self.create_subscription(SystemStatus, '/system/status', self._set('system'), 10)
        self.create_subscription(SensorDiagnostics, '/sensor/status', self._set('sensor'), 10)
        self.create_subscription(MaintenanceStatus, '/maintenance/status', self._set('maintenance'), 10)
        self.create_subscription(CommunicationStatus, '/communication/status', self._set('communication'), 10)
        
        self.status_pub = self.create_publisher(DiagnosticsStatus, '/diagnostics/status', 10)
        self.error_pub = self.create_publisher(DiagnosticsAlert, '/diagnostics/error', 10)
        self.warning_pub = self.create_publisher(DiagnosticsAlert, '/diagnostics/warning', 10)
 
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('diagnostics_manager_node ready')
        
    def _set(self, attr):
        def cb(msg):
            setattr(self, attr, msg)
            self._check_alerts(attr, msg)
        return cb
    
    def _check_alerts(self, source, msg):
        """Compare each named item's level against last-seen and emit an alert only on a worsening transition."""
        items = []
        if hasattr(msg, 'devices'):
            items = [(d.name, d.level, d.message) for d in msg.devices]
        elif hasattr(msg, 'links'):
            items = [(d.name, d.level, d.message) for d in msg.links]
        elif hasattr(msg, 'items'):
            # MaintenanceItem levels aren't OK/WARNING/ERROR; treat non-NONE as WARNING+.
            items = [(i.component, utils.OK if i.level == 0 else utils.WARNING, i.message)
                     for i in msg.items]
        
        for name, level, message in items:
            key = f'{source}.{name}'
            last = self._last_levels.get(key, utils.OK)
            if level > last:
                alert = DiagnosticsAlert()
                alert.source = key
                alert.code = utils.level_name(level)
                alert.message = message
                alert.stamp = self.get_clock().now().to_msg()
                (self.error_pub if level == utils.ERROR else self.warning_pub).publish(alert)
                self.get_logger().warn(f'{key}: {utils.level_name(last)} -> {utils.level_name(level)}')
                
            self._last_levels[key] = level
    
    def _tick(self):
        hardware_ok = self.hardware is None or self.hardware.overall_level != utils.ERROR
        sensor_ok = self.sensor is None or self.sensor.overall_level != utils.ERROR
        system_ok = self.system is None or self.system.overall_level != utils.ERROR
        comm_ok = self.communication is None or self.communication.overall_level != utils.ERROR
        maint_ok =  self.maintenance is None or self.maintenance.overall_level != utils.ERROR
        
        levels = [
            s.overall_level for s in 
            (self.hardware, self.system, self.sensor, self.communication, self.maintenance)
            if s is not None
        ]
        overall = max(levels) if levels else utils.WARNING 
        
        status = DiagnosticsStatus()
        status.overall_level = overall
        status.hardware_ok = hardware_ok
        status.sensor_ok = sensor_ok
        status.system_ok = system_ok
        status.communication_ok = comm_ok
        status.maintenance_ok = maint_ok
        status.message = f'Robot diagnostics: {utils.level_name(overall)}'
        self.status_pub.publish(status)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()