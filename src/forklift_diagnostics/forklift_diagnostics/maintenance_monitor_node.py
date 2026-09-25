"""Tracks calibration age, runtime hours, battery health, firmware
version -> /maintenance/status.
 
Calibration/runtime/battery-health figures come from config (maintenance.yaml)
since there's no live calibration-tracking subsystem yet in this project -
this node's job is the threshold/expiry logic, not measurement. Wire real
"last calibrated" timestamps in from whichever node actually performs
each calibration once those exist.
"""

import rclpy
from rclpy.node import Node
from forklift_interfaces.msg import MaintenanceStatus, MaintenanceItem
 
from forklift_diagnostics import utils

LEVEL_NONE = MaintenanceItem.LEVEL_NONE
LEVEL_CAL = MaintenanceItem.LEVEL_CALIBRATION_REQUIRED
LEVEL_SERVICE = MaintenanceItem.LEVEL_SERVICE_REQUIRED
LEVEL_REPLACE = MaintenanceItem.LEVEL_REPLACE_REQUIRED


def _item(component, level, message='', value=0.0):
    i = MaintenanceItem()
    i.component = component
    i.level = level
    i.message = message
    i.value = float(value)
    return i

def _calibration_item(name, last_calibrated_days_ago, interval_days):
    overdue = last_calibrated_days_ago >= interval_days
    level = LEVEL_CAL if overdue else LEVEL_NONE
    msg = ('Calibration required' if overdue else
           f'OK ({last_calibrated_days_ago:.0f}/{interval_days:.0f} days)')
    return _item(name, level, msg, last_calibrated_days_ago)

class MaintenanceMonitorNode(Node):
    def __init__(self):
        super().__init__('maintenance_monitor_node')
        
        self.battery_health = self.declare_parameter('battery_health_percent', 92.0).value
        self.battery_replace_threshold = \
            self.declare_parameter('battery_replace_threshold', 70.0).value
            
        self.cal_params = {
            'camera_calibration': (
                self.declare_parameter('camera_last_calibrated_days_ago', 5.0).value,
                self.declare_parameter('camera_calibration_interval_days', 30.0).value),
            'lidar_calibration': (
                self.declare_parameter('lidar_last_calibrated_days_ago', 10.0).value,
                self.declare_parameter('lidar_calibration_interval_days', 60.0).value),
            'imu_calibration': (
                self.declare_parameter('imu_last_calibrated_days_ago', 3.0).value,
                self.declare_parameter('imu_calibration_interval_days', 14.0).value),
            'wheel_calibration': (
                self.declare_parameter('wheel_last_calibrated_days_ago', 20.0).value,
                self.declare_parameter('wheel_calibration_interval_days', 90.0).value),
        }
        
        self.motor_runtime_hours = self.declare_parameter('motor_runtime_hours', 120.0).value
        self.motor_service_interval_hours = \
            self.declare_parameter('motor_service_interval_hours', 500.0).value
 
        self.firmware_version = self.declare_parameter('firmware_version', '1.0.0').value
        self.firmware_expected_version = \
            self.declare_parameter('firmware_expected_version', '1.0.0').value
 
        rate = self.declare_parameter('check_rate_hz', 0.1).value
 
        self.status_pub = self.create_publisher(MaintenanceStatus, '/maintenance/status', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('maintenance_monitor_node ready')
        
    def _tick(self):
        items = []
        
        battery_level = (
            LEVEL_REPLACE if self.self.battery_health <= self.self.battery_replace_threshold else LEVEL_NONE
        )
        items.append(_item(
            'battery_health', battery_level,
            f'{self.battery_health:.0f}%' + (' - replace required' if battery_level == LEVEL_REPLACE else ''),
            self.battery_health))
        
        for name, (days_ago, interval) in self.call_params.item():
            items.append(_calibration_item(name, days_ago, interval))
            
        runtime_level = (
            LEVEL_SERVICE if self.self.motor_runtime_hours >= self.self.motor_service_interval_hours 
            else LEVEL_NONE
        )
        
        items.append(_item(
            'motor_runtime', runtime_level,
            f'{self.motor_runtime_hours:.0f}h' + (' - service required' if runtime_level == LEVEL_SERVICE else ''),
            self.motor_runtime_hours
        ))
        
        # No Live drift measurement source yet - reported as informational
        items.append(_item('encoder_drift', LEVEL_NONE, 'Not yet measured'))
        
        firmware_level = (
            LEVEL_NONE if self.self.firmware_version == self.self.firmware_expected_version else LEVEL_SERVICE
        )
        
        items.append(_item(
            'firmware_version', firmware_level,
            f'{self.firmware_version} (expected {self.firmware_expected_version})'))
        
        status = MaintenanceStatus()
        status.items = items
        # Map MaintenanceItem levels onto DeviceStatus OK/WARNING/ERROR:
        # NONE->OK, CALIBRATION_REQUIRED->WARNING, SERVICE_REQUIRED->WARNING,
        # REPLACE_REQUIRED->ERROR.
        worst = max((i.level for i in items), default=LEVEL_NONE)
        status.overall_level = {
            LEVEL_NONE: utils.OK,
            LEVEL_CAL: utils.WARNING,
            LEVEL_SERVICE: utils.WARNING,
            LEVEL_REPLACE: utils.ERROR,
        }[worst]
        status.message = f'Maintenance: {utils.level_name(status.overall_level)}'
        self.status_pub.publish(status)
        
def main(args=None):
    rclpy.init(args=args)
    node = MaintenanceMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()