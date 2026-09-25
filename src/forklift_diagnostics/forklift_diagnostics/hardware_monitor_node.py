"""Monitors battery, motor drivers, and encoder hardware -> /hardware/status
Expecte raw reading on Float32 topics from a BMS/motor-driver bridge (not yet built elesewhere in this project) - if  a topic never publishes,
that device reports ERROR ("no data") rather than silently defaulting to OK. Rename the *_topic parameters in hardware.yaml once a real bridge
exists, or leave as-is and just wire that bridge to publish these names.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from forklift_interfaces.msg import HardwareStatus, LiftDriverStatus

from forklift_diagnostics import utils

class HardwareMonitorNode(Node):
    
    def __init__(self):
        super().__init__('hardware_monitor_node')
        
        self.stale_timeout = self.declare_parameter('stale_timeout_sec', 3.0).value
        rate = self.declare_parameter('check_rate_hz', 2.0).value
        
        self.thresholds = {
            'battery_voltage': (
                self.declare_parameter('battery_voltage_warn_low', 22.0).value,
                self.declare_parameter('battery_voltage_error_low', 20.0).value, True),
            'battery_current': (
                self.declare_parameter('battery_current_warn', 25.0).value,
                self.declare_parameter('battery_current_error', 35.0).value, False),
            'battery_temperature': (
                self.declare_parameter('battery_temp_warn', 45.0).value,
                self.declare_parameter('battery_temp_error', 60.0).value, False),
            'left_motor': (
                self.declare_parameter('motor_temp_warn', 60.0).value,
                self.declare_parameter('motor_temp_error', 80.0).value, False),
            'right_motor': (
                self.declare_parameter('motor_temp_warn').value,
                self.declare_parameter('motor_temp_error').value, False),
        }
        
        self.last_seen = {}
        self.last_value = {}
        
        raw_topics = {
            'battery_voltage': '/hardware/battery_voltage',
            'battery_current': '/hardware/battery_current',
            'battery_temperature': '/hardware/battery_temperature',
            'left_motor': '/hardware/left_motor_temperature',
            'right_motor': '/hardware/right_motor_temperature', 
        }
        self._subs = []
        for name, topic in raw_topics.items():
            self._subs.append(
                self.create_subscription(Float32, topic, self._make_cb(name), 10))
            
        
        # Lift motor health piggybacks on forklift_controller's own status msg.
        self.lift_status = None
        self.create_subscription(LiftDriverStatus, '/lift_driver_status', self._lift_cb, 10)
        
        self.status_pub = self.create_publisher(HardwareStatus, '/hardware/status', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('hardware_monitor_node ready')
        
    def _make_cb(self, name):
        def cb(msg):
            self.last_seen[name] = self.get_clock().now()
            self.last_value[name] = msg.data
        return cb
    
    def _lift_cb(self, msg):
        self.lift_status = msg
        self.last_seen['lift_motor'] = self.get_clock().now()
        
    def _fresh(self, name):
        if name not in self.last_seen:
            return False
        age = (self.get_clock().now() - self.last_seen[name]).nanoseconds / 1e9
        return age <= self.stale_timeout
    
    def _tick(self):
        devices = []
        
        for name, (warn, error, lower_is_worse) in self.thresholds.items():
            if not self._fresh(name):
                devices.append(utils.device(name, utils.ERROR, 'No data / stale'))
                continue
            value = self.last_value[name]
            level = (utils.level_from_low_range(value, warn, error) if lower_is_worse
                     else utils.level_from_range(value, warn, error))
            devices.append(utils.device(name, level, '', value))
            
        if self._fresh('lift_motor') and self.lift_status is not None:
            level = utils.ERROR if self.lift_status.fault else utils.OK
            devices.append(
                utils.device('lift_motor', level, self.lift_status.message,
                             self.lift_status.motor_current))
            
        else: 
            devices.append(utils.device('lift_motor', utils.ERROR, 'No data / stale'))
            
        
         # motor_driver / encoder: no dedicated raw topic defined yet -
        # reported as a single combined "not yet integrated" WARNING so
        # the field exists in the message without pretending to health
        # data that isn't available.
        devices.append(utils.device('motor_driver', utils.WARNING, 'Not yet integrated'))
        devices.append(utils.device('encoder', utils.WARNING, 'Not yet integrated'))
        
        status = HardwareStatus()
        status.devices = devices
        status.overall_level = utils.overall_level(devices)
        status.message = f'Hardware: {utils.level_name(status.overall_level)}'
        self.status_pub.publish(status)
        
def main(args=None):
    rclpy.init(args=args)
    node = HardwareMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()    
            
            
            
                   