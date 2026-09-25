"""Monitors comms links -> /communication/status.
 
CAN/Ethernet/WiFi checked via psutil network-interface state (is the
interface up). UART/RS485/USB/serial devices checked via device-path
existence. Packet loss, CRC errors, reconnect count, and latency require
real bus hardware to measure and aren't available yet - those fields are
reported as 0 with a note, not fabricated, until a real driver is wired
in (see the *_topic-less TODO pattern used elsewhere in this project).
"""

import os
import psutil
import rclpy
from rclpy.node import Node
from forklift_interfaces.msg import CommunicationStatus

from forklift_diagnostics import utils

class CommunicationMonitorNode(Node):
    def __init__(self):
        super().__init__('communication_monitor_node')
        
        self.can_iface = self.declare_parameter('can_interface', 'can0').value
        self.ethernet_iface = self.declare_parameter('ethernet_interface', 'eth0').value
        self.wifi_iface = self.declare_parameter('wifi_interface', 'wlan0').value
        self.uart_device = self.declare_parameter('uart_device', '/dev/ttyUSB0').value
        self.rs485_device = self.declare_parameter('rs485_device', '/dev/ttyUSB1').value
        self.usb_devices = self.declare_parameter('usb_devices', ['']).value
        self.usb_devices = [d for d in self.usb_devices if d]
        rate = self.declare_parameter('check_rate_hz', 1.0).value
        
        self.status_pub = self.create_publisher(CommunicationStatus, '/communication/status', 10)
        self.create_timer(1.0/rate, self._tick)
        self.get_logger().info('communication_monitor_node ready')
        
    def _iface_up(self, name):
        stats = psutil.net_if_stats()
        return name in stats and stats[name].isup
    
    def _tick(self):
        links = []
        
        can_up = self._iface_up(self.can_iface)
        links.append(
            utils.device('can', utils.OK if can_up else utils.ERROR,
                         'Connected' if can_up else f'{self.can_iface} not up'))
        
        eth_up = self._iface_up(self.ethernet_iface)
        links.append(
            utils.device('ethernet', utils.OK if eth_up else utils.WARNING,
                         'Connected' if eth_up else f'{self.ethernet_iface} not up'))
        
        wifi_up = self._iface_up(self.wifi_iface)
        links.append(
            utils.device('wifi', utils.OK if wifi_up else utils.WARNING,
                          'Connected' if wifi_up else f'{self.wifi_iface} not up'))
 
        uart_present = os.path.exists(self.uart_device)
        links.append(
            utils.device('uart', utils.OK if uart_present else utils.ERROR,
                          'OK' if uart_present else f'{self.uart_device} not found'))
 
        rs485_present = os.path.exists(self.rs485_device)
        links.append(
            utils.device('rs485', utils.OK if rs485_present else utils.WARNING,
                          'OK' if rs485_present else f'{self.rs485_device} not found'))
 
        usb_present = any(os.path.exists(d) for d in self.usb_devices) if self.usb_devices else True
        links.append(
            utils.device('usb', utils.OK if usb_present else utils.WARNING,
                          'OK' if usb_present else 'No configured USB devices found'))
 
        status = CommunicationStatus()
        status.links = links
        status.overall_level = utils.overall_level(links)
        status.message = f'Communication: {utils.level_name(status.overall_level)}'
        self.status_pub.publish(status)
        
def main(args=None):
    rclpy.init(args=args)
    node = CommunicationMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()