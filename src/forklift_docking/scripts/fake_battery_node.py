#!/usr/bin/env python3
"""
fake_battery_node.py

Simulated battery for one robot (Gazebo has no battery model wired up
here). Publishes sensor_msgs/BatteryState on `battery_state` -- relative,
so it lands in the robot's namespace (e.g. /forklift1/battery_state),
where forklift_task_manager's MONITOR_CHARGING step reads it.

  * Charging: while THIS robot's body touches a charger pad. Contacts come
    from `charger_contact` (ros_gz_interfaces/Contacts, every charger pad's
    sensor bridged into this robot's namespace, e.g. /forklift1/charger_contact). A
    contact counts only when one side is a charger's
    "dock_contact_collision" and the other belongs to the Gazebo model
    `robot_name` -- same rule as forklift_docking's RearChargingManeuver.
    percentage rises by `charge_rate` per second, current = +charge_current.
  * Full: percentage reaches 1.0 -> status FULL, current 0.
  * Otherwise: discharging by `discharge_rate` per second,
    current = -discharge_current.

Percentages follow the BatteryState convention: 0.0 - 1.0.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from ros_gz_interfaces.msg import Contacts

CHARGER_PAD_COLLISION = "dock_contact_collision"


class FakeBatteryNode(Node):
    def __init__(self):
        super().__init__("fake_battery_node")

        default_name = self.get_namespace().strip("/") or "forklift"
        self.robot_name = self.declare_parameter("robot_name", default_name).value
        contact_topic = self.declare_parameter("contact_topic", "charger_contact").value
        battery_topic = self.declare_parameter("battery_topic", "battery_state").value
        self.percentage = float(self.declare_parameter("initial_percentage", 0.30).value)
        self.charge_rate = float(self.declare_parameter("charge_rate", 0.1).value)            # /s
        self.discharge_rate = float(self.declare_parameter("discharge_rate", 0.0002).value)    # /s
        self.charge_current = float(self.declare_parameter("charge_current", 2.0).value)       # A
        self.discharge_current = float(self.declare_parameter("discharge_current", 0.5).value)  # A
        self.capacity = float(self.declare_parameter("capacity", 5.0).value)                   # Ah
        self.full_voltage = float(self.declare_parameter("full_voltage", 12.6).value)
        self.empty_voltage = float(self.declare_parameter("empty_voltage", 10.5).value)
        # A robot resting against the pad may not report contact every
        # frame; contacts younger than this still count as "on the charger".
        self.contact_timeout = float(self.declare_parameter("contact_timeout", 1.0).value)
        publish_rate = float(self.declare_parameter("publish_rate", 2.0).value)

        self.last_contact_time = None
        self.last_update = self.get_clock().now()
        self.was_charging = False

        self.pub = self.create_publisher(BatteryState, battery_topic, 10)
        self.create_subscription(Contacts, contact_topic, self.on_contacts, 10)
        self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info(
            f"Fake battery for '{self.robot_name}': {self.percentage * 100:.0f}% start, "
            f"charges {self.charge_rate * 100:.2f}%/s on '{contact_topic}' contact, "
            f"publishing '{battery_topic}'")

    def is_own_charger_contact(self, pad, robot):
        return CHARGER_PAD_COLLISION in pad and robot.startswith(self.robot_name + "::")

    def on_contacts(self, msg):
        for c in msg.contacts:
            a, b = c.collision1.name, c.collision2.name
            if self.is_own_charger_contact(a, b) or self.is_own_charger_contact(b, a):
                self.last_contact_time = self.get_clock().now()
                return

    def on_charger(self, now):
        return (self.last_contact_time is not None and
                (now - self.last_contact_time).nanoseconds * 1e-9 < self.contact_timeout)

    def update(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_update).nanoseconds * 1e-9)
        self.last_update = now

        msg = BatteryState()
        charging = self.on_charger(now)
        if charging and self.percentage < 1.0:
            self.percentage = min(1.0, self.percentage + self.charge_rate * dt)
            msg.current = self.charge_current
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif charging:
            msg.current = 0.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            self.percentage = max(0.0, self.percentage - self.discharge_rate * dt)
            msg.current = -self.discharge_current
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        if charging != self.was_charging:
            self.get_logger().info(
                f"{'On' if charging else 'Off'} charger at {self.percentage * 100:.1f}%")
            self.was_charging = charging

        msg.header.stamp = now.to_msg()
        msg.percentage = self.percentage
        msg.voltage = self.empty_voltage + (self.full_voltage - self.empty_voltage) * self.percentage
        msg.capacity = self.capacity
        msg.design_capacity = self.capacity
        msg.charge = self.capacity * self.percentage
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.present = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeBatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
