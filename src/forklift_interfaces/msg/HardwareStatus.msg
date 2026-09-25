# forklift_interfaces/msg/HardwareStatus.msg
# Published by hardware_monitor_node on /hardware/status.
# devices includes: battery_voltage, battery_current, battery_temperature,
# motor_driver, left_motor, right_motor, lift_motor, encoder.

DeviceStatus[] devices
uint8 overall_level
string message
