# forklift_interfaces/msg/CommunicationStatus.msg
# Published by communication_monitor_node on /communication/status.
# links includes: can, rs485, uart, ethernet, wifi, usb, and any
# configured serial devices.

DeviceStatus[] links
uint8 overall_level
string message
