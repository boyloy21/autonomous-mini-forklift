# forklift_interfaces/msg/DeviceStatus.msg
# Shared building block used inside HardwareStatus, SensorDiagnostics,
# CommunicationStatus: one line item with a level, name, message, and an
# optional numeric reading (voltage, temp, usage %, latency, etc).

uint8 OK=0
uint8 WARNING=1
uint8 ERROR=2

string name
uint8 level
string message
float32 value
