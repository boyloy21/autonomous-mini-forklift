# forklift_interfaces/msg/SensorDiagnostics.msg
# Published by sensor_monitor_node (forklift_diagnostics) on /sensor/status.
# Named SensorDiagnostics, not SensorStatus, to avoid colliding with
# forklift_safety's existing (simpler, bool-based) SensorStatus.msg.
# devices includes: lidar, camera, depth_camera, imu, ultrasonic, bumper,
# encoder, gps (optional).

DeviceStatus[] devices
uint8 overall_level
string message
