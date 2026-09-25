# forklift_interfaces/msg/MaintenanceStatus.msg
# Published by maintenance_monitor_node on /maintenance/status.
# items includes: battery_health, motor_runtime, encoder_drift,
# camera_calibration, lidar_calibration, imu_calibration,
# wheel_calibration, firmware_version.

MaintenanceItem[] items
uint8 overall_level
string message
