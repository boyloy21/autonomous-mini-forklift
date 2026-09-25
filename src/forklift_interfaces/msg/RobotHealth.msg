# forklift_interfaces/msg/RobotHealth.msg
# Published by diagnostics_publisher_node on /robot_health - single
# consolidated message for the dashboard.

uint8 overall_status   # DeviceStatus::OK / WARNING / ERROR
float32 battery_percent
float32 cpu_usage
float32 gpu_usage
float32 ram_usage
uint8 lidar_status
uint8 camera_status
uint8 imu_status
uint8 can_status
uint8 uart_status
uint8 rs485_status
string temperature_summary
string maintenance_summary
string message
