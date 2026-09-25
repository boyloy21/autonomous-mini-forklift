# forklift_interfaces/msg/SystemStatus.msg
# Published by system_monitor_node on /system/status.

float32 cpu_usage
float32 cpu_temperature
float32 gpu_usage
float32 gpu_temperature
float32 ram_usage
float32 disk_usage
float32 swap_usage
bool network_ok
DeviceStatus[] ros_graph   # per watched-node/topic presence checks
uint8 overall_level
string message
