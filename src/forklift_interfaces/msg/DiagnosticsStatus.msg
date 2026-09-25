# forklift_interfaces/msg/DiagnosticsStatus.msg
# Published by diagnostics_manager_node on /diagnostics/status - the
# aggregated overall robot health verdict.

uint8 overall_level   # DeviceStatus::OK / WARNING / ERROR
bool hardware_ok
bool sensor_ok
bool system_ok
bool communication_ok
bool maintenance_ok
string message
