# forklift_interfaces/msg/MaintenanceItem.msg

uint8 LEVEL_NONE=0
uint8 LEVEL_CALIBRATION_REQUIRED=1
uint8 LEVEL_SERVICE_REQUIRED=2
uint8 LEVEL_REPLACE_REQUIRED=3

string component
uint8 level
string message
float32 value   # e.g. battery health %, runtime hours, drift magnitude
