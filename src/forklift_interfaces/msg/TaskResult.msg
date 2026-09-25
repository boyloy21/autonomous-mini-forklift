# forklift_interfaces/msg/TaskResult.msg

uint8 ERROR_NONE=0
uint8 ERROR_NAV_FAILED=1
uint8 ERROR_DOCK_FAILED=2
uint8 ERROR_LIFT_FAILED=3
uint8 ERROR_SAFETY_STOP=4
uint8 ERROR_TIMEOUT=5
uint8 ERROR_ABORTED=6

string task_id
bool success
uint8 error_code
string message
