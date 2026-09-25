# forklift_interfaces/msg/SafetyStatus.msg
# Published by safety_manager_node on /safety/status.

uint8 STATE_SAFE=0
uint8 STATE_WARNING=1
uint8 STATE_SLOWDOWN=2
uint8 STATE_STOP=3
uint8 STATE_EMERGENCY=4
uint8 STATE_RESET=5

uint8 state
bool allow_motion
string message
