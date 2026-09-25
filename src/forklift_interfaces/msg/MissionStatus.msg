# forklift_interfaces/msg/MissionStatus.msg
# Published by mission_monitor_node on /mission/status.

uint8 STATE_PENDING=0
uint8 STATE_EXECUTING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_FAILED=3
uint8 STATE_CANCELLED=4

string mission_id
uint8 state
uint32 current_step
uint32 total_steps
float32 progress
string message
