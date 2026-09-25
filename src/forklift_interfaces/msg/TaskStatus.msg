# forklift_interfaces/msg/TaskStatus.msg
# Aggregated status of a task's execution, published by task_monitor_node

uint8 STATE_PENDING=0
uint8 STATE_QUEUED=1
uint8 STATE_EXECUTING=2
uint8 STATE_SUCCEEDED=3
uint8 STATE_FAILED=4
uint8 STATE_CANCELLED=5
uint8 STATE_PAUSED=6

string task_id
uint8 state
string current_step        # e.g. "navigating_to_pallet", "docking", "lifting", "charging"
float32 progress           # 0.0 - 1.0
string message
