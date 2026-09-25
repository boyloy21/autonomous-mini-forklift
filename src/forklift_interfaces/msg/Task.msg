# forklift_interfaces/msg/Task.msg
# Represents a single mission request in the new task-based workflow:
#   Pick Pallet -> Drop Pallet -> Go Charge -> Wait -> Return Home

uint8 TASK_PICK_PALLET=0
uint8 TASK_DROP_PALLET=1
uint8 TASK_GO_CHARGE=2
uint8 TASK_WAIT=3
uint8 TASK_RETURN_HOME=4

string task_id                     # unique ID assigned by task_scheduler_node
uint8 task_type                    # one of the TASK_* constants above
uint8 priority                     # higher = more urgent, used by task_queue_node

geometry_msgs/PoseStamped target_pose   # where to navigate to (station, shelf, charger, home)
string pallet_id                  # relevant for PICK/DROP tasks (aruco/pallet id)
string station_id                 # relevant for DOCK/CHARGE tasks
float32 lift_height               # target lift height for pick/drop (meters)
float32 wait_duration             # seconds, relevant for TASK_WAIT

builtin_interfaces/Time created_at
