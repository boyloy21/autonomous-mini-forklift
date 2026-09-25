# forklift_interfaces/msg/MissionStep.msg
# One step of a Mission. Values match forklift_interfaces/msg/Task's
# TASK_* constants exactly (0-4) so mission_executor_node can copy
# step_type straight into Task.task_type without translation.

uint8 STEP_PICK_PALLET=0
uint8 STEP_DROP_PALLET=1
uint8 STEP_GO_CHARGE=2
uint8 STEP_WAIT=3
uint8 STEP_RETURN_HOME=4

uint8 step_type
geometry_msgs/PoseStamped target_pose
string pallet_id
string station_id
float32 lift_height
float32 wait_duration
