# forklift_interfaces/msg/LiftRequest.msg
# Published by task_executor_node on /execute_lift.
# lift_client_node translates this into a call to /lift_up, /lift_down,
# or /set_height depending on target_height vs current height.

string task_id
float32 target_height
float32 speed
