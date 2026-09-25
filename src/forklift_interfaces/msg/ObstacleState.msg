# forklift_interfaces/msg/ObstacleState.msg
# Published by obstacle_monitor_node on /obstacle_state.

bool obstacle_detected
bool collision_detected
bool path_blocked
float32 distance
string source     # which sensor triggered: "lidar", "ultrasonic", "bumper"
string message
