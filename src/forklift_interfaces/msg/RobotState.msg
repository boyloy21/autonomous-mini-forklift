# forklift_interfaces/msg/RobotState.msg
# Published by robot_state_node on /robot_state - single aggregated
# snapshot of the robot for external systems.

string robot_id
geometry_msgs/PoseStamped pose
geometry_msgs/Twist velocity
float32 battery_percent
float32 battery_voltage
uint8 health_level        # DeviceStatus::OK/WARNING/ERROR (from /robot_health)
string task_state
string mission_state
uint8 safety_state        # SafetyStatus::STATE_*
bool safety_allow_motion
builtin_interfaces/Time stamp
