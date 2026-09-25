# forklift_interfaces/msg/CurrentHeight.msg
# Published by lift_feedback_node on /CurrentHeight.

float32 current_height   # meters
float32 velocity         # m/s, signed (+up / -down), from finite difference
bool moving
