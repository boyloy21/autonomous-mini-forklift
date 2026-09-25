# forklift_interfaces/msg/LiftLimitStatus.msg
# Published by lift_limit_node on /lift_limit.

bool top_limit             # hard limit reached at max travel
bool bottom_limit          # hard limit reached at min travel
bool soft_limit_exceeded   # inside the soft-limit margin, should slow/stop
string message
