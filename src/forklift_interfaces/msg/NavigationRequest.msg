# forklift_interfaces/msg/NavigationRequest.msg
# Published by task_executor_node on /execute_navigation.
# navigation_client_node translates this into a NavigateToPose or
# FollowWaypoints action goal.

string task_id
geometry_msgs/PoseStamped target_pose
bool use_waypoints
geometry_msgs/PoseStamped[] waypoints
