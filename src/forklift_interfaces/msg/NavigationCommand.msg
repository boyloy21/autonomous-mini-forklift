# forklift_interfaces/msg/NavigationCommand.msg
# Shared input funnel for navigation_bridge_node - published by
# fastapi_bridge_node, mqtt_bridge_node, or directly by a roslibjs client
# via rosbridge_server, on /web_bridge/navigation_command.

uint8 CMD_GOTO=0
uint8 CMD_WAYPOINTS=1
uint8 CMD_CANCEL=2
uint8 CMD_PAUSE=3
uint8 CMD_RESUME=4

uint8 command
string request_id
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped[] waypoints
