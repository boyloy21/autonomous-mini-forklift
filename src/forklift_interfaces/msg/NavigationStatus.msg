# forklift_interfaces/msg/NavigationStatus.msg
# Expected on /navigation/status from forklift_navigation (not yet
# published by anything in this project - navigation_bridge_node
# subscribes defensively; this topic will simply stay silent until
# forklift_navigation is extended to publish it).

string request_id
uint8 STATE_NAVIGATING=0
uint8 STATE_SUCCEEDED=1
uint8 STATE_FAILED=2
uint8 STATE_CANCELLED=3
uint8 state
float32 progress
string message
