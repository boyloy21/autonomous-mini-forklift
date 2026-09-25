# forklift_interfaces/msg/MissionCommand.msg
# Shared input funnel for mission_bridge_node, on /web_bridge/mission_command.

uint8 CMD_START=0
uint8 CMD_PAUSE=1
uint8 CMD_RESUME=2
uint8 CMD_CANCEL=3

uint8 command
Mission mission       # used for CMD_START
string mission_id     # used for CMD_CANCEL
