# forklift_interfaces/msg/Mission.msg
# Published on /mission/request (by dashboard) and /mission/execute
# (scheduler -> executor, with mission_id assigned).

string mission_id
MissionStep[] steps
uint8 priority
builtin_interfaces/Time created_at
