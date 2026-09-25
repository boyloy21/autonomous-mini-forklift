# TaskManagerStatus.msg
#
# Published periodically on /task_manager/status for anyone observing
# (dashboards, forklift_mission_manager) without needing to be the
# specific client that submitted the current task.
std_msgs/Header header
bool has_current_task
string current_task_id
string current_task_type
string current_state
int32 queue_length
