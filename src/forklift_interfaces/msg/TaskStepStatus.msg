# TaskStepStatus.msg
#
# Live, step-level view of what forklift_task_manager is doing, published
# on the `task_status` topic (namespaced per robot, e.g.
# /forklift1/task_status) every time a step or sub-step changes.
#
# ExecuteTask feedback only reaches the client that sent the goal
# (forklift_mission_manager), and TaskManagerStatus is a periodic summary
# with no step detail. This message is the observable, per-transition
# stream a dashboard needs: which dock operation is running, whether the
# dock marker was detected, whether the fork moved, whether it undocked,
# and whether the whole thing succeeded.

std_msgs/Header header

string task_id
# Task recipe being run:
#   SINGLE_NAVIGATION | PATROL_POINT | PICK_PALLET_DOCK | DROP_PALLET_DOCK |
#   CHARGING_DOCK | STATION_DOCK
string task_type

# Step, or docking sub-step. The plain step types come straight from
# StepType (NAVIGATE / DOCK / LIFT / LOWER / UNDOCK / MONITOR_CHARGING /
# STATION_OPERATION); the rest are reported from opennav_docking's own
# DockRobot feedback while a DOCK step runs:
#   STAGING          -- navigating to the dock's staging pose
#   DETECT           -- looking for the dock/ArUco marker
#   APPROACH         -- detected, driving onto the dock
#   WAIT_FOR_CHARGE  -- on the dock, waiting for charge to start
#   CONTACT          -- made contact with the dock
#   CHARGING         -- charging has started
#   UNDOCK_STAGING   -- back at the staging pose while undocking
# plus TASK, used for the task-level start/finish messages.
string step

# RUNNING | SUCCESS | FAILED | RETRY | CANCELED
string state

int32 step_index
int32 total_steps
float32 progress          # 0-100, over the task's steps

int32 attempt             # 1-based attempt number for this step
int32 max_attempts

string dock_id            # dock the step targets, when it targets one
float32 lift_height       # metres, for LIFT / LOWER steps
bool detected             # true once the dock was detected in this task
bool charging             # true once the robot reported charging on the dock
bool success              # task-level success (state SUCCESS on step TASK)
string error_code
string message
