#ifndef FORKLIFT_TASK_MANAGER__TASK_HPP_
#define FORKLIFT_TASK_MANAGER__TASK_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace forklift_task_manager
{

// The 6 task types a caller may request directly -- each is a complete,
// named operation with its own step recipe, fully owned by this package.
enum class TaskType
{
  SINGLE_NAVIGATION, PATROL_POINT,
  PICK_PALLET_DOCK, DROP_PALLET_DOCK, CHARGING_DOCK, STATION_DOCK,
  UNKNOWN
};

enum class TaskState
{
  IDLE, RECEIVED, VALIDATING, QUEUED, EXECUTING, MONITORING, COMPLETED,
  FAILED, CANCELED, TIMEOUT, SAFETY_STOP
};

// Primitive step types a Task expands into. MONITOR_CHARGING and
// STATION_OPERATION are the two "watch a condition / wait" steps that
// put the owning task into TaskState::MONITORING rather than EXECUTING
// while they run.
enum class StepType
{
  NAVIGATE, DOCK, LIFT, LOWER, UNDOCK, CHARGE_APPROACH, MONITOR_CHARGING, CHARGE_RELEASE,
  STATION_OPERATION
};

TaskType taskTypeFromString(const std::string & s);
std::string taskTypeToString(TaskType t);
std::string taskStateToString(TaskState s);
std::string stepTypeToString(StepType t);

struct StepParams
{
  geometry_msgs::msg::PoseStamped target_pose;
  std::string dock_id;
  double lift_height{0.0};
  double wait_duration{0.0};
  bool skip_internal_staging_nav{false};
};

struct Step
{
  StepType type;
  StepParams params;
  int retry_count{0};
  int max_retries{2};
};

struct GoalParams
{
  geometry_msgs::msg::PoseStamped goal_pose;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::string dock_id;
  double target_height{0.0};
  double station_wait_duration{0.0};
};

struct Task
{
  std::string task_id;
  TaskType type{TaskType::UNKNOWN};
  GoalParams goal_params;
  std::vector<Step> steps;
  size_t current_step_index{0};
  TaskState state{TaskState::IDLE};
  rclcpp::Time created_time;
};

// Expands task.type into task.steps -- the "recipe" for each of the 6
// task types. Returns false (with error_message set) on unknown
// task_type, missing required parameters, or dock_id not found in the
// database when one is required.
//
// NOTE on CHARGING_DOCK: NAVIGATE -> DOCK -> CHARGE_APPROACH ->
// MONITOR_CHARGING -> CHARGE_RELEASE. The camera is front-only while the
// charging contact is on the robot's rear: DOCK ends facing the charger,
// CHARGE_APPROACH turns around and reverses onto the pad, MONITOR_CHARGING
// waits for charging to start and then for a full battery, and
// CHARGE_RELEASE drives forward off the pad (the robot faces away from the
// charger, so UndockRobot -- which reverses -- must NOT be used here).
bool expandTaskToSteps(
  Task & task, const std::string & dock_database_path, double staging_x_offset,
  double staging_yaw_offset, std::string & error_message);

}  // namespace forklift_task_manager

#endif  // FORKLIFT_TASK_MANAGER__TASK_HPP_