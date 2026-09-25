#include "forklift_task_manager/task.hpp"
#include "forklift_task_manager/dock_database.hpp"

#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace forklift_task_manager
{

TaskType taskTypeFromString(const std::string & s)
{
  if (s == "SINGLE_NAVIGATION") {return TaskType::SINGLE_NAVIGATION;}
  if (s == "PATROL_POINT") {return TaskType::PATROL_POINT;}
  if (s == "PICK_PALLET_DOCK") {return TaskType::PICK_PALLET_DOCK;}
  if (s == "DROP_PALLET_DOCK") {return TaskType::DROP_PALLET_DOCK;}
  if (s == "CHARGING_DOCK") {return TaskType::CHARGING_DOCK;}
  if (s == "STATION_DOCK") {return TaskType::STATION_DOCK;}
  return TaskType::UNKNOWN;
}

std::string taskTypeToString(TaskType t)
{
  switch (t) {
    case TaskType::SINGLE_NAVIGATION: return "SINGLE_NAVIGATION";
    case TaskType::PATROL_POINT: return "PATROL_POINT";
    case TaskType::PICK_PALLET_DOCK: return "PICK_PALLET_DOCK";
    case TaskType::DROP_PALLET_DOCK: return "DROP_PALLET_DOCK";
    case TaskType::CHARGING_DOCK: return "CHARGING_DOCK";
    case TaskType::STATION_DOCK: return "STATION_DOCK";
    default: return "UNKNOWN";
  }
}

std::string taskStateToString(TaskState s)
{
  switch (s) {
    case TaskState::IDLE: return "IDLE";
    case TaskState::RECEIVED: return "RECEIVED";
    case TaskState::VALIDATING: return "VALIDATING";
    case TaskState::QUEUED: return "QUEUED";
    case TaskState::EXECUTING: return "EXECUTING";
    case TaskState::MONITORING: return "MONITORING";
    case TaskState::COMPLETED: return "COMPLETED";
    case TaskState::FAILED: return "FAILED";
    case TaskState::CANCELED: return "CANCELED";
    case TaskState::TIMEOUT: return "TIMEOUT";
    case TaskState::SAFETY_STOP: return "SAFETY_STOP";
    default: return "UNKNOWN";
  }
}

std::string stepTypeToString(StepType t)
{
  switch (t) {
    case StepType::NAVIGATE: return "NAVIGATE";
    case StepType::DOCK: return "DOCK";
    case StepType::LIFT: return "LIFT";
    case StepType::LOWER: return "LOWER";
    case StepType::UNDOCK: return "UNDOCK";
    case StepType::CHARGE_APPROACH: return "CHARGE_APPROACH";
    case StepType::CHARGE_RELEASE: return "CHARGE_RELEASE";
    case StepType::MONITOR_CHARGING: return "MONITOR_CHARGING";
    case StepType::STATION_OPERATION: return "STATION_OPERATION";
    default: return "UNKNOWN";
  }
}

namespace
{
geometry_msgs::msg::PoseStamped computeStagingPose(
  const DockInstance & dock, double staging_x_offset, double staging_yaw_offset)
{
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = dock.frame;

  const double yaw = dock.theta + staging_yaw_offset;
  staging.pose.position.x = dock.x + staging_x_offset * std::cos(dock.theta);
  staging.pose.position.y = dock.y + staging_x_offset * std::sin(dock.theta);
  staging.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  staging.pose.orientation = tf2::toMsg(q);
  return staging;
}

bool lookupDock(
  const std::string & dock_id, const std::string & dock_database_path,
  DockInstance & out, std::string & error_message)
{
  static rclcpp::Logger logger = rclcpp::get_logger("task_expansion");
  DockDatabase db(logger);
  if (!db.loadFromFile(dock_database_path)) {
    error_message = "Could not load dock database";
    return false;
  }
  auto maybe = db.getDock(dock_id);
  if (!maybe) {
    error_message = "dock_id '" + dock_id + "' not found in database";
    return false;
  }
  out = *maybe;
  return true;
}

// Common NAVIGATE(staging) + DOCK(skip_staging=true) pair shared by all
// four dock-based task types.
void pushNavigateAndDock(Task & task, const DockInstance & dock, double staging_x_offset, double staging_yaw_offset)
{
  Step nav_step;
  nav_step.type = StepType::NAVIGATE;
  nav_step.params.target_pose = computeStagingPose(dock, staging_x_offset, staging_yaw_offset);
  task.steps.push_back(nav_step);

  Step dock_step;
  dock_step.type = StepType::DOCK;
  dock_step.params.dock_id = dock.dock_id;
  dock_step.params.skip_internal_staging_nav = true;
  task.steps.push_back(dock_step);
}
}  // namespace

bool expandTaskToSteps(
  Task & task, const std::string & dock_database_path, double staging_x_offset,
  double staging_yaw_offset, std::string & error_message)
{
  task.steps.clear();

  switch (task.type) {
    case TaskType::SINGLE_NAVIGATION: {
      Step step;
      step.type = StepType::NAVIGATE;
      step.params.target_pose = task.goal_params.goal_pose;
      task.steps.push_back(step);
      return true;
    }

    case TaskType::PATROL_POINT: {
      if (task.goal_params.waypoints.empty()) {
        error_message = "PATROL_POINT task requires at least one waypoint";
        return false;
      }
      for (const auto & wp : task.goal_params.waypoints) {
        Step step;
        step.type = StepType::NAVIGATE;
        step.params.target_pose = wp;
        task.steps.push_back(step);
      }
      return true;
    }

    case TaskType::PICK_PALLET_DOCK: {
      if (task.goal_params.dock_id.empty()) {
        error_message = "PICK_PALLET_DOCK task requires dock_id";
        return false;
      }
      DockInstance dock;
      if (!lookupDock(task.goal_params.dock_id, dock_database_path, dock, error_message)) {
        return false;
      }
      pushNavigateAndDock(task, dock, staging_x_offset, staging_yaw_offset);

      Step lift_step;
      lift_step.type = StepType::LIFT;
      lift_step.params.lift_height =
        task.goal_params.target_height > 0.0 ? task.goal_params.target_height : dock.lift_height;
      task.steps.push_back(lift_step);

      Step undock_step;
      undock_step.type = StepType::UNDOCK;
      undock_step.params.dock_id = task.goal_params.dock_id;
      task.steps.push_back(undock_step);
      return true;
    }

    case TaskType::DROP_PALLET_DOCK: {
      if (task.goal_params.dock_id.empty()) {
        error_message = "DROP_PALLET_DOCK task requires dock_id";
        return false;
      }
      DockInstance dock;
      if (!lookupDock(task.goal_params.dock_id, dock_database_path, dock, error_message)) {
        return false;
      }
      pushNavigateAndDock(task, dock, staging_x_offset, staging_yaw_offset);

      Step lower_step;
      lower_step.type = StepType::LOWER;
      lower_step.params.lift_height = dock.lift_height;  // typically 0.0, release height
      task.steps.push_back(lower_step);

      Step undock_step;
      undock_step.type = StepType::UNDOCK;
      undock_step.params.dock_id = task.goal_params.dock_id;
      task.steps.push_back(undock_step);
      return true;
    }

    case TaskType::CHARGING_DOCK: {
      if (task.goal_params.dock_id.empty()) {
        error_message = "CHARGING_DOCK task requires dock_id";
        return false;
      }
      DockInstance dock;
      if (!lookupDock(task.goal_params.dock_id, dock_database_path, dock, error_message)) {
        return false;
      }
      pushNavigateAndDock(task, dock, staging_x_offset, staging_yaw_offset);

      // Rear contact: turn around and reverse onto the pad. Never retried
      // -- a retry after a partial maneuver would turn the robot again.
      Step approach_step;
      approach_step.type = StepType::CHARGE_APPROACH;
      approach_step.params.dock_id = dock.dock_id;
      approach_step.max_retries = 0;
      task.steps.push_back(approach_step);

      // Charging started, then battery full (see the NOTE in task.hpp).
      Step monitor_step;
      monitor_step.type = StepType::MONITOR_CHARGING;
      task.steps.push_back(monitor_step);

      // Undock = drive forward off the pad (never UndockRobot here).
      Step release_step;
      release_step.type = StepType::CHARGE_RELEASE;
      release_step.params.dock_id = dock.dock_id;
      release_step.max_retries = 0;
      task.steps.push_back(release_step);
      return true;
    }

    case TaskType::STATION_DOCK: {
      if (task.goal_params.dock_id.empty()) {
        error_message = "STATION_DOCK task requires dock_id";
        return false;
      }
      DockInstance dock;
      if (!lookupDock(task.goal_params.dock_id, dock_database_path, dock, error_message)) {
        return false;
      }
      pushNavigateAndDock(task, dock, staging_x_offset, staging_yaw_offset);

      Step station_step;
      station_step.type = StepType::STATION_OPERATION;
      station_step.params.wait_duration =
        task.goal_params.station_wait_duration > 0.0 ? task.goal_params.station_wait_duration : 5.0;
      task.steps.push_back(station_step);

      Step undock_step;
      undock_step.type = StepType::UNDOCK;
      undock_step.params.dock_id = task.goal_params.dock_id;
      task.steps.push_back(undock_step);
      return true;
    }

    default:
      error_message = "Unknown task_type";
      return false;
  }
}

}  // namespace forklift_task_manager