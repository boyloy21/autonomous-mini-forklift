#include "forklift_mission_manager/mission_executor.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace forklift_mission_manager
{

MissionExecutor::MissionExecutor(rclcpp::Node * node, const std::string & execute_task_action_name)
: node_(node)
{
  task_client_ = rclcpp_action::create_client<ExecuteTask>(node_, execute_task_action_name);
}

MissionOutcome MissionExecutor::run(
  Mission & mission, const MissionFeedbackFn & feedback, const IsCancelingFn & is_canceling)
{
  MissionOutcome outcome;
  const int total_tasks = static_cast<int>(mission.tasks.size());

  for (int i = 0; i < total_tasks; ++i) {
    mission.current_task_index = static_cast<size_t>(i);

    if (is_canceling()) {
      outcome.success = false;
      outcome.message = "Canceled before task " + std::to_string(i);
      return outcome;
    }

    const auto & spec = mission.tasks[i];
    std::string error_message;

    const bool ok = executeTask(spec, i, total_tasks, feedback, is_canceling, error_message);

    if (!ok) {
      outcome.success = false;
      outcome.failed_task = spec.task_type;
      outcome.message = is_canceling() ?
        ("Canceled during task " + spec.task_type) :
        ("Task " + spec.task_type + " failed: " + error_message);
      return outcome;
    }
  }

  outcome.success = true;
  outcome.message = "Mission '" + mission.mission_id + "' completed successfully";
  return outcome;
}

bool MissionExecutor::executeTask(
  const TaskSpec & spec, int task_index, int total_tasks,
  const MissionFeedbackFn & feedback, const IsCancelingFn & is_canceling,
  std::string & error_message)
{
  if (!task_client_->wait_for_action_server(5s)) {
    error_message = "ExecuteTask action server not available";
    return false;
  }

  ExecuteTask::Goal goal;
  goal.task_type = spec.task_type;
  goal.goal_pose = spec.goal_pose;
  goal.waypoints = spec.waypoints;
  goal.dock_id = spec.dock_id;
  goal.target_height = spec.target_height;
  goal.station_wait_duration = spec.station_wait_duration;

  feedback(spec.task_type, task_index, total_tasks, "WAITING_TASK_RESULT", 0.0f,
    "Sending task " + spec.task_type);

  auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [&](
    rclcpp_action::ClientGoalHandle<ExecuteTask>::SharedPtr,
    const std::shared_ptr<const ExecuteTask::Feedback> task_feedback)
    {
      // Relay the task's own progress into mission-level feedback --
      // mission progress = (completed tasks + this task's own fractional
      // progress) / total tasks, giving a smooth overall percentage
      // rather than jumping in whole-task increments.
      const float overall_progress =
        100.0f * (static_cast<float>(task_index) + (task_feedback->progress / 100.0f)) /
        static_cast<float>(total_tasks);
      feedback(
        spec.task_type, task_index, total_tasks, "EXECUTING", overall_progress,
        task_feedback->message);
    };

  auto goal_handle_future = task_client_->async_send_goal(goal, send_goal_options);

  const auto send_start = node_->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {break;}
    if (is_canceling()) {return false;}
    if ((node_->now() - send_start).seconds() > 10.0) {
      error_message = "Timed out waiting for ExecuteTask goal acceptance";
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_message = "ExecuteTask goal was rejected";
    return false;
  }

  auto result_future = task_client_->async_get_result(goal_handle);
  const auto result_start = node_->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {break;}
    if (is_canceling()) {
      task_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if ((node_->now() - result_start).seconds() > 600.0) {
      task_client_->async_cancel_goal(goal_handle);
      error_message = "ExecuteTask result timed out";
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  const bool ok = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success;
  if (!ok) {
    error_message = wrapped_result.result ? wrapped_result.result->message : "no result";
  }
  return ok;
}

}  // namespace forklift_mission_manager