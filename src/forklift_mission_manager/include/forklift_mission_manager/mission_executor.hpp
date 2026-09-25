#ifndef FORKLIFT_MISSION_MANAGER__MISSION_EXECUTOR_HPP_
#define FORKLIFT_MISSION_MANAGER__MISSION_EXECUTOR_HPP_

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "forklift_interfaces/action/execute_task.hpp"
#include "forklift_mission_manager/mission.hpp"

namespace forklift_mission_manager
{

// (current_task, task_index, total_tasks, state, progress 0-100, message)
using MissionFeedbackFn = std::function<void (
    const std::string &, int, int, const std::string &, float, const std::string &)>;
using IsCancelingFn = std::function<bool ()>;

struct MissionOutcome
{
  bool success{false};
  std::string message;
  std::string failed_task;
};

// Sends mission.tasks to forklift_task_manager one at a time via the
// ExecuteTask action client, waiting for each result before starting the
// next -- stops the whole mission on the first task failure. Never talks
// to nav2/docking_server/lift_controller_node directly; that's exactly
// the layering forklift_task_manager exists to own (design rule 1: no
// duplication).
class MissionExecutor
{
public:
  explicit MissionExecutor(rclcpp::Node * node, const std::string & execute_task_action_name);

  MissionOutcome run(
    Mission & mission, const MissionFeedbackFn & feedback, const IsCancelingFn & is_canceling);

private:
  using ExecuteTask = forklift_interfaces::action::ExecuteTask;

  // Sends one TaskSpec, blocks (via polling, not spin_until_future_complete
  // -- same reasoning as every other downstream call built in this
  // project: the node is already being spun elsewhere) until it
  // completes. Relays ExecuteTask's own feedback into mission-level
  // feedback via the feedback callback, prefixed with task context.
  bool executeTask(
    const TaskSpec & spec, int task_index, int total_tasks,
    const MissionFeedbackFn & feedback, const IsCancelingFn & is_canceling,
    std::string & error_message);

  rclcpp::Node * node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr task_client_;
};

}  // namespace forklift_mission_manager

#endif  // FORKLIFT_MISSION_MANAGER__MISSION_EXECUTOR_HPP_