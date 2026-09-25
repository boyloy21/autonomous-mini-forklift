#ifndef FORKLIFT_MISSION_MANAGER__MISSION_SERVER_HPP_
#define FORKLIFT_MISSION_MANAGER__MISSION_SERVER_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "forklift_interfaces/action/execute_mission.hpp"

#include "forklift_mission_manager/mission.hpp"
#include "forklift_mission_manager/mission_database.hpp"
#include "forklift_mission_manager/mission_executor.hpp"

namespace forklift_mission_manager
{

// ExecuteMission action server. No queue -- a single mission executes at
// a time (matches the spec's flow diagram, which shows no queueing
// component at this layer, unlike forklift_task_manager's explicit task
// queue). A second goal arriving while one is executing is REJECTED, not
// queued; the caller (forklift_web_bridge) is expected to wait for
// completion or retry.
//
// Registered as an rclcpp_components plugin -- no main.cpp authored in
// this package, per design rule 6.
class MissionServer : public rclcpp::Node
{
public:
  using ExecuteMission = forklift_interfaces::action::ExecuteMission;
  using GoalHandleExecuteMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;

  explicit MissionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMission::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  void execute(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);

  static Mission goalToMission(const ExecuteMission::Goal & goal);
  void loadParameters();

  std::string mission_database_path_;
  std::string execute_task_action_name_;

  std::unique_ptr<MissionDatabase> mission_database_;
  std::unique_ptr<MissionExecutor> mission_executor_;

  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;

  std::atomic<bool> busy_{false};
};

}  // namespace forklift_mission_manager

#endif  // FORKLIFT_MISSION_MANAGER__MISSION_SERVER_HPP_