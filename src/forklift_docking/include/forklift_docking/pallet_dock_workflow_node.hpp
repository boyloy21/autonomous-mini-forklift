#ifndef FORKLIFT_DOCKING__PALLET_DOCK_WORKFLOW_NODE_HPP_
#define FORKLIFT_DOCKING__PALLET_DOCK_WORKFLOW_NODE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"
#include "forklift_interfaces/action/set_lift_height.hpp"
#include "forklift_interfaces/action/pallet_dock_workflow.hpp"
#include "forklift_docking/dock_database.hpp"
#include "forklift_docking/rear_charging_maneuver.hpp"

namespace forklift_docking
{

// Strips a trailing "_<N>" instance suffix, e.g. "pallet_pick_dock_1" ->
// "pallet_pick_dock" -- matches the naming convention used in
// dock_database.yaml. Falls back to the unmodified string if no numeric
// suffix is found.
std::string dockTypeFromDockId(const std::string & dock_id);

// Outcome of the shared dock->lift->undock step sequence, returned by
// executeWorkflowSteps() and translated into either an action Result
// (real interface) or a log line (legacy topic path) by the two thin
// callers.
struct WorkflowOutcome
{
  bool success{false};
  std::string error_code;
  std::string message;
};

// Orchestrates the dock->lift->undock sequence. Two ways to trigger it:
//   1. PalletDockWorkflow action (primary interface) -- real feedback,
//      real cancellation, propagated down into every downstream call.
//   2. /dock_pallet topic (std_msgs/String, legacy) -- kept for backward
//      compatibility and quick manual testing via `ros2 topic pub`, but
//      has no feedback stream and cannot be canceled.
// Both paths call the same executeWorkflowSteps() core, so there is
// exactly one implementation of the dock/lift/undock sequence -- not two
// copies that could drift apart.
//
// Registered as an rclcpp_components plugin (see
// pallet_dock_workflow_node.cpp) -- no main.cpp authored in this package,
// consistent with docking_server.
class PalletDockWorkflowNode : public rclcpp::Node
{
public:
  using DockRobot = opennav_docking_msgs::action::DockRobot;
  using UndockRobot = opennav_docking_msgs::action::UndockRobot;
  using SetLiftHeight = forklift_interfaces::action::SetLiftHeight;
  using PalletDockWorkflow = forklift_interfaces::action::PalletDockWorkflow;
  using GoalHandleWorkflow = rclcpp_action::ServerGoalHandle<PalletDockWorkflow>;

  // FeedbackFn: (state, message, progress 0-100) -- called at each step
  // transition. IsCancelingFn: polled at safe checkpoints inside every
  // downstream call; when it returns true, the in-flight downstream
  // action is actively canceled (not just abandoned) before returning.
  using FeedbackFn = std::function<void (const std::string &, const std::string &, float)>;
  using IsCancelingFn = std::function<bool ()>;

  explicit PalletDockWorkflowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();

  // ---- Legacy topic trigger ----
  void onDockPallet(const std_msgs::msg::String::SharedPtr msg);
  void runWorkflow(const std::string dock_id);

  // ---- PalletDockWorkflow action server ----
  rclcpp_action::GoalResponse handleWorkflowGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PalletDockWorkflow::Goal> goal);
  rclcpp_action::CancelResponse handleWorkflowCancel(
    const std::shared_ptr<GoalHandleWorkflow> goal_handle);
  void handleWorkflowAccepted(const std::shared_ptr<GoalHandleWorkflow> goal_handle);
  void executeWorkflowAction(const std::shared_ptr<GoalHandleWorkflow> goal_handle);

  // ---- Shared core, used by both trigger paths ----
  WorkflowOutcome executeWorkflowSteps(
    const std::string & dock_id, const FeedbackFn & feedback, const IsCancelingFn & is_canceling);

  bool sendDockGoal(const std::string & dock_id, const IsCancelingFn & is_canceling);
  bool callSetHeight(double target_height, const IsCancelingFn & is_canceling);
  bool sendUndockGoal(const std::string & dock_type, const IsCancelingFn & is_canceling);
  bool setArucoTargetMarker(int marker_id);

  // Parameters
  std::string dock_pallet_topic_;
  std::string workflow_action_name_;
  double post_dock_delay_;
  double lift_target_height_;
  double lift_speed_;
  std::string dock_action_name_;
  std::string undock_action_name_;
  std::string set_height_service_name_;
  double undock_max_time_;
  double set_height_timeout_;
  double lift_height_tolerance_;
  std::string dock_database_path_;
  std::string aruco_node_name_;

  rclcpp_action::Client<DockRobot>::SharedPtr dock_client_;
  rclcpp_action::Client<UndockRobot>::SharedPtr undock_client_;
  rclcpp_action::Client<SetLiftHeight>::SharedPtr set_height_client_;
  rclcpp_action::Server<PalletDockWorkflow>::SharedPtr workflow_action_server_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_pallet_sub_;
  std::shared_ptr<rclcpp::AsyncParametersClient> aruco_param_client_;
  std::unique_ptr<DockDatabase> dock_database_;

  // Charging-dock final stretch: the camera is front-only but the charging
  // contact is on the rear, so DockRobot ends facing the charger and this
  // turns the robot around and reverses it into contact (then undoes that
  // exactly before UndockRobot). Parameters: charging_* -- see
  // rear_charging_maneuver.hpp.
  std::unique_ptr<RearChargingManeuver> charging_maneuver_;

  // Prevents two overlapping workflows (from either trigger path) running
  // at once -- a real forklift can only do one thing at a time.
  std::atomic<bool> busy_{false};
};

}  // namespace forklift_docking

#endif  // FORKLIFT_DOCKING__PALLET_DOCK_WORKFLOW_NODE_HPP_