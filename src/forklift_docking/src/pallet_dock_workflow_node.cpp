#include "forklift_docking/pallet_dock_workflow_node.hpp"

#include <chrono>
#include <cmath>
#include <regex>
#include <thread>

using namespace std::chrono_literals;

namespace forklift_docking
{

std::string dockTypeFromDockId(const std::string & dock_id)
{
  static const std::regex suffix_re(R"(_\d+$)");
  return std::regex_replace(dock_id, suffix_re, "");
}

PalletDockWorkflowNode::PalletDockWorkflowNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pallet_dock_workflow_node", options)
{
  loadParameters();

  dock_client_ = rclcpp_action::create_client<DockRobot>(this, dock_action_name_);
  undock_client_ = rclcpp_action::create_client<UndockRobot>(this, undock_action_name_);
  set_height_client_ = rclcpp_action::create_client<SetLiftHeight>(
    this, set_height_service_name_);

  dock_database_ = std::make_unique<DockDatabase>(this->get_logger());
  if (!dock_database_->loadFromFile(dock_database_path_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to load dock database from '%s' -- automatic aruco target_marker_id set will be "
      "skipped for every dock (you'll need to set it manually, or this dock type may rely on "
      "blind/no-detection docking anyway).",
      dock_database_path_.c_str());
  }

  aruco_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, aruco_node_name_);

  charging_maneuver_ = std::make_unique<RearChargingManeuver>(
    this, RearChargingManeuver::declareParameters(this, "charging_"));

  // Primary interface: PalletDockWorkflow action -- real feedback, real
  // cancellation.
  workflow_action_server_ = rclcpp_action::create_server<PalletDockWorkflow>(
    this, workflow_action_name_,
    std::bind(&PalletDockWorkflowNode::handleWorkflowGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PalletDockWorkflowNode::handleWorkflowCancel, this, std::placeholders::_1),
    std::bind(&PalletDockWorkflowNode::handleWorkflowAccepted, this, std::placeholders::_1));

  // Legacy interface: kept for backward compatibility / quick manual
  // testing via `ros2 topic pub`. No feedback, no cancellation -- prefer
  // the action above for anything programmatic.
  dock_pallet_sub_ = this->create_subscription<std_msgs::msg::String>(
    dock_pallet_topic_, 10,
    std::bind(&PalletDockWorkflowNode::onDockPallet, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "pallet_dock_workflow_node ready. Primary interface: action '%s'. "
    "Legacy interface: topic '%s' (no feedback/cancel).",
    workflow_action_name_.c_str(), dock_pallet_topic_.c_str());
}

void PalletDockWorkflowNode::loadParameters()
{
  // MULTI-ROBOT FIX (reapplied -- this file had drifted back to absolute
  // defaults since the earlier namespace pass): all relative (no leading
  // "/") below, required for per-robot namespacing via
  // PushRosNamespace/Node(namespace=...) to actually apply. Every
  // downstream node these reference (docking_server, lift_controller_node,
  // aruco_node) must be launched under the same robot namespace as this
  // node -- same requirement as everywhere else in this project.
  dock_pallet_topic_ = this->declare_parameter("dock_pallet_topic", std::string("dock_pallet"));
  workflow_action_name_ =
    this->declare_parameter("workflow_action_name", std::string("pallet_dock_workflow"));
  post_dock_delay_ = this->declare_parameter("post_dock_delay", 2.0);
  lift_target_height_ = this->declare_parameter("lift_target_height", 0.15);
  lift_speed_ = this->declare_parameter("lift_speed", 0.0);
  dock_action_name_ = this->declare_parameter("dock_action_name", std::string("dock_robot"));
  undock_action_name_ = this->declare_parameter("undock_action_name", std::string("undock_robot"));
  set_height_service_name_ =
    this->declare_parameter("set_height_service_name", std::string("set_height"));
  undock_max_time_ = this->declare_parameter("undock_max_time", 15.0);
  set_height_timeout_ = this->declare_parameter("set_height_timeout", 35.0);
  lift_height_tolerance_ = this->declare_parameter("lift_height_tolerance", 0.03);
  dock_database_path_ = this->declare_parameter("dock_database", std::string(""));
  aruco_node_name_ = this->declare_parameter("aruco_node_name", std::string("aruco_node"));
}

// ==================== Legacy topic trigger ====================

void PalletDockWorkflowNode::onDockPallet(const std_msgs::msg::String::SharedPtr msg)
{
  std::string dock_id = msg->data;
  dock_id.erase(0, dock_id.find_first_not_of(" \t\n\r"));
  const auto last = dock_id.find_last_not_of(" \t\n\r");
  if (last != std::string::npos) {
    dock_id.erase(last + 1);
  }

  if (dock_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty dock_id on /dock_pallet, ignoring.");
    return;
  }

  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Workflow already in progress, ignoring new /dock_pallet request for '%s'. "
      "(Use the PalletDockWorkflow action instead for proper queuing/feedback.)",
      dock_id.c_str());
    return;
  }

  std::thread(&PalletDockWorkflowNode::runWorkflow, this, dock_id).detach();
}

void PalletDockWorkflowNode::runWorkflow(const std::string dock_id)
{
  RCLCPP_INFO(this->get_logger(), "=== [legacy topic] Starting workflow for '%s' ===", dock_id.c_str());

  auto outcome = executeWorkflowSteps(
    dock_id,
    [this](const std::string & state, const std::string & message, float progress) {
      RCLCPP_INFO(
        this->get_logger(), "[%s] %s (%.0f%%)", state.c_str(), message.c_str(), progress);
    },
    []() {return false;});  // legacy topic path has no way to be canceled

  if (outcome.success) {
    RCLCPP_INFO(this->get_logger(), "=== [legacy topic] Workflow for '%s' complete ===", dock_id.c_str());
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "=== [legacy topic] Workflow for '%s' failed: %s (%s) ===",
      dock_id.c_str(), outcome.error_code.c_str(), outcome.message.c_str());
  }

  busy_.store(false);
}

// ==================== PalletDockWorkflow action server ====================

rclcpp_action::GoalResponse PalletDockWorkflowNode::handleWorkflowGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PalletDockWorkflow::Goal> goal)
{
  if (goal->dock_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: empty dock_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (busy_.load()) {
    RCLCPP_WARN(
      this->get_logger(), "Rejecting goal for '%s': a workflow is already running "
      "(this node runs one workflow at a time; task_manager will add real queuing later).",
      goal->dock_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PalletDockWorkflowNode::handleWorkflowCancel(
  const std::shared_ptr<GoalHandleWorkflow>)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request received for PalletDockWorkflow goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PalletDockWorkflowNode::handleWorkflowAccepted(
  const std::shared_ptr<GoalHandleWorkflow> goal_handle)
{
  busy_.store(true);
  std::thread(&PalletDockWorkflowNode::executeWorkflowAction, this, goal_handle).detach();
}

void PalletDockWorkflowNode::executeWorkflowAction(
  const std::shared_ptr<GoalHandleWorkflow> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback_msg = std::make_shared<PalletDockWorkflow::Feedback>();

  auto outcome = executeWorkflowSteps(
    goal->dock_id,
    [&](const std::string & state, const std::string & message, float progress) {
      feedback_msg->state = state;
      feedback_msg->message = message;
      feedback_msg->progress = progress;
      goal_handle->publish_feedback(feedback_msg);
    },
    [goal_handle]() {return goal_handle->is_canceling();});

  auto result = std::make_shared<PalletDockWorkflow::Result>();
  result->success = outcome.success;
  result->error_code = outcome.error_code;
  result->message = outcome.message;

  if (outcome.error_code == "CANCELED") {
    goal_handle->canceled(result);
  } else if (outcome.success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }

  busy_.store(false);
}

// ==================== Shared core ====================

WorkflowOutcome PalletDockWorkflowNode::executeWorkflowSteps(
  const std::string & dock_id, const FeedbackFn & feedback, const IsCancelingFn & is_canceling)
{
  WorkflowOutcome outcome;

  // Re-read per goal (like forklift_task_manager's TaskExecutor) so docks
  // added through the web bridge's POST /docks are usable without a
  // restart. On failure the previously loaded docks are cleared, same as
  // at startup, and marker lookup below is just skipped.
  dock_database_->loadFromFile(dock_database_path_);
  auto maybe_dock = dock_database_->getDock(dock_id);

  // ---- Step 0: marker id lookup + set on aruco_node ----
  feedback("SET_MARKER", "Looking up ArUco marker for this dock", 5.0f);
  if (maybe_dock && maybe_dock->marker_id >= 0) {
    if (!setArucoTargetMarker(maybe_dock->marker_id)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not set aruco target marker for '%s' -- continuing anyway (this dock may rely "
        "on blind/detection-less docking, or aruco_node may need attention separately).",
        dock_id.c_str());
    }
  }

  if (is_canceling()) {
    outcome.error_code = "CANCELED";
    outcome.message = "Canceled before docking started";
    return outcome;
  }

  // ---- Step 1: DockRobot ----
  feedback("DOCKING", "Navigating and docking", 15.0f);
  if (!sendDockGoal(dock_id, is_canceling)) {
    outcome.error_code = is_canceling() ? "CANCELED" : "DOCK_FAILED";
    outcome.message = "DockRobot failed for '" + dock_id + "'";
    return outcome;
  }

  // ---- Step 1b: charging-dock final approach (turn around, reverse into contact) ----
  const bool is_charging_dock = maybe_dock && maybe_dock->type == DockType::CHARGING_DOCK;
  if (is_charging_dock) {
    feedback("CHARGING_APPROACH", "Turning around and reversing into charging contact", 45.0f);
    if (!charging_maneuver_->approach(is_canceling)) {
      outcome.error_code = is_canceling() ? "CANCELED" : "CHARGING_APPROACH_FAILED";
      outcome.message = "Charging approach maneuver failed for '" + dock_id + "'";
      return outcome;
    }
  }

  if (is_canceling()) {
    outcome.error_code = "CANCELED";
    outcome.message = "Canceled after docking, before lift";
    return outcome;
  }

  feedback("WAITING", "Settling before lift", 55.0f);
  std::this_thread::sleep_for(std::chrono::duration<double>(post_dock_delay_));

  // ---- Step 3: lift, per-dock height from database ----
  const double target_height = maybe_dock ? maybe_dock->lift_height : lift_target_height_;
  feedback("LIFTING", "Setting lift height", 65.0f);
  if (!callSetHeight(target_height, is_canceling)) {
    outcome.error_code = is_canceling() ? "CANCELED" : "LIFT_FAILED";
    outcome.message = "SetLiftHeight failed for '" + dock_id + "'";
    return outcome;
  }

  // ---- Step 3b: undo charging maneuver before UndockRobot ----
  if (is_charging_dock) {
    feedback("CHARGING_APPROACH_UNDO", "Restoring pre-contact pose", 85.0f);
    charging_maneuver_->undo(is_canceling);
    // deliberately not fatal if this fails partway -- still attempt undock below
  }

  if (is_canceling()) {
    outcome.error_code = "CANCELED";
    outcome.message = "Canceled before undock";
    return outcome;
  }

  // ---- Step 4: UndockRobot ----
  const std::string dock_type = dockTypeFromDockId(dock_id);
  feedback("UNDOCKING", "Backing away from dock", 92.0f);
  sendUndockGoal(dock_type, is_canceling);

  feedback("DONE", "Workflow complete", 100.0f);
  outcome.success = true;
  outcome.message = "Workflow for '" + dock_id + "' completed successfully";
  return outcome;
}

// ==================== Downstream call wrappers (cancel-aware) ====================

bool PalletDockWorkflowNode::sendDockGoal(
  const std::string & dock_id, const IsCancelingFn & is_canceling)
{
  if (!dock_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "DockRobot action server not available.");
    return false;
  }

  DockRobot::Goal goal;
  goal.use_dock_id = true;
  goal.dock_id = dock_id;
  goal.navigate_to_staging_pose = true;

  auto goal_handle_future = dock_client_->async_send_goal(goal);

  const auto send_start = this->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {
      break;
    }
    if (is_canceling()) {
      return false;  // goal not yet accepted, nothing to cancel downstream
    }
    if ((this->now() - send_start).seconds() > 10.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for DockRobot goal acceptance.");
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "DockRobot goal was rejected.");
    return false;
  }

  auto result_future = dock_client_->async_get_result(goal_handle);
  const auto result_start = this->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      break;
    }
    if (is_canceling()) {
      dock_client_->async_cancel_goal(goal_handle);
      RCLCPP_INFO(this->get_logger(), "DockRobot canceled by workflow cancellation.");
      return false;
    }
    if ((this->now() - result_start).seconds() > 60.0) {
      RCLCPP_ERROR(this->get_logger(), "DockRobot result timed out.");
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
         wrapped_result.result->success;
}

bool PalletDockWorkflowNode::callSetHeight(
  double target_height, const IsCancelingFn & is_canceling)
{
  if (!set_height_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "SetLiftHeight action server not available.");
    return false;
  }

  SetLiftHeight::Goal goal;
  goal.target_height = static_cast<float>(target_height);
  goal.speed = static_cast<float>(lift_speed_);

  auto goal_handle_future = set_height_client_->async_send_goal(goal);

  const auto send_start = this->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {
      break;
    }
    if (is_canceling()) {
      return false;
    }
    if ((this->now() - send_start).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for SetLiftHeight goal acceptance.");
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "SetLiftHeight goal was rejected.");
    return false;
  }

  auto result_future = set_height_client_->async_get_result(goal_handle);
  const auto result_start = this->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      break;
    }
    if (is_canceling()) {
      set_height_client_->async_cancel_goal(goal_handle);
      RCLCPP_INFO(this->get_logger(), "SetLiftHeight canceled by workflow cancellation.");
      return false;
    }
    if ((this->now() - result_start).seconds() > set_height_timeout_) {
      RCLCPP_ERROR(this->get_logger(), "SetLiftHeight result timed out (server unresponsive?).");
      return false;
    }
  }

  auto wrapped_result = result_future.get();

  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success)
  {
    return true;
  }

  const double current_height = wrapped_result.result ? wrapped_result.result->current_height : 0.0;
  const double height_diff = std::fabs(current_height - target_height);
  if (height_diff <= lift_height_tolerance_) {
    RCLCPP_WARN(
      this->get_logger(),
      "SetLiftHeight server reported failure, but current_height=%.3fm is within %.3fm of "
      "target=%.3fm -- accepting.",
      current_height, lift_height_tolerance_, target_height);
    return true;
  }

  return false;
}

bool PalletDockWorkflowNode::sendUndockGoal(
  const std::string & dock_type, const IsCancelingFn & is_canceling)
{
  if (!undock_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "UndockRobot action server not available.");
    return false;
  }

  UndockRobot::Goal goal;
  goal.dock_type = dock_type;
  goal.max_undocking_time = static_cast<float>(undock_max_time_);

  auto goal_handle_future = undock_client_->async_send_goal(goal);

  const auto send_start = this->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {
      break;
    }
    if (is_canceling()) {
      return false;
    }
    if ((this->now() - send_start).seconds() > 10.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for UndockRobot goal acceptance.");
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "UndockRobot goal was rejected.");
    return false;
  }

  auto result_future = undock_client_->async_get_result(goal_handle);
  const auto result_start = this->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      break;
    }
    if (is_canceling()) {
      undock_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if ((this->now() - result_start).seconds() > 30.0) {
      RCLCPP_ERROR(this->get_logger(), "UndockRobot result timed out.");
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  const bool ok = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success;
  if (ok) {
    RCLCPP_INFO(this->get_logger(), "Undock succeeded (dock_type='%s').", dock_type.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Undock reported failure.");
  }
  return ok;
}

bool PalletDockWorkflowNode::setArucoTargetMarker(int marker_id)
{
  if (!aruco_param_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(
      this->get_logger(), "Parameter service for '%s' not available.", aruco_node_name_.c_str());
    return false;
  }

  auto future = aruco_param_client_->set_parameters(
    {rclcpp::Parameter("target_marker_id", marker_id)});

  const auto start = this->now();
  while (rclcpp::ok()) {
    if (future.wait_for(50ms) == std::future_status::ready) {
      break;
    }
    if ((this->now() - start).seconds() > 5.0) {
      RCLCPP_ERROR(
        this->get_logger(), "Timed out setting target_marker_id on '%s'.",
        aruco_node_name_.c_str());
      return false;
    }
  }

  auto results = future.get();
  for (const auto & result : results) {
    if (!result.successful) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set target_marker_id: %s", result.reason.c_str());
      return false;
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Set %s target_marker_id = %d", aruco_node_name_.c_str(), marker_id);
  return true;
}

}  // namespace forklift_docking

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(forklift_docking::PalletDockWorkflowNode)