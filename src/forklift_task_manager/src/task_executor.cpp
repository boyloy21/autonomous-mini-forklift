#include "forklift_task_manager/task_executor.hpp"
#include "forklift_task_manager/dock_database.hpp"

#include <chrono>
#include <regex>
#include <thread>

using namespace std::chrono_literals;

namespace forklift_task_manager
{

namespace
{
std::string dockTypeFromDockId(const std::string & dock_id)
{
  static const std::regex suffix_re(R"(_\d+$)");
  return std::regex_replace(dock_id, suffix_re, "");
}
}  // namespace

TaskExecutor::TaskExecutor(
  rclcpp::Node * node,
  const std::string & navigate_action_name,
  const std::string & dock_action_name,
  const std::string & undock_action_name,
  const std::string & lift_action_name,
  const std::string & aruco_node_name,
  const std::string & dock_database_path,
  const std::string & battery_topic,
  double charging_current_threshold,
  double charging_monitor_timeout)
: node_(node),
  aruco_node_name_(aruco_node_name),
  dock_database_path_(dock_database_path),
  charging_current_threshold_(charging_current_threshold),
  charging_monitor_timeout_(charging_monitor_timeout)
{
  navigate_client_ = rclcpp_action::create_client<NavigateToPose>(node_, navigate_action_name);
  dock_client_ = rclcpp_action::create_client<DockRobot>(node_, dock_action_name);
  undock_client_ = rclcpp_action::create_client<UndockRobot>(node_, undock_action_name);
  lift_client_ = rclcpp_action::create_client<SetLiftHeight>(node_, lift_action_name);
  aruco_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, aruco_node_name_);

  // Subscribed once, kept live for the node's whole lifetime -- cheap,
  // and means MONITOR_CHARGING never has to wait for a fresh subscription
  // to receive its first message before it can start comparing.
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic, rclcpp::SensorDataQoS(),
    std::bind(&TaskExecutor::batteryStateCallback, this, std::placeholders::_1));

  charging_maneuver_ = std::make_unique<forklift_docking::RearChargingManeuver>(
    node_, forklift_docking::RearChargingManeuver::declareParameters(node_, "charging_"));
  charging_full_percentage_ = node_->declare_parameter("charging_full_percentage", 1.0);
  charging_full_timeout_ = node_->declare_parameter("charging_full_timeout", 1800.0);
}

void TaskExecutor::batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  last_battery_current_.store(msg->current);
  last_battery_percentage_.store(msg->percentage);
  have_battery_reading_.store(true);
}

bool TaskExecutor::executeStep(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_code,
  std::string & error_message)
{
  bool ok = false;
  switch (step.type) {
    case StepType::NAVIGATE: ok = executeNavigate(step, is_canceling, error_message); break;
    case StepType::DOCK: ok = executeDock(step, is_canceling, error_message); break;
    case StepType::LIFT:
    case StepType::LOWER: ok = executeLift(step, is_canceling, error_message); break;
    case StepType::UNDOCK: ok = executeUndock(step, is_canceling, error_message); break;
    case StepType::CHARGE_APPROACH: ok = executeChargeApproach(step, is_canceling, error_message); break;
    case StepType::MONITOR_CHARGING: ok = executeMonitorCharging(is_canceling, error_message); break;
    case StepType::CHARGE_RELEASE: ok = executeChargeRelease(step, is_canceling, error_message); break;
    case StepType::STATION_OPERATION: ok = executeStationOperation(step, is_canceling, error_message); break;
  }

  if (!ok) {
    error_code = is_canceling() ? "CANCELED" : (stepTypeToString(step.type) + "_FAILED");
  }
  return ok;
}

// ==================== NAVIGATE ====================

bool TaskExecutor::executeNavigate(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_message)
{
  if (!navigate_client_->wait_for_action_server(5s)) {
    error_message = "NavigateToPose action server not available";
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose = step.params.target_pose;

  auto goal_handle_future = navigate_client_->async_send_goal(goal);

  const auto send_start = node_->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {break;}
    if (is_canceling()) {return false;}
    if ((node_->now() - send_start).seconds() > 5.0) {
      error_message = "Timed out waiting for NavigateToPose goal acceptance";
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_message = "NavigateToPose goal was rejected";
    return false;
  }

  auto result_future = navigate_client_->async_get_result(goal_handle);
  const auto result_start = node_->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {break;}
    if (is_canceling()) {
      navigate_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if ((node_->now() - result_start).seconds() > 120.0) {
      navigate_client_->async_cancel_goal(goal_handle);
      error_message = "NavigateToPose timed out";
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error_message = "NavigateToPose did not succeed";
    return false;
  }
  return true;
}

// ==================== DOCK ====================

bool TaskExecutor::executeDock(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_message)
{
  DockDatabase db(node_->get_logger());
  if (db.loadFromFile(dock_database_path_)) {
    auto maybe_dock = db.getDock(step.params.dock_id);
    if (maybe_dock && maybe_dock->marker_id >= 0) {
      if (!setArucoTargetMarker(maybe_dock->marker_id)) {
        RCLCPP_WARN(
          node_->get_logger(), "Could not set aruco target marker for '%s', continuing anyway.",
          step.params.dock_id.c_str());
      }
    }
  }

  if (!dock_client_->wait_for_action_server(5s)) {
    error_message = "DockRobot action server not available";
    return false;
  }

  DockRobot::Goal goal;
  goal.use_dock_id = true;
  goal.dock_id = step.params.dock_id;
  goal.navigate_to_staging_pose = !step.params.skip_internal_staging_nav;

  // docking_server's own state machine is only visible to its action
  // client -- forward each transition so task_status can report DETECT /
  // APPROACH / WAIT_FOR_CHARGE instead of one opaque 30s DOCK step.
  rclcpp_action::Client<DockRobot>::SendGoalOptions send_options;
  last_dock_substep_.clear();
  send_options.feedback_callback =
    [this](
    rclcpp_action::ClientGoalHandle<DockRobot>::SharedPtr,
    const std::shared_ptr<const DockRobot::Feedback> feedback)
    {
      std::string substep;
      switch (feedback->state) {
        case DockRobot::Feedback::NAV_TO_STAGING_POSE: substep = "STAGING"; break;
        case DockRobot::Feedback::INITIAL_PERCEPTION: substep = "DETECT"; break;
        case DockRobot::Feedback::CONTROLLING: substep = "APPROACH"; break;
        case DockRobot::Feedback::WAIT_FOR_CHARGE: substep = "WAIT_FOR_CHARGE"; break;
        case DockRobot::Feedback::RETRY: substep = "DETECT"; break;
        default: return;
      }
      const bool retrying = feedback->state == DockRobot::Feedback::RETRY;
      const std::string state = retrying ? "RETRY" : "RUNNING";
      // Feedback arrives at the docking control rate; only transitions
      // are worth a message.
      const std::string key = substep + state;
      if (key == last_dock_substep_) {return;}
      last_dock_substep_ = key;
      // CONTROLLING means the dock was found and is being driven onto.
      if (substep == "APPROACH") {
        this->reportSubStep("DETECT", "SUCCESS", "Dock detected");
      }
      this->reportSubStep(
        substep, state,
        retrying ? "Retrying dock detection (retry " + std::to_string(feedback->num_retries) + ")" :
        "Docking: " + substep);
    };

  auto goal_handle_future = dock_client_->async_send_goal(goal, send_options);

  const auto send_start = node_->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {break;}
    if (is_canceling()) {return false;}
    if ((node_->now() - send_start).seconds() > 10.0) {
      error_message = "Timed out waiting for DockRobot goal acceptance";
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_message = "DockRobot goal was rejected";
    return false;
  }

  auto result_future = dock_client_->async_get_result(goal_handle);
  const auto result_start = node_->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {break;}
    if (is_canceling()) {
      dock_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if ((node_->now() - result_start).seconds() > 60.0) {
      error_message = "DockRobot result timed out";
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  const bool ok = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success;
  if (!ok) {error_message = "DockRobot did not succeed";}
  return ok;
}

// ==================== UNDOCK ====================

bool TaskExecutor::executeUndock(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_message)
{
  if (!undock_client_->wait_for_action_server(5s)) {
    error_message = "UndockRobot action server not available";
    return false;
  }

  reportSubStep("UNDOCK", "RUNNING", "Undocking from " + step.params.dock_id);

  UndockRobot::Goal goal;
  goal.dock_type = dockTypeFromDockId(step.params.dock_id);
  goal.max_undocking_time = 15.0f;

  auto goal_handle_future = undock_client_->async_send_goal(goal);

  const auto send_start = node_->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {break;}
    if (is_canceling()) {return false;}
    if ((node_->now() - send_start).seconds() > 10.0) {
      error_message = "Timed out waiting for UndockRobot goal acceptance";
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_message = "UndockRobot goal was rejected";
    return false;
  }

  auto result_future = undock_client_->async_get_result(goal_handle);
  const auto result_start = node_->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {break;}
    if (is_canceling()) {
      undock_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if ((node_->now() - result_start).seconds() > 30.0) {
      error_message = "UndockRobot result timed out";
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  const bool ok = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success;
  if (!ok) {error_message = "UndockRobot did not succeed";}
  return ok;
}

// ==================== LIFT / LOWER ====================

bool TaskExecutor::executeLift(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_message)
{
  reportSubStep(
    step.type == StepType::LOWER ? "LOWER" : "LIFT", "RUNNING",
    "Moving fork to " + std::to_string(step.params.lift_height) + " m");
  if (!lift_client_->wait_for_action_server(5s)) {
    error_message = "SetLiftHeight action server not available";
    return false;
  }

  SetLiftHeight::Goal goal;
  goal.target_height = static_cast<float>(step.params.lift_height);
  goal.speed = 0.0f;

  auto goal_handle_future = lift_client_->async_send_goal(goal);

  const auto send_start = node_->now();
  while (rclcpp::ok()) {
    if (goal_handle_future.wait_for(50ms) == std::future_status::ready) {break;}
    if (is_canceling()) {return false;}
    if ((node_->now() - send_start).seconds() > 5.0) {
      error_message = "Timed out waiting for SetLiftHeight goal acceptance";
      return false;
    }
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_message = "SetLiftHeight goal was rejected";
    return false;
  }

  auto result_future = lift_client_->async_get_result(goal_handle);
  const auto result_start = node_->now();
  while (rclcpp::ok()) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {break;}
    if (is_canceling()) {
      lift_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if ((node_->now() - result_start).seconds() > 35.0) {
      error_message = "SetLiftHeight result timed out (server unresponsive?)";
      return false;
    }
  }

  auto wrapped_result = result_future.get();
  const bool ok = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success;
  if (!ok) {error_message = "SetLiftHeight did not succeed";}
  return ok;
}

// ==================== CHARGE_APPROACH ====================

bool TaskExecutor::executeChargeApproach(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_message)
{
  reportSubStep(
    "CHARGE_APPROACH", "RUNNING", "Reversing into charger " + step.params.dock_id);
  if (!charging_maneuver_->approach(is_canceling)) {
    error_message = "Could not reverse into charging contact at " + step.params.dock_id;
    return false;
  }
  reportSubStep("CHARGE_APPROACH", "SUCCESS", "Rear in contact with charger");
  return true;
}

// ==================== MONITOR_CHARGING ====================

bool TaskExecutor::executeMonitorCharging(
  const IsCancelingFn & is_canceling, std::string & error_message)
{
  monitoring_.store(true);
  reportSubStep("MONITOR_CHARGING", "RUNNING", "Waiting for charging current");
  auto start = node_->now();
  rclcpp::Rate rate(2.0);

  auto is_full = [this]() {
      return have_battery_reading_.load() &&
             last_battery_percentage_.load() >= charging_full_percentage_;
    };

  // Phase 1: charging has started (or the battery is already full).
  bool result = false;
  while (rclcpp::ok()) {
    if (is_canceling()) {
      break;
    }
    if (is_full() ||
      (have_battery_reading_.load() && last_battery_current_.load() > charging_current_threshold_))
    {
      result = true;
      RCLCPP_INFO(
        node_->get_logger(), "MONITOR_CHARGING: charging started at %d%%",
        static_cast<int>(last_battery_percentage_.load() * 100.0f));
      reportSubStep("MONITOR_CHARGING", "RUNNING", "Robot is charging");
      break;
    }
    if ((node_->now() - start).seconds() > charging_monitor_timeout_) {
      error_message = "Charging condition not reached within timeout "
        "(check battery_state is publishing and charging_current_threshold is realistic)";
      break;
    }
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000,
      "MONITOR_CHARGING: waiting for charging to start (battery %.0f%%, current %.2f A, "
      "need > %.2f A)", last_battery_percentage_.load() * 100.0f, last_battery_current_.load(),
      charging_current_threshold_);
    rate.sleep();
  }

  // Phase 2: wait for a full battery. Charging stopping early (robot
  // knocked off the pad) fails the step instead of waiting forever.
  start = node_->now();
  int last_reported = -1;
  while (result && rclcpp::ok()) {
    if (is_canceling()) {
      result = false;
      break;
    }
    if (is_full()) {
      RCLCPP_INFO(node_->get_logger(), "MONITOR_CHARGING: battery full");
      reportSubStep("MONITOR_CHARGING", "SUCCESS", "Battery full");
      break;
    }
    if (last_battery_current_.load() <= charging_current_threshold_) {
      result = false;
      error_message = "Charging stopped at " +
        std::to_string(static_cast<int>(last_battery_percentage_.load() * 100.0f)) +
        "% before the battery was full (lost charger contact?)";
      break;
    }
    if ((node_->now() - start).seconds() > charging_full_timeout_) {
      result = false;
      error_message = "Battery not full within charging_full_timeout";
      break;
    }
    const int pct = static_cast<int>(last_battery_percentage_.load() * 100.0f);
    if (pct / 10 != last_reported / 10) {
      last_reported = pct;
      RCLCPP_INFO(node_->get_logger(), "MONITOR_CHARGING: charging %d%%", pct);
      reportSubStep("MONITOR_CHARGING", "RUNNING", "Charging: " + std::to_string(pct) + "%");
    }
    rate.sleep();
  }

  monitoring_.store(false);
  return result;
}

// ==================== CHARGE_RELEASE ====================

bool TaskExecutor::executeChargeRelease(
  Step & step, const IsCancelingFn & is_canceling, std::string & error_message)
{
  reportSubStep("CHARGE_RELEASE", "RUNNING", "Driving forward off charger " + step.params.dock_id);
  if (!charging_maneuver_->release(is_canceling)) {
    error_message = "Could not drive off charger " + step.params.dock_id;
    return false;
  }
  reportSubStep("CHARGE_RELEASE", "SUCCESS", "Released from charger");
  return true;
}

// ==================== STATION_OPERATION ====================

bool TaskExecutor::executeStationOperation(
  Step & step, const IsCancelingFn & is_canceling, std::string &)
{
  reportSubStep(
    "STATION_OPERATION", "RUNNING",
    "Station wait of " + std::to_string(step.params.wait_duration) + " s");
  rclcpp::Rate rate(10.0);
  const auto start = node_->now();
  while (rclcpp::ok() && (node_->now() - start).seconds() < step.params.wait_duration) {
    if (is_canceling()) {return false;}
    rate.sleep();
  }
  return true;
}

// ==================== Helpers ====================

bool TaskExecutor::setArucoTargetMarker(int marker_id)
{
  if (!aruco_param_client_->wait_for_service(5s)) {
    return false;
  }
  auto future = aruco_param_client_->set_parameters(
    {rclcpp::Parameter("target_marker_id", marker_id)});

  const auto start = node_->now();
  while (rclcpp::ok()) {
    if (future.wait_for(50ms) == std::future_status::ready) {break;}
    if ((node_->now() - start).seconds() > 5.0) {return false;}
  }
  auto results = future.get();
  for (const auto & r : results) {
    if (!r.successful) {return false;}
  }
  return true;
}

}  // namespace forklift_task_manager