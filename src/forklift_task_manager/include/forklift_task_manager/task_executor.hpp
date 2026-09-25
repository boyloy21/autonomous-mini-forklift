#ifndef FORKLIFT_TASK_MANAGER__TASK_EXECUTOR_HPP_
#define FORKLIFT_TASK_MANAGER__TASK_EXECUTOR_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"
#include "forklift_interfaces/action/set_lift_height.hpp"

#include "forklift_docking/rear_charging_maneuver.hpp"
#include "forklift_task_manager/task.hpp"

namespace forklift_task_manager
{

using StepFeedbackFn = std::function<void (const std::string &, const std::string &, float)>;
using IsCancelingFn = std::function<bool ()>;

// Reports a sub-step transition to whoever is publishing task_status:
// (step, state, message). "step" is a TaskStepStatus step name such as
// DETECT or APPROACH, not necessarily a StepType -- docking's internal
// phases come from opennav_docking's DockRobot feedback, which only this
// class (its action client) receives.
using SubStepFn = std::function<void (const std::string &, const std::string &,
    const std::string &)>;

// Dispatches a single Step to whichever downstream subsystem owns that
// primitive. Adds three step types vs. the previous design:
//   CHARGE_APPROACH   -- after DOCK at a charging dock: turn around and
//                         reverse until the rear touches the charger pad
//                         (forklift_docking::RearChargingManeuver, closed
//                         loop on odom, charging_* parameters).
//   MONITOR_CHARGING  -- watches battery_state (subscribed once, kept
//                         live): current must exceed charging_threshold_
//                         within charging_monitor_timeout, then percentage
//                         must reach charging_full_percentage within
//                         charging_full_timeout.
//   CHARGE_RELEASE    -- drive forward off the charger pad once full. This is the one step type that
//                         puts its owning task into TaskState::MONITORING
//                         rather than EXECUTING.
//   STATION_OPERATION -- simple cancel-aware wait, generic placeholder
//                         for whatever a station's operation turns out to
//                         require; currently just a duration.
class TaskExecutor
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using DockRobot = opennav_docking_msgs::action::DockRobot;
  using UndockRobot = opennav_docking_msgs::action::UndockRobot;
  using SetLiftHeight = forklift_interfaces::action::SetLiftHeight;

  TaskExecutor(
    rclcpp::Node * node,
    const std::string & navigate_action_name,
    const std::string & dock_action_name,
    const std::string & undock_action_name,
    const std::string & lift_action_name,
    const std::string & aruco_node_name,
    const std::string & dock_database_path,
    const std::string & battery_topic,
    double charging_current_threshold,
    double charging_monitor_timeout);

  bool executeStep(
    Step & step, const IsCancelingFn & is_canceling,
    std::string & error_code, std::string & error_message);

  // Set by TaskManagerNode so docking sub-steps reach task_status. Left
  // unset, every report is simply dropped and behaviour is unchanged.
  void setSubStepCallback(SubStepFn fn) {sub_step_fn_ = std::move(fn);}

  // true only while a MONITOR_CHARGING step is actually running -- lets
  // TaskManagerNode's worker loop know to report TaskState::MONITORING
  // instead of EXECUTING for this step, without needing its own copy of
  // the battery-watching logic.
  bool isMonitoring() const {return monitoring_.load();}

private:
  bool executeNavigate(Step & step, const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeDock(Step & step, const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeUndock(Step & step, const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeLift(Step & step, const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeChargeApproach(
    Step & step, const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeMonitorCharging(const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeChargeRelease(
    Step & step, const IsCancelingFn & is_canceling, std::string & error_message);
  bool executeStationOperation(
    Step & step, const IsCancelingFn & is_canceling, std::string & error_message);

  // No-op when no callback is set, so every call site can report freely.
  void reportSubStep(
    const std::string & step, const std::string & state, const std::string & message = "") const
  {
    if (sub_step_fn_) {sub_step_fn_(step, state, message);}
  }

  bool setArucoTargetMarker(int marker_id);
  void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node * node_;
  SubStepFn sub_step_fn_;
  // Last docking sub-step reported, so the DockRobot feedback callback
  // (which fires at the docking control rate) only emits transitions.
  // A member, not a local: feedback can still arrive after executeDock
  // returns on the cancel path.
  std::string last_dock_substep_;
  std::string aruco_node_name_;
  std::string dock_database_path_;
  double charging_current_threshold_;
  double charging_monitor_timeout_;
  // MONITOR_CHARGING, after charging starts: wait until the battery
  // percentage (0-1) reaches charging_full_percentage_.
  double charging_full_percentage_;
  double charging_full_timeout_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp_action::Client<DockRobot>::SharedPtr dock_client_;
  rclcpp_action::Client<UndockRobot>::SharedPtr undock_client_;
  rclcpp_action::Client<SetLiftHeight>::SharedPtr lift_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> aruco_param_client_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::unique_ptr<forklift_docking::RearChargingManeuver> charging_maneuver_;

  std::atomic<float> last_battery_current_{0.0f};
  std::atomic<float> last_battery_percentage_{0.0f};
  std::atomic<bool> have_battery_reading_{false};
  std::atomic<bool> monitoring_{false};
};

}  // namespace forklift_task_manager

#endif  // FORKLIFT_TASK_MANAGER__TASK_EXECUTOR_HPP_