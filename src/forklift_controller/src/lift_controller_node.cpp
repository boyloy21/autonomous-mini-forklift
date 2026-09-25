// src/lift_controller_node.cpp
//
// Minimal standalone lift controller: /lift_up, /lift_down, /set_height
// only. No lift_safety_node/lift_limit_node/lift_feedback_node/
// lift_driver_node dependency. Height feedback comes from /joint_states.
//
// /lift_cmd is a POSITION setpoint (std_msgs/Float64, meters), for a
// ros2_control JointGroupPositionController.
//
// CHANGE FROM PREVIOUS VERSION: /set_height was previously a blocking
// rclcpp::Service whose callback ran a polling while-loop internally for
// up to timeout_s_ (30s) before returning. This is the wrong tool for a
// multi-second operation -- it ties up an executor thread the whole time,
// and forces every caller to guess a client-side wait long enough to
// exceed the server's own internal timeout (a fragile, easy-to-mismatch
// coupling -- exactly what caused the earlier "call timed out" bug).
//
// /set_height is now an rclcpp_action::Server<SetLiftHeight>. The goal is
// accepted (or rejected) immediately; the actual ramp-and-wait runs on its
// own detached thread via handleAccepted() -> execute(), publishing live
// feedback (current_height/target_height) as it goes, and delivering a
// proper succeed()/abort()/canceled() result whenever the lift actually
// arrives (or fails to). No executor thread is ever blocked for the
// duration of the move, and callers get real progress feedback instead of
// staring at a silent, uninterruptible service call.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "forklift_interfaces/action/set_lift_height.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

enum class LiftMode { IDLE, JOG, AUTO };

class LiftControllerNode : public rclcpp::Node
{
public:
  using SetLiftHeight = forklift_interfaces::action::SetLiftHeight;
  using GoalHandleSetLiftHeight = rclcpp_action::ServerGoalHandle<SetLiftHeight>;

  LiftControllerNode() : Node("lift_controller_node")
  {
    min_h_ = this->declare_parameter("min_height", 0.0);
    max_h_ = this->declare_parameter("max_height", 0.12);
    jog_speed_ = this->declare_parameter("jog_speed", 0.05);
    default_speed_ = this->declare_parameter("default_auto_speed", 0.08);
    max_speed_ = this->declare_parameter("max_speed", 0.15);
    tolerance_ = this->declare_parameter("height_tolerance", 0.01);
    timeout_s_ = this->declare_parameter("set_height_timeout_sec", 30.0);
    joint_name_ = this->declare_parameter("lift_joint_name", std::string("lift_joint"));
    control_loop_hz_ = this->declare_parameter("control_loop_hz", 20.0);

    sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = sub_group_;

    lift_up_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/lift_up", 10, std::bind(&LiftControllerNode::liftUpCallback, this, _1), opts);
    lift_down_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/lift_down", 10, std::bind(&LiftControllerNode::liftDownCallback, this, _1), opts);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&LiftControllerNode::jointStateCallback, this, _1), opts);

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/lift_cmd", 10);

    set_height_action_server_ = rclcpp_action::create_server<SetLiftHeight>(
      this, "set_height",
      std::bind(&LiftControllerNode::handleGoal, this, _1, _2),
      std::bind(&LiftControllerNode::handleCancel, this, _1),
      std::bind(&LiftControllerNode::handleAccepted, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_loop_hz_)),
      std::bind(&LiftControllerNode::controlLoop, this), sub_group_);

    RCLCPP_INFO(
      this->get_logger(), "lift_controller_node ready [%.2f, %.2f], joint='%s', mode=position (action)",
      min_h_, max_h_, joint_name_.c_str());
  }

private:
  void liftUpCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mode_mutex_);
    if (mode_ == LiftMode::AUTO) {return;}
    jog_up_ = msg->data;
    mode_ = (jog_up_ || jog_down_) ? LiftMode::JOG : LiftMode::IDLE;
  }

  void liftDownCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mode_mutex_);
    if (mode_ == LiftMode::AUTO) {return;}
    jog_down_ = msg->data;
    mode_ = (jog_up_ || jog_down_) ? LiftMode::JOG : LiftMode::IDLE;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == joint_name_) {
        if (i < msg->position.size()) {
          current_height_ = static_cast<float>(msg->position[i]);
          if (!have_height_) {
            std::lock_guard<std::mutex> lk(mode_mutex_);
            command_position_ = current_height_.load();
          }
          have_height_ = true;
        }
        return;
      }
    }
  }

  // ------------------------------------------------------------------
  // SetLiftHeight action server callbacks
  // ------------------------------------------------------------------
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const SetLiftHeight::Goal> goal)
  {
    if (goal->target_height < min_h_ || goal->target_height > max_h_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting SetLiftHeight goal: target_height out of range");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!have_height_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejecting SetLiftHeight goal: no feedback yet from /joint_states (check lift_joint_name)");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleSetLiftHeight>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleSetLiftHeight> goal_handle)
  {
    // Ramp-and-wait runs on its own thread -- handle_goal already
    // returned instantly, so the executor is never blocked by this.
    std::thread(&LiftControllerNode::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetLiftHeight> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetLiftHeight::Feedback>();
    auto result = std::make_shared<SetLiftHeight::Result>();

    const float target = goal->target_height;

    {
      std::lock_guard<std::mutex> lk(mode_mutex_);
      jog_up_ = jog_down_ = false;
      target_ = target;
      auto_speed_ = goal->speed > 0.0f ?
        std::min(goal->speed, static_cast<float>(max_speed_)) :
        static_cast<float>(default_speed_);
      mode_ = LiftMode::AUTO;
    }

    const auto start = this->now();
    bool success = false;
    std::string message;
    rclcpp::Rate feedback_rate(5.0);  // publish feedback at 5Hz while ramping

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        std::lock_guard<std::mutex> lk(mode_mutex_);
        mode_ = LiftMode::IDLE;
        result->success = false;
        result->message = "canceled";
        result->current_height = current_height_;
        goal_handle->canceled(result);
        return;
      }

      const float current = current_height_.load();
      const float diff = target - current;

      if (std::fabs(diff) <= tolerance_) {success = true; message = "reached"; break;}
      if (diff > 0 && current >= max_h_) {message = "blocked by max_height"; break;}
      if (diff < 0 && current <= min_h_) {message = "blocked by min_height"; break;}
      if ((this->now() - start).seconds() > timeout_s_) {message = "timed out"; break;}

      feedback->current_height = current;
      feedback->target_height = target;
      goal_handle->publish_feedback(feedback);

      feedback_rate.sleep();
    }

    {
      std::lock_guard<std::mutex> lk(mode_mutex_);
      mode_ = LiftMode::IDLE;  // holds command_position_ wherever the ramp last left it
    }

    result->success = success;
    result->message = message;
    result->current_height = current_height_;

    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void controlLoop()
  {
    if (!have_height_) {return;}

    std::lock_guard<std::mutex> lk(mode_mutex_);
    const float dt = static_cast<float>(1.0 / control_loop_hz_);

    if (mode_ == LiftMode::JOG) {
      if (jog_up_) {
        command_position_ = std::min(
          static_cast<float>(max_h_), command_position_ + static_cast<float>(jog_speed_) * dt);
      } else if (jog_down_) {
        command_position_ = std::max(
          static_cast<float>(min_h_), command_position_ - static_cast<float>(jog_speed_) * dt);
      }
    } else if (mode_ == LiftMode::AUTO) {
      const float step = std::min(auto_speed_, static_cast<float>(max_speed_)) * dt;
      if (command_position_ < target_) {
        command_position_ = std::min(target_, command_position_ + step);
      } else if (command_position_ > target_) {
        command_position_ = std::max(target_, command_position_ - step);
      }
    }

    auto cmd = std_msgs::msg::Float64();
    cmd.data = command_position_;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_up_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_down_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
  rclcpp_action::Server<SetLiftHeight>::SharedPtr set_height_action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr sub_group_;

  std::atomic<float> current_height_{0.0f};
  std::atomic<bool> have_height_{false};
  std::atomic<bool> jog_up_{false};
  std::atomic<bool> jog_down_{false};

  std::mutex mode_mutex_;
  LiftMode mode_{LiftMode::IDLE};
  float target_{0.0f};
  float auto_speed_{0.0f};
  float command_position_{0.0f};

  std::string joint_name_;
  double min_h_, max_h_, jog_speed_, default_speed_, max_speed_, tolerance_, timeout_s_, control_loop_hz_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LiftControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}