// Keyboard teleop for the Mini Forklift AMR.
//
// Controls:
//   w / x       -> drive forward / backward
//   a / d       -> turn left / right
//   s (or space)-> stop driving (does NOT reset lift position -- the fork
//                  holds wherever it last was, same as pausing an actuator)
//   q / z       -> increase / decrease linear speed
//   e / c       -> increase / decrease angular speed
//   u / j       -> raise / lower the fork lift (latches like w/a/d/x --
//                  press the opposite lift key, or 'k', to stop adjusting it)
//   k           -> stop adjusting lift (hold current position)
//   Ctrl+C      -> quit
//
// Drive command  -> geometry_msgs/msg/Twist on "forklift/cmd_vel"
// Lift command   -> std_msgs/msg/Float64 on "forklift/lift_cmd"
//   (bridged to Ignition's native JointPositionController plugin on
//    lift_joint, NOT through ros2_control -- ros2_control's velocity AND
//    position interfaces were both confirmed non-functional for this
//    prismatic joint on this ign_ros2_control build, while the native
//    Ignition plugin was proven to work via direct testing. Target
//    position in meters, clamped to [lift_lower_limit_, lift_upper_limit_],
//    accumulated once per publish tick while raise/lower is latched on.)
//
// Reads raw keypresses from stdin in a background thread (terminal is put
// into non-canonical, no-echo mode so keys register without pressing
// Enter) and republishes the latest command state at a fixed rate from a
// wall timer, mirroring ps4_control_node.cpp's structure.

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace
{
// RAII helper: puts the terminal into raw (non-canonical, no-echo) mode on
// construction and restores the original settings on destruction, so
// Ctrl+C / normal shutdown never leaves the user's terminal in a broken
// state.
class RawTerminalGuard
{
public:
  RawTerminalGuard()
  {
    tcgetattr(STDIN_FILENO, &old_settings_);
    termios raw = old_settings_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~RawTerminalGuard()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
  }

private:
  termios old_settings_{};
};
}  // namespace

class KeyboardControlNode : public rclcpp::Node
{
public:
  KeyboardControlNode()
  : Node("keyboard_control_node")
  {
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    lift_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/lift_cmd", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&KeyboardControlNode::publishCmd, this));  // 10 Hz

    printHelp();

    running_ = true;
    key_thread_ = std::thread(std::bind(&KeyboardControlNode::keyLoop, this));
  }

  ~KeyboardControlNode() override
  {
    running_ = false;
    if (key_thread_.joinable()) {
      key_thread_.join();
    }
  }

private:
  void printHelp()
  {
    RCLCPP_INFO(this->get_logger(), "Mini Forklift keyboard teleop");
    RCLCPP_INFO(this->get_logger(), "  w/x : forward/backward   a/d : turn left/right");
    RCLCPP_INFO(this->get_logger(), "  s or space : stop drive  q/z : linear speed +/-");
    RCLCPP_INFO(this->get_logger(), "  e/c : angular speed +/-  u/j : lift up/down (latches)");
    RCLCPP_INFO(this->get_logger(), "  k : stop adjusting lift (holds current position)");
    RCLCPP_INFO(this->get_logger(), "  Ctrl+C : quit");
  }

  // Runs on a background thread; reads one raw keypress at a time and
  // updates the shared command state under a mutex.
  void keyLoop()
  {
    RawTerminalGuard raw_guard;
    char c;
    while (running_ && rclcpp::ok()) {
      if (::read(STDIN_FILENO, &c, 1) <= 0) {
        continue;
      }

      std::lock_guard<std::mutex> lock(state_mutex_);
      switch (c) {
        case 'w':
          vx_ = 1.0;
          break;
        case 'x':
          vx_ = -1.0;
          break;
        case 'a':
          omega_ = 1.0;
          break;
        case 'd':
          omega_ = -1.0;
          break;
        case 's':
        case ' ':
          vx_ = 0.0;
          omega_ = 0.0;
          break;
        case 'q':
          speed_linear_ += 0.1;
          break;
        case 'z':
          speed_linear_ = std::max(0.0, speed_linear_ - 0.1);
          break;
        case 'e':
          speed_angular_ += 0.1;
          break;
        case 'c':
          speed_angular_ = std::max(0.0, speed_angular_ - 0.1);
          break;
        case 'u':
          lift_direction_ = 1;
          break;
        case 'j':
          lift_direction_ = -1;
          break;
        case 'k':
          lift_direction_ = 0;
          break;
        default:
          // Unrecognized key -- ignore, keep last command state.
          break;
      }
    }
  }

  void publishCmd()
  {
    double vx, omega, speed_linear, speed_angular, lift_position;
    int lift_direction;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      vx = vx_;
      omega = omega_;
      speed_linear = speed_linear_;
      speed_angular = speed_angular_;
      lift_direction = lift_direction_;

      if (lift_direction != 0) {
        lift_position_ = std::clamp(
          lift_position_ + lift_direction * lift_step_,
          lift_lower_limit_, lift_upper_limit_);
      }
      lift_position = lift_position_;
    }

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = vx * speed_linear;
    cmd_msg.angular.z = omega * speed_angular;

    cmd_msg.linear.x = std::clamp(cmd_msg.linear.x, -limit_linear_speed_, limit_linear_speed_);
    cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -limit_angular_speed_, limit_angular_speed_);

    cmd_publisher_->publish(cmd_msg);

    std_msgs::msg::Float64 lift_msg;
    lift_msg.data = lift_position;
    lift_publisher_->publish(lift_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing: Vx=%.2f, Omega=%.2f, LiftPos=%.3f (lin_speed=%.2f, ang_speed=%.2f)",
      cmd_msg.linear.x, cmd_msg.angular.z, lift_position, speed_linear, speed_angular);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread key_thread_;
  std::atomic<bool> running_{false};

  std::mutex state_mutex_;

  // Drive state (protected by state_mutex_)
  double vx_ = 0.0;
  double omega_ = 0.0;
  double speed_linear_ = 0.6;
  double speed_angular_ = 1.0;

  double limit_linear_speed_ = 1.5;
  double limit_angular_speed_ = 2.0;

  // Lift state (protected by state_mutex_)
  // NOTE: lift_lower_limit_/lift_upper_limit_ must match
  // lift_lower_limit/lift_upper_limit in forklift_description/urdf/const.xacro
  // (currently 0.0 / lift_travel=0.15) -- update both together if that
  // changes.
  double lift_position_ = 0.0;
  int lift_direction_ = 0;   // -1, 0, or 1; latched by u/j/k
  double lift_step_ = 0.005;  // meters per publish tick (10 Hz) while latched
  double lift_lower_limit_ = 0.0;
  double lift_upper_limit_ = 0.15;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardControlNode>());
  rclcpp::shutdown();
  return 0;
}