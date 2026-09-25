// PS4 controller teleop for the Mini Forklift AMR.
// - Left stick vertical (axes[1])  -> linear.x
// - Right stick horizontal (axes[3]) -> angular.z
// - L1 (buttons[4]) / R1 (buttons[5])   -> increase linear / angular speed
// - L2 (buttons[7]) / R2 (buttons[6])   -> decrease linear / angular speed
//   (button indices preserved from the original Python script's mapping)
// - Triangle (buttons[2]) / Cross (buttons[0]) -> raise / lower the fork lift
//
// Drive command  -> geometry_msgs/msg/Twist on "forklift/cmd_vel"
// Lift command   -> std_msgs/msg/Float64 on "forklift/lift_cmd"
//   (bridged to Ignition's native JointPositionController plugin on
//    lift_joint, NOT through ros2_control -- ros2_control's velocity AND
//    position interfaces were both confirmed non-functional for this
//    prismatic joint on this ign_ros2_control build, while the native
//    Ignition plugin was proven to work via direct testing. Target
//    position in meters, clamped to [lift_lower_limit_, lift_upper_limit_],
//    accumulated while raise/lower is held.)

#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class PS4ControlNode : public rclcpp::Node
{
public:
  PS4ControlNode()
  : Node("ps4_control_node")
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&PS4ControlNode::joyCallback, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "forklift/cmd_vel", 10);

    lift_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "forklift/lift_cmd", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&PS4ControlNode::publishCmd, this));  // 10 Hz
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // --- Speed adjustment buttons (mapping preserved from the original
    //     Python script) ---
    bool increase_linear = msg->buttons.size() > 4 && msg->buttons[4];   // L1
    bool increase_angular = msg->buttons.size() > 5 && msg->buttons[5];  // R1
    bool decrease_angular = msg->buttons.size() > 6 && msg->buttons[6];  // R2
    bool decrease_linear = msg->buttons.size() > 7 && msg->buttons[7];   // L2

    if (increase_linear) {speed_linear_ += 0.1;}
    if (decrease_linear) {speed_linear_ -= 0.1;}
    if (increase_angular) {speed_angular_ += 0.1;}
    if (decrease_angular) {speed_angular_ -= 0.1;}

    // --- Drive axes ---
    vx_ = msg->axes.size() > 1 ? msg->axes[1] : 0.0;      // left stick vertical
    omega_ = msg->axes.size() > 3 ? msg->axes[3] : 0.0;   // right stick horizontal

    // --- Lift control: Triangle raises, Cross lowers ---
    // Target position accumulates while held, clamped to the joint's
    // travel range, and holds its last value when neither is pressed
    // (the JointPositionController plugin holds position via its own PID
    // once commanded, same as a real linear actuator would).
    bool lift_up = msg->buttons.size() > 2 && msg->buttons[2];    // Triangle
    bool lift_down = msg->buttons.size() > 0 && msg->buttons[0];  // Cross

    if (lift_up && !lift_down) {
      lift_position_ = std::clamp(
        lift_position_ + lift_step_, lift_lower_limit_, lift_upper_limit_);
    } else if (lift_down && !lift_up) {
      lift_position_ = std::clamp(
        lift_position_ - lift_step_, lift_lower_limit_, lift_upper_limit_);
    }
  }

  void publishCmd()
  {
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = vx_ * speed_linear_;
    cmd_msg.linear.y = vy_ * speed_linear_;
    cmd_msg.angular.z = omega_ * speed_angular_;

    cmd_msg.linear.x = std::clamp(cmd_msg.linear.x, -limit_linear_speed_, limit_linear_speed_);
    cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -limit_angular_speed_, limit_angular_speed_);

    cmd_publisher_->publish(cmd_msg);

    std_msgs::msg::Float64 lift_msg;
    lift_msg.data = lift_position_;
    lift_publisher_->publish(lift_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing: Vx=%.2f, Vy=%.2f, Omega=%.2f, LiftPos=%.3f",
      cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z, lift_position_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Drive state
  double vx_ = 0.0;
  double vy_ = 0.0;
  double omega_ = 0.0;

  double speed_linear_ = 0.3;
  double speed_angular_ = 1.0;

  double limit_linear_speed_ = 1.5;
  double limit_angular_speed_ = 2.0;

  // Lift state
  // NOTE: lift_lower_limit_/lift_upper_limit_ must match
  // lift_lower_limit/lift_upper_limit in forklift_description/urdf/const.xacro
  // (currently 0.0 / lift_travel=0.15) -- update both together if that
  // changes.
  double lift_position_ = 0.0;
  double lift_step_ = 0.005;  // meters per joystick message while held
  double lift_lower_limit_ = 0.0;
  double lift_upper_limit_ = 0.15;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PS4ControlNode>());
  rclcpp::shutdown();
  return 0;
}