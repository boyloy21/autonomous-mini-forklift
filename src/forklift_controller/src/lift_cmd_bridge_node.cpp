// src/lift_cmd_bridge_node.cpp
//
// Bridges the rich forklift_interfaces/msg/LiftMotorCommand on
// /lift_motor_cmd into a plain std_msgs/msg/Float64 velocity command on
// /lift_cmd - for a ros2_control velocity controller (or anything else
// that only understands a bare Float64), without touching
// lift_controller_node/lift_driver_node's existing message type.

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "forklift_interfaces/msg/lift_motor_command.hpp"

class LiftCmdBridgeNode : public rclcpp::Node
{
public:
  LiftCmdBridgeNode() : Node("lift_cmd_bridge_node")
  {
    pub_ = this->create_publisher<std_msgs::msg::Float64>("/lift_cmd", 10);
    sub_ = this->create_subscription<forklift_interfaces::msg::LiftMotorCommand>(
      "/lift_motor_cmd", 10,
      [this](const forklift_interfaces::msg::LiftMotorCommand::SharedPtr msg) {
        auto out = std_msgs::msg::Float64();
        // `enable` is already honored upstream (lift_controller_node
        // zeroes velocity when disabled) - zeroing here too is a
        // defensive second layer, in case anything else ever publishes
        // directly to /lift_motor_cmd bypassing that safety logic.
        out.data = msg->enable ? static_cast<double>(msg->velocity) : 0.0;
        pub_->publish(out);
      });
    RCLCPP_INFO(this->get_logger(), "lift_cmd_bridge_node ready: /lift_motor_cmd -> /lift_cmd");
  }

private:
  rclcpp::Subscription<forklift_interfaces::msg::LiftMotorCommand>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiftCmdBridgeNode>());
  rclcpp::shutdown();
  return 0;
}