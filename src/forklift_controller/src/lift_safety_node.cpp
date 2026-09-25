#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "forklift_interfaces/msg/lift_limit_status.hpp"

using std::placeholders::_1;

// Master safety gate: /lift_enable = false on e-stop or a top+bottom-limit
// sensor fault. Per-direction limit blocking lives in lift_controller_node.
class LiftSafetyNode : public rclcpp::Node
{
public:
  LiftSafetyNode()
  : Node("lift_safety_node")
  {
    limit_sub_ = this->create_subscription<forklift_interfaces::msg::LiftLimitStatus>(
      "/lift_limit", 10, std::bind(&LiftSafetyNode::limitCallback, this, _1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(10).best_effort(),
      std::bind(&LiftSafetyNode::estopCallback, this, _1));

    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/lift_enable", 10);
    RCLCPP_INFO(this->get_logger(), "lift_safety_node ready");
  }

private:
  void limitCallback(const forklift_interfaces::msg::LiftLimitStatus::SharedPtr msg)
  {
    fault_ = msg->top_limit && msg->bottom_limit;
    evaluate();
  }

  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    estop_ = msg->data;
    evaluate();
  }

  void evaluate()
  {
    const bool enabled = !estop_ && !fault_;
    if (enabled != enabled_) {
      enabled_ = enabled;
      auto msg = std_msgs::msg::Bool();
      msg.data = enabled;
      enable_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "/lift_enable -> %s", enabled ? "true" : "false");
    }
  }

  rclcpp::Subscription<forklift_interfaces::msg::LiftLimitStatus>::SharedPtr limit_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  bool estop_{false};
  bool fault_{false};
  bool enabled_{true};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiftSafetyNode>());
  rclcpp::shutdown();
  return 0;
}