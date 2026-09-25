#include "forklift_safety/safety_io_node.hpp"

namespace forklift_safety
{
    SafetyIoNode::SafetyIoNode() : Node("safety_io_node")
    {
        status_pub_ = create_publisher<forklift_interfaces::msg::SafetyIOStatus>("/safety_io", 10);

        input_sub_ = create_subscription<forklift_interfaces::msg::SafetyIOInput>(
            "/safety_input", 10,
            [this](const forklift_interfaces::msg::SafetyIOInput::SharedPtr msg) {
                forklift_interfaces::msg::SafetyIOStatus status;
                status.door_ok = msg->door_switch_closed;
                status.light_curtain_ok = msg->light_curtain_clear;
                status.relay_ok = msg->safety_relay_ok;
                status.external_ok = msg->external_ok;
                status.all_ok = status.door_ok && status.light_curtain_ok && status.relay_ok && status.external_ok;

                if (!status.all_ok) {
                    std::string missing;
                    if (!status.door_ok) {missing += "door ";}
                    if (!status.light_curtain_ok) {missing += "light_curtain ";}
                    if (!status.relay_ok) {missing += "relay ";}
                    if (!status.external_ok) {missing += "external ";}
                    status.message = "Fault: " + missing;
                } else {
                    status.message = "All safety IO nominal";
                }
                status_pub_->publish(status);
            });
        RCLCPP_INFO(get_logger(), "safety_io_node ready");
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::SafetyIoNode>());
    rclcpp::shutdown();
    return 0;
}