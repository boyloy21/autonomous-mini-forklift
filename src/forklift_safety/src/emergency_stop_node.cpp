#include "forklift_safety/emergency_stop_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace forklift_safety
{
    EmergencyStopNode::EmergencyStopNode() : Node("emergency_stop_node")
    {
        button_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop_button", rclcpp::QoS(10).best_effort(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                button_active_ = msg->data;
                evaluate();
            });
        
        web_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/web/emergency_stop", rclcpp::QoS(10).best_effort(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                web_active_ = msg->data;
                evaluate();
            });

        estop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "/emergency_stop/reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (button_active_ || web_active_) {
                        response->success = false;
                        response->message = "Cannot reset: an E-Stop source is still active";
                        return;
                    }
                    latched_ = false;
                    response->success = true;
                    response->message = "Emergency stop reset";
                    RCLCPP_INFO(get_logger(), "Emergency stop reset by operator");
                    evaluate();
                   });
        // Publish initial (cleared) state.
        std_msgs::msg::Bool initial;
        initial.data = false;
        estop_pub_->publish(initial);

        RCLCPP_INFO(get_logger(), "emergency_stop_node ready");
    }

    void EmergencyStopNode::evaluate()
    {
        // Caller holds mutex_
        if (button_active_ || web_active_) {
            latched_ = true;
        }
        std_msgs::msg::Bool msg;
        msg.data = latched_;
        estop_pub_->publish(msg);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::EmergencyStopNode>());
    rclcpp::shutdown();
    return 0;
}