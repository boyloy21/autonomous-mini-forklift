#include "forklift_safety/safety_manager_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace forklift_safety
{
    SafetyManagerNode::SafetyManagerNode() : Node("safety_manager_node") {
        const auto max_speed_topic = declare_parameter("max_speed_topic", std::string("/max_speed"));
        declare_parameter("status_timeout_sec", 2.0);

        estop_sub_ = create_subscription<std_msgs::msg::Bool>("/emergency_stop", rclcpp::QoS(10).best_effort(),
            [this](const std_msgs::msg::Bool::SharedPtr msg){
                std::lock_guard<std::mutex> lock(mutex_);
                estop_ = msg->data;
                evaluate();
            });

        field_sub_ = create_subscription<forklift_interfaces::msg::ProtectiveFieldStatus>(
            "/protective_field", 10,
            [this](const forklift_interfaces::msg::ProtectiveFieldStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                field_ = *msg;
                evaluate();
            });

        obstacle_sub_ = create_subscription<forklift_interfaces::msg::ObstacleState>(
            "/obtacle_state", 10,
            [this](const forklift_interfaces::msg::ObstacleState::SharedPtr msg){
                std::lock_guard<std::mutex> lock(mutex_);
                obstacle_ = *msg;
                evaluate();
            });
        
        max_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
            max_speed_topic, 10,
            [this](const std_msgs::msg::Float32::SharedPtr) {
                // Currently informational only; motion gating comes from the other
                // four inputs. Kept as its own subscription so it's easy to fold
                // zone-awareness into evaluate() later if needed.
            });


        safety_io_sub_ = create_subscription<forklift_interfaces::msg::SafetyIOStatus>(
            "/safety_io", 10,
            [this](const forklift_interfaces::msg::SafetyIOStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                safety_io_ = *msg;
                have_safety_io_ = true;
                evaluate();
            });

        status_pub_ = create_publisher<forklift_interfaces::msg::SafetyStatus>("/safety/status", 10);
        allow_motion_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/allow_motion", 10);

        RCLCPP_INFO(get_logger(), "safety_manager_node ready");
        
    }

    void SafetyManagerNode::evaluate()
    {
        // Caller hold mutex_
        using S = forklift_interfaces::msg::SafetyStatus;
        uint8_t state = S::STATE_SAFE;
        std::string message = "All clear";
        bool allow_motion = true;

        const bool io_fault = have_safety_io_ && !safety_io_.all_ok;

        if (estop_) {
            state = S::STATE_EMERGENCY;
            message = "Emergency stop enaged";
            allow_motion = false;
        } else if (obstacle_.collision_detected) {
            state = S::STATE_EMERGENCY;
            message = "Collision detected:" + obstacle_.message;
            allow_motion = false;
        } else if (io_fault) {
            state = S::STATE_STOP;
            message = "Safety IO fault: " + safety_io_.message;
            allow_motion = false;
        } else if (field_.stop_zone) {
            state = S::STATE_STOP;
            message = "Object in stop zone";
            allow_motion = false;
        } else if (field_.slow_zone || obstacle_.obstacle_detected) {
            state = S::STATE_SLOWDOWN;
            message = "Object in slow zone / obstacle nearby";
            allow_motion = true;
        } else if (field_.warning_zone) {
            state = S::STATE_WARNING;
            message = "Object in warning zone";
            allow_motion = true;
        } else if (last_state_ == S::STATE_EMERGENCY) {
            state = S::STATE_RESET;
            message = "Recovering from emergency stop";
            allow_motion = false;
        }

        forklift_interfaces::msg::SafetyStatus status;
        status.state = state;
        status.allow_motion = allow_motion;
        status.message = message;
        status_pub_->publish(status);

        std_msgs::msg::Bool motion;
        motion.data = allow_motion;
        allow_motion_pub_->publish(motion);

        last_state_ = state;
    }
}

int main(int argc, char ** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::SafetyManagerNode>());
    rclcpp::shutdown();
    return 0;
}