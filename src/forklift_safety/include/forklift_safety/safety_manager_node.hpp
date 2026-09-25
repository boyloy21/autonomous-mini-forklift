#ifndef FORKLIFT_SAFETY__SAFETY_MANAGER_NODE_HPP_
#define FORKLIFT_SAFETY__SAFETY_MANAGER_NODE_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "forklift_interfaces/msg/safety_status.hpp"
#include "forklift_interfaces/msg/protective_field_status.hpp"
#include "forklift_interfaces/msg/obstacle_state.hpp"
#include "forklift_interfaces/msg/safety_io_status.hpp"

namespace forklift_safety
{
    // Core decision node, combines e-stop, protective field, Obstacle state,
    // speed zone, and safety-IO into one SAFE/WARNING/SLOWDOWN/STOP/EMERGENCY
    // state and a single allow_motion gate.

    // NOTE: spec lists "/speed_zone" as an input, but speed_zone_node (as built) publishes "/max_speed" - subscribing to that instead (name configuable via max_speed_topic param). 
    // Safety_manager_node itself doesn't republish speed; it only factors zone info into its status
    // message, motion gating comes from e-stop/obstacle/field/IO.
    class SafetyManagerNode : public rclcpp::Node
    {
        public:
            SafetyManagerNode();
        private:
            void evaluate();

            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
            rclcpp::Subscription<forklift_interfaces::msg::ProtectiveFieldStatus>::SharedPtr field_sub_;
            rclcpp::Subscription<forklift_interfaces::msg::ObstacleState>::SharedPtr obstacle_sub_;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_speed_sub_;
            rclcpp::Subscription<forklift_interfaces::msg::SafetyIOStatus>::SharedPtr safety_io_sub_;
            rclcpp::Publisher<forklift_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allow_motion_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::mutex mutex_;
            bool estop_{false};
            forklift_interfaces::msg::ProtectiveFieldStatus field_;
            forklift_interfaces::msg::ObstacleState obstacle_;
            forklift_interfaces::msg::SafetyIOStatus safety_io_;
            bool have_safety_io_{false};
            uint8_t last_state_{forklift_interfaces::msg::SafetyStatus::STATE_SAFE};
    };
}  // namespace forklift_safety
#endif // FORKLIFT_