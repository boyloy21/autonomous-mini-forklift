#ifndef FORKLIFT_SAFETY__EMERGENCY_STOP_NODE_HPP_
#define FORKLIFT_SAFETY__EMERGENCY_STOP_NODE_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace forklift_safety
{
    // Latches /emergency_stop = true if either the physical button or 
    // the dashboard/wireless E-Stop is active. Once latched, only clears via the
    // /emergency_stop/reset service, and only if both sources have released
    // (can't reset while physically held) - "reset after operator"
    // confirmation "per spec: the service call itself IS that confirmation"
    class EmergencyStopNode : public rclcpp::Node
    {
        public:
            EmergencyStopNode();
        
        private:
            void evaluate();

            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr web_sub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

            std::mutex mutex_;
            bool button_active_{false};
            bool web_active_{false};
            bool latched_{false};
    };
}

#endif