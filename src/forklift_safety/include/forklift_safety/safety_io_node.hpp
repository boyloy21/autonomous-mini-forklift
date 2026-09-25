#ifndef FORKLIFT_SAFETY__SAFETY_IO_NODE_HPP_
#define FORKLIFT_SAFETY__SAFETY_IO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "forklift_interfaces/msg/safety_io_input.hpp"
#include "forklift_interfaces/msg/safety_io_status.hpp"

namespace forklift_safety
{
    // For real hardware: passes through /safety_input (door switch, light curtain, safety relay, exteranl signals)
    // into /safety_io with an
    // aggregated all_ok flag for safety_manager_node.
    class SafetyIoNode : public rclcpp::Node
    {
        public: 
            SafetyIoNode();
        
        private:
            rclcpp::Subscription<forklift_interfaces::msg::SafetyIOInput>::SharedPtr input_sub_;
            rclcpp::Publisher<forklift_interfaces::msg::SafetyIOStatus>::SharedPtr status_pub_;
    };

}
#endif