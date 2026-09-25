#ifndef FORKLIFT_SAFETY__PROTECTIVE_FIELD_NODE_HPP_
#define FORKLIFT_SAFETY__PROTECTIVE_FIELD_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "forklift_interfaces/msg/protective_field_status.hpp"

namespace forklift_safety
{
    // Derives nested warning/slow/stop zones form the minimum LiDAR range
    // within a forward angular cone (+/- front_angle_deg around 0 rad).
    class ProtectiveFieldNode : public rclcpp::Node
    {
        public: 
            ProtectiveFieldNode();
        
        private:
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<forklift_interfaces::msg::ProtectiveFieldStatus>::SharedPtr field_pub_;

            double warning_distance_;
            double slow_distance_;
            double stop_distance_;
            double front_angle_deg_;
    };

}
#endif