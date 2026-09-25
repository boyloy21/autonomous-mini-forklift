#ifndef FORKLIFT_SAFETY__OBSTACLE_MONITOR_NODE_HPP_
#define FORKLIFT_SAFETY__OBSTACLE_MONITOR_NODE_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "forklift_interfaces/msg/ultrasonic_array.hpp"
#include "forklift_interfaces/msg/bumper_state.hpp"
#include "forklift_interfaces/msg/obstacle_state.hpp"

namespace forklift_safety
{
    // fuses /scan, /ultrasonics, /bumper (and /depth_camera as a presense)
    // check only - no image processing here) into /obstacle_state.
    // collision_detected comes solely from the bumper (physical contact);
    // obstacle_detected/path_blocked come from lidar+ultrasonic proximity.

    class ObstacleMonitorNode : public rclcpp::Node 
    {
        public:
            ObstacleMonitorNode();
        
        private: 
            void evaluate();

            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Subscription<forklift_interfaces::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
            rclcpp::Subscription<forklift_interfaces::msg::BumperState>::SharedPtr bumper_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_camera_sub_;
            rclcpp::Publisher<forklift_interfaces::msg::ObstacleState>::SharedPtr obstacle_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::mutex mutex_;
            float min_lidar_range_{99.0f};
            float min_ultrasonic_range_{99.0f};
            bool bumper_triggered_{false};
            double obstacle_distance_;
    };
}
#endif