#ifndef FORKLIFT_SAFETY__SENSOR_MONITOR_NODE_HPP_
#define FORKLIFT_SAFETY__SENSOR_MONITOR_NODE_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "forklift_interfaces/msg/ultrasonic_array.hpp"
#include "forklift_interfaces/msg/sensor_status.hpp"

namespace forklift_safety
{
    // Tracks last-message time per sensor; declare a sensor "down" if i
    // hasn't published within its configured timeout. Publishes /sensor_status
    // at check_rate_hz. "/encoder" is read as nav_msgs/Odometry (wheel odom)
    class SensorMonitorNode : public rclcpp::Node
    {
        public:
            SensorMonitorNode();
        
        private:
            void checkTick();
            bool isFresh(const rclcpp::Time & last, double timeout_sec) const;

            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            rclcpp::Subscription<forklift_interfaces::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_;
            rclcpp::Publisher<forklift_interfaces::msg::SensorStatus>::SharedPtr status_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::mutex mutex_;
            rclcpp::Time last_scan_, last_imu_, last_ultrasonic_, last_camera_, last_encoder_;
            bool have_scan_{false}, have_imu_{false}, have_ultrasonic_{false}, have_camera_{false}, have_encoder_{false};

            double lidar_timeout_, imu_timeout_, ultrasonic_timeout_, camera_timeout_, encoder_timeout_;
    };

}

#endif  // FORKLIFT_SAFETY__SENSOR_MONITOR_NODE_HPP_