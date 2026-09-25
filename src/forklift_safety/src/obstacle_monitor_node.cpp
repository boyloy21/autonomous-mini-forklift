#include "forklift_safety/obstacle_monitor_node.hpp"
#include <algorithm>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;

namespace forklift_safety
{
    ObstacleMonitorNode::ObstacleMonitorNode() : Node("obstacle_monitor_node")
    {
        obstacle_distance_ = declare_parameter("obstacle_distance", 0.5);
        const double rate = declare_parameter("check_rate_hz", 10.0);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "forklift/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                float min_r = std::numeric_limits<float>::infinity();
                for (const float r : msg->ranges) {
                    if (r > msg->range_min && r < msg->range_max) {min_r = std::min(min_r, r);}
                }
                min_lidar_range_ = std::isfinite(min_r) ? min_r : 99.0f;
            }); 
        
        ultrasonic_sub_ = create_subscription<forklift_interfaces::msg::UltrasonicArray>(
            "/ultrasonic", 10,
            [this](const forklift_interfaces::msg::UltrasonicArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                float min_r = 99.0f;
                for (const float r : msg->ranges) {min_r = std::min(min_r, r);}
                min_ultrasonic_range_ = min_r;
            });

        bumper_sub_ = create_subscription<forklift_interfaces::msg::BumperState>(
            "/bumper", 10,
            [this](const forklift_interfaces::msg::BumperState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                bumper_triggered_ = msg->front || msg->rear || msg->left || msg->right;
            });

        depth_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/depth_camera", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr) {
            // Presence-only: confirms the depth camera is streaming. Full
            // point-cloud/segmentation-based obstacle detection belongs in a
            // dedicated perception node (e.g. forklift_pallet_vision), not here.
            });

        obstacle_pub_ = create_publisher<forklift_interfaces::msg::ObstacleState>("/obstacle_state", 10);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
            std::bind(&ObstacleMonitorNode::evaluate, this));
        
        RCLCPP_INFO(get_logger(), "obstacle_monitor_node ready");
    }

    void ObstacleMonitorNode::evaluate()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const float min_range = std::min(min_lidar_range_, min_ultrasonic_range_);
        const bool obstacle_detected = min_range < static_cast<float>(obstacle_distance_);

        forklift_interfaces::msg::ObstacleState status;
        status.collision_detected = bumper_triggered_;
        status.obstacle_detected = obstacle_detected || bumper_triggered_;
        status.path_blocked = status.obstacle_detected;
        status.distance = min_range;
        status.source = bumper_triggered_ ? "bumper" : (min_lidar_range_ <= min_ultrasonic_range_ ? "lidar" : "ultrasonic");
        status.message = status.collision_detected ? "Collision (bumper triggered)" : 
                        status.obstacle_detected ? "Obstacle within" + std::to_string(obstacle_distance_) + "m" : "Clear";
        obstacle_pub_->publish(status);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::ObstacleMonitorNode>());
    rclcpp::shutdown();
    return 0;
}

