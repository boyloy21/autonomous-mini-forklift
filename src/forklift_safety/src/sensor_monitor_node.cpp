#include "forklift_safety/sensor_monitor_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace forklift_safety
{
    SensorMonitorNode::SensorMonitorNode() : Node("sensor_monitor_node")
    {
        lidar_timeout_ = declare_parameter("lidar_timeout_sec", 1.0);
        imu_timeout_ = declare_parameter("imu_timeout_sec", 0.5);
        ultrasonic_timeout_ = declare_parameter("ultrasonic_timeout_sec", 1.0);
        camera_timeout_ = declare_parameter("camera_timeout_sec", 2.0);
        encoder_timeout_ = declare_parameter("encoder_timeout_sec", 0.5);
        const double rate = declare_parameter("check_rate_hz", 5.0);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "forklift/scan", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_scan_ = this->now();
                have_scan_ = true;
            });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "forklift/imu", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_imu_ = this->now();
                have_imu_ = true;
            });
        
        ultrasonic_sub_ = create_subscription<forklift_interfaces::msg::UltrasonicArray>(
            "forklift/ultrasonic", 10,
            [this](const forklift_interfaces::msg::UltrasonicArray::SharedPtr) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_ultrasonic_ = this->now();
                have_ultrasonic_ = true;
            });

        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "forklift/camera/image_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_camera_ = this->now();
                have_camera_ = true;
            });

        encoder_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "forklift/encoder", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr) {
                last_encoder_ = this->now();
                have_encoder_ = true;
            });

        status_pub_ = create_publisher<forklift_interfaces::msg::SensorStatus>("forklift/sensor_status", 10);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
            std::bind(&SensorMonitorNode::checkTick, this));
        
        RCLCPP_INFO(get_logger(), "sensor_monitor_node ready");
        
    }

    bool SensorMonitorNode::isFresh(const rclcpp::Time & last , double timeout_sec) const
    {
        return (this->now() - last).seconds() <= timeout_sec;
    }

    void SensorMonitorNode::checkTick()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        forklift_interfaces::msg::SensorStatus status;
        status.lidar_ok = have_scan_ && isFresh(last_scan_, lidar_timeout_);
        status.imu_ok = have_imu_ && isFresh(last_imu_, imu_timeout_);
        status.ultrasonic_ok = have_ultrasonic_ && isFresh(last_ultrasonic_, ultrasonic_timeout_);
        status.camera_ok = have_camera_ && isFresh(last_camera_, camera_timeout_);
        status.encoder_ok = have_encoder_ && isFresh(last_encoder_, encoder_timeout_);
        status.all_ok = status.lidar_ok && status.imu_ok && status.ultrasonic_ok && status.camera_ok && status.encoder_ok;

        if (!status.all_ok) {
            std::string missing;
            if (!status.lidar_ok) {missing += "lidar ";}
            if (!status.imu_ok) {missing += "imu ";}
            if (!status.ultrasonic_ok) {missing += "ultrasonic ";}
            if (!status.camera_ok) {missing += "camera ";}
            if (!status.encoder_ok) {missing += "encoder ";}
            status.message = "Stale/missing: " + missing;
        } else {
            status.message = "All sensor nominal";
        }

        status_pub_->publish(status);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::SensorMonitorNode>());
    rclcpp::shutdown();
    return 0;
}