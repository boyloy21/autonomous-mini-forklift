#include "forklift_safety/protective_field_node.hpp"
#include <cmath>
#include <limits>

namespace forklift_safety
{
    ProtectiveFieldNode::ProtectiveFieldNode() : Node("protective_field_node")
    {
        warning_distance_ = declare_parameter("warning_distance", 1.5);
        slow_distance_ = declare_parameter("slow_distance", 0.8);
        stop_distance_ = declare_parameter("stop_distance", -0.4);
        front_angle_deg_ = declare_parameter("front_angle_de", 90.0);

        field_pub_ = create_publisher<forklift_interfaces::msg::ProtectiveFieldStatus>(
            "/protective_field", 10
        );

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "forklift/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                const double half_cone = front_angle_deg_ * M_PI / 180.0;
                float min_r = std::numeric_limits<float>::infinity();

                for (size_t i = 0; i < msg->ranges.size(); ++i) {
                    const double angle = msg->angle_min + i * msg->angle_increment;
                    if (std::fabs(angle) > half_cone) {continue;}
                    const float r = msg->ranges[i];
                    if (r > msg->range_min && r < msg->range_max) {min_r = std::min(min_r, r);}
                }
                const float closest = std::isfinite(min_r) ? min_r : 99.0f;

                forklift_interfaces::msg::ProtectiveFieldStatus status;
                status.closest_distance = closest;
                status.warning_zone = closest < warning_distance_;
                status.slow_zone = closest < slow_distance_;
                status.stop_zone = closest < stop_distance_;
                status.message = status.stop_zone ? "STOP zone" : status.slow_zone ? "SLOW zone" : status.warning_zone ? "WARNING zone" : "Clear";
                field_pub_->publish(status);
            });
        
        RCLCPP_INFO(
            get_logger(), "protective_field_node ready (warning=%.2f slow=%.2f stop=%.2f)",
            warning_distance_, slow_distance_, stop_distance_);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::ProtectiveFieldNode>());
    rclcpp::shutdown();
    return 0;
}