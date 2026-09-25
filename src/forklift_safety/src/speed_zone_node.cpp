#include "forklift_safety/speed_zone_node.hpp"

namespace forklift_safety
{
    SpeedZoneNode::SpeedZoneNode() : Node("speed_zones_node")
    {
        default_max_speed_ = declare_parameter("default_speed_max", 0.5);
        const auto zone_names = declare_parameter("zone_names", std::vector<std::string>{});

        for (const auto & name : zone_names) {
            SpeedZone zone;
            zone.name = name;
            zone.min_x = declare_parameter(name + ".min_x", 0.0);
            zone.max_x = declare_parameter(name + ".max_x", 0.0);
            zone.min_y = declare_parameter(name + ".min_y", 0.0);
            zone.max_y = declare_parameter(name + ".max_y", 0.0);
            zone.max_speed = declare_parameter(name + ".max_speed", default_max_speed_);
            zones_.push_back(zone);
        }

        max_speed_pub_ = create_publisher<std_msgs::msg::Float32>("/max_speed", 10);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr) {
                 // Reserved for future map-aware zone lookup; not used yet.
            });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                const double speed = lookupMaxSpeed(msg->pose.position.x, msg->pose.position.y);
                std_msgs::msg::Float32 out;
                out.data = static_cast<float>(speed);
                max_speed_pub_->publish(out);
            });

        RCLCPP_INFO(get_logger(), "speed_zone_node ready (%zu zones)", zones_.size());
    }

    double SpeedZoneNode::lookupMaxSpeed(double x, double y) const
    {
        for (const auto & zone : zones_) {
            if (x >= zone.min_x && x <= zone.max_x && y >= zone.min_y && y <= zone.max_y) {
                return zone.max_speed;
            }
        }
        return default_max_speed_;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<forklift_safety::SpeedZoneNode>());
    rclcpp::shutdown();
    return 0;
}