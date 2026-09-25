#ifndef FORKLIFT_SAFETY__SPEED_ZONE_NODE_HPP_
#define FORKLIFT_SAFETY__SPEED_ZONE_NODE_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace forklift_safety
{
    struct SpeedZone
    {
        std::string name;
        double min_x, max_x, min_y, max_y;
        double max_speed;
    };

    // Looks up robot_pose against named rectangular zone (config-defined,)
    // not derived from /map semantics) and publishes the matching zone's
    // max_speed. /map is subscribed for future map-aware zone lookup but
    // not currently used to compute anything - documented assumption since
    // semantic-zone-from-occupancy-grid parsing isn't standardized.

    class SpeedZoneNode : public rclcpp::Node 
    {
        public: 
            SpeedZoneNode();
        private:
            double lookupMaxSpeed(double x, double y) const;

            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_speed_pub_;

            std::vector<SpeedZone> zones_;
            double default_max_speed_;
    };
}

#endif