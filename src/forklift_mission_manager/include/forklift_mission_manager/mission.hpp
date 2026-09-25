#ifndef FORKLIFT_MISSION_MANAGER__MISSION_HPP_
#define FORKLIFT_MISSION_MANAGER__MISSION_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "forklift_mission_manager/mission_state_machine.hpp"

namespace forklift_mission_manager
{
    enum class MissionType { PALLET_TRANSPORT, PATROL, CHARGING, STATION_OPERATION, UNKNOWN };

    MissionType missionTypeFromString(const std::string & s);
    std::string missionTypeToString(MissionType t);

    struct TaskSpec
    {
        std::string task_type;
        geometry_msgs::msg::PoseStamped goal_pose;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::string dock_id;
        float target_height{0.0f};
        float station_wait_duration{0.0f};
    };

    struct Mission
    {
        std::string mission_id;
        MissionType type{MissionType::UNKNOWN};

        // RAW goal parameters, as received
        geometry_msgs::msg::PoseStamped pick_area_pose;
        std::string pick_dock_id;
        geometry_msgs::msg::PoseStamped drop_area_pose;
        std::string drop_dock_id;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::string dock_id;
        float station_wait_duration{0.0f};

        std::vector<TaskSpec> tasks;
        size_t current_task_index{0};
        MissionState state{MissionState::IDLE};
        rclcpp::Time created_time;
    };

    bool expandMissionToTasks(Mission & mission, std::string & error_message);
}  // namespace forklift_mission_manager
 
#endif  // FORKLIFT_MISSION_MANAGER__MISSION_HPP_