#include "forklift_mission_manager/mission.hpp"

namespace forklift_mission_manager
{
    MissionType missionTypeFromString(const std::string & s)
    {
        if (s == "PALLET_TRANSPORT") {return MissionType::PALLET_TRANSPORT;}
        if (s == "PATROL") {return MissionType::PATROL;}
        if (s == "CHARGING") {return MissionType::CHARGING;}
        if (s == "STATION_OPERATION") {return MissionType::STATION_OPERATION;}
        return MissionType::UNKNOWN;
    }

    std::string missionTypeToString(MissionType t)
    {
        switch (t) {
            case MissionType::PALLET_TRANSPORT: return "PALLET_TRANSPORT";
            case MissionType::PATROL: return "PATROL";
            case MissionType::CHARGING: return "CHARGING";
            case MissionType::STATION_OPERATION: return "STATION_OPERATION";
            default: return "UNKNOWN";
            }
    }

    namespace
    {
        bool hasPose(const geometry_msgs::msg::PoseStamped & p)
        {
            return !p.header.frame_id.empty();
        }
    } // namespace

    bool expandMissionToTasks(Mission & mission, std::string & error_message)
    {
        mission.tasks.clear();

        switch (mission.type)           
        {
            case MissionType::PALLET_TRANSPORT: {
                if (mission.pick_dock_id.empty() || mission.drop_dock_id.empty()) {
                    error_message = "PALLET_TRANSPORT requires pick_dock_id and drop_dock_id";
                    return false;
                }
                if (hasPose(mission.pick_area_pose)) {
                    TaskSpec nav;
                    nav.task_type = "SINGLE_NAVIGATION";
                    nav.goal_pose = mission.pick_area_pose;
                    mission.tasks.push_back(nav);
                }

                TaskSpec pick;
                pick.task_type = "PICK_PALLET_DOCK";
                pick.dock_id = mission.pick_dock_id;
                mission.tasks.push_back(pick);

                if (hasPose(mission.drop_area_pose)) {
                    TaskSpec nav;
                    nav.task_type = "SINGLE_NAVIGATION";
                    nav.goal_pose = mission.drop_area_pose;
                    mission.tasks.push_back(nav);
                }
            
                TaskSpec drop;
                drop.task_type = "DROP_PALLET_DOCK";
                drop.dock_id = mission.drop_dock_id;
                mission.tasks.push_back(drop);
                return true;
            }

            case MissionType::PATROL: {
                if (mission.waypoints.empty()) {
                    error_message = "PATROL requires at least one waypoint";
                    return false;
                }
                // Single task -- forklift_task_manager's PATROL_POINT task type
                // already handles multi-waypoint sequencing itself; not duplicated
                // here.
                TaskSpec patrol;
                patrol.task_type = "PATROL_POINT";
                patrol.waypoints = mission.waypoints;
                mission.tasks.push_back(patrol);
                return true;
            }

            case MissionType::CHARGING: {
                if (mission.dock_id.empty()) {
                    error_message = "CHARGING requires dock_id";
                    return false;
                }
                TaskSpec charge;
                charge.task_type = "CHARGING_DOCK";
                charge.dock_id = mission.dock_id;
                mission.tasks.push_back(charge);
                return true;
            }

            case MissionType::STATION_OPERATION: {
                if (mission.dock_id.empty()) {
                    error_message = "STATION_OPERATION requires dock_id";
                    return false;
                }
                TaskSpec station;
                station.task_type = "STATION_DOCK";
                station.dock_id = mission.dock_id;
                station.station_wait_duration = mission.station_wait_duration;
                mission.tasks.push_back(station);
                return true;
            }
            
            default:
                error_message = "Unknown mission_type";
                return false;

        }
    }

}  // namespace forklift_mission_manager

