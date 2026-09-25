#ifndef FORKLIFT_MISSION_MANAGER__MISSION_DATABASE_HPP_
#define FORKLIFT_MISSION_MANAGER__MISSION_DATABASE_HPP_

#include <string>
#include <unordered_map>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "forklift_mission_manager/mission.hpp"

namespace forklift_mission_manager
{

// A named, reusable mission template from missions.yaml -- lets a caller
// (forklift_web_bridge) trigger a well-known mission by a short id
// instead of resending full parameters every time. Optional: a caller
// can still send a fully-specified ad-hoc ExecuteMission goal directly
// without any entry existing here.
struct MissionTemplate
{
  std::string mission_id;
  MissionType type{MissionType::UNKNOWN};
  geometry_msgs::msg::PoseStamped pick_area_pose;
  std::string pick_dock_id;
  geometry_msgs::msg::PoseStamped drop_area_pose;
  std::string drop_dock_id;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::string dock_id;
  float station_wait_duration{0.0f};
};

class MissionDatabase
{
public:
  explicit MissionDatabase(const rclcpp::Logger & logger);

  // missions.yaml is optional -- an empty/missing file is not an error,
  // it just means no named templates are available (every mission must
  // then be sent fully-specified).
  bool loadFromFile(const std::string & filepath);

  std::optional<MissionTemplate> getTemplate(const std::string & mission_id) const;

  // Structural validation only: known mission_type, required top-level
  // fields present for that type. Does NOT check dock_id existence --
  // see the note on expandMissionToTasks() in mission.hpp for why.
  bool validateMission(const Mission & mission, std::string & error_message) const;

private:
  std::unordered_map<std::string, MissionTemplate> templates_;
  rclcpp::Logger logger_;
};

}  // namespace forklift_mission_manager

#endif  // FORKLIFT_MISSION_MANAGER__MISSION_DATABASE_HPP_