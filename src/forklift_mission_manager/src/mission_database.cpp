#include "forklift_mission_manager/mission_database.hpp"

#include <yaml-cpp/yaml.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace forklift_mission_manager
{

namespace
{
geometry_msgs::msg::PoseStamped poseFromYaml(const YAML::Node & node, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped p;
  if (!node || node.size() < 3) {return p;}  // empty frame_id signals "not set"
  p.header.frame_id = frame;
  p.pose.position.x = node[0].as<double>();
  p.pose.position.y = node[1].as<double>();
  tf2::Quaternion q;
  q.setRPY(0, 0, node[2].as<double>());
  p.pose.orientation = tf2::toMsg(q);
  return p;
}
}  // namespace

MissionDatabase::MissionDatabase(const rclcpp::Logger & logger)
: logger_(logger)
{
}

bool MissionDatabase::loadFromFile(const std::string & filepath)
{
  templates_.clear();

  if (filepath.empty()) {
    RCLCPP_INFO(logger_, "No missions.yaml path configured -- named mission templates disabled.");
    return true;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(filepath);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Could not load missions.yaml at '%s': %s -- named templates disabled.",
      filepath.c_str(), e.what());
    return true;  // non-fatal, matches the "optional" contract
  }

  if (!root["missions"]) {
    RCLCPP_WARN(logger_, "missions.yaml has no top-level 'missions' key.");
    return true;
  }

  YAML::Node missions_node = root["missions"];
  for (YAML::const_iterator it = missions_node.begin(); it != missions_node.end(); ++it) {
    const std::string mission_id = it->first.as<std::string>();
    const YAML::Node m = it->second;

    if (!m["type"]) {continue;}
    MissionTemplate tmpl;
    tmpl.mission_id = mission_id;
    tmpl.type = missionTypeFromString(m["type"].as<std::string>());
    if (tmpl.type == MissionType::UNKNOWN) {
      RCLCPP_WARN(logger_, "Mission '%s' has unknown type, skipping.", mission_id.c_str());
      continue;
    }

    if (m["pick_dock_id"]) {tmpl.pick_dock_id = m["pick_dock_id"].as<std::string>();}
    if (m["drop_dock_id"]) {tmpl.drop_dock_id = m["drop_dock_id"].as<std::string>();}
    if (m["dock_id"]) {tmpl.dock_id = m["dock_id"].as<std::string>();}
    if (m["station_wait_duration"]) {
      tmpl.station_wait_duration = m["station_wait_duration"].as<float>();
    }
    if (m["pick_area_pose"]) {tmpl.pick_area_pose = poseFromYaml(m["pick_area_pose"], "map");}
    if (m["drop_area_pose"]) {tmpl.drop_area_pose = poseFromYaml(m["drop_area_pose"], "map");}
    if (m["waypoints"]) {
      for (const auto & wp : m["waypoints"]) {
        tmpl.waypoints.push_back(poseFromYaml(wp, "map"));
      }
    }

    templates_[mission_id] = tmpl;
  }

  RCLCPP_INFO(logger_, "Loaded %zu named mission template(s) from '%s'", templates_.size(), filepath.c_str());
  return true;
}

std::optional<MissionTemplate> MissionDatabase::getTemplate(const std::string & mission_id) const
{
  auto it = templates_.find(mission_id);
  if (it == templates_.end()) {return std::nullopt;}
  return it->second;
}

bool MissionDatabase::validateMission(const Mission & mission, std::string & error_message) const
{
  if (mission.type == MissionType::UNKNOWN) {
    error_message = "Unknown mission_type";
    return false;
  }
  // Deeper structural checks are handled by expandMissionToTasks() itself
  // (called right after this in both handleGoal and handleAccepted) --
  // this function is intentionally light, since duplicating that
  // per-type field validation here would just be the same checks twice.
  return true;
}

}  // namespace forklift_mission_manager