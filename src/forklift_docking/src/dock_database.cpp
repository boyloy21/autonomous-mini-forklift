#include "forklift_docking/dock_database.hpp"

#include <yaml-cpp/yaml.h>

namespace forklift_docking
{

DockType dockTypeFromString(const std::string & type_str)
{
  if (type_str == "charging_dock") {return DockType::CHARGING_DOCK;}
  if (type_str == "station_dock") {return DockType::STATION_DOCK;}
  if (type_str == "pallet_pick_dock") {return DockType::PALLET_PICK_DOCK;}
  if (type_str == "pallet_drop_dock") {return DockType::PALLET_DROP_DOCK;}
  return DockType::UNKNOWN;
}

std::string dockTypeToString(DockType type)
{
  switch (type) {
    case DockType::CHARGING_DOCK: return "charging_dock";
    case DockType::STATION_DOCK: return "station_dock";
    case DockType::PALLET_PICK_DOCK: return "pallet_pick_dock";
    case DockType::PALLET_DROP_DOCK: return "pallet_drop_dock";
    default: return "unknown";
  }
}

DockDatabase::DockDatabase(const rclcpp::Logger & logger)
: logger_(logger)
{
}

bool DockDatabase::loadFromFile(const std::string & filepath)
{
  docks_.clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(filepath);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to load dock database file '%s': %s", filepath.c_str(), e.what());
    return false;
  }

  if (!root["docks"]) {
    RCLCPP_ERROR(logger_, "Dock database file '%s' has no top-level 'docks' key", filepath.c_str());
    return false;
  }

  YAML::Node docks_node = root["docks"];
  for (YAML::const_iterator it = docks_node.begin(); it != docks_node.end(); ++it) {
    const std::string dock_id = it->first.as<std::string>();
    const YAML::Node dock_node = it->second;

    DockInstance dock;
    dock.dock_id = dock_id;

    if (!dock_node["type"]) {
      RCLCPP_ERROR(logger_, "Dock '%s' missing required 'type' field, skipping", dock_id.c_str());
      continue;
    }
    dock.type = dockTypeFromString(dock_node["type"].as<std::string>());
    if (dock.type == DockType::UNKNOWN) {
      RCLCPP_ERROR(
        logger_, "Dock '%s' has unrecognized type '%s', skipping",
        dock_id.c_str(), dock_node["type"].as<std::string>().c_str());
      continue;
    }

    dock.frame = dock_node["frame"] ? dock_node["frame"].as<std::string>() : "map";

    if (!dock_node["pose"] || dock_node["pose"].size() != 3) {
      RCLCPP_ERROR(
        logger_, "Dock '%s' missing or malformed 'pose' field (expected [x, y, theta]), skipping",
        dock_id.c_str());
      continue;
    }
    dock.x = dock_node["pose"][0].as<double>();
    dock.y = dock_node["pose"][1].as<double>();
    dock.theta = dock_node["pose"][2].as<double>();

    if (dock_node["id"]) {
      try {
        dock.marker_id = dock_node["id"].as<int>();
      } catch (const std::exception &) {
        RCLCPP_WARN(logger_, "Dock '%s' has non-integer 'id' field, ignoring", dock_id.c_str());
      }
    }

    if (dock_node["lift"]) {
      try {
        dock.lift_height = dock_node["lift"].as<double>();
      } catch (const std::exception &) {
        RCLCPP_WARN(
          logger_, "Dock '%s' has non-numeric 'lift' field, defaulting to 0.0", dock_id.c_str());
      }
    }

    docks_[dock_id] = dock;
  }

  RCLCPP_INFO(logger_, "Loaded %zu dock(s) from '%s'", docks_.size(), filepath.c_str());
  return !docks_.empty();
}

std::optional<DockInstance> DockDatabase::getDock(const std::string & dock_id) const
{
  auto it = docks_.find(dock_id);
  if (it == docks_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<DockInstance> DockDatabase::listDocks() const
{
  std::vector<DockInstance> result;
  result.reserve(docks_.size());
  for (const auto & [id, dock] : docks_) {
    (void)id;
    result.push_back(dock);
  }
  return result;
}

}  // namespace forklift_docking