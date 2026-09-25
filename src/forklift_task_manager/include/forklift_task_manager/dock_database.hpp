#ifndef FORKLIFT_TASK_MANAGER__DOCK_DATABASE_HPP_
#define FORKLIFT_TASK_MANAGER__DOCK_DATABASE_HPP_

#include <string>
#include <unordered_map>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace forklift_task_manager
{

// NOTE: this duplicates forklift_docking's DockDatabase class. Not
// currently shared as a library across packages -- worth factoring into a
// small forklift_dock_common package later if it needs to change in more
// than one place at once.
enum class DockType { CHARGING_DOCK, STATION_DOCK, PALLET_PICK_DOCK, PALLET_DROP_DOCK, UNKNOWN };

DockType dockTypeFromString(const std::string & type_str);
std::string dockTypeToString(DockType type);

struct DockInstance
{
  std::string dock_id;
  DockType type{DockType::UNKNOWN};
  std::string frame{"map"};
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  int marker_id{-1};
  double lift_height{0.0};
};

class DockDatabase
{
public:
  explicit DockDatabase(const rclcpp::Logger & logger);
  bool loadFromFile(const std::string & filepath);
  std::optional<DockInstance> getDock(const std::string & dock_id) const;
  std::vector<DockInstance> listDocks() const;
  size_t size() const {return docks_.size();}

private:
  std::unordered_map<std::string, DockInstance> docks_;
  rclcpp::Logger logger_;
};

}  // namespace forklift_task_manager

#endif  // FORKLIFT_TASK_MANAGER__DOCK_DATABASE_HPP_