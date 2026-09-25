#ifndef FORKLIFT_DOCKING__DOCK_DATABASE_HPP_
#define FORKLIFT_DOCKING__DOCK_DATABASE_HPP_

#include <string>
#include <unordered_map>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace forklift_docking
{

enum class DockType
{
  CHARGING_DOCK,
  STATION_DOCK,
  PALLET_PICK_DOCK,
  PALLET_DROP_DOCK,
  UNKNOWN
};

DockType dockTypeFromString(const std::string & type_str);
std::string dockTypeToString(DockType type);

// A single named dock instance in the world: its type, the frame its pose
// is expressed in, its coarse [x, y, theta] pose, and optionally the ArUco
// marker ID that identifies it up close (used by DockDetect).
struct DockInstance
{
  std::string dock_id;
  DockType type{DockType::UNKNOWN};
  std::string frame{"map"};
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  int marker_id{-1};  // -1 = unset / not marker-based
  double lift_height{0.0};  // meters, target fork height for this dock (0.0 = no lift needed)
};

// Loads dock instances from a YAML file (see config/dock_database.yaml) and
// provides lookup by dock_id. Read-only after loadFromFile() -- this class
// does not persist changes back to disk.
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

}  // namespace forklift_docking

#endif  // FORKLIFT_DOCKING__DOCK_DATABASE_HPP_