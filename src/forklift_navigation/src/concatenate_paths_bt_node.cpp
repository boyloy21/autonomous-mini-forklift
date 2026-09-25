// Appends input_path2 onto input_path1, producing a single continuous path.
// Needed for route_server-based navigation to reach goals off the GeoJSON
// route graph ("outside route"): ComputeRoute snaps the requested goal to
// the nearest node/edge on the graph rather than routing literally to the
// requested pose, so the dense route path's last point can land short of
// (or beside) the real target. The fix is to plan a real freespace "last
// mile" segment (ComputePathToPose, from the route's end pose to the
// literal goal) and concatenate it onto the route path with this node --
// unlike overwriting the final pose in place, this keeps the path
// collision-checked and continuous, and ComputePathToPose's own orientation
// filter already stamps the literal goal pose onto its last point, so no
// separate pose-correction step is needed afterward.
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"

namespace forklift_navigation
{

class ConcatenatePaths : public BT::SyncActionNode
{
public:
  ConcatenatePaths(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path1", "Leading path segment"),
      BT::InputPort<nav_msgs::msg::Path>("input_path2", "Trailing path segment"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "input_path1 followed by input_path2"),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path1;
    nav_msgs::msg::Path path2;

    if (!getInput("input_path1", path1)) {
      throw BT::RuntimeError("ConcatenatePaths: missing required input [input_path1]");
    }
    if (!getInput("input_path2", path2)) {
      throw BT::RuntimeError("ConcatenatePaths: missing required input [input_path2]");
    }

    if (path1.poses.empty()) {
      setOutput("output_path", path2);
      return BT::NodeStatus::SUCCESS;
    }
    if (path2.poses.empty()) {
      setOutput("output_path", path1);
      return BT::NodeStatus::SUCCESS;
    }

    // path2 starts at (approximately) path1's last pose -- drop that
    // duplicate boundary point so the seam doesn't leave a zero-length
    // segment in the combined path.
    nav_msgs::msg::Path combined = path1;
    combined.poses.insert(combined.poses.end(), path2.poses.begin() + 1, path2.poses.end());

    setOutput("output_path", combined);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace forklift_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<forklift_navigation::ConcatenatePaths>("ConcatenatePaths");
}
