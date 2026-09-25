// Publishes a nav_msgs/Path blackboard entry to a ROS topic for RViz.
// Needed because the route-graph portion of navigation never goes through
// planner_server::ComputePathToPose (only the freespace "last mile" segment
// does), so the standard /plan topic RViz's path display subscribes to only
// ever shows that short last-mile piece, not the full route plan -- the
// robot appeared to be navigating with no visible planned path while it was
// still on the graph. This node republishes the full combined route path
// (or any {path} blackboard variable) on its own topic instead.
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forklift_navigation
{

class PublishPath : public BT::SyncActionNode
{
public:
  PublishPath(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to publish"),
      BT::InputPort<std::string>("topic", "plan", "Topic to publish the path on"),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    std::string topic;

    if (!getInput("path", path)) {
      throw BT::RuntimeError("PublishPath: missing required input [path]");
    }
    if (!getInput("topic", topic)) {
      throw BT::RuntimeError("PublishPath: missing required input [topic]");
    }

    if (topic != current_topic_ || !publisher_) {
      // Transient local so RViz keeps showing the last published path even
      // if this node's tick is gated behind a rate/reactive controller and
      // doesn't fire on every BT tick.
      publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
        topic, rclcpp::QoS(1).transient_local());
      current_topic_ = topic;
    }

    publisher_->publish(path);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  std::string current_topic_;
};

}  // namespace forklift_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<forklift_navigation::PublishPath>("PublishPath");
}
