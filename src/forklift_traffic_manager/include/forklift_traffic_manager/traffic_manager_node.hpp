#ifndef FORKLIFT_TRAFFIC_MANAGER__TRAFFIC_MANAGER_NODE_HPP_
#define FORKLIFT_TRAFFIC_MANAGER__TRAFFIC_MANAGER_NODE_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "forklift_interfaces/srv/reserve_edge.hpp"
#include "nav2_msgs/srv/dynamic_edges.hpp"

namespace forklift_traffic_manager
{

// Fleet-wide arbiter for single-lane route_server graph edges. Each robot's
// route_server runs its own DynamicEdgesScorer + TrafficReservation
// RouteOperation (see traffic_reservation.hpp); this node is the single
// source of truth for who currently owns a contested edge, and broadcasts
// closed/opened edges to every OTHER robot's DynamicEdgesScorer so nobody
// else's planner routes through an edge that's actually occupied.
class TrafficManagerNode : public rclcpp::Node
{
public:
  explicit TrafficManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handleReserve(
    const std::shared_ptr<forklift_interfaces::srv::ReserveEdge::Request> request,
    std::shared_ptr<forklift_interfaces::srv::ReserveEdge::Response> response);

  void broadcastEdgeState(
    uint16_t edge_id, const std::string & owning_robot, bool closed);

  std::vector<std::string> robots_;
  std::string dynamic_edges_service_name_;

  std::mutex mutex_;
  std::map<uint16_t, std::string> owner_;  // edge_id -> owning robot, absent = free

  std::map<std::string, rclcpp::Client<nav2_msgs::srv::DynamicEdges>::SharedPtr> edge_clients_;
  rclcpp::Service<forklift_interfaces::srv::ReserveEdge>::SharedPtr reserve_service_;
};

}  // namespace forklift_traffic_manager

#endif  // FORKLIFT_TRAFFIC_MANAGER__TRAFFIC_MANAGER_NODE_HPP_
