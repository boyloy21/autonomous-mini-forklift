#ifndef FORKLIFT_TRAFFIC_MANAGER__TRAFFIC_RESERVATION_HPP_
#define FORKLIFT_TRAFFIC_MANAGER__TRAFFIC_RESERVATION_HPP_

#include <memory>
#include <set>
#include <string>

#include "nav2_route/interfaces/route_operation.hpp"
#include "forklift_interfaces/srv/reserve_edge.hpp"

namespace forklift_traffic_manager
{

// A nav2_route::RouteOperation that arbitrates single-lane graph edges
// across the fleet through the traffic_manager node. Configured with
// ON_STATUS_CHANGE so it fires on every node the tracker achieves, without
// needing per-edge "operations" entries authored into the GeoJSON graph.
//
// On entering a contested edge: asks traffic_manager for the reservation.
// If denied (another robot holds it), reports back to route_server that
// this edge should be treated as blocked for THIS robot's own replanning
// (OperationResult::reroute + blocked_ids) -- route_server will either find
// an alternate path or, if none exists, ComputeRoute simply fails this
// cycle and the existing BT retry/wait recovery (RateController + the
// outer RecoveryNode's Wait/BackUp) makes the robot wait its turn.
//
// On exiting a contested edge: releases the reservation.
class TrafficReservation : public nav2_route::RouteOperation
{
public:
  TrafficReservation() = default;
  ~TrafficReservation() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) override;

  std::string getName() override {return name_;}

  nav2_route::RouteOperationType processType() override
  {
    return nav2_route::RouteOperationType::ON_STATUS_CHANGE;
  }

  nav2_route::OperationResult perform(
    nav2_route::NodePtr node_achieved,
    nav2_route::EdgePtr edge_entered,
    nav2_route::EdgePtr edge_exited,
    const nav2_route::Route & route,
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const nav2_route::Metadata * mdata = nullptr) override;

protected:
  bool requestReservation(uint16_t edge_id, bool acquire);

  std::string name_;
  std::string robot_name_;
  std::set<uint16_t> contested_edges_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<forklift_interfaces::srv::ReserveEdge>::SharedPtr client_;
  rclcpp::Logger logger_{rclcpp::get_logger("TrafficReservation")};
};

}  // namespace forklift_traffic_manager

#endif  // FORKLIFT_TRAFFIC_MANAGER__TRAFFIC_RESERVATION_HPP_
