#include "forklift_traffic_manager/traffic_manager_node.hpp"

namespace forklift_traffic_manager
{

TrafficManagerNode::TrafficManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("traffic_manager", options)
{
  robots_ = this->declare_parameter("robots", std::vector<std::string>{});
  dynamic_edges_service_name_ = this->declare_parameter(
    "dynamic_edges_service_name", std::string("route_server/DynamicEdgesScorer/adjust_edges"));

  if (robots_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No 'robots' parameter provided -- traffic_manager has nothing to arbitrate between.");
  }

  for (const auto & robot : robots_) {
    const std::string service_name = "/" + robot + "/" + dynamic_edges_service_name_;
    edge_clients_[robot] = this->create_client<nav2_msgs::srv::DynamicEdges>(service_name);
    RCLCPP_INFO(this->get_logger(), "Registered edge client for '%s'", service_name.c_str());
  }

  reserve_service_ = this->create_service<forklift_interfaces::srv::ReserveEdge>(
    "~/reserve_edge",
    std::bind(
      &TrafficManagerNode::handleReserve, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(), "traffic_manager ready, arbitrating %zu edge(s)-worth of robots",
    robots_.size());
}

void TrafficManagerNode::handleReserve(
  const std::shared_ptr<forklift_interfaces::srv::ReserveEdge::Request> request,
  std::shared_ptr<forklift_interfaces::srv::ReserveEdge::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint16_t edge_id = request->edge_id;
  const std::string & robot = request->robot_name;

  if (request->acquire) {
    auto it = owner_.find(edge_id);
    if (it == owner_.end() || it->second == robot) {
      owner_[edge_id] = robot;
      response->granted = true;
      RCLCPP_INFO(this->get_logger(), "Edge %u granted to '%s'", edge_id, robot.c_str());
      broadcastEdgeState(edge_id, robot, true);
    } else {
      response->granted = false;
      RCLCPP_INFO(
        this->get_logger(), "Edge %u requested by '%s' but held by '%s' -- denied",
        edge_id, robot.c_str(), it->second.c_str());
    }
  } else {
    auto it = owner_.find(edge_id);
    if (it != owner_.end() && it->second == robot) {
      owner_.erase(it);
      RCLCPP_INFO(this->get_logger(), "Edge %u released by '%s'", edge_id, robot.c_str());
      broadcastEdgeState(edge_id, robot, false);
    }
    response->granted = true;
  }
}

void TrafficManagerNode::broadcastEdgeState(
  uint16_t edge_id, const std::string & owning_robot, bool closed)
{
  for (const auto & [robot, client] : edge_clients_) {
    // The owning robot needs the edge OPEN in its own route_server to
    // actually traverse it -- only tell every OTHER robot.
    if (robot == owning_robot) {continue;}

    if (!client->service_is_ready()) {
      RCLCPP_WARN(
        this->get_logger(), "DynamicEdges service for '%s' not ready, skipping broadcast",
        robot.c_str());
      continue;
    }

    auto request = std::make_shared<nav2_msgs::srv::DynamicEdges::Request>();
    if (closed) {
      request->closed_edges = {edge_id};
    } else {
      request->opened_edges = {edge_id};
    }
    // Fire-and-forget: the requester only needs to know they hold the
    // reservation, not that every other robot has already been told.
    client->async_send_request(request);
  }
}

}  // namespace forklift_traffic_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forklift_traffic_manager::TrafficManagerNode>());
  rclcpp::shutdown();
  return 0;
}
