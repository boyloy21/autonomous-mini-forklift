#include "forklift_traffic_manager/traffic_reservation.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace forklift_traffic_manager
{

void TrafficReservation::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/*costmap_subscriber*/,
  const std::string & name)
{
  node_ = node;
  name_ = name;

  std::string ns = node->get_namespace();
  if (!ns.empty() && ns.front() == '/') {ns.erase(0, 1);}
  robot_name_ = node->declare_parameter(name_ + ".robot_name", ns);

  std::vector<int64_t> contested_edges_raw =
    node->declare_parameter(name_ + ".contested_edges", std::vector<int64_t>{});
  for (const auto & e : contested_edges_raw) {contested_edges_.insert(static_cast<uint16_t>(e));}

  std::string service_name = node->declare_parameter(
    name_ + ".traffic_manager_service", std::string("/traffic_manager/reserve_edge"));

  // Dedicated callback group + executor so this plugin's blocking service
  // call doesn't reenter/deadlock route_server's own executor -- same
  // pattern nav2_behavior_tree::BtActionNode uses for its action clients.
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  client_ = node->create_client<forklift_interfaces::srv::ReserveEdge>(
    service_name, rmw_qos_profile_services_default, callback_group_);

  RCLCPP_INFO(
    logger_, "TrafficReservation('%s') configured for robot '%s', %zu contested edge(s), "
    "traffic_manager at '%s'",
    name_.c_str(), robot_name_.c_str(), contested_edges_.size(), service_name.c_str());
}

nav2_route::OperationResult TrafficReservation::perform(
  nav2_route::NodePtr/*node_achieved*/,
  nav2_route::EdgePtr edge_entered,
  nav2_route::EdgePtr edge_exited,
  const nav2_route::Route &/*route*/,
  const geometry_msgs::msg::PoseStamped &/*curr_pose*/,
  const nav2_route::Metadata */*mdata*/)
{
  nav2_route::OperationResult result;

  if (edge_exited && contested_edges_.count(static_cast<uint16_t>(edge_exited->edgeid))) {
    requestReservation(static_cast<uint16_t>(edge_exited->edgeid), false);
  }

  if (edge_entered && contested_edges_.count(static_cast<uint16_t>(edge_entered->edgeid))) {
    const uint16_t edge_id = static_cast<uint16_t>(edge_entered->edgeid);
    if (!requestReservation(edge_id, true)) {
      RCLCPP_INFO(
        logger_, "'%s' denied edge %u, requesting reroute around it",
        robot_name_.c_str(), edge_id);
      result.reroute = true;
      result.blocked_ids.push_back(edge_id);
    }
  }

  return result;
}

bool TrafficReservation::requestReservation(uint16_t edge_id, bool acquire)
{
  if (!client_->wait_for_service(200ms)) {
    // Fail-open: if the fleet coordinator is unreachable, don't
    // permanently strand the robot -- fall back to per-robot reactive
    // collision avoidance (local costmap/collision_monitor) as the safety
    // net instead. Loud warning so this is never silent.
    RCLCPP_WARN(
      logger_, "traffic_manager unreachable for edge %u -- proceeding without reservation "
      "(relying on local collision avoidance only)", edge_id);
    return true;
  }

  auto request = std::make_shared<forklift_interfaces::srv::ReserveEdge::Request>();
  request->robot_name = robot_name_;
  request->edge_id = edge_id;
  request->acquire = acquire;

  auto future = client_->async_send_request(request);
  auto status = callback_group_executor_.spin_until_future_complete(future, 500ms);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(
      logger_, "traffic_manager did not respond in time for edge %u -- proceeding without "
      "reservation (relying on local collision avoidance only)", edge_id);
    return true;
  }

  return future.get()->granted;
}

}  // namespace forklift_traffic_manager

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(forklift_traffic_manager::TrafficReservation, nav2_route::RouteOperation)
