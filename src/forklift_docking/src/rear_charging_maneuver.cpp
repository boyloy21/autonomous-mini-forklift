#include "forklift_docking/rear_charging_maneuver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace forklift_docking
{

namespace
{
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool startsWith(const std::string & s, const std::string & prefix)
{
  return s.compare(0, prefix.size(), prefix) == 0;
}

// Name of the collision on every charger_station model that marks the
// charging contact pad (see forklift_gazebo/models/charger_station).
constexpr char kChargerPadCollision[] = "dock_contact_collision";
// A contact older than this no longer counts as "touching".
constexpr double kContactFreshness = 0.3;
}  // namespace

RearChargingManeuver::RearChargingManeuver(rclcpp::Node * node, const Params & params)
: node_(node), params_(params), last_contact_time_(0, 0, node->get_clock()->get_clock_type())
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(params_.cmd_vel_topic, 10);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    params_.odom_topic, rclcpp::QoS(10),
    std::bind(&RearChargingManeuver::odomCallback, this, std::placeholders::_1));
  if (!params_.contact_topic.empty()) {
    contact_sub_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      params_.contact_topic, rclcpp::QoS(10),
      std::bind(&RearChargingManeuver::contactCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Rear charging maneuver: robot_name='%s', contact_topic='%s', odom='%s', cmd_vel='%s'",
    params_.robot_name.c_str(), params_.contact_topic.c_str(), params_.odom_topic.c_str(),
    params_.cmd_vel_topic.c_str());
}

RearChargingManeuver::Params RearChargingManeuver::declareParameters(
  rclcpp::Node * node, const std::string & prefix)
{
  Params p;
  p.pre_rotate_backoff = node->declare_parameter(prefix + "pre_rotate_backoff", p.pre_rotate_backoff);
  p.rotate_angle = node->declare_parameter(prefix + "rotate_angle", p.rotate_angle);
  p.max_backup_distance = node->declare_parameter(prefix + "backup_distance", p.max_backup_distance);
  p.release_distance = node->declare_parameter(prefix + "release_distance", p.release_distance);
  p.linear_speed = node->declare_parameter(prefix + "linear_speed", p.linear_speed);
  p.angular_speed = node->declare_parameter(prefix + "angular_speed", p.angular_speed);
  p.min_angular_speed = node->declare_parameter(prefix + "min_angular_speed", p.min_angular_speed);
  p.yaw_tolerance = node->declare_parameter(prefix + "yaw_tolerance", p.yaw_tolerance);
  p.rotate_timeout = node->declare_parameter(prefix + "rotate_timeout", p.rotate_timeout);
  p.rotate_stall_timeout =
    node->declare_parameter(prefix + "rotate_stall_timeout", p.rotate_stall_timeout);
  p.rotate_stall_angle = node->declare_parameter(prefix + "rotate_stall_angle", p.rotate_stall_angle);
  p.stall_timeout = node->declare_parameter(prefix + "stall_timeout", p.stall_timeout);
  p.stall_distance = node->declare_parameter(prefix + "stall_distance", p.stall_distance);
  p.drive_stall_timeout =
    node->declare_parameter(prefix + "drive_stall_timeout", p.drive_stall_timeout);
  p.cmd_vel_topic = node->declare_parameter(prefix + "cmd_vel_topic", p.cmd_vel_topic);
  p.odom_topic = node->declare_parameter(prefix + "odom_topic", p.odom_topic);
  p.contact_topic = node->declare_parameter(prefix + "contact_topic", p.contact_topic);

  std::string default_name = node->get_namespace();
  default_name.erase(0, default_name.find_first_not_of('/'));
  if (default_name.empty()) {
    default_name = "forklift";
  }
  p.robot_name = node->declare_parameter(prefix + "robot_name", default_name);
  return p;
}

// ==================== Public maneuvers ====================

bool RearChargingManeuver::approach(const IsCancelingFn & is_canceling)
{
  done_backoff_ = done_rotation_ = done_backup_ = 0.0;

  if (!waitForOdom(2.0)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Rear charging: no odometry on '%s'", params_.odom_topic.c_str());
    return false;
  }

  bool contact = false;
  if (params_.pre_rotate_backoff > 0.0) {
    RCLCPP_INFO(
      node_->get_logger(), "Rear charging: backing off %.2fm for turn clearance",
      params_.pre_rotate_backoff);
    if (!drive(
        -params_.linear_speed, params_.pre_rotate_backoff, false, is_canceling, done_backoff_,
        contact))
    {
      return false;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(), "Rear charging: rotating %.2f rad to face rear to charger",
    params_.rotate_angle);
  if (!rotate(params_.rotate_angle, is_canceling, done_rotation_)) {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Rear charging: reversing into charger (max %.2fm)",
    params_.max_backup_distance);
  if (!drive(
      -params_.linear_speed, params_.max_backup_distance, true, is_canceling, done_backup_,
      contact))
  {
    return false;
  }
  if (!contact) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Rear charging: reversed %.2fm without touching the charger -- increase "
      "charging_backup_distance or check the dock pose", done_backup_);
    return false;
  }
  if (contact_sub_ && !inContact()) {
    // Stopped by a wheel stall, but the pad never reported this robot:
    // anything that charges on pad contact (fake_battery_node) won't charge.
    RCLCPP_WARN(
      node_->get_logger(),
      "Rear charging: stopped by wheel stall but NO charger pad contact on '%s' -- the robot "
      "is pushing against something else, or the contact sensor/bridge is missing "
      "(check: ign topic -l | grep charger_contact)", contact_sub_->get_topic_name());
  }

  RCLCPP_INFO(
    node_->get_logger(), "Rear charging: in contact after reversing %.2fm", done_backup_);
  return true;
}

bool RearChargingManeuver::undo(const IsCancelingFn & is_canceling)
{
  bool contact = false;
  double traveled = 0.0;
  double rotated = 0.0;

  if (!waitForOdom(2.0)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Rear charging undo: no odometry on '%s'", params_.odom_topic.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Rear charging undo: driving %.2fm away from charger", done_backup_);
  if (done_backup_ > 0.0 &&
    !drive(params_.linear_speed, done_backup_, false, is_canceling, traveled, contact))
  {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Rear charging undo: rotating back %.2f rad", -done_rotation_);
  if (std::fabs(done_rotation_) > 0.0 && !rotate(-done_rotation_, is_canceling, rotated)) {
    return false;
  }

  if (done_backoff_ > 0.0 &&
    !drive(params_.linear_speed, done_backoff_, false, is_canceling, traveled, contact))
  {
    return false;
  }

  done_backoff_ = done_rotation_ = done_backup_ = 0.0;
  return true;
}

bool RearChargingManeuver::release(const IsCancelingFn & is_canceling)
{
  if (!waitForOdom(2.0)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Rear charging release: no odometry on '%s'",
      params_.odom_topic.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Rear charging release: driving %.2fm forward off the charger",
    params_.release_distance);
  bool contact = false;
  double traveled = 0.0;
  if (!drive(params_.linear_speed, params_.release_distance, false, is_canceling, traveled, contact)) {
    return false;
  }

  done_backoff_ = done_rotation_ = done_backup_ = 0.0;
  return true;
}

bool RearChargingManeuver::inContact() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return have_contact_ && (node_->now() - last_contact_time_).seconds() < kContactFreshness;
}

// ==================== Closed-loop legs ====================

bool RearChargingManeuver::drive(
  double speed, double distance, bool stop_on_contact, const IsCancelingFn & is_canceling,
  double & traveled, bool & contact)
{
  traveled = 0.0;
  contact = false;

  const OdomSample start = latestOdom();
  OdomSample stall_ref = start;
  const auto start_time = node_->now();
  auto stall_ref_time = start_time;
  const double timeout = 2.0 * distance / std::fabs(speed) + 5.0;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = speed;

  rclcpp::Rate rate(20.0);
  while (rclcpp::ok()) {
    if (is_canceling()) {
      stop();
      return false;
    }

    const OdomSample now_odom = latestOdom();
    traveled = std::hypot(now_odom.x - start.x, now_odom.y - start.y);
    if (traveled >= distance) {
      break;
    }

    const auto now = node_->now();
    if (stop_on_contact && inContact()) {
      contact = true;
      break;
    }
    if (std::hypot(now_odom.x - stall_ref.x, now_odom.y - stall_ref.y) > params_.stall_distance) {
      stall_ref = now_odom;
      stall_ref_time = now;
    } else if (stop_on_contact && (now - stall_ref_time).seconds() > params_.stall_timeout) {
      // Pushing against something solid: wheels turn but the robot doesn't
      // move. Treated as contact -- covers setups without a contact topic.
      RCLCPP_INFO(node_->get_logger(), "Rear charging: wheel stall detected, treating as contact");
      contact = true;
      break;
    } else if (!stop_on_contact && (now - stall_ref_time).seconds() > params_.drive_stall_timeout) {
      stop();
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rear charging: robot not moving (%.2f/%.2fm after %.1fs at %.2f m/s commanded) -- "
        "stuck on something (forks under/against the charger?) or another node is "
        "publishing cmd_vel", traveled, distance, (now - start_time).seconds(), speed);
      return false;
    }

    if ((now - start_time).seconds() > timeout) {
      stop();
      RCLCPP_ERROR(
        node_->get_logger(), "Rear charging: drive timed out after %.2f/%.2fm", traveled, distance);
      return false;
    }

    cmd_vel_pub_->publish(cmd);
    rate.sleep();
  }

  stop();
  return rclcpp::ok();
}

bool RearChargingManeuver::rotate(
  double angle, const IsCancelingFn & is_canceling, double & rotated)
{
  rotated = 0.0;
  double prev_yaw = latestOdom().yaw;
  const auto start_time = node_->now();
  const double min_speed = std::min(params_.min_angular_speed, params_.angular_speed);
  // Auto timeout: the whole turn at the slowest commanded speed, plus margin.
  const double timeout = params_.rotate_timeout > 0.0 ?
    params_.rotate_timeout : 1.5 * std::fabs(angle) / min_speed + 10.0;
  double stall_ref = 0.0;
  auto stall_ref_time = start_time;

  rclcpp::Rate rate(20.0);
  while (rclcpp::ok()) {
    if (is_canceling()) {
      stop();
      return false;
    }

    // Accumulate unwrapped yaw so a pi turn across the +-pi seam works.
    const double yaw = latestOdom().yaw;
    rotated += normalizeAngle(yaw - prev_yaw);
    prev_yaw = yaw;

    const double remaining = angle - rotated;
    if (std::fabs(remaining) <= params_.yaw_tolerance) {
      break;
    }
    const auto now = node_->now();
    const double elapsed = (now - start_time).seconds();
    if (elapsed > timeout) {
      stop();
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rear charging: rotation timed out at %.2f/%.2f rad after %.1fs (avg %.3f rad/s, "
        "commanded %.2f-%.2f) -- robot is not following cmd_vel: check for another cmd_vel "
        "publisher or the DiffDrive limits", rotated, angle, elapsed,
        std::fabs(rotated) / std::max(elapsed, 1e-3), min_speed, params_.angular_speed);
      return false;
    }

    // Commanded but not turning: something is in the way (forks against
    // the charger, a wall) -- fail fast instead of scraping until timeout.
    if (std::fabs(rotated - stall_ref) > params_.rotate_stall_angle) {
      stall_ref = rotated;
      stall_ref_time = now;
    } else if ((now - stall_ref_time).seconds() > params_.rotate_stall_timeout) {
      stop();
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rear charging: rotation blocked at %.2f/%.2f rad (less than %.2f rad in %.1fs) -- "
        "increase charging_pre_rotate_backoff or clear the area around the robot",
        rotated, angle, params_.rotate_stall_angle, params_.rotate_stall_timeout);
      return false;
    }

    geometry_msgs::msg::Twist cmd;
    const double magnitude =
      std::clamp(1.5 * std::fabs(remaining), min_speed, params_.angular_speed);
    cmd.angular.z = std::copysign(magnitude, remaining);
    cmd_vel_pub_->publish(cmd);
    rate.sleep();
  }

  stop();
  return rclcpp::ok();
}

void RearChargingManeuver::stop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

// ==================== Inputs ====================

bool RearChargingManeuver::waitForOdom(double timeout_sec)
{
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_odom_) {
        return true;
      }
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >
      timeout_sec)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

RearChargingManeuver::OdomSample RearChargingManeuver::latestOdom() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_;
}

void RearChargingManeuver::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_.x = msg->pose.pose.position.x;
  odom_.y = msg->pose.pose.position.y;
  odom_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
  have_odom_ = true;
}

void RearChargingManeuver::contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
{
  for (const auto & c : msg->contacts) {
    if (isOwnChargerContact(c.collision1.name, c.collision2.name) ||
      isOwnChargerContact(c.collision2.name, c.collision1.name))
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_contact_time_ = node_->now();
      have_contact_ = true;
      return;
    }
  }
}

bool RearChargingManeuver::isOwnChargerContact(
  const std::string & pad, const std::string & robot) const
{
  // Gazebo scoped names: "ChargerStation2::base_link::dock_contact_collision"
  // and "forklift1::base_link::base_link_collision".
  return pad.find(kChargerPadCollision) != std::string::npos &&
         startsWith(robot, params_.robot_name + "::");
}

}  // namespace forklift_docking
