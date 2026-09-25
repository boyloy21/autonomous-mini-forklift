#ifndef FORKLIFT_DOCKING__REAR_CHARGING_MANEUVER_HPP_
#define FORKLIFT_DOCKING__REAR_CHARGING_MANEUVER_HPP_

#include <functional>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

namespace forklift_docking
{

// Rear-contact charging for a robot whose camera is front-only.
//
// opennav_docking (Humble backport) can only dock forward here -- the
// front camera has to see the charger's ArUco marker -- so DockRobot ends
// with the robot FACING the charger. The charging contact is on the
// robot's rear, so this class finishes the job:
//
//   approach(): 1. reverse `pre_rotate_backoff` so the turn has clearance
//               2. rotate `rotate_angle` (pi) in place -> rear faces charger
//               3. reverse until charger contact (or wheel stall), at most
//                  `max_backup_distance`
//   undo():     exact mirror of what approach() actually did, returning
//               the robot to the pose DockRobot finished at, so the stock
//               UndockRobot action stays consistent with reality.
//   release():  after charging, drive FORWARD `release_distance` off the
//               pad. The robot faces away from the charger, so this is the
//               undock -- never UndockRobot, which reverses into it.
//
// Every leg is closed-loop on odometry (not timed open-loop), so the
// result doesn't depend on acceleration limits or wheel slip.
//
// Multi-robot: cmd_vel/odom/charger_contact are all relative, e.g.
// /forklift1/charger_contact -- every charger pad's contact sensor
// (Gazebo /charger_contacts), bridged into each robot's namespace. A
// contact only counts when one side is a charger's
// `dock_contact_collision` pad and the other side belongs to the Gazebo
// model named `robot_name`, so bumping a wall or another robot never
// looks like docking.
class RearChargingManeuver
{
public:
  using IsCancelingFn = std::function<bool ()>;

  struct Params
  {
    double pre_rotate_backoff{0.35};    // m, forks sweep ~0.36 m around the wheel axle
    double rotate_angle{M_PI};          // rad, positive = counter-clockwise
    double max_backup_distance{0.80};   // m
    double release_distance{0.60};      // m, forward off the pad after charging
    double linear_speed{0.05};          // m/s
    double angular_speed{0.8};          // rad/s, cruise speed of the turn
    double min_angular_speed{0.3};      // rad/s, floor while slowing near the target
    double yaw_tolerance{0.02};         // rad
    double rotate_timeout{0.0};         // s, 0 = auto from angle and speeds
    double rotate_stall_timeout{3.0};   // s turning less than rotate_stall_angle = blocked
    double rotate_stall_angle{0.03};    // rad
    double stall_timeout{1.0};          // s without progress while reversing = contact
    double stall_distance{0.005};       // m, "no progress" threshold
    double drive_stall_timeout{3.0};    // s without progress on a non-contact leg = stuck
    std::string cmd_vel_topic{"cmd_vel"};
    std::string odom_topic{"odom"};
    std::string contact_topic{"charger_contact"};  // empty = stall detection only
    std::string robot_name;             // Gazebo model name, e.g. "forklift1"
  };

  RearChargingManeuver(rclcpp::Node * node, const Params & params);

  // Declares every parameter under `prefix` (e.g. "charging_") on `node`
  // and returns the resulting Params. robot_name defaults to the node's
  // namespace without the leading "/" ("forklift" when unnamespaced,
  // matching the single-robot spawn name).
  static Params declareParameters(rclcpp::Node * node, const std::string & prefix);

  bool approach(const IsCancelingFn & is_canceling);
  bool undo(const IsCancelingFn & is_canceling);
  bool release(const IsCancelingFn & is_canceling);

  bool inContact() const;

private:
  struct OdomSample
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  bool waitForOdom(double timeout_sec);
  OdomSample latestOdom() const;

  // Drives straight at `speed` (negative = reverse) until `distance` is
  // covered. With stop_on_contact, also stops (successfully) on charger
  // contact or on a wheel stall. `traveled` receives the distance actually
  // covered either way.
  bool drive(
    double speed, double distance, bool stop_on_contact, const IsCancelingFn & is_canceling,
    double & traveled, bool & contact);
  bool rotate(double angle, const IsCancelingFn & is_canceling, double & rotated);
  void stop();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);
  bool isOwnChargerContact(const std::string & a, const std::string & b) const;

  rclcpp::Node * node_;
  Params params_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;

  mutable std::mutex mutex_;
  OdomSample odom_;
  bool have_odom_{false};
  rclcpp::Time last_contact_time_;
  bool have_contact_{false};

  // What approach() actually did, so undo() can mirror it exactly.
  double done_backoff_{0.0};
  double done_rotation_{0.0};
  double done_backup_{0.0};
};

}  // namespace forklift_docking

#endif  // FORKLIFT_DOCKING__REAR_CHARGING_MANEUVER_HPP_
