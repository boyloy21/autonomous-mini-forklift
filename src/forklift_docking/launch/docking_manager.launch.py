"""
docking_manager.launch.py

Starts pallet_dock_workflow_node -- the orchestrator that listens on
dock_pallet and drives the full sequence:

    DockRobot -> delay -> SetLiftHeight (per-dock height from
    dock_database.yaml) -> UndockRobot

This launch file does NOT start docking_server, forklift_aruco,
lift_controller_node, or navigation -- those are separate, expected to
already be running.

MULTI-ROBOT FIX: added the `namespace` argument (was missing entirely --
same gap that broke docking_server/aruco_node before their own fixes),
applied to the Node via namespace=. All the other DeclareLaunchArgument
defaults below are also now relative (no leading "/"), matching the same
fix already applied in pallet_dock_workflow_node.cpp's own internal
defaults -- these two need to agree, since this launch file's arguments
override the C++ node's defaults.

Also completed the charging_rotate_angle/charging_backup_distance/
charging_linear_speed/charging_angular_speed launch arguments -- these
existed only as broken, commented-out draft code before (including a
variable-name collision that would have silently orphaned the
lift_height_tolerance argument if ever uncommented as-is).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_docking")

    # Same reasoning as docking.launch.py's dock_database override: a bare
    # relative filename resolves against the process's CWD at runtime, not
    # this launch file's location, so we always pass an absolute path.
    default_dock_database = os.path.join(pkg_share, "config", "dock_database.yaml")

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace -- e.g. \"forklift1\". Empty means unnamespaced "
                     "(single-robot use).",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo /clock instead of wall time",
    )

    dock_database = LaunchConfiguration("dock_database")
    declare_dock_database_arg = DeclareLaunchArgument(
        "dock_database",
        default_value=default_dock_database,
        description="Absolute path to dock_database.yaml, used to look up each "
                     "dock's ArUco marker id and lift height. Deliberately the SAME "
                     "file for every robot -- physical docks are fixed map features, "
                     "not per-robot.",
    )

    dock_pallet_topic = LaunchConfiguration("dock_pallet_topic")
    declare_dock_pallet_topic_arg = DeclareLaunchArgument(
        "dock_pallet_topic",
        default_value="dock_pallet",
        description="Trigger topic (std_msgs/String, data=dock_id)",
    )

    post_dock_delay = LaunchConfiguration("post_dock_delay")
    declare_post_dock_delay_arg = DeclareLaunchArgument(
        "post_dock_delay",
        default_value="2.0",
        description="Seconds to wait after DockRobot succeeds, before calling SetLiftHeight",
    )

    lift_target_height = LaunchConfiguration("lift_target_height")
    declare_lift_target_height_arg = DeclareLaunchArgument(
        "lift_target_height",
        default_value="0.1",
        description="Fallback lift height (m) used only if the dock_id isn't found "
                     "in dock_database.yaml -- normally each dock's own 'lift:' "
                     "value from the database is used instead",
    )

    lift_speed = LaunchConfiguration("lift_speed")
    declare_lift_speed_arg = DeclareLaunchArgument(
        "lift_speed",
        default_value="0.0",
        description="Lift speed (m/s) sent to SetLiftHeight, 0.0 = use lift_controller default",
    )

    dock_action_name = LaunchConfiguration("dock_action_name")
    declare_dock_action_name_arg = DeclareLaunchArgument(
        "dock_action_name",
        default_value="dock_robot",
        description="DockRobot action name",
    )

    undock_action_name = LaunchConfiguration("undock_action_name")
    declare_undock_action_name_arg = DeclareLaunchArgument(
        "undock_action_name",
        default_value="undock_robot",
        description="UndockRobot action name",
    )

    set_height_service_name = LaunchConfiguration("set_height_service_name")
    declare_set_height_service_name_arg = DeclareLaunchArgument(
        "set_height_service_name",
        default_value="set_height",
        description="SetLiftHeight action name (served by lift_controller_node)",
    )

    undock_max_time = LaunchConfiguration("undock_max_time")
    declare_undock_max_time_arg = DeclareLaunchArgument(
        "undock_max_time",
        default_value="15.0",
        description="max_undocking_time sent to UndockRobot (seconds)",
    )

    set_height_timeout = LaunchConfiguration("set_height_timeout")
    declare_set_height_timeout_arg = DeclareLaunchArgument(
        "set_height_timeout",
        default_value="35.0",
        description="Client-side sanity ceiling waiting for a SetLiftHeight result "
                     "(seconds) -- guards against an unresponsive server, not a race "
                     "against the server's own internal timeout",
    )

    aruco_node_name = LaunchConfiguration("aruco_node_name")
    declare_aruco_node_name_arg = DeclareLaunchArgument(
        "aruco_node_name",
        default_value="aruco_node",
        description="Node name whose target_marker_id parameter gets set automatically "
                     "before each DockRobot goal, from dock_database.yaml's id: field. "
                     "Relative -- resolves to e.g. /forklift1/aruco_node once this node "
                     "runs under the same robot namespace.",
    )

    lift_height_tolerance = LaunchConfiguration("lift_height_tolerance")
    declare_lift_height_tolerance_arg = DeclareLaunchArgument(
        "lift_height_tolerance",
        default_value="0.01",
        description="Acceptable deviation (m) from target_height for a SetLiftHeight "
                     "result to still be treated as success",
    )

    # Charging-dock rear-contact maneuver (RearChargingManeuver): closed-loop
    # on odom -- back off, turn pi, reverse until the charger pad contact
    # (or a wheel stall), then mirrored exactly before undocking.
    charging_pre_rotate_backoff = LaunchConfiguration("charging_pre_rotate_backoff")
    declare_charging_pre_rotate_backoff_arg = DeclareLaunchArgument(
        "charging_pre_rotate_backoff",
        default_value="0.35",
        description="Meters to reverse after DockRobot so the in-place turn clears the charger",
    )

    charging_rotate_angle = LaunchConfiguration("charging_rotate_angle")
    declare_charging_rotate_angle_arg = DeclareLaunchArgument(
        "charging_rotate_angle",
        default_value="3.14159",
        description="Radians to rotate in place so the robot's rear faces the charger",
    )

    charging_backup_distance = LaunchConfiguration("charging_backup_distance")
    declare_charging_backup_distance_arg = DeclareLaunchArgument(
        "charging_backup_distance",
        default_value="0.8",
        description="Maximum meters to reverse into the charger; stops early on contact/stall",
    )

    charging_contact_topic = LaunchConfiguration("charging_contact_topic")
    declare_charging_contact_topic_arg = DeclareLaunchArgument(
        "charging_contact_topic",
        default_value="charger_contact",
        description="ros_gz_interfaces/Contacts of this robot's body (relative: "
                     "/<namespace>/charger_contact). Empty = wheel-stall detection only.",
    )

    charging_linear_speed = LaunchConfiguration("charging_linear_speed")
    declare_charging_linear_speed_arg = DeclareLaunchArgument(
        "charging_linear_speed",
        default_value="0.05",
        description="Linear speed (m/s) used for the charging-dock backup maneuver",
    )

    charging_angular_speed = LaunchConfiguration("charging_angular_speed")
    declare_charging_angular_speed_arg = DeclareLaunchArgument(
        "charging_angular_speed",
        default_value="0.8",
        description="Angular speed (rad/s) used for the charging-dock rotate maneuver",
    )

    pallet_dock_workflow_node = Node(
        package="forklift_docking",
        executable="pallet_dock_workflow_node",
        name="pallet_dock_workflow_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "dock_database": dock_database,
                "dock_pallet_topic": dock_pallet_topic,
                "post_dock_delay": post_dock_delay,
                "lift_target_height": lift_target_height,
                "lift_speed": lift_speed,
                "dock_action_name": dock_action_name,
                "undock_action_name": undock_action_name,
                "set_height_service_name": set_height_service_name,
                "undock_max_time": undock_max_time,
                "set_height_timeout": set_height_timeout,
                "aruco_node_name": aruco_node_name,
                "lift_height_tolerance": lift_height_tolerance,
                "charging_pre_rotate_backoff": charging_pre_rotate_backoff,
                "charging_rotate_angle": charging_rotate_angle,
                "charging_backup_distance": charging_backup_distance,
                "charging_linear_speed": charging_linear_speed,
                "charging_angular_speed": charging_angular_speed,
                "charging_contact_topic": charging_contact_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_use_sim_time_arg,
            declare_dock_database_arg,
            declare_dock_pallet_topic_arg,
            declare_post_dock_delay_arg,
            declare_lift_target_height_arg,
            declare_lift_speed_arg,
            declare_dock_action_name_arg,
            declare_undock_action_name_arg,
            declare_set_height_service_name_arg,
            declare_undock_max_time_arg,
            declare_set_height_timeout_arg,
            declare_aruco_node_name_arg,
            declare_lift_height_tolerance_arg,
            declare_charging_pre_rotate_backoff_arg,
            declare_charging_rotate_angle_arg,
            declare_charging_backup_distance_arg,
            declare_charging_linear_speed_arg,
            declare_charging_angular_speed_arg,
            declare_charging_contact_topic_arg,
            pallet_dock_workflow_node,
        ]
    )