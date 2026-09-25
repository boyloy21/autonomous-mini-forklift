"""
docking.launch.py

Starts:
  1. docking_server            (opennav_docking executable, our config)
  2. lifecycle_manager_docking (brings docking_server through
                                 configure -> activate automatically)

Two things this launch file does that the YAML config alone cannot:
  - Overrides `dock_database` with a path computed from this package's own
    install location, so the config isn't tied to one user's home
    directory (the yaml currently hardcodes
    /home/yunboyloy/miniforklift_ws/... -- this override takes precedence
    and works on any machine the package is installed on).
  - Remaps SimpleChargingDock's internal, hardcoded subscription topic
    "detected_dock_pose" to "/dock_pose" -- this is NOT configurable via
    any YAML parameter. Without this remap, docking_server never receives
    any ArUco-based correction pose at all.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_docking")

    docking_server_yaml = os.path.join(pkg_share, "config", "docking_server.yaml")
    dock_database_yaml = os.path.join(pkg_share, "config", "dock_database.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo /clock instead of wall time",
    )

    autostart = LaunchConfiguration("autostart")
    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically bring docking_server to the active lifecycle state",
    )

    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            docking_server_yaml,
            {
                "use_sim_time": use_sim_time,
                # portable override -- takes precedence over the hardcoded
                # absolute path inside docking_server.yaml
                "dock_database": dock_database_yaml,
            },
        ],
        remappings=[
            ("detected_dock_pose", "/aruco_dock_pose"),  # was "/dock_pose"
        ],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_docking",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["docking_server"],
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_arg,
            declare_autostart_arg,
            docking_server_node,
            lifecycle_manager_node,
        ]
    )