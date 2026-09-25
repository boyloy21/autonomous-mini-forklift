#!/usr/bin/env python3
"""
task_manager.launch.py

Starts task_manager_node -- the ExecuteTask action server orchestrating
NAVIGATE/DOCK/LIFT/LOWER/UNDOCK/WAIT and the composite PICK_PALLET/
DROP_PALLET task types.

Does NOT start docking_server, forklift_aruco, lift_controller_node, or
nav2 -- those must already be running; this node is purely an orchestrator
client of all of them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_task_manager")
    docking_pkg_share = get_package_share_directory("forklift_docking")

    task_manager_yaml = os.path.join(pkg_share, "config", "task_manager.yaml")

    # Reuse forklift_docking's own dock_database.yaml as the single source
    # of truth -- avoids a second, potentially-drifting copy.
    default_dock_database = os.path.join(docking_pkg_share, "config", "dock_database.yaml")
    
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", 
        default_value="", 
        description="Top-level namespace -- e.g. 'forklift1'. Empty means unnamespaced (single-robot use).",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo /clock instead of wall time",
    )

    dock_database = LaunchConfiguration("dock_database")
    declare_dock_database_arg = DeclareLaunchArgument(
        "dock_database",
        default_value=default_dock_database,
        description="Absolute path to dock_database.yaml (shared with forklift_docking)",
    )

    task_manager_node = Node(
        package="forklift_task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            task_manager_yaml,
            {
                "use_sim_time": use_sim_time,
                "dock_database": dock_database,
            },
        ],
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_use_sim_time_arg,
            declare_dock_database_arg,
            task_manager_node,
        ]
    )