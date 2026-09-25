#!/usr/bin/env python3
"""
diagnostics.launch.py

Starts sensor_monitor_node and system_monitor_node, both publishing to
/diagnostics (diagnostic_msgs/DiagnosticArray). No robot control or
safety-stop logic anywhere in this package -- monitoring/reporting only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_diagnostics")
    diagnostics_yaml = os.path.join(pkg_share, "config", "diagnostics.yaml")
    
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for all nodes"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo /clock instead of wall time",
    )

    sensor_monitor_node = Node(
        package="forklift_diagnostics",
        executable="sensor_monitor_node",
        name="sensor_monitor_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[diagnostics_yaml, {"use_sim_time": use_sim_time}],
    )

    system_monitor_node = Node(
        package="forklift_diagnostics",
        executable="system_monitor_node",
        name="system_monitor_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[diagnostics_yaml, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_use_sim_time_arg,
            sensor_monitor_node,
            system_monitor_node,
        ]
    )