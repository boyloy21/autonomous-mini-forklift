#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_aruco")
    param_file = os.path.join(pkg_share, "config", "aruco_parameters.yaml")

    detect_node = Node(
        package="forklift_aruco",
        executable="detect_aruco_video",
        name="detect_aruco_video",
        output="screen",
        parameters=[param_file],
    )

    pose_node = Node(
        package="forklift_aruco",
        executable="pose_estimate",
        name="pose_estimate",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription([detect_node, pose_node])
