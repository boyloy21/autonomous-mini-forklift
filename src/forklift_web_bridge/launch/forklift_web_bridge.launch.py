#!/usr/bin/env python3
"""
web_bridge.launch.py

Starts:
  - rosbridge_websocket (port 9090) -- WebSocket entry point for roslibjs
  - web_video_server    (port 8080) -- camera streams for the dashboard
  - bridge_manager_node, robot_state_node, navigation_bridge_node,
    mission_bridge_node, diagnostics_bridge_node, fastapi_bridge_node
    (REST API, rest_port)

Multi-robot: pass namespace:=forklift1 (and distinct ports per robot, e.g.
rosbridge_port:=9091 video_port:=8081 rest_port:=8001). Every node then runs
under /forklift1 and all relative topic/action names in web_bridge.yaml
(odom, navigate_to_pose, web_bridge/*, ...) resolve to /forklift1/...
Default namespace is "" (single robot, root namespace).

Does NOT start Nav2, forklift_mission_manager, forklift_task_manager, or
forklift_diagnostics -- this package only bridges to them, all must
already be running (part of forklift_bringup's combined launch).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_web_bridge")
    web_bridge_yaml = os.path.join(pkg_share, "config", "web_bridge.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo /clock instead of wall time",
    )

    namespace = LaunchConfiguration("namespace")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    video_port = LaunchConfiguration("video_port")
    rest_port = LaunchConfiguration("rest_port")
    declare_args = [
        DeclareLaunchArgument("namespace", default_value="",
                              description="Robot namespace; empty = single robot"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090",
                              description="rosbridge WebSocket port (unique per robot)"),
        DeclareLaunchArgument("video_port", default_value="8080",
                              description="web_video_server port (unique per robot)"),
        DeclareLaunchArgument("rest_port", default_value="8000",
                              description="FastAPI REST port (unique per robot)"),
    ]

    # ---- rosbridge WebSocket ----
    rosbridge_server = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_server_node",
        output="screen",
        namespace=namespace,
        parameters=[{"port": ParameterValue(rosbridge_port, value_type=int)}, {"address": "0.0.0.0"}],
    )

    # ---- Camera streaming ----
    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server_node",
        output="screen",
        namespace=namespace,
        parameters=[{"port": ParameterValue(video_port, value_type=int)}, {"address": "0.0.0.0"}, {"server_threads": 4}],
    )

    # ---- Custom bridge nodes ----
    bridge_manager_node = Node(
        package="forklift_web_bridge",
        executable="bridge_manager_node",
        name="bridge_manager_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[web_bridge_yaml, {"use_sim_time": use_sim_time}],
    )

    robot_state_node = Node(
        package="forklift_web_bridge",
        executable="robot_state_node",
        name="robot_state_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[web_bridge_yaml, {"use_sim_time": use_sim_time}],
    )

    navigation_bridge_node = Node(
        package="forklift_web_bridge",
        executable="navigation_bridge_node",
        name="navigation_bridge_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[web_bridge_yaml, {"use_sim_time": use_sim_time}],
    )

    mission_bridge_node = Node(
        package="forklift_web_bridge",
        executable="mission_bridge_node",
        name="mission_bridge_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[web_bridge_yaml, {"use_sim_time": use_sim_time}],
    )

    diagnostics_bridge_node = Node(
        package="forklift_web_bridge",
        executable="diagnostics_bridge_node",
        name="diagnostics_bridge_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[web_bridge_yaml, {"use_sim_time": use_sim_time}],
    )

    fastapi_bridge_node = Node(
        package="forklift_web_bridge",
        executable="fastapi_bridge_node",
        name="fastapi_bridge_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[web_bridge_yaml, {"use_sim_time": use_sim_time,
                     "rest_port": ParameterValue(rest_port, value_type=int)}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_arg,
            *declare_args,
            rosbridge_server,
            web_video_server,
            bridge_manager_node,
            robot_state_node,
            navigation_bridge_node,
            mission_bridge_node,
            diagnostics_bridge_node,
            fastapi_bridge_node,
        ]
    )