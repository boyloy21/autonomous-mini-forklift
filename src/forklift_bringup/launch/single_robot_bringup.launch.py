#!/usr/bin/env python3
"""
forklift_bringup.launch.py

Single command replacing the manual 6-step bring-up sequence:

  1. forklift_gazebo          - robot model + sensors in Gazebo         (t=0s)
  2. forklift_navigation      - nav2 + localization + docking_server    (t=navigation_delay)
  3. forklift_controller      - ros2_control controllers                (t=controller_delay)
  4. forklift_aruco           - ArUco marker pose estimation            (t=aruco_delay)
  5. forklift_task_manager    - ExecuteTask action server               (t=task_manager_delay)
  6. forklift_mission_manager - ExecuteMission action server            (t=mission_manager_delay)

TIMING: delays are staggered via TimerAction, all measured from launch
start (t=0 = when this file itself starts, not from the end of the
previous stage). This is a heuristic, not a readiness guarantee -- there
is no cheap, general way in ROS2 launch to know "Gazebo has actually
finished loading the world and the robot is fully spawned" versus "the
Gazebo process has started" without a dedicated readiness-check node.
The defaults below are conservative starting points; if a stage still
comes up before its dependency is truly ready on your machine (slower
Gazebo world load, etc.), increase that stage's *_delay argument rather
than changing this file. All delays are overridable from the command
line without rebuilding.

forklift_task_manager and forklift_mission_manager are the least timing-
sensitive of the six: their downstream action clients call
wait_for_action_server() with their own timeouts INSIDE task/mission
execution, not at node construction, so they tolerate their dependencies
not being ready for the first several seconds after they themselves
start. The Gazebo -> navigation gap is the one that matters most --
localization and costmaps can behave badly if the robot's TF/topics
aren't publishing yet when they start.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    def pkg_launch(pkg, filename):
        return os.path.join(get_package_share_directory(pkg), "launch", filename)

    use_sim_time = LaunchConfiguration("use_sim_time")
    controller = LaunchConfiguration("controller")
    navigation_delay = LaunchConfiguration("navigation_delay")
    docking_delay = LaunchConfiguration("docking_delay")
    controller_delay = LaunchConfiguration("controller_delay")
    aruco_delay = LaunchConfiguration("aruco_delay")
    task_manager_delay = LaunchConfiguration("task_manager_delay")
    mission_manager_delay = LaunchConfiguration("mission_manager_delay")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Forwarded to every stage that accepts it"),
        DeclareLaunchArgument(
            "controller", default_value="mppi",
            description="Nav2 controller plugin, forwarded to forklift_navigation's bringup_launch.py "
                        "(matches your existing `controller:=mppi` argument)"),
        DeclareLaunchArgument(
            "navigation_delay", default_value="2.0",
            description="Seconds after this launch starts before starting navigation/localization/"
                        "docking -- needs Gazebo's world fully loaded and the robot spawned first"),
        DeclareLaunchArgument(
            "docking_delay", default_value="2.0",
            description="Seconds after this launch starts before starting docking_server -- needs Gazebo's world fully loaded and the robot spawned first"),
        DeclareLaunchArgument(
            "controller_delay", default_value="10.0",
            description="Seconds before starting forklift_controller"),
        DeclareLaunchArgument(
            "aruco_delay", default_value="10.0",
            description="Seconds before starting forklift_aruco -- needs camera topics publishing"),
        DeclareLaunchArgument(
            "task_manager_delay", default_value="14.0",
            description="Seconds before starting forklift_task_manager"),
        DeclareLaunchArgument(
            "mission_manager_delay", default_value="16.0",
            description="Seconds before starting forklift_mission_manager"),
    ]


    # ---- 2. Navigation (nav2 + localization + docking_server) ----
    navigation = TimerAction(
        period=navigation_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch("forklift_navigation", "bringup_launch.py")),
                launch_arguments={
                    "controller": controller,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )
    
    docking = TimerAction(
        period=docking_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch("forklift_docking", 'docking.launch.py')),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ]
    )
    
    rviz = TimerAction(
        period=navigation_delay,
        actions=[  
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch('forklift_navigation', 'rviz_launch.py')),
            )
        ]
    )

    # ---- 3. Controller ----
    controller_stage = TimerAction(
        period=controller_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch("forklift_controller", "forklift_controller.launch.py")),
            )
        ],
    )

    # ---- 4. ArUco detection ----
    aruco = TimerAction(
        period=aruco_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch("forklift_aruco", "aruco_pose_estimation2.launch.py")),
            )
        ],
    )

    # ---- 5. Task manager ----
    task_manager = TimerAction(
        period=task_manager_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch("forklift_task_manager", "task_manager.launch.py")),
            )
        ],
    )

    # ---- 6. Mission manager ----
    mission_manager = TimerAction(
        period=mission_manager_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_launch("forklift_mission_manager", "mission_manager.launch.py")),
            )
        ],
    )

    return LaunchDescription(
        declare_args + [
            navigation,
            docking,
            rviz,
            controller_stage,
            aruco,
            task_manager,
            mission_manager,
        ]
    )