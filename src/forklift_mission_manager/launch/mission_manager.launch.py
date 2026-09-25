"""
mission_manager.launch.py

Starts mission_server -- the ExecuteMission action server. Client of
forklift_task_manager's ExecuteTask action; does not itself start
task_manager, docking_server, nav2, or lift_controller_node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_mission_manager")

    default_missions_db = os.path.join(pkg_share, "config", "missions.yaml")
    
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the mission server"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo /clock instead of wall time",
    )

    mission_database = LaunchConfiguration("mission_database")
    declare_mission_database_arg = DeclareLaunchArgument(
        "mission_database",
        default_value=default_missions_db,
        description="Absolute path to missions.yaml (optional named mission templates)",
    )

    execute_task_action_name = LaunchConfiguration("execute_task_action_name")
    declare_execute_task_action_arg = DeclareLaunchArgument(
        "execute_task_action_name",
        default_value="execute_task",
        description="forklift_task_manager's ExecuteTask action name",
    )

    mission_server_node = Node(
        package="forklift_mission_manager",
        executable="mission_server",
        name="mission_server",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "mission_database": mission_database,
                "execute_task_action_name": execute_task_action_name,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_use_sim_time_arg,
            declare_mission_database_arg,
            declare_execute_task_action_arg,
            mission_server_node,
        ]
    )