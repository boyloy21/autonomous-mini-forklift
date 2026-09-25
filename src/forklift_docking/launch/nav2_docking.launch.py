import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_docking")
    marker_dock_config = os.path.join(pkg_share, "config", "dock_marker.yaml")
    dock_database= os.path.join(pkg_share, "config", "dock_database.yaml")
    docking_server_params = os.path.join(pkg_share, "config", "docking_server_params.yaml")
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", 
        description="Use simulataion (Gazebo) clock if true"
    )
    
    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn", default_value="true",
        description="Whether to respawn docking_server if it crashes match Nav2's own bringup default"
    )
    
    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="log level (debug, info , warn, error)"
    )
    
    brige_node = Node(
        package="forklift_docking",
        executable="aruco_dock_bridge",
        name="aruco_dock_bridge",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"marker_dock_config_file": marker_dock_config, "use_sim_time": use_sim_time}]
    )
    
    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="opennav_docking",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[docking_server_params, {"dock_database" : dock_database, "use_sim_time" : use_sim_time}]
    )
    
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="docking_server_lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[docking_server_params, {"use_sim_time" : use_sim_time}],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_respawn,
        declare_log_level,
        brige_node,
        docking_server_node,
        lifecycle_manager_node,
    ])