#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit




def generate_launch_description():
    pkg_path_robot = get_package_share_directory('forklift_description')
    pkg_path = get_package_share_directory('forklift_gazebo')
    pkg_localization = get_package_share_directory('forklift_localization')
    position_x = LaunchConfiguration('position_x', default='0.0')
    position_y = LaunchConfiguration('position_y', default='0.0')
    orientation_yaw = LaunchConfiguration('orientation_yaw', default='0.0')
    bridge_config_path = os.path.join(pkg_path, 'config', 'bridge_config.yaml')
    world_file_path = os.path.join(pkg_path, 'worlds', 'small_warehouse.sdf')
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    # pkg_share = get_package_share_directory("forklift_description")
    resource_path = os.path.dirname(pkg_path_robot)
    
    gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path
    )

    ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_path
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource((os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py'))),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items(),
    )
    

    # urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    # with open(urdf_file, "r") as f:
    #     robot_description = f.read()
        
    xacro_file = os.path.join(pkg_path_robot, 'urdf', 'robot.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'GAZEBO': 'true',
            'wheel_odom_topic': '/odom',
        }
    ).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
                'publish_frequency': 50.0,
                'ignore_timestamp': True
            }
        ],
        output='screen'
    )
    
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "forklift",
            "-allow_renaming", "true",
            "-x", position_x,
            "-y", position_y,
            "-z", "0.1",
            "-Y", orientation_yaw,
        ]
    )
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'container_name': 'ros_gz_bridge_container'
        }]
    )
    
    transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.1",
                    "--Y", "0.0",
                    "--frame-id", "camera_link",
                    "--child-frame-id", "forklift/lift_link/camera_link"]
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    lift_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_position_controller'],
        output='screen',
    )
    
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_spawner,
                lift_position_spawner,
            ],
        )
    )
    
    # ekf_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             pkg_localization,
    #             'launch',
    #             'ekf_gazebo.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true'
    #     }.items()
    # )
    
    return LaunchDescription([
        gz_resource,
        ign_resource,
        DeclareLaunchArgument('position_x', default_value='0.0', description='X position of the robot'),
        DeclareLaunchArgument('position_y', default_value='0.0', description='Y position of the robot'),
        DeclareLaunchArgument('orientation_yaw', default_value='0.0', description='Yaw orientation of the robot'),
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        bridge_node,
        # transform_publisher,
        # delayed_controllers,
        # ekf_launch
    ])