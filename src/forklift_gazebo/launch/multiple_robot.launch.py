#!/usr/bin/env python3
"""
Multi-robot launch file for mini_forklift_ws / forklift_gazebo.

Pattern combines:
  - REZ3LIET/IGN_Multi_Robot_Nav2: per-robot namespace + robot_state_publisher
    + ros_gz_bridge, driven by a robots[] list of name/pose dicts.
  - arshadlab/tb3_multi_robot: sequential spawning via
    RegisterEventHandler(OnProcessExit) so Ignition/Gazebo doesn't choke on
    parallel spawn calls.

IMPORTANT — READ BEFORE RUNNING:
Your current forklift_gazebo/models/mini_forklift/model.sdf has NO <plugin>
tags (no DiffDrive controller, no lidar/imu/camera <sensor> blocks). This
launch file assumes those plugins exist in whichever SDF/xacro your working
single-robot setup actually spawns, and that the DiffDrive controller's
<topic> can be overridden per-instance (either via a namespace-aware xacro,
or because the plugin already reads a <namespace> tag).

If your DiffDrive plugin has a HARDCODED topic like <topic>cmd_vel</topic>,
every spawned robot will collide on the same topic. Two ways to fix that:
  1. Convert model.sdf -> model.sdf.xacro with a ${namespace} placeholder
     inside the plugin's <topic>/<odom_topic>/<tf_topic> tags, then this
     launch file's xacro.process_doc() call will inject it per-robot
     (this is what reference #1 does).
  2. Or, if you don't want xacro, keep one static model.sdf and instead
     bridge everything through ros_gz_bridge with REMAPS per namespace
     (the bridge below already does this for cmd_vel/odom/scan/imu/tf/
     joint_states) -- but the plugin's INTERNAL gz-transport topic still
     needs to be unique, otherwise the bridge for robot2 will read
     robot1's data. Namespacing the plugin is the safer route.

Adjust ROBOT_SDF, WORLD_SDF, and the bridge argument list below to match
your actual working single-robot bridge config once you confirm it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('forklift_gazebo')
    # resource_path = os.path.dirname(pkg_path_robot)
    # world_file_path = os.path.join(pkg_path, 'worlds', 'small_warehouse.sdf')
    

    # ---------------------------------------------------------------
    # Robots to spawn: name, starting pose. Add/remove entries here.
    # ---------------------------------------------------------------
    robots = [
        {'name': 'forklift1', 'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': '0.05', 'yaw': '0.0'},
        {'name': 'forklift2', 'x_pose': '1.0', 'y_pose': '0.0', 'z_pose': '0.05', 'yaw': '0.0'},
        {'name': 'forklift3', 'x_pose': '0.0', 'y_pose': '-1.0', 'z_pose': '0.05', 'yaw': '0.0'},
        {'name': 'forklift4', 'x_pose': '1.0', 'y_pose': '-1.0', 'z_pose': '0.05', 'yaw': '0.0'},
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    # Let Gazebo find your models/meshes (forklift_description meshes referenced
    # via model:// URIs inside model.sdf).
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join('/opt/ros/humble', 'share') + ':' +
            os.path.join(pkg_share, 'models')
        ],
    )


    world_sdf = os.path.join(pkg_share, 'worlds', 'small_warehouse.sdf')
    robot_sdf = os.path.join(pkg_share, 'models', 'mini_forklift', 'model.sdf')

    # ---------------------------------------------------------------
    # Start Gazebo (server + GUI) with the warehouse world
    # ---------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v4 {world_sdf}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(ign_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)

    # Remapping /tf and /tf_static so each robot's state publisher doesn't
    # stomp on the root namespace.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None

    for robot in robots:
        namespace = robot['name']

        # If your model.sdf is a plain SDF with no ${namespace} placeholders,
        # xacro.process_doc() below is a no-op passthrough -- harmless to
        # leave in, but it won't namespace internal plugin topics. Once you
        # convert model.sdf -> model.sdf.xacro with placeholders, this starts
        # doing real work.
        doc = xacro.parse(open(robot_sdf))
        xacro.process_doc(doc, mappings={'namespace': namespace})
        robot_description = {'robot_description': doc.toxml()}

        state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            remappings=remappings,
        )

        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', robot['name'],
                '-topic', f'/{namespace}/robot_description',
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', robot['z_pose'],
                '-Y', robot['yaw'],
                '-allow_renaming', 'false',
            ],
        )

        # Adjust these to match your real single-robot bridge topics/types.
        # Sensor topics here assume Gazebo's per-model auto-namespaced gz
        # topics land on e.g. /forklift1/scan -- confirm against your working
        # single-robot bridge before trusting this list.
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                f'/{namespace}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/{namespace}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{namespace}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{namespace}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{namespace}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
        )
        
        transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.1', '--Y', '0.0',
                '--frame-id', f'{namespace}/camera_link',
                '--child-frame-id', f'{namespace}/lift_link/camera_link',
            ],
        )
        
        # joint_state_broadcaster_spawner = Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     namespace=namespace,
        #     arguments=[
        #         'joint_state_broadcaster',
        #         '--controller-manager', f'/{namespace}/controller_manager',
        #     ],
        #     output='screen',
        # )
 
        # diff_drive_spawner = Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     namespace=namespace,
        #     arguments=[
        #         'diff_drive_controller',
        #         '--controller-manager', f'/{namespace}/controller_manager',
        #     ],
        #     output='screen',
        # )
 
        # lift_position_spawner = Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     namespace=namespace,
        #     arguments=[
        #         'lift_position_controller',
        #         '--controller-manager', f'/{namespace}/controller_manager',
        #     ],
        #     output='screen',
        # )
 
        # --- per-robot EKF ---
        # TODO: confirm ekf_gazebo.launch.py accepts a namespace arg and
        # namespaces its internal node + remaps odom/imu topics
        # accordingly. If it doesn't yet, it needs to, or all robots'
        # EKF instances will fight over /odometry/filtered.
        # ekf_launch = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_localization, 'launch', 'ekf_gazebo.launch.py')
        #     ),
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'namespace': namespace,
        #     }.items(),
        # )
 
        # delayed_controllers = RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_robot,
        #         on_exit=[
        #             # joint_state_broadcaster_spawner,
        #             # diff_drive_spawner,
        #             # lift_position_spawner,
        #             transform_publisher,
        #             # ekf_launch,
        #         ],
        #     )
        # )

        if last_action is None:
            ld.add_action(state_publisher)
            ld.add_action(spawn_robot)
            ld.add_action(bridge)
            ld.add_action(transform_publisher)
        else:
            spawn_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[state_publisher, spawn_robot, bridge, transform_publisher],
                )
            )
            ld.add_action(spawn_event)

        last_action = spawn_robot

    return ld