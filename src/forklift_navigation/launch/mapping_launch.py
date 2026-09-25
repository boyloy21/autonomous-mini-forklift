import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_nav2 = get_package_share_directory('forklift_navigation')
    pkg_nav2_map_server = get_package_share_directory('nav2_map_server')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the slam_toolbox stack'
    )
    
    
    # Inclusde slam_toolbox launch file
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_nav2, 'config', 'map_params_online_async.yaml')
        }.items()
    )
    
    
    # Launch Rviz
    rviz_launch_cmd = Node(
        package='rviz2',
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d', os.path.join(pkg_nav2, 'rviz', 'map.rviz')
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    # Static transform publisher
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    map_saver_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_map_server, 'launch', 'map_saver_server.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_url': '/home/yunboyloy/miniforklift_ws/src/forklift_navigation/map'
        }.items()
    )
    

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(joint_state_publisher)
    ld.add_action(slam_toolbox_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(map_saver_server)
    

    return ld

