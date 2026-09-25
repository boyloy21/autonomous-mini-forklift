import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('forklift_traffic_manager')
    default_params = os.path.join(pkg_share, 'config', 'traffic_manager.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to traffic_manager params file (declares the fleet robots list)',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock if true'
    )

    traffic_manager_node = Node(
        package='forklift_traffic_manager',
        executable='traffic_manager_node',
        name='traffic_manager',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(traffic_manager_node)
    return ld
