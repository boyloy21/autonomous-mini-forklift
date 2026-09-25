import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_safety")
    config_dir = os.path.join(pkg_share, "config")
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    
    all_configs = [
        os.path.join(config_dir, "safety.yaml"),
        os.path.join(config_dir, 'speed_zone.yaml'),
        os.path.join(config_dir, 'sensor.yaml')
    ]
    
    def make_node(executable):
        return Node(
            package='forklift_safety',
            executable=executable,
            name=executable,
            output='screen',
            emulate_tty=True,
            parameters=[*all_configs, {'use_sim_time': use_sim_time}]
        )
        
        
    nodes = [
        make_node('sensor_monitor_node'),
        make_node('obstacle_monitor_node'),
        make_node('protective_field_node'),
        make_node('speed_zone_node'),
        make_node('emergency_stop_node'),
        make_node('safety_io_node'),
        make_node('safety_manager_node'),
    ]
    return LaunchDescription([use_sim_time_arg, *nodes])