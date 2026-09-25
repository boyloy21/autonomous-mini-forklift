import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_controller")
    default_config = os.path.join(pkg_share, "config", "controller.yaml")
    
    
    config_arg = DeclareLaunchArgument('config_file', default_value=default_config)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # def make_node(executable):
    #     return Node(
    #         package='forklift_controller',
    #         executable=executable,
    #         name=executable,
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[config_file, {'use_sim_time': use_sim_time}],
    #     )
    
    # nodes = [
    #     # make_node('lift_safety_node'),
    #     # make_node('lift_limit_node'),
    #     # make_node('lift_feedback_node'),
    #     # make_node('lift_driver_node'),
    #     make_node('lift_controller_node'),
    #     # make_node('lift_cmd_bridge_node'),
    # ]
    lift_controller_node = Node(
        package="forklift_controller",
        executable="lift_controller_node",
        name="lift_controller_node",
        namespace=namespace,   # <-- if missing, same collision bug as docking_server had
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    return LaunchDescription([use_sim_time_arg, namespace_arg, lift_controller_node])