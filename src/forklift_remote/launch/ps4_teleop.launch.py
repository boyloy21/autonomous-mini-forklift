from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    ps4_control_node = Node(
        package='forklift_remote',
        executable='ps4_control_node',
        name='ps4_control_node',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        ps4_control_node,
    ])