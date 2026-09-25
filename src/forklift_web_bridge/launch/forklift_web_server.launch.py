from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Include the Web_server launch file
    web_bridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_server_node',
        output='screen',
        parameters=[
            {'port': 9090},
            {'address': '0.0.0.0'}
        ]
    )
    
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server_node',
        output='scree',
        parameters=[
            {'port': 8080},
            {'address': '0.0.0.0'},
            {'server_threads': 4}
        ]
    )
    return LaunchDescription([web_bridge_server])