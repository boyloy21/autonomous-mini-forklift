#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    The function creates a launch sequence that ensures controllers are started
    in the correct order.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    
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
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_spawner,
                lift_position_spawner,
            ],
        )
    )
    
    # Add delay to joint state broadcaster (if necessary)
    delayed_start = TimerAction(
        period=25.0,
        actions=[joint_state_broadcaster_spawner]
    )

    
    # Create and populate the launch description
    ld = LaunchDescription()

    # Add the actions to the launch description in sequence
    ld.add_action(delayed_start)
    ld.add_action(delayed_controllers)

    return ld