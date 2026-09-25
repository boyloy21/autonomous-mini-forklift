import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_path = get_package_share_directory('forklift_navigation')
    
    # Laucnch Cofiguration 
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    controller_type = LaunchConfiguration('controller_type')
    
    controller_params_file = LaunchConfiguration('controller_params_file')
    
    lifecycle_nodes = ['controller_server',
                           'smoother_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator',
                           'waypoint_follower',
                           'velocity_smoother',
                           'collision_monitor',
                           'route_server',
                           ]
    
    # lifecycle_nodes = ['controller_server',
    #                     'planner_server',
    #                     'behavior_server',
    #                     'bt_navigator',
    #                     'velocity_smoother',
    #                     'collision_monitor',
    #                     'route_server',]
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    configured_controller_params = ParameterFile(
        RewrittenYaml(
            source_file=controller_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top level namepsace'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description="Use simulation (Gazebo) clock if true")
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically nav stack'
    )
    
    
    declare_params_file = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='full path to the ROS2 parameters file to use for all launch nodes'
    )
    
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='pure_pursuit',
        choices=['pure_pursuit', 'mppi', 'dwb'],
        description="Controller to test: 'pure_pursuit', 'mppi', or 'dwb'. "
                     "Selects config/controller_<controller_type>.yaml as the default "
                     "controller_params_file."
    )
    
    declare_controller_params_file = DeclareLaunchArgument(
        'controller_params_file',
        default_value=[
            os.path.join(pkg_path, 'config', 'controller_'),
            controller_type,
            '.yaml'
        ],
        description='Full path to the controller_server-only params file'
    )
    
    
    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='false',
        description='Use composed bringup if true'
    )
    
    declare_container_name = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of container that nodes will load in if use composition'
    )
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    
    
    
    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_controller_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )
    
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_controller_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_route',
                plugin='nav2_route::RouteServer',
                name='route_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ]
    )
    
    
    ld = LaunchDescription()
    
    # Set enviroment variables
    ld.add_action(stdout_linebuf_envvar)
    
    # Declare the launch options
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_controller_type)          # must come before controller_params_file
    ld.add_action(declare_controller_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_container_name)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    
    return ld