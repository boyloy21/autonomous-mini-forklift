#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Kept as a SEPARATE launch file from multiple_robot_world.launch.py, run
second, after the world file. Two independent `ros2 launch` processes
can't share a process handle, so the OnProcessExit-based "wait for the
last robot to finish spawning" gate used in a combined draft of this file
isn't possible here directly.

What IS possible: both launch processes join the SAME ROS 2 graph over
DDS, so this file can wait for evidence that the last robot in the fleet
has actually finished spawning by polling for one of its topics (e.g.
`/forklift5/odom`) before starting any nav2/AMCL stacks. That's the
`wait_for_last_robot` action below - a process-boundary-safe substitute
for the OnProcessExit gate, not a downgrade of it.

USAGE: start multiple_robot_world.launch.py first, then this file, in a
second terminal (or however you're already sourcing/launching). This file
will sit and wait (printing nothing alarming) until it sees the last
robot's odom topic appear, then proceed automatically - no need to time
it manually.

Fleet source: this file now reads the SAME robots_file (config/robots.yaml
by default) as multiple_robot_world.launch.py, instead of its own
separate, previously out-of-sync, partially-commented-out hardcoded list.
This avoids bringing up nav2 stacks for robots that were never spawned
(or missing stacks for robots that were).

Everything else below is unchanged from the version you uploaded:
  - No extra joint_state_publisher node (Gazebo bridge already provides
    real simulated joint states).
  - No static map->odom TF (AMCL provides this dynamically per robot).
  - No manual map_server/lifecycle_manager (nav2's own bringup_launch.py
    already runs one, since map_server='True' is passed to it).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import yaml


def replace_namespace(d, old_str, new_str):
    if isinstance(d, dict):
        for key, value in d.items():
            d[key] = replace_namespace(value, old_str, new_str)
    elif isinstance(d, list):
        for i in range(len(d)):
            d[i] = replace_namespace(d[i], old_str, new_str)
    elif isinstance(d, str):
        return d.replace(old_str, new_str)
    return d


def replace_namespace_in_params(yaml_file_path, namespace):
    if not namespace.startswith('/'):
        namespace = '/' + namespace

    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)

    data = replace_namespace(data, 'namespace', namespace)

    modified_yaml = yaml.dump(data, default_flow_style=False)

    # Output path includes the SOURCE file's basename AND extension, not
    # just the namespace -- otherwise calling this twice for the same
    # robot with two different source files (e.g. nav2_params.yaml AND
    # an rviz config) would collide on the exact same /tmp path and
    # silently overwrite each other. Extension preserved (not hardcoded
    # to .yaml) so an .rviz source stays .rviz.
    source_name, source_ext = os.path.splitext(os.path.basename(yaml_file_path))
    path = f"/tmp{namespace}_{source_name}{source_ext}"
    with open(path, 'w') as file:
        file.write(modified_yaml)

    print(f"Modified YAML has been saved to {path}")
    return path


def launch_setup(context, *args, **kwargs):
    """
    Everything needing the actual robot list as real Python objects (not a
    LaunchConfiguration placeholder) lives here - OpaqueFunction is the
    only place `.perform(context)` can resolve launch args to concrete
    strings before we loop over them. Mirrors multiple_robot_world.launch.py's
    loader so both files always agree on the fleet.
    """
    robots_file = LaunchConfiguration("robots_file").perform(context)
    robots_arg = LaunchConfiguration("robots").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_stagger_delay = LaunchConfiguration("robot_stagger_delay")

    if robots_arg.strip():
        robots = yaml.safe_load(robots_arg)
    else:
        with open(robots_file, "r") as f:
            data = yaml.safe_load(f)
        robots = data["robots"] if isinstance(data, dict) else data

    if not isinstance(robots, list) or not robots:
        raise RuntimeError(
            "Fleet must decode to a non-empty list of {name, x_pose, y_pose, "
            "z_pose} dicts. Got: %r (from %s)"
            % (robots, "robots arg" if robots_arg.strip() else robots_file)
        )

    for robot in robots:
        robot["x_pose"] = str(robot["x_pose"])
        robot["y_pose"] = str(robot["y_pose"])
        robot["z_pose"] = str(robot["z_pose"])
        robot["name"] = str(robot["name"])

    forklift_navigation = get_package_share_directory('forklift_navigation')
    nav_launch_dir = os.path.join(forklift_navigation, 'launch')
    params_file = os.path.join(forklift_navigation, 'config', 'nav2_multirobot_params.yaml')
    yaml_filename = os.path.join(forklift_navigation, 'map', 'small_warehouse.yaml')
    rviz_config_template = os.path.join(forklift_navigation, 'rviz', 'nav2_multirobot_view.rviz')
    docking_server_yaml = os.path.join(get_package_share_directory('forklift_docking'), 'config', 'docking_server_multirobot.yaml')

    # -------------------------------------------------------------------
    # Wait gate: substitutes for the OnProcessExit chain from the combined
    # draft, which can't work across two separate `ros2 launch` processes.
    # Polls the ROS graph (which both processes share via DDS) for the
    # LAST robot's odom topic, which only exists once its bridge Node in
    # multiple_robot_world.launch.py is up and running - i.e. that robot
    # has genuinely finished spawning, not just "some seconds have passed".
    # -------------------------------------------------------------------
    last_robot_name = robots[-1]["name"]
    wait_topic = f"/{last_robot_name}/odom"
    wait_for_last_robot = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f'echo "[multiple_bringup_launch] waiting for {wait_topic} '
            f'(i.e. all {len(robots)} robots spawned in Gazebo)..."; '
            f'until ros2 topic list 2>/dev/null | grep -qx "{wait_topic}"; '
            f'do sleep 1; done; '
            f'echo "[multiple_bringup_launch] {wait_topic} found, proceeding with nav2 bringup."',
        ],
        output="screen",
    )

    nav2_timer_actions = []
    for i, robot in enumerate(robots):
        namespace = '/' + robot['name']

        namespaced_param_file = replace_namespace_in_params(params_file, namespace)
        namespaced_docking_file = replace_namespace_in_params(docking_server_yaml, namespace)

        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        # Seeds this robot's starting pose (nav2_multirobot_params.yaml's
        # amcl block has no declarative set_initial_pose/initial_pose of
        # its own, so this is the only thing that sets it to the spawn
        # coordinates from robots.yaml instead of AMCL's default).
        #
        # Publish ONCE ('-1'), not repeatedly: amcl_node.cpp's
        # handleInitialPose() does an immediate, no-wait tf lookup against
        # now() every time it receives /initialpose. The very first one is
        # exempted (AMCL hasn't broadcast map->odom yet), but any repeat
        # publish that lands after AMCL is already running races the next
        # odom tf sample by a few ms and logs "Failed to transform initial
        # pose in time" -- harmless (AMCL just skips the odometric
        # correction for that one update), but pure log spam since a
        # single publish already reliably seeds the pose.
        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-1', '--qos-reliability', 'reliable', namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'True',
                'map': yaml_filename,
                'map_server': 'True',
                'params_file': namespaced_param_file,
                'controller_type': 'mppi',
                # 'default_bt_xml_filename': os.path.join(
                #     get_package_share_directory('nav2_bt_navigator'),
                #     'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                'autostart': 'true',
                'use_sim_time': use_sim_time,
                'use_composition': 'False'
            }.items()
        )

        docking_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('forklift_docking'), 'launch', 'multirobot_docking.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': namespaced_docking_file,   # <-- the missing piece
            }.items()
        )

        lift_controller_node = Node(
            package="forklift_controller",
            executable="lift_controller_node_multirobot",
            name="lift_controller_node",
            namespace=namespace,   # <-- if missing, same collision bug as docking_server had
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )

        rviz_config_file = replace_namespace_in_params(rviz_config_template, namespace)
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': [namespace],
                'use_namespace': 'True',
                'rviz_config': rviz_config_file,
                'log_level': 'warn',
            }.items()
        )

        aruco_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_aruco'), 'launch', 'aruco_pose_estimation2.launch.py')),
            launch_arguments={'namespace': namespace}.items()
        )

        docking_manager_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_docking'), 'launch', 'docking_manager.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
        )

        task_manager_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_task_manager'), 'launch', 'task_manager.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
        )

        # All three actions for this robot fire together at this robot's
        # own staggered time -- no dependency on any other robot's
        # process, no chaining, no NameError.
        period = PythonExpression([str(i), ' * ', robot_stagger_delay])
        nav2_timer_actions.append(
            TimerAction(period=period, actions=[bringup_cmd, docking_cmd, rviz_cmd, initial_pose_cmd, aruco_cmd, lift_controller_node, docking_manager_cmd, task_manager_cmd])
        )

    gate_on_wait = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_last_robot,
            on_exit=nav2_timer_actions,
        )
    )

    return [wait_for_last_robot, gate_on_wait]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    robot_stagger_delay = LaunchConfiguration('robot_stagger_delay', default='6.0')
    declare_robot_stagger_delay = DeclareLaunchArgument(
        name='robot_stagger_delay', default_value=robot_stagger_delay,
        description='Seconds added per robot index so N robots\' nav2/AMCL stacks don\'t '
                    'all activate at the exact same instant. Set to 0 to disable staggering.'
    )

    # Same default path/format as multiple_robot_world.launch.py's
    # robots_file, so both files read the identical fleet by default.
    declare_robots_file = DeclareLaunchArgument(
        name='robots_file',
        default_value=os.path.join(
            get_package_share_directory('forklift_gazebo'), 'config', 'robots.yaml'
        ),
        description="Path to a YAML file with a top-level 'robots' list (name, x_pose, y_pose, z_pose)."
                    " Must match the one multiple_robot_world.launch.py used.",
    )

    declare_robots = DeclareLaunchArgument(
        name='robots',
        default_value='',
        description='Optional inline JSON/YAML robot list, overrides robots_file when non-empty.'
                    ' Must match what multiple_robot_world.launch.py was given, if used there too.',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_stagger_delay,
        declare_robots_file,
        declare_robots,
        OpaqueFunction(function=launch_setup),
    ])
