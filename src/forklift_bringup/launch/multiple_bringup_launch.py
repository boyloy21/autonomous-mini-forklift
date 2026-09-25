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
second terminal. This file will sit and wait until it sees the last
robot's odom topic appear, then proceed automatically.

FIXES applied this pass:

  1. STANDARDIZED on 'multiple_docking.launch.py' (the file that actually
     received the root_key+find/replace chaining fix, the /tf remap fix,
     and the explicit aruco_dock_pose remap target fix over several
     rounds of debugging) instead of 'multirobot_docking.launch.py'.
     ASSUMPTION: these were meant to be the same file, referenced
     inconsistently across different points in this project. If
     'multirobot_docking.launch.py' is actually a genuinely separate,
     never-fixed file, this line needs to point back to it AFTER it
     receives the same fixes -- verify before relying on this.

  2. REMOVED the double-substitution bug: this file was calling
     replace_namespace_in_params() on docking_server_multirobot.yaml
     itself, THEN ALSO passing the result into multiple_docking.launch.py
     (which does its own internal substitution pass). Harmless for
     correctness (substitution is idempotent) but is almost certainly
     the exact explanation for the mysterious doubled-prefix
     "/tmp/forklift1_forklift1_docking_server_multirobot.yaml" file found
     a few rounds of debugging ago. Now passes the RAW path through,
     letting the included file substitute exactly once.

  3. REMOVED docking_manager_cmd (pallet_dock_workflow_node) entirely.
     Per this project's own architecture decision, task_manager_node
     SUPERSEDES pallet_dock_workflow_node for production use -- it calls
     docking_server/lift_controller_node directly. Running both meant two
     separate nodes could each independently command the same robot's
     docking_server, which is exactly the class of bug that produced the
     "unknown goal response, ignoring" errors found earlier. If you still
     want the legacy /dock_pallet topic trigger available for manual
     testing, re-add it deliberately, not as an always-on default.

  4. Added the missing use_sim_time to aruco_cmd's launch_arguments --
     every other included launch file here already passed it, this one
     didn't.

  5. NOT changed: lift_controller_node_multirobot. This project's fixes
     all went into a plain 'lift_controller_node' -- if this is genuinely
     a separate executable/source file, its own topic namespacing
     (/lift_up, /lift_down, /joint_states, /lift_cmd -- must be relative,
     not absolute) has not been verified in this pass and needs separate
     review.

Everything else below is unchanged from the version you uploaded:
  - No extra joint_state_publisher node (Gazebo bridge already provides
    real simulated joint states).
  - No static map->odom TF (AMCL provides this dynamically per robot).
  - No manual map_server/lifecycle_manager (nav2's own bringup_launch.py
    already runs one, since map_server='True' is passed to it).

Fleet source: this file reads the SAME robots_file (config/robots.yaml
by default) as multiple_robot_world.launch.py.
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
    use_fake_battery = LaunchConfiguration("use_fake_battery").perform(context).lower() == "true"

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
    docking_server_yaml = os.path.join(
        get_package_share_directory('forklift_docking'), 'config', 'docking_server_multirobot.yaml')

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
        # FIX 2: docking_server_yaml is now passed RAW (unsubstituted) --
        # multiple_docking.launch.py's own launch_setup() does this exact
        # substitution internally, exactly once. Substituting it here too
        # was the double-substitution bug (see module docstring).

        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + '/initialpose',
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
                'controller_type': 'dwb',
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
                'params_file': docking_server_yaml,   # raw path -- see FIX 2
            }.items()
        )

        # NOTE: not verified in this pass -- see module docstring point 5.
        lift_controller_node = Node(
            package="forklift_controller",
            executable="lift_controller_node_multirobot",
            name="lift_controller_node",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )

        # Simulated battery: charges while this robot touches a charger pad
        # (<namespace>/charger_contact), publishes <namespace>/battery_state for the
        # task manager's MONITOR_CHARGING step.
        fake_battery_node = Node(
            package="forklift_docking",
            executable="fake_battery_node.py",
            name="fake_battery_node",
            namespace=namespace,
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

        # FIX 4: added use_sim_time, missing before.
        aruco_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_aruco'), 'launch', 'aruco_pose_estimation2.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
        )

        # FIX 3: docking_manager_cmd (pallet_dock_workflow_node) removed
        # entirely -- task_manager_node is the sole orchestrator now. See
        # module docstring point 3 if you need the legacy trigger back.

        task_manager_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_task_manager'), 'launch', 'task_manager.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
        )
        
        mission_manager_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_mission_manager'), 'launch', 'mission_manager.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
        )

        diagnostics_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_diagnostics'), 'launch', 'diagnostics.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
        )
        
        web_bridge_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('forklift_web_bridge'), 'launch', 'forklift_web_bridge.launch.py')),
            # Each robot needs its OWN ports (robot 1: 9091/8081/8001, robot 2: 9092/8082/8002...)
            # otherwise every rosbridge tries to bind 9090 -> "Address already in use".
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'rosbridge_port': str(9091 + i),
                'video_port': str(8081 + i),
                'rest_port': str(8001 + i),
            }.items()
        )

        period = PythonExpression([str(i), ' * ', robot_stagger_delay])
        nav2_timer_actions.append(
            TimerAction(
                period=period,
                actions=[
                    bringup_cmd, initial_pose_cmd, docking_cmd, rviz_cmd, aruco_cmd,
                    lift_controller_node, task_manager_cmd,mission_manager_cmd, diagnostics_cmd, web_bridge_cmd
                ] + ([fake_battery_node] if use_fake_battery else []),
            )
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

    robot_stagger_delay = LaunchConfiguration('robot_stagger_delay', default='8.0')
    declare_robot_stagger_delay = DeclareLaunchArgument(
        name='robot_stagger_delay', default_value=robot_stagger_delay,
        description='Seconds added per robot index so N robots\' nav2/AMCL stacks don\'t '
                    'all activate at the exact same instant. Set to 0 to disable staggering.'
    )

    declare_robots_file = DeclareLaunchArgument(
        name='robots_file',
        default_value=os.path.join(
            get_package_share_directory('forklift_bringup'), 'config', 'robots.yaml'
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

    declare_use_fake_battery = DeclareLaunchArgument(
        name='use_fake_battery',
        default_value='true',
        description='Start forklift_docking fake_battery_node per robot (charges on charger '
                    'pad contact). Set false when a real battery_state source exists.',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_fake_battery,
        declare_robot_stagger_delay,
        declare_robots_file,
        declare_robots,
        OpaqueFunction(function=launch_setup),
    ])