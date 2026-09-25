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
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro
import yaml


# Default fleet now lives in config/robot.yaml (see declare_robots_file
# below) instead of being hardcoded here - edit that file to change the
# fleet without touching this launch file.


def launch_setup(context, *args, **kwargs):
    """
    Everything that needs the actual robot list as real Python objects
    (not a LaunchConfiguration placeholder) lives here, since
    OpaqueFunction is the only place `.perform(context)` can resolve
    launch arguments to concrete strings before we loop over them.
    """
    robots_file = LaunchConfiguration("robots_file").perform(context)
    robots_arg = LaunchConfiguration("robots").perform(context)

    if robots_arg.strip():
        # Explicit inline override takes precedence over the file.
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

    # Node arguments must be strings - robot.yaml's x_pose/y_pose/z_pose
    # parse as YAML floats (e.g. 1.5), not strings, so normalize here.
    for robot in robots:
        robot["x_pose"] = str(robot["x_pose"])
        robot["y_pose"] = str(robot["y_pose"])
        robot["z_pose"] = str(robot["z_pose"])
        robot["name"] = str(robot["name"])

    urdf = os.path.join(
        get_package_share_directory("forklift_description"), "urdf", "forklift.urdf"
    )
    sdf = os.path.join(
        get_package_share_directory("forklift_gazebo"),
        "models", "mini_forklift", "forklift_model.sdf",
    )

    
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    actions = []
    last_action = None

    for robot in robots:
        name = robot["name"]
        namespace = "/" + name
        
        

        doc = xacro.parse(open(sdf))
        xacro.process_doc(doc, mappings={"namespace": namespace})
        robot_description = {"robot_description": doc.toxml()}

        forklift_state_publisher = Node(
            package="robot_state_publisher",
            name="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            remappings=remappings,
            output="both",
            parameters=[{"use_sim_time": True, "publish_frequency": 10.0}, robot_description],
            arguments=[urdf],
        )
        

        # NOTE: -robot_namespace removed - not a recognized flag for
        # ros_gz_sim's "create" executable on Fortress (that's a classic-
        # Gazebo spawn_entity.py flag). The namespace is already baked into
        # the spawned SDF via the "namespace" xacro mapping above.
        spawn_forklift = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", f"{namespace}/robot_description",
                '-robot_namespace', namespace,
                "-name", name,
                "-x", robot["x_pose"],
                "-y", robot["y_pose"],
                "-z", robot["z_pose"],
            ],
        )
        

        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=namespace,
            remappings=[
                (f"/world/ware_house/model/{name}/joint_state", "joint_states"),
                # Charger pad contacts (all chargers) -> this robot's own
                # topic; forklift_docking keeps only contacts naming this
                # robot's model, so every robot sees just its own.
                ("/charger_contacts", f"{namespace}/charger_contact"),
            ],
            arguments=[
                f"{namespace}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                f"{namespace}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"{namespace}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"{namespace}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                f"{namespace}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                f"{namespace}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                f"{namespace}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                f"{namespace}/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                # Charger pad sensors (remapped to <namespace>/charger_contact above).
                "/charger_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                # NOTE: "]" not "@" or "[" - this is a command topic (ROS
                # publishes lift position, Gazebo's JointPositionController
                # consumes it), so it needs to be ROS -> GZ only. The
                # previous "[" direction meant commands published from ROS
                # never actually reached the plugin inside Gazebo.
                f"{namespace}/lift_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
                # f"{namespace}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                f"/world/ware_house/model/{name}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            output="screen",
        )

        if last_action is None:
            actions.append(forklift_state_publisher)
            actions.append(spawn_forklift)
            actions.append(bridge)
        else:
            actions.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[spawn_forklift, forklift_state_publisher, bridge],
                    )
                )
            )

        last_action = spawn_forklift

    return actions


def generate_launch_description():
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.path.join(get_package_prefix('forklift_gazebo'), 'share'),
            ":" + os.path.join(get_package_share_directory("forklift_gazebo"), "models"),
        ],
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value=use_sim_time, description="Use simulator time"
    )

    # NEW: fleet now lives in config/robot.yaml by default. Edit that file
    # to add/remove/reposition robots - no launch file or CLI changes needed.
    declare_robots_file = DeclareLaunchArgument(
        name="robots_file",
        default_value=os.path.join(
            get_package_share_directory("forklift_gazebo"), "config", "robots.yaml"
        ),
        description="Path to a YAML file with a top-level 'robots' list (name, x_pose, y_pose, z_pose).",
    )

    # Optional inline override - only used if explicitly passed (takes
    # precedence over robots_file when non-empty). Handy for one-off CLI
    # tests without touching robot.yaml, e.g.:
    #
    #   ros2 launch forklift_gazebo multi_forklift.launch.py \
    #     robots:='[{"name": "forklift1", "x_pose": "0.0", "y_pose": "0.0", "z_pose": "0.01"}]'
    declare_robots = DeclareLaunchArgument(
        name="robots",
        default_value="",
        description="Optional inline JSON/YAML robot list, overrides robots_file when non-empty.",
    )

    world_only = os.path.join(get_package_share_directory("forklift_gazebo"), "worlds", "small_warehouse.sdf")

    # NOTE: changed from 'ign_args' to 'gz_args' - confirm this matches
    # your installed ros_gz_sim's gz_sim.launch.py via:
    #   ros2 launch ros_gz_sim gz_sim.launch.py --show-args
    # 'ign_args' being silently ignored would mean Gazebo loads its default
    # empty world instead of small_warehouse.sdf.
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")]
        ),
        launch_arguments=[("gz_args", " -r -v4 " + world_only)],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # World-level view of every charger_station's pad (monitoring
            # only). Each robot's bridge also forwards it, remapped to
            # <namespace>/charger_contact.
            "/charger_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_robots_file,
            declare_robots,
            ign_resource_path,
            ign_gz,
            clock_bridge,
            OpaqueFunction(function=launch_setup),
        ]
    )