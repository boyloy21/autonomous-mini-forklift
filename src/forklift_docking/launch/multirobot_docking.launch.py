"""
multiple_docking.launch.py

Starts, PER ROBOT (called once per robot with a distinct `namespace`
argument):
  1. docking_server            (opennav_docking executable, our config)
  2. lifecycle_manager_docking (brings docking_server through
                                 configure -> activate automatically)

FIX (this version): the detected_dock_pose remap target is now built
EXPLICITLY as an absolute per-robot string (e.g. "/forklift1/aruco_dock_pose")
instead of relying on a relative target ("aruco_dock_pose") implicitly
picking up the node's own namespace. Both SHOULD produce the same result
under normal ROS2 relative-name resolution -- but given how many subtle
namespace-resolution surprises this project has hit (the root_key vs.
literal-substitution mismatch, absolute-vs-relative parameter values,
etc.), making this one explicit removes any doubt rather than depending
on implicit behavior.

Earlier fix (still in place): docking_server_multirobot.yaml's
"namespace" placeholder (embedded inside string VALUES like
"namespace/odom") needs the SAME find/replace helper already proven
working for nav2_params.yaml and aruco_parameters.yaml
(replace_namespace_in_params), chained BEFORE RewrittenYaml runs --
root_key alone does NOT do this substitution, it only restructures the
yaml's key nesting for node-name matching.

Also still in place: the /tf, /tf_static remap on docking_server_node --
tf2_ros::Buffer subscribes to these using hardcoded absolute names
internally, Node(namespace=...) alone does not redirect them.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


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


def replace_namespace_in_params(yaml_file_path, namespace_value):
    """Same proven mechanism as forklift_navigation's multiple_bringup_launch.py
    -- literal find/replace of the "namespace" token in every string value,
    writes to a distinct /tmp path (source basename included, avoids
    colliding with any other file substituted for the same robot)."""
    ns = namespace_value if namespace_value.startswith('/') else '/' + namespace_value

    with open(yaml_file_path, 'r') as f:
        data = yaml.safe_load(f)

    data = replace_namespace(data, 'namespace', ns)

    source_name, source_ext = os.path.splitext(os.path.basename(yaml_file_path))
    out_path = f"/tmp{ns}_{source_name}{source_ext}"
    with open(out_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    return out_path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("forklift_docking")
    dock_database_yaml = os.path.join(pkg_share, "config", "dock_database.yaml")

    namespace_value = LaunchConfiguration("namespace").perform(context)
    raw_params_file = LaunchConfiguration("params_file").perform(context)

    # Step 1: literal "namespace" token substitution in the params yaml.
    ns_substituted_params_file = replace_namespace_in_params(raw_params_file, namespace_value)

    # Step 2: existing use_sim_time/autostart substitution, chained on
    # top of the already-namespace-substituted file. root_key kept as-is
    # for whatever node-name-matching role it plays.
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'autostart': LaunchConfiguration('autostart'),
    }
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=ns_substituted_params_file,
            root_key=namespace_value,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Explicit, unambiguous per-robot remap target -- e.g.
    # "/forklift1/aruco_dock_pose", "/forklift2/aruco_dock_pose". Built
    # directly from the resolved namespace string rather than relying on
    # a relative target implicitly picking up the node's namespace.
    ns_clean = namespace_value if namespace_value.startswith('/') else '/' + namespace_value
    aruco_dock_pose_topic = f"{ns_clean}/aruco_dock_pose"

    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        namespace=namespace_value,
        output="screen",
        emulate_tty=True,
        parameters=[
            configured_params,
            {
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "dock_database": dock_database_yaml,
            },
        ],
        remappings=[
            # Explicit absolute target now, e.g.
            # "/forklift1/aruco_dock_pose" -- matches this robot's own
            # aruco_node output topic exactly, no ambiguity.
            ("detected_dock_pose", aruco_dock_pose_topic),
            # docking_server does TF lookups (map, odom, base_footprint)
            # via tf2_ros::Buffer, which subscribes to /tf and /tf_static
            # using hardcoded absolute names internally.
            # Node(namespace=...) alone does NOT redirect these -- same
            # remap every other TF-consuming node in this project needed.
            ("/tf", f"{ns_clean}/tf"),
            ("/tf_static", f"{ns_clean}/tf_static"),
        ],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_docking",
        namespace=namespace_value,
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "autostart": LaunchConfiguration('autostart'),
                "node_names": ["docking_server"],
            }
        ],
    )

    return [docking_server_node, lifecycle_manager_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("forklift_docking")
    docking_server_yaml = os.path.join(pkg_share, "config", "docking_server_multirobot.yaml")

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace -- e.g. "forklift1". Empty means unnamespaced '
                    '(single-robot use).'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo /clock instead of wall time",
    )

    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=docking_server_yaml,
        description="Full path to the multi-robot docking_server params template, with the "
                    "\"namespace\" placeholder embedded in its topic-override fields",
    )

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically bring docking_server to the active lifecycle state",
    )

    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_sim_time_arg,
            declare_params_file_arg,
            declare_autostart_arg,
            # OpaqueFunction because the substitution step needs the
            # actual resolved namespace/params_file string values, not
            # LaunchConfiguration placeholders.
            OpaqueFunction(function=launch_setup),
        ]
    )