# forklift_navigation

ROS 2 (Humble) `ament_cmake` package providing the full autonomous navigation
stack for the Mini Forklift AMR: SLAM (`slam_toolbox`), localization
(`nav2_amcl`), and path planning/following (Nav2, including its
`route_server` graph-based routing extension), for both single- and
multi-robot deployments in Gazebo Fortress.

---

## 1. Overview and Responsibility

`forklift_navigation` owns:

- **Mapping**: `slam_toolbox` online-async SLAM to build the warehouse
  occupancy map.
- **Localization**: `nav2_amcl` particle-filter localization against a
  saved map.
- **Path planning & following**: the full Nav2 stack (controller server,
  planner server, behavior server, BT navigator, waypoint follower,
  velocity smoother, collision monitor) plus **`nav2_route`'s
  `route_server`**, which plans over a predefined road-graph
  (`config/routes/*.geojson`) instead of (or blended with) freespace
  planning.
- **Controller selection**: three interchangeable local controllers —
  Regulated Pure Pursuit (with a rotation-shim), MPPI, and DWB — selectable
  per-launch via a `controller_type` argument.
- **Multi-robot navigation**: namespaced bringup of the entire stack per
  robot, staggered to avoid simultaneous activation.

It does not implement docking, task orchestration, or lift control — those
are `forklift_docking`, `forklift_task_manager`, and `forklift_controller`
respectively, which this package's Behavior Tree and route infrastructure
are designed to interoperate with.

## 2. Architecture / Role in the Overall System

```
                     +-------------------+
                     |  forklift_gazebo   |  (scan, odom, tf)
                     +---------+---------+
                               │
              +----------------+----------------+
              │                                  │
     Mapping mode                       Localization + Navigation mode
   (mapping_launch.py)                 (forklift_navigation.launch.py /
              │                          forklift_navigationV2_launch.py /
     slam_toolbox (online_async)          bringup_launch.py)
              │                                  │
              ▼                       +----------+-----------+
        map/*.yaml, *.pgm             │                      │
                                map_server + amcl      Nav2 stack (controller_server,
                                (localization_launch.py) planner_server, behavior_server,
                                       │                  bt_navigator, waypoint_follower,
                                       │                  velocity_smoother, collision_monitor,
                                       │                  route_server)
                                       └──────────┬───────┘
                                                  │ cmd_vel_nav -> velocity_smoother -> cmd_vel
                                                  ▼
                                         forklift_gazebo / forklift_remote's
                                              remapper.py -> forklift/cmd_vel
                                                  │
                                    forklift_docking / forklift_task_manager
                                    (send NavigateToPose / ComputeRoute+FollowPath
                                     goals to this stack)
```

## 3. Directory Structure

```
forklift_navigation/
├── config/
│   ├── amcl_params.yaml                     # standalone AMCL params (used by forklift_navigation.launch.py)
│   ├── nav2_params.yaml                     # full single-robot Nav2 param set (route_server included)
│   ├── nav2_multirobot_params.yaml          # namespace-templated Nav2 params for multi-robot bringup
│   ├── nav2_pure_pursuit_controller.yaml    # standalone params file for forklift_navigation.launch.py
│   ├── controller_pure_pursuit.yaml         # controller_server-only: RegulatedPurePursuitController
│   ├── controller_mppi.yaml                 # controller_server-only: MPPIController + costmaps
│   ├── controller_dwb.yaml                  # controller_server-only: DWBLocalPlanner + costmaps
│   ├── map_params_online_async.yaml         # slam_toolbox online-async mapping params
│   ├── behavior_trees/
│   │   ├── navigate_to_pose_w_route_server.xml     # BT: ComputeRoute + last-mile ComputePathToPose + SmoothPath
│   │   └── navigate_to_pose_w_route_server_v2.xml  # refined/updated version of the above
│   └── routes/
│       └── forklift_route*.geojson (V1-V4)  # route_server's road-graph definitions
├── launch/
│   ├── mapping_launch.py             # slam_toolbox + RViz + map_saver_server
│   ├── localization_launch.py        # map_server + amcl + lifecycle_manager (single robot)
│   ├── forklift_navigation.launch.py # legacy/manual: nav2_bringup + standalone amcl/map_server/remapper
│   ├── forklift_navigationV2_launch.py # controller/planner/BT/route_server stack only (controller_type arg)
│   ├── bringup_launch.py             # this package's own bringup: localization + forklift_navigationV2 (namespace-aware)
│   ├── multiple_bringup_launch.py    # multi-robot: per-robot bringup + docking + aruco + task_manager, staggered
│   └── rviz_launch.py                # namespaced RViz launcher
├── map/
│   └── small_warehouse.yaml/.pgm (+edited variants)
├── rviz/                             # RViz configs (single/multi-robot, mapping, route tool)
├── scripts/
│   ├── remapper.py                   # cmd_vel -> forklift/cmd_vel republisher node
│   ├── route_and_follow.py           # CLI test tool: ComputeRoute + FollowPath, bypassing bt_navigator
│   └── route_follow_test.py
├── src/
│   ├── concatenate_paths_bt_node.cpp # custom BT plugin: concatenates route-graph path + last-mile path
│   └── publish_path_bt_node.cpp      # custom BT plugin: publishes a path for visualization/debugging
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

This package launches the standard Nav2/`slam_toolbox` nodes (no custom
Nav2 server nodes), plus two small custom Python utilities and two custom
Behavior Tree plugin libraries:

| Node / Plugin | Package | Responsibility |
|---|---|---|
| `slam_toolbox` (online_async) | `slam_toolbox` | Builds the occupancy grid map from `/scan` + odometry during mapping mode |
| `map_server` | `nav2_map_server` | Serves the saved map YAML/PGM |
| `amcl` | `nav2_amcl` | Particle-filter localization, `map -> odom` TF broadcast |
| `controller_server` | `nav2_controller` | Runs the selected local controller (`pure_pursuit`/`mppi`/`dwb`) |
| `smoother_server` | `nav2_smoother` | Path smoothing (`SimpleSmoother`) |
| `planner_server` | `nav2_planner` | Global freespace planning (`NavfnPlanner`), used for the BT's "last mile" segment |
| `behavior_server` | `nav2_behaviors` | Recovery behaviors: spin, backup, drive-on-heading, assisted teleop, wait |
| `bt_navigator` | `nav2_bt_navigator` | Executes the Behavior Tree (default or this package's route-server BT) |
| `waypoint_follower` | `nav2_waypoint_follower` | Sequential waypoint navigation |
| `velocity_smoother` | `nav2_velocity_smoother` | Smooths `cmd_vel_nav` into the final `cmd_vel` |
| `collision_monitor` | `nav2_collision_monitor` | Safety monitoring layer |
| `route_server` | `nav2_route` | Graph-based routing over `config/routes/*.geojson` (`ComputeRoute` action) |
| `lifecycle_manager_navigation` / `lifecycle_manager_localization` | `nav2_lifecycle_manager` | Bring-up/shutdown orchestration for the above server nodes |
| `CmdVelRemapper` | `forklift_navigation` (`remapper.py`) | Republishes `cmd_vel` → `forklift/cmd_vel`, bridging Nav2's standard output topic to this project's robot-specific topic convention |
| `RouteAndFollow` (CLI) | `forklift_navigation` (`route_and_follow.py`) | Test tool: calls `ComputeRoute` then feeds the resulting path directly to `FollowPath`, bypassing `bt_navigator`, to isolate route/controller issues |
| `forklift_concatenate_paths_bt_node` | `forklift_navigation` (BT plugin) | Custom BT action: concatenates the route-graph path with a freespace "last mile" path to the exact goal pose |
| `forklift_publish_path_bt_node` | `forklift_navigation` (BT plugin) | Custom BT action: publishes a path for RViz visualization/debugging |

## 5. Input / Output Topics

| Name | Type | Direction | Description |
|---|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | Input | LiDAR, used by SLAM, AMCL, and costmaps |
| `odom` | `nav_msgs/Odometry` | Input | Wheel/ground-truth odometry |
| `map` | `nav_msgs/OccupancyGrid` | Output (map_server) / Input (costmaps, AMCL) | Static map |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Input | Seeds AMCL's initial pose (published once per robot at multi-robot bringup) |
| `amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output | AMCL's estimated pose |
| `cmd_vel_nav` | `geometry_msgs/Twist` | Internal | Raw controller output before smoothing |
| `cmd_vel` | `geometry_msgs/Twist` | Output | Smoothed velocity command (from `velocity_smoother`) |
| `forklift/cmd_vel` | `geometry_msgs/Twist` | Output (via `remapper.py`) | Final robot-specific drive topic |
| `plan` | `nav_msgs/Path` | Output | Global plan |
| `local_costmap/costmap`, `global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Output | Costmaps |
| `tf` / `tf_static` | `tf2_msgs/TFMessage` | Bidirectional | Namespaced TF (remapped from `/tf`, `/tf_static`) |

## 6. Services and Actions

| Name | Type | Provided by | Description |
|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` | Standard single-goal navigation entry point |
| `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `bt_navigator` | Multi-waypoint navigation |
| `follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | `waypoint_follower` | Waypoint-sequence following |
| `compute_route` | `nav2_msgs/action/ComputeRoute` | `route_server` | Plans a path over the road-graph (`geojson`), by node ID or by pose |
| `follow_path` | `nav2_msgs/action/FollowPath` | `controller_server` | Executes a given path with the selected local controller |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | `planner_server` | Freespace planning, used for the BT's last-mile segment |
| `spin`, `backup`, `drive_on_heading`, `wait`, `assisted_teleop` | `nav2_msgs/action/*` | `behavior_server` | Recovery behaviors |
| `clear_entirely_local_costmap` / `clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | costmap nodes | Costmap reset services |

## 7. Important Parameters

### Controller selection & rotate-in-place-before-forward behavior

`forklift_navigationV2_launch.py` exposes `controller_type` (`pure_pursuit` /
`mppi` / `dwb`), which selects `config/controller_<type>.yaml` as the
controller-only params file. Each controller achieves the "rotate in place
to face the path, then drive forward" behavior differently:

| Controller | Mechanism | Key parameters |
|---|---|---|
| **Pure Pursuit** (`controller_pure_pursuit.yaml`, `nav2_pure_pursuit_controller.yaml`) | `RegulatedPurePursuitController`'s own `use_rotate_to_heading`, or wrapped by `nav2_rotation_shim_controller::RotationShimController` (in `nav2_pure_pursuit_controller.yaml`) which explicitly rotates in place before handing off to the primary controller | `use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785` rad (~45°); shim's `angular_dist_threshold: 0.785`, `rotate_to_heading_angular_vel: 1.8`, `rotate_to_goal_heading: false` |
| **MPPI** (`controller_mppi.yaml`) | No explicit rotation shim — emergent from critic weighting. `GoalAngleCritic` (weight `12.0`) strongly rewards facing the goal; `PreferForwardCritic` (weight `18.0`) discourages *reverse* driving but does **not** penalize pure in-place rotation; `ConstraintCritic` enforces kinematic feasibility for the `DiffDrive` motion model, which has no minimum turning radius, so in-place rotation is always a legal, low-cost trajectory when poorly aligned | `motion_model: "DiffDrive"`, `GoalAngleCritic.cost_weight: 12.0`, `PreferForwardCritic.cost_weight: 18.0`, `ConstraintCritic.cost_weight: 7.0` |
| **DWB** (`controller_dwb.yaml`) | Also purely critic-weighting based (explicitly documented in-file, no `RotationShimController`): `PathAlign`/`PathDist` (scale `48.0` each) are weighted well above `GoalAlign`/`GoalDist` (scale `8.0`) and `RotateToGoal` (scale `24.0`), and `PreferForward` (scale `0.3`) does not penalize low-x/high-theta trajectories, so a stationary in-place turn that faces the path outscores a poorly-aligned forward drive | `PathAlign.scale: 48.0`, `PathDist.scale: 48.0`, `RotateToGoal.scale: 24.0`, `GoalAlign.scale: 8.0` |

### Costmaps (MPPI/DWB configs — representative values)

| Parameter | Local costmap | Global costmap |
|---|---|---|
| `update_frequency` | 5.0 Hz | 5.0 Hz |
| `resolution` | 0.05 m | 0.05 m |
| `inflation_radius` | 0.45 m (tuned down from 0.7 m — see below) | 0.8 m |
| `cost_scaling_factor` | 5.0 | 3.0 |
| `robot footprint` | `[[0.4,0.3],[0.4,-0.3],[-0.3,-0.3],[-0.3,0.3]]` (MPPI) | same |

In-source note: `inflation_radius` was lowered from `0.7` m to `0.45` m
because it exceeded the measured ~0.65 m clearance at a specific warehouse
turn, causing the full-footprint `CostCritic`/behavior chain to see every
forward trajectory as lethal cost there (a "Failed to make progress" /
recovery loop). This is a concrete example of costmap tuning driven by the
physical warehouse layout.

### AMCL (`amcl_params.yaml`)

| Parameter | Value | Description |
|---|---|---|
| `max_particles` / `min_particles` | 2000 / 500 | Particle filter size |
| `alpha1`–`alpha5` | 0.2 each | Odometry motion model noise |
| `update_min_a` / `update_min_d` | 0.05 / 0.05 | Minimum motion before filter update |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | Matches the diff-drive base |
| `laser_max_range` | 7.0 m | Matches the LiDAR's `two_d_lidar_max_range` |

### route_server (`nav2_params.yaml`)

| Parameter | Value | Description |
|---|---|---|
| `base_frame` / `route_frame` | `base_link` / `map` | Reference frames for routing |
| `max_planning_time` | 2.0 s | |
| `path_density` | 0.05 m | Spacing between generated path points |
| `smooth_corners` | false | Corner smoothing at route-graph junctions |

## 8. Launch Files and Run Commands

| Launch file | Purpose |
|---|---|
| `mapping_launch.py` | SLAM: `slam_toolbox` online-async + RViz + `map_saver_server` |
| `localization_launch.py` | `map_server` + `amcl` + lifecycle manager (single robot) |
| `forklift_navigationV2_launch.py` | Controller/planner/behavior/BT/route_server stack only, with `controller_type` selection and composition support |
| `bringup_launch.py` | This package's full single-robot bringup: `localization_launch.py` + `forklift_navigationV2_launch.py`, namespace-aware via `PushRosNamespace` |
| `forklift_navigation.launch.py` | Legacy/manual launch: includes `nav2_bringup`'s stock `bringup_launch.py` directly, plus standalone `amcl`, `map_server`, a static `map->odom` TF, and `remapper.py` |
| `multiple_bringup_launch.py` | Multi-robot: reads `forklift_gazebo/config/robots.yaml`, waits for the last robot's `/odom` topic, then launches a staggered (`TimerAction`, default 6 s apart) per-robot stack of `bringup_launch.py` + `forklift_docking`'s docking launch + RViz + an initial-pose publish + `forklift_aruco` + `forklift_controller`'s multirobot lift node + `forklift_docking`'s docking manager + `forklift_task_manager` |
| `rviz_launch.py` | Namespaced RViz launcher used by both single- and multi-robot bringups |

```bash
# 1. Build a map (mapping mode)
ros2 launch forklift_navigation mapping_launch.py

# 2. Single-robot navigation (this package's own bringup)
ros2 launch forklift_navigation bringup_launch.py controller_type:=mppi

# 3. Single-robot navigation (legacy nav2_bringup-based launch)
ros2 launch forklift_navigation forklift_navigation.launch.py

# 4. Multi-robot navigation (fleet from forklift_gazebo/config/robots.yaml)
#    Run AFTER forklift_gazebo's multiple_robot_world.launch.py:
ros2 launch forklift_navigation multiple_bringup_launch.py

# Test route_server + controller in isolation (no BT/bt_navigator)
ros2 run forklift_navigation route_and_follow.py --goal-x 1.5 --goal-y 4.5
```

## 9. Dependencies

From `package.xml`:

- `navigation2`, `nav2_bringup`, `nav2_simple_commander`, `nav2_util`,
  `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`,
  `nav2_waypoint_follower`, `nav2_lifecycle_manager`, `nav2_map_server`,
  `nav2_amcl`, `nav2_behavior_tree`, `behaviortree_cpp_v3`
- `slam_toolbox`, `robot_localization`
- `rclcpp`, `rclpy`, `tf2_ros`, `tf_transformations`, `topic_tools`
- `geometry_msgs`, `nav_msgs`, `builtin_interfaces`, `rcl_interfaces`, `std_msgs`
- `joint_state_broadcaster`, `joint_state_publisher`
- `launch`, `launch_ros`, `rviz2`, `ament_index_python`

Note: `nav2_route` (providing `route_server`) is used by the launch files
and config but is **not** listed as an explicit `<depend>` in `package.xml`
— it is pulled in transitively via `navigation2`/`nav2_bringup` on this
distro; worth confirming explicitly if the package is ever isolated in a
minimal workspace.

## 10. Communication Flow

1. **Mapping:** `/scan` + `/odom` → `slam_toolbox` (online-async) → occupancy
   grid, saved via `map_saver_server` to `map/*.yaml`/`*.pgm`.
2. **Localization:** `map_server` loads the saved map; `amcl` fuses it with
   `/scan` and `/odom` to broadcast `map -> odom` TF and publish
   `amcl_pose`.
3. **Goal dispatch:** an external client (`forklift_task_manager`,
   `forklift_docking`, or `nav2_simple_commander`/`route_and_follow.py` for
   testing) sends a `NavigateToPose`/`NavigateThroughPoses` goal to
   `bt_navigator`, or a `ComputeRoute` goal directly to `route_server`.
4. **Route-based planning:** the custom BT
   (`navigate_to_pose_w_route_server(_v2).xml`) calls `route_server`'s
   `ComputeRoute` over the `geojson` road-graph, then plans a freespace
   "last mile" from the route's end node to the literal goal via
   `planner_server::ComputePathToPose`, concatenates both paths with the
   custom `forklift_concatenate_paths_bt_node` BT plugin, smooths the result
   (`SimpleSmoother`), and hands it to `controller_server::FollowPath`.
5. **Local control:** the selected controller (`pure_pursuit`/`mppi`/`dwb`)
   produces `cmd_vel_nav`; `velocity_smoother` filters it into `cmd_vel`;
   `remapper.py` republishes it as `forklift/cmd_vel` for the simulated
   robot.
6. **Safety/recovery:** `collision_monitor` and `behavior_server` intervene
   (spin, backup, wait) if progress stalls or a collision risk is detected.

## 11. Integration with Other Packages

- **forklift_gazebo** — source of `/scan`, `/odom`, `/tf`, consumer of
  `forklift/cmd_vel`.
- **forklift_description** — TF frames (`base_link`, `base_footprint`,
  lidar frame) referenced throughout AMCL/costmap/controller configs.
- **forklift_docking** — sends `NavigateToPose`/`ComputeRoute` goals to
  approach a dock before handing off to its own precision-docking behavior;
  `multiple_bringup_launch.py` launches `forklift_docking`'s multi-robot
  docking stack alongside this package's navigation stack.
- **forklift_aruco** — launched alongside navigation in
  `multiple_bringup_launch.py` for vision-based docking after arrival.
- **forklift_controller** — its `lift_controller_node_multirobot` is
  launched per-robot in `multiple_bringup_launch.py`.
- **forklift_task_manager / forklift_mission_manager** — the primary
  clients that send navigation goals as part of pick/drop/charging task
  execution; `forklift_task_manager` is launched per-robot in
  `multiple_bringup_launch.py`.

## 12. Future Improvements

- Declare `nav2_route` as an explicit `package.xml` dependency instead of
  relying on transitive inclusion.
- Consolidate the two single-robot bringup paths
  (`forklift_navigation.launch.py`'s direct `nav2_bringup` include vs. this
  package's own `bringup_launch.py`) into one supported entry point.
- **Planned / Future:** re-enable and validate the commented-out EKF
  (`robot_localization`) include in `bringup_launch.py`/`localization_launch.py`
  for sensor-fused odometry ahead of AMCL.
- **Planned / Future:** replace the fixed 6-second-per-robot stagger in
  `multiple_bringup_launch.py` with an explicit readiness check (e.g.
  waiting for each robot's `amcl_pose` before starting the next), rather
  than a time-based heuristic.
- Continue reconciling `navigate_to_pose_w_route_server.xml` vs. `_v2.xml`
  into a single maintained Behavior Tree once the `_v2` refinements are
  fully validated.
