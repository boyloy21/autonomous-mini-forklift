# forklift_gazebo

ROS 2 (Humble) `ament_cmake` package that simulates the Mini Forklift AMR in
**Gazebo Fortress (Ignition)**: world files, robot/prop SDF models, spawning
launch files (single and multi‑robot), and the `ros_gz_bridge` configuration
that connects Ignition transport to ROS 2 topics.

---

## 1. Overview and Responsibility

`forklift_gazebo` owns everything needed to bring the robot model defined in
`forklift_description` into a simulated warehouse: the world SDF, prop
models (pallets, shelving, a charging station, ArUco boxes), the
`GZ_SIM_RESOURCE_PATH`/`IGN_GAZEBO_RESOURCE_PATH` wiring so Gazebo can find
those models and meshes, entity spawning via `ros_gz_sim`'s `create`, and the
`ros_gz_bridge` topic/type mapping between Ignition transport and ROS 2.

It does **not** define the robot's links/joints/sensors (that is
`forklift_description`) and does not run navigation or control logic — it is
purely the simulation/spawn/bridge layer.

## 2. Architecture / Role in the Overall System

```
                    +---------------------------+
                    |  Gazebo Fortress (gz sim)  |
                    |  worlds/small_warehouse.sdf|
                    +-------------+---------------+
                                  |
              spawns entity from /robot_description (xacro-processed
              forklift_description URDF, or a models/*.sdf prop)
                                  |
                    +-------------v---------------+
                    |   forklift_gazebo             |
                    |  - launch/*.launch.py         |
                    |  - config/bridge_config.yaml  |
                    |  - config/robots.yaml         |
                    |  - config/forklift_controllers.yaml |
                    +-------------+---------------+
                                  | ros_gz_bridge (Ignition <-> ROS 2 topics)
                    +-------------v---------------+
                    |  /cmd_vel, /odom, /scan, /imu,|
                    |  /camera/*, /joint_states,   |
                    |  /lift_cmd, /tf              |
                    +-------------------------------+
                                  |
        consumed by forklift_navigation, forklift_controller,
        forklift_aruco, forklift_localization, forklift_remote
```

## 3. Directory Structure

```
forklift_gazebo/
├── config/
│   ├── bridge_config.yaml          # ros_gz_bridge topic/type map (single robot)
│   ├── forklift_controllers.yaml   # ros2_control controller_manager config (diff drive + lift)
│   └── robots.yaml                 # fleet definition for multi-robot spawning
├── launch/
│   ├── forklift_ign_gazebo.launch.py   # single-robot: world + spawn + bridge + controllers
│   ├── multiple_robot.launch.py        # multi-robot: hardcoded 4-robot fleet, per-robot bridge
│   └── multiple_robot_world.launch.py  # multi-robot: fleet driven by config/robots.yaml
├── models/
│   ├── mini_forklift/               # robot SDF variants (model.sdf, forklift_model.sdf, forklift_model1.sdf)
│   ├── charger_station/             # charging dock prop
│   ├── aruco_box/, box_card/, mini_pallet/, pallet_box_mobile*/  # ArUco-marked pallets/props
│   └── aws_robomaker_warehouse_*/   # AWS RoboMaker warehouse shelving/props (imported assets)
├── worlds/
│   ├── empty.sdf
│   └── small_warehouse.sdf
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

This package defines no custom C++/Python nodes; it launches stock nodes
from `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, and
`controller_manager`:

| Node (executable) | Package | Responsibility |
|---|---|---|
| `gz_sim.launch.py` (included) | `ros_gz_sim` | Starts the Gazebo Fortress server + GUI with the chosen world SDF |
| `create` | `ros_gz_sim` | Spawns the robot entity from the `robot_description` topic or an SDF file, at a given pose |
| `parameter_bridge` | `ros_gz_bridge` | Bridges topics between Ignition transport and ROS 2, per `bridge_config.yaml` (single robot) or inline argument lists (multi‑robot launches) |
| `robot_state_publisher` | `robot_state_publisher` | Publishes TF from the xacro‑processed `forklift_description` model |
| `spawner` (joint_state_broadcaster / diff_drive_controller / lift_position_controller) | `controller_manager` | Activates `ros2_control` controllers defined in `forklift_controllers.yaml` (only used on the `ros2_control` code path, currently commented out in the active single‑robot launch) |
| `static_transform_publisher` | `tf2_ros` | Publishes a fixed camera TF correction (currently commented out in the active launch files) |

## 5. Input / Output Topics

`bridge_config.yaml` (single‑robot, unnamespaced):

| Name | Type | Direction | Description |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | IGN → ROS | Simulation clock |
| `/imu` | `sensor_msgs/Imu` | IGN → ROS | IMU sensor |
| `/odom` | `nav_msgs/Odometry` | IGN → ROS | Ground-truth/DiffDrive odometry |
| `/lift_cmd` | `std_msgs/Float64` | Bidirectional | Fork lift position command/feedback |
| `/joint_states` | `sensor_msgs/JointState` | IGN → ROS | Wheel + lift joint states |
| `/cmd_vel` | `geometry_msgs/Twist` | Bidirectional | Velocity command to the DiffDrive plugin |
| `/tf` | `tf2_msgs/TFMessage` | IGN → ROS | Dynamic transforms published by Gazebo plugins |
| `/scan`, `/scan/points` | `sensor_msgs/LaserScan`, `PointCloud2` | IGN → ROS | LiDAR |
| `/camera/camera_info`, `/camera/image_raw`, `/camera/depth_image`, `/camera/points` | `sensor_msgs/*` | IGN → ROS | RGB‑D camera |
| `/battery_charger_station/touched` | `std_msgs/Bool` | IGN → ROS | Charging-station contact sensor signal |

Multi‑robot launches (`multiple_robot.launch.py`, `multiple_robot_world.launch.py`)
build an equivalent per‑namespace list at runtime
(`/<robot_name>/cmd_vel`, `/<robot_name>/odom`, `/<robot_name>/scan`,
`/<robot_name>/imu`, `/<robot_name>/tf`, `/<robot_name>/joint_states`, and in
`multiple_robot_world.launch.py` also `/<robot_name>/camera/*` and
`/<robot_name>/lift_cmd`).

## 6. Services and Actions

None. This package performs no service/action processing; all communication
is topic-based (spawning is a one-shot process invocation of `ros_gz_sim create`,
not a ROS service call).

## 7. Important Parameters

| Parameter/Argument | Default | Used in | Description |
|---|---|---|---|
| `position_x` / `position_y` / `orientation_yaw` | `0.0` | `forklift_ign_gazebo.launch.py` | Single-robot spawn pose |
| `use_sim_time` | `true` | multi-robot launches | Propagated to all spawned nodes |
| `robots_file` | `config/robots.yaml` | `multiple_robot_world.launch.py` | Path to the fleet definition file |
| `robots` | `""` (empty) | `multiple_robot_world.launch.py` | Optional inline YAML/JSON fleet override, takes precedence over `robots_file` |
| `wheel_separation`, `wheel_radius` | `0.3` m, `0.04` m | `forklift_controllers.yaml` | Must match `forklift_description`'s `const.xacro` values |
| `update_rate` | `50` Hz | `forklift_controllers.yaml` | `controller_manager` control loop rate |

`robots.yaml` fleet entries (`name`, `x_pose`, `y_pose`, `z_pose`) currently
define one active robot (`forklift1`); four more are present but commented
out, ready to enable for multi-robot testing.

## 8. Launch Files and Run Commands

| Launch file | Robots | Fleet source | Notes |
|---|---|---|---|
| `forklift_ign_gazebo.launch.py` | 1 (unnamespaced) | — | Active single-robot launch: world, `robot_state_publisher` (xacro `GAZEBO:=true`), spawn, `ros_gz_bridge` via `bridge_config.yaml`. Controller spawners and the camera static TF are present but commented out. |
| `multiple_robot.launch.py` | 4 (hardcoded: `forklift1`‑`forklift4`) | inline Python list | Spawns from `models/mini_forklift/model.sdf`, sequential spawn via `OnProcessExit`. Explicitly documents (in its own header comment) that the referenced `model.sdf` has no `<plugin>` tags yet, so per-robot topic isolation is **not guaranteed** without further xacro/namespace work. |
| `multiple_robot_world.launch.py` | N (data-driven) | `config/robots.yaml` (or inline `robots` arg) | Spawns from `models/mini_forklift/forklift_model.sdf`, builds a per-robot `ros_gz_bridge` including camera and `lift_cmd` topics, sequential spawn via `OnProcessExit`. |

```bash
# Single-robot simulation
ros2 launch forklift_gazebo forklift_ign_gazebo.launch.py

# Single-robot simulation at a custom pose
ros2 launch forklift_gazebo forklift_ign_gazebo.launch.py position_x:=1.0 position_y:=0.5 orientation_yaw:=1.57

# Multi-robot simulation, fleet from config/robots.yaml
ros2 launch forklift_gazebo multiple_robot_world.launch.py

# Multi-robot simulation, inline fleet override
ros2 launch forklift_gazebo multiple_robot_world.launch.py \
  robots:='[{"name": "forklift1", "x_pose": "0.0", "y_pose": "0.0", "z_pose": "0.01"}]'

# Multi-robot simulation, hardcoded 4-robot fleet (needs model.sdf plugin work — see note above)
ros2 launch forklift_gazebo multiple_robot.launch.py
```

## 9. Dependencies

From `package.xml` (`ament_cmake` build type):

- `ros_gz_bridge`, `ros_gz_sim`, `ros_ign_bridge`, `ros_ign_gazebo`
- `controller_manager`, `ros2_control`, `ros2_controllers`, `ros2controlcli`
- `ign_ros2_control`, `gazebo_ros`, `gazebo_ros2_control`, `gazebo_plugins`
- `robot_state_publisher`, `xacro`
- `forklift_description`, `forklift_localization` (build + exec depend)
- `launch`, `launch_ros`
- Build-only: `control_msgs`, `geometry_msgs`, `rclcpp`

## 10. Communication Flow

1. `ros_gz_sim`'s `gz_sim.launch.py` starts the Gazebo Fortress server/GUI
   with the selected world SDF (`worlds/small_warehouse.sdf` by default).
2. `IGN_GAZEBO_RESOURCE_PATH`/`GZ_SIM_RESOURCE_PATH` are set so Ignition can
   resolve `model://` URIs used by `models/*` and the meshes referenced from
   `forklift_description`.
3. `forklift_description`'s `robot.xacro` is processed with `GAZEBO:=true`
   and published as `robot_description`; `ros_gz_sim create` spawns an
   entity from that topic (single robot) or from a fixed `model.sdf`/`forklift_model.sdf`
   file per fleet entry (multi-robot).
4. `ros_gz_bridge`'s `parameter_bridge` translates each Gazebo Ignition
   transport topic into the corresponding ROS 2 topic (and vice-versa for
   `/cmd_vel` and `/lift_cmd`), per `bridge_config.yaml` or the multi-robot
   launch files' inline argument lists.
5. Downstream ROS 2 nodes (Nav2, `forklift_controller`, `forklift_aruco`,
   teleop) interact with the simulated robot exclusively through these
   bridged topics — none of them talk to Ignition transport directly.

## 11. Integration with Other Packages

- **forklift_description** — supplies the URDF/Xacro model this package
  spawns; `robots.yaml`'s dimensional assumptions (`wheel_separation`,
  `wheel_radius` in `forklift_controllers.yaml`) must stay in sync with
  `const.xacro`.
- **forklift_localization** — declared as an `exec_depend`; its
  `ekf_gazebo.launch.py` include is present but commented out in
  `forklift_ign_gazebo.launch.py`.
- **forklift_navigation** — consumes `/scan`, `/odom`, `/tf` bridged here for
  SLAM/AMCL/Nav2.
- **forklift_controller** — sends `/lift_cmd` (bridged bidirectionally) to
  the `JointPositionController` Gazebo plugin defined in
  `forklift_description/urdf/gazebo.xacro`.
- **forklift_aruco** — consumes `/camera/image_raw`, `/camera/camera_info`,
  `/camera/depth_image` bridged here.
- **forklift_remote** — publishes `/cmd_vel`, consumed by the DiffDrive
  Gazebo plugin via this package's bridge.

## 12. Future Improvements

- **Planned / Future:** add the missing `<plugin>` tags (DiffDrive, sensors)
  to `models/mini_forklift/model.sdf`, or fully retire it in favor of the
  xacro-processed `forklift_description` model, per the explicit warning in
  `multiple_robot.launch.py`'s header comment.
- **Planned / Future:** namespace-safe internal Gazebo plugin topics for true
  multi-robot isolation (currently only the bridge layer is namespaced in
  `multiple_robot.launch.py`; the underlying Gazebo plugin topics may still
  collide unless the spawned SDF itself is namespace-aware).
- Re-enable and validate the commented-out `ros2_control` controller
  spawners and EKF include in `forklift_ign_gazebo.launch.py`, or remove them
  if the DiffDrive/JointPositionController Gazebo plugins are the
  permanent control path.
- Reconcile the three `mini_forklift` SDF variants (`model.sdf`,
  `forklift_model.sdf`, `forklift_model1.sdf`) into a single maintained
  model to avoid drift between single- and multi-robot launch files.
