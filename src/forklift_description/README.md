# forklift_description

ROS 2 (Humble) `ament_cmake` package containing the URDF/Xacro robot model,
meshes, RViz configuration, and viewing launch file for the Mini Forklift
AMR. This package defines *what the robot is* (links, joints, sensors,
inertia, TF tree) — it does not run any ROS nodes of its own beyond
`robot_state_publisher`, launched for visualization purposes only.

---

## 1. Overview and Responsibility

`forklift_description` is the single source of truth for the robot's
physical structure: base chassis, differential-drive wheels, caster, a
prismatic fork/lift mast, an IMU, a 2D LiDAR, and a front-facing RGB‑D
camera. It is consumed by every other package that needs to know the
robot's kinematics, TF frames, or sensor mounting (`forklift_gazebo`,
`forklift_navigation`, `forklift_controller`, `forklift_aruco`,
`forklift_bringup`).

The model is authored in Xacro and can be processed either for
**simulation** (Gazebo Fortress plugins) or for a **real-hardware /
ros2_control** target, selected by the `GAZEBO` xacro argument.

## 2. Architecture / Role in the Overall System

```
forklift_description (URDF/Xacro, meshes, TF frames)
        │
        ├── forklift_gazebo      -- spawns this model's SDF/robot_description into Gazebo Fortress
        ├── forklift_bringup     -- loads robot_description for single/multi-robot launches
        ├── forklift_navigation  -- consumes TF (base_link, lidar frame) for SLAM/AMCL/Nav2
        ├── forklift_controller  -- drives the lift_joint defined here
        └── forklift_aruco       -- uses the camera_link frame + camera topics published from
                                    the sensor defined here
```

`forklift_description` has no runtime dependency on any of these packages
(its `forklift_gazebo` CMake/package dependency is explicitly commented out)
— it is a pure model provider.

## 3. Directory Structure

```
forklift_description/
├── urdf/
│   ├── robot.xacro            # top-level assembly: includes all macros, defines links/joints
│   ├── robot.urdf              # pre-expanded/reference URDF (not the live source of truth)
│   ├── forklift.urdf           # additional/legacy exported URDF
│   ├── const.xacro             # all dimensional/mass/property constants
│   ├── material.xacro          # named color materials (silver, red, yellow, ...)
│   ├── inertia.xacro           # box/sphere/cylinder inertia macros
│   ├── wheel.xacro             # wheel link + continuous joint macro
│   ├── lift.xacro              # fork/lift prismatic joint + link macro
│   ├── transmission.xacro      # ros2_control <transmission> macros (legacy/hardware path)
│   ├── ros2_control.xacro      # <ros2_control> hardware description (non-Gazebo / real hardware)
│   └── gazebo.xacro            # Gazebo Fortress (Ignition) sensor + plugin macros
├── meshes/                     # STL/DAE visual & collision meshes (base, wheel, fork, lidar, camera)
├── rviz/
│   └── view_robot.rviz         # RViz configuration for view_robot.launch.py
├── launch/
│   └── view_robot.launch.py    # standalone model viewer (robot_state_publisher + joint_state_publisher_gui + RViz)
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

This package defines no custom nodes. `view_robot.launch.py` starts only
stock ROS 2 nodes for visualization:

| Node | Package | Responsibility |
|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | Publishes TF from the processed `robot.xacro` (`robot_description` parameter) and `/joint_states` |
| `joint_state_publisher_gui` | `joint_state_publisher_gui` | GUI sliders to manually move each non-fixed joint (wheels, `lift_joint`) for visual inspection |
| `rviz2` | `rviz2` | Displays the model using `rviz/view_robot.rviz` |

## 5. Links, Joints, and TF Tree

```
base_footprint
└── base_link                (fixed, +wheel_radius Z offset)
    ├── left_wheel_link       (continuous joint, axis Y)
    ├── right_wheel_link      (continuous joint, axis Y)
    ├── front_caster          (fixed, sphere)
    ├── imu_link              (fixed)
    ├── lidar_link            (fixed)
    └── lift_link             (prismatic joint, axis Z — the fork/mast)
        └── camera_link       (fixed, mounted on the lift/fork, not on base_link)
```

| Joint | Type | Parent → Child | Axis / Limits |
|---|---|---|---|
| `base_joint` | fixed | `base_footprint → base_link` | Z offset = `wheel_radius` (0.040 m) |
| `left_wheel_joint` / `right_wheel_joint` | continuous | `base_link → {left,right}_wheel_link` | axis Y, `wheel_separation` = 0.3 m, `wheel_radius` = 0.040 m |
| `caster_joint` | fixed | `base_link → front_caster` | sphere, radius 0.03 m |
| `imu_joint` | fixed | `base_link → imu_link` | Z = `base_height/2` |
| `lidar_joint` | fixed | `base_link → lidar_link` | near top of chassis |
| `lift_joint` | prismatic | `base_link → lift_link` | axis Z, range `[0.0, 0.15]` m, effort 500, velocity 0.10 m/s |
| `camera_joint` | fixed | `lift_link → camera_link` | mounted on the fork carriage (moves with the lift) |

**Robot dimensions** (from `const.xacro`):

| Property | Value |
|---|---|
| Base footprint | 0.245 m (L) × 0.2426 m (W) × 0.18 m (H) |
| Wheel radius / width | 0.040 m / 0.025 m |
| Wheel separation | 0.30 m |
| Caster radius | 0.03 m |
| Lift (fork) travel | 0.0 – 0.15 m |
| Fork geometry | 0.30 m (L) × 0.2426 m (W) × 0.03 m (H) |
| Camera housing | 0.022 × 0.022 × 0.022 m |
| LiDAR (YDLIDAR X2) | radius 0.045 m, length 0.045 m |

## 6. Sensors, Collision & Inertia

| Sensor | Link/Frame | Gazebo plugin (in `gazebo.xacro`) | Notes |
|---|---|---|---|
| IMU | `imu_link` | `ignition::gazebo::systems::Imu` sensor block (`imu` type), topic `${prefix}/imu` | Gaussian noise on angular velocity + linear acceleration |
| 2D LiDAR (YDLIDAR X2) | `lidar_link` (`lidar_frame_id` const) | `gpu_lidar` sensor, topic `${prefix}/scan` | 360 samples, 0.2–7 m range, full 360° FOV |
| RGB‑D camera | `camera_link` (`camera_frame_id` const) | `rgbd_camera` sensor, topic `${prefix}/camera`, `camera_info` on `${prefix}/camera_info` | 640×480, 1.08 rad HFOV, 0.05–5 m clip |

Every link with a visual mesh also defines a simplified **collision**
geometry (boxes/cylinders/spheres rather than the full mesh, for
performance) and an explicit **inertial** block, using `inertia.xacro`'s
`box_inertia` / `sphere_inertia` macros where masses are declared in
`const.xacro` (e.g. `mass_base = 3.0 kg`, `mass_wheel = 0.1 kg`,
`mass_lidar = 0.150 kg`, `mass_camera = 0.050 kg`).

**Note:** inline comments in `gazebo.xacro` reference link names
`front_camera_mount` and `ydlidar_lidar_mount`, but the actual links defined
in `robot.xacro` are `camera_link` and `lidar_link` (matching
`const.xacro`'s `camera_frame_id`/`lidar_frame_id` properties, which the
`<gazebo reference="...">` blocks correctly use). These comments are stale
from an earlier refactor and do not reflect a real naming mismatch.

## 7. Input / Output Topics

This package defines no ROS nodes that publish/subscribe; topics listed
below are produced by the **Gazebo Fortress sensor plugins** embedded in the
URDF once the model is spawned (see `forklift_gazebo`):

| Name | Type | Direction | Description |
|---|---|---|---|
| `${prefix}/imu` | `sensor_msgs/Imu` | Output (sim) | IMU sensor plugin |
| `${prefix}/scan` | `sensor_msgs/LaserScan` | Output (sim) | LiDAR sensor plugin |
| `${prefix}/camera` | `sensor_msgs/Image` | Output (sim) | RGB‑D camera color stream |
| `${prefix}/camera_info` | `sensor_msgs/CameraInfo` | Output (sim) | Camera intrinsics |
| `${prefix}/joint_states` | `sensor_msgs/JointState` | Output (sim) | `JointStatePublisher` plugin (wheels + lift) |
| `${prefix}/cmd_vel` | `geometry_msgs/Twist` | Input (sim) | `DiffDrive` plugin command |
| `${prefix}/lift_cmd` | *(controller command)* | Input (sim) | `JointPositionController` plugin for `lift_joint` |
| `robot_description` | `std_msgs/String` (parameter) | Output | Processed URDF, published as a parameter by `robot_state_publisher` |

`${prefix}` is empty for a single, unnamespaced robot, or the robot's
namespace (e.g. `forklift1`) in multi-robot setups (see `forklift_gazebo`).

## 8. Services and Actions

None. This is a static description package; it exposes no ROS services or
actions.

## 9. Important Parameters / Xacro Arguments

| Arg/Property | Default | Description |
|---|---|---|
| `GAZEBO` (xacro arg) | `true` | Selects `gazebo.xacro` (sim plugins) vs. `ros2_control.xacro` (hardware interface) |
| `wheel_odom_topic` (xacro arg) | `odom` | Odometry topic name used by the encoder-based odometry source |
| `odometry_source` (xacro arg) | `world` | `world` = ground-truth `OdometryPublisher` plugin; `encoders` = DiffDrive plugin's own odometry |
| `robot_name` (xacro arg) | `""` | Robot namespace prefix, propagated into `gz_plugins` as `prefix` |
| `robot_type` (launch arg, `view_robot.launch.py`) | `forklift` | Passed into `xacro.process_file` mappings |

## 10. Launch Files and Run Commands

| Launch file | Purpose |
|---|---|
| `view_robot.launch.py` | Standalone model viewer: processes `robot.xacro`, starts `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 with `view_robot.rviz`. No Gazebo involved. |

```bash
# Inspect the model in RViz with manual joint sliders
ros2 launch forklift_description view_robot.launch.py

# Override robot type argument
ros2 launch forklift_description view_robot.launch.py robot_type:=forklift
```

Actual simulation spawning (Gazebo Fortress) is handled by
`forklift_gazebo`/`forklift_bringup`, which process this package's
`robot.xacro` with `GAZEBO:=true`.

## 11. Dependencies

From `package.xml` (`ament_cmake` build type):

- `xacro`, `robot_state_publisher`, `urdf`
- `controller_manager`, `ros2_control`, `ros2_controllers`, `hardware_interface`, `ros2controlcli`
- `ign_ros2_control`, `ros_gz_bridge`, `ros_gz_sim`, `ros_ign_bridge`, `ros_ign_gazebo`
- `imu_sensor_broadcaster`, `joint_state_broadcaster`, `diff_drive_controller`
- `joint_state_publisher`, `joint_state_publisher_gui`, `rviz`
- `robot_localization`
- `launch`, `launch_ros`
- `control_msgs`, `rclcpp`, `rclcpp_action`, `std_msgs`, `geometry_msgs` (CMakeLists `find_package` only)

The `forklift_gazebo` dependency is present in comments only
(`<!-- <exec_depend>forklift_gazebo</exec_depend> -->`), confirming this
package intentionally has no build/runtime dependency on the simulation
package that consumes it.

## 12. Communication Flow

1. A launch file (here, or in `forklift_gazebo`/`forklift_bringup`)
   processes `urdf/robot.xacro` via `xacro.process_file(...)`, producing a
   flattened URDF/SDF-compatible XML string.
2. That string is passed as the `robot_description` parameter to
   `robot_state_publisher`, which publishes the static and dynamic TF tree
   from `/joint_states`.
3. If `GAZEBO:=true`, the embedded `<gazebo>` blocks (from `gazebo.xacro`)
   are interpreted by Ignition Gazebo when the model is spawned, activating
   the DiffDrive, JointPositionController, IMU, LiDAR, and camera plugins.
4. If `GAZEBO:=false`, `ros2_control.xacro`'s `<ros2_control>` block is used
   instead, exposing `left_wheel_joint`/`right_wheel_joint` (velocity) and
   `lift_joint` (position) as `ros2_control` command/state interfaces for a
   real hardware interface plugin (`ign_ros2_control/IgnitionSystem` is
   configured, implying this path is still simulation-oriented; a genuine
   hardware plugin is **Planned / Future**).

## 13. Integration with Other Packages

- **forklift_gazebo** — spawns this URDF (with `GAZEBO:=true`) into worlds,
  applies `robots.yaml` namespacing, and bridges the sensor/actuator topics
  declared here via `ros_gz_bridge`.
- **forklift_bringup** — loads `robot_description` for single- and
  multi-robot bringup launches.
- **forklift_controller** — commands `lift_joint` (via `${prefix}/lift_cmd`
  in simulation) to raise/lower the fork defined in `lift.xacro`.
- **forklift_navigation** — consumes `base_footprint`/`base_link` TF and the
  LiDAR's `${prefix}/scan` topic for SLAM/AMCL/Nav2 costmaps.
- **forklift_aruco** — subscribes to `${prefix}/camera`,
  `${prefix}/camera_info` published from the `camera_link` sensor defined
  here, and uses the camera's mount offset for its camera→base_link
  transform.

## 14. Future Improvements

- Resolve the stale `front_camera_mount`/`ydlidar_lidar_mount` comments in
  `gazebo.xacro` to match the actual link names, to avoid confusing future
  contributors.
- **Planned / Future:** a genuine hardware `ros2_control` plugin (currently
  `ros2_control.xacro` still targets `ign_ros2_control/IgnitionSystem`, not
  real motor/encoder hardware).
- **Planned / Future:** split the fork/mast into a fixed mast + prismatic
  fork carriage, as already noted in `lift.xacro`'s own comments, for a more
  physically accurate lift mechanism.
- Consolidate `robot.urdf` / `forklift.urdf` (pre-expanded exports) or
  remove them if `robot.xacro` is the sole authoritative source, to prevent
  drift between the xacro source and its exported copies.
