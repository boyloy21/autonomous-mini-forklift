<p align="center">
  <img src="docs/images/ai_farm_robotics_logo.png" alt="AI FARM ROBOTICS FACTORY logo" width="180"/>
</p>

<h3 align="center">AI FARM ROBOTICS FACTORY</h3>

# Mini Forklift AMR — ROS 2 Nav2 Workspace

> A project by **AI FARM ROBOTICS FACTORY**.

An autonomous mini forklift for warehouse pallet handling, built on **ROS 2 Humble**, **Nav2** and **Gazebo Fortress (Ignition)**. The stack covers mapping, localization, route-graph navigation (VDA5050-style routes), ArUco-guided docking and charging, pallet detection with a depth camera, lift control, safety, diagnostics, task and mission management, and a web/MQTT bridge for a fleet dashboard.

<p align="center">
  <img src="docs/images/robot_front_right.png" width="30%"/>
  <img src="docs/images/robot_side.png" width="30%" height="30%">
  <img src="docs/images/robot_rear.png" width="50%" height="30%"/>
</p>

<p align="center">
  <img src="docs/images/gazebo_full_env.png" width="80%"/>
</p>

---

## Table of Contents
- [System Architecture](#system-architecture)
  - [1. Robot Hardware & Sensors](#1-robot-hardware--sensors)
  - [2. TF, Odometry & Localization](#2-tf-odometry--localization)
  - [3. Costmap System](#3-costmap-system)
  - [4. BT Navigation Executive](#4-bt-navigation-executive)
  - [5. Nav2 Servers (Modular ROS 2 Lifecycle Servers)](#5-nav2-servers-modular-ros-2-lifecycle-servers)
  - [6. Execution Feedback Loop](#6-execution-feedback-loop)
  - [7. Lifecycle Manager](#7-lifecycle-manager)
- [Packages](#packages)
- [Installation](#installation)
- [Usage](#usage)
- [Demo Videos](#demo-videos)
- [Gallery](#gallery)
- [License](#license)

---

## System Architecture

<p align="center">
  <img src="docs/images/ROS2_Nav2_System_Architecture.jpeg" width="90%"/>
</p>

### 1. Robot Hardware & Sensors

The robot is a differential-drive forklift. Its URDF/xacro model lives in `forklift_description`, and it is simulated in Gazebo Fortress.

| Component | Frame | ROS topic(s) | Purpose |
|---|---|---|---|
| Differential drive (left/right wheel + caster) | `base_footprint` | `/cmd_vel`, `/odom` | Motion |
| Prismatic lift (`lift_joint`, 0 – 1.2 m) | `lift_link` | `/lift_cmd`, `/joint_states` | Pallet handling |
| 2D GPU lidar | `lidar_link` | `/scan`, `/scan/points` | Localization, costmaps, collision monitor |
| IMU (Gaussian noise) | `imu_link` | `/imu` | Odometry fusion (EKF) |
| RGB-D camera (on the lift) | `camera_link` | `/camera/image_raw`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info` | ArUco docking, pallet detection |
| Charger contact sensor | — | `/charger_contact` | Charging dock confirmation |

- **Simulation bridge:** `ros_gz_bridge` (`forklift_gazebo/config/bridge_config.yaml`) carries the clock, odometry, TF, lidar, camera, `cmd_vel` and lift topics.
- **ros2_control:** the `ForkliftSystem` (`ign_ros2_control/IgnitionSystem`) runs `joint_state_broadcaster`, `diff_drive_controller` and `lift_position_controller`.
- **Lift stack (`forklift_controller`):** separate controller, driver, feedback, limit and safety nodes. The driver can talk to real hardware over serial (`/dev/ttyUSB0`, `use_hardware: false` in sim).

### 2. TF, Odometry & Localization

```
map ──(AMCL)──▶ odom ──(diff-drive odometry)──▶ base_footprint ──▶ base_link
                                                                    ├── lidar_link
                                                                    ├── imu_link
                                                                    ├── left/right_wheel, caster
                                                                    └── lift_link ── camera_link
```

| Transform / source | Provided by |
|---|---|
| `map → odom` | `nav2_amcl` (likelihood-field model, `DifferentialMotionModel`, 500–2000 particles on `/scan`) |
| `odom → base_footprint` + `/odom` | Gazebo DiffDrive / OdometryPublisher plugin |
| Robot links | `robot_state_publisher` from the URDF |
| Static map | `nav2_map_server` → `small_warehouse.yaml` |
| Mapping | `slam_toolbox` online async (`mapping_launch.py`) |
| Odometry fusion (optional) | `robot_localization` EKF (`forklift_localization/config/ekf.yaml`) fuses `/odom` and `/imu`. It is disabled by default (its include is commented out in the launch files) |

<p align="center">
  <img src="docs/images/mapping.png" width="60%"/>
</p>

### 3. Costmap System

Both costmaps use a 0.6 × 0.6 m footprint and a 0.05 m resolution.

| | Local costmap | Global costmap |
|---|---|---|
| Global frame | `odom` (5 × 5 m rolling window) | `map` (tracks unknown space) |
| Layers (in order) | obstacle → **virtual** → voxel → inflation (0.45 m) | static → **virtual** → obstacle → inflation (0.8 m) |
| Sensor input | `/scan` | `/scan` + `/map` |

**Virtual layer (`nav2-virtual-layer`):** this custom costmap plugin, `nav2_virtual_layer::VirtualLayer`, adds virtual walls and keep-out zones. They are read from `forklift_navigation/config/virtual_layer/<robot>.yaml`. The dashboard edits them through `forklift_web_bridge`, which rewrites the file and calls the `reload_shapes` service so changes apply at runtime without a restart.

### 4. BT Navigation Executive

`bt_navigator` runs the `NavigateToPose` and `NavigateThroughPoses` navigators. It loads the standard Nav2 BT plugins plus two custom ones (`forklift_navigation/src/`):
- **`ConcatenatePaths`** joins two `nav_msgs/Path` segments.
- **`PublishPath`** republishes the combined path on `/plan` so RViz can display it.

The route-graph tree (`config/behavior_trees/navigate_to_pose_w_route_server_v2.xml`) works like this:

```
RecoveryNode (6 retries)
├── PipelineSequence
│   ├── ComputeRoute  (replan only when goal updated or path invalid, 0.5 Hz)
│   │     ComputeRoute (graph) → GetPoseFromPath → ComputePathToPose (last mile)
│   │     → ConcatenatePaths → SmoothPath → PublishPath
│   │     recovery: clear global + local costmaps
│   └── FollowPath  (guarded by Inverter(IsStuck))
│         recovery: clear local + global costmaps
└── Recovery RoundRobin: ClearCostmaps → Spin 1.57 → Wait 5 s → BackUp 0.3 m
```

> **Note:** the multi-robot params (`nav2_multirobot_params.yaml`) use this tree. Single-robot `nav2_params.yaml` currently uses Nav2's default `navigate_to_pose_w_replanning_and_recovery.xml`. To use the route tree there, uncomment `default_nav_to_pose_bt_xml`.

<p align="center">
  <img src="docs/images/design_route_in_VDA5050.png" width="45%"/>
  <img src="docs/images/rviz2.png" width="45%"/>
</p>

### 5. Nav2 Servers (Modular ROS 2 Lifecycle Servers)

| Server | Plugin / configuration |
|---|---|
| `planner_server` | `nav2_navfn_planner/NavfnPlanner` (tolerance 0.5 m) |
| `route_server` | Graph routing on `config/routes/forklift_routeV2.geojson`. Scorers: `DistanceScorer`, `DynamicEdgesScorer`. Operations: `AdjustSpeedLimit`, `ReroutingService` |
| `controller_server` | **MPPI** (default, 30 Hz, `DiffDrive`, 56 steps × 0.05 s, 2000 samples, vx −0.2 – 0.6 m/s). **Regulated Pure Pursuit** and **DWB** are also available (`controller_<type>.yaml`). Uses a SimpleProgressChecker and a stateful SimpleGoalChecker (0.2 m / 0.2 rad) |
| `smoother_server` | `nav2_smoother::SimpleSmoother` |
| `behavior_server` | Spin, BackUp, DriveOnHeading, AssistedTeleop, Wait |
| `waypoint_follower` | `WaitAtWaypoint` |
| `velocity_smoother` | Closed-loop, 20 Hz |
| `collision_monitor` | Stop circle 0.3 m, slowdown circle 0.9 m (× 0.55), from `/scan` |
| `docking_server` | `opennav_docking` with 4 dock types (`charging_dock`, `station_dock`, `pallet_pick_dock`, `pallet_drop_dock`). Dock poses come from ArUco detection |

### 6. Execution Feedback Loop

```
Web dashboard / MQTT ──▶ forklift_web_bridge
        │
        ▼
forklift_mission_manager (ExecuteMission) ──▶ forklift_task_manager (ExecuteTask)
        │                                      ├── NavigateToPose ──▶ bt_navigator
        │                                      ├── DockRobot / UndockRobot ──▶ docking_server
        │                                      └── SetLiftHeight ──▶ forklift_controller
        ▼
bt_navigator ──▶ route_server / planner_server ──▶ smoother_server ──▶ path
        ▼
controller_server (MPPI + local costmap + /odom + TF) ──▶ cmd_vel_nav
        ▼
velocity_smoother ──▶ collision_monitor ──▶ /cmd_vel ──▶ robot (Gazebo / hardware)
        ▼
/odom, /tf, /scan, /imu ──▶ AMCL + costmaps ──▶ back to controller & BT
```

The progress checker, goal checker and BT conditions (`IsStuck`, `IsPathValid`, `GoalUpdated`) close the loop by triggering replanning or recovery. Action feedback and results go up to the task and mission managers and on to the dashboard. `forklift_traffic_manager` handles edge reservation (`ReserveEdge.srv`) between robots, and `forklift_safety` and `forklift_diagnostics` report robot health.

### 7. Lifecycle Manager

Each group of Nav2 nodes is managed by an `nav2_lifecycle_manager` instance with `autostart: true`. The manager steps every node through *configure* and then *activate* in order, and uses bond heartbeats to bring the group down safely if a node crashes.

| Lifecycle manager | Managed nodes |
|---|---|
| `lifecycle_manager_localization` | `map_server`, `amcl` |
| `lifecycle_manager_navigation` | `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `collision_monitor`, `route_server` |
| `lifecycle_manager_docking` | `docking_server` |

Nodes can run as separate processes or composed in a `component_container_isolated` (`use_composition:=true`). At the system level, `forklift_bringup` staggers the stages with `TimerAction` delays (navigation at 2 s, controller and ArUco at 10 s, task manager at 14 s, mission manager at 16 s).

---

## Packages

| Package | Description |
|---|---|
| `forklift_description` | URDF/xacro robot model (base, wheels, lift, sensors, ros2_control) |
| `forklift_gazebo` | Gazebo Fortress worlds, models, spawning and ROS–Gazebo bridge |
| `forklift_bringup` | One-command single- and multi-robot bringup |
| `forklift_localization` | EKF sensor fusion (robot_localization) |
| `forklift_navigation` | Nav2 config, maps, route graphs, behavior trees, custom BT nodes |
| `nav2-virtual-layer` | Costmap plugin for virtual walls and keep-out zones |
| `forklift_docking` | `opennav_docking` config and plugins for charging and pallet docks |
| `forklift_aruco` | ArUco marker pose estimation for docking |
| `pallet_vision` | Pallet detection with the depth camera |
| `forklift_controller` | Lift controller, driver, feedback, limits and safety |
| `forklift_safety` | Safety fields, speed zones, sensor monitoring |
| `forklift_diagnostics` | Hardware, sensor and maintenance diagnostics |
| `forklift_task_manager` | `ExecuteTask` action server (navigate → dock → lift workflows) |
| `forklift_mission_manager` | `ExecuteMission` action server (sequences of tasks) |
| `forklift_traffic_manager` | Multi-robot traffic and edge reservation |
| `forklift_web_bridge` | FastAPI / MQTT bridge to the web dashboard |
| `forklift_remote` | PS4 and keyboard teleoperation |
| `forklift_interfaces` | Custom messages, services and actions |

Each package has its own `README.md` with more detail.

---

## Installation

### Requirements

| Item | Version |
|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) |
| ROS 2 | Humble Hawksbill (desktop) |
| Simulator | Gazebo Fortress (Ignition Gazebo 6) |
| Python | 3.10 |
| GPU (optional) | NVIDIA GPU + CUDA. Recommended for `pallet_vision` (YOLO) and the GPU lidar in Gazebo |

### 1. Install ROS 2 Humble

Follow the official guide: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-pip
sudo rosdep init   # skip if already initialised
rosdep update
```

### 2. Install ROS packages (Nav2, Gazebo Fortress, control, etc.)

`rosdep` (step 5) installs most of these. Listing them explicitly helps when rosdep misses one:

```bash
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-route ros-humble-nav2-mppi-controller \
  ros-humble-opennav-docking ros-humble-opennav-docking-bt \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-tf-transformations ros-humble-cv-bridge \
  ros-humble-rosbridge-server ros-humble-web-video-server \
  ros-humble-joy ros-humble-topic-tools
```

> `ros-humble-ros-gz` pulls in Gazebo Fortress (`ignition-fortress`) on Ubuntu 22.04. Check it with `ign gazebo --version`, which should print 6.x.

### 3. Clone the repository

```bash
mkdir -p ~/miniforklift_ws && cd ~/miniforklift_ws
git clone https://github.com/boyloy21/autonomous-mini-forklift.git .
```

### 4. Fix machine-specific absolute paths

Some config, URDF/SDF and world files still contain absolute paths under `/home/yunboyloy/miniforklift_ws`. If you cloned into `~/miniforklift_ws` as in step 3, rewrite them to your home directory:

```bash
cd ~/miniforklift_ws
grep -rl "/home/yunboyloy" src | xargs sed -i "s#/home/yunboyloy#$HOME#g"
```

### 5. Install dependencies

```bash
cd ~/miniforklift_ws
source /opt/ros/humble/setup.bash

# ROS dependencies declared in each package.xml
rosdep install --from-paths src --ignore-src -r -y

# Python dependencies not available through rosdep
pip3 install -r requirements.txt
```

> For GPU inference, install the CUDA build of PyTorch that matches your driver. See <https://pytorch.org/get-started/locally/>. Without it, `pallet_vision` runs on the CPU.

### 6. Build

```bash
cd ~/miniforklift_ws
colcon build --symlink-install
```

To build a single package after changes: `colcon build --symlink-install --packages-select <package>`

### 7. Environment setup

Add to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/miniforklift_ws/install/setup.bash
export GZ_VERSION=fortress
export ROS_DOMAIN_ID=22          # choose a unique ID per robot/developer on the same network
```

Then `source ~/.bashrc`.

### 8. Verify

```bash
ros2 pkg list | grep forklift              # all forklift_* packages are listed
ros2 interface list | grep forklift_interfaces
ign gazebo --version                       # Gazebo Sim, version 6.x
```

### Troubleshooting

| Problem | Fix |
|---|---|
| `Unable to find uri[model://...]` / missing models in Gazebo | Re-run step 4 and rebuild. The world files reference models in the `install/` space |
| `controller_manager` cannot load `forklift_controllers.yaml` | Same cause as above: the absolute path in the URDF/SDF must point to your workspace |
| `module 'cv2' has no attribute 'aruco'` | `pip3 uninstall opencv-python && pip3 install opencv-contrib-python` |
| `cv_bridge` crashes with a NumPy 2.x error | `pip3 install "numpy<2"` |
| Topics missing between ROS and Gazebo | Make sure `GZ_VERSION=fortress` is set and `ros_gz_bridge` is running (`ros2 node list`) |
| Nodes from other machines appear in `ros2 node list` | Set a unique `ROS_DOMAIN_ID` |

---

## Usage

```bash
# 1. Start the simulation (Gazebo + robot + bridge)
ros2 launch forklift_gazebo forklift_ign_gazebo.launch.py

# 2. Start navigation, docking, controller, ArUco, task and mission managers
ros2 launch forklift_bringup single_robot_bringup.launch.py
```

Other entry points:

```bash
# Mapping with slam_toolbox
ros2 launch forklift_navigation mapping_launch.py

# Navigation only, with a specific controller (mppi | pure_pursuit | dwb)
ros2 launch forklift_navigation bringup_launch.py controller_type:=mppi

# Multi-robot
ros2 launch forklift_bringup multiple_robot_world.launch.py
ros2 launch forklift_bringup multiple_bringup_launch.py

# Web dashboard bridge
ros2 launch forklift_web_bridge forklift_web_bridge.launch.py

# Teleoperation
ros2 launch forklift_remote ps4_teleop.launch.py
ros2 run forklift_remote keyboard_control_node
```

---

## Demo Videos

Click a preview to open the video.

| Single-robot navigation (MPPI) | Patrol-point navigation |
|---|---|
| [![Single-robot navigation with MPPI](docs/images/video_thumbs/single_navigation_mppi.jpg)](docs/videos/single_navigation_mppi.mp4) | [![Patrol-point navigation](docs/images/video_thumbs/make_patrol_point_navigation.jpg)](docs/videos/make_patrol_point_navigation.mp4) |
| Point-to-point navigation with the MPPI controller | Navigating through a sequence of patrol points |

| Docking and charging | Multi-robot pallet docking |
|---|---|
| [![Docking and charging](docs/images/video_thumbs/robot_dock_charging.jpg)](docs/videos/robot_dock_charging.mp4) | [![Multi-robot pallet docking task](docs/images/video_thumbs/Mulltiple_mini_forklift_pallet_docking_task.jpg)](docs/videos/Mulltiple_mini_forklift_pallet_docking_task.mp4) |
| ArUco-guided docking at the charging station | Several forklifts running pallet docking tasks |

---

## Gallery

| ArUco marker detection | Pallet detection (depth camera) |
|---|---|
| ![](docs/images/aruco_marker_detect.png) | ![](docs/images/pallet_detect_with_depth_camera.png) |

| Charging station | Pallet in Gazebo |
|---|---|
| ![](docs/images/charging_station_gazebo.png) | ![](docs/images/pallet_gazebo.png) |

---

## License

Apache License 2.0. See the `LICENSE` file in each package.

Developed by **AI FARM ROBOTICS FACTORY**.
