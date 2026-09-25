# forklift_localization

ROS 2 (Humble) `ament_cmake` package providing a thin configuration
wrapper around `robot_localization`'s `ekf_node`: it fuses wheel odometry
and IMU data into a filtered `odom`→`base_link` estimate for the Mini
Forklift AMR. The package contains no source code of its own — only a
parameter YAML and a launch file for the external `ekf_node` executable.

---

## 1. Overview and Responsibility

`forklift_localization`'s entire job is configuring and launching the
`robot_localization` package's `ekf_node` (an Extended Kalman Filter) with
this robot's specific odometry and IMU topics and frame names. It does
not implement any estimation logic itself, publish any custom messages,
or provide any services/actions — the actual filtering is entirely inside
`robot_localization`, an external dependency.

**Important:** despite existing and being fully configured, this
package's launch file is **not currently included by any other package's
launch/bringup chain** — see §2 and §9.

## 2. Architecture / Role in the Overall System

```
              /odom (nav_msgs/Odometry, wheel odometry)
                    │
                    ▼
   ┌─────────────────────────────────┐
   │  ekf_node (robot_localization,   │──▶ /odometry/filtered (implicit,
   │   external package, configured   │      robot_localization default)
   │   here via ekf.yaml)             │──▶ odom -> base_link TF (publish_tf: true)
   └─────────────────────────────────┘
                    ▲
                    │
              /imu (sensor_msgs/Imu, yaw + yaw rate only)
```

`world_frame: odom` means this filter fuses *continuous* odometry data
(wheel odom + IMU) into the `odom` frame only — it is **not** configured
to fuse any discrete/global correction source (GPS, AMCL, ArUco), so it
plays no role in map-frame localization; that is handled separately by
Nav2's AMCL in `forklift_navigation`.

`forklift_gazebo` declares `forklift_localization` as an `exec_depend`
and its `forklift_ign_gazebo.launch.py` imports
`get_package_share_directory('forklift_localization')`, but the actual
`IncludeLaunchDescription` for `ekf_gazebo.launch.py` is **commented out**
in that file — so in the currently running simulation stack, this
package's EKF is configured for use but not actually started. Whatever
provides `odom`→`base_link` TF in practice today comes from elsewhere
(e.g. a Gazebo diff-drive plugin's own odometry publisher), not from this
package's `ekf_node`.

## 3. Directory Structure

```
forklift_localization/
├── config/
│   └── ekf.yaml              # ekf_node parameters: frames, fusion config, rate
├── launch/
│   └── ekf_gazebo.launch.py  # starts robot_localization's ekf_node
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Package | Responsibility |
|---|---|---|---|
| `ekf_node` | `ekf_node` | `robot_localization` (external) | Fuses `/odom` (linear x/y velocity + yaw + yaw velocity, per `odom0_config`) and `/imu` (yaw + yaw rate, per `imu0_config`) into a filtered state estimate at `frequency` Hz, publishing `odom`→`base_link` TF. |

This package builds and installs no executables of its own.

## 5. Input/Output Topics

| Name | Type | Direction | Description |
|---|---|---|---|
| `/odom` (`odom0`) | `nav_msgs/Odometry` | Input | Wheel odometry; fused fields per `odom0_config`: `x`, `y` position, `yaw` orientation, `vx` linear velocity, `vyaw` angular velocity |
| `/imu` (`imu0`) | `sensor_msgs/Imu` | Input | Fused fields per `imu0_config`: `vyaw` angular velocity, `ax`/`ay` linear acceleration — **not** absolute `yaw` orientation itself (that field is `false` in `imu0_config`) |
| `/odometry/filtered` (implicit `robot_localization` default output topic) | `nav_msgs/Odometry` | Output | Filtered pose/twist estimate |
| `odom` → `base_link` TF | `tf2_msgs/TFMessage` | Output | Published because `publish_tf: true` |

No custom messages, services, or actions are defined or used by this
package.

## 6. Services and Actions

None.

## 7. Important Parameters

`config/ekf.yaml` (`ekf_node`):

| Parameter | Value | Description |
|---|---|---|
| `use_sim_time` | `true` | Hardcoded in the YAML itself (in addition to the launch argument of the same name) |
| `frequency` | `30.0` Hz | Filter output rate |
| `two_d_mode` | `false` | Full 3D omnidirectional motion model is used, not simplified to planar |
| `publish_acceleration` | `true` | |
| `publish_tf` | `true` | Broadcasts `odom`→`base_link` over `/tf` |
| `map_frame` / `odom_frame` / `base_link_frame` | `map` / `odom` / `base_link` | |
| `world_frame` | `odom` | Continuous-fusion mode (no discrete global corrections fused here — see §2) |
| `odom0` | `/odom` | Wheel odometry input topic |
| `odom0_config` | `[X=T, Y=T, Z=F, roll=F, pitch=F, yaw=T, vX=T, vY=F, vZ=F, vroll=F, vpitch=F, vyaw=T, aX=F, aY=F, aZ=F]` | Fuses X, Y position, yaw orientation, X-velocity, and yaw-velocity from wheel odom (standard robot_localization 15-element layout) |
| `imu0` | `/imu` | IMU input topic |
| `imu0_config` | `[X=F, Y=F, Z=F, roll=F, pitch=F, yaw=F, vX=F, vY=F, vZ=F, vroll=F, vpitch=F, vyaw=T, aX=T, aY=T, aZ=F]` | Fuses only yaw angular velocity and X/Y linear acceleration from the IMU — absolute `yaw` orientation is **not** fused from the IMU, only from wheel odometry above |

## 8. Launch Files and How to Run Them

| Launch file | Starts | Notes |
|---|---|---|
| `ekf_gazebo.launch.py` | `ekf_node` (from `robot_localization`) | Declares `ekf_config_file` (default: this package's `ekf.yaml`) and `use_sim_time` (default `true`) launch arguments |

```bash
ros2 launch forklift_localization ekf_gazebo.launch.py

# Or with an overridden config
ros2 launch forklift_localization ekf_gazebo.launch.py ekf_config_file:=/path/to/custom_ekf.yaml
```

Not currently invoked by any other package's launch chain — see §2/§9.

## 9. Dependencies

From `package.xml`:

- `robot_localization` (the only functional dependency — provides `ekf_node`)

No dependency on `forklift_interfaces`, `forklift_description`, or any
other `forklift_*` package; this is a minimal, standalone wrapper.

## 10. Integration with the Rest of the System

- **forklift_gazebo** — declares `forklift_localization` as an
  `exec_depend` and resolves its share directory, but the actual
  `IncludeLaunchDescription` for `ekf_gazebo.launch.py` is commented out
  in `forklift_ign_gazebo.launch.py`. This package is therefore **not
  currently active** in the running simulation stack.
- **forklift_navigation** — consumes `/odom` and TF for AMCL/Nav2, but
  independently of whether this package's EKF is running (AMCL provides
  its own map-frame correction via `/scan` regardless).
- No other package references `forklift_localization`.

## 11. Known Issues / TODO / Future Work

- **Not wired into the running system**: the package is fully configured
  and ready to launch, but `forklift_gazebo`'s include of
  `ekf_gazebo.launch.py` is commented out. Either re-enable that include
  (and confirm nothing else is already publishing conflicting
  `odom`→`base_link` TF) or remove the now-stale `exec_depend` and
  share-directory lookup from `forklift_gazebo` if this filter is not
  meant to be used.
- **No map-frame fusion configured**: with `world_frame: odom` and no
  second, map-frame `ekf_node`/`ukf_node` instance, this package plays no
  part in absolute (map-frame) localization — that's entirely AMCL's job
  in `forklift_navigation`. If a two-tier EKF (local `odom` + global
  `map`) is ever wanted, it would need a second `ekf_node` instance fusing
  a global correction source, which does not exist here today.
- No launch argument exists to select between a real-hardware IMU/odometry
  source and the simulated ones — `use_sim_time` defaults to `true` and
  the topics (`/odom`, `/imu`) are hardcoded in `ekf.yaml` rather than
  exposed as launch arguments, so real-hardware use would require editing
  the YAML directly.
