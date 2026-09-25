# forklift_safety

ROS 2 (Humble) `ament_cmake` package implementing the Mini Forklift AMR's
safety subsystem: seven C++ nodes covering sensor liveness, obstacle
fusion, a LiDAR-based protective field, position-based speed zones,
emergency stop, discrete safety I/O, and a top-level safety manager that
aggregates all of them into one SAFE/WARNING/SLOWDOWN/STOP/EMERGENCY
state and a single `allow_motion` gate.

---

## 1. Overview and Responsibility

`forklift_safety` is the package responsible for deciding whether the
robot is currently safe to move, and at what speed. It does not itself
send velocity commands to the base — it publishes a `bool`
(`/safety/allow_motion`) and an advisory max speed (`/max_speed`) that a
motion-consuming layer (e.g. a velocity smoother/limiter, or
`forklift_controller`) is expected to respect; nothing in this package
enforces that respect itself.

All seven nodes are launched together by `forklift_safety.launch.py`, but
**that launch file is not included by any other package's bringup chain**
— `forklift_safety` must be launched manually and is not currently active
in the normal simulation/bringup flow.

## 2. Architecture / Role in the Overall System

```
/scan ──────┬──────────────────────────────────────────────────────────┐
            ▼                                                          ▼
   protective_field_node                                    obstacle_monitor_node
   (nested warning/slow/stop                                (fuses /scan, /ultrasonic,
    zones from forward LiDAR                                 /bumper, /depth_camera
    cone) ─▶ /protective_field                                presence) ─▶ /obstacle_state
                                                                    │
/ultrasonic, /bumper, /depth_camera ───────────────────────────────┘
forklift/scan, forklift/imu, forklift/ultrasonic,
forklift/camera/image_raw, forklift/encoder
            │
            ▼
   sensor_monitor_node ─▶ forklift/sensor_status (liveness only, not consumed by safety_manager_node)

/emergency_stop_button, /web/emergency_stop
            │
            ▼
   emergency_stop_node (latching, requires /emergency_stop/reset
                          Trigger service to clear) ─▶ /emergency_stop

/map, /robot_pose
            │
            ▼
   speed_zone_node (rectangular zone lookup, config-defined) ─▶ /max_speed

/safety_input
            │
            ▼
   safety_io_node (door/light-curtain/relay/external passthrough) ─▶ /safety_io

┌───────────────────────────────────────────────────────────────────────┐
│                          safety_manager_node                          │
│  subscribes: /emergency_stop, /protective_field, /obtacle_state (sic, │
│  see §12 -- typo, does NOT match obstacle_monitor_node's actual       │
│  "/obstacle_state" topic), /max_speed, /safety_io                     │
│  publishes:  /safety/status (SafetyStatus), /safety/allow_motion      │
└───────────────────────────────────────────────────────────────────────┘
```

## 3. Directory Structure

```
forklift_safety/
├── config/
│   ├── safety.yaml       # safety_manager_node params (loaded)
│   ├── sensor.yaml       # sensor_monitor_node, protective_field_node, obstacle_monitor_node params (loaded)
│   └── speed_zone.yaml   # speed_zone_node zone definitions (loaded)
├── include/forklift_safety/
│   ├── emergency_stop_node.hpp
│   ├── obstacle_monitor_node.hpp
│   ├── protective_field_node.hpp
│   ├── safety_io_node.hpp
│   ├── safety_manager_node.hpp
│   ├── sensor_monitor_node.hpp
│   └── speed_zone_node.hpp
├── src/
│   └── *.cpp   # one .cpp per node above, each with its own main()
├── launch/
│   └── forklift_safety.launch.py   # starts all 7 nodes; NOT included by any other package
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Responsibility |
|---|---|---|
| `SensorMonitorNode` | `sensor_monitor_node` | Tracks last-message time per sensor (LiDAR, IMU, ultrasonic, camera, wheel encoder); declares a sensor "down" if it hasn't published within its configured timeout. Publishes `forklift_interfaces/SensorStatus` at `check_rate_hz`. Purely observational — not consumed by `safety_manager_node`. |
| `ObstacleMonitorNode` | `obstacle_monitor_node` | Fuses `forklift/scan` (min LiDAR range), `/ultrasonic` (min ultrasonic range), `/bumper` (any-side contact), and `/depth_camera` (presence-only, no image processing) into one `ObstacleState`. `collision_detected` comes **solely** from the bumper (physical contact); `obstacle_detected`/`path_blocked` come from LiDAR+ultrasonic proximity vs. `obstacle_distance`. Publishes to `/obstacle_state`. |
| `ProtectiveFieldNode` | `protective_field_node` | Derives nested `warning`/`slow`/`stop` zone booleans from the minimum LiDAR range within a forward angular cone (`front_angle_deg`, default ±90°). Publishes `ProtectiveFieldStatus` to `/protective_field`. |
| `SpeedZoneNode` | `speed_zone_node` | Looks up the robot's `/robot_pose` against named rectangular zones (config-defined, **not** derived from `/map` semantics — `/map` is subscribed but not currently used for anything) and publishes the matching zone's `max_speed` to `/max_speed`. |
| `EmergencyStopNode` | `emergency_stop_node` | Latches `/emergency_stop = true` if either the physical E-stop button (`/emergency_stop_button`) or the dashboard/wireless E-stop (`/web/emergency_stop`) is active. Once latched, only clears via the `/emergency_stop/reset` `std_srvs/Trigger` service, and only if both sources have already released — "reset after operator confirmation," where the service call itself **is** that confirmation, per an in-source design note. |
| `SafetyIoNode` | `safety_io_node` | For real hardware: passes through `/safety_input` (door switch, light curtain, safety relay, external signals) into `/safety_io` with an aggregated `all_ok` flag for `safety_manager_node`. |
| `SafetyManagerNode` | `safety_manager_node` | Core decision node. Combines e-stop, obstacle state, protective field, max-speed, and safety-IO into one `SafetyStatus` (`SAFE`/`WARNING`/`SLOWDOWN`/`STOP`/`EMERGENCY`/`RESET`) and a single `allow_motion` gate, published on `/safety/status` and `/safety/allow_motion`. It does **not** republish speed itself — `/max_speed` only factors into its status message, not into gating (gating comes from e-stop/obstacle/field/IO only). See §12 for a real topic-name mismatch that breaks its obstacle input in practice. |

## 5. Input/Output Topics

| Name | Type | Direction | Node(s) | Notes |
|---|---|---|---|---|
| `forklift/scan` (relative) | `sensor_msgs/LaserScan` | Input | `sensor_monitor_node`, `obstacle_monitor_node` | Relative topic name, not `/scan` |
| `forklift/imu` | `sensor_msgs/Imu` | Input | `sensor_monitor_node` | |
| `forklift/ultrasonic` | `forklift_interfaces/UltrasonicArray` | Input | `sensor_monitor_node` | Relative — note `obstacle_monitor_node` subscribes to the **different**, absolute `/ultrasonic` topic for the same data (see §12) |
| `forklift/camera/image_raw` | `sensor_msgs/Image` | Input | `sensor_monitor_node` | |
| `forklift/encoder` | `nav_msgs/Odometry` | Input | `sensor_monitor_node` | Wheel odometry, read as `Odometry` |
| `/ultrasonic` | `forklift_interfaces/UltrasonicArray` | Input | `obstacle_monitor_node` | Absolute — see topic-naming mismatch note in §12 |
| `/bumper` | `forklift_interfaces/BumperState` | Input | `obstacle_monitor_node` | |
| `/depth_camera` | `sensor_msgs/Image` | Input | `obstacle_monitor_node` | Presence-only; no processing |
| `/scan` (LaserScan subscription inside `protective_field_node`) | `sensor_msgs/LaserScan` | Input | `protective_field_node` | |
| `/map` | `nav_msgs/OccupancyGrid` | Input | `speed_zone_node` | Subscribed, currently unused for any computation (reserved for future map-aware zone lookup) |
| `/robot_pose` | `geometry_msgs/PoseStamped` | Input | `speed_zone_node` | |
| `/emergency_stop_button` | `std_msgs/Bool` | Input | `emergency_stop_node` | Best-effort QoS |
| `/web/emergency_stop` | `std_msgs/Bool` | Input | `emergency_stop_node` | Best-effort QoS |
| `/safety_input` | `forklift_interfaces/SafetyIOInput` | Input | `safety_io_node` | |
| `forklift/sensor_status` | `forklift_interfaces/SensorStatus` | Output | `sensor_monitor_node` | Relative topic |
| `/obstacle_state` | `forklift_interfaces/ObstacleState` | Output | `obstacle_monitor_node` | See §12 — `safety_manager_node` subscribes to a differently-spelled topic |
| `/protective_field` | `forklift_interfaces/ProtectiveFieldStatus` | Output | `protective_field_node` | |
| `/max_speed` | `std_msgs/Float32` | Output | `speed_zone_node` | Topic name configurable in `safety_manager_node` via `max_speed_topic` param |
| `/emergency_stop` | `std_msgs/Bool` | Output | `emergency_stop_node` | Best-effort QoS |
| `/safety_io` | `forklift_interfaces/SafetyIOStatus` | Output | `safety_io_node` | |
| `/safety/status` | `forklift_interfaces/SafetyStatus` | Output | `safety_manager_node` | |
| `/safety/allow_motion` | `std_msgs/Bool` | Output | `safety_manager_node` | Primary motion-gating signal for the rest of the system |

## 6. Services and Actions

| Name | Type | Provided by | Description |
|---|---|---|---|
| `/emergency_stop/reset` | `std_srvs/Trigger` | `emergency_stop_node` | Clears the latched e-stop, but only if both the physical button and the web/dashboard e-stop have already released. |

No actions are provided by this package.

## 7. Important Parameters

`config/safety.yaml`:

| Node | Parameter | Default | Description |
|---|---|---|---|
| `safety_manager_node` | `max_speed_topic` | `/max_speed` | Topic subscribed for informational max-speed (not used for gating) |
| | `status_timeout_sec` | `2.0` | Declared but not read anywhere in `safety_manager_node.cpp`'s constructor logic beyond declaration — currently has no observable effect |
| `emergency_stop_node` | *(none — empty `ros__parameters: {}` block)* | — | |
| `safety_io_node` | *(none — empty block)* | — | |

`config/sensor.yaml`:

| Node | Parameter | Default | Description |
|---|---|---|---|
| `sensor_monitor_node` | `lidar_timeout_sec` / `imu_timeout_sec` / `ultrasonic_timeout_sec` / `camera_timeout_sec` / `encoder_timeout_sec` | `1.0` / `0.5` / `1.0` / `2.0` / `0.5` s | Per-sensor staleness thresholds |
| | `check_rate_hz` | `5.0` | Publish rate for `forklift/sensor_status` |
| `protective_field_node` | `warning_distance` / `slow_distance` / `stop_distance` | `1.5` / `0.8` / `0.4` m | Nested zone thresholds |
| | `front_angle_deg` | `90.0` | **Not actually applied** — the code declares this parameter as `front_angle_de` (missing the trailing `g`), so the YAML value is silently ignored and the code default (`90.0`, which happens to match) is always used. See §12. |
| `obstacle_monitor_node` | `obstacle_distance` | `0.5` m | Proximity threshold for `obstacle_detected`/`path_blocked` |
| | `check_rate_hz` | `10.0` | |

`config/speed_zone.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `default_max_speed` | `0.5` m/s | **Not actually applied** — the code declares this parameter as `default_speed_max` (words swapped), so the YAML value is silently ignored and the code default (`0.5`, which happens to match) is always used. See §12. |
| `zone_names` | `["aisle_a", "dock_area", "open_area"]` | List of configured rectangular zones |
| `<zone>.min_x/max_x/min_y/max_y/max_speed` | e.g. `aisle_a`: `[0,5]×[0,2]`, `max_speed=0.3`; `dock_area`: `[5,8]×[0,2]`, `max_speed=0.2`; `open_area`: `[-10,0]×[-10,10]`, `max_speed=1.0` | Per-zone axis-aligned bounding box and speed cap |

## 8. Launch Files and How to Run Them

| Launch file | Starts |
|---|---|
| `forklift_safety.launch.py` | All seven nodes: `sensor_monitor_node`, `obstacle_monitor_node`, `protective_field_node`, `speed_zone_node`, `emergency_stop_node`, `safety_io_node`, `safety_manager_node` — loaded with `safety.yaml` + `speed_zone.yaml` + `sensor.yaml` together and `use_sim_time` (default `true`) |

```bash
ros2 launch forklift_safety forklift_safety.launch.py

# Check the aggregated safety state
ros2 topic echo /safety/status
ros2 topic echo /safety/allow_motion

# Reset a latched e-stop (only succeeds once both sources have released)
ros2 service call /emergency_stop/reset std_srvs/srv/Trigger
```

Not currently included by `forklift_bringup` or any other package's
launch chain — must be run standalone.

## 9. Dependencies

From `package.xml`:

- `rclcpp`, `std_msgs`, `std_srvs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `forklift_interfaces` (`UltrasonicArray`, `BumperState`, `ObstacleState`,
  `ProtectiveFieldStatus`, `SafetyIOInput`, `SafetyIOStatus`,
  `SafetyStatus`, `SensorStatus`)
- `launch`, `launch_ros`, `ament_index_python` (exec, for the launch file)

## 10. Integration with the Rest of the System

- **forklift_interfaces** — supplies every custom message type this
  package publishes/subscribes.
- **forklift_bringup / other packages** — none currently launch or
  consume this package's topics; `/safety/allow_motion` and `/max_speed`
  are designed to be consumed by a motion-limiting layer (e.g. a velocity
  smoother, or `forklift_controller`), but no such consumer currently
  exists in this workspace.
- **forklift_diagnostics** — conceptually adjacent (both do read-only
  monitoring), but the two packages are entirely independent: no shared
  topics, no shared code.

## 11. Known Issues / TODO / Future Work

- **`safety_manager_node` subscribes to a topic `obstacle_monitor_node`
  doesn't publish**: `safety_manager_node.cpp` subscribes to
  `"/obtacle_state"` (typo — missing the second `s`), while
  `obstacle_monitor_node.cpp` publishes to `"/obstacle_state"` (correct
  spelling). This means `safety_manager_node` **never receives real
  obstacle data** in the running system — its `obstacle_.collision_detected`
  and `obstacle_.obstacle_detected` fields stay at their default
  (`false`), so collision/obstacle-triggered `EMERGENCY`/`SLOWDOWN` states
  can never actually fire through this path today. This is a real,
  currently-live bug, not a hypothetical one.
- **Two silently-ignored config parameters**: `protective_field_node`
  declares its front-cone-angle parameter as `front_angle_de` instead of
  `front_angle_deg` (typo), and `speed_zone_node` declares its default
  speed parameter as `default_speed_max` instead of `default_max_speed`
  (word order). Both bugs are currently masked because the code's
  hardcoded default happens to equal the YAML's intended value (`90.0`
  and `0.5` respectively) — changing either value in the YAML would have
  **no effect** until the parameter name is fixed to match.
- **`obstacle_monitor_node` and `sensor_monitor_node` disagree on the
  ultrasonic topic**: `sensor_monitor_node` watches the relative
  `forklift/ultrasonic`, while `obstacle_monitor_node` watches the
  absolute `/ultrasonic` — likely meant to be the same physical sensor
  feed, but as configured they can only both receive data if something
  publishes to both names.
- **`status_timeout_sec` in `safety.yaml` has no observable effect** —
  declared in `safety_manager_node` but never read anywhere else in that
  node's logic.
- **No motion-limiting consumer exists yet**: `/safety/allow_motion` and
  `/max_speed` are published but nothing in this workspace currently
  subscribes to either — the safety gate is computed but not enforced
  anywhere in the current system.
- **Not included in any bringup**: `forklift_safety.launch.py` must be
  run manually; consider adding it to `forklift_bringup` once the topic
  mismatches above are fixed and a motion-limiting consumer exists.
