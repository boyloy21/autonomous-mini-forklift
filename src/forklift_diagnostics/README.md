# forklift_diagnostics

ROS 2 (Humble) `ament_python` package implementing read-only health
monitoring for the Mini Forklift AMR: sensor liveness/frequency checks,
host system resource checks (CPU/RAM/temperature/GPU), and (implemented
but not currently launched) hardware, communication, and maintenance
monitoring, aggregated into standard `diagnostic_msgs/DiagnosticArray`
output plus a set of custom `forklift_interfaces` status messages.

---

## 1. Overview and Responsibility

`forklift_diagnostics` observes the rest of the system and reports on its
health — it never commands anything and contains no safety-stop or
control logic of any kind (that is `forklift_safety`'s job). It ships
seven nodes:

- Two that are actually wired into the shipped launch file
  (`sensor_monitor_node`, `system_monitor_node`), both publishing to the
  standard ROS `/diagnostics` topic (`diagnostic_msgs/DiagnosticArray`).
- Five more (`hardware_monitor_node`, `communication_monitor_node`,
  `maintenance_monitor_node`, `diagnostics_manager_node`,
  `diagnostics_publisher_node`) that are fully implemented, registered as
  `console_scripts`, and even have their own config files — but are
  **never started by `diagnostics.launch.py`** and are not referenced by
  any other package's launch file either. They only run if invoked
  manually with `ros2 run`.

## 2. Architecture / Role in the Overall System

```
                    ┌────────────────────────┐
   /scan  ─────────▶│  sensor_monitor_node     │──▶ /diagnostics (DiagnosticArray)
   /camera/image_raw│  (lidar/camera/imu        │      [LAUNCHED]
   /imu   ─────────▶│   liveness + frequency)   │
                    └────────────────────────┘

                    ┌────────────────────────┐
   psutil (CPU/RAM) │  system_monitor_node     │──▶ /diagnostics (DiagnosticArray)
   nvidia-smi (GPU) │  (host resource usage)    │      [LAUNCHED]
                    └────────────────────────┘

  ---- everything below exists in source but is NOT launched ----

  /hardware/*_topic ─▶ hardware_monitor_node   ──▶ /hardware/status
  (BMS/motor bridge,     (Float32 topics; bridge
   not yet built)         not yet built elsewhere)

  psutil ifaces,     ─▶ communication_monitor_node ─▶ /communication/status
  /dev/tty* paths        (CAN/Ethernet/WiFi/UART/
                          RS485/USB link presence)

  config only        ─▶ maintenance_monitor_node ─▶ /maintenance/status
  (calibration age,      (no live calibration-tracking
   runtime hours)         subsystem exists yet)

  /hardware/status,  ─▶ diagnostics_manager_node ─▶ /diagnostics/status
  /system/status,        (aggregates all 5 topics;    /diagnostics/error
  /sensor/status,         edge-triggered alerts)       /diagnostics/warning
  /communication/status,
  /maintenance/status

  (same 5 topics)    ─▶ diagnostics_publisher_node ─▶ /robot_health
                         (single dashboard-facing        (RobotHealth)
                          consolidated message)
```

Note the naming split: the two launched nodes publish standard
`diagnostic_msgs/DiagnosticArray` on `/diagnostics` (the conventional ROS
diagnostics topic, consumable by `diagnostic_aggregator`/`rqt_robot_monitor`).
The five unlaunched nodes instead publish custom
`forklift_interfaces` status messages (`HardwareStatus`, `CommunicationStatus`,
`MaintenanceStatus`, `DiagnosticsStatus`, `RobotHealth`) on their own
package-specific topics — a second, parallel diagnostics scheme that
never gets exercised in the running system today.

## 3. Directory Structure

```
forklift_diagnostics/
├── config/
│   ├── diagnostics.yaml      # sensor_monitor_node + system_monitor_node params -- the ONLY file loaded by the launch file
│   ├── sensor.yaml           # sensor_monitor_node params, DIFFERENT schema/keys than diagnostics.yaml -- unused
│   ├── hardware.yaml         # hardware_monitor_node params -- unused (node not launched)
│   └── maintenance.yaml      # maintenance_monitor_node params -- unused (node not launched)
├── forklift_diagnostics/
│   ├── sensor_monitor_node.py         # LAUNCHED
│   ├── system_monitor_node.py         # LAUNCHED
│   ├── hardware_monitor_node.py       # not launched
│   ├── communication_monitor_node.py  # not launched
│   ├── maintenance_monitor_node.py    # not launched
│   ├── diagnostics_manager_node.py    # not launched -- aggregator for the unlaunched 5
│   ├── diagnostics_publisher_node.py  # not launched -- dashboard consolidator
│   └── utils.py
├── launch/
│   └── diagnostics.launch.py   # starts ONLY sensor_monitor_node + system_monitor_node
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
└── setup.py
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Launched? | Responsibility |
|---|---|---|---|
| `SensorWatcher`-based node | `sensor_monitor_node` | **Yes** | Watches `/scan` (LiDAR), `/camera/image_raw`, `/imu` message arrival timing and frequency; per-sensor status is `ERROR` (never received, or stale beyond timeout), `WARN` (receiving but below min frequency), or `OK`. Publishes a `DiagnosticArray` at `publish_rate_hz`. Does not touch message payloads. |
| `SystemMonitorNode` | `system_monitor_node` | **Yes** | Reads host CPU%, RAM%, temperature via `psutil`, and (optionally) GPU utilization/temperature via `nvidia-smi` subprocess — degrades gracefully to a `WARN` status (not a crash) if `nvidia-smi` isn't present. Publishes a `DiagnosticArray` at `publish_rate_hz`. Read-only; no control action of any kind. |
| `HardwareMonitorNode` | `hardware_monitor_node` | No | Watches `Float32` topics for battery voltage/current/temperature and motor temperatures, comparing against warn/error thresholds and a staleness timeout. Its own docstring notes the expected BMS/motor-driver bridge that would actually publish those `Float32` topics **has not been built elsewhere in this project yet** — so today every device reports `ERROR` ("no data") rather than a real reading. Publishes `forklift_interfaces/HardwareStatus` to `/hardware/status`. |
| `CommunicationMonitorNode` | `communication_monitor_node` | No | Checks CAN/Ethernet/WiFi interface up/down state via `psutil.net_if_stats()`, and UART/RS485/USB device presence via path existence. Packet loss, CRC errors, reconnect count, and latency are reported as `0` with an explanatory note rather than fabricated, since no real bus driver exists to measure them yet. Publishes `forklift_interfaces/CommunicationStatus` to `/communication/status`. |
| `MaintenanceMonitorNode` | `maintenance_monitor_node` | No | Compares config-supplied "days since last calibration" figures against configured intervals (camera/LiDAR/IMU/wheel), motor runtime hours against a service interval, battery health against a replace threshold, and firmware version against an expected value — all figures come from `maintenance.yaml`, not any live subsystem, since no calibration-tracking subsystem exists yet. Publishes `forklift_interfaces/MaintenanceStatus` to `/maintenance/status`. |
| `DiagnosticsManagerNode` | `diagnostics_manager_node` | No | Subscribes to all five of `/hardware/status`, `/system/status`, `/sensor/status`, `/communication/status`, `/maintenance/status`; republishes a consolidated `DiagnosticsStatus` on `/diagnostics/status` and emits **edge-triggered** alerts (only when a device's level worsens vs. the previous tick, not every cycle) on `/diagnostics/error` / `/diagnostics/warning`. Note: it subscribes to `/sensor/status` (a `SensorDiagnostics` message), which nothing in this package currently publishes — `sensor_monitor_node` publishes plain `DiagnosticArray` on `/diagnostics`, not `SensorDiagnostics` on `/sensor/status`. |
| `DiagnosticsPublisherNode` | `diagnostics_publisher_node` | No | Same five subscriptions as the manager node; consolidates them into one `RobotHealth` message on `/robot_health` for a dashboard. `battery_percent` is sourced from `maintenance_monitor_node`'s configured `battery_health_percent` figure — a static health estimate, not a live state-of-charge reading. |

## 5. Input/Output Topics

| Name | Type | Direction | Node | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Input | `sensor_monitor_node` | `lidar_timeout_sec`/`lidar_min_freq_hz` |
| `/camera/image_raw` | `sensor_msgs/Image` | Input | `sensor_monitor_node` | `camera_timeout_sec`/`camera_min_freq_hz` |
| `/imu` | `sensor_msgs/Imu` | Input | `sensor_monitor_node` | `imu_timeout_sec`/`imu_min_freq_hz` |
| `diagnostics` (relative, param `diagnostics_topic`) | `diagnostic_msgs/DiagnosticArray` | Output | `sensor_monitor_node`, `system_monitor_node` | Both nodes publish to the same relative topic name |
| `/hardware/status` | `forklift_interfaces/HardwareStatus` | Output | `hardware_monitor_node` (unlaunched) | |
| `/communication/status` | `forklift_interfaces/CommunicationStatus` | Output | `communication_monitor_node` (unlaunched) | |
| `/maintenance/status` | `forklift_interfaces/MaintenanceStatus` | Output | `maintenance_monitor_node` (unlaunched) | |
| `/diagnostics/status`, `/diagnostics/error`, `/diagnostics/warning` | `forklift_interfaces/DiagnosticsStatus` / `DiagnosticsAlert` | Output | `diagnostics_manager_node` (unlaunched) | |
| `/robot_health` | `forklift_interfaces/RobotHealth` | Output | `diagnostics_publisher_node` (unlaunched) | |

No services or actions are provided by this package.

## 6. Important Parameters

`config/diagnostics.yaml` (the only config file actually loaded, by `diagnostics.launch.py`):

| Node | Parameter | Default | Description |
|---|---|---|---|
| `sensor_monitor_node` | `lidar_topic` / `camera_topic` / `imu_topic` | `/scan` / `/camera/image_raw` / `/imu` | Topics watched |
| | `lidar_timeout_sec` / `camera_timeout_sec` / `imu_timeout_sec` | `2.0` / `2.0` / `1.0` | Seconds since last message before `ERROR` |
| | `lidar_min_freq_hz` / `camera_min_freq_hz` / `imu_min_freq_hz` | `5.0` / `10.0` / `20.0` | Below this → `WARN` |
| | `publish_rate_hz` | `1.0` | `/diagnostics` publish rate |
| `system_monitor_node` | `cpu_warn_percent` / `cpu_error_percent` | `80.0` / `95.0` | |
| | `ram_warn_percent` / `ram_error_percent` | `80.0` / `95.0` | |
| | `temp_warn_c` / `temp_error_c` | `75.0` / `90.0` | |
| | `gpu_enabled` | `true` | Enables the `nvidia-smi` subprocess check |
| | `publish_rate_hz` | `1.0` | `/diagnostics` publish rate |

The three other config files (`sensor.yaml`, `hardware.yaml`,
`maintenance.yaml`) are installed by `setup.py` but **never referenced by
the launch file** — they only take effect if their corresponding node is
started manually with an explicit `--params-file`. `sensor.yaml` in
particular uses an entirely different parameter set (`check_rate_hz`,
`enable_gps`, `*_expected_hz`, depth camera/ultrasonic/bumper/encoder
timeouts) than `diagnostics.yaml`'s `sensor_monitor_node` block — the two
files are not interchangeable despite both targeting a node of the same
name.

## 7. Launch Files and Run Commands

| Launch file | Starts |
|---|---|
| `diagnostics.launch.py` | `sensor_monitor_node` + `system_monitor_node` only, both loaded from `config/diagnostics.yaml`, namespace + `use_sim_time` (default `true`) configurable |

```bash
ros2 launch forklift_diagnostics diagnostics.launch.py

# Inspect standard ROS diagnostics
ros2 topic echo /diagnostics

# Manually run one of the five nodes that aren't in the launch file
ros2 run forklift_diagnostics hardware_monitor_node --ros-args --params-file \
  install/forklift_diagnostics/share/forklift_diagnostics/config/hardware.yaml
```

## 8. Dependencies

From `package.xml`:

- `rclpy`
- `diagnostic_msgs`, `sensor_msgs`
- `python3-psutil` (exec)

Note: `forklift_interfaces` is **not** declared as a dependency in
`package.xml`, even though `hardware_monitor_node.py`,
`communication_monitor_node.py`, `maintenance_monitor_node.py`,
`diagnostics_manager_node.py`, and `diagnostics_publisher_node.py` all
`import` message types from it (`HardwareStatus`, `CommunicationStatus`,
`MaintenanceStatus`, `DiagnosticsStatus`, `DiagnosticsAlert`,
`RobotHealth`, `SensorDiagnostics`, `SystemStatus`). A commented-out block
in `package.xml` shows an earlier, more complete dependency list
(`std_msgs`, `nav_msgs`, `forklift_interfaces`, `python3-psutil`) that was
never uncommented. This works today only because none of those five nodes
are exercised by the launch file or CI — running one directly would still
succeed since `forklift_interfaces` is built as part of the same
workspace, but the missing `<depend>` is a real gap for anyone building
this package in isolation.

## 9. Integration with the Rest of the System

- **forklift_bringup** / other launch files — none currently include
  `diagnostics.launch.py`; this package must be launched standalone.
- **forklift_interfaces** — supplies all the custom status message types
  used by the five unlaunched nodes (see §8 for the missing dependency
  declaration).
- Nothing in this package publishes to or subscribes from
  `forklift_safety`, `forklift_task_manager`, or any other package's
  topics — it is a pure, standalone observer.

## 10. Known Issues / TODO / Future Work

- **Five of seven nodes are dead weight in practice**: `hardware_monitor_node`,
  `communication_monitor_node`, `maintenance_monitor_node`,
  `diagnostics_manager_node`, and `diagnostics_publisher_node` are fully
  implemented, have their own config files, and are registered as
  `console_scripts`, but are never started by `diagnostics.launch.py` and
  never referenced by any other package. Either wire them into the launch
  file (once the topics they depend on — a BMS/motor bridge, real
  calibration tracking — actually exist) or remove them.
- **`diagnostics_manager_node` subscribes to a topic nothing publishes**:
  it listens on `/sensor/status` (`forklift_interfaces/SensorDiagnostics`),
  but `sensor_monitor_node` (the only sensor-monitoring node actually
  launched) publishes plain `diagnostic_msgs/DiagnosticArray` on
  `/diagnostics` instead — there is no `SensorDiagnostics` publisher
  anywhere in this package.
- **`forklift_interfaces` missing from `package.xml`**: five node files
  import message types from it without a corresponding `<depend>` (see
  §8).
- **`hardware_monitor_node.hpp`-equivalent typo**: `config/hardware.yaml`
  declares `statle_timeout_sec` (misspelled), while the node code declares
  the parameter as `stale_timeout_sec` — so even if this config were
  wired up, that particular value would silently fall back to the code
  default (`3.0`) rather than the YAML's value.
- **Two incompatible "sensor" config schemas**: `config/diagnostics.yaml`'s
  `sensor_monitor_node` block and `config/sensor.yaml`'s
  `sensor_monitor_node` block use entirely different parameter names for
  what looks like the same node — worth consolidating into one schema.
- **No real hardware/communication data sources yet**: both
  `hardware_monitor_node` and `communication_monitor_node` are honest in
  their own docstrings that they're waiting on bridges/drivers that don't
  exist yet in this project (a BMS/motor-driver bridge, and real CAN/UART
  bus statistics).
