# forklift_docking

ROS 2 (Humble) `ament_cmake` package implementing precision docking for the
Mini Forklift AMR: charging-station docking, station docking, and pallet
pick/drop docking, built on Nav2's `opennav_docking` server plus a custom
orchestration node that chains docking, fork lifting, and undocking into one
operation.

---

## 1. Overview and Responsibility

`forklift_docking` is responsible for the **final-approach and physical
docking** stage of a task — everything after the robot has been
navigated (by `forklift_navigation`) to the vicinity of a dock, and up to
backing safely away again. It owns:

- Configuration of Nav2's stock `opennav_docking` server (`docking_server`)
  for four dock types: `charging_dock`, `station_dock`, `pallet_pick_dock`,
  `pallet_drop_dock` — all currently implemented with the same
  `opennav_docking::SimpleChargingDock` plugin (this Humble backport of
  `opennav_docking` does not ship any other `ChargingDock` plugin besides
  `SimpleChargingDock`/`TestFailureDock`).
- A **dock instance database** (`config/dock_database.yaml`) mapping named
  docks to a world pose, ArUco marker ID, and target fork-lift height.
- **`PalletDockWorkflowNode`**: the orchestrator that combines
  `DockRobot` → (charging-specific final approach) → `SetLiftHeight` →
  (charging-specific undo) → `UndockRobot` into one cancelable, feedback-
  reporting operation.
- Single- and multi-robot launch variants.

It does **not** perform ArUco detection itself (that's `forklift_aruco`,
consumed via its `aruco_dock_pose`/`target_marker_id` interface) and does
not decide *when* to dock — that is `forklift_task_manager`'s job, which
this package's action interface is designed to be driven by.

## 2. Architecture / Role in the Overall System

```
forklift_navigation (gets robot to the dock's staging pose)
            │
            ▼
+---------------------------+        +----------------------+
|  docking_server            |◀──────▶|  forklift_aruco       |
|  (opennav_docking,         | detected_dock_pose  (aruco_dock_pose,
|   4x SimpleChargingDock)   |  remap  target_marker_id set via
+---------------------------+         parameter client)
            ▲  DockRobot / UndockRobot (opennav_docking_msgs actions)
            │
+---------------------------+        +------------------------+
|  PalletDockWorkflowNode    |───────▶|  forklift_controller    |
|  (pallet_dock_workflow_    | set_height (action)  (lift_controller_node)
|   node)                    |
+---------------------------+
     ▲ pallet_dock_workflow (action)  │ /dock_pallet (legacy topic)
     │
forklift_task_manager / forklift_mission_manager (decides WHEN to dock)
```

## 3. Directory Structure

```
forklift_docking/
├── config/
│   ├── docking_server.yaml              # single-robot opennav_docking config (4 dock plugins)
│   ├── docking_server_multirobot.yaml   # namespace-templated variant
│   └── dock_database.yaml               # named dock instances: pose, ArUco id, lift height
├── include/forklift_docking/
│   ├── dock_database.hpp                # DockInstance / DockType / DockDatabase
│   └── pallet_dock_workflow_node.hpp
├── src/
│   ├── dock_database.cpp
│   └── pallet_dock_workflow_node.cpp    # the orchestrator (rclcpp_components node)
├── scripts/
│   └── pallet_dock_node.py              # legacy/alternate Python implementation of the same workflow
├── launch/
│   ├── docking.launch.py                # single-robot docking_server + lifecycle_manager
│   ├── docking_manager.launch.py        # single-robot pallet_dock_workflow_node
│   ├── multirobot_docking.launch.py     # per-robot docking_server + lifecycle_manager, namespace-templated
│   └── nav2_docking.launch.py           # references files/executables that do not exist in this package (see §12)
├── plugin.xml                           # pluginlib descriptions for a custom dock-plugin library — NOT built (see §4)
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Status | Responsibility |
|---|---|---|---|
| `docking_server` | `opennav_docking` (external, package `opennav_docking`) | Active | Stock Nav2 docking server; configured here for 4 dock types, all via `SimpleChargingDock`. Provides `DockRobot`/`UndockRobot` actions and a "shared approach controller" (`k_phi`/`k_delta`/`beta`/`lambda` — a Smooth Nav-style dock approach controller). |
| `lifecycle_manager_docking` | `lifecycle_manager` (external, package `nav2_lifecycle_manager`) | Active | Brings `docking_server` through `configure → activate` automatically. |
| `PalletDockWorkflowNode` | `pallet_dock_workflow_node` | **Active** | Custom `rclcpp_components` node (no `main.cpp` — registered as a component, consistent with `docking_server`'s own pattern). Orchestrates `DockRobot` → charging-specific rotate+backup → `SetLiftHeight` → charging-specific undo → `UndockRobot`, exposed via the `pallet_dock_workflow` action (primary) and a legacy `/dock_pallet` topic trigger. Both paths call the same internal `executeWorkflowSteps()` so there is one implementation, not two that could drift. |
| `fake_battery_node` | `fake_battery_node.py` | Active (sim) | Simulated battery per robot, started by `forklift_bringup/multiple_bringup_launch.py` (`use_fake_battery:=true`). Publishes `<ns>/battery_state`: charges `charge_rate` (default 1 %/s, +2 A) while this robot touches a charger pad on `<ns>/charger_contact`, otherwise slowly discharges. The task manager's CHARGING_DOCK waits for 100 % (`charging_full_percentage`), then CHARGE_RELEASE drives `charging_release_distance` forward off the pad. |
| `pallet_dock_workflow_node.py` | `pallet_dock_node.py` | Legacy/alternate, not referenced by any launch file | An earlier, Python `rclpy` re-implementation of the same dock→delay→lift→undock sequence, using `SetLiftHeight` as a **service** rather than the action the C++ node and current `forklift_controller` actually implement. Kept in the package but effectively superseded by the C++ `PalletDockWorkflowNode`. |

Not built (dead code, see `CMakeLists.txt`): a `forklift_docking_plugins`
pluginlib library (`ChargingDock`, `StationDock`, `PalletPickDock`,
`PalletDropDock` plugins implementing per-dock-type behavior), a
`dock_manager` executable, and a `dock_detector_node` — all fully
commented out. `plugin.xml` still describes this unbuilt library. The
project currently achieves all four dock "types" using generic
`SimpleChargingDock` instances differentiated only by parameters, not by
distinct plugin classes.

## 5. Input / Output Topics

| Name | Type | Direction | Node | Description |
|---|---|---|---|---|
| `detected_dock_pose` (remapped to `aruco_dock_pose` / `/forklift<N>/aruco_dock_pose`) | `geometry_msgs/PoseStamped` | Input | `docking_server` | Vision correction pose from `forklift_aruco`'s `aruco_node`; `SimpleChargingDock`'s subscription topic name is hardcoded internally and only reachable via this remap |
| `odom` | `nav_msgs/Odometry` | Input | `docking_server` | Used for the approach controller and `dock_backwards` staging |
| `dock_pallet` | `std_msgs/String` (data = dock_id) | Input | `pallet_dock_workflow_node` | Legacy trigger — no feedback, no cancellation |
| `cmd_vel` (or namespaced) | `geometry_msgs/Twist` | Output | `pallet_dock_workflow_node` | Drive commands for the charging-dock rear-contact maneuver only |
| `odom` (namespaced) | `nav_msgs/Odometry` | Input | `pallet_dock_workflow_node` | Closed-loop feedback for the rear-contact maneuver |
| `charger_contact` (e.g. `/forklift1/charger_contact`) | `ros_gz_interfaces/Contacts` | Input | `pallet_dock_workflow_node`, `task_manager_node`, `fake_battery_node` | Every charger pad's contact sensor (Gazebo `/charger_contacts`), bridged into each robot's namespace by `multiple_robot_world.launch.py`. Only contacts with a charger's `dock_contact_collision` pad that name this robot's Gazebo model (`charging_robot_name`, default = namespace) count |
| `/tf`, `/tf_static` (remapped per-namespace) | `tf2_msgs/TFMessage` | Input | `docking_server` | TF lookups (`map`, `odom`, `base_footprint`); `tf2_ros::Buffer` subscribes to hardcoded absolute names internally, so an explicit remap is required for namespacing |

## 6. Services and Actions

| Name | Type | Provided by | Consumed by | Description |
|---|---|---|---|---|
| `dock_robot` | `opennav_docking_msgs/action/DockRobot` | `docking_server` | `PalletDockWorkflowNode` | Navigate to staging pose + dock, by `dock_id` |
| `undock_robot` | `opennav_docking_msgs/action/UndockRobot` | `docking_server` | `PalletDockWorkflowNode` | Back away from the current dock |
| `set_height` | `forklift_interfaces/action/SetLiftHeight` | `forklift_controller` | `PalletDockWorkflowNode` | Raise/lower the fork to the dock's configured height |
| `pallet_dock_workflow` | `forklift_interfaces/action/PalletDockWorkflow` | `PalletDockWorkflowNode` | `forklift_task_manager` (or manual testing) | **Primary interface.** Goal: `dock_id`. Feedback: `state` (`SET_MARKER`/`DOCKING`/`CHARGING_APPROACH`/`WAITING`/`LIFTING`/`CHARGING_APPROACH_UNDO`/`UNDOCKING`/`DONE`), `message`, `progress` (0–100). Result: `success`, `error_code` (`""`/`DOCK_FAILED`/`LIFT_FAILED`/`CHARGING_APPROACH_FAILED`/`CANCELED`), `message`. Fully cancelable — cancellation is propagated into whichever downstream action is currently in flight. |

`forklift_interfaces` also defines standalone `DockRobot.action` and
`UndockRobot.action` message types, but **neither is referenced anywhere
in the codebase** — all actual docking uses `opennav_docking_msgs`'
`DockRobot`/`UndockRobot` instead. These two `forklift_interfaces` actions
appear to be unused/superseded and are candidates for removal or for
documenting as explicitly deprecated.

## 7. Important Parameters

### `docking_server.yaml`

| Parameter | Value | Description |
|---|---|---|
| `controller_frequency` | 50.0 Hz | Approach control loop rate |
| `dock_prestaging_tolerance` | 0.5 m | How close staging pose must be reached before final approach |
| `max_retries` | 3 | Docking attempt retries |
| `dock_backwards` | false | Docks are approached forward |
| Approach controller `k_phi` / `k_delta` / `beta` / `lambda` | 3.0 / 2.0 / 0.4 / 2.0 | Smooth-approach control law gains, shared across all dock types |
| `v_linear_min` / `v_linear_max` | 0.05 / 0.15 m/s | Approach speed range |
| `slowdown_radius` | 0.25 m | Distance at which approach speed begins ramping down |

### Per-dock-type `SimpleChargingDock` parameters (all use the same plugin class)

| Dock type | `staging_x_offset` / `staging_yaw_offset` | `use_external_detection_pose` | Notes |
|---|---|---|---|
| `charging_dock` | −0.5 m / 0.0 (face the marker) | true | `external_detection_timeout: 3.0`; docks forward, then the rear-contact maneuver reverses into the pad — see §8 |
| `station_dock` | −0.7 m / 0.0 | true | `external_detection_timeout: 1.0` |
| `pallet_pick_dock` | −0.7 m / 0.0 | true | Also declares `detector_service_name: 'detector/toggle'` (a toggle-able detector service — not implemented in `forklift_aruco`, which runs continuously; **Planned/Future**) |
| `pallet_drop_dock` | −0.7 m / 0.0 | **false** (blind/database-pose docking) | No live vision correction for the drop approach |

### `dock_database.yaml`

Each entry: `type`, `frame` (always `map`), `pose: [x, y, yaw]`, `id`
(ArUco marker ID as a string, used to set `aruco_node`'s
`target_marker_id`), `lift` (target fork height in meters). Example
entries: `charging_dock_1` (marker `4`), two `station_dock_*` (both marker
`5`), four `pallet_pick_dock_*` (markers `0`–`3`, lift `0.08` m), four
`pallet_drop_dock_*` (no marker id — blind docking, lift `0.0` m).

### `PalletDockWorkflowNode` parameters

| Parameter | Default | Description |
|---|---|---|
| `dock_pallet_topic` | `dock_pallet` | Legacy trigger topic |
| `workflow_action_name` | `pallet_dock_workflow` | Primary action name |
| `post_dock_delay` | 2.0 s | Settle time after `DockRobot` succeeds, before lifting |
| `lift_target_height` / `lift_speed` | 0.15 m / 0.0 (= controller default) | Fallback if `dock_id` isn't found in the database (normally each dock's own `lift:` value is used) |
| `dock_action_name` / `undock_action_name` / `set_height_service_name` | `dock_robot` / `undock_robot` / `set_height` | Downstream action names (relative — must share the robot's namespace) |
| `set_height_timeout` | 35.0 s | Client-side ceiling waiting for `SetLiftHeight`'s result |
| `lift_height_tolerance` | 0.03 m | If `SetLiftHeight` reports failure but ends within this tolerance of target, the workflow still accepts it |
| `aruco_node_name` | `aruco_node` | Node whose `target_marker_id` parameter is set automatically from the dock database's `id` field before docking |
| `charging_pre_rotate_backoff` | 0.35 m | Reverse after `DockRobot` so the in-place turn clears the charger |
| `charging_rotate_angle` | π rad | Turn so the rear faces the charger |
| `charging_backup_distance` | 0.8 m | **Maximum** reverse; stops early on pad contact or wheel stall, fails if neither happens |
| `charging_linear_speed` / `charging_angular_speed` / `charging_min_angular_speed` | 0.05 m/s / 0.8 rad/s / 0.3 rad/s | Maneuver speeds (turn slows toward the minimum near π) |
| `charging_rotate_timeout` | 0 (auto) | Turn timeout; auto = 1.5·angle/min speed + 10 s |
| `charging_rotate_stall_timeout` / `charging_rotate_stall_angle` | 3.0 s / 0.03 rad | Turn counted as blocked when it makes less than this progress |
| `charging_contact_topic` / `charging_robot_name` | `charger_contact` / namespace (`forklift` if none) | Contact source and the Gazebo model name to match; empty topic = stall detection only |
| `charging_stall_timeout` / `charging_stall_distance` | 1.0 s / 0.005 m | "Pushing against the charger" detection |

## 8. Charging, Station, and Pallet Docking Behavior

All four dock types share the same `DockRobot`/`UndockRobot` flow through
`opennav_docking`, but `PalletDockWorkflowNode` adds dock-type-specific
behavior around it:

- **Approach & alignment**: `docking_server`'s shared approach controller
  drives the robot from the staging pose toward the dock, correcting using
  `forklift_aruco`'s live `detected_dock_pose` (except `pallet_drop_dock`,
  which docks blind, database-pose only).
- **Charging dock — final approach (rear-contact problem)**: the robot's
  camera is front-only, but the charging contact is at the rear.
  `DockRobot`'s vision-corrected approach only gets the robot correctly
  staged and facing the dock; it cannot drive the final rear-contact motion.
  `RearChargingManeuver` (library `rear_charging_maneuver`, also used by
  `forklift_task_manager`'s `CHARGE_APPROACH` step) finishes it,
  closed-loop on odometry: reverse `charging_pre_rotate_backoff`, turn
  `charging_rotate_angle` (π), then reverse until the charger pad reports
  contact with *this* robot (or the wheels stall), at most
  `charging_backup_distance`. Before `UndockRobot`, the workflow node
  mirrors exactly what was driven, restoring the `DockRobot` end pose so
  `UndockRobot` (which reverses) doesn't push into the charger.
- **Pallet pick / drop**: `PalletDockWorkflowNode` calls `SetLiftHeight`
  with the dock's database `lift` height after docking (`0.08` m for pick
  docks) and before undocking, so the fork is raised/lowered while the
  robot is stationary at the pallet.
- **Station docking**: same `DockRobot`/`UndockRobot` flow with no lift step
  beyond the (typically `0.0`) database value and no charging-specific
  maneuver.
- **Tolerances, timeouts, recovery**: `docking_threshold`/
  `docking_threshold_angular` (0.05–0.1 m / 0.05 rad) gate final-alignment
  success per dock type; `dock_approach_timeout` (30 s),
  `initial_perception_timeout` (5 s), and `wait_charge_timeout` (5 s) bound
  the server-side operation; `max_retries: 3` governs `docking_server`'s own
  retry behavior. `PalletDockWorkflowNode` layers its own client-side
  timeouts (10 s goal-acceptance, 60 s `DockRobot` result, 35 s
  `SetLiftHeight` result, 30 s `UndockRobot` result) as a sanity ceiling
  against an unresponsive server, distinct from the server's own internal
  timeout.

## 9. Launch Files and Run Commands

| Launch file | Starts | Notes |
|---|---|---|
| `docking.launch.py` | `docking_server` + `lifecycle_manager_docking` (single robot) | Overrides `dock_database` to a portable install-relative path; remaps `detected_dock_pose` → `/aruco_dock_pose` |
| `docking_manager.launch.py` | `pallet_dock_workflow_node` (single robot, namespace-aware) | Does **not** start `docking_server`, `forklift_aruco`, or `lift_controller_node` — those must already be running |
| `multirobot_docking.launch.py` | `docking_server` + `lifecycle_manager_docking`, per robot (`OpaqueFunction`, namespace-templated) | Applies the same `namespace`-token substitution technique used by `forklift_navigation`'s multi-robot launch; explicitly builds the `detected_dock_pose` remap target as an absolute per-robot topic |
| `nav2_docking.launch.py` | An `aruco_dock_bridge` executable + `opennav_docking` + `lifecycle_manager` | **Not currently functional as-is** — see §12: references `aruco_dock_bridge` (no such executable is built by this package's `CMakeLists.txt`) and `config/dock_marker.yaml`/`config/docking_server_params.yaml` (neither file exists in `config/`) |

```bash
# Single-robot: docking server + lifecycle manager
ros2 launch forklift_docking docking.launch.py

# Single-robot: the pallet dock workflow orchestrator
ros2 launch forklift_docking docking_manager.launch.py

# Multi-robot: per-robot docking server (called once per robot, typically
# from forklift_navigation's multiple_bringup_launch.py)
ros2 launch forklift_docking multirobot_docking.launch.py namespace:=forklift1

# Trigger a dock/lift/undock workflow
ros2 action send_goal /pallet_dock_workflow forklift_interfaces/action/PalletDockWorkflow "{dock_id: 'pallet_pick_dock_1'}"

# Legacy trigger (no feedback/cancel)
ros2 topic pub -1 /dock_pallet std_msgs/msg/String "{data: 'charging_dock_1'}"
```

## 10. Dependencies

From `package.xml`:

- `opennav_docking`, `opennav_docking_core`, `opennav_docking_bt`,
  `opennav_docking_msgs`
- `nav2_lifecycle_manager`, `nav2_common`, `nav2_msgs`
- `rclcpp`, `rclcpp_action`, `rclcpp_components`, `rclpy`
- `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `yaml_cpp_vendor` (+ `yaml-cpp` linked directly)
- `forklift_aruco` (exec dependency — vision correction pose)
- `forklift_interfaces` (`SetLiftHeight`, `PalletDockWorkflow` actions)
- `launch`, `launch_ros`

## 11. Communication Flow

1. `forklift_navigation` (via Nav2) delivers the robot to a dock's staging
   pose (`DockRobot`'s own `navigate_to_staging_pose: true` can also trigger
   this internally).
2. `PalletDockWorkflowNode` looks up the dock in `dock_database.yaml`, sets
   `forklift_aruco`'s `target_marker_id` parameter to the dock's marker ID
   (`SET_MARKER` state), then sends a `DockRobot` goal.
3. `docking_server`'s `SimpleChargingDock` plugin corrects the final
   approach using `forklift_aruco`'s vision pose (via the
   `detected_dock_pose` remap), reporting success once within
   `docking_threshold`/`docking_threshold_angular`.
4. For `charging_dock` only: an explicit open-loop rotate+backup maneuver
   completes the rear-contact approach (`CHARGING_APPROACH` state).
5. After `post_dock_delay`, `PalletDockWorkflowNode` calls `set_height` on
   `forklift_controller` with the dock's database `lift` value
   (`LIFTING` state).
6. For `charging_dock` only: the rotate+backup maneuver is undone
   (`CHARGING_APPROACH_UNDO`) before undocking.
7. `PalletDockWorkflowNode` sends an `UndockRobot` goal
   (`UNDOCKING` state), then reports `DONE`/100% and returns success/failure
   via the `pallet_dock_workflow` action result.

## 12. Integration with Other Packages

- **forklift_aruco** — supplies `detected_dock_pose` (remapped from
  `aruco_dock_pose`) and receives the `target_marker_id` parameter set from
  `dock_database.yaml`.
- **forklift_controller** — provides the `set_height` action consumed for
  every pick/drop/charging lift adjustment.
- **forklift_navigation** — delivers the robot to the dock's vicinity;
  `multiple_bringup_launch.py` launches this package's multi-robot docking
  stack alongside navigation.
- **forklift_task_manager** — the intended primary client of the
  `pallet_dock_workflow` action, combining it with navigation and lift
  steps into a full pick/drop/charge/station task (see that package's
  README).
- **forklift_mission_manager** — indirectly, via `forklift_task_manager`.

## 13. Future Improvements

- **Planned / Future:** fix or remove `nav2_docking.launch.py` — it
  references a non-existent `aruco_dock_bridge` executable and two
  non-existent config files (`dock_marker.yaml`,
  `docking_server_params.yaml`); it does not currently work as shipped.
- **Planned / Future:** implement (or remove) the commented-out
  `forklift_docking_plugins` pluginlib library
  (`ChargingDock`/`StationDock`/`PalletPickDock`/`PalletDropDock`) described
  in `plugin.xml` but never built — currently all four dock "types" are
  parameter variations of the same `SimpleChargingDock` plugin, not
  distinct behavior classes.
- **Planned / Future:** implement `pallet_pick_dock`'s
  `detector_service_name: 'detector/toggle'` toggle service in
  `forklift_aruco`, or remove the parameter — `aruco_node` currently runs
  continuously and has no such service.
- Remove or clearly deprecate `forklift_interfaces`' unused
  `DockRobot.action`/`UndockRobot.action` in favor of the
  `opennav_docking_msgs` actions actually used everywhere.
- Reconcile `scripts/pallet_dock_node.py` (service-based `SetLiftHeight`
  call) with the current action-based `forklift_controller` interface, or
  remove it since it is not referenced by any launch file and would fail
  against the current lift controller.
- Add explicit charge-detection wiring (`use_battery_status` /
  `use_stall_detection`, both currently `false` for `charging_dock`) so
  charging completion is actually detected rather than assumed.
