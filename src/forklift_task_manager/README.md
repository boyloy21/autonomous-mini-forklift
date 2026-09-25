# forklift_task_manager

ROS 2 (Humble) `ament_cmake` package implementing the task-level state
machine for the Mini Forklift AMR: it combines navigation, docking, and
fork-lift control into six named, queueable task types (pick pallet, drop
pallet, charge, station visit, single navigation, patrol), each exposed
through one `ExecuteTask` action.

---

## 1. Overview and Responsibility

`forklift_task_manager` is the layer that turns a single high-level request
("pick pallet at dock X") into an ordered sequence of primitive steps
(`NAVIGATE`, `DOCK`, `LIFT`/`LOWER`, `UNDOCK`, `MONITOR_CHARGING`,
`STATION_OPERATION`) and executes them one at a time against the
navigation, docking, and lift subsystems — with retries, cancellation, a
FIFO task queue, and live status reporting.

It is a pure **orchestrator/client**: it contains no navigation, docking, or
lift control logic of its own — it drives `forklift_navigation`'s
`NavigateToPose` action, `forklift_docking`'s `docking_server`
(`DockRobot`/`UndockRobot` actions, called **directly**, not via
`forklift_docking`'s own `PalletDockWorkflowNode`/`pallet_dock_workflow`
action), `forklift_controller`'s `SetLiftHeight` action, and
`forklift_aruco`'s `target_marker_id` parameter.

## 2. Architecture / Role in the Overall System

```
                          execute_task (action, ExecuteTask)
                                    │
                                    ▼
                         +----------------------+
                         |   TaskManagerNode     |──▶ get_task_status (srv)
                         |  (1 action server,    |──▶ clear_task_queue (srv)
                         |   TaskQueue, worker    |──▶ task_manager/status (topic)
                         |   thread)              |
                         +----------┬-----------+
                                    │ expandTaskToSteps() -- Task -> Step[]
                                    ▼
                         +----------------------+
                         |    TaskExecutor        |
                         |  (executes ONE step    |
                         |   at a time, retries)  |
                         +----------┬-----------+
        ┌───────────────┬──────────┼───────────────┬────────────────┐
        ▼               ▼          ▼               ▼                ▼
 NAVIGATE            DOCK/UNDOCK  LIFT/LOWER   MONITOR_CHARGING  STATION_OPERATION
 navigate_to_pose    dock_robot/  set_height   /battery_state     (timed wait)
 (forklift_          undock_robot (forklift_    (current >
  navigation/Nav2)   (forklift_    controller)   threshold)
                      docking's
                      docking_server,
                      called directly)
```

Note: `forklift_task_manager` and `forklift_docking`'s
`PalletDockWorkflowNode` are **two independent orchestrators** that both
call the same underlying `dock_robot`/`undock_robot`/`set_height` actions
and the same `aruco_node.target_marker_id` parameter directly — they do not
call each other. `forklift_task_manager`'s `TaskExecutor` re-implements the
dock/undock/lift sequencing (including the ArUco marker lookup) itself
rather than delegating to `forklift_docking`'s `pallet_dock_workflow`
action. This is a real architectural duplication worth being aware of when
extending either package (see §12).

## 3. Directory Structure

```
forklift_task_manager/
├── config/
│   └── task_manager.yaml            # staging offsets, action/topic names, charging thresholds
├── include/forklift_task_manager/
│   ├── task.hpp                     # Task/Step/TaskType/StepType definitions + expandTaskToSteps()
│   ├── task_queue.hpp                # thread-safe FIFO task queue
│   ├── task_executor.hpp             # per-step execution against Nav2/docking/lift action servers
│   ├── task_manager_node.hpp         # ExecuteTask action server + services + status
│   └── dock_database.hpp             # DockInstance/DockDatabase (duplicated from forklift_docking)
├── src/
│   ├── task.cpp                      # the 6 task-type "recipes"
│   ├── task_queue.cpp
│   ├── task_executor.cpp
│   ├── task_manager_node.cpp
│   └── dock_database.cpp
├── launch/
│   └── task_manager.launch.py
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Responsibility |
|---|---|---|
| `TaskManagerNode` | `task_manager_node` | `rclcpp_components` node (no `main.cpp`, same pattern as `forklift_docking`). Hosts the `execute_task` action server, a FIFO `TaskQueue`, a dedicated worker thread that pops and executes one task at a time via `TaskExecutor`, the `get_task_status`/`clear_task_queue` services, and a periodic `task_manager/status` publisher. |

## 5. Input / Output Topics

| Name | Type | Direction | Description |
|---|---|---|---|
| `battery_topic` (param, default `battery_state`) | `sensor_msgs/BatteryState` | Input | Subscribed once at startup; `current` field compared against `charging_current_threshold` during a `MONITOR_CHARGING` step |
| `task_manager/status` | `forklift_interfaces/TaskManagerStatus` | Output | Published at `status_publish_rate_hz` (default 2 Hz): current task id/type/state, queue length |
| `task_status` | `forklift_interfaces/TaskStepStatus` | Output | One message per step/sub-step transition (transient local): task type, step, state, attempt, dock id, lift height, `detected`, `success`. See §5.1 |

`TaskExecutor` also acts as a **client** of topics/actions owned by other
packages (`navigate_to_pose`, `dock_robot`, `undock_robot`, `set_height`,
and `aruco_node`'s parameter service) — see §11.

### 5.1 `task_status` -- the step-level stream

`ExecuteTask` feedback only reaches the client that sent the goal
(`forklift_mission_manager`), and `task_manager/status` is a periodic
summary with no step detail. `task_status` is the observable stream for
everyone else -- `forklift_web_bridge` merges it into `mission_status`,
and a dashboard can subscribe to it directly.

One message is published per transition:

- `step: "TASK"`, `state: "RUNNING"` when a task starts, and
  `"SUCCESS"`/`"FAILED"`/`"CANCELED"` when it ends (`success` is set on
  the successful one).
- Each step: `RUNNING` when it starts, `RETRY` per failed attempt (with
  `attempt`/`max_attempts`), then `SUCCESS` or `FAILED` with `error_code`.
  Step names come from `StepType`: `NAVIGATE`, `DOCK`, `LIFT`, `LOWER`,
  `UNDOCK`, `MONITOR_CHARGING`, `STATION_OPERATION`.
- Docking sub-steps, forwarded from opennav_docking's `DockRobot`
  **action feedback** (only `TaskExecutor`, its action client, receives
  it): `NAV_TO_STAGING_POSE -> STAGING`, `INITIAL_PERCEPTION -> DETECT`,
  `CONTROLLING -> APPROACH` (also reports `DETECT`/`SUCCESS` and latches
  `detected: true`), `WAIT_FOR_CHARGE`, and `RETRY` while it re-detects.
  So a stalled dock is visible as `DETECT`/`RETRY` rather than one opaque
  `DOCK` step, without reading any log output.
- `MONITOR_CHARGING` reports `SUCCESS` once charging current is detected;
  `LIFT`/`LOWER` carry the commanded `lift_height`.
- Phases docking_server reports **only in its log output** -- no message
  or action feedback field carries them (verified against the installed
  opennav_docking) -- are translated here as well, while a DOCK/UNDOCK
  step of this node's task is running:

  | docking_server line | `step` / `state` |
  |---|---|
  | `Lost detection or did not detect: timeout exceeded` | `DETECT` / `RETRY` |
  | `Failed initial dock detection` | `DETECT` / `FAILED` |
  | `Successful initial dock detection` | `DETECT` / `SUCCESS` (sets `detected`) |
  | `Made contact with dock, waiting for charge to start` | `CONTACT` / `SUCCESS` |
  | `Robot is charging!` | `CHARGING` / `SUCCESS` (sets `charging`) |
  | `Attempting to undock robot ...` | `UNDOCK` / `RUNNING` |
  | `Robot has reached staging pose` | `UNDOCK_STAGING` / `SUCCESS` |
  | `Robot has undocked!` | `UNDOCK` / `SUCCESS` |

  This is why the node subscribes to `/rosout`: it keeps the log reading
  in the one node that owns these steps, instead of every observer
  (web bridge, dashboard) scraping logs itself. Only this robot's
  `docking_server` logger is read (`<namespace>.docking_server`), only
  while a DOCK/UNDOCK step is in progress, and repeats are collapsed --
  `Lost detection` is logged every control cycle (~50 Hz) but reported
  once per transition. Set `use_docking_logs: false` to turn it off; the
  action-feedback sub-steps above keep working.

The topic is transient local, so a late subscriber immediately sees the
current step.

```bash
ros2 topic echo /forklift1/task_status
```

## 6. Services and Actions

| Name | Type | Description |
|---|---|---|
| `execute_task` | `forklift_interfaces/action/ExecuteTask` | **Primary interface.** Goal: `task_id`, `task_type` (one of the 6 below), plus type-specific fields (`goal_pose`, `waypoints[]`, `dock_id`, `target_height`, `station_wait_duration`). A goal is validated by expanding it into steps at *goal-acceptance time* (rejected immediately if the type is unknown or required fields/dock lookup fail), then queued and executed by the worker thread. Feedback: `task_id`, `task_type`, `current_step`, `step_index`, `total_steps`, `state`, `progress` (0–100), `message`. Result: `success`, `error_code` (`""`/`VALIDATION_FAILED`/`<STEP>_FAILED`/`CANCELED`), `message`. |
| `get_task_status` | `forklift_interfaces/srv/GetTaskStatus` | Snapshot: whether a task is currently running, its id/type/state/step index/total steps, and the queue length. |
| `clear_task_queue` | `forklift_interfaces/srv/ClearQueue` | Removes all **pending** (not-yet-started) tasks from the queue. Does not affect the currently executing task — cancel that via the `execute_task` action's own cancel. |

### Task types and their step recipes (`expandTaskToSteps` in `task.cpp`)

| Task type | Required fields | Step sequence |
|---|---|---|
| `SINGLE_NAVIGATION` | `goal_pose` | `NAVIGATE(goal_pose)` |
| `PATROL_POINT` | `waypoints[]` (≥1) | `NAVIGATE(wp)` for each waypoint, in order |
| `PICK_PALLET_DOCK` | `dock_id` | `NAVIGATE(staging pose)` → `DOCK(dock_id, skip_staging=true)` → `LIFT(target_height or dock's `lift`)` → `UNDOCK` |
| `DROP_PALLET_DOCK` | `dock_id` | `NAVIGATE(staging pose)` → `DOCK` → `LOWER(dock's `lift`, typically 0.0)` → `UNDOCK` |
| `CHARGING_DOCK` | `dock_id` | `NAVIGATE(staging pose)` → `DOCK` → `MONITOR_CHARGING` — **deliberately no `UNDOCK` step**; the task completes with the robot still docked and charging. Undocking is expected to be a separate, later task. |
| `STATION_DOCK` | `dock_id` | `NAVIGATE(staging pose)` → `DOCK` → `STATION_OPERATION(wait_duration, default 5.0s)` → `UNDOCK` |

The staging pose for every dock-based task is computed from the dock's
database pose offset by `staging_x_offset`/`staging_yaw_offset` along the
dock's own heading — `NAVIGATE` gets the robot close, then `DOCK` is sent
with `skip_internal_staging_nav=true` (i.e. `navigate_to_staging_pose:
false` on the `DockRobot` goal) so the general-purpose Nav2 controller
doesn't also attempt the close-quarters final approach.

## 7. Important Parameters

`config/task_manager.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `dock_database` | `dock_database.yaml` (overridden by launch to `forklift_docking`'s copy) | Shared source of truth for dock poses/marker ids/lift heights — reuses `forklift_docking`'s file rather than duplicating it |
| `staging_x_offset` | `-1.2` m | Distance behind the dock (along its heading) that `NAVIGATE` targets, before `DOCK` takes over the close-quarters approach. In-source note: raised from `-0.7` after a specific drop-dock (`pallet_drop_dock_1`) staged the robot too close (`x=-1.3`) to an unmapped wall, causing the general Nav2 controller to fail its final approach there; `-1.2` (staging `x=-0.8` for that dock) leaves more of the final stretch to `docking_server`'s close-quarters approach controller |
| `staging_yaw_offset` | `0.0` rad | Additional yaw offset applied to the staging pose |
| `status_publish_rate_hz` | `2.0` Hz | `task_manager/status` publish rate |
| `use_docking_logs` | `true` | Translate this robot's docking_server log phases (detection lost/OK, contact, charging, undock staging) into `task_status` (see §5.1) |
| `navigate_action_name` / `dock_action_name` / `undock_action_name` / `lift_action_name` | `navigate_to_pose` / `dock_robot` / `undock_robot` / `set_height` | Downstream action names (relative — must share the robot's namespace) |
| `aruco_node_name` | `aruco_node` | Node whose `target_marker_id` parameter `TaskExecutor` sets before a `DOCK` step |
| `battery_topic` | `/battery_state` | Battery telemetry for `MONITOR_CHARGING` |
| `charging_current_threshold` | `0.5` A | Current above this value is treated as "charging" |
| `charging_monitor_timeout` | `300.0` s | `MONITOR_CHARGING` gives up (task fails) if this current threshold is never reached |

Each `Step` also carries `max_retries` (default `2`): `TaskManagerNode`'s
worker loop retries a failed step up to that many times (with a warning
log) before failing the whole task.

## 8. Launch Files and Run Commands

| Launch file | Starts |
|---|---|
| `task_manager.launch.py` | `task_manager_node` only (namespace + `use_sim_time` + `dock_database` path, defaulting to `forklift_docking`'s file). Does **not** start `docking_server`, `forklift_aruco`, `lift_controller_node`, or Nav2 — this node is purely a client of all of them. |

```bash
ros2 launch forklift_task_manager task_manager.launch.py

# Pick a pallet
ros2 action send_goal /execute_task forklift_interfaces/action/ExecuteTask \
  "{task_id: 'pick1', task_type: 'PICK_PALLET_DOCK', dock_id: 'pallet_pick_dock_1'}"

# Check status / clear the queue
ros2 service call /get_task_status forklift_interfaces/srv/GetTaskStatus
ros2 service call /clear_task_queue forklift_interfaces/srv/ClearQueue
```

## 9. Dependencies

From `package.xml`:

- `rclcpp`, `rclcpp_action`, `rclcpp_components`
- `std_msgs`, `std_srvs`, `geometry_msgs`, `nav2_msgs`, `sensor_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `opennav_docking_msgs` (for `DockRobot`/`UndockRobot`, matching
  `forklift_docking`'s actual action types)
- `forklift_interfaces` (`ExecuteTask.action`, `GetTaskStatus.srv`,
  `ClearQueue.srv`, `TaskManagerStatus.msg`)
- `yaml_cpp_vendor` (+ `yaml-cpp` linked directly, for dock database parsing)
- `forklift_docking`, `forklift_navigation`, `forklift_aruco`,
  `forklift_controller` (exec dependencies — the systems this node
  orchestrates)
- `launch`, `launch_ros`, `ament_index_python`

## 10. Communication Flow

1. A client (`forklift_mission_manager`, `forklift_web_bridge`, or manual
   `ros2 action send_goal`) sends an `execute_task` goal.
2. `handleGoal()` validates the goal by running `expandTaskToSteps()` as a
   dry-run probe (same function used for the real execution) — an unknown
   `task_type`, a missing `dock_id`, or a `dock_id` not found in the
   database rejects the goal immediately.
3. `handleAccepted()` builds the real `Task`, expands it into `steps[]`
   again, and pushes it onto the thread-safe `TaskQueue`.
4. The dedicated worker thread pops one task at a time (FIFO), executes its
   steps sequentially via `TaskExecutor::executeStep()`, retrying each step
   up to `max_retries` times, publishing `ExecuteTask` feedback (state,
   step index, progress) after every step transition.
5. Each step type drives a different downstream system directly:
   `NAVIGATE` → Nav2's `navigate_to_pose`; `DOCK`/`UNDOCK` → `docking_server`
   (setting `aruco_node`'s `target_marker_id` first, from the dock
   database); `LIFT`/`LOWER` → `forklift_controller`'s `set_height`;
   `MONITOR_CHARGING` → polls the cached `/battery_state` reading;
   `STATION_OPERATION` → a plain timed wait.
6. On completion, the action result (`success`/`error_code`/`message`) is
   returned, and `task_manager/status` reflects `IDLE` again until the next
   task is popped.

## 11. Integration with Other Packages

- **forklift_navigation** — `NAVIGATE` steps call its `navigate_to_pose`
  action (Nav2's `bt_navigator`).
- **forklift_docking** — `DOCK`/`UNDOCK` steps call `docking_server`'s
  `dock_robot`/`undock_robot` actions **directly** (not via
  `forklift_docking`'s own `PalletDockWorkflowNode`), duplicating some of
  that node's logic (ArUco marker lookup from the same
  `dock_database.yaml`, per-dock-type handling) — see the architecture note
  in §2.
- **forklift_aruco** — `TaskExecutor::setArucoTargetMarker()` sets
  `target_marker_id` before every `DOCK` step, exactly as
  `forklift_docking`'s workflow node does independently.
- **forklift_controller** — `LIFT`/`LOWER` steps call `set_height`.
- **forklift_mission_manager** — the intended primary client, chaining
  multiple `execute_task` goals into multi-step missions.
- **forklift_web_bridge** — can also drive `execute_task` directly for a
  single dashboard-triggered task.

## 12. Future Improvements

- **Planned / Future:** resolve the duplication between this package's
  `TaskExecutor` (direct `dock_robot`/`undock_robot`/`set_height` calls +
  ArUco marker lookup) and `forklift_docking`'s `PalletDockWorkflowNode`
  (which does the same thing, plus the charging-dock rear-contact
  maneuver). Currently, using `CHARGING_DOCK` via `forklift_task_manager`
  does **not** get the charging-specific rotate+backup final approach that
  `forklift_docking`'s workflow implements — a real behavioral gap between
  the two orchestrators for that task type specifically.
- Consider having `TaskExecutor::executeDock`/`executeUndock` call
  `forklift_docking`'s `pallet_dock_workflow` action instead of the raw
  `dock_robot`/`undock_robot` actions, to get a single source of truth for
  dock-type-specific behavior (charging maneuver, lift, marker selection)
  instead of two independently maintained copies.
- `include/forklift_task_manager/dock_database.hpp` duplicates
  `forklift_docking`'s `DockDatabase` class; consider extracting a shared
  library or interface package instead of maintaining two copies of the
  same parser.
- Persist the task queue across a `task_manager_node` restart (currently
  in-memory only — a crash or restart loses all pending tasks).
