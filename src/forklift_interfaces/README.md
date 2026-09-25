# forklift_interfaces

ROS 2 (Humble) `ament_cmake` interface-only package (a member of
`rosidl_interface_packages`) defining every custom message, service, and
action type shared across the Mini Forklift AMR's other packages. It
contains no nodes, no launch files, and no logic of its own — purely
`.msg`/`.srv`/`.action` definitions plus the generated
`rosidl_default_generators`/`rosidl_default_runtime` bindings.

---

## 1. Overview and Responsibility

Every other package in this workspace that needs a custom ROS message
type depends on `forklift_interfaces` rather than defining its own. The
`CMakeLists.txt` groups the ~35 messages, 5 services, and 6 actions with
comments naming which consuming package each group belongs to
(`forklift_controller`, `forklift_mission_manager`, `forklift_safety`,
`forklift_diagnostics`, `forklift_web_bridge`, `forklift_task_manager`,
`forklift_traffic_manager`), which makes this file the closest thing this
workspace has to a single cross-package interface changelog. Not every
type defined here is actually used, though — see §6.

## 2. Architecture / Role in the Overall System

```
forklift_interfaces (msg/srv/action definitions only, no nodes)
        ▲
        │  build_depend / exec_depend
        │
  ┌─────┼───────────────────────────────────────────────────────────┐
  │     │                                                             │
forklift_controller   forklift_safety      forklift_diagnostics       │
(Lift*.msg,            (UltrasonicArray,    (DeviceStatus,            │
 SetLiftHeight          BumperState,         HardwareStatus,          │
 .action/.srv)          SafetyStatus, ...)   SensorDiagnostics, ...)  │
                                                                       │
forklift_task_manager  forklift_traffic_    forklift_web_bridge  forklift_docking
(ExecuteTask.action,    manager              (RobotState,        (SetLiftHeight.action
 TaskManagerStatus,     (ReserveEdge.srv)     WebBridgeStatus,    via forklift_controller,
 GetTaskStatus.srv,                           Navigation/Mission  PalletDockWorkflow.action)
 ClearQueue.srv)                              Command/Status)
  │
forklift_mission_manager
(Mission*.msg, ExecuteMission.action)
```

## 3. Directory Structure

```
forklift_interfaces/
├── action/
│   ├── DockRobot.action           # UNUSED -- superseded by opennav_docking_msgs/DockRobot
│   ├── UndockRobot.action         # UNUSED -- superseded by opennav_docking_msgs/UndockRobot
│   ├── ExecuteMission.action      # forklift_mission_manager
│   ├── ExecuteTask.action         # forklift_task_manager
│   ├── PalletDockWorkflow.action  # forklift_docking
│   └── SetLiftHeight.action       # forklift_controller / forklift_docking
├── msg/                           # ~35 message definitions (see §5 for the full grouping)
├── srv/
│   ├── ClearQueue.srv             # forklift_task_manager
│   ├── GetTaskStatus.srv          # forklift_task_manager
│   ├── ReserveEdge.srv            # forklift_traffic_manager
│   ├── SetLiftHeight.srv          # legacy service form, see §6
│   └── StartDocking.srv           # UNUSED anywhere in this workspace
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

None — this package builds only interface definitions, no executables.

## 5. Message / Service / Action Catalogue

Grouped as `CMakeLists.txt` itself groups them (by consuming package),
with an added **Used?** column reflecting what a workspace-wide search
actually found consuming each type.

### Actions

| Action | Goal → Result / Feedback (summary) | Used by | Status |
|---|---|---|---|
| `DockRobot` | `station_id`, `dock_type` (charging/pick/drop), `approach_speed` → `success`/`final_alignment_error`; feedback `stage` (searching/aligning/approaching/final_adjust/locked), `distance_to_target`, `lateral_error`, `yaw_error` | — | **Unused.** `forklift_docking` uses `opennav_docking_msgs/DockRobot` instead (same name, different, richer package). Candidate for removal. |
| `UndockRobot` | `retreat_distance`, `retreat_speed` → `success`/`distance_traveled` | — | **Unused**, same reason as above (`opennav_docking_msgs/UndockRobot` is used instead). |
| `SetLiftHeight` | `target_height`, `speed` → `success`, `current_height`; feedback `current_height`/`target_height` | `forklift_controller` (server), `forklift_docking`, `forklift_task_manager` (clients) | **Active** — the primary lift interface. |
| `PalletDockWorkflow` | `dock_id` → `success`, `error_code`, `message`; feedback `state` (`SET_MARKER`/`DOCKING`/`CHARGING_APPROACH`/`LIFTING`/`UNDOCKING`/...), `message`, `progress` | `forklift_docking`'s `PalletDockWorkflowNode` | **Active.** |
| `ExecuteTask` | `task_id`, `task_type` (6 types), plus type-specific fields → `success`, `error_code`, `message`; feedback `current_step`, `step_index`/`total_steps`, `state`, `progress` | `forklift_task_manager` | **Active** — the task-level orchestration interface. |
| `ExecuteMission` | `mission_id`, `mission_type` (`PALLET_TRANSPORT`/`PATROL`/`CHARGING`/`STATION_OPERATION`) plus type-specific fields → `success`/`failed_task`; feedback `current_task`, `progress`, `state` | `forklift_mission_manager` | **Active** — chains multiple `ExecuteTask` goals. |

### Services

| Service | Fields | Used by | Status |
|---|---|---|---|
| `ReserveEdge` | Req: `robot_name`, `edge_id`, `acquire`; Res: `granted` | `forklift_traffic_manager`'s `traffic_manager_node` (server) and its `TrafficReservation` `nav2_route` plugin (client) | **Active** — the fleet edge-arbitration interface. |
| `GetTaskStatus` | Res only: `has_current_task`, `current_task_id/type/state`, `current_step_index`, `total_steps`, `queue_length` | `forklift_task_manager` | **Active.** |
| `ClearQueue` | Res only: `success`, `tasks_cleared` | `forklift_task_manager` | **Active.** |
| `SetLiftHeight` | Req: `target_height`, `speed`; Res: `success`, `message`, `current_height` | Comment says "used by lift_client_node for `/lift_up`/`/lift_down`/`/set_height`" | **Legacy/duplicate** — `forklift_controller`'s actual lift interface is the `SetLiftHeight` **action** above, not this service. Only `forklift_docking`'s superseded `scripts/pallet_dock_node.py` (not referenced by any launch file) calls a `SetLiftHeight` service. |
| `StartDocking` | Req: `dock_name`; Res: `success`, `message` | — | **Unused anywhere in this workspace.** No node advertises or calls it. |

### Messages (by consuming package, per `CMakeLists.txt`'s own grouping)

| Group / package | Messages |
|---|---|
| Early/ungrouped (docking + task + lift, pre-dating the per-package comments) | `ArucoMarkers`, `Task`, `TaskStatus`, `TaskResult`, `NavigationRequest`, `NavigationResult`, `DockingResult`, `DockingRequest`, `LiftRequest`, `LiftResult` |
| `forklift_controller` | `LiftMotorCommand`, `CurrentHeight`, `LiftLimitStatus`, `LiftEncoderHeight`, `LiftDriverStatus` |
| `forklift_mission_manager` | `Mission`, `MissionStatus`, `MissionStep` |
| `forklift_safety` | `UltrasonicArray`, `BumperState`, `SafetyStatus`, `ProtectiveFieldStatus`, `ObstacleState`, `SafetyIOInput`, `SafetyIOStatus`, `SensorStatus` |
| `forklift_diagnostics` | `DeviceStatus`, `HardwareStatus`, `SensorDiagnostics`, `SystemStatus`, `CommunicationStatus`, `MaintenanceItem`, `MaintenanceStatus`, `DiagnosticsAlert`, `DiagnosticsStatus`, `RobotHealth` |
| `forklift_web_bridge` | `RobotState`, `WebBridgeStatus`, `NavigationCommand`, `NavigationStatus`, `MissionCommand` |
| `forklift_task_manager` | `TaskManagerStatus` (plus `ExecuteTask.action`, `GetTaskStatus.srv`, `ClearQueue.srv` above) |

Several of the "early/ungrouped" messages (`Task`, `TaskResult`,
`NavigationRequest`/`Result`, `DockingRequest`/`Result`,
`LiftRequest`/`Result`) look like an earlier, message-based design for
task/navigation/docking/lift coordination that predates the current
action-based interfaces (`ExecuteTask`, `SetLiftHeight` action,
`PalletDockWorkflow`) — a workspace-wide search found no publisher or
subscriber for any of these nine messages in the other 16 packages; they
appear to be dead/superseded definitions kept for compatibility or
historical reasons rather than active interfaces. The exception in this
same early group is `TaskStatus`, which **is** actively used —
`forklift_web_bridge`'s `mqtt_bridge_node.py` subscribes to it on
`/task_feedback` and republishes it over MQTT, even though nothing in the
current `forklift_task_manager`/`forklift_mission_manager` action-based
pipeline appears to publish a `TaskStatus` message to that topic (they
report progress via `ExecuteTask`/`ExecuteMission` action feedback
instead) — worth checking whether `/task_feedback` still has an actual
publisher anywhere before relying on it.

## 6. Services and Actions

Covered together with the message catalogue in §5, since this package's
entire content is interface definitions.

## 7. Important Parameters

Not applicable — this package has no nodes and no runtime parameters.

## 8. Launch Files and How to Run Them

None. `forklift_interfaces` is a build-time/interface dependency only;
there is nothing to launch. To inspect a generated interface:

```bash
ros2 interface show forklift_interfaces/action/ExecuteTask
ros2 interface show forklift_interfaces/srv/ReserveEdge
ros2 interface list | grep forklift_interfaces
```

## 9. Dependencies

From `package.xml`:

- `ament_cmake` (buildtool), `rosidl_default_generators` (buildtool),
  `rosidl_default_runtime` (exec)
- `geometry_msgs`, `std_msgs`, `builtin_interfaces`
- `member_of_group: rosidl_interface_packages`

No dependency on any other `forklift_*` package — by design, this is a
leaf/foundation package that everything else depends on, never the
reverse.

## 10. Integration with the Rest of the System

Every other package in this workspace except `forklift_description` and
`forklift_remote` (which have no custom interfaces of their own) depends
on `forklift_interfaces` for at least one message, service, or action —
see the per-package groupings in §5. Because it has no runtime behavior
itself, its only "integration" surface is the interface definitions
themselves; changing a field here is a breaking change for every consumer
listed above.

## 11. Known Issues / TODO / Future Work

- **`DockRobot.action`/`UndockRobot.action` are unused** — `forklift_docking`
  and `forklift_task_manager` both use `opennav_docking_msgs`' actions of
  the same name instead. These two files are dead weight; either delete
  them or clearly mark them deprecated so nobody starts building against
  them expecting them to be the docking interface actually in use.
- **`StartDocking.srv` is unused anywhere in the workspace** — no server
  advertises it and no client calls it. Candidate for removal, or
  documentation of what it was originally meant to replace/support.
- **`SetLiftHeight.srv` vs. `SetLiftHeight.action` duplication** — the
  action is the interface `forklift_controller` actually serves and that
  `forklift_docking`/`forklift_task_manager` actually call; the service
  form is only referenced by `forklift_docking`'s superseded
  `pallet_dock_node.py` script (not wired into any launch file). Worth
  removing the service form once that script is retired.
- **Ten "early/ungrouped" messages appear dead**: `Task`, `TaskStatus`,
  `TaskResult`, `NavigationRequest`, `NavigationResult`, `DockingRequest`,
  `DockingResult`, `LiftRequest`, `LiftResult` — no publisher/subscriber
  found anywhere in the other 16 packages. They likely predate the
  current action-based task/dock/lift interfaces and are candidates for
  removal once confirmed unused by any external/undiscovered consumer.
- No versioning/changelog convention exists for interface changes across
  the ~15 consuming packages beyond the per-group comments in
  `CMakeLists.txt` — a breaking field rename here silently breaks every
  consumer at build time with no deprecation window.
