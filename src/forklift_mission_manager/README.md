# forklift_mission_manager

ROS 2 (Humble) `ament_cmake` package implementing the mission-level layer
for the Mini Forklift AMR: it combines multiple `forklift_task_manager`
tasks into one higher-level mission, e.g. **transport = navigate → pick
pallet → navigate → drop pallet**.

---

## 1. Overview and Responsibility

`forklift_mission_manager` sits one layer above `forklift_task_manager`. It
accepts a single `ExecuteMission` goal describing a mission (pallet
transport, patrol, charging, or a station visit), expands it into an
ordered list of `forklift_task_manager` `TaskSpec`s, and executes them
**sequentially by calling `forklift_task_manager`'s `execute_task` action
one task at a time**, stopping the whole mission on the first task failure.

It deliberately contains **no** navigation, docking, or lift logic itself,
and never talks to Nav2, `docking_server`, or `lift_controller_node`
directly — that separation of concerns belongs entirely to
`forklift_task_manager`. This is a cleaner layering than
`forklift_task_manager` itself has with `forklift_docking` (see that
package's README, §2/§12) — `MissionExecutor` is explicitly documented
in-source as "never talks to nav2/docking_server/lift_controller_node
directly; that's exactly the layering `forklift_task_manager` exists to
own."

## 2. Architecture / Role in the Overall System

```
                    execute_mission (action, ExecuteMission)
                              │
                              ▼
                   +----------------------+
                   |    MissionServer      |   (single mission at a time,
                   |  (1 action server,    |    no queue at this layer --
                   |   no queue)           |    a 2nd goal while busy is
                   +----------┬-----------+    REJECTED, not queued)
                              │ expandMissionToTasks() -- Mission -> TaskSpec[]
                              ▼
                   +----------------------+
                   |   MissionExecutor      |
                   |  (sends TaskSpecs to    |
                   |   task_manager ONE AT   |
                   |   A TIME, waits for     |
                   |   each result)          |
                   +----------┬-----------+
                              │ execute_task (action, per TaskSpec)
                              ▼
                   forklift_task_manager (NAVIGATE/DOCK/LIFT/UNDOCK/... steps)
```

## 3. Directory Structure

```
forklift_mission_manager/
├── config/
│   └── missions.yaml                 # named, reusable mission templates (optional convenience layer)
├── include/forklift_mission_manager/
│   ├── mission.hpp                   # Mission/TaskSpec/MissionType + expandMissionToTasks()
│   ├── mission_state_machine.hpp     # MissionState enum + small state wrapper
│   ├── mission_database.hpp          # MissionTemplate/MissionDatabase (loads missions.yaml)
│   ├── mission_executor.hpp          # sequential ExecuteTask client
│   └── mission_server.hpp            # ExecuteMission action server
├── src/
│   ├── mission.cpp                   # the 4 mission-type "recipes"
│   ├── mission_state_machine.cpp
│   ├── mission_database.cpp
│   ├── mission_executor.cpp
│   └── mission_server.cpp
├── launch/
│   └── mission_manager.launch.py
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Responsibility |
|---|---|---|
| `MissionServer` | `mission_server` | `rclcpp_components` node (no `main.cpp`). Hosts the `execute_mission` action server. Runs **one mission at a time** — unlike `forklift_task_manager`'s explicit `TaskQueue`, there is no queueing at this layer; a second goal received while one is executing is rejected outright, and the caller (`forklift_web_bridge`) is expected to wait or retry. |

## 5. Input / Output Topics

None. All communication is action-based: `execute_mission` (server) and
`execute_task` (client of `forklift_task_manager`).

## 6. Services and Actions

| Name | Type | Description |
|---|---|---|
| `execute_mission` | `forklift_interfaces/action/ExecuteMission` | **Primary interface.** Goal: `mission_id`, `mission_type` (`PALLET_TRANSPORT`/`PATROL`/`CHARGING`/`STATION_OPERATION`), plus type-specific fields (`pick_area_pose`, `pick_dock_id`, `drop_area_pose`, `drop_dock_id`, `waypoints[]`, `dock_id`, `station_wait_duration`). Feedback: `mission_id`, `current_task`, `current_task_index`, `total_tasks`, `progress`, `state`, `message`. Result: `success`, `message`, `failed_task`. |

### Mission types and their task recipes (`expandMissionToTasks` in `mission.cpp`)

| Mission type | Required fields | Task sequence sent to `forklift_task_manager` |
|---|---|---|
| `PALLET_TRANSPORT` | `pick_dock_id`, `drop_dock_id` | `[SINGLE_NAVIGATION(pick_area_pose)]` (only if `pick_area_pose.header.frame_id` is non-empty) → `PICK_PALLET_DOCK(pick_dock_id)` → `[SINGLE_NAVIGATION(drop_area_pose)]` (optional, same rule) → `DROP_PALLET_DOCK(drop_dock_id)` |
| `PATROL` | `waypoints[]` (≥1) | Single `PATROL_POINT(waypoints[])` task — `forklift_task_manager` already iterates multiple waypoints internally, so this is not duplicated here |
| `CHARGING` | `dock_id` | Single `CHARGING_DOCK(dock_id)` task |
| `STATION_OPERATION` | `dock_id` | Single `STATION_DOCK(dock_id, station_wait_duration)` task |

The optional `pick_area_pose`/`drop_area_pose` let a `PALLET_TRANSPORT`
mission route through a coarser waypoint (e.g. a specific aisle) before the
dock's own precise staging navigation — `PICK_PALLET_DOCK`/
`DROP_PALLET_DOCK` already navigate to their own staging pose internally,
so these area poses are purely an optional extra leg, skipped entirely
when their `PoseStamped.header.frame_id` is left empty.

## 7. Important Parameters

| Parameter | Default | Description |
|---|---|---|
| `mission_database` | `""` (launch overrides to `config/missions.yaml`) | Path to named mission templates. **Loading is non-fatal if missing** — an empty/absent file just means no named templates are available and every mission must be sent fully-specified. |
| `execute_task_action_name` | `/execute_task` (launch default `execute_task`) | `forklift_task_manager`'s action name this node calls as a client |

### `config/missions.yaml` (named templates)

Example entries: `transport_pick1_to_drop1` … `transport_pick4_to_drop4`
(`PALLET_TRANSPORT`, one per pick/drop dock pair), `morning_charge`
(`CHARGING`, `charging_dock_1`), `patrol_main_aisle` (`PATROL`, 3
waypoints), `station_1_operation` (`STATION_OPERATION`,
`station_wait_duration: 8.0`).

**Not yet wired up:** `MissionDatabase::getTemplate()` is implemented and
`missions.yaml` is loaded at startup, but `MissionServer` never calls
`getTemplate()` — there is currently no way to trigger a mission by
`mission_id` lookup alone. Every `ExecuteMission` goal must be sent fully
specified; `missions.yaml`'s named templates are inert until this wiring
(explicitly flagged in-source as intended "once this is wired into
`forklift_web_bridge`") is completed. Mark as **Planned / Future**.

## 8. Launch Files and Run Commands

| Launch file | Starts |
|---|---|
| `mission_manager.launch.py` | `mission_server` only. Does not start `forklift_task_manager`, `docking_server`, Nav2, or `lift_controller_node` — this node is purely a client of `forklift_task_manager`. |

```bash
ros2 launch forklift_mission_manager mission_manager.launch.py

# Transport a pallet from pick dock 1 to drop dock 1
ros2 action send_goal /execute_mission forklift_interfaces/action/ExecuteMission \
  "{mission_id: 'transport1', mission_type: 'PALLET_TRANSPORT', pick_dock_id: 'pallet_pick_dock_1', drop_dock_id: 'pallet_drop_dock_1'}"

# Send the robot to charge
ros2 action send_goal /execute_mission forklift_interfaces/action/ExecuteMission \
  "{mission_id: 'charge1', mission_type: 'CHARGING', dock_id: 'charging_dock_1'}"
```

## 9. Dependencies

From `package.xml`:

- `rclcpp`, `rclcpp_action`, `rclcpp_components`
- `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `forklift_interfaces` (`ExecuteMission.action`)
- `yaml_cpp_vendor` (+ `yaml-cpp` linked directly, for `missions.yaml` parsing)
- `forklift_task_manager` (exec dependency — the system this node
  orchestrates)

## 10. Communication Flow

1. A client (typically `forklift_web_bridge`) sends an `execute_mission`
   goal.
2. `handleGoal()` rejects immediately if another mission is already
   executing (`busy_` flag — no queueing at this layer), if
   `MissionDatabase::validateMission()` finds a structural problem (unknown
   `mission_type` or a required top-level field missing for that type —
   note: this does **not** check that a referenced `dock_id` actually
   exists in `forklift_docking`'s dock database; that check happens later,
   inside `forklift_task_manager`), or if `expandMissionToTasks()` fails.
3. `handleAccepted()` runs the mission on a detached thread:
   `expandMissionToTasks()` builds the `TaskSpec[]` list, then
   `MissionExecutor::run()` sends each spec to `forklift_task_manager` via
   the `execute_task` action, **waiting for each task's result before
   sending the next** (polling the future, not
   `spin_until_future_complete`, since the node is already being spun
   elsewhere).
4. `ExecuteTask`'s own feedback is relayed upward as `ExecuteMission`
   feedback, prefixed with mission-level task-index/progress context.
5. The mission stops at the first task failure (or cancellation) and
   returns `success`/`message`/`failed_task` via the action result.

## 11. Integration with Other Packages

- **forklift_task_manager** — the sole downstream dependency; every
  mission is realized purely as a sequence of `execute_task` action calls.
  `forklift_mission_manager` has no direct knowledge of navigation,
  docking, or lift interfaces.
- **forklift_web_bridge** — the intended primary client, exposing
  mission triggering (and, once wired, named-template lookup) to the web
  dashboard.

## 12. Future Improvements

- **Planned / Future:** wire `MissionDatabase::getTemplate()` into
  `MissionServer`, so a caller can trigger a named mission from
  `missions.yaml` by `mission_id` alone instead of resending full
  parameters every time — currently dead code.
- **Planned / Future:** add mission queueing (or explicit multi-mission
  support), matching `forklift_task_manager`'s task-queue pattern, instead
  of hard-rejecting a second `execute_mission` goal while one is running.
- Extend `MissionDatabase::validateMission()` (or the underlying
  `forklift_task_manager` validation it relies on) to surface an unknown
  `dock_id` earlier, at mission-accept time, rather than only failing once
  the corresponding task actually starts executing.
- Add a mission-level retry/recovery policy distinct from
  `forklift_task_manager`'s per-step retries, e.g. re-attempting a failed
  `DROP_PALLET_DOCK` task at a different drop dock.
