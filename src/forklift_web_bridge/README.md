# forklift_web_bridge

ROS 2 (Humble) `ament_python` package exposing the Mini Forklift AMR to a
web dashboard: a REST API (FastAPI), a WebSocket bridge (`rosbridge_server`,
for `roslibjs`), camera streaming (`web_video_server`), and a set of thin
translation nodes that convert ROS actions/topics into simple JSON
commands/status and back.

---

## 1. Overview and Responsibility

`forklift_web_bridge` is purely a **translation and aggregation layer** — it
contains no navigation, docking, task, or mission logic of its own. Every
bridge node's docstring is explicit about this: they forward commands to
the real action servers (`forklift_navigation`/Nav2, `forklift_mission_manager`,
`forklift_task_manager`) and relay their feedback/results back as
JSON, without re-implementing any sequencing or decision logic.

Two independent web-facing surfaces are provided over the same underlying
command/status topics:

- **WebSocket** (`rosbridge_server`, port 9090) — for a `roslibjs`
  dashboard subscribing/publishing directly to `web_bridge/*` topics.
- **REST** (`fastapi_bridge_node`, port 8000) — a thin HTTP facade over the
  exact same `web_bridge/*` topics, for clients that prefer plain HTTP/JSON
  (e.g. a Next.js dashboard's server-side fetches).

Both surfaces ultimately drive the same `navigation_bridge_node` /
`mission_bridge_node`, so REST and WebSocket clients never bypass each
other or duplicate business logic.

## 2. Architecture / Role in the Overall System

```
                Next.js web_dashboard (or any roslibjs/HTTP client)
                          │                        │
                 REST (HTTP :8000)        WebSocket (rosbridge :9090)
                          │                        │
                +---------▼------------------------▼---------+
                |                                              |
                |  fastapi_bridge_node        rosbridge_server |
                |  (caches status, publishes                   |
                |   web_bridge/nav_command,                    |
                |   web_bridge/mission_command)                |
                +---------┬------------------------------------+
                          │  web_bridge/* topics (std_msgs/String, JSON)
        ┌─────────────────┼─────────────────┬───────────────────┬───────────┐
        ▼                 ▼                 ▼                   ▼           ▼
 navigation_bridge   mission_bridge   robot_state_node   diagnostics_bridge  bridge_manager
 _node                _node                                _node             _node
        │                 │                 │                   │           │
   navigate_to_pose  execute_mission     /odom,             /diagnostics   (readiness
   (Nav2 action)     (forklift_          /battery_state     (forklift_      probe over
                      mission_manager                        diagnostics)   the ROS graph)
                      action)

   web_video_server (port 8080) ── streams camera topics (e.g. from forklift_aruco/forklift_gazebo) directly
```

## 3. Directory Structure

```
forklift_web_bridge/
├── config/
│   ├── web_bridge.yaml     # parameters for all launched nodes -- the actual config in use
│   ├── bridge.yaml         # (unused by the launch file — see §12)
│   ├── fastapi.yaml        # legacy/unused config; param names don't match fastapi_bridge_node.py's actual parameters
│   ├── mqtt.yaml           # config for mqtt_bridge_node.py (not launched by any launch file)
│   └── robot.yaml
├── forklift_web_bridge/
│   ├── fastapi_bridge_node.py       # REST API (FastAPI/uvicorn), port 8000
│   ├── navigation_bridge_node.py    # web_bridge/nav_command <-> Nav2's navigate_to_pose
│   ├── mission_bridge_node.py       # web_bridge/mission_command <-> forklift_mission_manager's execute_mission
│   ├── robot_state_node.py          # aggregates /odom + /battery_state -> web_bridge/robot_state
│   ├── diagnostics_bridge_node.py   # simplifies /diagnostics -> web_bridge/diagnostics_summary
│   ├── bridge_manager_node.py       # readiness probe -> web_bridge/status
│   ├── mqtt_bridge_node.py          # optional ROS<->MQTT bridge (not launched by default — see §12)
│   └── utils.py
├── launch/
│   ├── forklift_web_bridge.launch.py  # the active, complete launch file
│   └── forklift_web_server.launch.py  # legacy/incomplete — see §12
├── setup.py / setup.cfg
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Responsibility |
|---|---|---|
| `FastApiBridgeNode` | `fastapi_bridge_node` | Runs a FastAPI app (`uvicorn`) on a background-spun `rclpy` node. GET endpoints serve the latest cached status from `web_bridge/*` status topics; POST endpoints publish onto `web_bridge/nav_command`/`web_bridge/mission_command`. No action-calling logic here — it only republishes onto the same topics `navigation_bridge_node`/`mission_bridge_node` already consume. |
| `NavigationBridgeNode` | `navigation_bridge_node` | Translates simple JSON navigate/cancel commands into a Nav2 `NavigateToPose` action call, relaying feedback (`distance_remaining`) and result as JSON status. Multi-waypoint patrol is explicitly **not** handled here — routed through `mission_bridge_node` instead, to avoid duplicating waypoint sequencing. |
| `MissionBridgeNode` | `mission_bridge_node` | Translates JSON execute/cancel/query commands into `forklift_mission_manager`'s `ExecuteMission` action, relaying feedback/result as JSON status. Also subscribes to `forklift_task_manager`'s `task_status` topic to add the per-step `task` detail object to `mission_status` (§6.1). |
| `RobotStateNode` | `robot_state_node` | Aggregates `/odom` (pose + velocity) and `/battery_state` into one JSON message at a fixed rate — pure aggregation, no interpretation. |
| `DiagnosticsBridgeNode` | `diagnostics_bridge_node` | Subscribes to `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, from `forklift_diagnostics`) and republishes a simplified JSON summary (overall level + per-component list). |
| `BridgeManagerNode` | `bridge_manager_node` | Periodically checks whether the ROS graph endpoints this bridge depends on are actually present (Nav2's `navigate_to_pose`, `execute_mission`, `execute_task` action servers, `/diagnostics`, `/odom` topics) and publishes one readiness summary. Does **not** manage other nodes' process lifecycle — only graph connectivity monitoring. |
| `MqttBridgeNode` | `mqtt_bridge_node` | Optional ROS↔MQTT bridge (`paho-mqtt`) publishing robot status/navigation/task/battery/diagnostics to `factory/<robot_id>/*` MQTT topics and subscribing to command/mission topics. **Not started by either launch file** — must be run manually if MQTT integration is needed. |

## 5. REST API (`fastapi_bridge_node`, port 8000)

| Method | Path | Body | Description |
|---|---|---|---|
| GET | `/health` | — | `{"status": "ok"}` |
| GET | `/robot/state` | — | Latest cached `web_bridge/robot_state` (pose, velocity, battery) |
| GET | `/navigation/status` | — | Latest cached `web_bridge/navigation_status` |
| POST | `/navigation/goal` | `{"x": float, "y": float, "yaw": float=0.0, "frame_id": str="map"}` | Publishes a `navigate` command |
| POST | `/navigation/cancel` | — | Publishes a `cancel` navigation command |
| GET | `/mission/status` | — | Latest cached `web_bridge/mission_status` |
| POST | `/mission/execute` | `MissionRequest` (`mission_id`, `mission_type`, `pick_dock_id`, `drop_dock_id`, `dock_id`, `station_wait_duration`, `waypoints`, `pick_area_pose`, `drop_area_pose`) | Publishes an `execute` mission command |
| POST | `/mission/cancel` | — | Publishes a `cancel` mission command |
| GET | `/task/status` | — | Progress of the task `forklift_task_manager` is running now (the same object `mission_status.task` carries) |
| GET | `/diagnostics/summary` | — | Latest cached `web_bridge/diagnostics_summary` |
| GET | `/bridge/status` | — | Latest cached `web_bridge/status` (readiness) |
| GET | `/routes/graph` | — | The route graph GeoJSON route_server is configured with, returned as-is (FeatureCollection of `Point` nodes `{id, frame}` + `MultiLineString` edges `{id, startid, endid, cost, overridable}`) |
| GET | `/missions/templates` | — | `{"missions": [{name, mission_type, ...type-specific fields}]}` from `missions.yaml` |
| POST | `/missions/templates` | `{name, mission_type, ...type-specific fields}` | Creates one named template (create-only), responds `201` with the saved entry |
| GET | `/docks` | — | Every dock in `dock_database.yaml`, in file order: `[{name, type, frame, pose: [x, y, yaw], id, lift}]` |
| POST | `/docks` | `{name?, type, frame="map", pose: [x, y, yaw], id?, lift?}` | Creates one dock (create-only), reloads docking_server(s), responds `201` with the saved entry |
| POST | `/routes/graph` (`?force=true` optional) | Full graph GeoJSON (same shape as GET) | Validates, writes the file, and reloads it in route_server via `route_server/set_route_graph` |
| PUT | `/virtual-layer/shapes` | `{revision, shapes: [...]}` — the **full** set of virtual walls/zones | Writes every robot's `nav2_virtual_layer` forms file and reloads it in each running `global_costmap` |
| GET | `/virtual-layer/status` | — | Per robot: online, file, revision on disk, number of forms, last sync result |

Example `curl` commands:

```bash
curl http://localhost:8000/health

curl http://localhost:8000/robot/state

curl -X POST http://localhost:8000/navigation/goal \
  -H "Content-Type: application/json" \
  -d '{"x": 1.5, "y": -0.5, "yaw": 0.0}'

curl -X POST http://localhost:8000/mission/execute \
  -H "Content-Type: application/json" \
  -d '{"mission_type": "PALLET_TRANSPORT", "pick_dock_id": "pallet_pick_dock_1", "drop_dock_id": "pallet_drop_dock_1"}'

curl http://localhost:8000/diagnostics/summary
```

### Route graph (`/routes/graph`)

Unlike the other endpoints this one works on a file, not a topic:

- **File path**: read from route_server's own `graph_filepath` parameter
  (`route_server/get_parameters`), so it is always the graph route_server
  actually loaded. Set `route_graph_file` in `web_bridge.yaml` to override.
  fastapi_bridge_node must run on the same machine as route_server.
- **GET** returns the file contents unchanged. `503` if route_server can't
  be reached to learn the path, `404` if the file is missing.
- **POST** checks the graph first (`400` with a list of `errors`: integer
  and unique node/edge ids, `[x, y]` node coordinates, and every edge's
  `startid`/`endid` pointing at an existing node). It then writes the
  file atomically and calls `route_server/set_route_graph`
  (`nav2_msgs/srv/SetRouteGraph`), so **no route_server restart is needed**.
  On success the previous file is kept as `<file>.bak`. If route_server
  rejects the graph, the old file is written back and reloaded, and the
  response is `502` with `previous_graph_restored`.
- POST is refused with `409` while `web_bridge/navigation_status` reports
  `goal_active` (route_server drops its graph while loading the new one).
  Cancel first, or pass `?force=true`.
- `graph_filepath` points into `install/.../share`. With
  `colcon build --symlink-install` (this workspace), that path is a symlink
  to `src/`. The write follows the link, so edits land in `src/` and survive
  rebuilds. Without `--symlink-install`, the next build overwrites the
  edited copy.
- Multi-robot: each robot's fastapi_bridge_node reloads only **its own**
  route_server. If robots share one graph file, POST to each robot's REST
  port, or all robots except that one keep the old graph in memory.

```bash
curl http://localhost:8000/routes/graph -o graph.geojson
curl -X POST http://localhost:8000/routes/graph \
  -H "Content-Type: application/json" --data @graph.geojson
```

Live route state (`/route_graph` MarkerArray, `/plan`, `/local_plan`) is
unchanged and still comes over rosbridge.

### Dock database (`/docks`)

- **File path**: docking_server's own `dock_database` parameter
  (`docking_server/get_parameters`). If docking_server isn't running,
  `forklift_docking`'s installed `config/dock_database.yaml` is used, so
  GET still works. Set `dock_database_file` to override.
- **GET** returns `[{name, type, frame, pose, id, lift}]`. `id` is the
  ArUco marker id as an int (stored as a quoted string in the file), or
  `null` when the dock has none (e.g. the drop docks). `lift` is in metres.
- **POST** body: `type` must be one of `charging_dock`, `station_dock`,
  `pallet_pick_dock` or `pallet_drop_dock`. `pose` is `[x, y, yaw]`.
  `name` is optional: if it's left out, the next free `<type>_<n>` is used
  (max existing n + 1). `id` is accepted for any type, because charging
  and station docks carry marker ids too. `lift` defaults to `0.0`.
  - `400` if the name already exists (create-only), or if a field is invalid.
  - The entry is **appended as text** under `docks:`, so existing comments
    and alignment are never reformatted. The result is re-parsed before
    writing, and the file is left untouched unless it comes out as exactly
    the old docks plus the new one.
  - `201` returns the saved entry plus `docking_server_reload`:
    `{"/forklift1/docking_server/reload_database": true, ...}`
    (`false` = rejected, `null` = no answer).
- **Who needs a reload**: `opennav_docking`'s docking_server caches the
  file when it activates, so POST calls `reload_database`
  (`opennav_docking_msgs/srv/ReloadDatabase`) on **every** docking_server
  on the graph. Docks are shared by the whole fleet, and every robot uses
  the same file. `forklift_task_manager` and `pallet_dock_workflow_node`
  re-read the file for each task/goal, so they need nothing.
- Symlinked paths (`colcon build --symlink-install`) are resolved before
  writing, so the edit lands in `src/` and survives rebuilds.

```bash
curl http://localhost:8000/docks
curl -X POST http://localhost:8000/docks -H "Content-Type: application/json" \
  -d '{"type": "charging_dock", "pose": [-1.5, -3.0, 3.14], "id": 6}'
```

### Mission templates (`/missions/templates`)

Named, reusable missions in `forklift_mission_manager/config/missions.yaml`.

- **File path**: mission_server's `mission_database` parameter, falling
  back to forklift_mission_manager's installed `config/missions.yaml`.
  Override with `mission_database_file`.
- **GET** returns `{"missions": [...]}`, in file order. The file's `type:`
  key is returned as `mission_type` (as in `ExecuteMission.action`), and
  only the fields each entry actually has are included.
- **POST** body: `name` (required) plus `mission_type` and its fields:

  | `mission_type` | Required | Also accepted |
  |---|---|---|
  | `PALLET_TRANSPORT` | `pick_dock_id`, `drop_dock_id` | `pick_area_pose`, `drop_area_pose` (`[x, y, yaw]`) |
  | `CHARGING` | `dock_id` | — |
  | `STATION_OPERATION` | `dock_id` | `station_wait_duration` (seconds) |
  | `PATROL` | `waypoints` (`[[x, y, yaw], ...]`) | — |

  - `400` if `name` is blank, already exists (create-only), holds anything
    but letters/digits/`_`, if a required field is missing, or if a
    referenced dock id isn't in `dock_database.yaml`.
  - Fields that don't belong to the mission type are dropped, so stale
    dashboard form state can't write e.g. a `CHARGING` entry with waypoints.
  - Appended as text and verified the same way as `/docks`, so comments
    and formatting in `missions.yaml` are preserved.
  - `201` returns the saved entry.
- **No reload needed.** mission_server reads `missions.yaml` only at
  startup, and nothing looks the templates up when a mission runs — every
  `ExecuteMission` goal carries its own parameters (the web bridge builds
  the goal from the dashboard's fields). A new template is usable as soon
  as it is written. If named-template lookup is ever wired into
  mission_server, this endpoint will need a reload call like `/docks` has.

```bash
curl http://localhost:8000/missions/templates
curl -X POST http://localhost:8000/missions/templates \
  -H "Content-Type: application/json" \
  -d '{"name": "transport_pick2_to_drop3", "mission_type": "PALLET_TRANSPORT",
       "pick_dock_id": "pallet_pick_dock_2", "drop_dock_id": "pallet_drop_dock_3"}'
```

### Virtual walls and zones (`/virtual-layer/*`)

Dashboard virtual walls and zones become costs in each robot's
`global_costmap` through `nav2_virtual_layer` (`src/nav2-virtual-layer`).
The planner (NavFn, including the BT's last-mile `ComputePathToPose`)
plans around them. `route_server` does not: it routes over graph edges,
which ignore the costmap unless a `CostmapScorer` is configured. This
endpoint only reloads `global_costmap`; a `local_costmap` virtual_layer
(controller YAMLs) is not reloaded by it.

- **Who owns the data**: the backend (MongoDB). It sends the **complete**
  list on every create/update/delete, never a single change, so repeating
  a request is harmless and a robot that missed one catches up on the next.
- **Files**: one per robot, `forklift_navigation/config/virtual_layer/<robot>.yaml`
  (the `forms_file` in `nav2_params.yaml` / `nav2_multirobot_params.yaml`).
  A running robot's own `global_costmap` is asked for
  `virtual_layer.forms_file` first; set `virtual_layer_dir` to override.
  With `--symlink-install` the write lands in `src/`.
- **Robots**: `virtual_layer_robots` (default `forklift1..3`) always get a
  file, even while offline, and Nav2 loads it at startup. Running robots are
  also found by their `<ns>/global_costmap/virtual_layer/reload_shapes`
  service; a namespace-less one (single-robot bringup) is
  `virtual_layer_default_robot`.
- **PUT body**: `{"revision": int, "shapes": [shape, ...]}`, coordinates in
  metres in the `map` frame (the plugin ignores any other frame).

  | Field | Wall | Zone |
  |---|---|---|
  | `id` | backend id (unique) | same |
  | `kind` | `"wall"` | `"zone"` |
  | `geometry` | `{"type": "polyline", "points": [[x, y], ...]}` (≥ 2 points) | `{"type": "polygon", "points": [...]}` (≥ 3) or `{"type": "circle", "center": [x, y], "radius": r}` |
  | `thickness` | full width, 0.1–5.0 m | — |
  | `zone_type` | — | `"keepout"` (cost 254, lethal) or `"avoid"` |
  | `cost` | — | `avoid` only: 1–252 (higher = avoided harder) |
  | `robots` | `[]` = all robots, else names | same |
  | `enabled` | `false` = left out of the files | same |

  - `400` with `{"errors": [...]}` if a shape is invalid; nothing is written.
  - Walls are written as one filled rectangle per segment plus a circle at
    each inner corner, and polygon zones as filled `POLYGON((), ...)`. The
    plugin's own `add_polygon` service only draws an open outline, and its
    `LINESTRING` thickness is unreliable, so neither is used.
  - Response: `{"revision", "robots": {name: {"status", "file", "forms",
    "detail"}}, "unknown_robots": [...]}`. `status` is `ok` (reloaded),
    `offline` (file written, loaded when Nav2 starts), `rejected` (e.g.
    the file didn't exist when Nav2 started, so it can't reload until a
    restart), `no_answer`, or `write_failed`. `unknown_robots` lists names in
    `robots` fields that match no robot, so those shapes are applied nowhere.
- **Expiry**: handle it in the backend (send a new set without the shape).
  The plugin's own `DURATION` restarts on every reload.
- **GET `/virtual-layer/status`**: `{"robots": {name: {"online", "file",
  "exists", "revision", "forms", "last_sync"}}}`. Compare `revision` with
  the backend's to find a robot that needs the set sent again.

```bash
curl -X PUT http://localhost:8000/virtual-layer/shapes -H "Content-Type: application/json" -d '{
  "revision": 1,
  "shapes": [
    {"id": "w1", "kind": "wall", "geometry": {"type": "polyline", "points": [[1.2, -3.0], [4.5, -3.0]]},
     "thickness": 0.1},
    {"id": "z1", "kind": "zone", "zone_type": "keepout", "robots": ["forklift2"],
     "geometry": {"type": "polygon", "points": [[0.5, -2.5], [1.5, -2.5], [1.5, -1.5], [0.5, -1.5]]}}
  ]}'
curl http://localhost:8000/virtual-layer/status
```

**Note:** all navigation/mission POST endpoints respond `{"accepted": true}` immediately —
this only confirms the command was *published*, not that the underlying
Nav2/mission action was accepted or completed. Poll the corresponding GET
status endpoint (or subscribe to the WebSocket status topic) for the
actual outcome.

## 6. ROS Topics (WebSocket / internal command-status protocol)

| Name | Type | Direction | Description |
|---|---|---|---|
| `web_bridge/nav_command` | `std_msgs/String` (JSON) | Input to `navigation_bridge_node` | `{"command": "navigate", "x", "y", "yaw", "frame_id"}` or `{"command": "cancel"}` |
| `web_bridge/navigation_status` | `std_msgs/String` (JSON) | Output | `{"state": "IDLE"\|"EXECUTING"\|"SUCCEEDED"\|"FAILED"\|"CANCELED", "message", "goal_active"}` |
| `web_bridge/mission_command` | `std_msgs/String` (JSON) | Input to `mission_bridge_node` | `{"command": "execute", "mission_type", ...}` / `{"command": "cancel"}` / `{"command": "query"}` |
| `web_bridge/mission_status` | `std_msgs/String` (JSON) | Output | Mirrors `ExecuteMission`'s feedback/result fields, plus a `task` sub-object with per-step detail (see §6.1) |
| `web_bridge/task_status` | `std_msgs/String` (JSON) | Output | The `task` object on its own, one message per step transition (transient local), for task progress without the mission wrapper. Served by REST at `GET /task/status` |
| `web_bridge/robot_state` | `std_msgs/String` (JSON) | Output | `{"pose", "velocity", "battery", "timestamp"}` |
| `web_bridge/diagnostics_summary` | `std_msgs/String` (JSON) | Output | `{"overall_level", "components": [...], "timestamp"}` |
| `web_bridge/status` | `std_msgs/String` (JSON) | Output | `{"bridges_ready", "checks": {...}, "timestamp"}` |
| `/odom` | `nav_msgs/Odometry` | Input | Consumed by `robot_state_node` |
| `/battery_state` | `sensor_msgs/BatteryState` | Input | Consumed by `robot_state_node` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Input | Consumed by `diagnostics_bridge_node`, produced by `forklift_diagnostics` |

### 6.1 Step detail in `mission_status` (`task` object)

`ExecuteMission` feedback only names the running task (`"STATION_DOCK"`),
so a DOCK step could stall for 30s and fail with no explanation reaching
the dashboard. `forklift_task_manager` therefore publishes every step and
sub-step transition on **`task_status`**
(`forklift_interfaces/TaskStepStatus`, transient local, namespaced per
robot), and `mission_bridge_node` merges the latest one into every
`mission_status` message as a `task` object:

```json
"task": {
  "task_id": "STATION_DOCK_416688000000",
  "task_type": "STATION_DOCK",        // PICK_PALLET_DOCK | DROP_PALLET_DOCK |
                                      // CHARGING_DOCK | STATION_DOCK | ...
  "step": "DETECT",                   // NAVIGATE | DOCK | STAGING | DETECT | APPROACH |
                                      // CONTACT | WAIT_FOR_CHARGE | CHARGING | LIFT |
                                      // LOWER | UNDOCK | UNDOCK_STAGING |
                                      // MONITOR_CHARGING | STATION_OPERATION | TASK
  "state": "RETRY",                   // RUNNING | SUCCESS | FAILED | RETRY | CANCELED
  "step_index": 1, "total_steps": 3, "progress": 33.3,
  "attempt": 3, "max_attempts": 3,
  "dock_id": "station_dock_1",
  "lift_height": 0.08,                // metres, for LIFT / LOWER
  "detected": false,                  // dock detected during this task
  "charging": false,                  // robot reported charging on the dock
  "success": false,                   // true on step "TASK", state "SUCCESS"
  "error_code": "DOCK_FAILED",
  "message": "DockRobot did not succeed",
  "events": [ {"stamp": ..., "step": "DETECT", "state": "RETRY",
               "message": "Retrying dock detection (retry 2)"} ]
}
```

Where the detail comes from: `task_manager_node` publishes the task start
(`step: "TASK"`), each step's `RUNNING`/`RETRY`/`SUCCESS`/`FAILED`
transition, and the task-level finish. The docking sub-steps come from
opennav_docking's own `DockRobot` **action feedback**, which only
`task_executor` (its action client) receives, and which it forwards:
`NAV_TO_STAGING_POSE -> STAGING`, `INITIAL_PERCEPTION -> DETECT`,
`CONTROLLING -> APPROACH` (which also marks `detected: true`),
`WAIT_FOR_CHARGE`, and `RETRY`. Lift height, undock and the charging wait
are reported by the executor's own steps. The phases docking_server only
writes to its log -- detection lost/succeeded, contact, charging started,
undock staging reached, undocked -- are translated into the same stream
by task_manager (see that package's README §5.1), so the web bridge
itself stays a plain topic subscriber.

- `events` keeps the last `task_event_history` transitions (default 10)
  for the running task, and clears when a new task or mission starts.
- Republishing for step-only changes is throttled to
  `task_status_min_period_sec` (default 0.2s). Mission feedback and
  results publish immediately.
- `task_status` is transient local, so a dashboard connecting mid-task
  sees the current step at once.

#### Showing task progress

The same object is published on its own as **`web_bridge/task_status`**
(JSON, transient local) and served at **`GET /task/status`**, so a
progress widget doesn't have to dig into `mission_status`:

```js
new ROSLIB.Topic({ros, name: '/forklift1/web_bridge/task_status',
                  messageType: 'std_msgs/String'})
  .subscribe(m => { const t = JSON.parse(m.data);
                    setBar(t.progress); setCaption(t.label); });
```
```bash
curl http://<ROBOT_IP>:8000/task/status
# {"task_type": "PICK_PALLET_DOCK", "step": "LIFT", "state": "RUNNING",
#  "progress": 50.0, "label": "Lifting fork to 0.08 m", "detected": true, ...}
```

Two fields exist purely for that widget:

- **`progress`** -- 0-100 across the task's steps. Only real steps
  (`NAVIGATE`, `DOCK`, `LIFT`, `LOWER`, `UNDOCK`, `MONITOR_CHARGING`,
  `STATION_OPERATION`) advance it; docking sub-steps report from *inside*
  the running step, so they never move the bar backwards. It is monotonic
  within a task and reads 100 when the task completes. (`step_progress`
  keeps task_manager's own raw figure, which counts steps *started*.)
- **`label`** -- a ready-to-display line: "Navigating", "Detecting dock -
  retry 2/3", "Lifting fork to 0.08 m", "Task completed".

A worked example, from a real `PICK_PALLET_DOCK` run:

```
  0%  TASK     RUNNING   Task started
  0%  NAVIGATE RUNNING   Navigating
 25%  NAVIGATE SUCCESS   Navigating
 25%  DETECT   RETRY     Detecting dock - retry 1/3
 25%  CONTACT  SUCCESS   Docked
 50%  DOCK     SUCCESS   Docking
 75%  LIFT     SUCCESS   Lifting fork to 0.08 m
100%  TASK     SUCCESS   Task completed
```

`GET /mission/status` still returns the same `task` object inline, and
`task_status` (the raw `TaskStepStatus` topic) can also be subscribed
directly over rosbridge.

## 7. Services and Actions (consumed as a client, not provided)

| Name | Type | Consumed by | Description |
|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `navigation_bridge_node`, checked by `bridge_manager_node` | Single-point navigation |
| `execute_mission` | `forklift_interfaces/action/ExecuteMission` | `mission_bridge_node`, checked by `bridge_manager_node` | Multi-task missions |
| `execute_task` | `forklift_interfaces/action/ExecuteTask` | Checked (presence only) by `bridge_manager_node`; not called directly by any bridge node — missions are always the entry point from the web layer |

This package provides no ROS services or actions itself; the REST API is
its own equivalent "service" surface (see §5).

## 8. Camera Streaming

`web_video_server` (port 8080, 4 server threads) streams any
`sensor_msgs/Image` topic already present on the graph — e.g.
`forklift_aruco`'s `aruco_image`/`aruco/image_raw` debug overlays or the
raw `camera/image_raw` from `forklift_gazebo`'s bridged simulated camera —
as MJPEG/HTTP, viewable directly in a browser `<img>` tag at
`http://<host>:8080/stream?topic=/camera/image_raw`, without any
`forklift_web_bridge` code involved beyond starting the server.

## 9. Important Parameters

`config/web_bridge.yaml` (the config actually loaded by the launch file):

| Node | Parameter | Default | Description |
|---|---|---|---|
| `rosbridge_server_node` | `port` / `address` | `9090` / `0.0.0.0` | WebSocket endpoint |
| `web_video_server_node` | `port` / `address` / `server_threads` | `8080` / `0.0.0.0` / `4` | Camera streaming endpoint |
| `fastapi_bridge_node` | `rest_port` / `rest_address` | `8000` / `0.0.0.0` | REST API endpoint |
| `bridge_manager_node` | `publish_rate_hz` | `1.0` Hz | Readiness-check publish rate |
| `robot_state_node` | `publish_rate_hz` | `5.0` Hz | Aggregated state publish rate |
| all bridge nodes | `*_action_name` / `*_topic` params | `/navigate_to_pose`, `/execute_mission`, `/execute_task`, `/diagnostics`, `/odom`, `/battery_state`, `web_bridge/*` | Downstream/upstream endpoint names, all overridable |

`config/fastapi.yaml`, `config/bridge.yaml`, and `config/mqtt.yaml` exist
but are **not referenced** by `forklift_web_bridge.launch.py` (only
`web_bridge.yaml` is loaded for every node); `fastapi.yaml` in particular
declares parameter names (`rest_host`, port `8080`) that don't match
`fastapi_bridge_node.py`'s actual declared parameters (`rest_address`, port
`8000` default) — treat it as stale/superseded, not as documentation of the
current REST config.

## 10. Launch Files and Run Commands

| Launch file | Starts | Notes |
|---|---|---|
| `forklift_web_bridge.launch.py` | `rosbridge_websocket`, `web_video_server`, `bridge_manager_node`, `robot_state_node`, `navigation_bridge_node`, `mission_bridge_node`, `diagnostics_bridge_node`, `fastapi_bridge_node` | The complete, active launch file. Does not start Nav2, `forklift_mission_manager`, `forklift_task_manager`, or `forklift_diagnostics` — those must already be running (typically via `forklift_bringup`). |
| `forklift_web_server.launch.py` | `rosbridge_websocket` only | **Legacy/incomplete**: defines a `web_video_server` Node object (with a typo, `output='scree'`) but never adds it to the returned `LaunchDescription` — running this file starts only the WebSocket bridge, not camera streaming. Superseded by `forklift_web_bridge.launch.py`. |

```bash
ros2 launch forklift_web_bridge forklift_web_bridge.launch.py

# Optional, not started by the launch file:
ros2 run forklift_web_bridge mqtt_bridge_node --ros-args -p broker_host:=localhost
```

## 11. Dependencies

From `package.xml`:

- `rclpy`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`,
  `diagnostic_msgs`, `nav2_msgs`
- `forklift_interfaces` (`ExecuteMission.action`, and `mqtt_bridge_node.py`
  additionally uses several `forklift_interfaces` messages)
- `rosbridge_server`, `web_video_server` (exec dependencies)
- `forklift_docking` (exec dependency — declared but not directly imported
  by any bridge node's Python source; likely a launch-time convenience or
  leftover)
- `python3-aiohttp` (declared; not directly imported by any node read
  during this documentation pass — `fastapi`/`uvicorn`/`pydantic`, used by
  `fastapi_bridge_node.py`, and `paho-mqtt`, used by `mqtt_bridge_node.py`,
  are **not** declared in `package.xml` and must be available in the
  Python environment separately)

## 12. Communication Flow

1. A browser dashboard connects either via `roslibjs` to
   `rosbridge_websocket` (port 9090) or via HTTP to `fastapi_bridge_node`
   (port 8000).
2. Both paths converge on the same `web_bridge/*` topics:
   REST POSTs and WebSocket publishes both land on
   `web_bridge/nav_command`/`web_bridge/mission_command`; REST GETs and
   WebSocket subscribes both read from the same status topics.
3. `navigation_bridge_node`/`mission_bridge_node` translate those JSON
   commands into real Nav2/`forklift_mission_manager` action calls, and
   translate the resulting feedback/result back into JSON status messages.
4. `robot_state_node` and `diagnostics_bridge_node` continuously aggregate
   telemetry (`/odom`, `/battery_state`, `/diagnostics`) into web-friendly
   summaries, independent of any command flow.
5. `bridge_manager_node` independently polls the ROS graph for the
   presence of the action servers/topics this whole package depends on,
   giving the dashboard one readiness check instead of several.
6. Camera feeds bypass this JSON protocol entirely — `web_video_server`
   serves any `sensor_msgs/Image` topic directly as MJPEG over HTTP.

## 13. Integration with Other Packages

- **forklift_navigation** — `navigation_bridge_node` is a client of Nav2's
  `navigate_to_pose` action.
- **forklift_mission_manager** — `mission_bridge_node` is a client of
  `execute_mission`.
- **forklift_task_manager** — only checked for presence by
  `bridge_manager_node`; not called directly (missions are the web layer's
  entry point, matching `forklift_mission_manager`'s own layering).
- **forklift_diagnostics** — `diagnostics_bridge_node` consumes its
  `/diagnostics` output.
- **forklift_aruco / forklift_gazebo** — indirectly, as sources of the
  camera topics `web_video_server` streams.
- **web_dashboard (Next.js + MongoDB)** — the primary consumer of both the
  REST API and the WebSocket bridge; MongoDB persistence (mission history,
  task logs, etc.) lives entirely on the dashboard side and has no
  corresponding ROS interface in this package.

## 14. Future Improvements

- **Planned / Future:** wire up or remove `mqtt_bridge_node.py` — fully
  implemented but not included in either launch file.
- **Planned / Future:** reconcile or remove `config/fastapi.yaml`,
  `config/bridge.yaml` — currently unused and, in `fastapi.yaml`'s case,
  inconsistent with the actual node parameters.
- Fix `forklift_web_server.launch.py` (missing `web_video_server` in the
  returned `LaunchDescription`, `output='scree'` typo) or remove it in
  favor of `forklift_web_bridge.launch.py`.
- Add authentication/authorization to the REST API and WebSocket bridge —
  both are currently open on `0.0.0.0` with no access control.
- Make POST endpoint responses reflect actual downstream acceptance
  (e.g. by waiting briefly for the corresponding bridge node's status
  update) instead of always returning `{"accepted": true}` on publish
  alone.
- Declare `fastapi`, `uvicorn`, `pydantic`, and `paho-mqtt` as explicit
  Python dependencies (e.g. in a `requirements.txt` or `package.xml`
  `exec_depend`) rather than relying on them being present in the
  environment.
