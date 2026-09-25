# forklift_traffic_manager

ROS 2 (Humble) `ament_cmake` package implementing fleet-wide traffic
arbitration for shared `nav2_route` graph edges: a single fleet-level
`traffic_manager_node` grants/denies exclusive claims on contested route
edges, and a per-robot `nav2_route::RouteOperation` plugin
(`TrafficReservation`) requests/releases those claims as each robot's
route tracker enters and exits them.

---

## 1. Overview and Responsibility

Multiple forklifts sharing the same `nav2_route` graph can be routed
through the same narrow aisle or single-lane segment at the same time.
`forklift_traffic_manager` prevents that by making edge occupancy a
fleet-wide, centrally-arbitrated resource:

- **`traffic_manager_node`** (one instance, fleet-wide) is the single
  source of truth for who currently owns each contested edge. It exposes
  a `~/reserve_edge` service and, whenever an edge changes hands, actively
  pushes a `DynamicEdges` cost/open/close update to every *other* robot's
  `route_server` so their planners stop considering that edge available.
- **`TrafficReservation`** (one plugin instance per robot, loaded inside
  that robot's own `route_server`) is a `nav2_route::RouteOperation` that
  fires `ON_STATUS_CHANGE` (i.e. whenever the route tracker enters or
  exits a graph edge) and calls `traffic_manager_node`'s `reserve_edge`
  service to acquire the edge it's entering and release the edge it's
  leaving, but **only for edges listed in that robot's own
  `contested_edges` parameter** — most edges are not arbitrated at all.

This package contains no navigation logic of its own — it only decides
*whether a robot may currently enter a specific edge*, and relies on
`nav2_route`'s own BT retry/recovery machinery (rerouting, or a
Wait/BackUp recovery) to handle a denied request.

## 2. Architecture / Role in the Overall System

```
                         forklift1's route_server                forklift2's route_server
                    ┌─────────────────────────────┐        ┌─────────────────────────────┐
                    │  TrafficReservation           │        │  TrafficReservation           │
                    │  (RouteOperation plugin,       │        │  (RouteOperation plugin,       │
                    │   ON_STATUS_CHANGE)             │        │   ON_STATUS_CHANGE)             │
                    │  contested_edges: [...]         │        │  contested_edges: [...]         │
                    └───────────────┬───────────────┘        └───────────────┬───────────────┘
                                    │ ~/reserve_edge (ReserveEdge.srv)        │ ~/reserve_edge
                                    │  robot_name, edge_id, acquire           │
                                    ▼                                        ▼
                    ┌───────────────────────────────────────────────────────────────┐
                    │                     traffic_manager_node                       │
                    │   map<edge_id, owning_robot>  (in-memory, single source        │
                    │   of truth for who currently owns each contested edge)          │
                    └───────────────┬───────────────────────────────┬───────────────┘
                     on grant/release: DynamicEdges to EVERY         │
                     OTHER robot's route_server (not the owner)      │
                                    ▼                                ▼
                     forklift2's route_server/DynamicEdgesScorer/  forklift1's ...
                     adjust_edges  (closes/opens the edge for
                     forklift2's OWN future replanning only)
```

If `traffic_manager_node` is unreachable or slow (>500 ms) to respond,
`TrafficReservation::requestReservation()` **fails open**: it logs a loud
warning and returns `true` (treats the reservation as granted) rather
than permanently stranding the robot — collision avoidance in that
degraded case falls back entirely to each robot's own local
costmap/collision_monitor, with no fleet-level coordination.

## 3. Directory Structure

```
forklift_traffic_manager/
├── config/
│   └── traffic_manager.yaml         # fleet robot list + DynamicEdgesScorer service name suffix
├── include/forklift_traffic_manager/
│   ├── traffic_manager_node.hpp     # fleet-wide arbiter
│   └── traffic_reservation.hpp      # per-robot RouteOperation plugin
├── src/
│   ├── traffic_manager_node.cpp
│   └── traffic_reservation.cpp
├── launch/
│   └── traffic_manager.launch.py    # starts traffic_manager_node only
├── route_operations_plugins.xml     # pluginlib export for TrafficReservation
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node / Plugin | Executable / Library | Responsibility |
|---|---|---|
| `TrafficManagerNode` | `traffic_manager_node` | Fleet-wide, single-instance coordinator. Holds an in-memory `map<edge_id, owning_robot>`. Serves `~/reserve_edge`: grants an `acquire` request if the edge is free or already owned by the requester; denies it otherwise. On every grant/release, calls `DynamicEdges` on every *other* robot's `route_server/DynamicEdgesScorer/adjust_edges` service (fire-and-forget) so their planners see the edge as closed/opened — never on the owning robot's own route_server, since it needs the edge open to actually traverse it. |
| `TrafficReservation` | `traffic_reservation` (shared library, `pluginlib`-loaded `nav2_route::RouteOperation`) | Runs **inside each robot's own `route_server`**, one instance per robot. Configured with a `robot_name` (defaults to the node's own namespace) and a `contested_edges` list. On `ON_STATUS_CHANGE`, releases the edge just exited and requests the edge just entered — only for edges in `contested_edges`; everything else is untouched. If denied, sets `OperationResult::reroute = true` with the edge added to `blocked_ids`, letting `route_server`'s own recompute/recovery logic take over. Uses a dedicated `MutuallyExclusive` callback group + single-threaded executor for its blocking service call, so it doesn't reenter or deadlock `route_server`'s own executor (the same pattern `nav2_behavior_tree::BtActionNode` uses for its action clients). |

## 5. Input/Output Topics

This package has no topic-based interfaces — all communication is via
services (§6).

## 6. Services and Actions

| Name | Type | Provided by | Consumed by | Description |
|---|---|---|---|---|
| `~/reserve_edge` (i.e. `/traffic_manager/reserve_edge`) | `forklift_interfaces/srv/ReserveEdge` | `traffic_manager_node` | Each robot's `TrafficReservation` plugin instance | Request: `robot_name`, `edge_id`, `acquire` (`true` = claim, `false` = release). Response: `granted`. A release always returns `granted: true`, whether or not the requester actually held the edge. |
| `route_server/DynamicEdgesScorer/adjust_edges` (per robot, name configurable via `dynamic_edges_service_name`) | `nav2_msgs/srv/DynamicEdges` | Each robot's `route_server` (external, `nav2_route`) | `traffic_manager_node` (client) | `traffic_manager_node` calls this on every *other* robot when an edge is granted (`closed_edges`) or released (`opened_edges`) — fire-and-forget, no result handling. |

No actions.

## 7. Important Parameters

`config/traffic_manager.yaml` (`traffic_manager_node`):

| Parameter | Default | Description |
|---|---|---|
| `robots` | `["forklift1", "forklift2"]` | Fleet membership — must list every robot namespace that has the `TrafficReservation` operation enabled. Must be kept in manual sync with `forklift_navigation/config/robots.yaml`'s active robot list (same manual-sync convention used elsewhere between that file and `forklift_gazebo`'s spawn list — see those packages' READMEs). If empty, the node logs an error and has nothing to arbitrate. |
| `dynamic_edges_service_name` | `route_server/DynamicEdgesScorer/adjust_edges` | Relative suffix appended to `/<robot>/` to reach each robot's `DynamicEdgesScorer` service; must match the `edge_cost_functions` plugin instance name (`"DynamicEdgesScorer"`) configured in `nav2_multirobot_params.yaml`. |

`TrafficReservation` plugin parameters (declared per-instance under
`<route_server_plugin_name>.*` inside each robot's own `route_server`
params, **not** in this package's `traffic_manager.yaml`):

| Parameter | Default | Description |
|---|---|---|
| `<name>.robot_name` | node's own namespace | Identity sent in `ReserveEdge` requests |
| `<name>.contested_edges` | `[]` (empty) | Only edges in this list are arbitrated at all — everything else is ignored by this plugin |
| `<name>.traffic_manager_service` | `/traffic_manager/reserve_edge` | Where to reach the fleet coordinator |

## 8. Launch Files and How to Run Them

| Launch file | Starts | Notes |
|---|---|---|
| `traffic_manager.launch.py` | `traffic_manager_node` only | Loads `config/traffic_manager.yaml` by default (overridable via `params_file` argument); `use_sim_time` defaults to `True`. Does **not** start any `route_server` — the `TrafficReservation` plugin is loaded separately, inside each robot's own `route_server` process, via that server's own plugin configuration (in `forklift_navigation`), not by this package. |

```bash
ros2 launch forklift_traffic_manager traffic_manager.launch.py

# Manually test the reservation service
ros2 service call /traffic_manager/reserve_edge forklift_interfaces/srv/ReserveEdge \
  "{robot_name: 'forklift1', edge_id: 12, acquire: true}"
```

To actually exercise `TrafficReservation` in a route, the plugin must be
registered under a robot's `route_server`'s `operations` parameter (via
`route_operations_plugins.xml`'s pluginlib export) with a matching
`contested_edges` list — **this is not currently done anywhere in this
workspace**. `forklift_navigation/config/nav2_params.yaml`'s
`route_server.operations` list is `["AdjustSpeedLimit", "ReroutingService"]`
— `TrafficReservation` is absent. See §11.

## 9. Dependencies

From `package.xml`:

- `rclcpp`, `rclcpp_lifecycle`, `pluginlib`
- `nav2_route`, `nav2_msgs`, `nav2_costmap_2d`
- `geometry_msgs`
- `forklift_interfaces` (`ReserveEdge.srv`)
- `launch`, `launch_ros`
- Exports its `nav2_route` plugin via
  `<nav2_route plugin="${prefix}/route_operations_plugins.xml" />` in
  `package.xml`'s `<export>` block, so any `route_server` that depends on
  this package can `pluginlib`-load `TrafficReservation` by class name
  (`forklift_traffic_manager::TrafficReservation`).

## 10. Integration with the Rest of the System

- **forklift_navigation** — owns and configures each robot's
  `route_server`. It **does** configure `DynamicEdgesScorer` as one of
  `route_server`'s `edge_cost_functions`
  (`nav2_params.yaml: edge_cost_functions: ["DistanceScorer",
  "DynamicEdgesScorer"]`) — the piece `traffic_manager_node` pushes
  `DynamicEdges` updates into. But it does **not** list
  `TrafficReservation` in `route_server.operations`
  (currently `["AdjustSpeedLimit", "ReroutingService"]` only) — so the
  reservation-request half of this package's design (the plugin that
  actually calls `~/reserve_edge` as a robot enters a contested edge) is
  not loaded by any robot's `route_server` in this workspace today. See
  §11.
- **forklift_interfaces** — supplies `ReserveEdge.srv`, the sole custom
  interface this package uses.
- **nav2_route / nav2_msgs** (external, Nav2) — this package is built
  entirely as an extension of `nav2_route`'s `RouteOperation` plugin
  interface and its `DynamicEdges`/`DynamicEdgesScorer` edge-cost
  mechanism, both external to this workspace.

## 11. Known Issues / TODO / Future Work

- **`TrafficReservation` is not currently loaded by any robot's
  `route_server`**: `forklift_navigation/config/nav2_params.yaml`
  configures `DynamicEdgesScorer` (the receiving end, which lets
  `traffic_manager_node` close/open edges for a robot's planner) but its
  `route_server.operations` list does not include `TrafficReservation`
  (the requesting end, which would actually call `~/reserve_edge` as a
  robot enters/exits a contested edge). As shipped, `traffic_manager_node`
  can run and would accept reservation requests, but nothing in the
  current navigation stack ever sends one — the whole
  request/grant/deny/reroute loop this package implements is inert until
  `TrafficReservation` (with a populated `contested_edges` list) is added
  to `route_server.operations` for each robot.
- **Manual sync risk**: `traffic_manager.yaml`'s `robots` list must be
  kept manually in sync with `forklift_navigation/config/robots.yaml`'s
  active robot names — nothing enforces this, and a mismatch would leave
  a real robot unarbitrated (its edges silently never broadcast to other
  robots) with no error beyond the one-time startup log.
- **Fire-and-forget broadcast has no delivery guarantee**:
  `broadcastEdgeState()` does not check the result of `async_send_request()`
  calls to other robots' `DynamicEdgesScorer` services — if one of those
  calls silently fails after passing the `service_is_ready()` check, that
  robot's planner never learns the edge closed, and there is no retry.
- **No timeout/staleness handling on held reservations**: if a robot
  crashes or loses connectivity while holding an edge reservation,
  `traffic_manager_node` has no mechanism to expire or reclaim it — the
  edge stays permanently "owned" by the dead robot until it (or an
  operator) explicitly releases it.
- **Fail-open behavior on timeout is a deliberate but real safety
  trade-off**: a robot losing contact with `traffic_manager_node` (down,
  or the 500 ms `spin_until_future_complete` timing out) proceeds as if
  granted, relying entirely on local costmap/collision_monitor avoidance.
  This avoids permanently stranding the fleet at the cost of temporarily
  losing centralized arbitration — worth being aware of when tuning the
  200 ms/500 ms `wait_for_service`/response timeouts.
- Single point of failure: there is exactly one `traffic_manager_node`
  instance for the whole fleet, with no restart/redundancy story beyond
  the fail-open behavior above.
