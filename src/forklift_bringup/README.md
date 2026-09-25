# forklift_bringup

ROS 2 (Humble) `ament_cmake` package providing the top-level, one-command
launch files for the Mini Forklift AMR: single-robot full-stack bringup,
multi-robot Gazebo Fortress spawning, and multi-robot navigation/task/
mission bringup.

---

## 1. Overview and Responsibility

`forklift_bringup` is the **composition root** of the project: it does not
implement any robot behavior itself, but sequences and parameterizes the
launch of every other package into a small number of top-level commands —
one for a single simulated robot, and a pair for a namespaced multi-robot
fleet (world spawning + per-robot navigation/docking/task/mission stack).

It contains no nodes of its own beyond what its included launch files
start; its only original content is launch orchestration (staggered
`TimerAction`s, a fleet YAML, and a legacy `ros2_control` spawner sequence).

## 2. Architecture / Role in the Overall System

```
                    single_robot_bringup.launch.py
                              │
        (does NOT start forklift_gazebo -- see §10 note)
                              │
     t=navigation_delay ──▶ forklift_navigation/bringup_launch.py (nav2+localization)
     t=navigation_delay ──▶ forklift_navigation/rviz_launch.py
     t=docking_delay    ──▶ forklift_docking/docking.launch.py
     t=controller_delay ──▶ forklift_controller/forklift_controller.launch.py
     t=aruco_delay       ──▶ forklift_aruco/aruco_pose_estimation2.launch.py
     t=task_manager_delay──▶ forklift_task_manager/task_manager.launch.py
     t=mission_manager_delay ▶ forklift_mission_manager/mission_manager.launch.py


                    multiple_robot_world.launch.py   (world + N robots spawned)
                              │  (shares the ROS graph over DDS with the file below)
                    multiple_bringup_launch.py        (run 2nd, in a separate terminal)
                              │  waits for the LAST robot's /odom topic
              per robot, staggered by robot_stagger_delay:
                nav2 bringup (forklift_navigation) + initial pose
                docking (forklift_docking, multirobot)
                RViz + ArUco + lift_controller_node_multirobot
                forklift_task_manager + forklift_mission_manager + forklift_diagnostics
```

## 3. Directory Structure

```
forklift_bringup/
├── config/
│   └── robots.yaml                        # fleet definition: name, x/y/z spawn pose (1 robot active, 2 commented)
├── launch/
│   ├── single_robot_bringup.launch.py     # single-robot: nav2/docking/controller/aruco/task/mission, staggered
│   ├── multiple_robot_world.launch.py     # multi-robot: Gazebo world + N robot spawns (does NOT include navigation)
│   ├── multiple_bringup_launch.py         # multi-robot: per-robot nav2/docking/aruco/task/mission/diagnostics
│   └── load_ros2_controller.launch,.py    # legacy ros2_control controller spawner sequence (note the literal comma in the filename — see §10)
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

This package defines no nodes of its own. It composes the following
node-launching packages:

| Launch file | Includes / starts |
|---|---|
| `single_robot_bringup.launch.py` | `forklift_navigation`'s `bringup_launch.py` + `rviz_launch.py`, `forklift_docking`'s `docking.launch.py`, `forklift_controller`'s `forklift_controller.launch.py`, `forklift_aruco`'s `aruco_pose_estimation2.launch.py`, `forklift_task_manager`'s `task_manager.launch.py`, `forklift_mission_manager`'s `mission_manager.launch.py` |
| `multiple_robot_world.launch.py` | Gazebo Fortress (`ros_gz_sim`'s `gz_sim.launch.py`), a clock bridge, and per-robot `robot_state_publisher` + `ros_gz_sim create` (spawn) + `ros_gz_bridge` (`parameter_bridge`), for every robot in `config/robots.yaml` |
| `multiple_bringup_launch.py` | Per robot: `forklift_navigation`'s `bringup_launch.py`, an initial-pose publish, `forklift_docking`'s `multirobot_docking.launch.py`, `forklift_navigation`'s `rviz_launch.py`, `forklift_aruco`'s `aruco_pose_estimation2.launch.py`, `forklift_controller`'s `lift_controller_node_multirobot`, `forklift_task_manager`'s `task_manager.launch.py`, `forklift_mission_manager`'s `mission_manager.launch.py`, `forklift_diagnostics`'s `diagnostics.launch.py` |
| `load_ros2_controller.launch,.py` | Legacy: `joint_state_broadcaster`/`diff_drive_controller`/`lift_position_controller` `ros2_control` spawners, sequenced via `OnProcessExit` — for the (currently unused, per `forklift_gazebo`'s README) `ros2_control`/`ign_ros2_control` code path rather than the active native-Ignition-plugin path |

## 5. Input / Output Topics

None directly — this package only launches other packages' nodes, each of
which owns its own topics (see the individual package READMEs).

## 6. Services and Actions

None directly. This package's job ends once the downstream action servers
(`navigate_to_pose`, `dock_robot`/`undock_robot`, `set_height`,
`execute_task`, `execute_mission`) are up and running.

## 7. Important Parameters

`config/robots.yaml` — single source of truth for which robots exist and
their spawn poses, shared by both multi-robot launch files:

```yaml
robots:
  - name: forklift1
    x_pose: -1.0
    y_pose: -0.5
    z_pose: 0.0
  # forklift2, forklift3 present but commented out
```

In-source warning: robot `name` values here must match the literal
`"forklift1"`/`"forklift2"`/... strings that would need to be baked into
any per-robot static Nav2 params files, and must be kept in sync with
`forklift_gazebo`'s own (separate) multi-robot spawn fleet list — neither
consistency is automatically checked.

### `single_robot_bringup.launch.py` staggered delays

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Forwarded to every stage |
| `controller` | `mppi` | Nav2 controller plugin, forwarded to `forklift_navigation`'s `bringup_launch.py` |
| `navigation_delay` | `2.0` s | Before starting Nav2/localization/RViz — needs Gazebo's world fully loaded and the robot spawned |
| `docking_delay` | `2.0` s | Before starting `docking_server` |
| `controller_delay` | `10.0` s | Before starting `forklift_controller` |
| `aruco_delay` | `10.0` s | Before starting `forklift_aruco` — needs camera topics publishing |
| `task_manager_delay` | `14.0` s | Before starting `forklift_task_manager` |
| `mission_manager_delay` | `16.0` s | Before starting `forklift_mission_manager` |

All delays are measured from this launch file's own start (t=0), not
chained from the end of the previous stage, and are explicitly documented
as a **heuristic, not a readiness guarantee** — there is no cheap, general
way in ROS 2 launch to detect "Gazebo has actually finished loading the
world" versus "the Gazebo process has started." `forklift_task_manager`
and `forklift_mission_manager` are the least timing-sensitive (their
action clients call `wait_for_action_server()` inside task/mission
execution, not at construction), so the Gazebo→navigation gap matters most.

### `multiple_bringup_launch.py` / `multiple_robot_world.launch.py`

| Argument | Default | Description |
|---|---|---|
| `robots_file` | `config/robots.yaml` | Fleet definition path |
| `robots` | `""` | Optional inline JSON/YAML fleet override |
| `use_sim_time` | `true` | Forwarded throughout |
| `robot_stagger_delay` | `8.0` s | Seconds added per robot index so N robots' Nav2/AMCL stacks don't all activate simultaneously |

## 8. Launch Files and Run Commands

```bash
# Single robot: full stack (assumes Gazebo/forklift_gazebo already running — see §10)
ros2 launch forklift_bringup single_robot_bringup.launch.py controller:=mppi

# Multi-robot: terminal 1 — spawn the world + fleet in Gazebo
ros2 launch forklift_bringup multiple_robot_world.launch.py

# Multi-robot: terminal 2 — bring up nav/docking/task/mission per robot
# (waits automatically for the last robot's /odom topic before starting)
ros2 launch forklift_bringup multiple_bringup_launch.py
```

## 9. Dependencies

From `package.xml`:

- `forklift_navigation`, `forklift_controller`, `forklift_aruco`,
  `forklift_task_manager`, `forklift_mission_manager` (exec dependencies)

Not declared but included at launch time via absolute package-share paths
(a real gap, not just an omission — see §12): `forklift_docking`,
`forklift_diagnostics`, `forklift_gazebo`, `forklift_description`.

## 10. Communication Flow

1. **Single robot:** the user (or a script) is expected to start
   `forklift_gazebo` separately first — despite
   `single_robot_bringup.launch.py`'s own module docstring numbering
   "1. forklift_gazebo" as the first of six stages, **no such include
   exists in the file**; Gazebo is not actually started by this launch
   file. Once Gazebo (and thus the robot's TF/topics) is up,
   `single_robot_bringup.launch.py` starts Nav2/localization, docking, the
   lift controller, ArUco detection, the task manager, and the mission
   manager, each after its own fixed delay from t=0.
2. **Multi-robot:** `multiple_robot_world.launch.py` starts Gazebo Fortress
   with the warehouse world and spawns every robot in `robots.yaml`
   (`robot_state_publisher` + `ros_gz_sim create` + a per-robot
   `ros_gz_bridge`). Because two separate `ros2 launch` processes can't
   share a process handle for an `OnProcessExit` gate,
   `multiple_bringup_launch.py` (run second, in a separate terminal)
   instead polls the shared ROS/DDS graph for the **last** robot's `/odom`
   topic before proceeding — a process-boundary-safe substitute for the
   single-process `OnProcessExit` pattern. Once all robots are confirmed
   spawned, it launches each robot's full navigation/docking/lift/task/
   mission/diagnostics stack, staggered by `robot_stagger_delay` seconds
   per robot index.

## 11. Integration with Other Packages

- **forklift_gazebo** — expected to already be running for
  `single_robot_bringup.launch.py`; **not** included by that file despite
  being documented as step 1 in its own docstring (see §12).
- **forklift_navigation, forklift_docking, forklift_controller,
  forklift_aruco, forklift_task_manager, forklift_mission_manager,
  forklift_diagnostics** — all directly launched/composed by this package.
- **web_dashboard (Next.js + MongoDB)** — not launched by this package.
  The dashboard connects to `forklift_web_bridge`'s REST/WebSocket
  endpoints (ports 8000/9090/8080) once the ROS-side stack brought up here
  is running; `forklift_web_bridge` itself is not included in any of this
  package's launch files and must be started separately (e.g.
  `ros2 launch forklift_web_bridge forklift_web_bridge.launch.py`) alongside
  the dashboard's own `next dev`/`next start` process and its MongoDB
  instance.

## 12. Future Improvements

- **Planned / Future:** either add a `forklift_gazebo` include (with its
  own startup delay before `navigation_delay`) to
  `single_robot_bringup.launch.py`, or correct its docstring to stop
  listing Gazebo as something this file starts.
- **Planned / Future:** add a `forklift_web_bridge` stage to a bringup
  launch file (or a documented combined command) so the web dashboard
  stack starts from the same one-command entry point as the rest of the
  system.
- Declare `forklift_docking`, `forklift_diagnostics`, `forklift_gazebo`,
  and `forklift_description` as explicit `exec_depend`s in `package.xml` —
  they are referenced via `get_package_share_directory()` in the launch
  files but are not currently declared dependencies of this package.
- Rename `load_ros2_controller.launch,.py` (the literal comma before
  `.py` is almost certainly a typo) and clarify whether it is still needed
  given `forklift_gazebo`'s native-Ignition-plugin control path is what's
  actually active.
- Replace the multi-robot "wait for the last robot's `/odom` topic" and
  the single-robot fixed-delay `TimerAction`s with a more robust readiness
  signal where practical (e.g. a lifecycle/service-based check), since both
  are explicitly documented as heuristics rather than guarantees.
- Keep `forklift_bringup/config/robots.yaml` and `forklift_gazebo`'s own
  multi-robot fleet list in sync automatically (e.g. by having one include
  or generate the other) instead of relying on manual consistency, as
  currently documented in both files' comments.
