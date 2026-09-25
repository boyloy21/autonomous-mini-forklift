# forklift_controller

ROS 2 (Humble) `ament_cmake` package implementing fork-lift height control
for the Mini Forklift AMR: jog (raise/lower) control, a `SetLiftHeight`
action for precise positioning, and a set of auxiliary nodes for a future
real-hardware driver/feedback/safety chain.

---

## 1. Overview and Responsibility

`forklift_controller` owns the vertical fork/lift axis (`lift_joint`,
0–0.15 m travel per `forklift_description`). It provides:

- **Active, launched control path**: `lift_controller_node` (or
  `lift_controller_node_multirobot` for namespaced fleets) — jog control via
  `/lift_up`/`/lift_down` and precise positioning via a `SetLiftHeight`
  action server (`set_height`), publishing a position setpoint directly on
  `lift_cmd` based on live feedback from `/joint_states`.
- **Auxiliary/not-currently-launched nodes**: `lift_driver_node`,
  `lift_feedback_node`, `lift_limit_node`, `lift_safety_node`,
  `lift_cmd_bridge_node` — a more elaborate, hardware-oriented pipeline
  (motor command → encoder feedback → limit/safety gating) that is fully
  implemented but **commented out** in the current launch file in favor of
  the simpler, `/joint_states`-fed `lift_controller_node`.

## 2. Architecture / Role in the Overall System

### Active path (as actually launched)

```
              /lift_up, /lift_down (std_msgs/Bool, jog)
                         │
              set_height (action, forklift_interfaces/SetLiftHeight)
                         │        ▲
                         ▼        │ goal from forklift_task_manager / forklift_docking
              +----------------------+
              |  lift_controller_node |── /joint_states (feedback, lift_joint position)
              +----------------------+
                         │
                         ▼
                    lift_cmd (std_msgs/Float64, position setpoint)
                         │
                         ▼
        forklift_gazebo (bridge) -> Ignition JointPositionController plugin
                         │
                         ▼
              forklift_description (lift_joint, prismatic, lift.xacro)
```

### Auxiliary hardware-oriented pipeline (implemented, not currently wired into any launch file)

```
/lift_motor_cmd (LiftMotorCommand) ──▶ lift_driver_node ──▶ /encoder_height (LiftEncoderHeight)
       ▲                                    │                        │
       │ (bridge to plain Float64)          ▼                        ▼
lift_cmd_bridge_node                 /lift_drive_status      lift_feedback_node ──▶ /CurrentHeight
                                      (LiftDriverStatus)                                │
                                                                                         ▼
                                                                                lift_limit_node ──▶ /lift_limit
                                                                                                        │
                                                                            /emergency_stop ──▶ lift_safety_node ──▶ /lift_enable
```

## 3. Directory Structure

```
forklift_controller/
├── config/
│   └── controller.yaml               # parameters for all six nodes (only lift_controller_node's are active)
├── launch/
│   └── forklift_controller.launch.py # launches lift_controller_node only (others commented out)
├── src/
│   ├── lift_controller_node.cpp             # active: single-robot jog + SetLiftHeight action
│   ├── lift_controller_node_multirobot.cpp  # active: same, with all topics as namespace-safe parameters
│   ├── lift_driver_node.cpp                 # auxiliary: motor command -> encoder feedback (sim or future hardware)
│   ├── lift_feedback_node.cpp               # auxiliary: encoder height -> CurrentHeight + velocity/moving flag
│   ├── lift_limit_node.cpp                  # auxiliary: CurrentHeight -> top/bottom/soft limit status
│   ├── lift_safety_node.cpp                 # auxiliary: e-stop + limit fault -> /lift_enable gate
│   └── lift_cmd_bridge_node.cpp             # auxiliary: LiftMotorCommand -> plain Float64 /lift_cmd
├── CMakeLists.txt
└── package.xml
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Status | Responsibility |
|---|---|---|---|
| `LiftControllerNode` | `lift_controller_node` | **Active** (launched) | Single-robot jog (`/lift_up`/`/lift_down`) + `SetLiftHeight` action server (`set_height`); reads `lift_joint` position from `/joint_states`; ramps a position setpoint on `/lift_cmd` at `control_loop_hz`. |
| `LiftControllerNode` (multirobot variant) | `lift_controller_node_multirobot` | **Active** (launched per-robot by `forklift_navigation`'s `multiple_bringup_launch.py`) | Identical logic, but `lift_up_topic`/`lift_down_topic`/`joint_states_topic`/`lift_cmd_topic` are declared as parameters (relative names) instead of hardcoded absolute topics, so per-robot namespacing via `Node(namespace=...)` actually isolates each robot's lift topics. |
| `LiftDriverNode` | `lift_driver_node` | Implemented, not launched | Simulates (or would bridge to real STM32 hardware via `use_hardware:=true`, **not implemented** — logs a warning and falls back to simulation) a velocity-driven lift: integrates `/lift_motor_cmd`'s velocity into a height, publishes `/encoder_height` and `/lift_drive_status`. |
| `LiftFeedbackNode` | `lift_feedback_node` | Implemented, not launched | Converts `/encoder_height` into `/CurrentHeight` (position + derived velocity + `moving` flag). |
| `LiftLimitNode` | `lift_limit_node` | Implemented, not launched | Converts `/CurrentHeight` into `/lift_limit` (top/bottom/soft-margin limit flags). |
| `LiftSafetyNode` | `lift_safety_node` | Implemented, not launched | Master safety gate: combines `/emergency_stop` and a top+bottom limit sensor fault (`/lift_limit`) into `/lift_enable`. |
| `LiftCmdBridgeNode` | `lift_cmd_bridge_node` | Implemented, not launched | Converts the richer `LiftMotorCommand` (velocity + enable) into a plain `std_msgs/Float64` on `/lift_cmd`, for a velocity-mode `ros2_control` controller. |

## 5. Input / Output Topics

### Active path (`lift_controller_node` / multirobot variant)

| Name | Type | Direction | Description |
|---|---|---|---|
| `lift_up` | `std_msgs/Bool` | Input | Jog raise while `true` (ignored while an `AUTO`/action-driven move is in progress) |
| `lift_down` | `std_msgs/Bool` | Input | Jog lower while `true` |
| `joint_states` | `sensor_msgs/JointState` | Input | Live `lift_joint` position, used as the control-loop feedback |
| `lift_cmd` | `std_msgs/Float64` | Output | Position setpoint (meters), ramped toward the jog/auto target at `control_loop_hz` |

### Auxiliary hardware pipeline

| Name | Type | Direction | Node | Description |
|---|---|---|---|---|
| `/lift_motor_cmd` | `forklift_interfaces/LiftMotorCommand` | Input | `lift_driver_node`, `lift_cmd_bridge_node` | Signed velocity command + enable flag |
| `/lift_enable` | `std_msgs/Bool` | Input | `lift_driver_node` | Hard motor enable/disable, zeros velocity when false |
| `/encoder_height` | `forklift_interfaces/LiftEncoderHeight` | Output | `lift_driver_node` | Raw simulated/real encoder height |
| `/lift_drive_status` | `forklift_interfaces/LiftDriverStatus` | Output | `lift_driver_node` | Connection/fault status |
| `/CurrentHeight` | `forklift_interfaces/CurrentHeight` | Output | `lift_feedback_node` | Height + derived velocity + `moving` flag |
| `/lift_limit` | `forklift_interfaces/LiftLimitStatus` | Output | `lift_limit_node` | Top/bottom/soft-margin limit flags |
| `/emergency_stop` | `std_msgs/Bool` | Input | `lift_safety_node` | System-wide E-stop signal |
| `/lift_enable` | `std_msgs/Bool` | Output | `lift_safety_node` | Master enable gate (false on E-stop or top+bottom fault) |
| `/lift_cmd` | `std_msgs/Float64` | Output | `lift_cmd_bridge_node` | Bridged plain velocity command |

## 6. Services and Actions

| Name | Type | Provided by | Description |
|---|---|---|---|
| `set_height` | `forklift_interfaces/action/SetLiftHeight` | `lift_controller_node` / `lift_controller_node_multirobot` | Goal: `target_height` (m), `speed` (m/s, 0 = default). Feedback: `current_height`, `target_height` at 5 Hz. Result: `success`, `message` (`"reached"` / `"blocked by max_height"` / `"blocked by min_height"` / `"timed out"` / `"canceled"`), `current_height`. Runs on a detached thread so the executor is never blocked; supersedes an earlier blocking-service design specifically to avoid tying up an executor thread for a multi-second move. |

`ExecuteLift`/`/execute_lift` mentioned in some project documentation is
**not implemented** in this package — the only precise-positioning
interface is the `set_height` action. Jog control (`/lift_up`/`/lift_down`)
is a topic-based interface, not a service/action.

## 7. Important Parameters

`config/controller.yaml`:

| Node | Parameter | Default | Description |
|---|---|---|---|
| `lift_controller_node` | `min_height` / `max_height` | `0.0` / `1.2`* m | Absolute travel clamp |
| | `jog_speed` | `0.05` m/s | Speed while jogging via `/lift_up`/`/lift_down` |
| | `default_auto_speed` | `0.08` m/s | Default speed for a `SetLiftHeight` goal with `speed: 0` |
| | `max_speed` | `0.15` m/s | Hard cap on any commanded speed |
| | `height_tolerance` | `0.01` m | `SetLiftHeight` goal success tolerance |
| | `set_height_timeout_sec` | `30.0` s | `SetLiftHeight` goal timeout |
| | `control_loop_hz` | `20.0` Hz | Position ramp / `lift_cmd` publish rate |
| | `lift_joint_name` | `"lift_joint"` | Joint name looked up in `/joint_states` |
| `lift_controller_node_multirobot` (additional) | `lift_up_topic` / `lift_down_topic` / `joint_states_topic` / `lift_cmd_topic` | `lift_up` / `lift_down` / `joint_states` / `lift_cmd` | Relative topic names, safe to namespace per robot |
| `lift_driver_node` | `use_hardware` | `false` | `true` logs a warning and still falls back to simulation (no hardware backend implemented) |
| | `serial_port` / `serial_baud` | `/dev/ttyUSB0` / `115200` | Reserved for a future STM32 serial link |
| | `min_height` / `max_height` | `0.0` / `1.2` m | Simulated integration clamp |
| `lift_feedback_node` | `moving_velocity_threshold` | `0.005` m/s | Threshold for the `moving` flag |
| `lift_limit_node` | `min_height` / `max_height` / `soft_margin` | `0.0` / `1.2` / `0.05` m | Limit + soft-margin thresholds |

*Note: `config/controller.yaml`'s `lift_controller_node.max_height` is
`1.2` m, while `lift_controller_node.cpp`'s in-code default (used if the
parameter is unset) is `0.12` m, and `forklift_description/urdf/const.xacro`'s
actual `lift_travel` is `0.15` m. These three values disagree; the YAML file
(loaded when the node is launched with `config_file`) is the one actually
in effect for `set_height`'s range-rejection check, but does not match the
physical URDF joint limit. This is a real inconsistency worth resolving,
not a documentation error.

## 8. Simulation / Hardware Transition

- **Simulation (current, active):** `lift_controller_node` publishes a
  **position** setpoint on `lift_cmd`, consumed in Gazebo Fortress by the
  `ignition::gazebo::systems::JointPositionController` plugin on
  `lift_joint` (defined in `forklift_description/urdf/gazebo.xacro`), via
  `forklift_gazebo`'s `ros_gz_bridge` (`/lift_cmd`, bidirectional
  `std_msgs/Float64`).
- **Hardware (planned, not implemented):** the auxiliary pipeline
  (`lift_driver_node` with `use_hardware:=true`) is scaffolded to bridge to
  a real STM32 controller over serial/CAN, but the actual hardware I/O is a
  `TODO` in source — `use_hardware:=true` currently just logs a warning and
  continues in simulation mode. `lift_cmd_bridge_node` and
  `lift_feedback_node`/`lift_limit_node`/`lift_safety_node` form the rest of
  a velocity-command + encoder-feedback + limit-switch + E-stop chain
  intended for that hardware path, but none of them are included in the
  current launch file.
- **Limit switch / ToF sensor:** no physical limit-switch or Time-of-Flight
  sensor driver exists in this package. `lift_limit_node`'s top/bottom
  limits are derived purely from the *commanded/estimated* height
  (`/CurrentHeight`), not an independent physical sensor — a genuine
  hardware limit switch or ToF-based height sensor is **Planned / Future**.
- **E-stop:** `lift_safety_node` subscribes to a project-wide
  `/emergency_stop` topic (source not defined in this package — presumably
  `forklift_safety`) and gates `/lift_enable`; this is part of the
  unlaunched auxiliary pipeline, so E-stop is **not currently enforced** on
  the active `lift_controller_node` path.

## 9. Launch Files and Run Commands

| Launch file | Starts |
|---|---|
| `forklift_controller.launch.py` | `lift_controller_node` only (namespace + `use_sim_time` arguments); all other nodes are present in a commented-out `make_node()` helper for future re-enabling |

```bash
# Single-robot lift controller
ros2 launch forklift_controller forklift_controller.launch.py

# Namespaced (multi-robot; typically launched by forklift_navigation's
# multiple_bringup_launch.py, not run standalone)
ros2 run forklift_controller lift_controller_node_multirobot \
  --ros-args -r __ns:=/forklift1 --params-file $(ros2 pkg prefix forklift_controller)/share/forklift_controller/config/controller.yaml

# Manual jog test
ros2 topic pub /lift_up std_msgs/msg/Bool "{data: true}" -1

# Precise positioning via the action
ros2 action send_goal /set_height forklift_interfaces/action/SetLiftHeight "{target_height: 0.1, speed: 0.05}"
```

## 10. Dependencies

From `package.xml`:

- `rclcpp`, `rclcpp_action`, `std_msgs`, `sensor_msgs`, `action_msgs`
- `forklift_interfaces` (for `SetLiftHeight.action`, `LiftMotorCommand.msg`,
  `LiftEncoderHeight.msg`, `LiftDriverStatus.msg`, `CurrentHeight.msg`,
  `LiftLimitStatus.msg`)
- `launch`, `launch_ros`, `ament_index_python`

## 11. Communication Flow

1. A jog command (`/lift_up`/`/lift_down`) or a `SetLiftHeight` action goal
   sets the controller's internal mode (`JOG` or `AUTO`) and target.
2. A 20 Hz control loop ramps `command_position_` toward the target at the
   configured speed, clamped to `[min_height, max_height]`, and publishes it
   on `lift_cmd`.
3. `forklift_gazebo`'s `ros_gz_bridge` forwards `lift_cmd` to Ignition's
   `JointPositionController` plugin, which drives the physical `lift_joint`
   in simulation.
4. `/joint_states` (published by Gazebo's `JointStatePublisher` plugin)
   feeds back the real simulated position, closing the loop for the
   `SetLiftHeight` action's success/feedback reporting.
5. A `SetLiftHeight` goal handler runs on its own detached thread,
   publishing feedback at 5 Hz and returning `succeed()`/`abort()`/
   `canceled()` once the target is reached, blocked by a limit, timed out,
   or canceled — never blocking the node's executor for the duration of the
   move.

## 12. Integration with Other Packages

- **forklift_description** — defines the `lift_joint` (prismatic, 0–0.15 m)
  this package commands; the URDF's true travel range should be the
  authoritative limit (see the `max_height` discrepancy noted in §7).
- **forklift_gazebo** — bridges `lift_cmd` to/from the Ignition
  `JointPositionController` plugin.
- **forklift_docking** — `pallet_dock_workflow_node` calls the `set_height`
  action (parameterized as `set_height_service_name`, default `"set_height"`)
  to raise/lower the fork during pallet pick/drop docking, with its own
  35 s client-side timeout.
- **forklift_task_manager** — `task_executor.cpp` also calls the
  `set_height` action (`lift_action_name` parameter, default `"set_height"`)
  as part of pick/drop task execution.
- **forklift_remote** — bypasses this package entirely for manual
  teleoperation, publishing directly to `lift_cmd` (position) rather than
  going through `lift_controller_node`'s jog/action interface.

## 13. Future Improvements

- Resolve the `max_height` mismatch between `config/controller.yaml`
  (`1.2` m), `lift_controller_node.cpp`'s in-code default (`0.12` m), and
  `forklift_description`'s actual URDF travel (`0.15` m).
- **Planned / Future:** wire in a real E-stop and limit-switch/ToF sensor
  chain (`lift_safety_node`, `lift_limit_node`, `lift_feedback_node`,
  `lift_driver_node`) — currently implemented but not launched, and the
  active `lift_controller_node` path has no hardware safety interlock.
- **Planned / Future:** implement the actual serial/CAN backend in
  `lift_driver_node` for `use_hardware:=true` (currently a no-op that falls
  back to simulation).
- **Planned / Future:** reconcile `forklift_remote`'s direct `lift_cmd`
  publishing with this package's jog/action interface, so manual and
  autonomous lift control don't bypass the same safety/limit logic
  differently.
- Add an `ExecuteLift`-style composite action (approach + raise + confirm)
  if higher-level packages need it, rather than each caller re-implementing
  the same `set_height` call pattern.
