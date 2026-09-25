# forklift_remote

ROS 2 (Humble) `ament_cmake` package providing manual teleoperation of the
Mini Forklift AMR: a keyboard driver and a PS4 (DualShock) joystick driver,
both controlling drive velocity and the fork lift position.

---

## 1. Overview and Responsibility

`forklift_remote` is the manual-control package. It converts either raw
keyboard input or a `sensor_msgs/Joy` message from a PS4 controller into a
`geometry_msgs/Twist` drive command and a `std_msgs/Float64` lift-position
command, with adjustable linear/angular speed scaling and lift travel
clamping. It has no awareness of navigation, docking, or tasks — it is a
pure human-in-the-loop input layer, primarily used for manual driving,
sim/hardware validation, and staging the robot near a marker/dock before
automated docking takes over.

## 2. Architecture / Role in the Overall System

```
        Keyboard (stdin)                PS4 controller
              │                               │
              │                        joy_node (package "joy")
              │                               │ sensor_msgs/Joy on "joy"
              ▼                               ▼
   keyboard_control_node              ps4_control_node
              │                               │
              └───────────────┬───────────────┘
                               │
                 geometry_msgs/Twist  ->  forklift/cmd_vel
                 std_msgs/Float64     ->  forklift/lift_cmd
                               │
                               ▼
                  forklift_gazebo (ros_gz_bridge) / forklift_controller
```

## 3. Directory Structure

```
forklift_remote/
├── src/
│   ├── keyboard_control_node.cpp   # stdin-based teleop node
│   └── ps4_control_node.cpp        # sensor_msgs/Joy-based teleop node
├── launch/
│   └── ps4_teleop.launch.py        # joy_node + ps4_control_node
├── CMakeLists.txt
└── package.xml
```

There is no dedicated launch file for the keyboard node; it is run directly
via `ros2 run`.

## 4. Nodes and Node Responsibilities

| Node | Executable | Responsibility |
|---|---|---|
| `KeyboardControlNode` | `keyboard_control_node` | Reads raw keypresses from stdin in a background thread (raw terminal mode via `termios`), maintains latched drive/lift state, and republishes it at 10 Hz. |
| `PS4ControlNode` | `ps4_control_node` | Subscribes to `sensor_msgs/Joy`, maps stick axes to drive velocity and button presses to speed adjustment / lift up-down, republishes at 10 Hz. |

Both nodes share the same output topics, speed-limiting behavior, and lift
clamping range, so only one should be run at a time to avoid two
publishers fighting over `forklift/cmd_vel` / `forklift/lift_cmd`.

## 5. Input / Output Topics

| Name | Type | Direction | Node(s) | Description |
|---|---|---|---|---|
| `joy` | `sensor_msgs/Joy` | Input | `ps4_control_node` | Raw joystick state from `joy_node` |
| stdin (raw keypresses) | n/a | Input | `keyboard_control_node` | Not a ROS topic; read directly from the terminal via `termios` |
| `forklift/cmd_vel` | `geometry_msgs/Twist` | Output | both | Drive velocity command |
| `forklift/lift_cmd` | `std_msgs/Float64` | Output | both | Target fork lift position (meters), bridged to Ignition's native `JointPositionController` plugin on `lift_joint` (documented in-source as the working path, after `ros2_control`'s velocity/position interfaces were found non-functional for this joint on the current `ign_ros2_control` build) |

**Note:** both nodes publish on the relative topic `forklift/cmd_vel`, while
`forklift_gazebo`'s single-robot bridge (`bridge_config.yaml`) maps the
Ignition DiffDrive plugin to the ROS topic `/cmd_vel` (no `forklift/` prefix).
Connecting this package's teleop output to the simulated robot in a
single-robot setup therefore requires either a topic remap at launch (e.g.
`--remap forklift/cmd_vel:=/cmd_vel`) or aligning the bridge/robot namespace
— this is not automatically resolved by any file in this package. Mark as a
configuration point to verify per deployment.

## 6. Services and Actions

None. Teleoperation is entirely topic-based.

## 7. Important Parameters

Neither node declares ROS parameters; tunables are compiled-in constants:

| Constant | Node | Value | Description |
|---|---|---|---|
| `speed_linear_` | keyboard | `0.6` (initial), step `0.1` via `q`/`z` | Linear speed scale |
| `speed_angular_` | keyboard | `1.0` (initial), step `0.1` via `e`/`c` | Angular speed scale |
| `speed_linear_` | ps4 | `0.3` (initial), step `0.1` via L1/L2 | Linear speed scale |
| `speed_angular_` | ps4 | `1.0` (initial), step `0.1` via R1/R2 | Angular speed scale |
| `limit_linear_speed_` / `limit_angular_speed_` | both | `1.5` m/s / `2.0` rad/s | Hard clamp applied to the final `Twist` command |
| `lift_lower_limit_` / `lift_upper_limit_` | both | `0.0` / `0.15` m | Must match `lift_lower_limit`/`lift_upper_limit` (`lift_travel`) in `forklift_description/urdf/const.xacro` |
| `lift_step_` | both | `0.005` m per publish tick (keyboard, while latched) / per joystick message (ps4, while held) | Lift position increment |
| Publish rate | both | 10 Hz (100 ms wall timer) | |

**Planned / Future:** expose these as ROS 2 parameters (`declare_parameter`)
instead of hardcoded member defaults, so speed/lift limits can be tuned per
robot without recompiling.

## 8. Keyboard and PS4 Control Mapping

**Keyboard (`keyboard_control_node`):**

| Key(s) | Action |
|---|---|
| `w` / `x` | Drive forward / backward |
| `a` / `d` | Turn left / right |
| `s` or space | Stop driving (lift holds position) |
| `q` / `z` | Increase / decrease linear speed |
| `e` / `c` | Increase / decrease angular speed |
| `u` / `j` | Raise / lower the fork lift (latches until `k` or the opposite key) |
| `k` | Stop adjusting lift (hold current position) |
| Ctrl+C | Quit |

**PS4 controller (`ps4_control_node`):**

| Input | Action |
|---|---|
| Left stick vertical (`axes[1]`) | Linear velocity |
| Right stick horizontal (`axes[3]`) | Angular velocity |
| L1 (`buttons[4]`) / L2 (`buttons[7]`) | Increase / decrease linear speed |
| R1 (`buttons[5]`) / R2 (`buttons[6]`) | Increase / decrease angular speed |
| Triangle (`buttons[2]`) | Raise fork lift |
| Cross (`buttons[0]`) | Lower fork lift |

## 9. Launch Files and Run Commands

| Launch file | Starts |
|---|---|
| `ps4_teleop.launch.py` | `joy_node` (package `joy`) + `ps4_control_node` |

```bash
# PS4 teleop
ros2 launch forklift_remote ps4_teleop.launch.py

# Keyboard teleop (no launch file — run directly, needs an interactive terminal)
ros2 run forklift_remote keyboard_control_node
```

## 10. Dependencies

From `package.xml`:

- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`
- `joy` (exec dependency, provides `joy_node` for the PS4 pipeline)

## 11. Communication Flow

1. **Keyboard path:** `keyboard_control_node` reads raw keypresses on a
   background thread (terminal set to non-canonical/no-echo mode) and
   updates shared, mutex-protected drive/lift state; a 10 Hz wall timer
   publishes the latest `Twist`/`Float64` command regardless of whether a
   new key was just pressed (so commands keep flowing while a key is held
   or latched).
2. **PS4 path:** `joy_node` publishes raw controller state on `joy`;
   `ps4_control_node`'s callback updates drive/lift state immediately, and
   the same 10 Hz wall timer pattern republishes the command.
3. Both nodes publish identical output topics (`forklift/cmd_vel`,
   `forklift/lift_cmd`), so the destination (Gazebo bridge, or a future
   hardware driver) is agnostic to which teleop source is active — only one
   should run at a time.

## 12. Integration with Other Packages

- **forklift_gazebo** — consumes `forklift/cmd_vel`/`forklift/lift_cmd` via
  `ros_gz_bridge` to drive the simulated DiffDrive and `lift_joint`
  `JointPositionController` plugins (subject to the topic-name note in
  §5 above).
- **forklift_controller** — the lift command produced here follows the same
  `std_msgs/Float64` convention as `forklift_controller`'s lift command path;
  this package bypasses `forklift_controller`'s lift services/actions
  entirely and drives the joint position directly.
- **forklift_description** — the hardcoded lift travel limits
  (`0.0`–`0.15` m) must be kept in sync with `const.xacro`'s
  `lift_lower_limit`/`lift_upper_limit` properties.

## 13. Future Improvements

- **Planned / Future:** reconcile the `forklift/cmd_vel` topic name used
  here with `forklift_gazebo`'s `/cmd_vel` bridge target (or make it
  configurable/namespaced) so teleop works out-of-the-box in both
  single- and multi-robot setups without a manual remap.
- **Planned / Future:** promote hardcoded speed/lift constants to ROS 2
  parameters.
- **Planned / Future:** add a dedicated keyboard teleop launch file (with
  parameters) instead of requiring a bare `ros2 run`.
- **Planned / Future:** add an explicit E-stop / deadman input, and
  namespace support for multi-robot teleop (selecting which robot's
  `cmd_vel`/`lift_cmd` to target).
