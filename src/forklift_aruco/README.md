# forklift_aruco

Vision-based marker detection and pose estimation package for the Mini Forklift AMR Prototype:
detects ArUco fiducial markers in the front camera image, estimates their 3D pose relative to
the robot, and republishes a single filtered "dock target" pose for precision docking alignment.

> Platform: ROS 2 Humble + Gazebo Fortress

## 1. Overview and Responsibility

`forklift_aruco` is the perception layer that turns a camera image into a usable dock/pallet
alignment pose. Its main node, `aruco_node`, performs the full pipeline:

**Camera image → ArUco detection → pose estimation → TF-equivalent frame transform → precision
alignment target.**

Specifically it:
1. Subscribes to the RGB camera image (and optionally a synchronized depth image).
2. Runs OpenCV's `cv2.aruco.ArucoDetector` with detector parameters explicitly widened from
   OpenCV's defaults (documented in-source as the fix for markers that were visible but never
   detected — see §7).
3. Estimates each detected marker's 3D pose via `cv2.solvePnP` (or a depth-centroid translation
   when `use_depth_input` is enabled), corrected for a marker-axis convention fix so that "facing
   the robot squarely" reads as 0° yaw.
4. Transforms every marker pose from the camera optical frame into `base_link` (or further into
   `map`/`odom` if live odometry is available), using an explicit 4×4 homogeneous matrix chain
   rather than the `tf2` library.
5. Publishes the full multi-marker array, and separately republishes **only** the marker matching
   a live-settable `target_marker_id` parameter as a single `PoseStamped` on a dedicated
   "dock pose" topic — the actual signal consumed by the docking system for precision alignment.

A family of standalone CLI utility scripts (`generate_marker_image.py`,
`generate_marker_cube.py`, `calibration_camera.py`, `detect_aruco_image.py`,
`detect_aruco_video.py`, `pose_estimate.py`) support marker generation, camera calibration, and
offline/video-only detection testing outside the main ROS graph.

## 2. Architecture / Role in the Overall System

```
camera/image_raw, camera/camera_info, [camera/depth_image] ──▶ aruco_node
                                                                    │
                                          cv2.aruco.ArucoDetector + solvePnP
                                                                    │
                                     T_camera_to_base (constant, from URDF mount params)
                                     T_base_to_map (dynamic, from /odom, if output_frame=map)
                                                                    │
                        ┌───────────────────────────────────────────┼──────────────────────┐
                        ▼                                           ▼                       ▼
              camera/aruco/markers                         camera/aruco/image       aruco_dock_pose
           (ArucoMarkers: all detected)               (annotated debug image)   (PoseStamped: target_marker_id only)
                                                                                              │
                                                                                              ▼
                                                                          forklift_docking (SimpleChargingDock's
                                                                          fixed "detected_dock_pose" topic, via remap)
```

`aruco_node`'s `target_marker_id` parameter is the selection mechanism: `opennav_docking` has no
concept of marker IDs and consumes a single, unlabeled pose stream, so whichever downstream
package is about to dock (`forklift_task_manager`, `forklift_docking`) sets
`target_marker_id` at runtime (via `ros2 param set`, driven by the relevant dock's `id` field in
`dock_database.yaml`) immediately before sending a docking goal.

## 3. Directory Structure

```
forklift_aruco/
├── config/
│   └── aruco_parameters.yaml       # aruco_node's default parameters (relative topic names)
├── forklift_aruco/
│   ├── aruco_node.py               # main perception node (detection + pose + dock_pose filter)
│   ├── pose_estimation.py          # solvePnP wrapper, frame-transform math, drawing helpers
│   ├── aruco_dicts.py, utils.py    # ArUco dictionary table, aruco_display() overlay helper
│   ├── ros_image_utils.py
│   ├── generate_marker_image.py    # CLI: render a single marker to an image file
│   ├── generate_marker_cube.py     # CLI: render markers onto a cube net
│   ├── calibration_camera.py       # CLI: camera intrinsic calibration
│   ├── detect_aruco_image.py       # CLI/node: detect markers in a static image
│   ├── detect_aruco_video.py       # node: detect markers in a live video/camera stream
│   └── pose_estimate.py            # node: pose estimation companion to detect_aruco_video
├── launch/
│   ├── aruco_pose_estimation.launch.py    # detect_aruco_video + pose_estimate (test/legacy pair)
│   └── aruco_pose_estimation2.launch.py   # aruco_node (the active, full-pipeline launch)
└── test/                            # ament copyright/flake8/pep257 lint tests
```

## 4. Nodes and Node Responsibilities

| Node | Executable | Responsibility |
| ---- | ---------- | --------------- |
| `ArucoNode` | `aruco_node` | Primary node: subscribes to camera image(+depth)/info, detects ArUco markers, estimates and transforms their poses, publishes the full marker array, an annotated debug image, and the filtered single-target dock pose. Supports live `target_marker_id` updates via a parameter callback. |
| `detect_aruco_video` | `detect_aruco_video` | Standalone/legacy node: marker detection over a live video stream, launched alongside `pose_estimate` in `aruco_pose_estimation.launch.py` |
| `pose_estimate` | `pose_estimate` | Standalone/legacy pose-estimation companion node to `detect_aruco_video` |
| `detect_aruco_image` | `detect_aruco_image` | CLI utility: detect markers in a single static image |
| `generate_marker_image` | `generate_marker_image` | CLI utility: render an ArUco marker to an image file |
| `generate_marker_cube` | `generate_marker_cube` | CLI utility: render markers onto a foldable cube net (for a physical calibration/test target) |
| `calibration_camera` | `calibration_camera` | CLI utility: camera intrinsic/distortion calibration |

`aruco_pose_estimation2.launch.py` — the launch file that starts the full-pipeline `aruco_node`
with all parameters exposed as launch arguments — is the one referenced by other packages
(`forklift_task_manager`'s parameter client targets a node literally named `aruco_node`).

## 5. Input Topics and Output Topics

| Name (parameter) | Type | Direction | Description |
| ----------------- | ---- | --------- | ----------- |
| `camera/image_raw` (`image_topic`) | `sensor_msgs/msg/Image` | Input | RGB camera stream |
| `camera/camera_info` (`camera_info_topic`) | `sensor_msgs/msg/CameraInfo` | Input | Camera intrinsics/distortion; subscription is destroyed after the first message |
| `camera/depth_image` (`depth_image_topic`) | `sensor_msgs/msg/Image` | Input (optional, `use_depth_input=true`) | Depth image, time-synchronized with the RGB image via `message_filters.ApproximateTimeSynchronizer` (slop 0.05s) |
| `odom` (`odom_topic`) | `nav_msgs/msg/Odometry` | Input (optional, `output_frame=map`) | Robot pose used to transform marker poses into `map`/`odom` instead of `base_link` |
| `camera/aruco/markers` (`detected_markers_topic`) | `forklift_interfaces/msg/ArucoMarkers` | Output | All detected markers this frame: `marker_ids[]` + `poses[]` |
| `aruco/poses` (`markers_visualization_topic`) | `geometry_msgs/msg/PoseArray` | Output | Same poses as a bare `PoseArray`, for RViz visualization |
| `aruco/image_raw` (`output_image_topic`) | `sensor_msgs/msg/Image` | Output | Debug image with marker outlines, axes, and per-marker pose text overlay |
| `aruco_dock_pose` (`dock_pose_topic`) | `geometry_msgs/msg/PoseStamped` | Output | **The** docking-relevant output: only published when the currently-visible markers include `target_marker_id`; nothing is published otherwise |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | (declared, `publish_tf: true` in config) | Config declares TF broadcasting; the launch file remaps `/tf`→`tf` per the project's multi-robot convention — actual TF broadcast is not implemented in `aruco_node.py`'s current source (poses are computed via explicit matrix math, not `tf2_ros`) — **Planned / Future** |

## 6. Services and Actions

None. `aruco_node` exposes no ROS 2 services or actions; its only externally-controllable
runtime input is the `target_marker_id` parameter, settable live via
`ros2 param set /<namespace>/aruco_node target_marker_id <id>` through its
`add_on_set_parameters_callback`.

## 7. Important Parameters

| Parameter | Default | Description |
| --------- | ------- | ------------ |
| `marker_size` | `0.04` m (config) / `0.0625` (code default) | Physical marker edge length, used in `solvePnP`'s object points |
| `aruco_dictionary_id` | `DICT_4X4_100` | OpenCV predefined ArUco dictionary |
| `use_depth_input` | `false` | If `true`, uses `depth_to_pointcloud_centroid` (depth-image region average) as the translation source instead of `solvePnP`'s `tvec`, keeping `solvePnP`'s rotation |
| `image_topic`, `depth_image_topic`, `camera_info_topic` | `camera/image_raw`, `camera/depth_image`, `camera/camera_info` | Relative (namespace-safe) input topic names |
| `camera_frame` | `camera_link` (config) | Falls back to `camera_info`'s `frame_id` if empty; kept bare (unprefixed) since this project isolates multi-robot TF via the `/tf`→`tf` remap, not frame-name prefixes |
| `odom_topic` | `odom` | Odometry source when `output_frame=map` |
| `output_frame` | `base_link` | `"base_link"` or `"map"`; invalid values fall back to `base_link` with a warning |
| `camera_mount_xyz` / `camera_mount_rpy` | `[0,0,0.1]` / `[0,0,0]` | Camera mount pose in `base_link`, used to build the constant `T_camera_to_base` matrix once at startup |
| `target_marker_id` | `-1` | ID of the marker currently relevant for docking; `-1` disables `dock_pose_topic` output entirely. Set at runtime by the requesting package right before a docking goal |
| `detected_markers_topic`, `markers_visualization_topic`, `output_image_topic`, `dock_pose_topic` | `camera/aruco/markers`, `aruco/poses`, `aruco/image_raw`, `aruco_dock_pose` | Output topic names (all relative) |
| `publish_tf` | `true` | Declared in `aruco_parameters.yaml` but not currently acted on in `aruco_node.py` — see §5 note |

### Detection tuning (from in-source comments)

The stock `cv2.aruco.DetectorParameters()` defaults were confirmed (via multi-robot testing) to
be the actual root cause of markers never being detected despite being visible in frame — not a
topic/namespace wiring bug. The following were widened:

| Parameter | Tuned value | Reason |
| --------- | ----------- | ------ |
| `adaptiveThreshWinSizeMin/Max/Step` | `3` / `53` / `4` | Wider adaptive-threshold search window covers more lighting/contrast conditions |
| `minMarkerPerimeterRate` | `0.01` | Accepts smaller markers relative to image size (distant markers) |
| `polygonalApproxAccuracyRate` | `0.08` | Tolerates a less-perfectly-square outline (motion blur, compression) |
| `errorCorrectionRate` | `0.8` | More tolerant of misread bits per frame |
| `adaptiveThreshConstant` | `5` | More sensitive to subtle contrast |
| `cornerRefinementMethod` | `CORNER_REFINE_SUBPIX` | Improves pose accuracy once a marker is found |

### Pose correction (from in-source comments)

A fixed rotation matrix, `R_MARKER_AXIS_FIX`, is applied to every detected marker's rotation
before building its quaternion. This was derived empirically from a real frontal-view detection
that reported ≈−82.2° yaw when 0° was expected — the correction is a one-time change of basis
(not per-frame), valid at all viewing angles, that makes "marker facing the robot squarely" read
as 0° yaw for downstream docking alignment logic.

## 8. Launch Files and Run Commands

| Launch file | Description |
| ----------- | ------------ |
| `aruco_pose_estimation2.launch.py` | **Active/full-pipeline launch.** Starts `aruco_node` with every parameter exposed as a launch argument (defaults loaded from `config/aruco_parameters.yaml`), supports `namespace` for multi-robot use, and remaps `/tf`/`/tf_static` to the namespace-relative convention |
| `aruco_pose_estimation.launch.py` | Starts `detect_aruco_video` + `pose_estimate` — an older/parallel two-node detection+pose pair, not the primary pipeline |

```bash
# Full pipeline
ros2 launch forklift_aruco aruco_pose_estimation2.launch.py

# Multi-robot
ros2 launch forklift_aruco aruco_pose_estimation2.launch.py namespace:=forklift1

# Select the dock marker to track before sending a docking goal
ros2 param set /aruco_node target_marker_id 2

# CLI utilities
ros2 run forklift_aruco generate_marker_image
ros2 run forklift_aruco calibration_camera
```

## 9. Dependencies

From `package.xml`: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `visualization_msgs`,
`cv_bridge`, `tf2_ros`, `python3-opencv`, `python3-yaml`, `python3-numpy`, `forklift_interfaces`
(`ArucoMarkers` message). Python-level (not in `package.xml` but imported):
`message_filters`, `tf_transformations`.

## 10. Communication Flow

1. `aruco_node` subscribes to the camera image/info (and optionally depth), building the
   constant `T_camera_to_base` transform once at startup from `camera_mount_xyz`/`camera_mount_rpy`.
2. On each image, `pose_estimation()` runs marker detection, per-marker `solvePnP` (or depth
   centroid), applies the axis-convention fix, and transforms each pose through
   `T_camera_to_base` (and `T_base_to_map` if `output_frame=map` and odometry is available).
3. If any markers were detected, the full array is published on
   `camera/aruco/markers`/`aruco/poses`, and `_publish_dock_pose()` checks whether
   `target_marker_id` is among them — if so, republishes that single pose on `aruco_dock_pose`.
4. An annotated debug image (marker outlines, pose axes, text overlay) is published every frame
   regardless of detection result.
5. `target_marker_id` can be changed at any time via a ROS 2 parameter set, without restarting
   the node, so the same running `aruco_node` instance can be re-targeted at successive docks.

## 11. Integration with Other Packages

- **`forklift_description`** — `camera_link` is mounted on `lift_link` (not `base_link`
  directly), meaning the camera's height changes with fork position; `camera_mount_xyz`/`_rpy`
  should reflect this if precise `base_link`-frame poses are required at non-zero lift heights.
- **`forklift_gazebo`** — supplies the simulated `camera/image_raw`, `camera/camera_info`, and
  `camera/depth_image` topics via `ros_gz_bridge`, and the `charger_station`/`aruco_box` models'
  embedded ArUco marker textures are what this node detects in simulation.
- **`forklift_docking`** — the primary consumer: `aruco_dock_pose` is remapped in
  `forklift_docking`'s launch file into `opennav_docking`'s `SimpleChargingDock` plugin's fixed,
  non-configurable `detected_dock_pose` topic.
- **`forklift_task_manager`** — sets `target_marker_id` via a parameter client
  (`AsyncParametersClient` targeting `aruco_node`) immediately before executing a `DOCK` step,
  using the `marker_id` field from the dock being approached in `dock_database.yaml`.

## 12. Future Improvements (Planned / Future)

- `publish_tf: true` is declared in `config/aruco_parameters.yaml` but `aruco_node.py`'s current
  source does not broadcast any TF frames — poses are transformed via explicit NumPy matrix math
  (`build_camera_to_base_matrix`, `marker_pose_in_base`/`marker_pose_in_map`), not `tf2_ros`.
  Wiring an actual `tf2_ros.TransformBroadcaster` would let RViz and other TF consumers see
  detected marker frames directly.
- `dock_pose_topic` only ever republishes the single `target_marker_id` marker; no smoothing,
  outlier rejection, or multi-frame filtering is applied to the pose used for precision alignment.
- `use_depth_input`'s depth-centroid path (`depth_to_pointcloud_centroid`) is a simple bounding-
  region average without robust outlier handling; a mis-segmented region (e.g. an object
  partially occluding the marker) would bias the centroid.
- The two `aruco_pose_estimation*.launch.py` files represent two different, seemingly
  overlapping pipelines (`aruco_node` vs. `detect_aruco_video`+`pose_estimate`); consolidating or
  clearly deprecating one would reduce confusion.
- No automated pose-accuracy test — the marker-axis correction (`R_MARKER_AXIS_FIX`) and
  detection-parameter tuning were both derived empirically and are not covered by regression
  tests.
