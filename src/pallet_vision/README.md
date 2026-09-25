# pallet_vision

ROS 2 (Humble) `ament_python` package implementing a standalone YOLOv11
(Ultralytics)-based pallet detector for the Mini Forklift AMR's RGB-D
camera. It is a self-contained perception experiment — it is **not**
consumed by `forklift_docking`, `forklift_task_manager`, or any other
package's pick/drop workflow, which instead rely entirely on
`forklift_aruco`'s marker-based docking pose.

---

## 1. Overview and Responsibility

`pallet_vision` subscribes to an RGB image, a depth image, and camera
intrinsics, runs a YOLOv11 detection model on the RGB frame, filters the
resulting boxes with a handful of heuristics (area ratio, vertical
position, aspect ratio) to reject floor/ground false positives, and
back-projects the best remaining detection's pixel center into a 3D pose
using the sampled depth. It publishes that pose, a normalized
pixel-offset/confidence point, a `Bool` detected flag, and an annotated
debug image.

It ships two near-duplicate node implementations
(`pallet_detection.py` and `pallet_detectionV2.py`), two unused/orphaned
scripts (`inference.py`, `publishImage.py`), and three committed model
checkpoints. **No launch file exists anywhere in this package** — every
node must be started with a bare `ros2 run`.

## 2. Architecture / Role in the Overall System

```
/forklift/camera/image_raw   (sensor_msgs/Image, RGB)
/forklift/camera/depth_image (sensor_msgs/Image, 32FC1 or 16UC1 depth)
/forklift/camera/camera_info (sensor_msgs/CameraInfo, intrinsics)
/forklift/camera/points      (sensor_msgs/PointCloud2, V2 only, optional depth fallback)
            │
            ▼
   ┌───────────────────────────┐
   │  pallet_detection(V2).py    │   YOLOv11 (Ultralytics) inference
   │  (message_filters synced,   │   + area/position/aspect filters
   │   ApproximateTimeSynchronizer)│  + pixel->3D back-projection
   └─────────────┬───────────────┘
                 │
   /pallet/pose          (geometry_msgs/PoseStamped)
   /pallet/pixel_center   (geometry_msgs/PointStamped: norm_x, norm_y, confidence)
   /pallet/debug_image    (sensor_msgs/Image, annotated frame)
   /pallet/detected       (std_msgs/Bool)

   ── Nothing in this workspace subscribes to any of the four topics
      above. forklift_task_manager / forklift_docking's pick/drop flow
      uses forklift_aruco's marker-based pose instead — this package's
      output is not wired into the actual docking pipeline. ──
```

## 3. Directory Structure

```
pallet_vision/
├── pallet_vision/
│   ├── pallet_detection.py     # v1 node -- image+depth only, weighted composite score
│   ├── pallet_detectionV2.py   # v2 node -- adds QoS param + PointCloud2 depth fallback
│   ├── inference.py            # orphaned node -- console_scripts entry commented out (see §12)
│   ├── publishImage.py         # orphaned node -- console_scripts entry commented out (see §12)
│   ├── detectionUtil.py        # SingleImageDetection helper class, used by inference.py only
│   ├── segmentationUtil.py     # SingleImageSegInference helper class, used by inference.py only
│   └── model_checkpoints/
│       ├── best_detect.pt      # detection model, default for both live nodes
│       ├── best_segment.pt     # segmentation model, used only by inference.py (orphaned)
│       └── detection_model.pt  # unreferenced by any node -- purpose/origin undocumented
├── resource/pallet_vision
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── LICENSE
├── package.xml
└── setup.py
```

No `launch/` directory exists in this package.

## 4. Nodes and Node Responsibilities

| Node | Executable | Registered? | Responsibility |
|---|---|---|---|
| `PalletDetector` (v1) | `pallet_detection` | Yes (`console_scripts`) | Synchronizes RGB + depth (`ApproximateTimeSynchronizer`, no PointCloud2 fallback), runs YOLOv11 detection, filters candidate boxes by area ratio / center-Y ratio (floor rejection) / aspect ratio, then scores survivors with a weighted composite (`0.5*confidence + 0.3*center_distance_score + 0.2*size_score`) and picks the highest-scoring box — not simply the highest-confidence one. Publishes pose/pixel-center/debug-image/detected. |
| `PalletDetector` (v2) | `pallet_detection2` | Yes (`console_scripts`) | Same filtering pipeline as v1, but (a) exposes a `qos_reliability` parameter (`best_effort`/`reliable` — important because Gazebo Fortress sensor topics are commonly `BEST_EFFORT`, and a mismatched `RELIABLE` subscriber silently receives nothing), (b) optionally subscribes to a `PointCloud2` and falls back to sampling XYZ directly from it if the depth-image sample at the pallet center is invalid, and (c) selects the best box by **highest confidence only** (no composite center/size scoring, unlike v1). |
| `InferenceNode` | `inference_node` | **No** — `console_scripts` entry is commented out in `setup.py` | Loads both a segmentation (`best_segment.pt`) and a detection (`best_detect.pt`) model via `detectionUtil`/`segmentationUtil`, subscribing to a hardcoded `/robot1/zed2i/left/image_rect_color` topic (a different camera naming convention than the live nodes' `/forklift/camera/*`, suggesting this predates or targets a different robot/bag setup). **Has a real bug**: it resolves its model directory with `get_package_share_directory('pallet')` — the wrong package name; this package is `pallet_vision`, so calling this would raise `PackageNotFoundError` even if it were registered. Its shebang (`#!/home/yunboyloy/miniconda3/envs/pallet_vision/bin/python3`) is also a hardcoded, machine-specific conda path. |
| `ImagePublisher` | `image_publisher` | **No** — `console_scripts` entry is commented out in `setup.py` | Republishes `.jpg` files from a hardcoded local directory (`~/work_no_backup/rosws/src/data/test/images/`) on `rgb_images` at 1 Hz, for offline testing without a live camera. Directory path is developer-machine-specific and does not exist in this workspace. |

## 5. Input/Output Topics

| Name | Type | Direction | Node(s) | Description |
|---|---|---|---|---|
| `/forklift/camera/image_raw` (param `image_topic`) | `sensor_msgs/Image` | Input | `pallet_detection`, `pallet_detection2` | RGB source |
| `/forklift/camera/depth_image` (param `depth_topic`) | `sensor_msgs/Image` | Input | `pallet_detection`, `pallet_detection2` | 32FC1 (meters) or 16UC1 (mm, auto-converted) depth |
| `/forklift/camera/camera_info` (param `camera_info_topic`) | `sensor_msgs/CameraInfo` | Input | `pallet_detection`, `pallet_detection2` | Intrinsics (`fx`,`fy`,`cx`,`cy`) for pixel→3D back-projection |
| `/forklift/camera/points` (param `points_topic`) | `sensor_msgs/PointCloud2` | Input | `pallet_detection2` only | Optional organized-cloud fallback when the depth-image sample is invalid; requires `sensor_msgs_py` and `height > 1` (organized cloud) |
| `/pallet/pose` | `geometry_msgs/PoseStamped` | Output | both | 3D position of the pallet center in `camera_frame` (default `forklift/camera_link`); orientation is always identity (`w=1`) — no orientation estimate is computed |
| `/pallet/pixel_center` | `geometry_msgs/PointStamped` | Output | both | `x`/`y` = normalized pixel offset from frame center (±1 range), `z` = detection confidence (an unconventional reuse of a `PointStamped`'s `z` field) |
| `/pallet/debug_image` | `sensor_msgs/Image` | Output | both | Annotated BGR frame: bounding box, center dot, crosshair, lateral-error bar, ground-rejection line, depth thumbnail overlay |
| `/pallet/detected` | `std_msgs/Bool` | Output | both | `true` iff a box survived all filters this frame |
| `rgb_images` | `sensor_msgs/Image` | Output | `publishImage.py` (unregistered) | Republished test images |

## 6. Services and Actions

None.

## 7. Important Parameters

Both `pallet_detection` and `pallet_detection2` declare (v2-only
differences noted):

| Parameter | v1 default | v2 default | Description |
|---|---|---|---|
| `model_path` | `<share>/models/best_detect.pt` | same | Resolved via `get_package_share_directory('pallet_vision')` — **correct** package name in the live nodes (unlike `inference.py`'s bug) |
| `confidence_threshold` | `0.6` | `0.65` | YOLO confidence cutoff |
| `input_size` | `640` | `640` | YOLO inference size |
| `max_box_area_ratio` / `min_box_area_ratio` | `0.8` / `0.01` | `0.6` / `0.01` | Reject boxes covering too much/too little of the frame |
| `max_center_y_ratio` | `0.65` | `0.65` | Reject boxes whose center is in the bottom 35% of the frame (floor/ground rejection) |
| `min_aspect_ratio` | `0.4` | `0.4` | Reject portrait-oriented boxes (pallet faces are wide, not tall) |
| `target_class_id` | `-1` (any class) | `-1` | Restrict to one YOLO class if set |
| `camera_frame` | `forklift/camera_link` | same | Frame ID stamped on published poses |
| `depth_sample_radius` | `5` px | `5` px | Median-filtered patch radius around the pixel center for depth sampling |
| `max_depth_m` / `min_depth_m` | `5.0` / `0.05` | same | Valid depth range |
| `sync_slop` | `0.1` s | `0.1` s | `ApproximateTimeSynchronizer` tolerance |
| `qos_reliability` | — (not present) | `best_effort` | v2 only — `best_effort` or `reliable` for all three camera subscriptions |
| `use_pointcloud_fallback` | — (not present) | `true` | v2 only — enables the `PointCloud2` depth fallback |

## 8. Launch Files and How to Run Them

**No launch file exists in this package.** Both nodes must be started
directly:

```bash
ros2 run pallet_vision pallet_detection
# or the v2 variant with QoS/pointcloud-fallback support
ros2 run pallet_vision pallet_detection2 --ros-args \
  -p qos_reliability:=best_effort \
  -p use_pointcloud_fallback:=true

# Inspect output
ros2 topic echo /pallet/pose
ros2 topic echo /pallet/detected
```

## 9. Dependencies

From `package.xml`:

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`,
  `python3-opencv`

**Not declared**, despite being imported by the live nodes:
`ultralytics` (YOLO), `message_filters`, `numpy`, `ament_index_python`.
`sensor_msgs_py` is imported defensively (`try`/`except ImportError`) by
`pallet_detectionV2.py` for its `PointCloud2` fallback and is also not
declared. None of these are `rosdep`-resolvable Python packages via
`package.xml` in the way this workspace's other packages declare their
Python deps — anyone building this package fresh would need to
`pip install ultralytics message_filters numpy` (and
`ros-humble-sensor-msgs-py` for the fallback) manually, matching the
node's own runtime warning if `sensor_msgs_py` is missing.

Both live nodes' shebang lines
(`#!/home/yunboyloy/miniconda3/envs/pallet_vision/bin/python3`) hardcode a
specific developer machine's conda environment path — this only matters
if the file is executed directly rather than via `ros2 run`
(`ros2 run` invokes it through the Python interpreter that built the
workspace, not via the shebang).

## 10. Model Checkpoints

Three `.pt` files are committed directly in
`pallet_vision/model_checkpoints/` and installed to
`share/pallet_vision/models/` by `setup.py`:

| File | Used by |
|---|---|
| `best_detect.pt` | `pallet_detection.py` and `pallet_detectionV2.py` (default `model_path`) |
| `best_segment.pt` | `inference.py` only (orphaned, unregistered node) |
| `detection_model.pt` | Not referenced by any node in this package |

## 11. Integration with the Rest of the System

- **None currently.** No other package subscribes to `/pallet/pose`,
  `/pallet/pixel_center`, `/pallet/detected`, or `/pallet/debug_image`.
  `forklift_docking`/`forklift_task_manager`'s pallet pick/drop workflow
  is driven entirely by `forklift_aruco`'s ArUco marker detection
  (`aruco_dock_pose`/`target_marker_id`), which is a completely separate
  vision pipeline from this package.
- **forklift_safety** — `obstacle_monitor_node`'s own source comment
  explicitly notes that full point-cloud/segmentation-based obstacle
  detection "belongs in a dedicated perception node (e.g.
  `forklift_pallet_vision`), not here" — a naming mismatch (that comment
  refers to a `forklift_pallet_vision` package that doesn't exist; this
  package is named `pallet_vision`), and in any case no such integration
  exists today.

## 12. Known Issues / TODO / Future Work

- **Not integrated into the docking/task pipeline** — this is a
  standalone perception experiment; if pallet-relative (rather than
  marker-relative) docking is ever wanted, `/pallet/pose` would need to be
  consumed by `forklift_docking` or `forklift_task_manager`, and its
  accuracy/robustness validated against the ArUco-based approach
  currently in production use.
- **No launch file** — both nodes require manual `ros2 run` with
  hand-specified parameters; there is no `launch/` directory at all.
- **`inference.py` has a real, live bug**: `get_package_share_directory
  ('pallet')` uses the wrong package name (should be `'pallet_vision'`) —
  calling this code today raises `PackageNotFoundError`. Its
  `console_scripts` entry is commented out, so this has gone unnoticed.
- **`inference.py` and `publishImage.py` are orphaned** — fully written,
  but their `console_scripts` entries are commented out in `setup.py`, so
  neither is installed as a runnable executable. `inference.py` also
  subscribes to a different, hardcoded camera topic
  (`/robot1/zed2i/left/image_rect_color`) that doesn't match this
  package's `/forklift/camera/*` convention, suggesting it targets a
  different robot/dataset than the current sim setup.
- **Two near-duplicate node implementations** (`pallet_detection.py` vs.
  `pallet_detectionV2.py`) with subtly different box-selection logic
  (weighted composite score vs. highest-confidence-only) and no shared
  base class — a bugfix or tuning change to one is easy to forget to
  apply to the other.
- **`detection_model.pt` is unreferenced** — unclear if it's a leftover,
  a planned default, or safe to delete.
- **Missing Python dependencies in `package.xml`**: `ultralytics`,
  `message_filters`, `numpy`, `ament_index_python`, and (optionally)
  `sensor_msgs_py` are all imported but not declared — see §9.
- **Hardcoded, machine-specific paths**: both live nodes' shebang lines
  point at a specific developer's conda environment
  (`/home/yunboyloy/miniconda3/envs/pallet_vision/...`), and
  `publishImage.py`'s image directory
  (`~/work_no_backup/rosws/src/data/test/images/`) is similarly
  machine-specific.
- **`/pallet/pixel_center`'s `PointStamped.z` reused for confidence** is
  a slightly unconventional message choice (confidence isn't a spatial
  coordinate) — a custom message with named fields would be clearer for
  any future consumer.
