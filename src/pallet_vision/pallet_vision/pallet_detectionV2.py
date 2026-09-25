#!/home/yunboyloy/miniconda3/envs/pallet_vision/bin/python3
"""
YOLOv11-based pallet detector for ROS 2 Humble — Gazebo Fortress simulation.

Subscribes:
  /forklift/camera/image        (sensor_msgs/Image)       RGB image
  /forklift/camera/depth_image  (sensor_msgs/Image)       32FC1 depth in metres
  /forklift/camera/camera_info  (sensor_msgs/CameraInfo)  intrinsics
  /forklift/camera/points       (sensor_msgs/PointCloud2) optional fallback depth
                                                           (organized cloud only)

Publishes:
  /pallet/pose         (geometry_msgs/PoseStamped)  3D position of pallet centre
  /pallet/pixel_center (geometry_msgs/PointStamped) normalised pixel offset + conf
  /pallet/debug_image  (sensor_msgs/Image)           annotated frame
  /pallet/detected     (std_msgs/Bool)               True when pallet visible

Changes vs previous version:
  - Configurable QoS (best_effort/reliable) for the camera subscriptions.
    Gazebo sensor topics are commonly published BEST_EFFORT; a mismatched
    RELIABLE subscriber silently receives nothing, which looks like "no
    detections" but is actually "no data ever arrived".
  - Optional PointCloud2 fallback: if the depth image gives an invalid
    (NaN / out-of-range) sample at the pallet centre, the node tries to
    pull XYZ directly from the organized point cloud at that same pixel
    before giving up on the frame.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import message_filters
import numpy as np
import cv2
import os
import math

try:
    import sensor_msgs_py.point_cloud2 as pc2
    _HAVE_PC2 = True
except ImportError:
    _HAVE_PC2 = False


class PalletDetector(Node):
    def __init__(self):
        super().__init__('pallet_detector')

        # ── Parameters ────────────────────────────────────────────────────────
        default_model = os.path.join(
            get_package_share_directory('pallet_vision'),
            'models', 'best_detect.pt'
        )
        self.declare_parameter('model_path',            default_model)
        self.declare_parameter('confidence_threshold',  0.65)
        self.declare_parameter('input_size',            640)

        # ── Box area filters ───────────────────────────────────────────────────
        # Reject boxes covering more than 60% or less than 1% of the frame
        self.declare_parameter('max_box_area_ratio',    0.6)
        self.declare_parameter('min_box_area_ratio',    0.01)

        # ── Box position filter ────────────────────────────────────────────────
        # Reject detections whose CENTRE Y is below this fraction of frame height
        # (0.65 = reject anything whose centre is in the bottom 35% of the frame)
        # This blocks floor/ground false positives -- the pallet face should
        # always appear in the upper portion of the frame for a forklift approach.
        self.declare_parameter('max_center_y_ratio',    0.65)

        # ── Aspect ratio filter ────────────────────────────────────────────────
        # A pallet face is typically wide-and-short (width > height).
        # Reject boxes that are taller than wide (portrait aspect ratio).
        # min_aspect_ratio = width/height -- e.g. 0.5 allows up to 2:1 portrait,
        # set higher (e.g. 1.0) to require landscape boxes only.
        self.declare_parameter('min_aspect_ratio',      0.4)

        self.declare_parameter('target_class_id',       -1)
        self.declare_parameter('camera_frame',          'forklift/camera_link')
        self.declare_parameter('depth_sample_radius',   5)
        self.declare_parameter('max_depth_m',           5.0)
        self.declare_parameter('min_depth_m',           0.05)
        self.declare_parameter('image_topic',           '/forklift/camera/image_raw')
        self.declare_parameter('depth_topic',           '/forklift/camera/depth_image')
        self.declare_parameter('camera_info_topic',     '/forklift/camera/camera_info')
        self.declare_parameter('points_topic',          '/forklift/camera/points')
        self.declare_parameter('sync_slop',             0.1)

        # ── QoS ───────────────────────────────────────────────────────────────
        # 'best_effort' matches typical Gazebo Fortress sensor QoS.
        # 'reliable' matches the default rclpy QoS most non-sim publishers use.
        self.declare_parameter('qos_reliability',       'best_effort')

        # ── PointCloud2 depth fallback ───────────────────────────────────────
        self.declare_parameter('use_pointcloud_fallback', True)

        model_path        = self.get_parameter('model_path').value
        self.conf_thresh  = self.get_parameter('confidence_threshold').value
        self.imgsz        = self.get_parameter('input_size').value
        self.max_area     = self.get_parameter('max_box_area_ratio').value
        self.min_area     = self.get_parameter('min_box_area_ratio').value
        self.max_cy_ratio = self.get_parameter('max_center_y_ratio').value
        self.min_aspect   = self.get_parameter('min_aspect_ratio').value
        self.target_cls   = self.get_parameter('target_class_id').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.depth_radius = self.get_parameter('depth_sample_radius').value
        self.max_depth    = self.get_parameter('max_depth_m').value
        self.min_depth    = self.get_parameter('min_depth_m').value
        image_topic       = self.get_parameter('image_topic').value
        depth_topic       = self.get_parameter('depth_topic').value
        info_topic        = self.get_parameter('camera_info_topic').value
        points_topic      = self.get_parameter('points_topic').value
        sync_slop         = self.get_parameter('sync_slop').value
        qos_reliability   = self.get_parameter('qos_reliability').value
        self.use_pc_fallback = bool(self.get_parameter('use_pointcloud_fallback').value)

        if self.use_pc_fallback and not _HAVE_PC2:
            self.get_logger().warn(
                'sensor_msgs_py.point_cloud2 not importable -- disabling '
                'pointcloud depth fallback. (sudo apt install ros-humble-sensor-msgs-py)'
            )
            self.use_pc_fallback = False

        # ── Model ─────────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info(f'Model ready. Classes: {self.model.names}')

        # ── Camera intrinsics ─────────────────────────────────────────────────
        self.fx = self.fy = self.cx = self.cy = None

        self.bridge = CvBridge()
        self._sync_confirmed = False

        # ── QoS profile ───────────────────────────────────────────────────────
        reliability = (QoSReliabilityPolicy.BEST_EFFORT
                       if qos_reliability.lower() == 'best_effort'
                       else QoSReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.get_logger().info(f'Subscribing with QoS reliability={qos_reliability}')

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo, info_topic, self.camera_info_cb, sensor_qos
        )

        rgb_sub   = message_filters.Subscriber(self, Image, image_topic,   qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic,   qos_profile=sensor_qos)

        if self.use_pc_fallback:
            points_sub = message_filters.Subscriber(self, PointCloud2, points_topic, qos_profile=sensor_qos)
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub, points_sub], queue_size=10, slop=sync_slop
            )
            self._sync.registerCallback(self.image_depth_points_cb)
            self.get_logger().info(f'Point cloud fallback ENABLED on {points_topic}')
        else:
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub], queue_size=10, slop=sync_slop
            )
            self._sync.registerCallback(self.image_depth_cb)
            self.get_logger().info('Point cloud fallback DISABLED')

        # ── Publishers ────────────────────────────────────────────────────────
        self.pose_pub   = self.create_publisher(PoseStamped,  '/pallet/pose',        10)
        self.center_pub = self.create_publisher(PointStamped, '/pallet/pixel_center', 10)
        self.debug_pub  = self.create_publisher(Image,        '/pallet/debug_image',  10)
        self.detect_pub = self.create_publisher(Bool,         '/pallet/detected',     10)

        self.get_logger().info('Pallet detector started.')
        self.get_logger().info(f'  Filters: conf>={self.conf_thresh}  '
                               f'area=[{self.min_area:.2f},{self.max_area:.2f}]  '
                               f'cy<={self.max_cy_ratio:.2f}  '
                               f'aspect>={self.min_aspect:.2f}')

    # ── Camera info ───────────────────────────────────────────────────────────
    def camera_info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]

    # ── Pixel → 3D ────────────────────────────────────────────────────────────
    def pixel_to_3d(self, u, v, z):
        X = (u - self.cx) * z / self.fx
        Y = (v - self.cy) * z / self.fy
        return float(X), float(Y), float(z)

    # ── Robust depth sampling ─────────────────────────────────────────────────
    def sample_depth(self, depth_img, cx, cy):
        h, w = depth_img.shape[:2]
        r    = self.depth_radius
        patch = depth_img[
            max(0, cy - r):min(h, cy + r + 1),
            max(0, cx - r):min(w, cx + r + 1)
        ]
        valid = patch[np.isfinite(patch) & (patch > self.min_depth)]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    # ── PointCloud2 fallback sampling ────────────────────────────────────────
    def sample_pointcloud_xyz(self, cloud_msg: PointCloud2, cx, cy):
        """
        Sample XYZ directly from an organized PointCloud2 around pixel (cx, cy).
        Returns (X, Y, Z) in the cloud's frame, or None if unavailable/invalid.
        Only works for organized clouds (height > 1, height/width match image).
        """
        if not _HAVE_PC2:
            return None
        if cloud_msg.height <= 1:
            # Unorganized cloud -- no reliable pixel indexing possible.
            return None

        r = self.depth_radius
        w, h = cloud_msg.width, cloud_msg.height

        # ROS 2's sensor_msgs_py expects `uvs` as FLAT indices (v*width + u)
        # into the point array, NOT (u, v) tuples like the old ROS 1 API.
        flat_idxs = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                u = cx + dx
                v = cy + dy
                if 0 <= u < w and 0 <= v < h:
                    flat_idxs.append(v * w + u)
        if not flat_idxs:
            return None

        try:
            pts = pc2.read_points(
                cloud_msg, field_names=('x', 'y', 'z'),
                skip_nans=True, uvs=flat_idxs
            )
        except Exception as e:
            self.get_logger().warn(f'PointCloud2 read_points failed: {e}',
                                    throttle_duration_sec=2.0)
            return None

        # read_points returns a structured numpy array; may be empty.
        if pts.size == 0:
            return None

        xs = np.asarray(pts['x'], dtype=np.float64)
        ys = np.asarray(pts['y'], dtype=np.float64)
        zs = np.asarray(pts['z'], dtype=np.float64)

        valid_mask = np.isfinite(zs) & (zs > self.min_depth) & (zs < self.max_depth)
        if not np.any(valid_mask):
            return None

        return (float(np.median(xs[valid_mask])),
                float(np.median(ys[valid_mask])),
                float(np.median(zs[valid_mask])))

    # ── Decode RGB safely for Gazebo (rgb8 or bgr8) ──────────────────────────
    def decode_rgb(self, msg):
        if msg.encoding.lower() in ('rgb8',):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if msg.encoding.lower() in ('bgr8',):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if len(frame.shape) == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return frame

    # ── Decode depth safely (32FC1 metres or 16UC1 mm) ───────────────────────
    def decode_depth(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            pass
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        return raw.astype(np.float32) / 1000.0

    # ── Entry points ──────────────────────────────────────────────────────────
    def image_depth_cb(self, rgb_msg: Image, depth_msg: Image):
        self._process(rgb_msg, depth_msg, points_msg=None)

    def image_depth_points_cb(self, rgb_msg: Image, depth_msg: Image, points_msg: PointCloud2):
        self._process(rgb_msg, depth_msg, points_msg=points_msg)

    # ── Main synced callback ──────────────────────────────────────────────────
    def _process(self, rgb_msg: Image, depth_msg: Image, points_msg):
        if self.fx is None:
            self.get_logger().warn(
                'Waiting for camera_info...', throttle_duration_sec=3.0
            )
            return

        if not self._sync_confirmed:
            self._sync_confirmed = True
            pc_info = f'  Points: {points_msg.width}x{points_msg.height}' if points_msg is not None else ''
            self.get_logger().info(
                f'Sync OK. RGB: {rgb_msg.encoding}  Depth: {depth_msg.encoding}{pc_info}'
            )

        frame = self.decode_rgb(rgb_msg)
        h, w  = frame.shape[:2]
        frame_area = float(h * w)

        try:
            depth = self.decode_depth(depth_msg)
        except Exception as e:
            self.get_logger().warn(f'Depth decode failed: {e}',
                                    throttle_duration_sec=2.0)
            return

        # ── YOLO ─────────────────────────────────────────────────────────────
        results = self.model.predict(
            frame, imgsz=self.imgsz, conf=self.conf_thresh, verbose=False
        )

        best_box  = None
        best_conf = 0.0
        best_cls  = None
        reject_log = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if self.target_cls != -1 and cls_id != self.target_cls:
                    continue

                conf = float(box.conf[0])
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy

                bw = x2 - x1
                bh = y2 - y1
                box_area   = bw * bh
                area_ratio = box_area / frame_area
                cx_px      = (x1 + x2) / 2.0
                cy_px      = (y1 + y2) / 2.0
                aspect     = bw / bh if bh > 0 else 0
                cy_ratio   = cy_px / h

                # ── Sanity filters ────────────────────────────────────────────
                if area_ratio > self.max_area:
                    reject_log.append(f'area={area_ratio:.2f}>max')
                    continue
                if area_ratio < self.min_area:
                    reject_log.append(f'area={area_ratio:.2f}<min')
                    continue
                if cy_ratio > self.max_cy_ratio:
                    reject_log.append(
                        f'cy_ratio={cy_ratio:.2f}>max({self.max_cy_ratio}) '
                        f'-- floor/ground rejection'
                    )
                    continue
                if aspect < self.min_aspect:
                    reject_log.append(
                        f'aspect={aspect:.2f}<min({self.min_aspect}) '
                        f'-- too tall/narrow'
                    )
                    continue

                if conf > best_conf:
                    best_conf = conf
                    best_box  = xyxy
                    best_cls  = cls_id

        # Log rejections for tuning
        for msg_str in reject_log:
            self.get_logger().debug(f'Rejected: {msg_str}')

        # ── No detection ──────────────────────────────────────────────────────
        det_msg      = Bool()
        det_msg.data = best_box is not None
        self.detect_pub.publish(det_msg)

        if best_box is None:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )
            return

        x1, y1, x2, y2 = best_box
        cx_px = (x1 + x2) / 2.0
        cy_px = (y1 + y2) / 2.0

        # ── Depth at pallet centre (with pointcloud fallback) ────────────────
        depth_z = self.sample_depth(depth, int(cx_px), int(cy_px))
        used_pointcloud = False
        pc_xyz = None

        depth_invalid = np.isnan(depth_z) or depth_z > self.max_depth
        if depth_invalid and self.use_pc_fallback and points_msg is not None:
            pc_xyz = self.sample_pointcloud_xyz(points_msg, int(cx_px), int(cy_px))
            if pc_xyz is not None:
                used_pointcloud = True
                self.get_logger().info(
                    'Depth image sample invalid -- used PointCloud2 fallback',
                    throttle_duration_sec=1.0
                )

        if depth_invalid and not used_pointcloud:
            self.get_logger().warn(
                f'Invalid depth at pallet centre: {depth_z:.3f}m'
                + (' (pointcloud fallback also failed)' if self.use_pc_fallback else ''),
                throttle_duration_sec=1.0
            )
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )
            return

        # ── 3-D position ───────────────────────────────────────────────────
        if used_pointcloud:
            X, Y, Z = pc_xyz
        else:
            X, Y, Z = self.pixel_to_3d(cx_px, cy_px, depth_z)

        norm_x = (cx_px - w / 2.0) / (w / 2.0)
        norm_y = (cy_px - h / 2.0) / (h / 2.0)

        stamp = rgb_msg.header.stamp

        pose                 = PoseStamped()
        pose.header.stamp    = stamp
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        pt               = PointStamped()
        pt.header.stamp    = stamp
        pt.header.frame_id = self.camera_frame
        pt.point.x = float(norm_x)
        pt.point.y = float(norm_y)
        pt.point.z = float(best_conf)
        self.center_pub.publish(pt)

        self.get_logger().info(
            f'Pallet: cls={self.model.names.get(best_cls)}  '
            f'conf={best_conf:.2f}  '
            f'3D=({X:.3f}, {Y:.3f}, {Z:.3f})m  '
            f'norm=({norm_x:.2f}, {norm_y:.2f})'
            f'{"  [pointcloud]" if used_pointcloud else ""}',
            throttle_duration_sec=1.0
        )

        # ── Debug image ───────────────────────────────────────────────────
        class_name = self.model.names.get(best_cls, str(best_cls))
        label = (
            f'{class_name} {best_conf:.2f} | '
            f'Z={Z:.2f}m X={X:.2f}m'
            + (' [PC]' if used_pointcloud else '')
        )

        # Bounding box + centre dot
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                      (0, 255, 0), 2)
        cv2.circle(frame, (int(cx_px), int(cy_px)), 6, (0, 0, 255), -1)
        cv2.putText(frame, label,
                    (int(x1), max(int(y1) - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Frame centre crosshair (shows how aligned the pallet is)
        cv2.line(frame, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2),
                 (255, 255, 0), 1)
        cv2.line(frame, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20),
                 (255, 255, 0), 1)

        # Lateral error bar at bottom
        bar_x   = int(w / 2 + norm_x * w / 2)
        bar_y   = h - 20
        cv2.line(frame, (w // 2, bar_y), (bar_x, bar_y), (0, 165, 255), 4)
        cv2.putText(frame, f'lateral: {norm_x:+.2f}',
                    (10, h - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 165, 255), 1)

        # max_cy filter line (shows the ground-rejection threshold)
        cy_line = int(self.max_cy_ratio * h)
        cv2.line(frame, (0, cy_line), (w, cy_line), (0, 0, 255), 1)
        cv2.putText(frame, 'ground reject',
                    (5, cy_line - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (0, 0, 255), 1)

        # Depth thumbnail (top-left corner)
        depth_disp = np.clip(depth / self.max_depth, 0.0, 1.0)
        depth_u8   = (depth_disp * 255).astype(np.uint8)
        depth_col  = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
        scale      = 0.25
        thumb      = cv2.resize(depth_col,
                                (int(depth_col.shape[1] * scale),
                                 int(depth_col.shape[0] * scale)))
        th, tw = thumb.shape[:2]
        frame[10:10 + th, 10:10 + tw] = thumb

        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        )


def main(args=None):
    rclpy.init(args=args)
    node = PalletDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
