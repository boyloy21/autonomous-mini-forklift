#!/home/yunboyloy/miniconda3/envs/pallet_vision/bin/python3
"""
YOLOv11-based pallet detector for ROS 2 Humble — Gazebo Fortress simulation.

Subscribes:
  /forklift/camera/image        (sensor_msgs/Image)       RGB image
  /forklift/camera/depth_image  (sensor_msgs/Image)       32FC1 depth in metres
  /forklift/camera/camera_info  (sensor_msgs/CameraInfo)  intrinsics

Publishes:
  /pallet/pose         (geometry_msgs/PoseStamped)  3D position of pallet centre
  /pallet/pixel_center (geometry_msgs/PointStamped) normalised pixel offset + conf
  /pallet/debug_image  (sensor_msgs/Image)           annotated frame
  /pallet/detected     (std_msgs/Bool)               True when pallet visible
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import message_filters
import numpy as np
import cv2
import os


class PalletDetector(Node):
    def __init__(self):
        super().__init__('pallet_detector')

        # ── Parameters ────────────────────────────────────────────────────────
        default_model = os.path.join(
            get_package_share_directory('pallet_vision'),
            'models', 'best_detect.pt'
        )
        self.declare_parameter('model_path',            default_model)
        self.declare_parameter('confidence_threshold',  0.6)
        self.declare_parameter('input_size',            640)  # YOLOv8 default is 640x640

        # ── Box area filters ───────────────────────────────────────────────────
        # Reject boxes covering more than 60% or less than 1% of the frame
        self.declare_parameter('max_box_area_ratio',    0.8)
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
        self.declare_parameter('sync_slop',             0.1)

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
        sync_slop         = self.get_parameter('sync_slop').value

        # ── Model ─────────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info(f'Model ready. Classes: {self.model.names}')

        # ── Camera intrinsics ─────────────────────────────────────────────────
        self.fx = self.fy = self.cx = self.cy = None

        self.bridge = CvBridge()
        self._sync_confirmed = False

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo, info_topic, self.camera_info_cb, 10
        )

        rgb_sub   = message_filters.Subscriber(self, Image, image_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=sync_slop
        )
        self._sync.registerCallback(self.image_depth_cb)

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

    # ── Main synced callback ──────────────────────────────────────────────────
    def image_depth_cb(self, rgb_msg: Image, depth_msg: Image):
        if self.fx is None:
            self.get_logger().warn(
                'Waiting for camera_info...', throttle_duration_sec=3.0
            )
            return

        if not self._sync_confirmed:
            self._sync_confirmed = True
            self.get_logger().info(
                f'Sync OK. RGB: {rgb_msg.encoding}  Depth: {depth_msg.encoding}'
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

        best_box = None
        best_cls = None
        best_score = -1.0
        best_conf = 0.0
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

                # -----------------------------
                # Composite score
                # -----------------------------
                center_dist = np.sqrt(
                    ((cx_px - w/2.0)/(w/2.0))**2 +
                    ((cy_px - h/2.0)/(h/2.0))**2
                )

                center_score = max(0.0, 1.0 - center_dist)

                size_score = area_ratio / self.max_area
                size_score = np.clip(size_score, 0.0, 1.0)

                score = (
                    0.5 * conf +
                    0.3 * center_score +
                    0.2 * size_score
                )

                if score > best_score:
                    best_score = score
                    best_conf = conf
                    best_box = xyxy
                    best_cls = cls_id

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

        # ── Depth at pallet centre ─────────────────────────────────────────
        depth_z = self.sample_depth(depth, int(cx_px), int(cy_px))

        if np.isnan(depth_z) or depth_z > self.max_depth:
            self.get_logger().warn(
                f'Invalid depth at pallet centre: {depth_z:.3f}m',
                throttle_duration_sec=1.0
            )
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )
            return

        # ── 3-D back-projection ───────────────────────────────────────────
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

        # self.get_logger().info(
        #     f'Pallet: cls={self.model.names.get(best_cls)}  '
        #     f'conf={best_conf:.2f}  '
        #     f'3D=({X:.3f}, {Y:.3f}, {Z:.3f})m  '
        #     f'norm=({norm_x:.2f}, {norm_y:.2f})',
        #     throttle_duration_sec=1.0
        # )
        
        self.get_logger().info(
            f'Pallet '
            f'conf={best_conf:.2f} '
            f'score={best_score:.2f} '
            f'XYZ=({X:.2f}, {Y:.2f}, {Z:.2f})',
            throttle_duration_sec=1.0
        )

        # ── Debug image ───────────────────────────────────────────────────
        class_name = self.model.names.get(best_cls, str(best_cls))
        label = (
            f'{class_name} {best_conf:.2f} | '
            f'Z={Z:.2f}m X={X:.2f}m'
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