#!/usr/bin/env python3
"""
pose_estimate.py
--------------------------
ROS2 node: subscribes to the camera image + camera_info topics, detects
ArUco markers, solves 6DOF pose per marker (using camera_info intrinsics --
no separate calibration file needed for the Gazebo sim camera), and:

  - publishes geometry_msgs/PoseArray of all detected markers in the
    camera optical frame
  - broadcasts a TF transform camera_frame -> aruco_marker_<id> for each
    detected marker (if publish_tf: true)
  - publishes an annotated debug image with 3D axes drawn on each marker,
    plus the marker's X/Y/Yaw overlaid as text

Subscribes:
    image_topic (sensor_msgs/Image)              default: /forklift/camera/image_raw
    camera_info_topic (sensor_msgs/CameraInfo)    default: /forklift/camera/camera_info

Publishes:
    detected_markers_topic (geometry_msgs/PoseArray)  default: /forklift/camera/aruco/markers
    annotated_image_topic (sensor_msgs/Image)         default: /forklift/camera/aruco/image
    TF: camera_frame -> aruco_marker_<id>

Params (see config/aruco_parameters.yaml):
    image_topic, camera_info_topic, camera_frame, marker_size,
    aruco_dictionary_id, detected_markers_topic, annotated_image_topic, publish_tf
"""


import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

from forklift_aruco.aruco_dicts import get_dictionary
from forklift_aruco.ros_image_utils import to_imgmsg_bgr8


def rotation_vector_to_quaternion(rvec):
    """Rodrigues rotation vector -> (x, y, z, w) quaternion."""
    rot_matrix, _ = cv2.Rodrigues(rvec)
    m = rot_matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def quaternion_to_yaw(qx, qy, qz, qw):
    """Yaw (rotation about Z, radians) from a quaternion -- same atan2
    convention used elsewhere in this project (e.g. robot pose yaw)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__("pose_estimate")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("marker_size", 0.2)
        self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50")
        self.declare_parameter("detected_markers_topic", "/aruco/markers")
        self.declare_parameter("annotated_image_topic", "/aruco/image")
        self.declare_parameter("markers_visualization_topic", "/aruco/poses")
        self.declare_parameter("publish_tf", True)

        self.camera_frame = self.get_parameter("camera_frame").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        dict_id = self.get_parameter("aruco_dictionary_id").value

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        markers_topic = self.get_parameter("detected_markers_topic").value
        annotated_topic = self.get_parameter("annotated_image_topic").value
        viz_topic = self.get_parameter("markers_visualization_topic").value

        dictionary = get_dictionary(dict_id)
        detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

        half = self.marker_size / 2.0
        self.marker_obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.pose_pub = self.create_publisher(PoseArray, markers_topic, 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.viz_pub = self.create_publisher(MarkerArray, viz_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"pose_estimate: image='{image_topic}' camera_info='{camera_info_topic}' "
            f"dict='{dict_id}' marker_size={self.marker_size} camera_frame='{self.camera_frame}' "
            f"publish_tf={self.publish_tf} | publishing poses='{markers_topic}' "
            f"annotated='{annotated_topic}' viz='{viz_topic}'"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera_info before running pose estimation...",
                                    throttle_duration_sec=5.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.header.frame_id = self.camera_frame

        marker_array = MarkerArray()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                ok, rvec, tvec = cv2.solvePnP(
                    self.marker_obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                )
                if not ok:
                    continue

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.5)

                x, y, z = tvec.flatten()
                qx, qy, qz, qw = rotation_vector_to_quaternion(rvec)
                yaw = quaternion_to_yaw(qx, qy, qz, qw)
                yaw_deg = math.degrees(yaw)

                # OpenCV's tvec is in the camera OPTICAL frame: +X=right,
                # +Y=down, +Z=forward(depth). That reads as "swapped" to
                # anyone expecting ROS convention (+X=forward, +Y=left,
                # +Z=up) - REP-103. Convert for the on-screen readout so
                # "forward" is actually the depth-to-marker number.
                forward = z       # was Z (depth) - now what's displayed as "X"
                left = -x         # was X (right-positive) - now "Y", left-positive
                up = -y           # was Y (down-positive) - now "up", not currently printed but computed for later use

                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
                pose.orientation.x, pose.orientation.y = float(qx), float(qy)
                pose.orientation.z, pose.orientation.w = float(qz), float(qw)
                pose_array.poses.append(pose)

                c = corners[i][0]
                cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
                text = f"ID {int(marker_id)}  X:{forward:.2f} Y:{left:.2f} Yaw:{yaw_deg:.1f}deg"
                cv2.putText(frame, text, (cx - 60, cy + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, text, (cx - 60, cy + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

                if self.publish_tf:
                    t = TransformStamped()
                    t.header = msg.header
                    t.header.frame_id = self.camera_frame
                    t.child_frame_id = f"aruco_marker_{int(marker_id)}"
                    t.transform.translation.x = float(x)
                    t.transform.translation.y = float(y)
                    t.transform.translation.z = float(z)
                    t.transform.rotation.x = float(qx)
                    t.transform.rotation.y = float(qy)
                    t.transform.rotation.z = float(qz)
                    t.transform.rotation.w = float(qw)
                    self.tf_broadcaster.sendTransform(t)

                cube = Marker()
                cube.header = msg.header
                cube.header.frame_id = self.camera_frame
                cube.ns = "aruco_markers"
                cube.id = int(marker_id)
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose = pose
                cube.scale.x = self.marker_size
                cube.scale.y = self.marker_size
                cube.scale.z = 0.01  # thin, like a flat printed marker
                cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 0.7, 1.0, 0.8
                cube.lifetime.sec = 0
                cube.lifetime.nanosec = int(2e8)  # marker fades if detection stops (~0.2s)
                marker_array.markers.append(cube)

                label = Marker()
                label.header = cube.header
                label.ns = "aruco_marker_labels"
                label.id = int(marker_id)
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = pose
                label.pose.position.z += self.marker_size * 0.6
                label.scale.z = self.marker_size * 0.5
                label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
                label.text = f"id {int(marker_id)}"
                label.lifetime.sec = 0
                label.lifetime.nanosec = int(2e8)
                marker_array.markers.append(label)

        self.pose_pub.publish(pose_array)
        self.viz_pub.publish(marker_array)

        out_msg = to_imgmsg_bgr8(frame, msg.header)
        self.annotated_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()