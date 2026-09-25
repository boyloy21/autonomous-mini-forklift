#!/usr/bin/env python3
"""
ROS2 wrapper code originally adapted from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates Aruco AR markers in images and publishes their ids and
poses, transformed into base_link (or map/odom, if odometry is available)
using an explicit rotation-matrix transform chain:

    marker (camera optical frame) -> base_link -> map/odom

It ALSO republishes a single target marker's pose as a plain
geometry_msgs/PoseStamped on /dock_pose -- this is what forklift_docking's
launch file remaps into opennav_docking's SimpleChargingDock plugin (which
subscribes internally to a fixed, non-configurable topic name
"detected_dock_pose"). opennav_docking has no concept of marker IDs and
cannot consume the multi-marker ArucoMarkers array directly; the
target_marker_id parameter is how the currently-relevant marker gets
selected out of that array.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/depth/image_raw (sensor_msgs.msg.Image)   [optional, if use_depth_input]
   /odom (nav_msgs.msg.Odometry)                      [optional, for map-frame output]

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
    /aruco_markers (aruco_interfaces.msg.ArucoMarkers)
    /aruco_image (sensor_msgs.msg.Image)
    /dock_pose (geometry_msgs.msg.PoseStamped)         [only the target_marker_id marker]

Parameters:
    marker_size - size of the markers in meters (default .065)
    aruco_dictionary_id - dictionary used to generate markers (default DICT_5X5_250)
    use_depth_input - use depth camera input for pose estimation (default True)
    image_topic - image topic to subscribe to
    depth_image_topic - depth image topic to subscribe to
    camera_info_topic - camera info topic to subscribe to
    camera_frame - camera optical frame name
    odom_topic - odometry topic for robot pose (default /odom)
    output_frame - "base_link" or "map" (default "base_link")
    camera_mount_xyz - [x, y, z] camera position in base_link (meters)
    camera_mount_rpy - [roll, pitch, yaw] camera orientation in base_link (radians)
    detected_markers_topic, markers_visualization_topic, output_image_topic
    dock_pose_topic - topic to publish the filtered single-marker pose on (default /dock_pose)
    target_marker_id - ArUco marker ID currently relevant for docking, -1 = none
                        (set at runtime via `ros2 param set` before sending a
                        DockRobot goal, ideally driven by dock_database.yaml's
                        `id` field for the dock being approached)

Author: Simone Giampà (original), updated with matrix-based frame transform
        and integrated dock_pose publisher
Version: 2026-08

DETECTION TUNING (this pass): the stock cv2.aruco.DetectorParameters()
defaults used previously were too conservative for a small, low-contrast,
or slightly-blurry marker -- confirmed as the actual root cause of
"aruco/poses and aruco_dock_pose never publish anything" across BOTH
forklift1 and forklift2 in multi-robot testing. The publisher itself was
always created correctly (unconditional in __init__); it simply never had
anything to publish, because pose_estimation() found zero markers, ever.
Not a namespace/wiring bug at all -- see the widened detector parameters
below.
"""

# ROS2 imports
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import message_filters

# Python imports
import numpy as np
import cv2
import tf_transformations

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from forklift_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult

# Local imports for custom defined functions
from forklift_aruco.utils import ARUCO_DICT
from forklift_aruco.pose_estimation import (
    pose_estimation,
    build_camera_to_base_matrix,
    quaternion_to_yaw,
)


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(self.dictionary_id_name)
            )
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error("valid options: {}".format(options))

        # --- Build the constant camera->base_link transform once ---
        self.T_camera_to_base = build_camera_to_base_matrix(
            cam_x=self.camera_mount_xyz[0],
            cam_y=self.camera_mount_xyz[1],
            cam_z=self.camera_mount_xyz[2],
            cam_roll=self.camera_mount_rpy[0],
            cam_pitch=self.camera_mount_rpy[1],
            cam_yaw=self.camera_mount_rpy[2],
        )
        self.get_logger().info(f"T_camera_to_base =\n{self.T_camera_to_base}")

        # --- Robot pose cache (only used if output_frame == 'map') ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False

        if self.output_frame == "map":
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
            )

        # Camera info subscription
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data
        )

        # Image (+ optional depth) subscriptions
        if (bool(self.use_depth_input)):
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic,
                                                        qos_profile=qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic,
                                                              qos_profile=qos_profile_sensor_data)
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, self.markers_visualization_topic, 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.dock_pose_pub = self.create_publisher(PoseStamped, self.dock_pose_topic, 10)

        # Allow target_marker_id to be changed live via `ros2 param set`,
        # e.g. by forklift_mission_manager right before sending a
        # DockRobot goal, without restarting this node.
        self.add_on_set_parameters_callback(self._on_param_change)

        # Camera parameters (populated by info_callback)
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        # TUNED (this pass): stock defaults are commonly too conservative
        # for a small, low-contrast, or slightly-blurry marker -- exactly
        # the case observed (marker visible to the eye in a captured
        # frame, never detected by OpenCV's default thresholds), which
        # was the actual root cause of /aruco/poses and /aruco_dock_pose
        # never publishing anything, on multiple robots, not a wiring bug.
        #   - Wider adaptive-threshold window search widens the range of
        #     lighting/contrast conditions considered per candidate region.
        #   - Lower minMarkerPerimeterRate accepts smaller markers
        #     relative to image size (a distant/small marker covers a
        #     smaller fraction of the frame than the default expects).
        #   - Looser polygonalApproxAccuracyRate tolerates a less
        #     perfectly-square marker outline (motion blur, compression
        #     artifacts, minor lens distortion).
        #   - cornerRefinementMethod improves pose accuracy once a marker
        #     IS found; doesn't affect the initial detection rate itself.
        self.aruco_parameters.adaptiveThreshWinSizeMin = 3
        self.aruco_parameters.adaptiveThreshWinSizeMax = 53
        self.aruco_parameters.adaptiveThreshWinSizeStep = 4
        self.aruco_parameters.minMarkerPerimeterRate = 0.01
        self.aruco_parameters.polygonalApproxAccuracyRate = 0.08
        self.aruco_parameters.errorCorrectionRate = 0.8   # more tolerant of a few misread bits per frame
        self.aruco_parameters.adaptiveThreshConstant = 5   # lower = more sensitive to subtle contrast
        self.aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        self.bridge = CvBridge()

        self.get_logger().info(
            f"Publishing filtered dock pose on '{self.dock_pose_topic}' "
            f"(target_marker_id={self.target_marker_id})"
        )

    # ------------------------------------------------------------------
    # Live parameter updates -- lets target_marker_id be changed at
    # runtime (e.g. `ros2 param set /aruco_node target_marker_id 2`)
    # without restarting the node.
    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        for param in params:
            if param.name == "target_marker_id":
                self.target_marker_id = param.value
                self.get_logger().info(f"target_marker_id updated to {self.target_marker_id}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Odometry callback: keeps robot's live (x, y, yaw) for the
    # base_link -> map transform, used only when output_frame == "map".
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_odom = True

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.get_logger().info("Camera info received.")
        self.get_logger().info("Intrinsic matrix: {}".format(self.intrinsic_mat))
        self.get_logger().info("Distortion coefficients: {}".format(self.distortion))
        self.get_logger().info("Camera frame: {}x{}".format(self.info_msg.width, self.info_msg.height))

        self.destroy_subscription(self.info_sub)

    def _current_robot_pose(self):
        """Returns (x, y, yaw) tuple if odom is available and output_frame
        is 'map', otherwise None (pose_estimation then outputs base_link)."""
        if self.output_frame == "map" and self.has_odom:
            return (self.robot_x, self.robot_y, self.robot_yaw)
        return None

    def _publish_dock_pose(self, markers: ArucoMarkers):
        """Filters the just-detected markers down to target_marker_id and,
        if found, republishes it as a single PoseStamped on dock_pose_topic
        for opennav_docking's SimpleChargingDock (via the
        detected_dock_pose remap in forklift_docking's launch file).

        If target_marker_id is unset (-1) or not currently visible, nothing
        is published this frame -- opennav_docking's
        external_detection_timeout handles staleness/absence on its side.
        """
        if self.target_marker_id < 0:
            return

        for marker_id, pose in zip(markers.marker_ids, markers.poses):
            if int(marker_id) == self.target_marker_id:
                dock_pose = PoseStamped()
                dock_pose.header = markers.header
                dock_pose.pose = pose
                self.dock_pose_pub.publish(dock_pose)
                return

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")

        markers = ArucoMarkers()
        pose_array = PoseArray()

        target_frame = "map" if self.output_frame == "map" and self.has_odom else "base_link"
        markers.header.frame_id = target_frame
        pose_array.header.frame_id = target_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        frame, pose_array, markers = pose_estimation(
            rgb_frame=cv_image, depth_frame=None,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers,
            T_camera_to_base=self.T_camera_to_base,
            robot_pose=self._current_robot_pose(),
        )

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self._publish_dock_pose(markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def depth_image_callback(self, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        markers = ArucoMarkers()
        pose_array = PoseArray()

        target_frame = "map" if self.output_frame == "map" and self.has_odom else "base_link"
        markers.header.frame_id = target_frame
        pose_array.header.frame_id = target_frame

        markers.header.stamp = rgb_msg.header.stamp
        pose_array.header.stamp = rgb_msg.header.stamp

        frame, pose_array, markers = pose_estimation(
            rgb_frame=cv_image, depth_frame=cv_depth_image,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers,
            T_camera_to_base=self.T_camera_to_base,
            robot_pose=self._current_robot_pose(),
        )

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self._publish_dock_pose(markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def initialize_parameters(self):
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation instead of RGB image",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="camera/image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="camera/depth/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="odom_topic",
            value="odom",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Odometry topic providing the robot's pose in map/odom frame.",
            ),
        )

        self.declare_parameter(
            name="output_frame",
            value="base_link",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame to publish marker poses in: 'base_link' or 'map'.",
            ),
        )

        self.declare_parameter(
            name="camera_mount_xyz",
            value=[0.0, 0.0, 0.1],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera position [x, y, z] in base_link frame (meters).",
            ),
        )

        self.declare_parameter(
            name="camera_mount_rpy",
            value=[0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera orientation [roll, pitch, yaw] in base_link frame (radians).",
            ),
        )

        self.declare_parameter(
            name="detected_markers_topic",
            value="aruco_markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected markers as array of marker ids and poses",
            ),
        )

        self.declare_parameter(
            name="markers_visualization_topic",
            value="aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish markers as pose array",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated images with markers drawn on them",
            ),
        )

        self.declare_parameter(
            name="dock_pose_topic",
            value="aruco_dock_pose",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish the filtered single-target-marker pose on, "
                            "for consumption by opennav_docking (via detected_dock_pose remap).",
            ),
        )

        self.declare_parameter(
            name="target_marker_id",
            value=-1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="ArUco marker ID currently relevant for docking. -1 = none, "
                            "don't publish on dock_pose_topic. Set at runtime before sending "
                            "a DockRobot goal, e.g.: ros2 param set /aruco_node target_marker_id 2",
            ),
        )

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        self.get_logger().info(f"Marker type: {self.dictionary_id_name}")

        self.use_depth_input = self.get_parameter("use_depth_input").get_parameter_value().bool_value
        self.get_logger().info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.get_logger().info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        self.get_logger().info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.get_logger().info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.get_logger().info(f"Odom topic: {self.odom_topic}")

        self.output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        if self.output_frame not in ("base_link", "map"):
            self.get_logger().warn(
                f"output_frame='{self.output_frame}' invalid, defaulting to 'base_link'")
            self.output_frame = "base_link"
        self.get_logger().info(f"Output frame: {self.output_frame}")

        self.camera_mount_xyz = list(
            self.get_parameter("camera_mount_xyz").get_parameter_value().double_array_value)
        self.camera_mount_rpy = list(
            self.get_parameter("camera_mount_rpy").get_parameter_value().double_array_value)
        self.get_logger().info(f"Camera mount xyz: {self.camera_mount_xyz}, rpy: {self.camera_mount_rpy}")

        self.detected_markers_topic = self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        self.markers_visualization_topic = self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        self.output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value

        self.dock_pose_topic = self.get_parameter("dock_pose_topic").get_parameter_value().string_value
        self.target_marker_id = self.get_parameter("target_marker_id").get_parameter_value().integer_value


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()