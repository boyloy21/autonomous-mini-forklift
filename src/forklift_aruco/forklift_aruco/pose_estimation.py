#!/usr/bin/env python3

# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main

# Python imports
import math

import numpy as np
import cv2
import tf_transformations

# ROS2 imports
from rclpy.impl import rcutils_logger

# ROS2 message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from forklift_interfaces.msg import ArucoMarkers

# utils import python code
from forklift_aruco.utils import aruco_display


# ---------------------------------------------------------------------------
# Static conversion: OpenCV optical frame (X-right, Y-down, Z-forward)
# -> ROS link/body frame (X-forward, Y-left, Z-up), per REP-103.
# This is a fixed rotation, independent of any specific camera mount pose.
# ---------------------------------------------------------------------------
R_OPTICAL_TO_LINK = np.array([
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


# ---------------------------------------------------------------------------
# Fixed correction for the marker's own axis convention (X-right, Y-up,
# Z-out-of-face-toward-camera). Left uncorrected, a marker facing the
# camera head-on produces a rotation matrix that is NOT identity once
# transformed into base_link -- it comes out ~90 degrees off, because the
# marker's "Z points toward the camera" convention doesn't match the
# semantic we want for docking ("yaw = 0 when the dock squarely faces the
# robot"). This was diagnosed from a real frontal-view detection reporting
# -82.2 deg (expected ~0 deg) -- solving for the correction that makes the
# frontal case output identity gives this fixed matrix. It's a one-time,
# constant change of basis, valid for all viewing angles, not just frontal.
R_MARKER_AXIS_FIX = np.array([
    [0.0, -1.0, 0.0],
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
])


def quaternion_to_yaw(qx, qy, qz, qw):
    """Yaw (rotation about Z, radians) from a quaternion - same atan2
    convention used elsewhere in this project (e.g. robot pose yaw)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def rpy_to_rotation_matrix(roll, pitch, yaw):
    """Returns a 3x3 rotation matrix from roll/pitch/yaw (radians)."""
    R4 = tf_transformations.euler_matrix(roll, pitch, yaw)
    return R4[0:3, 0:3]


def build_camera_to_base_matrix(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw):
    """
    Builds the CONSTANT 4x4 homogeneous transform that maps a point
    expressed in the camera OPTICAL frame into the robot base_link frame.

    cam_x, cam_y, cam_z       - camera mount position in base_link (meters)
    cam_roll, cam_pitch, cam_yaw - camera mount orientation in base_link (radians)

    This should be computed ONCE (e.g. at node startup) from your URDF/SDF
    camera mount values, not recomputed every frame.
    """
    R_mount = rpy_to_rotation_matrix(cam_roll, cam_pitch, cam_yaw)

    T = np.eye(4, dtype=np.float64)
    # combine the URDF mount rotation with the fixed optical->link axis swap
    T[0:3, 0:3] = R_mount @ R_OPTICAL_TO_LINK
    T[0:3, 3] = [cam_x, cam_y, cam_z]
    return T


def build_base_to_map_matrix(robot_x, robot_y, robot_yaw):
    """
    Builds the DYNAMIC 4x4 homogeneous transform that maps a point
    expressed in base_link into the map (or odom) frame, using the
    robot's current 2D pose (robot_roll = robot_pitch = 0 assumed).
    """
    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = rpy_to_rotation_matrix(0.0, 0.0, robot_yaw)
    T[0:3, 3] = [robot_x, robot_y, 0.0]
    return T


def marker_pose_in_map(rvec, tvec, T_camera_to_base, robot_x, robot_y, robot_yaw):
    """
    Full transform chain: marker (camera optical frame) -> base_link -> map.

    rvec, tvec        - solvePnP outputs, marker pose in camera optical frame
    T_camera_to_base  - constant 4x4 matrix from build_camera_to_base_matrix()
    robot_x, robot_y, robot_yaw - robot's current pose in map/odom frame

    Returns:
        position (np.array [x, y, z]) in map frame
        quaternion (np.array [x, y, z, w]) in map frame
    """
    rot_cam, _ = cv2.Rodrigues(rvec)
    T_marker_in_camera = np.eye(4, dtype=np.float64)
    T_marker_in_camera[0:3, 0:3] = rot_cam
    T_marker_in_camera[0:3, 3] = tvec.flatten()

    T_base_to_map = build_base_to_map_matrix(robot_x, robot_y, robot_yaw)

    # order matters: apply camera->base first, then base->map, to the marker pose
    T_marker_in_map = T_base_to_map @ T_camera_to_base @ T_marker_in_camera

    position = T_marker_in_map[0:3, 3].copy()

    R4 = np.eye(4, dtype=np.float64)
    R4[0:3, 0:3] = T_marker_in_map[0:3, 0:3]
    quat = tf_transformations.quaternion_from_matrix(R4)
    norm = np.linalg.norm(quat)
    quat = quat / norm if norm > 0 else quat

    return position, quat


def marker_pose_in_base(rvec, tvec, T_camera_to_base):
    """
    Transform chain: marker (camera optical frame) -> base_link only.
    Use this for docking control (depth, lateral offset, yaw error relative
    to the robot), where you don't need global map coordinates.
    """
    rot_cam, _ = cv2.Rodrigues(rvec)
    T_marker_in_camera = np.eye(4, dtype=np.float64)
    T_marker_in_camera[0:3, 0:3] = rot_cam
    T_marker_in_camera[0:3, 3] = tvec.flatten()

    T_marker_in_base = T_camera_to_base @ T_marker_in_camera

    position = T_marker_in_base[0:3, 3].copy()

    R4 = np.eye(4, dtype=np.float64)
    R4[0:3, 0:3] = T_marker_in_base[0:3, 0:3]
    quat = tf_transformations.quaternion_from_matrix(R4)
    norm = np.linalg.norm(quat)
    quat = quat / norm if norm > 0 else quat

    return position, quat


def draw_pose_text_top_left(frame, row_index, marker_id, x, y, yaw_deg, label="BASE"):
    text = f"ID {int(marker_id)} [{label}] X:{x:.2f} Y:{y:.2f} Yaw:{yaw_deg:.1f}deg"

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    margin = 10
    row_height = 22

    (_, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)

    x_pos = margin
    y_pos = margin + text_h + row_index * row_height

    cv2.putText(frame, text, (x_pos, y_pos),
                font, font_scale, (0, 0, 0),
                thickness + 2, cv2.LINE_AA)

    cv2.putText(frame, text, (x_pos, y_pos),
                font, font_scale, (0, 255, 255),
                thickness, cv2.LINE_AA)


def pose_estimation(rgb_frame: np.array, depth_frame: np.array, aruco_detector: cv2.aruco.ArucoDetector,
                    marker_size: float, matrix_coefficients: np.array, distortion_coefficients: np.array,
                    pose_array: PoseArray, markers: ArucoMarkers,
                    T_camera_to_base: np.array = None,
                    robot_pose: tuple = None) -> list:
    '''
    rgb_frame - Frame from the RGB camera stream
    depth_frame - Depth frame from the depth camera stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    pose_array - PoseArray message to be published
    markers - ArucoMarkers message to be published
    T_camera_to_base - constant 4x4 matrix (camera optical -> base_link), from
                        build_camera_to_base_matrix(). If None, poses are
                        published in raw camera optical frame (legacy behavior).
    robot_pose - optional (x, y, yaw) tuple of the robot in map/odom frame.
                 If provided (and T_camera_to_base is provided), poses are
                 published in the map/odom frame instead of base_link.

    return:-
    frame - The frame with the axis drawn on it
    pose_array - PoseArray with computed poses of the markers
    markers - ArucoMarkers message containing markers id number and pose
    '''

    corners, marker_ids, rejected = aruco_detector.detectMarkers(image=rgb_frame)

    frame_processed = rgb_frame
    logger = rcutils_logger.RcutilsLogger(name="aruco_node")

    if len(corners) > 0:

        logger.debug("Detected {} markers.".format(len(corners)))

        for i, marker_id in enumerate(marker_ids):

            tvec, rvec, quat_cam = my_estimatePoseSingleMarkers(
                corners=corners[i], marker_size=marker_size,
                camera_matrix=matrix_coefficients,
                distortion=distortion_coefficients)

            frame_processed = aruco_display(corners=corners, ids=marker_ids,
                                            image=frame_processed)

            frame_processed = cv2.drawFrameAxes(image=frame_processed, cameraMatrix=matrix_coefficients,
                                                distCoeffs=distortion_coefficients, rvec=rvec, tvec=tvec,
                                                length=0.05, thickness=3)

            if (depth_frame is not None):
                centroid = depth_to_pointcloud_centroid(depth_image=depth_frame,
                                                        intrinsic_matrix=matrix_coefficients,
                                                        corners=corners[i])
                logger.info(f"depthcloud centroid = {centroid}")
                logger.info(f"tvec = {tvec[0]} {tvec[1]} {tvec[2]}")
                # use depth centroid as the translation source, keep rvec's rotation
                tvec_used = np.array(centroid, dtype=np.float64).reshape(3, 1)
            else:
                tvec_used = tvec.astype(np.float64)

            # --- Transform chain ---
            frame_label = "CAM"
            if T_camera_to_base is not None:
                if robot_pose is not None:
                    robot_x, robot_y, robot_yaw = robot_pose
                    position, quat_out = marker_pose_in_map(
                        rvec, tvec_used, T_camera_to_base, robot_x, robot_y, robot_yaw)
                    frame_label = "MAP"
                else:
                    position, quat_out = marker_pose_in_base(
                        rvec, tvec_used, T_camera_to_base)
                    frame_label = "BASE"
            else:
                # legacy: publish raw camera-optical-frame pose, no transform
                position = tvec_used.flatten()
                quat_out = quat_cam
                frame_label = "CAM"

            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(quat_out[0])
            pose.orientation.y = float(quat_out[1])
            pose.orientation.z = float(quat_out[2])
            pose.orientation.w = float(quat_out[3])

            yaw_deg = math.degrees(quaternion_to_yaw(*quat_out))
            draw_pose_text_top_left(
                frame_processed, i, marker_id[0],
                pose.position.x, pose.position.y, yaw_deg, label=frame_label,
            )

            pose_array.poses.append(pose)
            markers.poses.append(pose)
            markers.marker_ids.append(marker_id[0])

    return frame_processed, pose_array, markers


def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion) -> tuple:
    '''
    Estimates the rvec and tvec for a single detected marker's corners.

    corners - detected corners for one marker, shape (1, 4, 2)
    marker_size - size of the marker in meters
    camera_matrix - camera intrinsic matrix
    distortion - camera distortion coefficients
    RETURN tvec (3,1), rvec (3,1), quaternion (x,y,z,w) - all in camera optical frame
    '''
    marker_points = np.array([[-marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, -marker_size / 2.0, 0],
                              [-marker_size / 2.0, -marker_size / 2.0, 0]], dtype=np.float32)

    image_points = corners[0].reshape(4, 2).astype(np.float32)

    retval, rvec, tvec = cv2.solvePnP(marker_points, image_points, camera_matrix, distortion,
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)

    rvec = rvec.reshape(3, 1)
    tvec = tvec.reshape(3, 1)

    rot, jacobian = cv2.Rodrigues(rvec)

    # Apply the fixed marker-axis correction (see R_MARKER_AXIS_FIX comment
    # above) BEFORE building the quaternion, so every downstream consumer
    # (base_link transform, docking controller heading error) sees a
    # convention where 0 deg yaw means "facing the robot squarely."
    rot = rot @ R_MARKER_AXIS_FIX

    # Recompute rvec from the corrected rotation too, so the on-image
    # drawFrameAxes() overlay (drawn elsewhere using this rvec) visually
    # matches the corrected frame instead of ArUco's raw native one.
    rvec, _ = cv2.Rodrigues(rot)

    rot_matrix = np.eye(4, dtype=np.float64)
    rot_matrix[0:3, 0:3] = rot

    quaternion = tf_transformations.quaternion_from_matrix(rot_matrix)
    norm_quat = np.linalg.norm(quaternion)
    quaternion = quaternion / norm_quat if norm_quat > 0 else quaternion

    return tvec, rvec, quaternion


def depth_to_pointcloud_centroid(depth_image: np.array, intrinsic_matrix: np.array,
                                 corners: np.array) -> np.array:
    """
    Takes a depth image and the corners of a quadrilateral, returns the
    centroid (x, y, z) in meters of the corresponding pointcloud region.
    """

    height, width = depth_image.shape

    corners_indices = np.array([(int(x), int(y)) for x, y in corners[0]])

    for x, y in corners_indices:
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError("One or more corners are outside the image bounds.")

    x_min = int(min(corners_indices[:, 0]))
    x_max = int(max(corners_indices[:, 0]))
    y_min = int(min(corners_indices[:, 1]))
    y_max = int(max(corners_indices[:, 1]))

    points = []
    for x in range(x_min, x_max):
        for y in range(y_min, y_max):
            if is_pixel_in_polygon(pixel=(x, y), corners=corners_indices):
                points.append([x, y, depth_image[y, x]])

    if len(points) == 0:
        raise ValueError("No depth pixels found inside marker polygon.")

    # pixel indices + raw depth (mm) - fine as integers here
    points = np.array(points, dtype=np.uint16)

    pointcloud = []
    for x, y, d in points:
        z = d / 1000.0
        px = (x - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
        py = (y - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]
        pointcloud.append([px, py, z])

    # FIX: real-world coordinates are floats (meters) - must NOT be cast
    # to uint16, or every value under 1.0 truncates to 0 and every value
    # between 1.0-1.99 truncates to 1, etc. This was the source of the
    # 0.7-1.0m depth error.
    centroid = np.mean(np.array(pointcloud, dtype=np.float32), axis=0)

    return centroid


def is_pixel_in_polygon(pixel: tuple, corners: np.array) -> bool:
    """
    Ray-casting point-in-polygon test.
    """
    num_intersections = 0

    for i in range(len(corners)):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % len(corners)]

        if (y1 <= pixel[1] < y2) or (y2 <= pixel[1] < y1):
            x_intersection = (x2 - x1) * (pixel[1] - y1) / (y2 - y1) + x1
            if x_intersection > pixel[0]:
                num_intersections += 1

    return num_intersections % 2 == 1