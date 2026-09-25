"""
calibration_camera.py
--------------------------
Two calibration modes:

  --mode sim (default)
      Subscribes to the camera_info_topic param and, on receiving the first
      message, saves camera_matrix/dist_coeffs to a YAML file. This is the
      right mode for Gazebo Fortress: the simulated camera plugin already
      publishes exact ground-truth intrinsics, so no physical chessboard
      capture is needed.

  --mode chessboard
      For real hardware without a working camera_info publisher. Opens
      /dev/videoN directly (NOT via ROS topics) and runs a standard
      OpenCV chessboard calibration, same procedure as calibrating any
      other webcam.

Usage:
    ros2 run aruco_pose_estimation calibration_camera --ros-args -p mode:=sim \
        -p camera_info_topic:=/forklift/camera/camera_info -p out_file:=calib.yaml

    ros2 run aruco_pose_estimation calibration_camera --ros-args -p mode:=chessboard \
        -p camera_index:=0 -p cols:=9 -p rows:=6 -p square_size:=0.025 -p out_file:=calib.yaml
"""

import sys
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoCalibrationSaver(Node):
    """Mode 'sim': dump an incoming CameraInfo message to a calibration YAML."""

    def __init__(self):
        super().__init__("calibration_camera_sim")
        self.declare_parameter("camera_info_topic", "/forklift/camera/camera_info")
        self.declare_parameter("out_file", "calib.yaml")

        topic = self.get_parameter("camera_info_topic").value
        self.out_file = self.get_parameter("out_file").value

        self.get_logger().info(f"Waiting for one CameraInfo message on '{topic}'...")
        self.sub = self.create_subscription(CameraInfo, topic, self.callback, 10)

    def callback(self, msg: CameraInfo):
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(msg.d, dtype=np.float64)

        data = {
            "image_width": msg.width,
            "image_height": msg.height,
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.tolist(),
            "distortion_model": msg.distortion_model,
        }
        with open(self.out_file, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

        # Also save a .npz for scripts that expect calib.npz (pose_estimate.py can read either)
        npz_path = self.out_file.rsplit(".", 1)[0] + ".npz"
        np.savez(npz_path, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

        self.get_logger().info(f"Saved calibration to '{self.out_file}' and '{npz_path}'")
        self.get_logger().info(f"camera_matrix:\n{camera_matrix}")
        self.get_logger().info(f"dist_coeffs: {dist_coeffs}")

        rclpy.shutdown()


def run_chessboard_mode(node_args):
    """Mode 'chessboard': real hardware calibration via direct OpenCV camera capture."""
    rclpy.init(args=node_args)
    node = Node("calibration_camera_chessboard")
    node.declare_parameter("camera_index", 0)
    node.declare_parameter("cols", 9)
    node.declare_parameter("rows", 6)
    node.declare_parameter("square_size", 0.025)
    node.declare_parameter("out_file", "calib.yaml")
    node.declare_parameter("min_samples", 10)

    camera_index = node.get_parameter("camera_index").value
    cols = node.get_parameter("cols").value
    rows = node.get_parameter("rows").value
    square_size = node.get_parameter("square_size").value
    out_file = node.get_parameter("out_file").value
    min_samples = node.get_parameter("min_samples").value
    pattern_size = (cols, rows)

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size

    objpoints, imgpoints = [], []
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        node.get_logger().error(f"Could not open camera index {camera_index}")
        rclpy.shutdown()
        return

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    frame_size = None
    sample_count = 0

    node.get_logger().info(f"Press 'c' to capture (need >= {min_samples}), 'q'/ESC to finish.")

    while rclpy.ok():
        ok, frame = cap.read()
        if not ok:
            break
        frame_size = frame.shape[1::-1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, pattern_size, corners, found)
        cv2.putText(display, f"Samples: {sample_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("Chessboard Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("c") and found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            sample_count += 1
            node.get_logger().info(f"Captured sample {sample_count}")

    cap.release()
    cv2.destroyAllWindows()

    if sample_count < min_samples:
        node.get_logger().error(f"Only {sample_count} samples (need {min_samples}). Aborting.")
        rclpy.shutdown()
        return

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, frame_size, None, None
    )
    if not ret:
        node.get_logger().error("Calibration failed.")
        rclpy.shutdown()
        return

    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        pts_detected = np.asarray(imgpoints[i], dtype=np.float32).reshape(-1, 2)
        pts_projected = np.asarray(imgpoints2, dtype=np.float32).reshape(-1, 2)
        total_error += cv2.norm(pts_detected, pts_projected, cv2.NORM_L2) / len(pts_projected)
    mean_error = total_error / len(objpoints)
    node.get_logger().info(f"Mean reprojection error: {mean_error:.4f} px")

    data = {
        "image_width": frame_size[0],
        "image_height": frame_size[1],
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.ravel().tolist(),
        "distortion_model": "plumb_bob",
    }
    with open(out_file, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    npz_path = out_file.rsplit(".", 1)[0] + ".npz"
    np.savez(npz_path, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    node.get_logger().info(f"Saved calibration to '{out_file}' and '{npz_path}'")

    rclpy.shutdown()


def main(args=None):
    # Peek at the 'mode' param from CLI args before full rclpy init, since the
    # two modes construct very different node graphs.
    mode = "sim"
    for i, a in enumerate(sys.argv):
        if a == "-p" and i + 1 < len(sys.argv) and sys.argv[i + 1].startswith("mode:="):
            mode = sys.argv[i + 1].split(":=", 1)[1]

    if mode == "chessboard":
        run_chessboard_mode(args)
        return

    rclpy.init(args=args)
    node = CameraInfoCalibrationSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()


if __name__ == "__main__":
    main()
