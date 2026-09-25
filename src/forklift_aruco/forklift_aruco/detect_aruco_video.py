"""
detect_aruco_video.py
--------------------------
ROS2 node: subscribes to the live camera image topic, runs ArUco detection
on every frame, and publishes an annotated debug image plus the list of
detected marker IDs. This node only detects/draws -- see pose_estimate.py
for 6DOF pose + TF broadcasting.

Subscribes:
    image_topic (sensor_msgs/Image)         default: /forklift/camera/image_raw

Publishes:
    annotated_image_topic (sensor_msgs/Image)   default: /forklift/camera/aruco/image
    <annotated_image_topic>/marker_ids (std_msgs/Int32MultiArray)

Params (see config/aruco_parameters.yaml):
    image_topic, aruco_dictionary_id, annotated_image_topic
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

from forklift_aruco.aruco_dicts import get_dictionary
from forklift_aruco.ros_image_utils import to_imgmsg_bgr8


class ArucoVideoDetector(Node):
    def __init__(self):
        super().__init__("detect_aruco_video")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50")
        self.declare_parameter("annotated_image_topic", "/aruco/image")

        image_topic = self.get_parameter("image_topic").value
        dict_id = self.get_parameter("aruco_dictionary_id").value
        annotated_topic = self.get_parameter("annotated_image_topic").value

        dictionary = get_dictionary(dict_id)
        detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, annotated_topic + "/marker_ids", 10)

        self.get_logger().info(
            f"Subscribed to '{image_topic}', publishing annotated frames to '{annotated_topic}' "
            f"using dictionary '{dict_id}'"
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        id_list = []
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            id_list = ids.flatten().tolist()
            for i, marker_id in enumerate(id_list):
                c = corners[i][0]
                cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
                cv2.putText(frame, f"ID {marker_id}", (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, f"Markers: {len(id_list)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

        out_msg = to_imgmsg_bgr8(frame, msg.header)
        self.annotated_pub.publish(out_msg)
        self.ids_pub.publish(Int32MultiArray(data=id_list))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoVideoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()