"""
ros_image_utils.py
--------------------------
Manual sensor_msgs/Image construction for BGR8 numpy frames.

Works around a known cv_bridge issue on ROS2 Humble where a pip-installed
OpenCV build (e.g. opencv-contrib-python) has a numpy ABI that doesn't match
what the apt-installed cv_bridge C extension expects. This causes
`CvBridge.cv2_to_imgmsg(frame, encoding="bgr8")` to raise
`KeyError: 16` (16 = cv2.CV_8UC3) even though the frame is a perfectly
valid BGR8 image.

`imgmsg_to_cv2` (the *incoming* direction, used for subscriptions) is not
affected by this bug, so cv_bridge is still used for that side.
"""

import numpy as np
from sensor_msgs.msg import Image


def to_imgmsg_bgr8(frame: np.ndarray, header) -> Image:
    """Build a sensor_msgs/Image (encoding='bgr8') from an OpenCV BGR numpy array."""
    frame = np.ascontiguousarray(frame, dtype=np.uint8)
    height, width, channels = frame.shape
    if channels != 3:
        raise ValueError(f"Expected a 3-channel BGR frame, got shape {frame.shape}")

    msg = Image()
    msg.header = header
    msg.height = height
    msg.width = width
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = width * channels
    msg.data = frame.tobytes()
    return msg