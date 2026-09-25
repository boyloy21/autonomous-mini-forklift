"""
detect_aruco_image.py
--------------------------
Detects ArUco markers in a single static image file. Not a ROS node --
useful for sanity-checking a marker/dictionary/lighting setup (e.g. a frame
grabbed from Gazebo with `ros2 topic echo` + image saver, or rqt_image_view)
before running the live detect_aruco_video.py node.

Usage:
    ros2 run aruco_pose_estimation detect_aruco_image --image frame.png --dict DICT_4X4_50
    python3 detect_aruco_image.py --image frame.png --dict DICT_4X4_50 --out annotated.png
"""

import argparse
import cv2

from forklift_aruco.aruco_dicts import get_dictionary, ARUCO_DICTS


def main():
    parser = argparse.ArgumentParser(description="Detect ArUco markers in a static image")
    parser.add_argument("--image", type=str, required=True, help="Path to input image file")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", choices=list(ARUCO_DICTS.keys()))
    parser.add_argument("--out", type=str, default=None, help="Path to save annotated output (default: shows a window)")
    args = parser.parse_args()

    frame = cv2.imread(args.image)
    if frame is None:
        print(f"Error: could not read image '{args.image}'")
        return

    dictionary = get_dictionary(args.dict)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is None:
        print("No markers detected.")
    else:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        print(f"Detected {len(ids)} marker(s): {ids.flatten().tolist()}")
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
            print(f"  id={marker_id}  center_px=({cx}, {cy})  corners_px={c.tolist()}")

    if args.out:
        cv2.imwrite(args.out, frame)
        print(f"Saved annotated image to '{args.out}'")
    else:
        cv2.imshow("Detected Markers", frame)
        print("Press any key to close the window.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
