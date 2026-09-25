"""
generate_marker_image.py
--------------------------
Generates individual ArUco marker PNGs for printing (real hardware) or for
use as Gazebo textures. Not a ROS node -- plain OpenCV utility, run directly
or via `ros2 run aruco_pose_estimation generate_marker_image`.

Defaults match this project's config/aruco_parameters.yaml
(aruco_dictionary_id: DICT_4X4_50, marker_size: 0.2 m).

Usage:
    ros2 run aruco_pose_estimation generate_marker_image --ids 0 1 2 3 --marker_size 0.2 --out markers
    python3 generate_marker_image.py --ids 0 1 2 3 --pixels 600 --dict DICT_4X4_50 --out markers
"""

import argparse
import os
import cv2
import numpy as np

from forklift_aruco.aruco_dicts import get_dictionary, ARUCO_DICTS


def generate_marker(dictionary, marker_id, size_px, border_bits=1):
    return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px, borderBits=border_bits)


def add_printable_border(marker_img, marker_id, dict_name, marker_size_m, quiet_zone=40):
    h, w = marker_img.shape[:2]
    label_h = 60
    canvas = np.full((h + 2 * quiet_zone + label_h, w + 2 * quiet_zone, 3), 255, dtype=np.uint8)
    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    canvas[quiet_zone:quiet_zone + h, quiet_zone:quiet_zone + w] = marker_bgr

    font = cv2.FONT_HERSHEY_SIMPLEX
    line1 = f"{dict_name}  id={marker_id}"
    line2 = f"side length = {marker_size_m:.3f} m -- print at actual size, no scaling"
    for i, text in enumerate([line1, line2]):
        size = cv2.getTextSize(text, font, 0.55, 1)[0]
        x = (canvas.shape[1] - size[0]) // 2
        y = quiet_zone + h + 25 + i * 22
        cv2.putText(canvas, text, (x, y), font, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description="Generate ArUco marker images")
    parser.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3])
    parser.add_argument("--pixels", type=int, default=600, help="Output image resolution in pixels (default: 600)")
    parser.add_argument("--marker_size", type=float, default=0.2,
                         help="Real-world side length in meters, only used for the printed label (default: 0.2)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", choices=list(ARUCO_DICTS.keys()))
    parser.add_argument("--out", type=str, default="markers")
    parser.add_argument("--no-label", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    dictionary = get_dictionary(args.dict)
    max_id = int(args.dict.split("_")[-1]) if args.dict.split("_")[-1].isdigit() else None

    for marker_id in args.ids:
        if max_id is not None and marker_id >= max_id:
            print(f"Skipping id={marker_id}: {args.dict} only supports ids 0-{max_id - 1}")
            continue
        marker_img = generate_marker(dictionary, marker_id, args.pixels)
        if not args.no_label:
            marker_img = add_printable_border(marker_img, marker_id, args.dict, args.marker_size)
        filename = os.path.join(args.out, f"marker_{args.dict}_id{marker_id}.png")
        cv2.imwrite(filename, marker_img)
        print(f"Saved {filename}")

    print(f"\nDone. Saved to '{args.out}/'. Dictionary must match aruco_dictionary_id param ({args.dict}).")


if __name__ == "__main__":
    main()
