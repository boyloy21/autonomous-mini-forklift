"""
aruco_dicts.py
--------------------------
Shared lookup table so every script/node in this package refers to ArUco
dictionaries the same way (matches the `aruco_dictionary_id` ROS parameter,
e.g. "DICT_4X4_50").
"""

import cv2

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def get_dictionary(name: str):
    """Return a cv2.aruco Dictionary object for a string id like 'DICT_4X4_50'.
    Raises KeyError with a helpful message if the name is unknown."""
    name = name.upper()
    if name not in ARUCO_DICTS:
        raise KeyError(
            f"Unknown aruco_dictionary_id '{name}'. Valid options: {sorted(ARUCO_DICTS.keys())}"
        )
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[name])
