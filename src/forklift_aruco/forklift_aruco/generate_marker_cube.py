"""
generate_marker_cube.py
--------------------------
Generates a 6-face ArUco marker tile map (as a texture sheet) for a cube
model in Gazebo, plus a marker_info.yml mapping each cube face to a marker
ID. Ported from a script using the legacy cv2.aruco API
(aruco.Dictionary_get / aruco.drawMarker) to the current API
(cv2.aruco.getPredefinedDictionary / cv2.aruco.generateImageMarker).

Layout (matches the original): a 4x3 grid of tiles, where only the middle
row (i=1) is fully populated left-to-right, and the top/bottom rows only
have a tile in the middle column (j=1) -- forming a cross/net shape you'd
fold into a cube:

        [ ] [top   ] [ ]
        [left][front][right][back]
        [ ] [bottom] [ ]

Not a ROS node -- plain OpenCV/click utility. Run via:
    ros2 run forklift_aruco generate_marker_cube ./out --dict DICT_4X4_50 --tile_size 200

Requires:
    pip install click opencv-contrib-python numpy pyyaml
"""

import os
import click
import cv2
import numpy as np
import yaml

from forklift_aruco.aruco_dicts import get_dictionary, ARUCO_DICTS


class MarkerFactory:
    def __init__(self, dictionary):
        self.dictionary = dictionary

    def create_marker(self, size, marker_id, margin):
        # White background, marker centered with a quiet-zone margin
        img = 255 * np.ones((size, size), dtype=np.uint8)
        img_marker = cv2.aruco.generateImageMarker(self.dictionary, marker_id, size - 2 * margin)
        img[margin:-margin, margin:-margin] = img_marker
        return img


class TileMap:
    def __init__(self, tile_size, grid_shape=(4, 3)):
        self.tile_size = tile_size
        self._map = 255 * np.ones((grid_shape[0], grid_shape[1], tile_size, tile_size), dtype=np.uint8)

    def set_tile(self, pos: tuple, img: np.ndarray):
        assert np.all(self._map[pos[0], pos[1]].shape == img.shape), "tile/image size mismatch"
        self._map[pos[0], pos[1]] = img

    def get_map_image(self):
        """Merges the tile grid into a single flat image (cross/net layout)."""
        img = np.concatenate(self._map, axis=-1)
        img = np.concatenate(img, axis=-2)
        img = img.T
        return img


@click.command()
@click.argument("path", type=click.Path(exists=True))
@click.option("--tile_size", type=int, default=200, help="Pixel size of each cube-face tile (default: 200)")
@click.option("--dict", "dict_name", type=click.Choice(list(ARUCO_DICTS.keys())), default="DICT_4X4_50",
              help="ArUco dictionary to draw from (default: DICT_4X4_50, matches aruco_dictionary_id param)")
@click.option("--box_size", type=float, default=0.1,
              help="Physical size of one cube face in meters (must match the box/mesh in Gazebo)")
@click.option("--marker_size", type=float, default=0.35, help="Cube edge length in meters, recorded in marker_info.yml (default: 0.2)")
@click.option("--start_id", type=int, default=0, help="First marker ID to assign (default: 0)")
def main(path, tile_size, dict_name, marker_size, box_size, start_id):
    if marker_size > box_size:
        raise click.UsageError("marker_size can't exceed box_size")
    margin_frac = (box_size - marker_size) / (2 * box_size)
    margin = int(margin_frac * tile_size)
    # margin = int(0.15 * tile_size)  # slightly tighter quiet zone than the 0.3 original, keeps marker larger
    dictionary = get_dictionary(dict_name)
    marker_factory = MarkerFactory(dictionary)
    tile_map = TileMap(tile_size)

    # Net layout: cross shape -> 6 faces of a cube
    # (row, col) positions used, matching the original script's grid
    face_positions = [
        (0, 1),  # top
        (1, 0),  # left
        (1, 1),  # front
        (1, 2),  # right (note: original grid only has 3 cols; back needs a 4th -> handled via wide grid below)
    ]

    order = ["left", "front", "right", "back", "top", "bottom"]
    ids = []
    marker_id = start_id

    # Build using a 4-row x 3-col grid like the source script:
    # row 0: (blank, top,    blank)
    # row 1: (left,  front,  right)   <- only 3 slots per row in this grid, "back" doesn't fit a 4x3 grid
    # row 2: (blank, bottom, blank)
    # row 3: (blank, back,   blank)   <- reuse row 3 middle column for "back"
    layout = {
        "top": (0, 1),
        "left": (1, 0),
        "front": (1, 1),
        "right": (1, 2),
        "bottom": (2, 1),
        "back": (3, 1),
    }

    for face in order:
        marker_img = marker_factory.create_marker(tile_size, marker_id, margin)
        tile_map.set_tile(layout[face], marker_img)
        ids.append(marker_id)
        marker_id += 1

    tile_img = tile_map.get_map_image()

    cv2.imwrite(os.path.join(path, "marker_cube_net.png"), tile_img)

    # Also save each face as its own texture file -- easier to wire into a
    # Gazebo Fortress <material><script> per-link/per-face texture setup
    # than cropping the net image by hand.
    face_dictionary = get_dictionary(dict_name)
    face_factory = MarkerFactory(face_dictionary)
    for face, fid in zip(order, ids):
        face_img = face_factory.create_marker(tile_size, fid, margin)
        cv2.imwrite(os.path.join(path, f"marker_cube_face_{face}_id{fid}.png"), face_img)

    marker_config = dict(zip(order, ids))
    config = {
        "aruco_dict": dict_name,
        "marker_size": marker_size,
        "markers": marker_config,
    }
    with open(os.path.join(path, "marker_info.yml"), "w") as yml_file:
        yaml.dump(config, yml_file, default_flow_style=False)

    print(f"Saved net image, per-face textures, and marker_info.yml to '{path}'")
    print(f"Face -> marker ID mapping: {marker_config}")


if __name__ == "__main__":
    main()
