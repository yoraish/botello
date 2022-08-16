'''
This script requires knowledge of:
1. Placements of all aruco_gridboards in the frame of grid0, which is the rotated map origin. This typically comes from botello_maps/config/aruco_boards.yaml but could also be hard coded here for testing.
2. The layout of each aruco gridboard within its own frame, and is stored in config/boardXX.yaml. The aruco board yamls are created with the create)aruco_gridboard_images_and_yamls.py script.
3. This script outputs a unified yaml file of all the aruco markers from all the specified boards (botello_maps/config/aruco_boards.yaml) and saves it to config/botello_layout.yaml. All boards are in the board-map frame, which is a rotated map-origin frame and in mm.

'''

import os
from matplotlib import projections
from scipy.spatial.transform import Rotation
import numpy as np
import yaml
import matplotlib.pyplot as plt


# ==============================
# Get boards map poses.
# ==============================
# Get the placement of the boards from the maps aruco boards yaml file. All in the same frame, the map frame.
boards_map_path = "../../botello_maps/config/aruco_boards.yaml"
boards_in_map = {} # Mapping name to [x,y,z,yaw] in [m] and [rad].
with open(boards_map_path, 'rb') as y:
        boards_in_map = yaml.safe_load(y)

# ==============================
# Convert boards map poses to 
# board0-frame.
# Hard code that here for testing.
# ==============================
# The following is in the format of [x, y, z, roll, pitch, yaw]. In mm and in radians.
# boards_in_board0 = {"board0": [0,0,0,0,0,0], "board00": [800, 0.0, 0.0, 0, 0, 0], "board10": [-500.0, 0.0, 2900.0, 0, -3.141592653, 0]} # All in mms.
# boards_in_board0 = {"board0": [0,0,0,0,0,0]} # All in mms.
boards_in_board0 = {b_name : [b_data[1] * 1000, b_data[2] * 1000, b_data[0] * 1000, 0, b_data[3], 0] for b_name, b_data in boards_in_map.items()}

# Get all board yaml files.
board_yaml_paths = ["../config/" + f for f in os.listdir("../config") if f.split(".")[-1] == "yaml" and f.split(".")[0][:5] == "board"]

# Parse each yaml as a dictionary mapping aruco id to a matrix (where each matrix is made out of concatenated fiducial corners).
out_yaml_corners = "corners:\n"
out_yaml_ids = "ids:\n"

# Visualization of the z-x plane.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for board_yaml_path in board_yaml_paths:
    with open(board_yaml_path, 'rb') as y:
        board_data = yaml.safe_load(y)
        board_name = board_yaml_path.split("/")[-1].split(".yaml")[0]
        if board_name not in boards_in_board0:
            continue
        print("\n\nProcessing ", board_name)
  
        board_corners = np.array(board_data["corners"])

        # Stack in order to transform all points at once (resize from n x 4 x 3 to 4*n x 3 where n is the number of fiducials on the board). Also transpose to work with column vectors.
        board_corners = board_corners.reshape((-1, 3)).T
        print("Original corners.")
        print(board_corners)

        # Get the desired transform between the origin board and this board.
        board_in_global = boards_in_board0[board_name]
        t = np.array([board_in_global[:3]]).T
        R = Rotation.from_euler("xyz", [board_in_global[3], board_in_global[4], board_in_global[5]]).as_matrix()
        print("R,t:")
        print(R)
        print(t)

        # Transform the points.
        transformed_corners = R.dot(board_corners) + t

        # Visualize.
        z = transformed_corners[2, :]
        y = transformed_corners[1, :]
        x = transformed_corners[0, :]
        ax.scatter(x, y, z)
        # TODO(yoraish): Draw normal vector arrow (z axis of the fiducial).
        # Find slope between first two points. Negate and invert for perpendicular slope. Create vector from the middle of the tag (mean of first two points).
        

        # Reshape back to have groups of 4 fiducials together (n x 4 x 3) and recover the transformed corners per tag. Transpose again to get row vectors.
        transformed_corners = transformed_corners.T.reshape((-1,4,3))
        print("Transformed corners.")
        print(transformed_corners)

        # Append the new data to the output yaml (combined).
        out_yaml_corners += "\n".join([" - " + str(transformed_corners[i].tolist()) for i in range(transformed_corners.shape[0])]) + "\n"
        out_yaml_ids += "\n".join([" - " + str(i) for i in board_data["ids"]]) + "\n"

        print("Adding to yaml:\n")
        print(out_yaml_corners)
        print(out_yaml_ids)


# ax.set_aspect('equal', 'box') # Not supported by 3Dplot.
plt.show()

out_yaml = "%YAML:1.0\nmm_per_unit: 1.0\n"
out_yaml += out_yaml_corners + out_yaml_ids
with open ("../../botello_maps/config/botello_layout.yaml", 'w') as f:
    f.write(out_yaml)