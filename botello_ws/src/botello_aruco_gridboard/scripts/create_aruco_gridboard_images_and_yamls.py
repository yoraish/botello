import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import numpy as np

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 800)
fontScale              = 0.5
fontColor              = (0,0,255)
thickness              = 1
lineType               = 2


marker_length = .067
marker_spacing = .02
grid_cols = 2
grid_rows = 3

# Populate after printing, these numbers will be used to create the corners yaml.
real_marker_length_mm =  marker_length * 1000
real_marker_spacing_mm = marker_spacing * 1000

bottom_left_corners = np.array([[0, 1, 0], [1, 1, 0], [1, 0, 0], [0, 0, 0]]) * real_marker_length_mm


for first_marker in [0, 10, 20, 30, 40]:
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    board = aruco.GridBoard_create(grid_cols, grid_rows, marker_length, marker_spacing, aruco_dict, firstMarker = first_marker)
    img = cv2.aruco_GridBoard.draw(board, (1000, 1200))
    ids = board.ids.reshape((grid_rows, grid_cols))

    cv2.putText(img, "ids:" + str(ids), 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
    board_suffix = str(int(first_marker/10)*10)
    if len(board_suffix) < 2:
        board_suffix = "0" + board_suffix
    board_frame_name = "board" + board_suffix
    board_img_path = "../data/" + board_frame_name + ".png"
    cv2.imwrite(board_img_path, img)
    print("\nCreated board image at: " + board_img_path)
    
    yaml_string = """#%YAML:1.0
mm_per_unit: 1.0
corners:
"""
    for i in range(grid_rows):
        for j in range(grid_cols):

            row = str((bottom_left_corners + np.array([j*real_marker_length_mm + j*real_marker_spacing_mm, i*real_marker_length_mm + i*real_marker_spacing_mm, 0])).tolist())
            # Add the row to the yaml.
            yaml_string += " - " + row + "\n"

    yaml_string += "ids:\n"
    for id_row in list(reversed(ids.tolist())):
        for id in id_row:
            yaml_string += " - " + str(id) + "\n"

    board_yaml_path = "../config/" + board_frame_name + ".yaml"
    with open (board_yaml_path, 'w') as f:
        f.write(yaml_string)
    print("Created YAML for board at: " + board_yaml_path)

# cv2.imshow("aruco", img)
# cv2.waitKey()
