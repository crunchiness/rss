__author__ = 'crunch'
import numpy as np

CAMERA_ANGLE = 0.5 * np.pi  # pi/2 is pointing straight ahead, 0 is pointing to the ground
HEIGHT = 7.0     # camera height
DISTANCE = 24.0  # distance from center of the robot
DIAGONAL_FOV = np.pi / 3.0  # field of view
RATIO_W = 4.0
RATIO_H = 3.0
RATIO_D = 5.0  # diagonal
VERTICAL_FOV = 2 * np.arctan((RATIO_H / RATIO_D) * np.tan(DIAGONAL_FOV / 2.0))
HORIZONTAL_FOV = 2 * np.arctan((RATIO_W / RATIO_D) * np.tan(DIAGONAL_FOV / 2.0))

