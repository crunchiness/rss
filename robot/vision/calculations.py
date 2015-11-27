import numpy as np
from robot.vision import constants as vision


def bottom_point(turn_angle, screen_width):
    """
    Calculates bottom pixel x coordinate, from this point we can draw line that indicates robot position after the turn
    :param turn_angle: angle by which robot will turn
    :param screen_width:
    :return: pixel x coordinate
    """
    alpha = vision.CAMERA_ANGLE - vision.VERTICAL_FOV / 2.0
    h = vision.HEIGHT / np.cos(alpha)
    x = 2.0 * h * np.tan(vision.HORIZONTAL_FOV / 2.0)
    y = np.tan(turn_angle) * (vision.DISTANCE + vision.HEIGHT * np.tan(alpha))
    pixel_x = screen_width * (x - y) / x
    pixel_x = 800 - int(800.0 - pixel_x / 2.0)  # ??
    return pixel_x


def correct_orientation_angle((x, y), (screen_width, screen_height)):
    """
    Calculates by what angle robot should turn to face the point, given it's coordinates on the screen
    :return: angle in radians
    """
    camera_angle = 0.5 * np.pi  # pi/2 is pointing straight ahead, 0 is pointing to the ground
    alpha = ((screen_width / 2.0 - x) / screen_width) * vision.HORIZONTAL_FOV
    beta = ((screen_height / 2.0 - y) / screen_height) * vision.VERTICAL_FOV
    gamma = camera_angle + beta
    numerator = vision.HEIGHT * np.tan(alpha) * np.tan(gamma)
    denominator = vision.HEIGHT * np.tan(gamma) + vision.DISTANCE
    return np.arctan(numerator / denominator)
