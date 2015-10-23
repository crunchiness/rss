import numpy as np
import cv2
import datetime
from state.map import X_MAX, Y_MAX, ARENA_WALLS


class Drawing:
    def __init__(self):
        """Draw map (template)"""
        img = np.zeros((Y_MAX, X_MAX, 3), np.uint8)
        cols, rows, _ = img.shape

        for line_segment in ARENA_WALLS:
            start_point, end_point = line_segment
            end_point = np.add(start_point, end_point)
            cv2.line(img, tuple(map(int, start_point)), tuple(map(int, end_point)), (255, 0, 0), 2)
        self.template = img
        self.image = np.copy(self.template)

    def add_point(self, x, y):
        """Adds a red dot at given coordinates"""
        try:
            self.image[int(y)][int(x)][0] = 0
            self.image[int(y)][int(x)][2] = 255
        except IndexError:
            raise Exception('Invalid coordinates x={0} y={1}'.format(x, y))

    def add_big_point(self, x, y):
        cv2.circle(self.image, (int(y), int(x)), radius=2, color=(0, 255, 0), thickness=3)

    def save(self):
        """Saves to file and starts new drawing"""
        cv2.imwrite('local-{0}.png'.format(datetime.datetime.now().isoformat()), cv2.flip(self.image, 1))
        self.image = np.copy(self.template)
