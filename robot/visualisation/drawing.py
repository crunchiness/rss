import datetime
import os
import numpy as np
import cv2

from robot.state.map import X_MAX, Y_MAX, ARENA_WALLS


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
            pass
            # raise Exception('Invalid coordinates x={0} y={1}'.format(x, y))

    def add_big_point(self, x, y, color=(0, 255, 0)):
        cv2.circle(self.image, (int(x), int(y)), radius=2, color=color, thickness=3)

    def save(self, name=None, path=None):
        """Saves to file and starts new drawing"""
        base_path = os.path.dirname(os.path.dirname(__file__)).replace('/robot', '')
        path = base_path + '/log/' if path is None else base_path + '/' + path
        if not os.path.exists(path):
            os.makedirs(path)
        if name is None:
            name = 'local-{0}.png'.format(datetime.datetime.now().isoformat())
        result = cv2.imwrite(path + name, cv2.flip(self.image, 1))
        if not result:
            print 'Failed to save image ' + path + name
        self.image = np.copy(self.template)
