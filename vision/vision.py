"""Main vision class"""

import numpy as np
import datetime
import cv2  # 3.0.0-dev
import time


class Vision:
    def __init__(self, io):
        self.io = io
        self.io.cameraSetResolution('low')
        self.belief = ''

    def do_image(self):
        for i in range(0, 5):
            self.io.cameraGrab()
        img = self.io.cameraRead()
        if img.__class__ == np.ndarray:
            cv2.imwrite('camera-' + datetime.datetime.now().isoformat() + '.png', img)
            self.io.imshow('window', img)
            time.sleep(0.5)
