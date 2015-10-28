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
        name = None
        if img.__class__ == np.ndarray:
            name = 'camera-' + datetime.datetime.now().isoformat() + '.png'
            cv2.imwrite(name, img)
            self.io.imshow('window', img)
            time.sleep(0.5)
        return name
