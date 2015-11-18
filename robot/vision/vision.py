"""Main vision class"""
import time
import numpy as np

from robot.vision.localisation import detect_pieces


class Vision:
    def __init__(self, io):
        self.io = io
        self.io.cameraSetResolution('low')
        self.belief = []

    def do_image(self):
        for i in range(0, 5):
            self.io.cameraGrab()
        img = self.io.cameraRead()
        if img.__class__ == np.ndarray:
            self.belief = detect_pieces(img, save=True, display=False)
            time.sleep(0.5)
