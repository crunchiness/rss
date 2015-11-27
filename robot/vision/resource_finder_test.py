import numpy as np
import cv2
from robot.vision.vision import Vision
from robot.vision.calculations import correct_orientation_angle, bottom_point


class MockIO:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 800)
        self.cap.set(4, 600)

    def cameraSetResolution(self, smth):
        return

    def cameraGrab(self):
        self.cap.grab()
        return

    def cameraRead(self):
        ret, frame = self.cap.read()
        return frame

    def imshow(self, x, y):
        cv2.imshow(x, y)
        cv2.waitKey(5)


def main():
    io = MockIO()
    vision = Vision(io)
    while True:
        resources, image = vision.see_resources('mario')
        try:
            x, y = resources['mario']['mean']
            turn_angle = correct_orientation_angle((x, y), (800.0, 600.0))
            bp = bottom_point(turn_angle, 800.0)
            cv2.line(image, (bp, 600), (int(x), int(y)), (100, 0, 200), 2)
            print str(turn_angle * 180.0 / np.pi) + ' degrees'
        except KeyError:
            pass
        io.imshow('Window', image)
main()
