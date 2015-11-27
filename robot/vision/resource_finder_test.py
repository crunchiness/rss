import cv2
from robot.vision.vision import Vision


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
        vision.see_resources('mario')

main()
