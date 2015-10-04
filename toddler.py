#!/usr/bin/env python
__TODDLER_VERSION__ = "1.0.0"

import time
import numpy
import cv2
import datetime

# Hardware test code
class Toddler:
    def __init__(self, IO):
        print 'I am a toddler playing in a sandboxz'
        self.IO = IO
        self.inp = [0, 0, 0, 0, 0, 0, 0, 0]

    def move(self, l=100, r=100):
        self.IO.setMotors(l, -r)

    # This is a callback that will be called repeatedly.

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            self.move(100, -100)
            time.sleep(0.05)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        self.IO.cameraSetResolution('low')
        hasImage = False
        res = 0
        sw = False
        swPrev = False
        while OK():
            if self.inp[4]:
                for i in range(0, 5):
                    self.IO.cameraGrab()
                img = self.IO.cameraRead()
                if img.__class__ == numpy.ndarray:
                    hasImage = True
                    cv2.imwrite('camera-' + datetime.datetime.now().isoformat() + '.png', img)
                    self.IO.imshow('window', img)
                    self.IO.setStatus('flash', cnt=2)
                    time.sleep(0.5)
            if hasImage:
                self.IO.imshow('window', img)

            sw = self.inp[5]
            if sw != swPrev and sw:
                res = (res + 1) % 4
                if res == 0:
                    self.IO.cameraSetResolution('low')
                    self.IO.setError('flash', cnt=1)
                if res == 1:
                    self.IO.cameraSetResolution('medium')
                    self.IO.setError('flash', cnt=2)
                if res == 2:
                    self.IO.cameraSetResolution('high')
                    self.IO.setError('flash', cnt=3)
                if res == 3:
                    self.IO.cameraSetResolution('full')
                    self.IO.setError('flash', cnt=4)
                time.sleep(0.5)
            swPrev = sw

            time.sleep(0.05)
