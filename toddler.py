#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time

from body.motors import Motors
from body.sensors import Sensors, SensorRunningAverage
from vision.vision import Vision
from state.particle_filtering import Particles
from visual import Drawing
from localization_logic import wander_and_travel

from vision_collection import vision_collection


# TODO: sanity check jumping between rooms
class Toddler:
    def __init__(self, io):
        print 'Toddler, the dragon has awoken... Beware.'
        self.io = io
        self.motors = Motors(io)
        self.sensors = Sensors(io)
        self.vision = Vision(io)
        self.particles = Particles(n=1000, drawing=Drawing(), where='1base')

    def stop(self):
        """For development only"""
        self.motors.halt()
        time.sleep(1000)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, ok):
        while ok():
            wander_and_travel(self.sensors, self.particles, self.motors, self.vision)
            # vision_collection(self.sensors, self.motors)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, ok):
        while ok():
            self.vision.do_image()
