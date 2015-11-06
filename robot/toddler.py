#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time

from body.motors import Motors
from body.sensors import Sensors
from vision.vision import Vision
from robot.state.particle_filtering import Particles
from robot.visualisation.visual import Drawing
from localization_logic import wander_and_travel


# TODO: sanity check jumping between rooms
class Toddler:
    def __init__(self, io):
        print 'Toddler, the dragon has awoken... Beware.'
        self.io = io
        self.motors = Motors(io)
        self.sensors = Sensors(io)
        self.vision = Vision(io)
        self.particles = Particles(n=1200, drawing=Drawing())

    def stop(self):
        """For development only"""
        self.motors.halt()
        time.sleep(1000)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, ok):
        while ok():
            # perform_basic_milestone(self.sensors, self.motors)
            wander_and_travel(self.sensors, self.particles, self.motors, self.vision)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, ok):
        while ok():
            self.vision.do_image()