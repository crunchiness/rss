#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time

from robot.body.motors import Motors
from robot.body.sensors import Sensors
from robot.vision.vision import Vision
from robot.state.particle_filtering import Particles
from robot.visualisation.drawing import Drawing
from robot.localization_logic import wander_and_travel

from robot.utils import log, collect_front_IR_and_sonar_measurements


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
            # wander_and_travel(self.sensors, self.particles, self.motors, self.vision)
            collect_front_IR_and_sonar_measurements(self.io)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, ok):
        while ok():
            self.vision.do_image()
