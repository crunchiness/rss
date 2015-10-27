#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time

from body.motors import Motors
from body.sensors import Sensors, SensorRunningAverage
from vision.vision import Vision
from state.particle_filtering import Particles
from visual import Drawing
from localization_logic import wander_and_travel


# TODO: sanity check jumping between rooms
class Toddler:
    def __init__(self, io):
        print 'Toddler, the dragon has awoken... Beware.'
        self.io = io
        self.motors = Motors(io)
        self.sensors = Sensors(io)
        self.vision = Vision(io)
        self.particles = Particles(drawing=Drawing())

    def stop(self):
        """For development only"""
        self.motors.halt()
        time.sleep(1000)

    def view_360(self):
        n = 12
        for i in xrange(n):
            self.vision.do_image()
            self.motors.turn_by(360/n)
        self.stop()

    def avoid_obstacles(self):
        front_ir = SensorRunningAverage()
        right_ir = SensorRunningAverage()

        front_avg = front_ir.get_avg()
        right_avg = right_ir.get_avg()

        while front_avg > 15 and right_avg > 15:
            front_ir.add_value(self.sensors.get_ir_front())
            right_ir.add_value(self.sensors.get_ir_right())
            self.motors.move(100, 100)
            front_avg = front_ir.get_avg()
            right_avg = right_ir.get_avg()

        if front_avg <= 15:
            self.motors.turn_by(30)
        elif right_avg <= 15:
            self.motors.turn_by(-30)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, ok):
        while ok():
            wander_and_travel(self.sensors, self.particles, self.motors)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, ok):
        while ok():
            pass
            # self.vision.do_image()
