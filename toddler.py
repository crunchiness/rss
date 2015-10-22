#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time
from body.motors import Motors
from body.sensors import Sensors, SensorRunningAverage
from vision.vision import Vision


class Toddler:
    def __init__(self, io):
        print 'Toddler, the dragon has awoken... Beware.'
        self.io = io
        self.motors = Motors(io)
        self.sensors = Sensors(io)
        self.vision = Vision(io)

    def stop(self):
        """For development only"""
        self.motors.halt()
        time.sleep(1000)

    def avoid_obstacles(self):
        left_ir = SensorRunningAverage()
        right_ir = SensorRunningAverage()

        left_avg = left_ir.get_avg()
        right_avg = right_ir.get_avg()

        while left_avg > 15 and right_avg > 15:
            left_ir.add_value(self.sensors.get_ir_left())
            right_ir.add_value(self.sensors.get_ir_right())
            self.motors.move(100, 100)
            left_avg = left_ir.get_avg()
            right_avg = right_ir.get_avg()

        if left_avg <= 15:
            self.motors.turn_by(30)
        elif right_avg <= 15:
            self.motors.turn_by(-30)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, ok):
        while ok():
            self.avoid_obstacles()

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, ok):
        while ok():
            self.vision.do_image()
