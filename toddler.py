#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time
from body.motors import Motors
from body.sensors import Sensors, SensorRunningAverage
from vision.vision import Vision
from state.particle_filtering import Particles
from visual import Drawing
from state.map import X_MAX, Y_MAX

DESTINATION_ROOM = 'D'

# When this is reached we are sure enough of our location
LOCALISATION_CONF = 0.6

# If we fall down to this level, start exploring again
LOCALISATION_CONF_BREAK = 0.4

def determine_room(x, y):
    # TODO: add obstacles
    if not 0 <= x < X_MAX:
        raise Exception("X coordinate doesn't make sense: {0}".format(x))
    if not 0 <= y < Y_MAX:
        raise Exception("Y coordinate doesn't make sense: {0}".format(y))

    if y < 132:
        # A or B
        if x < 143:
            return 'A'
        else:
            return 'B'
    elif y < 231-46:
        # C or B
        if x < 143:
            return 'C'
        else:
            return 'B'
    elif y < 231 - 46 + 49:
        # C, C2 or B
        if x < 143:
            return 'C'
        elif x < 143 + 34:
            return 'C2'
        else:
            return 'B'
    elif y < 132 + 162:
        # C or D
        if x < 143:
            return 'C'
        else:
            return 'D'
    elif y < 132 + 162 + 46:
        # E, C1 or D
        if x < 93:
            return 'E'
        elif x < 93 + 50:
            return 'C1'
        else:
            return 'D'
    elif y < 231 + 162:
        # E or D
        if x < 177:
            return 'E'
        else:
            return 'D'
    else:
        # E or F
        if x < 177:
            return 'E'
        else:
            return 'F'


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

    def localize_while_avoiding_obstacles(self):
        # sensor reading running average for low level actions
        front_ir = SensorRunningAverage()
        right_ir = SensorRunningAverage()

        front_ir_reading = self.sensors.get_ir_front()
        right_ir_reading = self.sensors.get_ir_right()

        front_ir.add_value(front_ir_reading)
        right_ir.add_value(right_ir_reading)

        # Main loop initialization
        self.particles.sense({
            'front_ir': front_ir_reading,
            'right_ir': right_ir_reading
        })
        x, y, o, xy_conf = self.particles.get_position_by_weight()

        # Main loop, while unsure of our location
        while xy_conf < LOCALISATION_CONF:
            # localization - driving around avoiding obstacles
            front_avg = front_ir.get_avg()
            right_avg = right_ir.get_avg()
            while front_avg > 15 and right_avg > 15:
                # Move forwards 10 cm
                self.motors.go_forward(10)
                self.particles.forward(10)
                # Update position via particle filter
                front_ir_reading = self.sensors.get_ir_front()
                right_ir_reading = self.sensors.get_ir_right()
                front_avg = front_ir.get_avg()
                right_avg = right_ir.get_avg()
                self.particles.sense({
                    'front_ir': front_ir_reading,
                    'right_ir': right_ir_reading
                })
                x, y, o, xy_conf = self.particles.get_position_by_weight()
                if xy_conf >= LOCALISATION_CONF:
                    # we are sure enough, go back to high level plan execution
                    break
                # Update running average
                front_ir.add_value(front_ir_reading)
                right_ir.add_value(right_ir_reading)

            if front_avg <= 15:
                self.motors.turn_by(30)
            elif right_avg <= 15:
                self.motors.turn_by(-30)
            # TODO: when we know where we are
        loc_room = determine_room(x, y)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, ok):
        while ok():
            self.localize_while_avoiding_obstacles()

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, ok):
        while ok():
            pass
            # self.vision.do_image()
