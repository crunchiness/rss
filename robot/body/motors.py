import time
import numpy as np
from math import pi
from robot.utils import log
# distance unit is cm

BRAKING_TIME_TRANSLATION = 0.2
BRAKING_TIME_ROTATION = 0.1
BRAKING_SPEED = 5
BRAKING_MULTIPLIER = 5

HALL_SENSOR_CHANGES_FOR_REVOLUTION = 2

DIAMETER = 16.0  # distance between (the middle of) wheels
WHEEL = 8.4  # wheel diameter
WHEEL_PERIMETER = WHEEL * pi  # because diameter
HALL_REVS_TO_WHEEL_REVS = 0.2
HALL_PERIMETER = HALL_REVS_TO_WHEEL_REVS * WHEEL_PERIMETER
HALL_ANGLE = HALL_PERIMETER / (0.5 * DIAMETER)

log('Distance per single hall sensor revolution: {}'.format(HALL_PERIMETER))
log('Angle per single hall sensor revolution: {}'.format(HALL_ANGLE))


class Motors:
    def __init__(self, io, sensors):
        self.io = io
        self.sensors = sensors
        self.speed = 7.35492206953
        self.turn_speed = 7.35492206953
        self.l = 0
        self.r = 0

    def move(self, l=100, r=100):
        self.io.setMotors(l, -r)
        self.l = l
        self.r = r

    def halt(self):
        log('Halting')
        if self.l > 0 and self.r > 0:
            self.move(-BRAKING_SPEED, -BRAKING_SPEED / BRAKING_MULTIPLIER)
            time.sleep(BRAKING_TIME_TRANSLATION)
        if self.l < 0 and self.r < 0:
            self.move(BRAKING_SPEED / BRAKING_MULTIPLIER, BRAKING_SPEED)
            time.sleep(BRAKING_TIME_TRANSLATION)
        if self.l < 0 < self.r:
            self.move(BRAKING_SPEED, -BRAKING_SPEED)
            time.sleep(BRAKING_TIME_ROTATION)
        if self.l > 0 > self.r:
            self.move(-BRAKING_SPEED, BRAKING_SPEED)
            time.sleep(BRAKING_TIME_ROTATION)
        self.move(0, 0)

    def go_forward_revs(self, revs, halt=True):
        log('Going by revs: {}'.format(revs))
        previous_hall = self.sensors.get_hall_sensor()
        changes = 0
        if revs < 0:
            self.move(-100, -100)
        else:
            self.move()
        start = time.time()

        # TODO may need something better here
        while True:
            temp_hall = self.sensors.get_hall_sensor()
            if temp_hall is not previous_hall:
                previous_hall = temp_hall
                changes += 1
            if changes is HALL_SENSOR_CHANGES_FOR_REVOLUTION * int(revs):
                time_spent = time.time() - start
                if halt:
                    self.halt()
                self.speed = float(revs) * float(HALL_PERIMETER) / float(time_spent)
                log('Updated self.speed: {}'.format(self.speed))
                return

    def go_backward_revs(self, revs):
        self.go_forward_revs(-revs)

    def turn_by_revs(self, revs, halt=True):
        log('Turning by revs: {}'.format(revs))

        previous_hall = self.sensors.get_hall_sensor()
        changes = 0

        direction = -1 if revs < 0 else 1
        revs = np.abs(revs)
        self.move(direction * 100, -(direction * 100))
        start = time.time()

        # TODO may need something better here
        while True:
            temp_hall = self.sensors.get_hall_sensor()
            if temp_hall is not previous_hall:
                previous_hall = temp_hall
                changes += 1
            if changes is HALL_SENSOR_CHANGES_FOR_REVOLUTION * int(revs):
                time_spent = time.time() - start
                if halt:
                    self.halt()
                self.turn_speed = float(revs) * float(HALL_ANGLE) / float(time_spent)
                log('Updated self.turn_speed: {}'.format(self.turn_speed))
                return

    # TODO maybe something wrong with negative angle
    def turn_by(self, value, radians=True, full=False):
        """
        Robot turns in place by an angle specified in value. Positive is clockwise.
        :param value:
        :param radians:
        :return: if true, value interpreted as radians otherwise degrees (default)
        """

        value = float(value)

        if full:
            log('Turning by angle {} but performing full rotation for better precision'.format(value))
            value += 2. * pi
        else:
            log('Turning by angle: {}'.format(value))

        revs = int(value / HALL_ANGLE)
        direction = -1 if value < 0 else 1
        value = abs(value)
        value %= HALL_ANGLE
        self.turn_by_revs(revs, halt=False)
        t = value / self.turn_speed
        time.sleep(t)
        self.halt()

    def go_forward(self, distance):
        """
        Robot moves forward specified distance then stops for .5s
        :param distance: distance in cm
        """
        distance = float(distance)

        log('Going forward by distance: {}'.format(distance))

        if distance < 0:
            self.go_backward(-distance)
            return

        revs = int(distance / HALL_PERIMETER)
        distance %= HALL_PERIMETER
        self.go_forward_revs(revs, halt=False)
        t = distance / self.speed
        time.sleep(t)
        self.halt()
        log('Going forward by remaining distance {} in time {}'.format(distance, t))

    # TODO nonhalting backwards
    def go_backward(self, distance):
        """
        Robot moves forward specified distance then stops for .5s
        :param distance: distance in cm
        """
        distance = float(distance)

        if distance < 0:
            self.go_forward(-distance)
            return

        revs = int(distance / HALL_PERIMETER)
        self.go_backward_revs(revs)

        distance %= HALL_PERIMETER
        t = distance / self.speed
        self.move(-100, -100)
        time.sleep(t)
        self.halt()
