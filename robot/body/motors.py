import time
import numpy as np
# distance unit is cm

DIAMETER = 16  # distance between (the middle of) wheels
WHEEL = 9  # wheel diameter


class Motors:
    def __init__(self, io):
        self.io = io

    def move(self, l=100, r=100):
        self.io.setMotors(l, -r)

    def halt(self):
        self.io.setMotors(0, 0)

    def go_forward(self, distance):
        """
        Robot moves forward specified distance then stops for .5s
        :param distance: distance in cm
        """
        # TODO: correct values; now 1m is ~97cm
        if distance < 0:
            self.go_backward(-distance)
            return
        t = (distance - 0.4 * 10.3125) / 7.2875
        self.move(100, 100)
        time.sleep(t)
        self.move(0, 100)
        time.sleep(0.4)
        self.halt()
        time.sleep(0.5)

    def go_backward(self, distance):
        """
        Robot moves forward specified distance then stops for .5s
        :param distance: distance in cm
        """
        # TODO: correct values; now 1m is ~91cm
        if distance < 0:
            self.go_forward(-distance)
            return
        t = (distance - 0.4 * 10.3125) / 7.2875
        self.move(-100, -100)
        time.sleep(t)
        self.move(-100, 0)
        time.sleep(0.4)
        self.halt()
        time.sleep(0.5)

    def turn_by(self, value, radians=False):
        """
        Robot turns in place by an angle specified in value. Positive is clockwise.
        :param value:
        :param radians:
        :return: if true, value interpreted as radians otherwise degrees (default)
        """
        if radians:
            value = 180. * value / np.pi
        direction = -1 if value < 0 else 1
        self.move(direction * 100, -(direction * 100))
        calc_time = (7.3 * abs(value)) / 360
        time.sleep(calc_time)
        self.halt()
