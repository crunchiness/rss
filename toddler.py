#!/usr/bin/env python
__TODDLER_VERSION__ = '1.0.0'

import time


class Toddler:
    def __init__(self, IO):
        print '{{some cool name}} has awoken... Beware.'
        self.IO = IO

    def move(self, l=100, r=100):
        self.IO.setMotors(l, -r)

    def close_gate(self):
        self.IO.servoEngage()
        self.IO.servoSet(0)
        time.sleep(2)  # seems like we need this for servo?
        self.IO.servoDisengage()

    def open_gate(self):
        self.IO.servoEngage()
        self.IO.servoSet(180)
        time.sleep(2)
        self.IO.servoDisengage()

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            self.open_gate()
            time.sleep(5)
            self.close_gate()
            time.sleep(5)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        pass
