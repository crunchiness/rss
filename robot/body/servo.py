import time


class Servo:
    def __init__(self, io):
        self.io = io

    def close_gate(self):
        self.io.servoEngage()
        self.io.servoSet(0)
        time.sleep(2)  # seems like we need this for servo?
        self.io.servoDisengage()

    def open_gate(self):
        self.io.servoEngage()
        self.io.servoSet(180)
        time.sleep(2)
        self.io.servoDisengage()
