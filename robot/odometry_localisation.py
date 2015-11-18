import numpy as np

class OdometryLocalisation:
    def __init__(self, x, y, angle):
        self.initial_x = x
        self.initial_y = y
        self.initial_angle = angle
        self.x = x
        self.y = y
        self.angle = angle

    def add_forward(self, distance):
        radian_angle = self.angle * np.pi / 180.0
        if self.angle > 270:
            self.x += distance * np.cos(radian_angle)
            self.y -= distance * np.sin(radian_angle)
        elif self.angle > 180:
            self.x -= distance * np.cos(radian_angle)
            self.y -= distance * np.sin(radian_angle)
        elif self.angle > 90:
            self.x -= distance * np.cos(radian_angle)
            self.y += distance * np.sin(radian_angle)
        else:
            self.x += distance * np.cos(radian_angle)
            self.y += distance * np.sin(radian_angle)

    def add_angle(self, angle):
        self.angle += angle
        while self.angle < 0:
            self.angle += 360
        while self.angle > 360:
            self.angle -= 360

    def get_coordinates(self):
        return self.x, self.y, self.angle
