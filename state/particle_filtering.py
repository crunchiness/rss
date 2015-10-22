from __future__ import division

import numpy as np
import random
from math import pi, exp, sqrt, sin, cos, tan
from body.sensors import Sensors

sensors = Sensors()

# normal pdf
# http://stackoverflow.com/a/8669381/3160671
def normpdf(x, mu, sigma):
    u = (x-mu)/abs(sigma)
    y = (1/(sqrt(2*pi)*abs(sigma)))*exp(-u*u/2)
    return y

# implemented from http://stackoverflow.com/a/565282
# line segments p to p+r and q to q+s
def intersects_at_helper(p, r, q, s):
    if np.cross(r, s) == 0:
        return None

    qp = np.subtract(q, p)
    rs = np.cross(r, s)
    t = np.cross(qp, s) / rs
    u = np.cross(qp, r) / rs
    return t, u


def intersects_at((p, r), (q, s)):
    t, u = intersects_at_helper(p, r, q, s)

    if 0 <= t <= 1 and 0 <= u <= 1:
        return np.add(p, np.multiply(t, r))
    else:
        return None


def intersects((p, r), (q, s)):
    t, u = intersects_at_helper(p, r, q, s)

    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return True
    else:
        return False

x_max = 106.5 * 3
y_max = 106.5 * 5

# edge r to r+s, tuples in the (r, s) format, not (r, r+s)
arena_walls = [
    ([0.0, 0.0], [0.0, y_max]),
    ([0.0, 0.0], [x_max, 0.0]),
    ([0.0, y_max], [x_max, 0.0]),
    ([x_max, 0.0], [0.0, y_max]),

    ([143.0, 0.0], [0.0, 93.0]),
    ([143.0, 93.0], [32.0, 0.0]),

    ([0.0, 132.0], [74.0, 0.0]),

    ([0.0, 294.0], [93.0, 0.0]),
    ([93.0, 294.0], [0.0, 46.0]),
    ([93.0, 340.0], [34.0, 0.0]),

    ([x_max, 231.0], [-93.0, 0.0]),
    ([x_max-93.0, 231.0], [0.0, -46.0]),
    ([x_max-93.0, 231.0-46.0], [-34.0, 0.0]),

    ([x_max, y_max-132.0], [-74.0, 0.0]),

    ([177.0, y_max], [0.0, -93.0]),
    ([177.0, y_max-93.0], [-32.0, 0.0]),

    ([x_max-70.0, 91.0], [-23.0, 0.0]),
    ([x_max-70.0, 91.0], [0.0, 23.0]),
    ([x_max-93.0, 114.0], [23.0, 0.0]),
    ([x_max-93.0, 114.0], [0.0, -23.0]),

    ([70.0, y_max-91.0], [23.0, 0.0]),
    ([70.0, y_max-91.0], [0.0, -23.0]),
    ([93.0, y_max-114.0], [-23.0, 0.0]),
    ([93.0, y_max-114.0], [0.0, 23.0]),

    ([x_max-143.0, 234.0], [-38.0, 0.0]),
    ([x_max-143.0, 234.0], [0.0, 31.0]),
    ([x_max-143.0-38.0, 234.0+31.0], [38.0, 0.0]),
    ([x_max-143.0-38.0, 234.0+31.0], [0.0, -31.0])
]

# adapted from http://pastebin.com/Jfyyyhxk
class Particles:
    # init: creates particle set with given initial position
    def __init__(self, n=1000):
        self.N = n

        self.data = []
        n_added = 0
        while n_added < n:
            #random coordinates and orientation
            x = random.random() * x_max
            y = random.random() * y_max
            orientation = random.random() * 2.0 * pi

            r = Robot()
            r.set(x, y, orientation)

            if not r.check_collision():
                self.data.append(r)
                n_added += 1

    # extract position from a particle set
    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((self.data[i].orientation
                              - self.data[0].orientation + pi) % (2.0 * pi))
                            + self.data[0].orientation - pi)
        return [x / self.N, y / self.N, orientation / self.N]

    # motion of the particles
    def move(self, rotation, forward):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move(rotation, forward)
            newdata.append(r)
        self.data = newdata

    # sensing and resampling
    def sense(self, Z):
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3

class Robot:
    def __init__(self, length=0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0

        self.rotation_during_forward_std_per_10cm = (1.0/360.0)*2.0*pi
        self.forward_during_rotation_std = 2

        self.rotation_std_abs = (5.0/360.0)*2.0*pi
        self.forward_std_frac = 0.1

        # edge r to r+s, tuples in the (r, s) format, not (r, r+s)
        self.edges = [
            ([-11.0, -13.0], [0.0, 26.0]),
            ([-11.0, 13.0], [32.0, 0.0]),
            ([21.0, 13.0], [0.0, -26.0]),
            ([21.0, -13.0], [-32.0, 0.0])
        ]

        self.IR_sensors_locations = [([0.0, 21.0], 0.0), ([-7.5, 15.0], pi/2.0)]
        self.max_beam_range = sqrt(2.0*((5*106.5)**2))

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)

    def at_orientation(self, vectors, orientation):

        rot_matrix = np.array([
            [np.cos(orientation), -np.sin(orientation)],
            [np.sin(orientation), np.cos(orientation)]
        ])
        if type(vectors) is tuple:
            return [np.multiply(rot_matrix, vectors[0]), np.multiply(rot_matrix, vectors[1])]
        else:
            return np.multiply(rot_matrix, input)

    # check: checks of the robot pose collides with an obstacle
    def check_collision(self):
        for wall in arena_walls:
            for edge in self.edges:
                if intersects(wall, self.at_orientation(edge)):
                    return False
        return True

    def move(self, rotation, forward):
        # make a new copy
        res = Robot()

        location = [self.x, self.y]
        orientation = self.orientation

        #TODO drift

        if rotation is not 0:
            rotation2 = random.gauss(rotation, self.rotation_std_abs)
            orientation = (orientation + rotation2) % (2.0 * pi)

        if forward is not 0:
            forward3 = random.gauss(forward, self.forward_std_frac * forward)
            rotation3 = random.gauss(0, self.rotation_std_abs)
            orientation = (orientation + rotation3) % (2.0 * pi)
            location = self.at_orientation([0, 1], orientation) * forward3

        res.x = location[0]
        res.y = location[1]
        res.orientation = orientation

        # TODO check for collision
        # res.check_collision()

        return res

    def sense(self):
          return [sensors.get_ir_left(), sensors.get_ir_right()]

    def measurement_prob(self, measurement):

        beam_front = self.at_orientation([0, self.max_beam_range], self.orientation + self.IR_sensors_locations[0][1])
        beam_right = self.at_orientation([0, self.max_beam_range], self.orientation + self.IR_sensors_locations[1][1])
        location = [self.x, self.y]
        front = np.add(location, self.IR_sensors_locations[0][0])
        right = np.add(location, self.IR_sensors_locations[1][0])

        distances = list()
        minimum_distance = self.max_beam_range
        for sensor, beam in (front, beam_front), (right, beam_right):
            for wall in arena_walls:
                intersection = intersects_at(wall, (sensor, beam))
                if intersection is not None:
                    distance = np.abs(np.subtract(intersection, location))
                    if distance < minimum_distance:
                        minimum_distance = np.abs(np.subtract(intersection, location))
            distances.append(minimum_distance)

        error = 1
        error *= self.measurement_prob_IR(measurement['IRfront'], distances[0])
        error *= self.measurement_prob_IR(measurement['IRright'], distances[1])

        return error

    def measurement_prob_IR(self, measurement, predicted):
        prob_hit_std = 5.0
        if predicted > 10 and predicted < 80:
            prob_hit = exp(-(measurement - predicted) ** 2 / (prob_hit_std ** 2) / 2.0) \
                       / sqrt(2.0 * pi * (prob_hit_std ** 2))
        else:
            prob_hit = 0

        prob_unexpected_decay_const = 0.5
        if measurement < predicted:
            prob_unexpected = prob_unexpected_decay_const * exp(-prob_unexpected_decay_const * measurement) \
                              / (1 - exp(-prob_unexpected_decay_const * predicted))
        else:
            prob_unexpected = 0

        prob_rand = 1/self.max_beam_range

        if predicted > 80:
            prob_max = 1

        weights = [0.7, 0.1, 0.1, 0.1]
        prob = 0
        prob += weights[0] * prob_hit
        prob += weights[1] * prob_unexpected
        prob += weights[2] * prob_rand
        prob += weights[3] * prob_max

        return prob

    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]' % (self.x, self.y)
