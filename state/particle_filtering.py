from __future__ import division

import numpy as np
import random
import math
from state.map import X_MAX, Y_MAX, ARENA_WALLS
import utils


class Particles:
    """adapted from http://pastebin.com/Jfyyyhxk"""
    def __init__(self, n=1000, drawing=None):
        """
        Creates particle set with given initial position
        :param n: number of particles
        :return:
        """
        self.N = n
        self.data = []
        self.drawing = drawing

        n_added = 0
        while n_added < n:
            # TODO: need a way to confine initial positions to one room
            # random coordinates and orientation
            x = random.random() * X_MAX
            y = random.random() * Y_MAX
            orientation = random.random() * 2.0 * math.pi

            r = Robot()
            r.set(x, y, orientation)

            if not r.check_collision():
                self.data.append(r)
                n_added += 1

    def get_position(self):
        # TODO: shouldn't this be weighted? only makes sense if stuff converged already
        """
        :return: average of particle positions
        """
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((self.data[i].orientation - self.data[0].orientation + math.pi) % (2.0 * math.pi)) +
                            self.data[0].orientation - math.pi)
        x_approx = x / self.N
        y_approx = y / self.N
        o_approx = orientation / self.N

        if self.drawing:
            for r in self.data:
                self.drawing.add_point(r.x, r.y)
            self.drawing.add_big_point(x_approx, y_approx)
            self.drawing.save()

        return [x_approx, y_approx, o_approx]

    def rotate(self, rotation):
        """Rotates all the particles"""
        self.data = map(lambda r: r.move(rotation), self.data)

    def forward(self, distance):
        """Moves the particles forward"""
        self.data = map(lambda r: r.move(distance), self.data)

    def backward(self, distance):
        """Moves the particles backward"""
        self.forward(-distance)

    def sense(self, measurement):
        """Sensing and resampling"""
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(measurement))

        # resampling (careful, this is using shallow copy) ??
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
    def __init__(self, x=0., y=0., orientation=0.):

        self.x = x
        self.y = y
        self.orientation = orientation

        self.rotation_std_abs = (5.0 / 360.0) * 2.0 * math.pi
        self.forward_std_frac = 0.1

        # edge r to r+s, tuples in the (r, s) format, not (r, r+s)
        self.edges = [
            ([-11.0, -13.0], [0.0, 26.0]),
            ([-11.0, 13.0], [32.0, 0.0]),
            ([21.0, 13.0], [0.0, -26.0]),
            ([21.0, -13.0], [-32.0, 0.0])
        ]

        self.IR_sensors_locations = [([0.0, 21.0], 0.0), ([-7.5, 15.0], math.pi / 2.0)]
        self.max_beam_range = math.sqrt(2.0 * ((5 * 106.5) ** 2))

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * math.pi)

    def at_orientation(self, vectors, orientation):
        # TODO: what is this doing
        rot_matrix = np.array([
            [np.cos(orientation), -np.sin(orientation)],
            [np.sin(orientation), np.cos(orientation)]
        ])
        if type(vectors) is tuple:
            return [rot_matrix.dot(vectors[0]), rot_matrix.dot(vectors[1])]
        else:
            return np.multiply(rot_matrix, input)

    def check_collision(self):
        """Checks of the robot pose collides with an obstacle"""
        for wall in ARENA_WALLS:
            for edge in self.edges:
                if utils.intersects(wall, self.at_orientation(edge, self.orientation)):
                    return False
        return True

    def rotate(self, rotation):
        """
        Infers true pose after rotation (draws from gaussian)
        :param rotation:
        :return: new particle
        """
        rotation_inferred = random.gauss(rotation, self.rotation_std_abs)
        orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)
        return Robot(x=self.x, y=self.y, orientation=orientation)

    def forward(self, distance):
        """
        Infers true coordinates and pose after forwards/backwards movement (draws from gaussian)
        :param distance:
        :return: new particle
        """
        forward_inferred = random.gauss(distance, self.forward_std_frac * distance)
        # taking into account possible unintended rotation
        rotation_inferred = random.gauss(0, self.rotation_std_abs)
        orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)
        location = self.at_orientation([0, 1], orientation) * forward_inferred
        return Robot(x=location[0], y=location[1], orientation=orientation)

    def measurement_prob(self, measurement):
        """
        :param measurement: dictionary with 'front_ir' and 'right_ir'
        :return: error ?
        """
        beam_front = self.at_orientation([0, self.max_beam_range], self.orientation + self.IR_sensors_locations[0][1])
        beam_right = self.at_orientation([0, self.max_beam_range], self.orientation + self.IR_sensors_locations[1][1])
        location = [self.x, self.y]
        front = np.add(location, self.IR_sensors_locations[0][0])
        right = np.add(location, self.IR_sensors_locations[1][0])

        # find distances to the closest walls
        distances = []
        minimum_distance = self.max_beam_range
        for sensor, beam in (front, beam_front), (right, beam_right):
            for wall in ARENA_WALLS:
                intersection = utils.intersects_at(wall, (sensor, beam))
                if intersection is not None:
                    distance = np.abs(np.subtract(intersection, location))
                    if distance < minimum_distance:
                        minimum_distance = np.abs(np.subtract(intersection, location))
            distances.append(minimum_distance)

        error = 1
        error *= self.measurement_prob_ir(measurement['front_ir'], distances[0])
        error *= self.measurement_prob_ir(measurement['right_ir'], distances[1])

        return error

    def measurement_prob_ir(self, measurement, predicted):
        prob_hit_std = 5.0
        if 10 < predicted < 80:
            prob_hit = math.exp(-(measurement - predicted) ** 2 / (prob_hit_std ** 2) / 2.0) \
                       / math.sqrt(2.0 * math.pi * (prob_hit_std ** 2))
        else:
            prob_hit = 0

        prob_unexpected_decay_const = 0.5
        if measurement < predicted:
            prob_unexpected = prob_unexpected_decay_const * math.exp(-prob_unexpected_decay_const * measurement) \
                              / (1 - math.exp(-prob_unexpected_decay_const * predicted))
        else:
            prob_unexpected = 0

        prob_rand = 1 / self.max_beam_range

        prob_max = 1 if predicted > 80 else 0

        weights = [0.7, 0.1, 0.1, 0.1]
        prob = 0
        prob += weights[0] * prob_hit
        prob += weights[1] * prob_unexpected
        prob += weights[2] * prob_rand
        prob += weights[3] * prob_max

        return prob

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)
