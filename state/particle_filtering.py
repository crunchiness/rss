from __future__ import division

import numpy as np
import random
import math
from math import pi
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

            if not r.is_collision():
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
            w.append(self.data[i].measurement_probability(measurement))

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

        self.sensors_locations = {
            'IR_front': { 'location': [0.0, 21.0], 'orientation': 0.0 },
            'IR_right': { 'location': [7.5, 15.0], 'orientation': pi / 2.0 },
            'sonar_front': { 'location': [0.0, 21.0], 'orientation': 0.0 },
        }
        self.max_beam_range = math.sqrt(2.0 * ((5 * 106.5) ** 2))

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * math.pi)

    def location(self):
        """
        Returns a location vector
        :return: location vector [x, y]
        """
        return np.array([self.x, self.y])

    def at_orientation(self, vectors):
        """
        Rotates the VECTORS by robot's orientation angle (measured from y axis clockwise)
        :param vectors:
        :return: rotated vectors
        """
        return utils.at_orientation(vectors, self.orientation)

    def is_collision(self):
        """
        Checks of the robot pose collides with an obstacle
        """
        for wall in ARENA_WALLS:
            for edge in self.edges:
                if utils.intersects(wall, self.location() + self.at_orientation(edge)):
                    return True
        return False

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
        location = utils.at_orientation([0, 1], orientation) * forward_inferred
        return Robot(x=location[0], y=location[1], orientation=orientation)

    def measurement_prediction(self):
        """
        Finds measurement predicitons based on particle location.
        :return: measurement predicitons
        """
        beam_front = utils.at_orientation([0, self.max_beam_range],
                                          self.orientation + self.sensors_locations['IR_front']['orientation'])
        beam_right = utils.at_orientation([0, self.max_beam_range],
                                          self.orientation + self.sensors_locations['IR_right']['orientation'])
        front = np.add(self.location(), self.sensors_locations['IR_front']['location'])
        right = np.add(self.location(), self.sensors_locations['IR_right']['location'])

        # find distances to the closest walls
        distances = {}
        for sensor_location, beam, label in (front, beam_front, 'IR_front'), (right, beam_right, 'IR_right'):
            minimum_distance = self.max_beam_range
            for wall in ARENA_WALLS:
                intersection = utils.intersects_at(wall, (sensor_location, beam))
                if intersection is not None:
                    distance = np.linalg.norm(np.subtract(intersection, sensor_location))
                    if distance < minimum_distance:
                        minimum_distance = distance
            distances[label] = minimum_distance

        return distances

    def measurement_probability(self, measurements, predictions):
        """
        Finds the measurements probability based on predictions.
        :param measurements: dictionary with 'front_ir' and 'right_ir'
        :param predictions: dictionary with 'IR_front' and 'IR_right'
        :return: probability of measurements
        """
        #TODO establish common labels

        probability = 1
        probability *= self.measurement_prob_ir(measurements['front_ir'], predictions['IR_front'])
        probability *= self.measurement_prob_ir(measurements['right_ir'], predictions['IR_right'])

        return probability

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
