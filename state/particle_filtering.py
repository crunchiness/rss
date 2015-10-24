from __future__ import division

import numpy as np
import random
import math
from math import pi
from state.map import X_MAX, Y_MAX, ARENA_WALLS
import utils


class Particles:
    """adapted from http://pastebin.com/Jfyyyhxk"""
    def __init__(self, n=5, drawing=None):
        """
        Creates particle set with given initial position
        :param n: number of particles
        :return:
        """
        self.N = n
        self.data = []
        self.drawing = drawing

        n_added = 0
        i = 0
        while n_added < n:
            #print 'Generated particles: {0}; Attempts: {1}'.format(n_added, i)
            i += 1
            # TODO: need a way to confine initial positions to one room
            # random coordinates and orientation
            x = random.random() * 150.0 #(X_MAX-2*11)+11
            y = random.random() * 150.0 #(Y_MAX-2*11)+11
            orientation = random.random() * 2.0 * math.pi

            r = Robot()
            r.set(x, y, orientation)

            #if not r.is_collision():
            self.data.append(r)
            n_added += 1

    def get_position_by_average(self):
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

    def get_position_by_weight(self):
        """
        :return: highest weighted particle position
        """
        highest_weight = 0.0
        highest_weight_particle = None
        for particle in self.data:
            if particle.weight > highest_weight:
                highest_weight = particle.weight
                highest_weight_particle = particle

        return [highest_weight_particle.x, highest_weight_particle.y, highest_weight_particle.orientation]

    def rotate(self, rotation):
        """Rotates all the particles"""
        self.data = map(lambda r: r.rotate(rotation), self.data)

    def forward(self, distance):
        """Moves the particles forward"""
        self.data = map(lambda r: r.forward(distance), self.data)

    def backward(self, distance):
        """Moves the particles backward"""
        self.forward(-distance)

    def sense(self, measurement):
        """Sensing"""
        w = []
        for i in range(self.N):
            predictions = self.data[i].measurement_prediction()
            self.data[i].weight *= self.data[i].measurement_probability(measurement, predictions)

    def resample(self):
        """Resampling"""
        #TODO may need to change
        # for i in xrange(self.N):
        #     if self.data[i].is_collision():
        #         self.data[i].weight *= 0.01

        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        w = [rob.weight for rob in self.data]
        mw = max(w)

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3

rotation_std_abs = (5.0 / 360.0) * 2.0 * math.pi
forward_std_frac = 0.1

# edge r to r+s, tuples in the (r, s) format, not (r, r+s)
robot_edges = [
    ([-11.0, -13.0], [0.0, 26.0]),
    ([-11.0, 13.0], [32.0, 0.0]),
    ([21.0, 13.0], [0.0, -26.0]),
    ([21.0, -13.0], [-32.0, 0.0])
]

sensors_locations = {
    'IR_front': { 'location': [0.0, 21.0], 'orientation': 0.0 },
    'IR_right': { 'location': [7.5, 15.0], 'orientation': pi / 2.0 },
    'sonar_front': { 'location': [0.0, 21.0], 'orientation': 0.0 },
}
max_beam_range = math.sqrt(2.0 * ((5 * 106.5) ** 2))

class Robot:
    def __init__(self, x=0., y=0., orientation=0.):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.weight = 1

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
        Checks if the robot pose collides with an obstacle
        """
        for wall in ARENA_WALLS:
            for edge in robot_edges:
                if utils.intersects(wall, self.location() + self.at_orientation(edge)):
                    return True
        return False

    def rotate(self, rotation):
        """
        Infers true pose after rotation (draws from gaussian)
        :param rotation:
        :return: new particle
        """
        rotation_inferred = random.gauss(rotation, rotation_std_abs)
        orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)
        return Robot(x=self.x, y=self.y, orientation=orientation)

    def forward(self, distance):
        """
        Infers true coordinates and pose after forwards/backwards movement (draws from gaussian)
        :param distance:
        :return: new particle
        """
        forward_inferred = random.gauss(distance, forward_std_frac * distance)
        # taking into account possible unintended rotation
        rotation_inferred = random.gauss(0, rotation_std_abs)
        orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)

        location = utils.at_orientation([0, 1], orientation) * forward_inferred

        x, y = np.add(location, self.location())

        # # Prevent out of arena predictions
        # x = 0 if x < 0 else x
        # x = X_MAX-1 if x >= X_MAX else x
        #
        # y = 0 if y < 0 else y
        # y = Y_MAX-1 if y >= Y_MAX else y

        return Robot(x=x, y=y, orientation=orientation)

    def measurement_prediction(self):
        """
        Finds measurement predictions based on particle location.
        :return: measurement predictions
        """

        beam_front = utils.at_orientation([0, max_beam_range],
                                          self.orientation + sensors_locations['IR_front']['orientation'])
        beam_right = utils.at_orientation([0, max_beam_range],
                                          self.orientation + sensors_locations['IR_right']['orientation'])
        front = np.add(self.location(), sensors_locations['IR_front']['location'])
        right = np.add(self.location(), sensors_locations['IR_right']['location'])

        # find distances to the closest walls
        distances = {}
        for sensor_location, beam, label in (front, beam_front, 'IR_front'), (right, beam_right, 'IR_right'):
            minimum_distance = max_beam_range
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

        :param measurements: dictionary with 'IR_front' and 'IR_right'
        :param predictions: dictionary with 'IR_front' and 'IR_right'
        :return: probability of measurements
        """
        # TODO establish common labels

        weights = [0.5, 0.5]
        probability = 0
        probability += weights[0] * self.measurement_prob_ir(measurements['IR_front'], predictions['IR_front'])
        probability += weights[1] * self.measurement_prob_ir(measurements['IR_right'], predictions['IR_right'])
        return probability

    def measurement_prob_ir(self, measurement, predicted):
        prob_hit_std = 5.0
        #if 10 < predicted < 80:
        prob_hit = math.exp(-(measurement - predicted) ** 2 / (prob_hit_std ** 2) / 2.0) \
                       / math.sqrt(2.0 * math.pi * (prob_hit_std ** 2))
        #else:
        #    prob_hit = 0

        prob_unexpected_decay_const = 0.5
        if measurement < predicted:
            prob_unexpected = prob_unexpected_decay_const * math.exp(-prob_unexpected_decay_const * measurement) \
                              / (1 - math.exp(-prob_unexpected_decay_const * predicted))
        else:
            prob_unexpected = 0

        prob_rand = 1 / max_beam_range

        prob_max = 0.1 if predicted > 80 else 0

        weights = [1, 0.0, 0.0, 0.0]
        prob = 0
        prob += weights[0] * prob_hit
        prob += weights[1] * prob_unexpected
        prob += weights[2] * prob_rand
        prob += weights[3] * prob_max

        return prob

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)
