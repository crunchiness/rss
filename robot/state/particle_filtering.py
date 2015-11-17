from __future__ import division

import random
import math
from math import pi
import os.path

import numpy as np

from robot.state.map import X_MAX, Y_MAX, ARENA_WALLS
import utils
from robot.utils import make_file_path, log

ROTATION_STD_ABS = (15.0 / 360.0) * 2.0 * math.pi
DRIFT_ROTATION_STD_ABS = (10.0 / 360.0) * 2.0 * math.pi
FORWARD_STD_FRAC = 0.25

BUFFER_ZONE_FROM_WALLS = 22

SIZE_OF_BINS = 4
NUMBER_OF_ANGLES = 256

# edge r to r+s, tuples in the (r, s) format, not (r, r+s)
ROBOT_EDGES = [
    ([-13.0, -13.0], [  0.0,  37.0]),
    ([-13.0,  24.0], [ 26.0,   0.0]),
    ([ 13.0,  24.0], [  0.0, -37.0]),
    ([ 13.0, -13.0], [-26.0,   0.0])
]

SENSORS_LOCATIONS = {
    'IR_left':  {'location': [-8.0, 23.0], 'orientation': 33.0/180.0*pi},
    'IR_right': {'location': [ 8.0, 23.0], 'orientation': (360.0-33.0)/180.0*pi},
    'sonar': {'location': [3.5, -13.0], 'orientation': pi},
}

MAX_BEAM_RANGE = math.sqrt(2.0 * ((5 * 106.5) ** 2))

X_BASE_OFFSET = 22
Y_BASE_OFFSET = 13
X_BASE_LENGTH = 38
Y_BASE_LENGTH = 44

UNEXPECTED_DECAY_CONST_IR = 0.017801905399325038
UNEXPECTED_DECAY_CONST_SONAR = 0.01813218480166167
PROB_HIT_STD_IR = 10.0
PROB_HIT_STD_SONAR = 25.0

KLD_Z_DELTA = 2.23 # for delta = 0.1
KLD_EPSILON = 0.05
KLD_BIN_SIZE = 8
KLD_NUMBER_OF_ANGLES = 8
KLD_MAX_PARTICLES = 10000

ESS_THRESHOLD = 0.9

DISTANCE_TO_CLOSEST_WALL_FILE = make_file_path('robot/data/') + 'closest_distances.npy'
if os.path.exists(DISTANCE_TO_CLOSEST_WALL_FILE):
    DISTANCE_TO_CLOSEST_WALL = np.load(DISTANCE_TO_CLOSEST_WALL_FILE).astype(np.uint8)
else:
    log('Couldnt find DISTANCE_TO_CLOSEST_WALL_FILE: {}'.format(str(DISTANCE_TO_CLOSEST_WALL_FILE)))

RAYCASTING_DISTANCES = None
RAYCASTING_DISTANCES_FILE = make_file_path('robot/data/') + 'raycasting_distances_SIZE_BIN_4.npy'
if os.path.exists(RAYCASTING_DISTANCES_FILE):
    RAYCASTING_DISTANCES = np.load(RAYCASTING_DISTANCES_FILE).astype(np.uint8)
else:
    log('Couldnt find RAYCASTING_DISTANCES_FILE: {}'.format(str(RAYCASTING_DISTANCES_FILE)))

class Particles:
    def __init__(self, n=500, where=None, drawing=None):
        """
        Creates particle set with given initial position
        :param n: number of particles
        :return:
        """
        self.N = n
        self.drawing = drawing

        log('Initiating particle filtering with setting where=' + str(where))

        self.initialize_KLD(where)

        self.weights = np.ones(self.N, dtype=np.float32)
        self.weights = np.divide(self.weights, np.sum(self.weights))

    def initialize_KLD(self, where):
        H = set()
        M = 0
        k = 0

        Mx = 1000

        new_locations = list()
        new_orientations = list()

        while M < Mx and M < KLD_MAX_PARTICLES:
            if where == 'bases':
                if np.random.random() > 0.5:
                    x = -int(np.random.random() * X_BASE_LENGTH) + int(X_MAX - X_BASE_OFFSET)
                    y = -int(np.random.random() * Y_BASE_LENGTH) + int(Y_MAX - Y_BASE_OFFSET)
                else:
                    x =  int(np.random.random() * X_BASE_LENGTH) + int(X_BASE_OFFSET)
                    y =  int(np.random.random() * Y_BASE_LENGTH) + int(Y_BASE_OFFSET)
            elif where == '1base':
                x =  int(np.random.random() * X_BASE_LENGTH) + int(X_BASE_OFFSET)
                y =  int(np.random.random() * Y_BASE_LENGTH) + int(Y_BASE_OFFSET)
            else:
                x = int(np.random.random() * X_MAX)
                y = int(np.random.random() * Y_MAX)

            orientation = np.random.random() * 2.0 * pi
            increment = 2.0*pi/KLD_NUMBER_OF_ANGLES
            binned_orientation = int(((orientation + 0.5*increment) % (2.0 * pi)) / increment)

            new_locations.append(np.array([x, y]))
            new_orientations.append(orientation)

            x = int(float(x)/float(KLD_BIN_SIZE))
            y = int(float(y)/float(KLD_BIN_SIZE))

            particle = tuple([x, y, binned_orientation])

            if particle not in H:
                k += 1
                H.add(particle)
                if k > 1:
                    k = float(k)
                    Mx = (k-1.0)/(2.0*KLD_EPSILON) * np.power(1.0 - 2.0/9.0/(k-1.0) + np.sqrt(2.0/9.0/(k-1.0))*KLD_Z_DELTA, 3)
            M += 1
            #if M%100==0: log('Mx: {}\nk: {}'.format(Mx, k))

        self.N = len(new_locations)
        log('CHANGED NUMBER OF PARTICLES: {}'.format(self.N))
        self.locations = np.array(new_locations).astype(np.int16)
        self.orientations = np.array(new_orientations).astype(np.float32)

    def initiate_standard(self, where):
        if where == 'bases':
            a = (np.array([X_BASE_OFFSET, Y_BASE_OFFSET])
                 + np.multiply(np.array([X_BASE_LENGTH, Y_BASE_LENGTH]), np.random.rand(self.N/2, 2)))\
                .astype(np.int16)
            b = (np.array([X_MAX - X_BASE_OFFSET, Y_MAX - Y_BASE_OFFSET])
                 - np.multiply(np.array([X_BASE_LENGTH, Y_BASE_LENGTH]), np.random.rand(self.N/2, 2)))\
                .astype(np.int16)
            self.locations = np.concatenate([a, b])
        elif where == '1base':
            self.locations = (np.array([X_BASE_OFFSET, Y_BASE_OFFSET])
                 + np.multiply(np.array([X_BASE_LENGTH, Y_BASE_LENGTH]), np.random.rand(self.N, 2)))\
                .astype(np.int16)
        else:
            self.locations = np.multiply(np.array([X_MAX, Y_MAX],dtype=np.float32), np.random.rand(self.N, 2)).astype(np.int16)

        self.orientations = np.multiply(np.array([2.0 * pi]), np.random.rand(self.N)).astype(np.float32)

    def get_position_by_average(self):
        """
        :return: average of particle positions
        """

        location_approx = np.average(self.locations)
        o_approx = np.average(self.orientations)

        if self.drawing:
            for r in self.locations:
                self.drawing.add_point(r[0], r[1])
            self.drawing.add_big_point(location_approx[0], location_approx[1])
            self.drawing.save()

        log('Average pose: {}, {}, {}'.format(location_approx[0], location_approx[1], o_approx))

        return location_approx[0], location_approx[1], o_approx, self.get_position_conf()

    def get_position_conf(self):
        x_norm = 0
        y_norm = 0
        for i, location in enumerate(self.locations):
            x_norm += (location[0] - .5 * X_MAX) * self.weights[i]
            y_norm += (location[1] - .5 * Y_MAX) * self.weights[i]
        x_norm /= X_MAX
        y_norm /= Y_MAX
        return .5 * (np.var(x_norm) + np.var(y_norm))

    def get_position_by_max_weight(self, position_confidence=True):
        """
        :return: highest weighted particle position
        """
        i = np.argmax(self.weights)

        x_approx, y_approx = self.locations[i]
        o_approx = self.orientations[i]
        if self.drawing:
            for r in self.locations:
                self.drawing.add_point(r[0], r[1])
            self.drawing.add_big_point(x_approx, y_approx)
            self.drawing.save()
        
        log('Max weight pose: {}, {}, {}'.format(x_approx, y_approx, o_approx))
            
        if position_confidence:
            return x_approx, y_approx, o_approx, self.get_position_conf()
        else:
            return x_approx, y_approx, o_approx

    def get_position_by_weighted_average(self):
        """
        :return: weighted average particle position
        """
        x_approx, y_approx = np.sum(
            np.multiply(
                self.weights,
                self.locations.T
            ),
            axis = 1
        )

        o_approx = np.sum(
            np.multiply(
                self.weights,
                self.orientations
            )
        )
        if self.drawing:
            for r in self.locations:
                self.drawing.add_point(r[0], r[1])
            self.drawing.add_big_point(x_approx, y_approx)
            self.drawing.save()
        
        log('Weighted average pose: {}, {}, {}'.format(x_approx, y_approx, o_approx))
        
        return x_approx, y_approx, o_approx

    def rotate(self, rotation):
        """Rotates all the particles"""
        self.orientations = np.mod(
            np.add(
                self.orientations,
                np.add(
                    np.multiply(
                        np.random.normal(size=self.N),
                        ROTATION_STD_ABS
                    ),
                    -0.5 * ROTATION_STD_ABS + rotation
                )
            ),
            2.0 * pi)\
            .astype(np.float32)

    def forward(self, distance):
        """Moves the particles forward"""
        # forward_inferred = random.gauss(distance, FORWARD_STD_FRAC * distance)
        # taking into account possible unintended rotation
        # TODO drift
        # rotation_inferred = random.gauss(0, ROTATION_STD_ABS)
        # orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)

        orientations = np.add(
            np.multiply(
                np.random.normal(size=self.N),
                DRIFT_ROTATION_STD_ABS
            ),
            self.orientations
        )

        vectors = np.concatenate([np.zeros((self.N,1)), np.ones((self.N,1))], axis=1)

        distances =\
        np.add(
            np.multiply(
                np.random.normal(size=self.N),
                FORWARD_STD_FRAC * distance
            ),
            (1 - 0.5 * FORWARD_STD_FRAC) * distance
        )

        for i in xrange(len(vectors)):
            orientation = orientations[i]
            rot_matrix = np.array([
                [ np.cos(orientation), np.sin(orientation)],
                [-np.sin(orientation), np.cos(orientation)]
            ])
            vectors[i] = np.multiply(np.dot(rot_matrix, vectors[i]),distances[i])

        self.locations =\
            np.add(
                self.locations,
                vectors
            ).astype(np.int16)

    def backward(self, distance):
        """Moves the particles backward"""
        self.forward(-distance)

    def sense(self, measurement):
        """Sensing"""
        probabilities = np.zeros(self.N, dtype=np.float32)
        for i in xrange(self.N):
            location = self.location(i)
            if X_MAX <= location[0] or location[0] <= 0 or \
                            Y_MAX <= location[1] or location[1] <= 0:
                probabilities[i] = 0
            else:
                probabilities[i] = self.measurement_probability(measurement, self.measurement_prediction(i))
                if DISTANCE_TO_CLOSEST_WALL[location[0]][location[1]] <= BUFFER_ZONE_FROM_WALLS:
                    probabilities[i] *= 0.01

        self.weights = np.multiply(self.weights, probabilities)
        self.weights = np.divide(self.weights, np.sum(self.weights))

    @staticmethod
    def is_forbidden(location):
        if DISTANCE_TO_CLOSEST_WALL[location[0]][location[1]] < BUFFER_ZONE_FROM_WALLS:
            return True
        return False

    def resample(self):
        ess = 1/np.sum(np.power(self.weights, 2))
        # print ess
        if ess > self.N * ESS_THRESHOLD:
            return

        self.resample_KLD()

    def resample_KLD(self):
        H = set()
        M = 0
        k = 0

        Mx = 1000

        new_locations = list()
        new_orientations = list()

        weights_cumsum = np.cumsum(self.weights)

        while M < Mx and M < KLD_MAX_PARTICLES:
            i = np.searchsorted(weights_cumsum, np.random.rand(1))[0]
            if i == self.N:
                i = self.N - 1
            new_locations.append(self.locations[i])
            new_orientations.append(self.orientations[i])

            x, y = self.locations[i]

            orientation = self.orientations[i]
            increment = 2.0*pi/KLD_NUMBER_OF_ANGLES
            binned_orientation = int(((orientation + 0.5*increment) % (2.0 * pi)) / increment)

            x = int(float(x)/float(KLD_BIN_SIZE))
            y = int(float(y)/float(KLD_BIN_SIZE))

            particle = tuple([x, y, binned_orientation])

            if particle not in H:
                k += 1
                H.add(particle)
                if k > 1:
                    Mx = (k-1)/(2*KLD_EPSILON) * np.power(1 - 2/9/(k-1) + np.sqrt(2/9/(k-1))*KLD_Z_DELTA, 3)
            M += 1

        self.N = len(new_locations)
        log('KLD PARTICLES: {}'.format(self.N))
        self.locations = np.array(new_locations).astype(np.int16)
        self.orientations = np.array(new_orientations).astype(np.float32)

        self.weights = np.ones(self.N).astype(np.float32)
        self.weights = np.divide(self.weights, np.sum(self.weights))

    def resample_standard(self):
        # TODO different resampling
        new_locations = np.zeros((self.N, 2), dtype=np.int16)
        new_orientations = np.zeros(self.N, dtype=np.float32)
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(self.weights)

        random_part = 0.0
        p3index = 0
        for i in range(int((1-random_part)*self.N)):
            beta += random.random() * 2.0 * mw
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % self.N
            new_locations[p3index] = self.locations[index]
            new_orientations[p3index] = self.orientations[index]
            p3index += 1

        self.locations = new_locations
        self.orientations = new_orientations

        self.locations[p3index:self.N] = np.multiply(np.array([X_MAX, Y_MAX],dtype=np.float32), np.random.rand(self.N - p3index, 2))
        self.orientations[p3index:self.N] = np.multiply(np.array([2.0 * pi]), np.random.rand(self.N - p3index)).astype(np.float32)

        self.weights = np.ones(self.N).astype(np.float32)
        self.weights = np.divide(self.weights, np.sum(self.weights))

    def location(self, i):
        """
        Returns a location vector
        :return: location vector [x, y]
        """
        return self.locations[i]

    def orientation(self, i):
        return self.orientations[i]

    def at_orientation(self, i, vectors):
        """
        Rotates the VECTORS by robot's orientation angle (measured from y axis clockwise)
        :param vectors:
        :return: rotated vectors
        """
        return utils.at_orientation(vectors, self.orientation(i))

    def is_collision(self, i):
        """
        Checks if the robot pose collides with an obstacle
        """
        for wall in ARENA_WALLS:
            for edge in ROBOT_EDGES:
                if utils.intersects(wall, self.location(i) + self.at_orientation(i, edge)):
                    return True
        return False

    @staticmethod
    def measurement_prediction_explicit(location, orientation):
        """
        Finds measurement predictions based on particle location.
        :return: measurement predictions
        """
        #TODO reverse
        if RAYCASTING_DISTANCES is not None:
            return Particles.measurement_prediction_from_cache(location, orientation)

        beam_IR_left = utils.at_orientation([0, MAX_BEAM_RANGE],
                                          orientation + SENSORS_LOCATIONS['IR_left']['orientation'])
        beam_IR_right = utils.at_orientation([0, MAX_BEAM_RANGE],
                                          orientation + SENSORS_LOCATIONS['IR_right']['orientation'])
        beam_sonar = utils.at_orientation([0, MAX_BEAM_RANGE],
                                          orientation + SENSORS_LOCATIONS['sonar']['orientation'])
        IR_left = np.add(location,
                       utils.at_orientation(SENSORS_LOCATIONS['IR_left']['location'],
                                            orientation))
        IR_right = np.add(location,
                       utils.at_orientation(SENSORS_LOCATIONS['IR_right']['location'],
                                            orientation))
        sonar = np.add(location,
                       utils.at_orientation(SENSORS_LOCATIONS['sonar']['location'],
                                            orientation))

        # print 'Robot: ' + str(self.location(i)[0]) + ' ' + str(self.location(i)[1]) + ' ' + str(self.orientation(i))
        # print 'Sensors: ' + str(front) + str(right)
        # print 'Beams: ' + str(beam_front) + str(beam_right)

        # find distances along beams to the closest walls
        distances = {}
        for sensor_location, beam, label in (IR_left, beam_IR_left, 'IR_left'), \
                                            (IR_right, beam_IR_right, 'IR_right'),\
                                            (sonar, beam_sonar, 'sonar'):
            minimum_distance = MAX_BEAM_RANGE
            for wall in ARENA_WALLS:
                intersection = utils.intersects_at(wall, (sensor_location, beam))
                if intersection is not None:
                    distance = np.linalg.norm(np.subtract(intersection, sensor_location))
                    if distance < minimum_distance:
                        minimum_distance = distance
            distances[label] = minimum_distance
        # print("{} {} {}".format(location, orientation, distances))
        return distances

    @staticmethod
    def measurement_prediction_from_cache(location, orientation):

        x = int(location[0] / SIZE_OF_BINS)
        y = int(location[1] / SIZE_OF_BINS)

        x = min(x, X_MAX/SIZE_OF_BINS-1)
        y = min(y, Y_MAX/SIZE_OF_BINS-1)

        increment = 2.0*pi/NUMBER_OF_ANGLES
        orientation = int(((orientation + 0.5*increment) % (2.0 * pi)) / increment)

        distances = {}
        temp = 0
        try:
            temp = RAYCASTING_DISTANCES[x][y][orientation]
        except IndexError:
            log('Failed to query RAYCASTING_DISTANCES x={} y={} angle={}'.format(x, y, orientation))

        try:
            distances['IR_left'] = temp[0]
            distances['IR_right'] = temp[1]
            distances['sonar'] = temp[2]
        except TypeError:
            log('Failed to get distances x={} y={} angle={} temp={}'.format(x, y, orientation, temp))

        return distances

    def measurement_prediction(self, i):
        """
        Finds measurement predictions based on particle location.
        :return: measurement predictions
        """

        return Particles.measurement_prediction_explicit(self.locations[i], self.orientation(i))

    def measurement_probability(self, measurements, predictions):
        """
        Finds the measurements probability based on predictions.

        :param measurements: dictionary with 'IR_left' and 'IR_right'
        :param predictions: dictionary with 'IR_left' and 'IR_right'
        :return: probability of measurements
        """

        weights = np.array([1, 1, 1], dtype=np.float32)
        weights = weights/np.sum(weights)
        probabilities = np.array([self.measurement_prob_ir(measurements['IR_left'], predictions['IR_left']),
                                  self.measurement_prob_ir(measurements['IR_right'], predictions['IR_right']),
                                  self.measurement_prob_sonar(measurements['sonar'], predictions['sonar'])],
                                 dtype=np.float32)
        probability = np.dot(weights, probabilities)

        return probability

    @staticmethod
    def measurement_prob_ir(measurement, predicted):
        measurement = float(measurement)
        predicted = float(predicted)

        prob_hit = math.exp(-(measurement - predicted) ** 2 / (PROB_HIT_STD_IR ** 2) / 2.0) \
                       / math.sqrt(2.0 * math.pi * (PROB_HIT_STD_IR ** 2))\

        if measurement < predicted:
            prob_unexpected = UNEXPECTED_DECAY_CONST_IR * math.exp(-UNEXPECTED_DECAY_CONST_IR * measurement) \
                              / (1 - math.exp(-UNEXPECTED_DECAY_CONST_IR * predicted))
        else:
            prob_unexpected = 0

        prob_rand = 1 / MAX_BEAM_RANGE

        prob_max = 0.1 if predicted > 80 else 0

        weights = np.array([3.5132378215960895, 0.3817877358033755, 0.11134520952984188, 0.0], dtype=np.float32)
        weights = weights/np.sum(weights)
        probabilities = np.array([prob_hit, prob_unexpected, prob_rand, prob_max],
                                 dtype=np.float32)
        return np.dot(weights, probabilities)

    @staticmethod
    def measurement_prob_sonar(measurement, predicted):
        measurement = float(measurement)
        predicted = float(predicted)

        prob_hit = math.exp(-(measurement - predicted) ** 2 / (PROB_HIT_STD_SONAR ** 2) / 2.0) \
                       / math.sqrt(2.0 * math.pi * (PROB_HIT_STD_SONAR ** 2))\

        if measurement < predicted:
            prob_unexpected = UNEXPECTED_DECAY_CONST_SONAR * math.exp(-UNEXPECTED_DECAY_CONST_SONAR * measurement) \
                              / (1 - math.exp(-UNEXPECTED_DECAY_CONST_SONAR * predicted))
        else:
            prob_unexpected = 0

        prob_rand = 1 / MAX_BEAM_RANGE

        prob_max = 0.1 if predicted > 80 else 0

        weights = np.array([3.1964663753394977, 0.45801763386224276, 0.1066430442794567, 0.0], dtype=np.float32)
        weights = weights/np.sum(weights)
        probabilities = np.array([prob_hit, prob_unexpected, prob_rand, prob_max],
                                 dtype=np.float32)
        return np.dot(weights, probabilities)

    @staticmethod
    def distance_to_wall(wall, p):
        v = wall[0]
        w = np.add(v, wall[1])
        wminusv = wall[1]
        # Return minimum distance between line segment vw and point p
        temp = np.subtract(v, w)
        l2 = np.dot(temp, temp);  # i.e. |w-v|^2 -  avoid a sqrt
        if l2 == 0.0:
            return np.linalg.norm(np.subtract(p, v))   # v == w case
        # Consider the line extending the segment, parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        t = np.divide(np.dot(np.subtract(p, v), np.subtract(w, v)),l2)
        if t < 0.0:
            return np.linalg.norm(np.subtract(p, v))       # Beyond the 'v' end of the segment
        elif t > 1.0:
            return np.linalg.norm(np.subtract(p, w))  # Beyond the 'w' end of the segment
        projection = np.add(v,np.multiply(t,wminusv))  #Projection falls on the segment
        return np.linalg.norm(np.subtract(p, projection))

    @staticmethod
    # http://stackoverflow.com/a/1501725/3160671
    def distance_to_closest_wall(x, y):
        p = np.array([x, y])

         # find distances to the closest walls
        distances = []
        for wall in ARENA_WALLS:
            distances.append(Particles.distance_to_wall(wall, p))

        return min(distances)

    @staticmethod
    def generate_closest_distances():
        xm = int(X_MAX)
        ym = int(Y_MAX)
        distances = np.zeros((xm, ym)).astype(np.uint8)
        for x in xrange(xm):
            print(x)
            for y in xrange(ym):
                distances[x][y] = Particles.distance_to_closest_wall(x, y)
        return distances

    @staticmethod
    def save_numpy_array(file, array):
        np.save(file, array)

    @staticmethod
    def generate_raycasting_distances(xmin, xmax):
        xm = int(X_MAX / SIZE_OF_BINS)
        ym = int(Y_MAX / SIZE_OF_BINS)

        angle_increment = 2.0 * pi / float(NUMBER_OF_ANGLES)

        distances = np.zeros((xm, ym, NUMBER_OF_ANGLES, 3)).astype(np.uint8)
        for x in range(xmin, xmax):
            print(x)
            for y in xrange(ym):
                # we need offset to calculate the center of the tile
                location = np.array([int(SIZE_OF_BINS * (x + 0.5)), int(SIZE_OF_BINS * (y + 0.5))]).astype(np.int16)
                for angle_number in xrange(NUMBER_OF_ANGLES):
                    temp = Particles.measurement_prediction_explicit(location, angle_number * angle_increment)
                    distances[x][y][angle_number][0] = temp['IR_left']
                    distances[x][y][angle_number][1] = temp['IR_right']
                    distances[x][y][angle_number][2] = temp['sonar']

        return distances

    @staticmethod
    def model_beam(distance, measurements):
        std = 5.0
        if distance <= 10:
            return np.random.uniform(0.0, 40.0, measurements)
        if distance >= 90:
            return np.multiply(np.ones(measurements), 90)
        else:
            return np.add(np.multiply(np.random.normal(size=measurements), std), -0.5*std + distance)

    @staticmethod
    def learn_intrinsic_parameters(measurements, predictions):
        prob_hit_std = 10.0
        unexpected_decay_const = 0.5
        z = np.ones(4)

        # potentially to be changed to while
        for l in xrange(10):
            e = np.empty((len(measurements), 4))

            for i in xrange(len(measurements)):
                probabilities = Particles.learn_intrinsic_parameters_helper(predictions[i], measurements[i],
                                                                            prob_hit_std, unexpected_decay_const)
                e[i] = np.divide(probabilities, np.sum(probabilities))
            z = np.divide(np.sum(e.T, axis=1), np.linalg.norm(measurements))

            prob_hit_std = \
            np.sqrt(
                np.divide(
                    np.sum(
                        np.multiply(
                            np.power(
                                np.subtract(measurements, predictions),
                                2
                            ),
                            e.T[0]
                        )
                    ),
                    np.sum(e.T[0])
                )
            )
            unexpected_decay_const = \
            np.divide(
                np.sum(e.T[1]),
                np.sum(
                    np.multiply(
                        e.T[1],
                        measurements
                    )
                )
            )
            print(z)

        output = z.tolist()
        output.append(prob_hit_std)
        output.append(unexpected_decay_const)
        return output

    @staticmethod
    def learn_intrinsic_parameters_helper(predicted, measurement, prob_hit_std, unexpected_decay_const):

        prob_hit = math.exp(-(measurement - predicted) ** 2 / (prob_hit_std ** 2) / 2.0) \
                       / math.sqrt(2.0 * math.pi * (prob_hit_std ** 2))

        if measurement < predicted:
            prob_unexpected = unexpected_decay_const * math.exp(-unexpected_decay_const * measurement) \
                              / (1 - math.exp(-unexpected_decay_const * predicted))
        else:
            prob_unexpected = 0

        prob_rand = 1.0 / MAX_BEAM_RANGE

        if 89 <= measurement <= 91:
            prob_max = 1
        else:
            prob_max = 0

        return np.array([prob_hit, prob_unexpected, prob_rand, prob_max],
                                 dtype=np.float32)

class Robot:
    """Only for keeping track of our real robot"""
    def __init__(self, x=0., y=0., orientation=0.):
        self.location = np.array([x,y]).astype(np.int16)
        self.orientation = orientation

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * math.pi)

    def at_orientation(self, vectors):
        """
        Rotates the VECTORS by robot's orientation angle (measured from y axis clockwise)
        """
        return utils.at_orientation(vectors, self.orientation)

    def is_collision(self):
        """
        Checks if the robot pose collides with an obstacle
        """
        for wall in ARENA_WALLS:
            for edge in ROBOT_EDGES:
                if utils.intersects(wall, self.location + self.at_orientation(edge)):
                    return True
        return False

    def rotate(self, rotation):
        """
        Infers true pose after rotation (draws from gaussian)
        """
        rotation_inferred = random.gauss(rotation, ROTATION_STD_ABS)
        self.orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)

    def forward(self, distance):
        """
        Infers true coordinates and pose after forwards/backwards movement (draws from gaussian)
        """
        forward_inferred = random.gauss(distance, FORWARD_STD_FRAC * distance)
        # taking into account possible unintended rotation
        rotation_inferred = random.gauss(0, DRIFT_ROTATION_STD_ABS)
        orientation = (self.orientation + rotation_inferred) % (2.0 * math.pi)

        self.location = np.add(self.location,
                               utils.at_orientation([0, 1], orientation) * forward_inferred)

    def measurement_prediction(self):
        return Particles.measurement_prediction_explicit(self.location, self.orientation)

    def measurement_probability(self, measurements, predictions):
        return Particles.measurement_probability(measurements, predictions)

    def measurement_prob_ir(measurement, predicted):
        return Particles.measurement_prob_ir(measurement, predicted)

    def __repr__(self):
        return '[x=%d y=%d orient=%.5f]' % (self.location[0], self.location[1], self.orientation)