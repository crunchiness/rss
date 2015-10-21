import numpy as np
import random
from math import pi, sqrt, sin, cos, tan
from scipy.stats import norm

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

# edge r to r+s, tuples in the (r, s) format, not (r, r+s)
arena_walls = [
    ([0.0, 0.0], [0, 106.5 * 5]),
    ([0.0, 0.0], [106.5 * 3, 0]),
    ([0, 106.5 * 5], [106.5 * 3, 0]),
    ([106.5 * 3, 0], [0, 106.5 * 5])
]


# adapted from http://pastebin.com/Jfyyyhxk
class Particles:
    # init: creates particle set with given initial position
    def __init__(self, x, y, theta,
                 steering_noise, distance_noise, measurement_noise, N=100):
        self.N = N
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        self.measurement_noise = measurement_noise

        self.data = []
        for i in range(self.N):
            r = Robot()
            r.set(x, y, theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)

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

    # --------
    #
    # motion of the particles
    #

    def move(self, grid, steer, speed):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move(grid, steer, speed)
            newdata.append(r)
        self.data = newdata

    # --------
    #
    # sensing and resampling
    #

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
    # --------
    # init:
    # creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length=0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.measurement_noise = 0.0
        self.num_collisions = 0
        self.num_steps = 0

        # edge r to r+s, tuples in the (r, s) format, not (r, r+s)
        self.edges = [
            ([-11.0, -13.0], [0.0, 26.0]),
            ([-11.0, 13.0], [32.0, 0.0]),
            ([21.0, 13.0], [0.0, -26.0]),
            ([21.0, -13.0], [-32.0, 0.0])
        ]

        self.IR_sensors_locations = [[7.5, 21.0], [-7.5, 21.0]]

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    def at_orientation(self, vectors):

        rot_matrix = np.array([
            [np.cos(self.orientation), -np.sin(self.orientation)],
            [np.sin(self.orientation), np.cos(self.orientation)]
        ])
        if type(vectors) is tuple:
            return [np.multiply(rot_matrix, vectors[0]), np.multiply(rot_matrix, vectors[1])]
        else:
            return np.multiply(rot_matrix, input)

    # --------
    # check:
    #    checks of the robot pose collides with an obstacle, or
    # is too far outside the plane
    def check_collision(self):
        for wall in arena_walls:
            for edge in self.edges:
                if intersects(wall, self.at_orientation(edge)):
                    self.num_collisions += 1
                    return False

        return True

    # --------
    # move:
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, grid, steering, distance,
             tolerance=0.001, max_steering_angle=pi / 4.0):
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # make a new copy
        res = Robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions = self.num_collisions
        res.num_steps = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # Execute motion
        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:

            # approximate by straight line motion

            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * pi)

        else:

            # approximate bicycle model for motion

            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

        # check for collision
        # res.check_collision(grid)

        return res

    def sense(self):
        beam = self.at_orientation([0, sqrt(2 * ((5 * 106.5) ** 2))])
        location = [self.x, self.y]
        left = np.add(location, self.IR_sensors_locations[0])
        right = np.add(location, self.IR_sensors_locations[1])

        distances = list()
        minimum_distance = np.abs(beam)
        for sensor in left, right:
            for wall in arena_walls:
                intersection = intersects_at(wall, (sensor, beam))
                if intersection is not None:
                    distance = np.abs(np.subtract(intersection, location))
                    if distance < minimum_distance:
                        minimum_distance = np.abs(np.subtract(intersection, location))
            distances.append(minimum_distance)

        return [random.gauss(distances[0], self.measurement_noise),
                random.gauss(distances[1], self.measurement_noise)]

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #

    def measurement_prob(self, measurement):
        beam = self.at_orientation([0, sqrt(2 * ((5 * 106.5) ** 2))])
        location = [self.x, self.y]
        left = np.add(location, self.IR_sensors_locations[0])
        right = np.add(location, self.IR_sensors_locations[1])

        distances = list()
        minimum_distance = np.abs(beam)
        for sensor in left, right:
            for wall in arena_walls:
                intersection = intersects_at(wall, (sensor, beam))
                if intersection is not None:
                    distance = np.abs(np.subtract(intersection, location))
                    if distance < minimum_distance:
                        minimum_distance = np.abs(np.subtract(intersection, location))
            distances.append(minimum_distance)

        prob = 1
        for distance in distances:
            if distance < 10 or distance > 80:
                prob *= 1.0 / np.abs(beam)
            else:
                pass
                # prob *= norm.pdf(error[0], self.measurement_noise)

        error = measurement - [distances[0], distances[1]]
        error = norm.pdf(error[0], self.measurement_noise)
        error *= norm.pdf(error[1], self.measurement_noise)

        return prob

    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]' % (self.x, self.y)
