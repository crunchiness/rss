import unittest
from particle_filtering import *
import utils
import numpy as np
from math import pi, sqrt

class MyTest(unittest.TestCase):
    def test_intersection_value(self):
        a = ([-1, -1], [2, 0])
        b = ([2, 2], [-1, -3])
        np.testing.assert_array_equal(utils.intersects_at(a, b), [1, -1])
        #TODO may need some more here

    def test_intersection_false(self):
        a = ([-1, -1], [2, 0])
        b = ([20, 20], [-1, -3])
        self.assertFalse(utils.intersects_at(a, b))

    def test_at_orientation(self):
        cases = list()
        cases.append(([1, 0], pi/2.0, [0, -1]))
        cases.append(([1, 1], pi/2.0, [1, -1]))
        cases.append(([1, 0], pi/4.0, [0.5*sqrt(2), -0.5*sqrt(2)]))
        cases.append(([0, 1], pi/3.0, [0.5*sqrt(3),  0.5*sqrt(1)]))
        cases.append(([0, 1], pi/6.0, [0.5*sqrt(1),  0.5*sqrt(3)]))

        for case in cases:
            np.testing.assert_almost_equal(utils.at_orientation(case[0], case[1]), case[2])

    def test_robot_collision(self):
        cases = list()
        cases.append([( 0,  0,  0),   True])
        cases.append([(25, 25,  0),   False])
        cases.append([(20, 20,  0),   False])
        cases.append([(20, 20,  pi), True])
        cases.append([(20, 20, -pi), True])

        particles = Particles(n=1)
        for case in cases:
            particles.locations[0] = np.array([case[0][0], case[0][1]])
            particles.orientations[0] = case[0][2]
            self.assertEqual(particles.is_collision(0), case[1])

    def test_robot_distance_predictions(self):
        particles = Particles(n=1)

        particles.locations[0] = np.array([25, 25])
        particles.orientations[0] = 0.0
        expected = { 'IR_front': 132.0-25.0-21.0, 'IR_right': 143.0-25.0-7.5 }

        actual = particles.measurement_prediction(0)
        for key, value in expected.iteritems():
            np.testing.assert_approx_equal(actual[key], value)

    def test_robot_forward_(self):
        return None
        # TODO
        # robot = Robot()
        #
        # robot.set(25, 25, pi/2.0)
        # robot = robot.forward(20)
        #
        # print robot.location()