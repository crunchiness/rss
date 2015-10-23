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

    def test_robot(self):
        rob = Robot()
        self.assertTrue(rob.is_collision())

