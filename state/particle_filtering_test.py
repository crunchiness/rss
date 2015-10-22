import unittest
from particle_filtering import *
import numpy as np

class MyTest(unittest.TestCase):
    def test_intersection_value(self):
        a = ([-1, -1], [2, 0])
        b = ([2, 2], [-1, -3])
        np.testing.assert_array_equal(intersects_at(a, b), [1, -1])

    def test_intersection_false(self):
        a = ([-1, -1], [2, 0])
        b = ([20, 20], [-1, -3])
        self.assertFalse(intersects_at(a, b))

    def test_robot(self):
        rob = Robot()
        self.assertTrue(rob.check_collision())