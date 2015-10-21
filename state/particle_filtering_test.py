import unittest
import particle_filterin
import numpy as np

class MyTest(unittest.TestCase):
    def test(self):
        a = ([-1, -1], [2, 0])
        b = ([2, 2], [-1, -3])
        np.testing.assert_array_equal(particle_filterin.intersects_at(a, b), [1, -1])