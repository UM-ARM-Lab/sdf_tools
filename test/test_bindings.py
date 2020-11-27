#! /usr/bin/env python

import unittest

import numpy as np

from sdf_tools import utils_2d


class TestSDFTools(unittest.TestCase):

    def test_sdf_tools(self):
        res = 0.05
        x_width = 20
        y_height = 40
        grid_world = np.zeros([y_height, x_width], dtype=np.uint8)
        grid_world[1, 3] = 1
        center_x = 0
        center_y = 0
        sdf_origin = [center_x - x_width / 2, center_y - y_height / 2]

        sdf, sdf_gradient = utils_2d.compute_sdf_and_gradient(grid_world, res, sdf_origin)

        self.assertAlmostEqual(sdf[1, 3], -res)
        self.assertAlmostEqual(sdf[2, 3], res)
        self.assertAlmostEqual(sdf[0, 3], res)
        self.assertAlmostEqual(sdf[1, 2], res)
        self.assertAlmostEqual(sdf[1, 4], res)
        self.assertGreater(sdf[3, 6], 3 * res)
        self.assertEqual(sdf.shape, (y_height, x_width))

        self.assertEqual(sdf_gradient.shape, (y_height, x_width, 2))
        np.testing.assert_allclose(sdf_gradient[1, 4], [1.5, 0])


if __name__ == '__main__':
    unittest.main()
