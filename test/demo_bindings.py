#! /usr/bin/env python

import numpy as np
from time import time
import matplotlib.pyplot as plt
from sdf_tools import utils_2d


def main():
    res = 0.01
    x_width = 100
    y_height = 100
    grid_world = np.zeros([y_height, x_width], dtype=np.uint8)
    grid_world[0:10, 0:10] = 1
    grid_world[90:100, 90:100] = 1
    grid_world[20:30, 40:50] = 1
    center_x = 0
    center_y = 0
    sdf_origin = [center_x - x_width / 2, center_y - y_height / 2]
    t0 = time()
    sdf, sdf_gradient = utils_2d.compute_sdf_and_gradient(grid_world, res, sdf_origin)
    dt = time() - t0
    print('time: {}s'.format(dt))

    plt.figure()
    plt.title("SDF")
    plt.imshow(np.flipud(sdf))

    plt.figure()
    plt.title("Gradient")
    xx, yy = np.meshgrid(range(x_width), range(y_height))
    plt.quiver(xx, yy, sdf_gradient[:, :, 0], sdf_gradient[:, :, 1])
    plt.axis("equal")

    plt.show()

if __name__ == '__main__':
    main()
