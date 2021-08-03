import numpy as np
import pysdf_tools
import tensorflow as tf


def compute_sdf_and_gradient(env, res, origin_point):
    """
    :param env: [w,h,c]
    :param res: float in meters
    :param origin_point: [3], [x,y,z]
    :return: a tuple (sdf, sdf_gradient) as numpy arrays
    """
    if isinstance(res, tf.Tensor):
        res = res.numpy()
    if isinstance(origin_point, tf.Tensor):
        origin_point = origin_point.numpy()
    origin_transform = pysdf_tools.Isometry3d([
        [1.0, 0.0, 0.0, origin_point[0]],
        [0.0, 1.0, 0.0, origin_point[1]],
        [0.0, 0.0, 1.0, origin_point[2]],
        [0.0, 0.0, 0.0, 1.0]
    ])

    oob_value = pysdf_tools.COLLISION_CELL(-10000)
    occupied_value = pysdf_tools.COLLISION_CELL(1)

    x_shape = env.shape[0]
    y_shape = env.shape[1]
    z_shape = env.shape[2]
    grid = pysdf_tools.CollisionMapGrid(origin_transform, 'world', res, x_shape, y_shape, z_shape, oob_value)
    for x_index in range(grid.GetNumXCells()):
        for y_index in range(grid.GetNumYCells()):
            for z_index in range(grid.GetNumZCells()):
                occupied = (env[x_index, y_index, z_index] == 1)
                if occupied:
                    grid.SetValue(x_index, y_index, z_index, occupied_value)
    sdf_result = grid.ExtractSignedDistanceField(oob_value.occupancy, False, False)
    sdf_voxelgrid = sdf_result[0]
    np_sdf = np.array(sdf_voxelgrid.GetRawData())
    # NOTE: when the data is flattened in C++, it is done so in column-major order. In 3d it's z,y,x order meaning that
    #  in memory [x=0,y=0,z=0] is followed by [0,0,1] and [0,0,2].
    #  However, the numpy reshape method assumes row major. Therefore we must transpose.
    np_sdf = np_sdf.reshape([sdf_voxelgrid.GetNumXCells(), sdf_voxelgrid.GetNumYCells(), sdf_voxelgrid.GetNumZCells()])
    np_sdf = np.transpose(np_sdf, [1, 0, 2]).astype(np.float32)

    def gradient_function(x_index, y_index, z_index, enable_edge_gradients=False):
        return sdf_voxelgrid.GetGradient(x_index, y_index, z_index, enable_edge_gradients)

    gradient_voxelgrid = sdf_voxelgrid.GetFullGradient(gradient_function, True)
    gradient = gradient_voxelgrid.GetRawData()
    np_gradient = np.array(gradient)
    d = np_gradient.shape[1]
    grad_shape = [gradient_voxelgrid.GetNumXCells(),
                  gradient_voxelgrid.GetNumYCells(),
                  gradient_voxelgrid.GetNumZCells(),
                  d]
    np_gradient = np_gradient.reshape(grad_shape)
    np_gradient = np.transpose(np_gradient, [1, 0, 2, 3]).astype(np.float32)
    np_gradient = np_gradient.astype(np.float32)

    return np_sdf, np_gradient
