import numpy as np

from sdf_tools import pysdf_tools


def compute_sdf_and_gradient(grid_world, sdf_resolution, sdf_origin, frame='world'):
    """
    :param grid_world: a 2d numpy array of the world full of 0 and 1
    :param sdf_resolution: float in meters
    :param sdf_origin: size 2 np.array (vector) in meters
    :param frame: unused
    :return: a tuple (sdf, sdf_gradient) as numpy arrays
    """
    y_height = grid_world.shape[0] * sdf_resolution
    x_width = grid_world.shape[1] * sdf_resolution
    origin_transform = pysdf_tools.Isometry3d([
        [1.0, 0.0, 0.0, sdf_origin[0]],
        [0.0, 1.0, 0.0, sdf_origin[1]],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    oob_value = pysdf_tools.COLLISION_CELL(-10000)
    occupied_value = pysdf_tools.COLLISION_CELL(1)

    # 2d means this should be one cell in z
    z = sdf_resolution

    grid = pysdf_tools.CollisionMapGrid(origin_transform, frame, sdf_resolution, x_width, y_height, z, oob_value)
    for x_index in range(grid.GetNumXCells()):
        for y_index in range(grid.GetNumYCells()):
            occupied = (grid_world[y_index, x_index] == 1)
            if occupied:
                grid.SetValue(x_index, y_index, 0, occupied_value)
    sdf_result = grid.ExtractSignedDistanceField(oob_value.occupancy, False, False)
    sdf_voxelgrid = sdf_result[0]
    np_sdf = np.array(sdf_voxelgrid.GetRawData())
    # NOTE: when the data is flattened in C++, it is done so in column-major order. In 3d it's z,y,x order meaning that
    #  in memory [x=0,y=0,z=0] is followed by [0,0,1] and [0,0,2].
    #  However, the numpy reshape method assumes row major. Therefore we must transpose.
    np_sdf = np_sdf.reshape(sdf_voxelgrid.GetNumXCells(), sdf_voxelgrid.GetNumYCells())
    np_sdf = np.transpose(np_sdf, [1, 0]).astype(np.float32)

    def gradient_function(x_index, y_index, z_index, enable_edge_gradients=False):
        return sdf_voxelgrid.GetGradient(x_index, y_index, z_index, enable_edge_gradients)

    gradient_voxelgrid = sdf_voxelgrid.GetFullGradient(gradient_function, True)
    gradient = gradient_voxelgrid.GetRawData()
    np_gradient = np.array(gradient)
    d = np_gradient.shape[1]
    np_gradient = np_gradient.reshape([gradient_voxelgrid.GetNumXCells(), gradient_voxelgrid.GetNumYCells(), d])
    # remove the z gradient
    np_gradient = np_gradient[:, :, 0:2]
    #
    np_gradient = np.transpose(np_gradient, [1, 0, 2]).astype(np.float32)
    np_gradient = np_gradient.astype(np.float32)

    return np_sdf, np_gradient


def compute_gradient(sdf):
    np_sdf = np.array(sdf.GetRawData())
    np_sdf = np_sdf.reshape(sdf.GetNumXCells(), sdf.GetNumYCells())

    def gradient_function(x_index, y_index, z_index, enable_edge_gradients=False):
        return sdf.GetGradient(x_index, y_index, z_index, enable_edge_gradients)

    gradient_voxelgrid = sdf.GetFullGradient(gradient_function, True)
    gradient = gradient_voxelgrid.GetRawData()
    np_gradient = np.array(gradient)
    d = np_gradient.shape[1]
    np_gradient = np_gradient.reshape([gradient_voxelgrid.GetNumXCells(), gradient_voxelgrid.GetNumYCells(), d])
    # remove the z gradient
    np_gradient = np_gradient[:, :, 0:2]

    return np_sdf, np_gradient


def to_np(sdf, gradient):
    return sdf_to_np(sdf), gradient_to_np(gradient)


def gradient_to_np(gradient):
    np_gradient = np.array(gradient.GetRawData())
    np_gradient = np_gradient.reshape(gradient.GetNumXCells(), gradient.GetNumYCells())

    return np_gradient


def sdf_to_np(sdf):
    np_sdf = np.array(sdf.GetRawData())
    np_sdf = np_sdf.reshape(sdf.GetNumXCells(), sdf.GetNumYCells())

    return np_sdf


collision_cell_type = np.dtype([('component', np.float32), ('occupancy', np.uint32)])


def grid_to_np(grid: pysdf_tools.CollisionMapGrid):
    np_grid = np.ndarray([grid.GetNumXCells(), grid.GetNumYCells()], dtype=np.float32)
    for i in range(grid.GetNumXCells()):
        for j in range(grid.GetNumYCells()):
            collision_cell, _ = grid.GetValueByIndex(i, j, 0)
            np_grid[i, j] = collision_cell.occupancy

    return np_grid
