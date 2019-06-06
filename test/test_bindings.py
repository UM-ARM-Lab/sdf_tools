import numpy as np
import sdf_tools

oob_value = sdf_tools.COLLISION_CELL(-10000)
occupied_value = sdf_tools.COLLISION_CELL(1)
resolution = 0.01
x_width = 10
y_height = 10
center_x = 0
center_y = 0
origin_transform = np.array([center_x - x_width / 2, center_y - y_height / 2, 0])
grid = sdf_tools.CollisionMapGrid(origin_transform, "/gazebo_world", resolution, x_width, y_height, resolution, oob_value)
grid.SetValue(2, 2, 0, occupied_value)
sdf, extrema = grid.ExtractSignedDistanceField(oob_value.occupancy, False, False)
