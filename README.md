Builds 2D signed distance fields from images, 3D signed distance fields from pointclouds, 3D signed distance fields from Octomap, provides a lightweight signed distance field library, message types for signed distance fields, and tools to compress signed distance fields for transport.

A simple example/tutorial is provided, see the [Wiki](https://github.com/UM-ARM-Lab/sdf_tools/wiki).

# Python Bindings

These will build by default, and require `ros-{distro}-pybind11-catkin` to be installed. You can disabled them with `catkin config --append --cmake-args -DSDF_TOOLS_PYTHON_BINDINGS=OFF`.
To test that the python bindings are working, you can run `./test/test_bindings.py`
