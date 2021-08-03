"""
Demonstrates creating a 3d SDF
displays it in rviz
"""
import ros_numpy
import rospy
import numpy as np

from geometry_msgs.msg import Point
from rviz_voxelgrid_visuals.conversions import vox_to_voxelgrid_stamped
from rviz_voxelgrid_visuals_msgs.msg import VoxelgridStamped
from sdf_tools.utils_3d import compute_sdf_and_gradient
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker


def create_point_cloud():
    rng = np.random.RandomState(0)
    return rng.uniform([0.0, 0.5, 0], [0.7, 0.6, 0.5], [10000, 3])


def point_cloud_to_voxel_grid(pc: np.ndarray, shape, res, origin_point):
    vg = np.zeros(shape, dtype=np.float32)
    indices = ((pc - origin_point) / res).astype(np.int64)
    for i in indices:
        if 0 <= i[0] < shape[0] and 0 <= i[1] < shape[1] and 0 <= i[2] < shape[2]:
            vg[i[0], i[1], i[2]] = 1.0
    return vg


def visualize_point_cloud(pub: rospy.Publisher, pc: np.ndarray):
    list_of_tuples = [tuple(point) for point in pc]
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
    np_record_array = np.array(list_of_tuples, dtype=dtype)
    msg = ros_numpy.msgify(PointCloud2, np_record_array, frame_id='world', stamp=rospy.Time.now())
    pub.publish(msg)


def visualize_sdf(pub, sdf: np.ndarray, shape, res, origin_point):
    points = get_grid_points(origin_point, res, shape)
    list_of_tuples = [(p[0], p[1], p[2], d) for p, d in zip(points.reshape([-1, 3]), sdf.flatten())]
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('distance', np.float32)]
    np_record_array = np.array(list_of_tuples, dtype=dtype)
    msg = ros_numpy.msgify(PointCloud2, np_record_array, frame_id='world', stamp=rospy.Time.now())
    pub.publish(msg)


def get_grid_points(origin_point, res, shape):
    indices = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), np.arange(shape[2]))
    indices = np.stack(indices, axis=-1)
    points = (indices * res) - origin_point
    return points


def rviz_arrow(position: np.ndarray, target_position: np.ndarray, label: str = 'arrow', **kwargs):
    idx = kwargs.get("idx", 0)

    arrow = Marker()
    arrow.action = Marker.ADD  # create or modify
    arrow.type = Marker.ARROW
    arrow.header.frame_id = "world"
    arrow.header.stamp = rospy.Time.now()
    arrow.ns = label
    arrow.id = idx

    arrow.scale.x = kwargs.get('scale', 1.0) * 0.0025
    arrow.scale.y = kwargs.get('scale', 1.0) * 0.004
    arrow.scale.z = kwargs.get('scale', 1.0) * 0.006

    arrow.pose.orientation.w = 1

    start = Point()
    start.x = position[0]
    start.y = position[1]
    start.z = position[2]
    end = Point()
    end.x = target_position[0]
    end.y = target_position[1]
    end.z = target_position[2]
    arrow.points.append(start)
    arrow.points.append(end)

    arrow.color.a = 1

    return arrow


def plot_arrows_rviz(pub, positions, directions):
    msg = MarkerArray()
    for i, (position, direction) in enumerate(zip(positions, directions)):
        msg.markers.append(rviz_arrow(position, position + direction, frame_id='world', idx=i, label='sdf_grad'))
    pub.publish(msg)


def visualize_voxelgrid(pub, vg, res, origin_point):
    msg = vox_to_voxelgrid_stamped(vg, scale=res, frame_id='world', origin=origin_point)
    pub.publish(msg)


def main():
    rospy.init_node("sdf_demo_rviz")

    pc_pub = rospy.Publisher("points", PointCloud2, queue_size=10)
    sdf_pub = rospy.Publisher("sdf", PointCloud2, queue_size=10)
    sdf_grad_pub = rospy.Publisher("sdf_grad", MarkerArray, queue_size=10)
    vg_pub = rospy.Publisher("vg", VoxelgridStamped, queue_size=10)

    rospy.sleep(0.1)

    pc = create_point_cloud()

    res = 0.04
    shape = [25, 20, 1]
    origin_point = np.array([0, 0, 0], dtype=np.float32)
    vg = point_cloud_to_voxel_grid(pc, shape, res, origin_point)
    sdf, sdf_grad = compute_sdf_and_gradient(vg, res, origin_point)

    grid_points = get_grid_points(origin_point, res, shape)
    subsample = 1
    grad_scale = 0.02

    for i in range(5):
        visualize_point_cloud(pc_pub, pc)
        visualize_voxelgrid(vg_pub, vg, res, origin_point)
        visualize_sdf(sdf_pub, sdf, shape, res, origin_point)
        arrow_start_points = grid_points.reshape([-1, 3])[::subsample]
        arrow_directions = sdf_grad.reshape([-1, 3])[::subsample] * grad_scale
        plot_arrows_rviz(sdf_grad_pub, arrow_start_points, arrow_directions)
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
