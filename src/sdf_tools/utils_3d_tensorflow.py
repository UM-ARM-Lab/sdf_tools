import tensorflow as tf

from sdf_tools.utils_3d import compute_sdf_and_gradient


def compute_sdf_and_gradient_batch(env, res, origin_point, batch_size: int):
    sdfs = []
    sdf_grads = []
    for b in range(batch_size):
        sdf, sdf_grad = compute_sdf_and_gradient(env[b], res[b], origin_point[b])
        sdfs.append(sdf)
        sdf_grads.append(sdf_grad)
    sdf_batch = tf.stack(sdfs, 0)
    sdf_grad_batch = tf.stack(sdf_grads, 0)
    return sdf_batch, sdf_grad_batch


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

    return compute_sdf_and_gradient(env, res, origin_point)
