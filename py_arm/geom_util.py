import numpy as np

import shapely.affinity
import shapely.speedups

shapely.speedups.enable()


def rot_matrix(theta):
    """2d rotation matrix.

    Args:
       theta - Rotation in degrees.

    Returns:
       3x3 homogeneous rotation matrix

    """
    mat = np.eye(3)
    angle = theta * np.pi / 180.0
    mat[0, 0] = np.cos(angle)
    mat[0, 1] = -np.sin(angle)
    mat[1, 0] = np.sin(angle)
    mat[1, 1] = np.cos(angle)
    return mat


def trans_matrix(x, y):
    """ 2D Translation matrix """
    mat = np.eye(3)
    mat[0, 2] = x
    mat[1, 2] = y
    return mat


def affine_transform(geom, mat):
    """Perform shapely affine transform (which requires a strange format for
    the transformation matrix)

    Args:
        geom - Any transformable shapely geometry.
        mat - 3x3 homogeneous transformation matrix.

    """
    sh_mat = [mat[0, 0], mat[0, 1], mat[1, 0], mat[1, 1], mat[0, 2],
              mat[1, 2]]
    return shapely.affinity.affine_transform(geom, sh_mat)


# https://stackoverflow.com/questions/42617529/
# how-can-i-vectorize-linspace-in-numpy
def vlinspace(a, b, N, endpoint=True):
    a, b = np.asanyarray(a), np.asanyarray(b)
    return a[..., None] + (b - a)[..., None] / (N - endpoint) * np.arange(N)


def angle_diff(x, y):
    # https://stackoverflow.com/questions/1878907/
    # the-smallest-difference-between-2-angles
    x = np.deg2rad(x)
    y = np.deg2rad(y)
    diffs = np.arctan2(np.sin(x - y), np.cos(x - y))
    diffs = np.rad2deg(diffs)
    diffs[np.isnan(diffs)] = float('inf')
    return diffs


def angle_metric_l2(x, y):
    diffs = angle_diff(x, y)
    return np.sqrt(np.sum(diffs ** 2))


def angle_sequence(q1, q2, max_delta):
    """
    Compute a linearly interpolated sequence of steps to move from q1 to q2
    without exceeding a maximum angular step size.

    Args:
        q1: Start configuration
        q2: Target configuration
        max_delta: Largest allowed angular step for any joint

    Returns:
        A linearly interpolated sequence of steps that moves from q1 to q2.
        q1 will not be included in the return value.  It will end with q2.

    """
    max_angle_diff = np.max(np.abs(angle_diff(q2, q1)))
    num_steps = np.ceil(float(max_angle_diff) / max_delta) + 1
    steps = vlinspace(q1, q1 + angle_diff(q2, q1), num_steps)
    steps = steps[:, 1::].T
    return steps
