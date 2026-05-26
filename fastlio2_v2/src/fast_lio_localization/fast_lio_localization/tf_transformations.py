import numpy as np


def quaternion_from_matrix(matrix):
    """Extract quaternion [w, x, y, z] from 4x4 transformation matrix."""
    m = np.array(matrix, dtype=float)
    qw = np.sqrt(max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    qx = np.sqrt(max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2
    qy = np.sqrt(max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2
    qz = np.sqrt(max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2
    qw = np.copysign(qw, m[2, 1] - m[1, 2])
    qx = np.copysign(qx, m[0, 2] - m[2, 0])
    qy = np.copysign(qy, m[1, 0] - m[0, 1])
    qz = np.copysign(qz, 1.0)
    return np.array([qw, qx, qy, qz])


def translation_from_matrix(matrix):
    """Extract translation [x, y, z] from 4x4 transformation matrix."""
    m = np.array(matrix, dtype=float)
    return m[:3, 3]
