import math

import casadi
import numpy as np

def get_map_from_euler_zyx_deriv_to_local_angular_vel(euler_angles: np.array):
    """Compute the matrix transform that maps derivatives of euler angles (zyx) to local angular velocities."""
    sy = math.sin(euler_angles[1])
    cy = math.cos(euler_angles[1])
    sx = math.sin(euler_angles[2])
    cx = math.cos(euler_angles[2])

    M = casadi.SX([[-sy,      0.0,    1.0],
         [cy * sx,  cx,     0.0],
         [cx * cy,  -sx,    0.0]])

    return M

