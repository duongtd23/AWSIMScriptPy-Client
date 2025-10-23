import numpy as np
import utils

def point_forward(base_position, forward_direction, x_shift, y_shift):
    """
    Get a point from the base position and forward direction with x_shift and y_shift.
    :param base_position: np array of size 2
    :param forward_direction: normalized direction vector. np array of size 2
    :param x_shift: longitudinal shift, should not be negative
    :param y_shift: lateral shift. Positive if right shift, negative if left shift.
    :return:
    """
    dis = np.sqrt(x_shift ** 2 + y_shift ** 2)
    fwd_point = base_position + forward_direction * dis
    angle = np.arctan2(np.abs(y_shift), x_shift)
    if y_shift > 0:
        angle = -angle
    return utils.rotate_point(fwd_point, base_position, angle)