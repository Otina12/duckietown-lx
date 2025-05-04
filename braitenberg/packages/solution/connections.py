import numpy as np
from typing import Tuple

def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    h, w = shape
    h_max, w_max = h - 1, w - 1
    center_x, center_y = w_max / 2, h_max

    ys, xs = np.indices((h, w))
    a = w_max / 2
    b = center_y - 0.5 * h_max
    
    dx = (xs - center_x) / a
    dy = (ys - center_y) / b
    d2 = dx**2 + dy**2

    res = np.clip(1 - 0.65 * d2, 0, 1)
    res[:, 0:w//2] *= -1
    res *= -1

    return res

def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    return get_motor_left_matrix(shape) * -1