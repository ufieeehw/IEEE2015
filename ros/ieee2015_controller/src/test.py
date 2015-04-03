import numpy as np

mecanum_matrix = np.matrix([
    [+1, +1, +1, +1],  # Unitless! Shooting for rad/s
    [+1, -1, +1, -1],  # Unitless! Shooting for rad/s
    [+1, +1, -1, -1],  # Unitless! Shooting for rad/s
    # [+1, -1, -1, +1],  # This is the error row (May not be necessary)
], dtype=np.float32) / 4.0  # All of the rows are divided by 4

v_target = np.array([0.0, 1.0, 0.0])

print np.linalg.lstsq(mecanum_matrix, v_target)[0]
