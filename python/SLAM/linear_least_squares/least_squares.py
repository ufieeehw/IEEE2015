__author__ = 'patrickemami'

import numpy as np

'''
Calculate the Rotation Matrix, R, and Translation Matrix, T, that
minimizes the linear least squares problem defined by two point sets

http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965
'''
def linear_least_squares(point_set_1, point_set_2):
    """
    :param point_set_1: 3-D series of points
    :param point_set_2: 3-D series of points
    :return:
    """
    point_set_1_mean = np.mean(point_set_1)
    point_set_2_mean = np.mean(point_set_2)

    q_1 = np.matrix([p_i - point_set_1_mean for p_i in point_set_1])
    q_2 = np.matrix([p_i - point_set_2_mean for p_i in point_set_2])

    # Dot product of the 3 x n matrix q_1 and n x 3 matrix q_2'
    h = q_1 * q_2

    # Calculate the SVD of 3 x 3 matrix h
    u, s, v = np.linalg.svd(h, full_matrices=True)

    # Generate an orthonormal matrix x
    x = v * u

    det = np.linalg.det(x)

    if np.isclose(det, 1.0):
        R = x
    elif np.isclose(det, -1.0):
        print "Linear least squares method failed."
        return
    else:
        print "Something not mathematical happened"
        return

    # T = p' - Rp
    T = point_set_2_mean - R * point_set_1_mean

    return R, T

if __name__ == '__main__':
    print linear_least_squares(np.random.randn(3, 3), np.random.randn(3, 3))