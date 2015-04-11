from __future__ import division
import numpy as np


def cv_to_forward(pt, shape=(640, 480)):
    '''Convert a pt from cv pixel frame to forward-facing'''
    forward_pt = (shape[1] - pt[1], (shape[0] // 2) - pt[0])
    return forward_pt


def forward_to_robot(pt, offset, scale=0.3/20.):
    robot_pt = np.array([offset + (pt[0] * scale), (pt[1] * scale)])
    return robot_pt


def cv_to_robot(pt, shape, offsets, scale):
    return forward_to_robot(cv_to_forward(pt, shape), offsets, scale)


if __name__ == '__main__':
    tests_a = [
        (0, 0),
        (320, 240),
        (360, 320),
    ]

    for test in tests_a:
        cv2f = cv_to_forward(test)
        print 'cv2f', cv2f
        print forward_to_robot(cv2f, 0.23, 1/20)