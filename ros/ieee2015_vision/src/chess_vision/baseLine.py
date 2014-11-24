import cv2
import numpy as np


def findAngle(filename):

    filename = filename
    gray = cv2.imread(filename, 0)

    edges = cv2.Canny(gray, 50, 150, apertureSize = 3)

    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

    rho, theta = lines[0][0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))

    if (theta > np.pi / 4 and theta < np.pi * (3 / 4)):
        angleOfRotation = theta - np.pi / 2
    elif (theta > 0 and theta < np.pi / 4):
        angleOfRotation = theta
    else:
        angleOfRotation = theta - np.pi

    return angleOfRotation * (180 / np.pi)
