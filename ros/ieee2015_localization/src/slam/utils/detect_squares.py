#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot
import os
import roslib
roslib.load_manifest('ieee2015_localization')
from slam.utils.scaling import cv_to_robot


DEBUG = True


def nav_detect_squares(image, shape, offset, scale):
    squares = detect_squares(image)
    world_frame_squares = []
    for square in squares:
        world_frame_squares.append(cv_to_robot(square['center'], shape, offset, scale))

    return world_frame_squares


def detect_squares(image, min_size=100):
    _, white_all = cv2.threshold(image, 170, 255, cv2.THRESH_BINARY)

    if DEBUG:
        cv2.imshow('whiteallindow', white_all)

    # Brandon's Method
    contours, _ = cv2.findContours(white_all, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    _, white_squares = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    # -----------------

    edged = cv2.Canny(white_squares, 30, 200)
    dilator_kernel = np.ones((8, 5), np.uint8)
    edged = cv2.dilate(edged, dilator_kernel)

    if DEBUG:
        cv2.imshow("Edges", edged)

    (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    objects = np.zeros_like(edged) * 255

    # Loop over our contours
    for object_contour in cnts:
        cvx_hull = cv2.convexHull(object_contour)
        area = cv2.contourArea(cvx_hull)

        if area < min_size:
            continue

        # Draw filled contours
        cv2.drawContours(objects, [cvx_hull], -1, 255, -10)
        # Expand these contours
        cv2.drawContours(objects, [cvx_hull], -1, 255, 10)
    cv2.imshow("Objects", objects)

    rectangle_contours, _ = cv2.findContours(objects, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    rectangles = []
    for contour in rectangle_contours:
        center, area = compute_centroid(contour)
        rectangles.append({
            'center': center,
            'area': area
            }
        )

    return rectangles


def compute_centroid(cntr):
    '''Return the centroid and area of a contour'''
    return np.average(cntr, 0).flatten(), cv2.contourArea(cntr)


def merge_rectangles(rectangles, dist_thresh=30):
    '''Don't use this yet'''
    merged = []
    to_examine = rectangles[:]
    for rectangle in rectangles:
        for key, other_rectangle in enumerate(to_examine):
            if np.linalg.norm(rectangle['center'] - other_rectangle['center']) < dist_thresh:
                to_examine.pop(key)


def main():
    fpath = os.path.dirname(os.path.realpath(__file__))

    for k in range(1, 8):
        print k

        oimg = cv2.imread(os.path.join(fpath, 'xformed{}.png'.format(k)))
        img = cv2.cvtColor(oimg, cv2.COLOR_BGR2GRAY)
        rectangles = detect_squares(img)

        for rectangle in rectangles:
            center, area = rectangle['center'], rectangle['area']
            print center
            cv2.circle(oimg, (int(center[0]), int(center[1])), 10, color=(255, 0, 0), thickness=-1)

        cv2.imshow("Image", oimg)

        merge_rectangles(rectangles)
        cv2.waitKey(0)

if __name__ == "__main__":
    main()