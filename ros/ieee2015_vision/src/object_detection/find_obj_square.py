import cv2
import numpy as np


def find_obj_square(img, draw):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #range for the white squares
    thresh = cv2.inRange(hsv, np.array((0, 0, 160)), np.array((255, 255, 255)))
    thresh = 255 - thresh
    kernel = np.ones((8, 8), np.uint8)
    #kernel for dilating
    kernel2 = np.ones((8, 5), np.uint8)
    thresh = cv2.dilate(thresh, kernel2)
    thresh = cv2.erode(thresh, kernel)

    if draw is True:
        cv2.imshow('thresh', thresh)

    l, w, h = img.shape
    #area_of_image = l * w

    thresh = 255 - thresh
    closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    points = []
    for cnt in contours:
        (x, y), (MA, ma), angle = cv2.fitEllipse(cnt)
        point = (int(x), int(y))
        points.append(point)
        if draw is True:
            cv2.circle(img, point, 5, [0, 0, 255], -1)
            cv2.drawContours(img, [cnt], 0, (255, 255, 255), 10)

    if draw is True:
        cv2.imshow('center points', img)

    return points
