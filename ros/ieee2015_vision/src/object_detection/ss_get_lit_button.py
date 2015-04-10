import cv2
import numpy as np


def get_lit_button(img, draw):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_on = np.array([0, 80, 140])
    upper_on = np.array([255, 255, 255])

    bright_button = cv2.inRange(hsv_img, lower_on, upper_on)

    kernel = np.ones((8, 8), np.uint8)
    kernel2 = np.ones((8, 8), np.uint8)
    kernel3 = np.ones((4, 4), np.uint8)
    eroded = cv2.erode(bright_button, kernel2)
    dilated = cv2.dilate(eroded, kernel3)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=10)

    if draw is True:
        cv2.imshow('closing', closing)

    ############################################Start of Contour Manipulation######################
    #get minimum contour
    contours, hierarchy = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return -1
    #best_ctn should ideally be length of one at this point
    lit_button = contours[0]

    leftmost = tuple(lit_button[lit_button[:, :, 0].argmin()][0])
    rightmost = tuple(lit_button[lit_button[:, :, 0].argmax()][0])
    topmost = tuple(lit_button[lit_button[:, :, 1].argmin()][0])
    bottommost = tuple(lit_button[lit_button[:, :, 1].argmax()][0])

    #Testing
    if draw is True:
        cv2.circle(img, (leftmost), 2, (0, 0, 255), 10)
        cv2.circle(img, (rightmost), 2, (0, 0, 255), 10)
        cv2.circle(img, (topmost), 2, (0, 0, 255), 10)
        cv2.circle(img, (bottommost), 2, (0, 0, 255), 10)

    mean_cols = int((leftmost[0] + rightmost[0]) / 2)

    mean_rows = int((topmost[1] + bottommost[1]) / 2)

    if draw is True:
        cv2.circle(img, (mean_cols, mean_rows), 2, (0, 0, 0), 10)
        cv2.imshow('points', img)

    return (mean_cols, mean_rows)
