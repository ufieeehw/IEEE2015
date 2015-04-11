import numpy as np
import cv2


def center_button_ss(img, draw):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh3 = cv2.threshold(gray, 250, 255, cv2.THRESH_TRUNC)
    #thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 23, 3)

    circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 10, 200, 100, 100, 100, 50)

    circle = circles[0]
    cv2.circle(img, (circle[0][0], circle[0][1]), 0, (0, 255, 0), 20)

    #if draw is True:
    #    cv2.imshow('circles', img)
        #cv2.waitKey(0)

    return circle

#img = cv2.imread('ti/15hss.jpg')
#center_button_ss(img, True)

