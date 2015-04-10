import numpy as np
import cv2

def detect_start_button_etch(img):
	ret, thresh3 = cv2.threshold(img, 250, 255, cv2.THRESH_TRUNC)
	thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 23, 3)

	cv2.imshow('thresh', thresh)
	circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 10, 200, 100, 100, 100, 50)
	    #circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 100, 100, 50)

	circle = circles[0]
	cv2.circle(img, (circle[0][0], circle[0][1]), 0, (0, 255, 0), 20)

	cv2.imshow('circles', img)
	cv2.waitKey(0)

	return circle

img = cv2.imread('closeTest/closess2.jpg', 0)
detect_start_button_etch(img)
	


