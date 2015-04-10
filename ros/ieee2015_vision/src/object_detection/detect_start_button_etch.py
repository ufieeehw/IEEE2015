import numpy as np
import cv2

def detect_start_button_etch(img, draw):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	#should only need the upped contrast image below
	ret, thresh3 = cv2.threshold(gray, 250, 255, cv2.THRESH_TRUNC)
	#thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 23, 3)
	
	if draw is True:
		cv2.imshow('thresh', thresh3)
	
	circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 10, 200, 100, 100, 100, 50)

	circle = circles[0]
	
	if draw is True:
		cv2.circle(img, (circle[0][0], circle[0][1]), 0, (0, 255, 0), 20)
		cv2.imshow('circles', img)
		#cv2.waitKey(0)

	return circle

#img = cv2.imread('closeTest/closess2.jpg')
#detect_start_button_etch(img, True)
	


