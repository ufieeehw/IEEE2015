import cv2
import numpy as np
import math


def detect_etcha_sketch(img, height, draw):
	grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gray = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 41, 7)
	#gray = cv2.erode(gray, kernelg)
	
	kernel2 = np.ones((10, 10), np.uint8)
	gray = cv2.erode(gray, kernel2)

	if draw is True:
		cv2.imshow('adaptive thresh', gray)
		cv2.waitKey(0)

	contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	area_of_inner_square = (14366097 * height**(2)) - (5004772 * height) + 471677
	sigma = 10000

	if draw is True:
		print 'this is area_of_inner_square'
		print area_of_inner_square
	
	lower_bound = area_of_inner_square - sigma
	upper_bound = area_of_inner_square + sigma

	squares = []
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > lower_bound and area < upper_bound: #area value is fishy need for straight across par
			if draw is True:
				print 'this is area'
				print area
			squares.append(cnt)
		
	if draw is True:
		for i in squares:
			cv2.drawContours(img, [i], 0, (255, 255, 255), 10)
		cv2.imshow('contours', img)
		cv2.waitKey(0)

	center_square = squares[0]

	boxpoints = cv2.minAreaRect(center_square)
	points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
	points = np.int0(np.around(points))
	cv2.drawContours(img,[points],0,(0,0,255),2)

	if draw is True:	
		cv2.imshow('img with contours', img)
		cv2.waitKey(0)

	angle = boxpoints[2]

	radians = math.radians(angle)

	center = boxpoints[0]
	
	return center, radians

#img = cv2.imread('es/11.5es.jpg')
#detect_etcha_sketch(img, .115, True)