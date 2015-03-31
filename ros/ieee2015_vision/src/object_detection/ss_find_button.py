import cv2
import getMidPoints
import math
import numpy as np
import ss_getLitUpButton as litButton

def ss_find_button(img, detectedBright):
	cv2.circle(img, detectedBright, 2, (255, 255, 255), 20)

	angle, dst, x, y, p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2 = getMidPoints.getAxisPoints(img)

	cv2.line(img, p1CentMinor, p2CentMinor, (0, 0, 0), 5)

	print 'this is angle'
	print angle

	temppoint = (x, y)

	cv2.line(img, temppoint, detectedBright, (0, 0, 0), 5)

	calcAngle = math.atan2((detectedBright[1] - y), (detectedBright[0] - x))
	print calcAngle
	calcAngle %= 2*np.pi
	degs = math.degrees(calcAngle)
	degs = int(360 - degs + angle)
	print degs

	#WHOOOT WHOOOT WE GOT ANGLES WORKING

	color = 0
	#1 is blue/right
	#2 is red/up
	#3 is green/left
	#4 is yellow/down
	if (degs > 0 and degs < 50) or degs > 315:
		print "we have a blue thing"
		color =  1
	elif degs >= 50 and degs <= 130:
		color = 2
		print "we have a red thing"
	elif degs >130 and degs <= 225:
		color = 3
		print "we have a green thing"
	elif degs > 225 and degs <= 315:
		color = 4
		print "we have a yellow thing"

	cv2.imshow('final image for real', img)
	cv2.waitKey(0)

	return color

img = cv2.imread('Images/Set3/snorm9.JPG')
img = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
findButton(img, (300, 100))