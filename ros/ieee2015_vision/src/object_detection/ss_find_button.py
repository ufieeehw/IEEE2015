import cv2
import ss_get_axis_points
import math
import numpy as np
import ss_get_lit_button


#now we don't care about what color the button is
#just care about location of bright center relative to major axis
def find_button(img, mean_cols, mean_rows):
	#testing
	cv2.circle(img, (mean_cols, mean_rows), 2, (255, 255, 255), 20)

	imgf = img.copy()
	
	cv2.imshow('img',imgf)
	cv2.waitKey(0)
	#get the lines of major and minor + angle or orientation for adjustments
	angle, points, goodcircle = ss_get_axis_points.get_axis_points(img, .17)
	
	#below is for testing requires a lot of uncommenting in get axis points too
	#cv2.line(img, p1CentMinor, p2CentMinor, (0, 0, 0), 5)
	x = goodcircle[0]
	y = goodcircle[1]
	#formating for testing
	temp_point = (x, y)

	cv2.line(img, temp_point, (mean_cols, mean_rows), (0, 0, 0), 5)

	#get the angle from 0-360 that the point lies, counting minor axis as x axis
	calc_angle = math.atan2((mean_rows - y), (mean_cols - x))
	calc_angle %= 2 * np.pi
	degs = math.degrees(calc_angle)
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

	imgg = img.copy()
	#imgg = cv2.resize(imgg, (0, 0), fx=0.2, fy=0.2)
	cv2.imshow('final image for real', imgg)
	cv2.waitKey(0)

	return color
img = cv2.imread('heights/17cmss.jpg')
#img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
find_button(img, 280, 300)