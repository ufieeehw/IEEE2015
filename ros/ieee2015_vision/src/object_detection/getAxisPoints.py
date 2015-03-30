import cv
import cv2
import numpy as np
import math
import getMidAndQuarterPoints


#read in the image, resize is just so that I can see on screen

def getAxisPoints(img):
	#gray color needed for threshold
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
	                cv2.THRESH_BINARY,37,8)

	#anytime we show an image copy it then show it
	#fakeThresh = thresh.copy()
	#cv2.imshow('using adaptiveThreshold', fakeThresh)
	#cv2.waitKey(0)

	#flip black and white pixels
	thresh = (255-thresh)

	#kernel for eroding
	kernel = np.ones((5,5), np.uint8)
	#kernel for dilating
	kernel2 = np.ones((8, 5), np.uint8)
	thresh = cv2.dilate(thresh, kernel2)
	thresh = cv2.erode(thresh, kernel)

	fakedilated = thresh.copy()
	#cv2.imshow('after dilating and eroding', fakedilated)
	#cv2.waitKey(0)

	contours,hierarchy = cv2.findContours(thresh,2,1)

	#finding contour for the toy 
	wholeToy = []
	for bae2 in contours:
	    area = cv2.contourArea(bae2)
	    print area
	    if area > 20000:
	    	print area
	    	#cv2.drawContours(img, [bae2], 0, (0, 255, 0), 10)
	    	wholeToy.append(bae2)

	#test img
	#tempimg = img.copy()
	#cv2.imshow('contours', tempimg)
	#cv2.waitKey(0)

	#first element should be only one and be the toy contour
	toyCnt = wholeToy[0]

	#gives us the angle of rotation and minor and jmajor axis lengths
	(x,y), (MA, ma), angle = cv2.minAreaRect(toyCnt)
	boxpoints = cv2.minAreaRect(toyCnt)
	points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
	points = np.int0(np.around(points)) 

	cv2.drawContours(img, [points], 0, (0, 0, 255), 1)
	cv2.imshow('detected box', img)
	cv2.waitKey(0)
	#PLOTS THE 4 CORNER POINTS OF THE RECTANGLE
	#Testing
	cv2.circle(img,(points[0][0], points[0][1]),2,(0,0,255),10)
	cv2.circle(img,(points[1][0], points[1][1]),2,(0,0,255),10)
	cv2.circle(img,(points[2][0], points[2][1]),2,(0,0,255),10)
	cv2.circle(img,(points[3][0], points[3][1]),2,(0,0,255),10)

	x = int(x)
	y = int(y)

	ma = int(ma)
	MA = int(MA)

	#angle = angle * -1
	cv2.circle(img,(x, y),2,(0,0,255),30)

	cv2.imshow('venterpoint', img)
	cv2.waitKey(0)

	#gives us the dimensions so that we can form rotation matrix
	rows,cols,pages = img.shape

	#positive angle goes counterclockwise
	M = cv2.getRotationMatrix2D((cols/2,rows/2), angle,1)
	dst = cv2.warpAffine(img, M, (cols,rows))

	cv2.imshow('rotated image', img)
	cv2.waitKey(0)

	###############3FOLLOWING WAS TO TEST POINT CHANGE######
	## angle is not moving correctly
	#tempangle = angle
	#tempangle = tempangle * -1
	#tempRadian = math.radians(tempangle)

	#xPrime = x*math.cos(tempRadian) - y*math.sin(tempRadian)
	#yPrime = x*math.sin(tempRadian) + y*math.cos(tempRadian)
	#xPrime = int(xPrime)
	#yPrime = int(yPrime)

	#print'this is x after'
	#print xPrime
	#print 'this is y after'
	#print yPrime


	#################METHOD 1##############################3
	#look into further!
	###use new detected center as a marker
	#	dstgray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)

	#circles = cv2.HoughCircles(dstgray, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 1, 100, 10, 5)
	#if circles is not None:
	 #   circles = np.uint16(np.around(circles))
	  #  for i in circles[0,:]:
	    # draw the outer circle
	   #     cv2.circle(dst,(i[0],i[1]),i[2],(0,255,0),2)
	        # draw the center of the circle
#	        cv2.circle(dst,(i[0],i[1]),2,(0,0,255),3)


	minAxisMid, majAxisMid, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2 = getMidAndQuarterPoints.getMidAndQuarterPoints(points, ma, MA)


	###Testing to display detected center points
	for a in minAxisMid:
		cv2.circle(img,a, 2,(255,255,255),10)
	for b in majAxisMid:
		cv2.circle(img,b, 2,(255,255,255),10)
	for c in quarterMin1:
		cv2.circle(img,c, 2,(255,255,255),10)
	for d in quarterMin2:
		cv2.circle(img,d, 2,(255,255,255),10)
	for e in quarterMaj1:
		cv2.circle(img,e, 2,(255,255,255),10)
	for f in quarterMaj2:
		cv2.circle(img,f, 2,(255,255,255),10)
	


	print 'this is quarterMin1'
	print quarterMin1

	print 'this is quarterMin2'
	print quarterMin2

	print 'this is quarterMaj2'
	print quarterMaj2

	print 'this is quarterMaj1'
	print quarterMaj1

	###Points in which to decide line for major and minor axis lines
	p1CentMinor = minAxisMid[0]
	p2CentMinor = minAxisMid[2]

	p1CentMajor = majAxisMid[0]
	p2CentMajor = majAxisMid[2]

	cv2.line(img, p1CentMajor, p2CentMajor, (0, 255, 0), 5)
	cv2.line(img, p1CentMinor, p2CentMinor, (0, 0, 0), 5)

	cv2.imshow('finla img', img)
	cv2.waitKey(0)

	return angle, dst, x, y, p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2