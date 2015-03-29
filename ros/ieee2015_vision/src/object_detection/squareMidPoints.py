import cv
import cv2
import numpy as np
import math
import determineSide
import ss_getLitUpButton


#read in the image, resize is just so that I can see on screen

def getAxisPoints(img):

	cv2.imshow('start',img) 
	cv2.waitKey(0)

	#gray color needed for threshold
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
	                cv2.THRESH_BINARY,37,8)


	#anytime we show an image copy it then show it
	fakeThresh = thresh.copy()
	cv2.imshow('using adaptiveThreshold', fakeThresh)
	cv2.waitKey(0)

	#flip black and white pixels
	thresh = (255-thresh)

	#kernel for eroding
	kernel = np.ones((5,5), np.uint8)
	#kernel for dilating
	kernel2 = np.ones((8, 5), np.uint8)
	thresh = cv2.dilate(thresh, kernel2)
	thresh = cv2.erode(thresh, kernel)


	fakedilated = thresh.copy()
	cv2.imshow('after dilating and eroding', fakedilated)
	cv2.waitKey(0)

	contours,hierarchy = cv2.findContours(thresh,2,1)

	#finding contour for the toy 
	wholeToy = []
	for bae2 in contours:
	    area = cv2.contourArea(bae2)
	    print area
	    if area > 200000:
	    	print area
	    	#cv2.drawContours(img, [bae2], 0, (0, 255, 0), 10)
	    	wholeToy.append(bae2)

	#test img
	tempimg = img.copy()
	cv2.imshow('contours', tempimg)
	cv2.waitKey(0)

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
	cv2.circle(img,(points[0][0], points[0][1]),2,(0,0,255),30)
	cv2.circle(img,(points[1][0], points[1][1]),2,(0,0,255),30)
	cv2.circle(img,(points[2][0], points[2][1]),2,(0,0,255),30)
	cv2.circle(img,(points[3][0], points[3][1]),2,(0,0,255),30)

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

	print 'this is rows'
	print rows

	print 'this is cols'
	print cols
	#positive angle goes counterclockwise
	M = cv2.getRotationMatrix2D((cols/2,rows/2), angle,1)
	dst = cv2.warpAffine(img, M, (cols,rows))


	cv2.imshow('rotated image', dst)
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


	#hold the two points that are the min axis distance
	minAxis = []
	#hold the two points that are the max axis distance
	majAxis = []
	for i in range(0, 4):
		for j in range(0, 4):
			tempnum = math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
			print 'this is tempnum'
			print tempnum
			if tempnum < ma + 10 and tempnum > ma - 10:
				#calculate midpoint between that stuff
				point = ((points[i][0] + points[j][0])/2, (points[i][1] + points[j][1])/2)
				minAxis.append(point)
			elif tempnum < MA + 10 and tempnum > MA - 10:
				point = ((points[i][0] + points[j][0])/2, (points[i][1] + points[j][1])/2)
				majAxis.append(point)

	print 'this is minAxis' #this example it is 338 ma is major axis
	print ma

	print 'this is majAxis' #t MA is minor axis his example is 236
	print MA

	print len(minAxis)
	print len(majAxis)

	print minAxis
	print majAxis


	###Testing to display detected center points
	for a in minAxis:
		cv2.circle(img,a, 2,(0,255,0),10)
	for b in majAxis:
		cv2.circle(img,b, 2,(0,255,0),10)

	cv2.imshow('centerpoints', img)
	cv2.waitKey(0)



	###Putting points in order###
	list(minAxis)
	list(majAxis)

	minAxis.sort()
	majAxis.sort()

	print 'should be sorted'
	print majAxis
	print minAxis


	###Points in which to decide line for major and minor axis lines
	p1Minor = minAxis[0]
	p2Minor = minAxis[2]

	p1Major = majAxis[0]
	p2Major = majAxis[2]

	cv2.line(img, p1Major, p2Major, (0, 255, 0), 5)
	cv2.line(img, p1Minor, p2Minor, (0, 0, 0), 5)

	
	
	#return p1Major, p2Major, p1Minor, p2Minor, angle
	img2 = cv2.imread('Images/Set3/snorm3.JPG')
	img2 = cv2.resize(img2, (0,0), fx=0.5, fy=0.5)
	img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
	meanCols, meanRows, closing = ss_getLitUpButton.getLitUpButton(img2)
	cv2.circle(img,(meanRows, meanCols), 2,(255,255,255),10)
	img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
	print meanRows
	print meanCols
	cv2.imshow('point to test', img)
	cv2.waitKey(0)


	majorVal, minorVal = determineSide.determineSide(p1Major, p2Major, p1Minor, p2Minor, (520,333), angle)

	print majorVal
	print minorVal

img = cv2.imread('Images/Set3/snorm1.JPG')
img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
getAxisPoints(img)