import cv
import cv2
import numpy as np
import math

img = cv2.imread('Images/ssR.jpg')
img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)

cv2.imshow('start',img) 
cv2.waitKey(0)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                cv2.THRESH_BINARY,37,8)

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
#below tickens lines


fakedilated = thresh.copy()
cv2.imshow('after dilating and eroding', fakedilated)
cv2.waitKey(0)

contours,hierarchy = cv2.findContours(thresh,2,1)


wholeToy = []
for bae2 in contours:
    area = cv2.contourArea(bae2)
    print area
    if area > 20000:
    	print area
    	#cv2.drawContours(img, [bae2], 0, (0, 255, 0), 10)
    	wholeToy.append(bae2)

tempimg = img.copy()
cv2.imshow('contours', tempimg)
cv2.waitKey(0)

toyCnt = wholeToy[0]

(x,y), (MA, ma), angle = cv2.minAreaRect(toyCnt)
boxpoints = cv2.minAreaRect(toyCnt)
points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
points = np.int0(np.around(points)) 
#points = points/2
#cv2.drawContours(img, [points], 0, (0, 0, 255), 10)

#PLOTS THE 4 CORNER POINTS OF THE RECTANGLE
cv2.circle(img,(points[0][0], points[0][1]),2,(0,0,255),30)
cv2.circle(img,(points[1][0], points[1][1]),2,(0,0,255),30)
cv2.circle(img,(points[2][0], points[2][1]),2,(0,0,255),30)
cv2.circle(img,(points[3][0], points[3][1]),2,(0,0,255),30)

#for i in range(0:4):
	#for j in range(0:4):
		

print 'this is what contours looks like'
print len(contours)
print points

x = int(x)
y = int(y)

ma = int(ma)
MA = int(MA)

#angle = angle * -1
cv2.circle(img,(x, y),2,(0,0,255),30)
print 'this is x before'
print x

print 'this is y before'
print y

cv2.imshow('venterpoint', img)
cv2.waitKey(0)

print 'this is angle of rotation'
print angle

rows,cols,pages = img.shape

#positive angle goes counterclockwise
M = cv2.getRotationMatrix2D((cols/2,rows/2), angle,1)
dst = cv2.warpAffine(img, M, (cols,rows))

#cv2.circle(dst,(x, y),2,(0,0,255),30)
cv2.imshow('roatted', dst)
cv2.waitKey(0)
#angle is -11.00689o

tempangle = angle
#tempangle = tempangle * -1
tempRadian = math.radians(tempangle)

xPrime = x*math.cos(tempRadian) - y*math.sin(tempRadian)
yPrime = x*math.sin(tempRadian) + y*math.cos(tempRadian)
xPrime = int(xPrime)
yPrime = int(yPrime)

print'this is x after'
print xPrime
print 'this is y after'
print yPrime

#look into further!
#img = cv2.rectangle(dst,(384,0),(510,128),(255,255,255),3)
dstgray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)

circles = cv2.HoughCircles(dstgray, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 1, 100, 10, 5)
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
    # draw the outer circle
        cv2.circle(dst,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(dst,(i[0],i[1]),2,(0,0,255),3)

#cv2.circle(dst,(xPrime, yPrime),2,(255,255,255),30)
cv2.imshow('new point', dst)
cv2.waitKey(0)
