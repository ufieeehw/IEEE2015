import numpy as np
import cv2
from matplotlib import pyplot as plt


#Reads image in two copies. One is color, one is grayscale.
colorimage = cv2.imread('card1.jpg') 
greyscale= cv2.cvtColor(colorimage,cv2.COLOR_BGR2GRAY)

#Creates an image called threshold, which is the threshold of the grayscale image
ret,thresh = cv2.threshold(greyscale,127,255,0)

#Displays original color image
cv2.namedWindow('colorimage', cv2.WINDOW_NORMAL)
cv2.imshow('colorimage', colorimage)
cv2.waitKey(0)
cv2.destroyAllWindows()

#Displays thresholded image
cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
cv2.imshow('thresh', thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()

#finds countours of threshold image 
contours,h = cv2.findContours(thresh,1,2)


#Goes through the contours image and draws green pixels where it finds rectangles
#on top of the original color image

#rectanglecounter = 0
for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    
    if len(approx)==4:
        cv2.drawContours(colorimage,[cnt],0,(0,255,0),-1)

#creates a new greyscale image
greyscale= cv2.cvtColor(colorimage,cv2.COLOR_BGR2GRAY)

#Displays new greyscaleimage
cv2.namedWindow('greyscale2', cv2.WINDOW_NORMAL)
cv2.imshow('greyscale2', greyscale)
cv2.waitKey(0)
cv2.destroyAllWindows()


#Creates an image called threshold, which is the threshold of the grayscale image
ret,thresh = cv2.threshold(greyscale,127,255,0)

#finds countours of threshold image 
contours,h = cv2.findContours(thresh,1,2)

#Goes through the contours image and draws green pixels where it finds rectangles
#on top of the original color image
rectanglecounter = 0
for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    
    if len(approx)==4:
        #print "Rectangle"
	rectanglecounter = rectanglecounter + 1
	#print rectanglecounter
        cv2.drawContours(colorimage,[cnt],0,(255,0,0),-1)

#Shows an error if there is more than one rectangle
if (rectanglecounter >1 ):
	#Put the return error here
	print "Error, more than one Rectangle!"
	print "Amount of Rectangles Detected:"
	print rectanglecounter

#Displays the color image with the green squares drawn on top of it
cv2.namedWindow('SquaredoutImage', cv2.WINDOW_NORMAL)
cv2.imshow('SquaredoutImage', colorimage)
cv2.waitKey(0)
cv2.destroyAllWindows
