# algorithm to detect the rubix cube
import numpy as np
import cv2
import math

#give the function a filename as an argument
#returns the center and angle (point and float) or none
def findRubix(filename):
  src = imread(filename, IMREAD_COLOR) #get the image
  if(!src.data):
    print 'Invalid Image'
    return None;  # no useful data
  
	hsv_img = cvtColor(src, CV_BGR2HSV) #convert to HSV
	fiter_img = inRange(hsv_img, array[0,150,100], array[180,255,255]) #filter all weak colors
	blur_img = medianBlur(filter_img, 5) #blur the image
  
  contours = findContours(img_blur, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE) #get the contours
  
  rectList = [] # create list for rectangles
  for c in contours:
    rectList.append(minAreaRect(c)) #get a rectangle around the contour
  
  # I'm realizing that extra contours might be an issue, will keep an eye on it
  
  #average the points
  for i in range(len(rectList)):
    angle = (angle*i + squares[i].angle/180*pi)/(i+1)  # convert to radians
    x = (x*i + squares[i].center.x)/(i+1)
    y = (y*i + squares[i].center.y)/(i+1)
  
  # move origin from upper left to center of image
  x = x - (img_blur.rows / 2)
  y = (img_blur.cols / 2) - y
  
  return {'center':(x,y), 'angle':angle}