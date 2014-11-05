# algorithm to detect the rubix cube
import numpy as np
import cv2
import math

#give the function a filename as an argument
#returns the center and angle (point and float) or none
def findRubix(filename):
  src = cv2.imread(filename, cv2.IMREAD_COLOR) #get the image
  if src == None:
    print 'Invalid Image'
    return None;  # no useful data
  
  hsv_img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV) #convert to HSV
  filter_img = cv2.inRange(hsv_img, (0,150,100), (180,255,255)) #filter all weak colors
  blur_img = cv2.medianBlur(filter_img, 5) #blur the image
  
  contours, hierarchy = cv2.findContours(blur_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #get the contours
  
  squares = [] # create list of the tile squares
  for c in contours:
    squares.append(cv2.minAreaRect(c)) #get a rectangle around the contour
  
  # I'm realizing that extra contours might be an issue, will keep an eye on it
  
  #average the points
  angle = x = y = 0
  print squares[0]
  for i in range(len(squares)):
    angle = (angle*i + squares[i][2]/180*math.pi)/(i+1)  # convert angle to radians
    x = (x*i + squares[i][0][0])/(i+1)  # x coordinate of center
    y = (y*i + squares[i][0][1])/(i+1)  # y coordinate of center
  
  # move origin from upper left to center of image
  x = x - (len(blur_img) / 2)    #should be rows
  y = (len(blur_img[0]) / 2) - y #should be cols
  
  return {'center':(x,y), 'angle':angle}
