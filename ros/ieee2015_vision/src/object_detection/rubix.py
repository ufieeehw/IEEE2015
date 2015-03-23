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
  filter_imgG = cv2.inRange(hsv_img, (60,23, 65), (83,255,255)) #filter all weak colors
  filter_imgFinal = cv2.bitwise_or(filter_img, filter_imgG)
  blur_img = cv2.medianBlur(filter_imgFinal, 5) #blur the image
  
  cv2.imshow('contos', blur_img);
  cv2.waitKey(0);

  contours, hierarchy = cv2.findContours(blur_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #get the contours
  
  goodContours = []
  for current in contours:
        area = cv2.contourArea(current)
        print area
        if area > 4000:
            goodContours.append(current)


  for cnt in goodContours:
    cv2.drawContours(src,[cnt],0,(0,255,0),1)


  cv2.imshow('contos', src);
  cv2.waitKey(0);

  squares = [] # create list of the tile squares
  for c in goodContours:
    squares.append(cv2.minAreaRect(c)) #get a rectangle around the contour
  

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

findRubix('Images/Rubix/r2.jpg')