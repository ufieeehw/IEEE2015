# algorithm to detect the rubix cube
import numpy as np
import cv2
# import cv
import math

#give the function a filename as an argument
#returns the center and angle (point and float) or none
def find_rubix(src, height):
<<<<<<< HEAD
=======

  #hsv_img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV) #convert to HSV
  #filter_img = cv2.inRange(hsv_img, (0,150,100), (180,255,255)) #filter all weak colors
  #filter_imgG = cv2.inRange(hsv_img, (60,23, 65), (83,255,255)) #filter all weak colors
  #filter_imgFinal = cv2.bitwise_or(filter_img, filter_imgG)
  #blur_img = cv2.medianBlur(filter_imgFinal, 5) #blur the image
  
  #cv2.imshow('contos', blur_img);
  # cv2.waitKey(0);

>>>>>>> 5abfa242a8d21007cf9664906ca4c796bf6e11da
  kernelg = np.ones((8,8), np.uint8)
  kernelD = np.ones((2, 2), np.uint8)
  grayscale = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
  gray = cv2.adaptiveThreshold(grayscale,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
          cv2.THRESH_BINARY,37,15)
  gray = cv2.erode(gray, kernelg)
<<<<<<< HEAD
=======
  #gray = cv2.dilate(gray, kernelD)
  cv2.imshow('adaptive thresh', gray)
  # cv2.waitKey(0)

  contours2, hierarchy2 = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
>>>>>>> 5abfa242a8d21007cf9664906ca4c796bf6e11da

  contours, hierarchy2 = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

  #using height for reference
  approx_area = (-415485 * height) + 139438
  #using 6000 as standard dev
  #using grayscale thresh
  goodContours2 = []
  for current in contours:
        area = cv2.contourArea(current)
        if area > (approx_area - 6000) and area < (approx_area + 6000): #will need to be adjusted !!!!!
            goodContours2.append(current)

<<<<<<< HEAD
=======
  for cnt in goodContours2:
    cv2.drawContours(src,[cnt],0,(0,255,0),1)

  cv2.imshow('contos', src);
  # cv2.waitKey(0);

  #squares = [] #goodContours[0] # create list of the tile squares
  #for c in goodContours:
    #squares.extend(c)
    #squares.append(cv2.minAreaRect(c)) #get a rectangle around the contour

>>>>>>> 5abfa242a8d21007cf9664906ca4c796bf6e11da
  boxpoints = cv2.minAreaRect(goodContours2[0])
  points = cv2.cv.BoxPoints(boxpoints) 
  points = np.int0(np.around(points)) 

  xVals =[]
  yVals = []
  for i in range(0, 4):
    xVals.append(points[i][0])
    yVals.append(points[i][1])

  maxX = max(xVals)
  minX = min(xVals)
  maxY = max(yVals)
  minY = min(yVals)

  centerX = (maxX + minX)/2
  centerY = (maxY + minY)/2

<<<<<<< HEAD
=======
  cv2.drawContours(src, [points], 0, (0, 0, 255), 1)

  cv2.imshow('contos', src);
  # cv2.waitKey(0);

>>>>>>> 5abfa242a8d21007cf9664906ca4c796bf6e11da
  angle = boxpoints[2]

<<<<<<< HEAD
  return (centerX, centerY), angle
=======

# img = cv2.imread('heights/rubix23.jpg')
# find_rubix(img, .23)
>>>>>>> 5abfa242a8d21007cf9664906ca4c796bf6e11da
