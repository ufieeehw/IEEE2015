import cv2
import cv
import numpy as np
import ss_getLitUpButton
import ss_findColor
import ss_getStandardState

meanColsGreen = 0
meanRowsGreen = 0
minColsGreen = 0
maxColsGreen = 0
minRowsGreen = 0
maxRowsGreen = 0

#Blue button
meanColsBlue = 0
meanRowsBlue = 0
minColsBlue = 0
maxColsBlue = 0
minRowsBlue = 0
maxRowsBlue = 0

bestCnts = []
buttonCnts = []

img1 = cv2.imread('Images/Set4/ss1.JPG')

hsv_img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2HSV)
gray_img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

bestCnts, buttonCnts, meanColsBlue, meanRowsBlue, meanColsGreen, meanRowsGreen, minColsGreen, maxColsGreen, minRowsGreen, maxRowsGreen, minColsBlue, maxColsBlue, minRowsBlue, maxRowsBlue = ss_getStandardState.getStandardState(hsv_img1)

#simulating bright button
img2 = cv2.imread('Images/Set4/ss7.JPG')
hsv_img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
gray_img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
meanXPoints, meanYPoints, closing = ss_getLitUpButton.getLitUpButton(hsv_img2)

print 'hi'
color = ss_findColor.findColor(img1, meanXPoints, meanYPoints, meanColsBlue, meanRowsBlue, meanColsGreen, meanRowsGreen, minColsGreen, maxColsGreen, minRowsGreen, maxRowsGreen, minColsBlue, maxColsBlue, minRowsBlue, maxRowsBlue)

print color