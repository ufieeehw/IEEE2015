import cv
import cv2
import numpy as np
import getButtonPoints
import getStandardState
import findColor
import getLitUpButton

#this list holds the colors in order
#different integers represent the different numbers
#1 is yellow
#2 is green
#3 is red
#4 is blue
#-1 error as always
colors = []


#Green button
meanXPointsL = 0
meanYPointsL = 0
minXPointL = 0
maxXPointL = 0
minYPointL = 0
maxYPointL = 0

#Blue button
meanXPointsR = 0
meanYPointsR = 0
minXPointR = 0
maxXPointR = 0
minYPointR = 0
maxYPointR = 0



def playSimonSays(img):
     hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

     #first step it to get standard state
     contours, meanXPointsR, meanYPointsR, meanXPointsL, meanYPointsL, minXPointL, 
     maxXPointL, minYPointL, maxYPointL, minXPointR, maxXPointR, minYPointR, maxYPointR = getStandardState(hsv_img)

     topPoint, bottomPoint = getButtonPoints(contours)

     while(len(colors) < 5)
     	meanBrightX, meanBrightY, closing = getLitUpButton(hsv_img)
     	#color is a number
     	color = findColor(meanBrightX, meanBrightY, meanXPointsL, meanYPointsL, meanXPointsR, meanYPointsR, 
     		minXPointL, maxXPointL, minYPointL, maxYPointL, 
     		minXPointR, maxXPointR, minYPointR, maxYPointR)
#input for this is the mean point value for the green, blue, and lit up space
#this is to check and see what color is lit up
#last input should be the BINARY <-----
#1 is yellow
#2 is green
#3 is red
#4 is nothing





####HEY SLEEP TEST OUT THAT THE COLORS READ IN RIGHT!#############3
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = makeHSV(newimg)
getStandardState(newimg)
getLitUpButton(newimg)
colors = []


    
