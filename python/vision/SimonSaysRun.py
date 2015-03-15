import cv
import cv2
import numpy as np
import getButtonPoints
import getStandardState
import findColor
import getLitUpButton
import getColorButtonCoord
import getStartingButtonCoord

#this list holds the colors in order
#different integers represent the different numbers
#1 is yellow
#2 is green
#3 is red
#4 is blue
#-1 error as always
colorsPlayed = []
pushArray = []

#Green button
meanXL = 0
meanYL = 0
minXL = 0
maxXL = 0
minYL = 0
maxYL = 0

#Blue button
meanXR = 0
meanYR = 0
minXR = 0
maxXR = 0
minYR = 0
maxYR = 0

contours = []

#first step it to get standard state
def setStandard(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    bestCnts, buttonCnts, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR = getStandardState.getStandardState(hsv_img)
    topPoint, bottomPoint = getStartingButtonCoord.getStartingButtonCoord(buttonCnts)
    return bestCnts, buttonCnts, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR, topPoint, bottomPoint

def playSimonSays(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    print 'we made it here'
    while(len(colorsPlayed) < 5):
        meanBrightX, meanBrightY, closing = getLitUpButton.getLitUpButton(hsv_img)
        #color is a number
        color = findColor.findColor(meanBrightX, meanBrightY, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR)

        colorsPlayed.append(color)

        xPush, yPush = getColorButtonCoord.getColorButtonCoord(colorsPlayed, minXL, maxXL, minYL, maxYL, 
            minXR, maxXR, minYR, maxYR)

        coordToAdd = [(xPush, yPush)]
        pushArray.append(coordToAdd)
    print(pushArray)
    print(len(colorsPlayed))


####HEY SLEEP TEST OUT THAT THE COLORS READ IN RIGHT!#############3
newimg = cv2.imread('Images/Set2/sbright2.JPG')

newimg2 = cv2.imread('Images/Set2/sbright5.JPG')
'''
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
'''
#newimg = makeHSV(newimg)
bestCnts, buttonCnts, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR, topPoint, bottomPoint = setStandard(newimg2)
playSimonSays(newimg)