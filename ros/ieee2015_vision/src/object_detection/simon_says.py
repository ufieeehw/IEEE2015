import cv2
import ss_getStandardState
import ss_findColor
import ss_getLitUpButton
import ss_getColorButtonCoord
import ss_getStartingButtonCoord
import ss_findButton
import ss_getAxisPoints

######################The following are fields for Simon
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
#the standard state is the base image we go off of between each button push or between player moves (TBD). It gets
#the location of the green and blue button and from their create a basis of where all the buttons are.


def setStandard(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    bestCnts, buttonCnts, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR = ss_getStandardState.getStandardState(hsv_img)
    topPoint, bottomPoint = ss_getStartingButtonCoord.getStartingButtonCoord(buttonCnts)
    return bestCnts, buttonCnts, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR, topPoint, bottomPoint


#the main method to get everything running. from the coordinates to push to indentifying what color lit up.
def playSimonSays(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #while(len(colorsPlayed) < 5): #hard coded in up to five colors, but we are being timed on this one
    brightCenterPoint, closing = ss_getLitUpButton.getLitUpButton(hsv_img)

    meanBrightCols = brightCenterPoint[0]
    meanBrightRows = brightCenterPoint[1]
    #color is a number
    #color = findColor.findColor(meanBrightCols, meanBrightRows, meanXR, meanYR, meanXL, meanYL, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR)

    color = ss_findButton.findButton(img, brightCenterPoint)

    colorsPlayed.append(color)

    xPush, yPush = ss_getColorButtonCoord.getColorButtonCoord(colorsPlayed, minXL, maxXL, minYL, maxYL, minXR, maxXR, minYR, maxYR)

    coordToAdd = [(xPush, yPush)]
    pushArray.append(coordToAdd)

    print "the color was"
    print color
