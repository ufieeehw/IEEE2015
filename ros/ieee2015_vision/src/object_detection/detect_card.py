import cv2
import numpy as np

def find_card(img, height):
    kernelg = np.ones((8,8), np.uint8)
    kernelD = np.ones((2, 2), np.uint8)
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #equ = cv2.equalizeHist(grayscale)
    gray = cv2.adaptiveThreshold(grayscale,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,37,15)
    gray = cv2.erode(gray, kernelg)

    contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    approx_height = (-838900 * height) + 239926
    sigma = 3000
     #using grayscale thresh
    goodContours = []
    for current in contours:
        area = cv2.contourArea(current)
        if area > (approx_height - sigma) and area < (approx_height + sigma): #will need to be adjusted !!!!!
            goodContours.append(current)

    boxpoints = cv2.minAreaRect(goodContours[0])
             # rect = ((center_x,center_y),(width,height),angle)
    points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
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

    angle = boxpoints[2]
    
    return (centerX, centerY), angle

