import cv2
import numpy as np


def find_card(img, height, draw):
    kernelg = np.ones((8, 8), np.uint8)
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 37, 15)
    gray = cv2.erode(gray, kernelg)

    if draw is True:
        cv2.imshow('adaptive thresh', gray)

    contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    approx_area = (-838900 * height) + 239926
    sigma = 20000
    low_bound = approx_area - sigma
    up_bound = approx_area + sigma
    #using grayscale thresh
    goodContours = []
    for current in contours:
        area = cv2.contourArea(current)
        if area > low_bound and area < up_bound:  # will need to be adjusted !!!!!
            if draw is True:
                print 'this is approx_area'
                print approx_area
                print 'this is area'
                print area
                cv2.drawContours(img, [current], 0, (0, 255, 0), 1)
            goodContours.append(current)

    if len(goodContours) == 0:
        return -1

    boxpoints = cv2.minAreaRect(goodContours[0])
    points = cv2.cv.BoxPoints(boxpoints)  # Find four vertices of rectangle from above rect
    points = np.int0(np.around(points))

    xVals = []
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

    if draw is True:
        cv2.drawContours(img, [points], 0, (0, 0, 255), 1)
        cv2.imshow('contos', img)

    angle = boxpoints[2]

    return (centerX, centerY), angle

#colorimage = cv2.imread('ti/25hc.jpg')
#find_card(colorimage, .255)
