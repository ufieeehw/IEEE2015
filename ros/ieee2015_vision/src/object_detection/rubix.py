# algorithm to detect the rubix cube
import numpy as np
import cv2


#give the function a filename as an argument
#returns the center and angle (point and float) or none
def find_rubix(src, height, draw):
    kernelg = np.ones((8, 8), np.uint8)
    grayscale = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    gray = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 37, 15)
    gray = cv2.erode(gray, kernelg)

    if draw is True:
        cv2.imshow('adaptive thresh', gray)

    contours2, hierarchy2 = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    #using height for reference
    approx_area = (-955284 * height) + 259205
    sigma = 30000
    #using 6000 as standard dev
    #using grayscale thresh
    goodContours2 = []
    for current in contours2:
        area = cv2.contourArea(current)
        if area > approx_area - sigma and area < approx_area + 30000:  # will need to be adjusted !!!!!
            goodContours2.append(current)
            if draw is True:
                print 'this is area'
                print area

    if draw is True:
        for cnt in goodContours2:
            cv2.drawContours(src, [cnt], 0, (0, 255, 0), 1)
        cv2.imshow('contos', src)

    boxpoints = cv2.minAreaRect(goodContours2[0])
               # rect = ((center_x,center_y),(width,height),angle)
    points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
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
        cv2.drawContours(src, [points], 0, (0, 0, 255), 1)
        cv2.imshow('contos', src)

    angle = boxpoints[2]

    return (centerX, centerY), angle

    #img = cv2.imread('ti/18.5r.jpg')
    #find_rubix(img, .185)
