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
    #gray = cv2.dilate(gray, kernelD)
    cv2.imshow('adaptive thresh', gray)
    cv2.waitKey(0)

    contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    approx_height = (-486526 * height) + 163108
    sigma = 3000
     #using grayscale thresh
    goodContours = []
    for current in contours:
        area = cv2.contourArea(current)
        if area > 30000: #will need to be adjusted !!!!!
            print 'this is area'
            print area
            goodContours.append(current)
            cv2.drawContours(img,[current],0,(0,255,0),1)

    #for cnt in goodContours:
        #cv2.drawContours(img,[cnt],0,(0,255,0),1)

    #cv2.imshow('contos', img);
    #cv2.waitKey(0);

      #squares = [] #goodContours[0] # create list of the tile squares
      #for c in goodContours:
        #squares.extend(c)
        #squares.append(cv2.minAreaRect(c)) #get a rectangle around the contour

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

    cv2.drawContours(img, [points], 0, (0, 0, 255), 1)

    cv2.imshow('contos', img);
    cv2.waitKey(0);

    angle = boxpoints[2]
    print angle
    return {'center':(centerX, centerY), 'angle': angle}

    #img = cv2.imread('heights/rubix23.jpg')
    #find_rubix(img)

colorimage = cv2.imread('ti/25hc.jpg')
find_card(colorimage, .255)