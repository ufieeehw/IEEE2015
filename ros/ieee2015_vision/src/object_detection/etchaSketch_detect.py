import cv2
import numpy as np
import cv2.cv as cv


#return value of function is cx_coord and cy_coord containing list of x and y coord
#of center points of knobs
def etchaSketch_detect(img, height, draw):
    ###FOLLOWING IS FOR GRAYSCALE ATTEMPT###= 
    kernelg = np.ones((4,4), np.uint8)
    #seems to be extremely less effective, just grouping old code together
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(grayscale)
    ret,thresh3 = cv2.threshold(equ, 247, 255, cv2.THRESH_TRUNC)
    #ret,thresh3 = cv2.threshold(thresh3,247,255,cv2.THRESH_TRUNC)
    ret,thresh4 = cv2.threshold(thresh3, 80, 255, cv2.THRESH_TOZERO)
     
    if draw == true:  
        cv2.imshow('thresh3', thresh3)
        cv2.imshow('mask', thresh4)

   
    #this is pretty
    gray = cv2.adaptiveThreshold(grayscale,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,33,12)
    #cv2.THRESH_BINARY,33,15)
    kernel = np.ones((8, 8), np.uint8)
    gray = cv2.erode(gray, kernel)

    if draw == true:
        cv2.imshow('gray', gray)
    ##################USE DISTANCE FROM SQUARE TO DETERMINE A GOOD CIRCLE
    #keep 1, 2, 2nd to last, last)
    #4th parameter seems to be very important
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    approx_area = (-800112 * height) + 243989
    sigma = 6000

    if draw == true
        print 'this is len of contr'
        print len(contours)
        print 'this is approx_area'
        print approx_area

    eas = []
    for current in contours:
        area = cv2.contourArea(current)
        if draw == true:
            print area
        if area > (approx_area - sigma) and area < (approx_area + sigma): # and perim < 2302:
            if draw == true:
                cv2.drawContours(img, [current], 0, (0, 255, 0), 10)
            #if we get a sm- aller value in the range we give it
            #we want the contour with the smaller perimeter
            #may swtich it to be larger area to be sure
            #testing will tell
            eas.insert(0, current)
          #  print 'this is good perim'
          #  print perim

    if draw == true:
        cv2.imshow('img',img)
    #gives us bounding rectangle to reference for points in and out
    #if problems arise we can use distances from contour
    boxpoints = cv2.minAreaRect(eas[0])
    points = cv2.cv.BoxPoints(boxpoints)         
    points = np.int0(np.around(points)) 

    if draw == true:
        cv2.imshow('img',img)
    circles = cv2.HoughCircles(thresh4, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 550, 10, 5)
    buttons = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
        # draw the outer circle
            temppoint = (i[0], i[1])
            tempans = cv2.pointPolygonTest(points, temppoint, False)
            if tempans >= 0:# and i[2] == 38 and i[2] == 39:
                if draw == true:
                    print 'this is good cricle'
                    print i[2]
                    cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # draw the center of the circle
                    cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
                buttons.append(temppoint)

    x, y = buttons[0]
    x2, y2 = buttons[1]

    cx_coord = [x, x2]
    cy_coord = [y, y2]

    angle = boxpoints[2]    
    ####START DISPLAY METHODS####
    #small = cv2.resize(image, (300, 250))
    if draw == true:
        cv2.imshow('detected circles', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return cx_coord, cy_coord, angle

#img = cv2.imread('ti/21he.jpg')
#etchaSketch_detect(img, .215)
