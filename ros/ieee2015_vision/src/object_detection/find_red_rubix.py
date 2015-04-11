import cv2
import numpy as np
import math


def find_red_rubix(img, draw):
    hsv_low = np.array([150, 100, 0], np.uint8)
    hsv_high = np.array([255, 255, 255], np.uint8)

    hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    image = cv2.inRange(hsvimg, hsv_low, hsv_high)
    kernel = np.ones((2, 2), np.uint8)
    kernel2 = np.ones((12, 12), np.uint8)
    image = cv2.erode(image, kernel)
    image = cv2.dilate(image, kernel2)

    contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return -1

    center_square = contours[0]

    boxpoints = cv2.minAreaRect(center_square)
    points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
    points = np.int0(np.around(points))
    cv2.drawContours(img,[points],0,(0,0,255),2)

    #print 'this is diameter', boxpoints[1][0]
    if boxpoints[1][0] < 100 and boxpoints[1][1] < 100:
        return -1

    center = boxpoints[0]
    cv2.circle(img, (int(center[0]), int(center[1])), 2, (0, 0, 255), 3)
    angle = boxpoints[2]
    radians = math.radians(angle) 
    
    #if draw is True:
    #    cv2.imshow('img', img)
        #cv2.waitKey(0)

    return center, radians

#img = cv2.imread('greenRed.jpg')
#res = cv2.resize(img, None,fx=.3, fy=.3)
#find_red_rubix(res, True)
