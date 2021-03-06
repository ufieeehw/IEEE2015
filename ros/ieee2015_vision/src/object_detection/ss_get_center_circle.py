import cv2
import numpy as np


def get_center_circle(img, points, height, draw):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #print 'and we made it here'
    ret, thresh3 = cv2.threshold(gray, 250, 255, cv2.THRESH_TRUNC)
    thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 23, 3)

    if draw is True:
        cv2.imshow('thresh', thresh)

    #seems to be good for circles
    circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 10, 200, 100, 100, 100, 50)
    #circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 100, 100, 50)

    if draw is True:
        cv2.imshow('circles', img)
 
    circle_distance = (-640 * height) + 225.969
    sigma = 15 #better value?

    good_circle = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            temppoint = (i[0], i[1])
            distance = cv2.pointPolygonTest(points, temppoint, True)
            # draw the outer circle
            if distance > circle_distance - sigma and distance < circle_distance + sigma:  ###########NEED TO CHANGE THAT
                if draw is True:
                    cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(img, (i[0], i[1]), i[2], (0, 0, 255), 10)
                    cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 10)
                good_circle.append(i[0])
                good_circle.append(i[1])

    if draw is True:
        cv2.imshow('img', img)

    return good_circle

#img = cv2.imread('ti/15hss.jpg')
#get_center_circle(img, 5)
