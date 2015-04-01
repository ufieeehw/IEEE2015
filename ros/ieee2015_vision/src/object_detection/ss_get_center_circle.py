import cv2
import numpy as np
import ss_get_axis_points


def get_center_circle(img, points):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret,thresh3 = cv2.threshold(gray,250,255,cv2.THRESH_TRUNC)
    thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                    cv2.THRESH_BINARY, 23, 3)

    cv2.imshow('img', thresh)
    cv2.waitKey(0)

    #seems to be good for circles
    circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 100, 100, 50)
    #circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 100, 100, 50)
    goodcircle = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            temppoint = (i[0], i[1])
            tempans = cv2.pointPolygonTest(points, temppoint, False)
            # draw the outer circle
            if tempans >= 0:
                cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
                goodcircle.append(i[0])
                goodcircle.append(i[1])
                goodcircle.append(i[2])
    else:
        print "You screwed up"


    print goodcircle
    thresh = 255 - thresh

    kernel = np.ones((2, 3), np.uint8)
        #kernel for dilating
    kernel2 = np.ones((3, 5), np.uint8)
    #thresh = cv2.dilate(thresh, kernel2)
    thresh = cv2.erode(thresh, kernel)
    cv2.imshow('thresh', img)
    cv2.waitKey(0)

    return goodcircle

#img = cv2.imread('heights/23cmss.jpg')
#returns = ss_get_axis_points.get_axis_points(img, .23)
##points = returns[1]
#get_center_circle(img, points)
