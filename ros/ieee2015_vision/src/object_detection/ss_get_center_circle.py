import cv2
import numpy as np
import ss_get_axis_points


def get_center_circle(img, points):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret,thresh3 = cv2.threshold(gray,250,255,cv2.THRESH_TRUNC)
    thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                    cv2.THRESH_BINARY, 23, 3)

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
                goodcircle.append(i[0])
                goodcircle.append(i[1])
    else:
        print "You screwed up"

    return goodcircle


