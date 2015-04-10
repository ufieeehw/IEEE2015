import numpy as np
import cv2

img = cv2.imread('closeTest/closess.jpg', 0)
regimg = cv2.imread('closeTest/closess.jpg')

ret, thresh3 = cv2.threshold(img, 250, 255, cv2.THRESH_TRUNC)
thresh = cv2.adaptiveThreshold(thresh3, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 23, 3)

circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 10, 200, 100, 100, 100, 50)
    #circles = cv2.HoughCircles(thresh3, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 100, 100, 50)
print 'the length of circles is'
print len(circles)
circle = circles[0]
cv2.circle(regimg, (circle[0][0], circle[0][1]), 0, (0, 255, 0), 20)

cv2.imshow('circles', regimg)
cv2.waitKey(0)



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


cv2.imshow('img', img)



#img = cv2.imread('ti/15hss.jpg')
#get_center_circle(img, 5)

cv2.imshow('thresh', thresh)
cv2.waitKey(0)