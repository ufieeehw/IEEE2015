import cv2
import numpy as np
import ss_get_center_circle


#read in the image, resize is just so that I can see on screen
def get_axis_points(img, height, draw):
    #gray color needed for threshold
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 37, 8)

    #anytime we show an image copy it then show it
    if draw is True:
        fakeThresh = thresh.copy()
        cv2.imshow('using adaptiveThreshold', fakeThresh)

    #flip black and white pixels
    thresh = (255 - thresh)

    #kernel for eroding
    kernel = np.ones((5, 5), np.uint8)
    #kernel for dilating
    kernel2 = np.ones((8, 5), np.uint8)
    thresh = cv2.dilate(thresh, kernel2)
    thresh = cv2.erode(thresh, kernel)

    #contours, hierarchy = cv2.findContours(thresh, 2, 1)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    approx_area = (-649593 * height) + 180070
    sigma = 14000

    #finding contour for the toy
    wholeToy = []
    for bae2 in contours:
        area = cv2.contourArea(bae2)
        if area > (approx_area - sigma) and area < (approx_area + sigma):
            if draw is True:
                cv2.drawContours(img, [bae2], 0, (0, 255, 0), 10)
            wholeToy.append(bae2)

    #test img
    if draw is True:
        cv2.imshow('contours', img)

    #error check to exit
    if len(wholeToy) == 0:
        return -1

    #first element should be only one and be the toy contour
    toyCnt = wholeToy[0]

    #gives us the angle of rotation and minor and jmajor axis lengths
    (x, y), (MA, ma), angle = cv2.fitEllipse(toyCnt)
    boxpoints = cv2.fitEllipse(toyCnt)
    points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
    points = np.int0(np.around(points))

    if draw is True:
        cv2.drawContours(img, [points], 0, (0, 0, 255), 1)
        cv2.imshow('detected box', img)
    #cv2.waitKey(0)

    good_circle = ss_get_center_circle.get_center_circle(img, points,height, True)
    #PLOTS THE 4 CORNER POINTS OF THE RECTANGLE
    #Testing
    if draw is True:
        cv2.circle(img, (points[0][0], points[0][1]), 2, (0, 0, 255), 10)
        cv2.circle(img, (points[1][0], points[1][1]), 2, (0, 0, 255), 10)
        cv2.circle(img, (points[2][0], points[2][1]), 2, (0, 0, 255), 10)
        cv2.circle(img, (points[3][0], points[3][1]), 2, (0, 0, 255), 10)

    x = int(x)
    y = int(y)

    ma = int(ma)
    MA = int(MA)

    #angle = angle * -1
    if draw is True:
        cv2.circle(img, (good_circle[0], good_circle[1]), 2, (0, 0, 255), 30)
        imgf = img.copy()
        #imgf = cv2.resize(imgf, (0, 0), fx=0.2, fy=0.2)
        cv2.imshow('venterpoint', imgf)

    #gives us the dimensions so that we can form rotation matrix
    rows, cols, pages = img.shape

    #angle = angle + 100
    #positive angle goes counterclockwise
    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
    dst = cv2.warpAffine(img, M, (cols, rows))

    if draw is True:
        cv2.imshow('rotated image', dst)

    if draw is True:
        cv2.imshow('finla img', img)
        cv2.waitKey(0)

    return angle, points, good_circle

img = cv2.imread('ti/17hss.jpg')
get_axis_points(img, .17, True)
