import cv2
import numpy as np
import ss_get_center_circle


#read in the image, resize is just so that I can see on screen
def get_axis_points(img, height):
    #gray color needed for threshold
    img2 = img.copy()

    gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                    cv2.THRESH_BINARY, 37, 8)

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
    sigma = 10000
    #finding contour for the toy
    wholeToy = []
    for bae2 in contours:
        area = cv2.contourArea(bae2)
        #if area > (approx_area - sig) and area < (approx_area + sig):
        if area > (approx_area - sigma) and area < (approx_area + sigma):
            wholeToy.append(bae2)
            #cv2.drawContours(img2, [bae2], 0, (0, 255, 0), 10)
    #first element should be only one and be the toy contour
    toyCnt = wholeToy[0]

    #gives us the angle of rotation and minor and jmajor axis lengths
    (x, y), (MA, ma), angle = cv2.minAreaRect(toyCnt)
    boxpoints = cv2.minAreaRect(toyCnt)
    points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
    points = np.int0(np.around(points))

    goodcircle = ss_get_center_circle_test.get_center_circle(img, points)

    x = int(x)
    y = int(y)

    ma = int(ma)
    MA = int(MA)

    #angle = angle * -1
    #cv2.circle(img2, (goodcircle[0], goodcircle[1]), 2, (0, 0, 255), 30)


    #gives us the dimensions so that we can form rotation matrix
    #rows, cols, pages = img2.shape
    #positive angle goes counterclockwise
    #to get rotated image
    #M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
    #dst = cv2.warpAffine(img2, M, (cols, rows))

    return angle, points, goodcircle#x, y, p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2

#img = cv2.imread('test_functions/ti/25hss.jpg')
#get_axis_points(img, .255)