import cv2
import numpy as np
import ss_get_mid_quarter_points
import ss_get_center_circle


#read in the image, resize is just so that I can see on screen
def get_axis_points(img, height):
    #gray color needed for threshold
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                    cv2.THRESH_BINARY, 37, 8)

    #anytime we show an image copy it then show it
    #fakeThresh = thresh.copy()
    #cv2.imshow('using adaptiveThreshold', fakeThresh)
    #cv2.waitKey(0)

    #flip black and white pixels
    thresh = (255 - thresh)

    #kernel for eroding
    kernel = np.ones((5, 5), np.uint8)
    #kernel for dilating
    kernel2 = np.ones((8, 5), np.uint8)
    thresh = cv2.dilate(thresh, kernel2)
    thresh = cv2.erode(thresh, kernel)

    #fakedilated = thresh.copy()
    #cv2.imshow('after dilating and eroding', fakedilated)
    #cv2.waitKey(0)

    #contours, hierarchy = cv2.findContours(thresh, 2, 1)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print 'this is length of contours'
    print len(contours)

    approx_area = (-541758 * height) + 160220
    sig = 6000
    #finding contour for the toy
    wholeToy = []
    for bae2 in contours:
        area = cv2.contourArea(bae2)
        if area > (approx_area - sig) and area < (approx_area + sig):
            print 'this is area'
            print area
            #cv2.drawContours(img, [bae2], 0, (0, 255, 0), 10)
            wholeToy.append(bae2)

    #test img
    tempimg = img.copy()
    #tempimg = cv2.resize(tempimg, (0, 0), fx=0.2, fy=0.2)
    cv2.imshow('contours', tempimg)
    cv2.waitKey(0)

    #first element should be only one and be the toy contour
    toyCnt = wholeToy[0]

    #gives us the angle of rotation and minor and jmajor axis lengths
    (x, y), (MA, ma), angle = cv2.minAreaRect(toyCnt)
    boxpoints = cv2.minAreaRect(toyCnt)
    points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
    points = np.int0(np.around(points))

    cv2.drawContours(img, [points], 0, (0, 0, 255), 1)
    #cv2.imshow('detected box', img)
    #cv2.waitKey(0)

    goodcircle = ss_get_center_circle.get_center_circle(img, points)
    #PLOTS THE 4 CORNER POINTS OF THE RECTANGLE
    #Testing
    cv2.circle(img, (points[0][0], points[0][1]), 2, (0, 0, 255), 10)
    cv2.circle(img, (points[1][0], points[1][1]), 2, (0, 0, 255), 10)
    cv2.circle(img, (points[2][0], points[2][1]), 2, (0, 0, 255), 10)
    cv2.circle(img, (points[3][0], points[3][1]), 2, (0, 0, 255), 10)

    x = int(x)
    y = int(y)

    ma = int(ma)
    MA = int(MA)

    #angle = angle * -1
    cv2.circle(img, (goodcircle[0], goodcircle[1]), 2, (0, 0, 255), 30)

    imgf = img.copy()
    #imgf = cv2.resize(imgf, (0, 0), fx=0.2, fy=0.2)
    cv2.imshow('venterpoint', imgf)
    cv2.waitKey(0)

    #gives us the dimensions so that we can form rotation matrix
    rows, cols, pages = img.shape

    #angle = angle + 100
    #positive angle goes counterclockwise
    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
    dst = cv2.warpAffine(img, M, (cols, rows))

    imgd = dst.copy()
    #imgd = cv2.resize(imgd, (0, 0), fx=0.2, fy=0.2)
    cv2.imshow('rotated image', dst)
    cv2.waitKey(0)

    #minAxisMid, majAxisMid, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2 = ss_get_mid_quarter_points.get_mid_quarter_points(points, ma, MA)

    ###Testing to display detected center points
    #for a in minAxisMid:
    #    cv2.circle(img, a, 2, (255, 255, 255), 10)
    #for b in majAxisMid:
    ##    cv2.circle(img, b, 2, (255, 255, 255), 10)
    #for c in quarterMin1:
    #    cv2.circle(img, c, 2, (255, 255, 255), 10)
    #for d in quarterMin2:
    #    cv2.circle(img, d, 2, (255, 255, 255), 10)
    ##for e in quarterMaj1:
     #   cv2.circle(img, e, 2, (255, 255, 255), 10)
    #for f in quarterMaj2:
    #    cv2.circle(img, f, 2, (255, 255, 255), 10)

    ###Points in which to decide line for major and minor axis lines
    #p1CentMinor = minAxisMid[0]
    #p2CentMinor = minAxisMid[2]

    #p1CentMajor = majAxisMid[0]
    #p2CentMajor = majAxisMid[2]

    #cv2.line(img, p1CentMajor, p2CentMajor, (0, 255, 0), 5)
    #cv2.line(img, p1CentMinor, p2CentMinor, (0, 0, 0), 5)

    #imgg = img.copy()
    #imgg = cv2.resize(imgg, (0, 0), fx=0.2, fy=0.2)
    cv2.imshow('finla img', img)
    cv2.waitKey(0)

    return angle, points, goodcircle#x, y, p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2
