import cv2
import numpy as np
import cv2.cv as cv

def etchaSketch(img):

    hsv_etchasketch = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #hsv_etchasketch = cv2.resize(hsv_etchasketch, (300,250))

    lower_white = np.array([70, 0, 170], np.uint8)  #estimated values until testing
    upper_white = np.array([179, 20, 255], np.uint8) #estimated values until testing
    # Searching the HSV image to extract the white colors

    binary_img = cv2.inRange(hsv_etchasketch, lower_white, upper_white)

    kernel = np.ones((3, 3))
    #kernel for eroding
    kernel2 = np.ones((8,8), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((12, 12), np.uint8)
    eroded = cv2.erode(binary_img, kernel2)
    dilated = cv2.dilate(eroded, kernel3)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=10)


    cv2.imshow('inrange small bin', closing)
    cv2.waitKey(0)
    cv2.destroyAllWindows


    ###FOLLOWING IS FOR GRAYSCALE ATTEMPT###= 
    kernelg = np.ones((4,4), np.uint8)
    #seems to be extremely less effective, just grouping old code together
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(grayscale)
    gray = cv2.adaptiveThreshold(equ,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,33,2)
    gray = cv2.erode(gray, kernelg)
    gray = cv2.dilate(gray, kernelg)
    cv2.imshow('adaptive thresh', gray)
    cv2.waitKey(0)
    #grayscale = cv2.resize(equ, (300, 250))
    #ret,thresh = cv2.threshold(grayscale,175,255,0)
    #cv2.imshow('binary', equ)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows
    #cv2.imshow('binary', thresh)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows
    #contours_k, hierarchy_k = cv2.findContours(thresh, 1, 2)

    #use distance as filter 
    #user area
    contours_knobs, hierarchy_knobs = cv2.findContours(binary_img, 1, 2)

    cnt = contours_knobs[6] #Detected contours. Each contour is stored as a vector of points.
    print cnt

    print type(cnt)

    bestCtn = []
    for bae in contours_knobs:
        area = cv2.contourArea(bae)
        print area
        if area > 8000 and area < 20000:
            bestCtn.append(bae)

    #should be the two buttons
    bae1 = bestCtn[0]
    bae2 = bestCtn[1]

    # The following code actually calculates center points and draws contours for testing
    # The first coordinate
    M = cv2.moments(bae1)

    cx_coord= int(M['m10']/M['m00'])
    cy_coord = int(M['m01']/M['m00'])

    cv2.circle(img, (cx_coord, cy_coord), 50, (100, 255, 0), thickness=1, lineType=8, shift=0)
    cv2.circle(img,(cx_coord, cy_coord),2,(0,0,255),3)
    cv2.drawContours(img,[bae1],0,(0,255,0),1)
    
    # The second coordinate
    M2 = cv2.moments(bae2)

    cx_coord2 = int(M2['m10']/M2['m00'])
    cy_coord2 = int(M2['m01']/M2['m00'])

    cv2.circle(img, (cx_coord2, cy_coord2), 50, (100, 255, 0), thickness=1, lineType=8, shift=0)
    cv2.circle(img,(cx_coord2, cy_coord2),2,(0,0,255),3)
    cv2.drawContours(img,[bae2],0,(255,0,0),1)

    #keep 1, 2, 2nd to last, last)
    #4th parameter seems to be very important
    circles = cv2.HoughCircles(equ, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 1, 520, 10, 5)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
        # draw the outer circle
            cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
    else:
        print "You screwed up"

    ####START DISPLAY METHODS####
    #small = cv2.resize(image, (300, 250))
    cv2.imshow('detected circles', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    return cx_coord, cy_coord


img = cv2.imread('Images/etch2.jpg')
etchaSketch(img)













'''

threshold_img = cv2.adaptiveThreshold(grayscale,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)

height, width = threshold_img.shape







new_thresh = cv2.resize(threshold_img, (300, 250))

cv2.imshow('hello', new_thresh)
cv2.waitKey(0)
cv2.destroyAllWindows
#extracted_white = cv2.inRange(hsveye, lower_white, upper_white)

kernel = np.ones((3,3), np.uint8) #square of 2x2 pixels to use for dilating
kernel2 = np.ones((2,2), np.uint8) #square of 1x1 pixels to use for eroding
eroded_image = cv2.erode(threshold_img,kernel2,iterations = 1)
cv2.imshow("cleaned up image", eroded_image)
cv2.waitKey(0)

dilated_image = cv2.dilate(eroded_image, kernel, iterations = 1)
cv2.imshow("dilated image", dilated_image)
cv2.waitKey(0)

#cv2.imshow("dilated course", course_image_binary)
#cv2.waitKey(0)
#cv2.destroyAllWindows
#finds countours of threshold image 

contours,h = cv2.findContours(threshold_img,1,2)
cnt = contours[0]



ellipse = cv2.fitEllipse(cnt)
cv2.ellipse(image,ellipse,(0,255,0),2)
cv2.imshow = ('ellipse', ellipse)
cv2.waitKey(0)
cv2.destroyAllWindows

#Goes through the contours image and draws green pixels where it finds rectangles
#on top of the original color image

#rectanglecounter = 0
for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    
    if len(approx)==4:
        cv2.drawContours(image,[cnt],0,(0,255,0),-1)





#Creates an image called threshold, which is the threshold of the grayscale image

#finds countours of threshold image 

'''