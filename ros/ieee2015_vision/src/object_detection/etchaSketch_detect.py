import cv2
import numpy as np
import cv2.cv as cv

def etchaSketch_detect(img):
    ###FOLLOWING IS FOR GRAYSCALE ATTEMPT###= 
    kernelg = np.ones((4,4), np.uint8)
    #seems to be extremely less effective, just grouping old code together
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(grayscale)
    ret,thresh3 = cv2.threshold(equ,247,255,cv2.THRESH_TRUNC)
    ret,thresh4 = cv2.threshold(thresh3,130,255,cv2.THRESH_TOZERO)
    cv2.imshow('thresh3', thresh4)
    cv2.waitKey(0)
    gray = cv2.adaptiveThreshold(thresh3,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,47,13)
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

    # The following code actually calculates center points and draws contours for testing
    # The first coordinate

    ##################USE DISTANCE FROM SQUARE TO DETERMINE A GOOD CIRCLE
    #keep 1, 2, 2nd to last, last)
    #4th parameter seems to be very important
    circles = cv2.HoughCircles(thresh4, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, 100, 520, 10, 5)
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


img = cv2.imread('heights/22cmeas.jpg')
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