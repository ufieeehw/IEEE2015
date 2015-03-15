import cv2
import numpy as np

def getLitUpButton(img):
    
    #below ranges should work for all of the lights on ranges
    lower_on = np.array([26, 180, 215])
    upper_on = np.array([255, 255, 255])

    brightButton = cv2.inRange(img, lower_on, upper_on)

    brightButton = cv2.resize(brightButton, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('lit up button', brightButton)
    cv2.waitKey(0)

    print 'we got here'

    kernel = np.ones((3, 3), np.uint8)
    #kernel for eroding
    kernel2 = np.ones((12,12), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((10, 10), np.uint8)
    eroded = cv2.erode(brightButton, kernel2)
    dilated = cv2.dilate(eroded, kernel3)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=10)

    #to be used later
    #cont_img = closing.copy()

    ############################################Start of Contour Manipulation######################
    #get minimum contour
    contours, hierarchy = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #filtering contours to find the green and blue button
    bestCtn = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 8000:
            bestCtn.append(cnt)

    #bestCtn should ideally be length of one at this point

    #step in order to get all the pixel values of the button
    grayIMG = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(grayIMG.shape,np.uint8)
    cv2.drawContours(mask,[bestCtn[0]],0,255,-1)
    pixelPointsContour = np.transpose(np.nonzero(mask))
    
    #get x and y values for each button
    xPoints = []
    yPoints = []
    for points in pixelPointsContour:
        xPoints.append(points[0])
        yPoints.append(points[1])

    
    meanXPoints = int(np.mean(xPoints))
    meanYPoints = int(np.mean(yPoints))
    
   
    return meanXPoints, meanYPoints, closing
 