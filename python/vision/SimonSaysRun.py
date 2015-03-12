import cv
import cv2
import numpy as np

def getButtonPoints(contours):
    buttonParts = []
    for cnt in contours:
        if (area > 200 and area < 4500):
            buttonParts.append(cnt)
    
    #get the individual button identitites
    bottomB = buttonParts[0]
    topB = buttonParts[1]

    #get topmost point of top button and bottommost point of bottom button 
    #these two will be points we aim to press, or in between them can be arranged
    topmost = tuple(topB[topB[:,:,1].argmin()][0])
    bottommost = tuple(bottomB[bottomB[:,:,1].argmax()][0])

    return topmost, bottommost

def getStandardState(img):
    '''
    finds blue and green color because easest to distinguish.
    from those two colors we find ranges for all four colors.
    this method also find the button initially needed to push.
    '''
    #changing color range
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #creating binary image
    lowerRange = np.array([64, 115, 39])
    upperRange = np.array([118, 255, 255])
    filteredIMG = cv2.inRange(hsv, lowerRange, upperRange)

    #morphological stuff, std
    kernel = np.ones((3, 3), np.uint8)
    #kernel for eroding
    kernel2 = np.ones((12,12), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((10, 10), np.uint8)
    eroded = cv2.erode(filteredIMG, kernel2)
    dilated = cv2.dilate(eroded, kernel3)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=10)

    #to be used later
    cont_img = closing.copy()

    ############################################Start of Contour Manipulation######################
    #get minimum contour
    contours, hierarchy = cv2.findContours(cont_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #get all the contour points
    contoursAll, hierarchyAll = cv2.findContours(cont_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #filtering contours to find the green and blue button
    bestCtn = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 8000:
            bestCtn.append(cnt)

    #testing        
    closing = cv2.resize(closing, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('filter', closing)
    cv2.waitKey(0);

    #distinguishing the buttons from the contours
    greenButton = bestCtn[0]
    blueButton = bestCtn[1]

    #step in order to get all the pixel values of the button
    grayIMG = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(grayIMG.shape,np.uint8)
    cv2.drawContours(mask,[greenButton],0,255,-1)
    pixelpointsL = np.transpose(np.nonzero(mask))
    cv2.drawContours(mask,[blueButton],0,255,-1)
    pixelpointsR = np.transpose(np.nonzero(mask))
    print pixelpointsL
    print pixelpointsR

 
    #gets min and max values in all cardinal directions for both buttons
    leftmostL = tuple(greenButton[greenButton[:,:,0].argmin()][0])
    rightmostL = tuple(greenButton[greenButton[:,:,0].argmax()][0])
    topmostL = tuple(greenButton[greenButton[:,:,1].argmin()][0])
    bottommostL = tuple(greenButton[greenButton[:,:,1].argmax()][0])

    leftmostR = tuple(blueButton[blueButton[:,:,0].argmin()][0])
    rightmostR = tuple(blueButton[blueButton[:,:,0].argmax()][0])
    topmostR = tuple(blueButton[blueButton[:,:,1].argmin()][0])
    bottommostR = tuple(blueButton[blueButton[:,:,1].argmax()][0])

    #testing
    cv2.drawContours(img, [greenButton], 0, (0,255,0), 3)
    
    #fit an ellipse to both blue and green button
    #ideally the one with the bigger area is going to be the
    #one that was segmented the best so we want to use that area
    ellipseG = cv2.fitEllipse(greenButton)
    ellipseB = cv2.fitEllipse(blueButton)
    #ellipseAverage = ellipseG
    print ellipseG

    areaG = cv2.contourArea(ellipseG)
    areaB = cv2.contourArea(ellipseB)

    ellipseBest = []

    if(areaG > areaB):
        ellipseBest = areaG
    elif areaB > areaG:
        ellipseBest = areaB

    #testing  
    cv2.ellipse(img,ellipse,(0,255,0),2)
    img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('ellipse', img)
    cv2.waitKey(0)

    print ellipseBest


def SSMoves(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #below ranges should work for all of the lights on ranges
    lower_on = np.array([0, 0, 245])
    upper_on = np.array([255, 255, 255])


    lower_all = np.array([0, 140, 60])
    upper_all = np.array([255, 255, 225])
    
    #below ranges are the ranges for the colors just plain
    lower_yellow = np.array([32, 125, 122])
    upper_yellow = np.array([60, 255, 151])

    lower_blue = np.array([90, 44, 85])
    upper_blue= np.array([120, 255, 255])

    lower_green = np.array([60, 29, 95])
    upper_green = np.array([85, 255, 255])

    lower_red = np.array([0, 185, 58])
    upper_red = np.array([20, 255, 255])

    
    filter_imgR = cv2.inRange(hsv_img, lower_red, upper_red) #filter all weak colors
   #cv2.imshow('red', filter_imgR)
    #cv2.waitKey(0);
    filter_imgY = cv2.inRange(hsv_img, lower_yellow, upper_yellow) #filter all weak colors
    #cv2.imshow('yellow', filter_imgY)
    #cv2.waitKey(0);
    filter_imgG = cv2.inRange(hsv_img, lower_green, upper_green) #filter all weak colors
    #cv2.imshow('green', filter_imgG)
    #cv2.waitKey(0);
    filter_imgB = cv2.inRange(hsv_img, lower_blue, upper_blue)
    #cv2.imshow('blue', filter_imgB)
    #cv2.waitKey(0);
    '''
    allcolors = cv2.inRange(hsv_img, lower_all, upper_all)
    smallall = cv2.resize(allcolors, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('all', smallall)
    cv2.waitKey(0);

    bright_spot = cv2.inRange(hsv_img, lower_on, upper_on)
    smallbright = cv2.resize(allcolors, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('bright', smallbright)
    cv2.waitKey(0);
'''
    filter_img = cv2.bitwise_or(filter_imgR, filter_imgY)
    filter_img = cv2.bitwise_or(filter_img, filter_imgG)
    filter_img = cv2.bitwise_or(filter_img, filter_imgB)

    filter_img = cv2.resize(filter_img, (0,0), fx=0.5, fy=0.5) 

    #kernel for closing
    kernel = np.ones((3, 3), np.uint8)
    #kernel for eroding
    kernel2 = np.ones((8,8), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((6, 6), np.uint8)
    
    closing = cv2.erode(filter_img, kernel)
    closing = cv2.dilate(closing, kernel3)
    closing = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernel, iterations=10)

    cont_img = closing.copy()
    contours, hierarchy = cv2.findContours(cont_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bestCtn = []
    buttonParts = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 10000:
            bestCtn.append(cnt)
        elif (area > 300 and area < 5000):
            buttonParts.append(cnt)


            ##Find a way to tell program that button parts will only ever be in the middle of the big blobs

    print len(bestCtn)
    a = bestCtn[1]
    print cv2.contourArea(a)
    for i in contours:
        print cv2.contourArea(i)

    print len(buttonParts)

    #storage = cv.CreateMemStorage(0)
    #contour = cv.FindContours(filter_img, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
    
    cv2.imshow('filter', closing)

    cv2.waitKey(0);



newimg = cv2.imread('Images/ss6.JPG')
getStandardState(newimg)

    

def find_connected_components(img):
    """Find the connected components in img being a binary image.
    it approximates by rectangles and returns its centers
    """

    storage = cv.CreateMemStorage(0)
    contour = cv.FindContours(img, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
    centers = []

    while contour:
        # Approximates rectangles
        bound_rect = cv.BoundingRect(list(contour))

        centers.append(bound_rect[0] + bound_rect[2] / 2, bound_rect[1] + bound_rect[3] / 2)

        contour = contour.h_next()
