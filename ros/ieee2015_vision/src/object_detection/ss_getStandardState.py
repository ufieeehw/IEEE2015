import cv2
import numpy as np

def getStandardState(img):
    '''
    finds blue and green color because easest to distinguish.
    from those two colors we find ranges for all four colors.
    this method also find the button initially needed to push.
    '''
    
    #creating binary image
    lowerRange = np.array([64, 115, 39])
    upperRange = np.array([118, 255, 255])
    filteredIMG = cv2.inRange(img, lowerRange, upperRange)


    #morphological stuff, std
    kernel = np.ones((3, 3), np.uint8)
    #kernel for eroding
    kernel2 = np.ones((12,12), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((10, 10), np.uint8)
    eroded = cv2.erode(filteredIMG, kernel2)
    dilated = cv2.dilate(eroded, kernel3)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=10)

    #so that when we display image for testing we dont mess with the image
    copy = closing.copy()

    #testing images
    closing = cv2.resize(closing, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('copy', closing)
    cv2.waitKey(0)

    ############################################Start of Contour Manipulation######################
    #get minimum contour
    contours, hierarchy = cv2.findContours(copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #get all the contour points
    #contoursAll, hierarchyAll = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print len(contours)
    #filtering contours to find the green and blue button
    bestCnts = []
    buttonCnts = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print 'this is area'
        print area
        if area > 60000:
            bestCnts.append(cnt) #  this contour is going to be button of interests
        elif area > 1000 and area < 5000:  # small buttons
            buttonCnts.append(cnt)

    #testing        
    #closing = cv2.resize(closing, (0,0), fx=0.5, fy=0.5) 
    #cv2.imshow('filter', closing)
    #cv2.waitKey(0);

    #distinguishing the buttons from the contours
    greenButton = bestCnts[0]
    blueButton = bestCnts[1]

    #step in order to get all the pixel values of the button
    #grayIMG = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #mask = np.zeros(grayIMG.shape,np.uint8)
    #cv2.drawContours(mask,[greenButton],0,255,-1)
    #pixelpointsL = np.transpose(np.nonzero(mask))
    #cv2.drawContours(mask,[blueButton],0,255,-1)
    #pixelpointsR = np.transpose(np.nonzero(mask))


    #get x and y values for each button
    #xPointsL = []
    #yPointsL = []
    #for points in pixelpointsL:
    #    xPointsL.append(points[0])
    #    yPointsL.append(points[1])

    #xPointsR = []
    #yPointsR = []
    #for points in pixelpointsR:
    #    xPointsR.append(points[0])
    #   yPointsR.append(points[1])

    leftmostG = tuple(greenButton[greenButton[:,:,0].argmin()][0])
    rightmostG = tuple(greenButton[greenButton[:,:,0].argmax()][0])
    topmostG = tuple(greenButton[greenButton[:,:,1].argmin()][0])
    bottommostG = tuple(greenButton[greenButton[:,:,1].argmax()][0])

    print 'this is leftmostG'
    print leftmostG
    print 'this is rightmostG'
    print rightmostG
    print 'this is topmostG'
    print topmostG
    print 'this is bottommostG'
    print bottommostG

    leftmostB = tuple(blueButton[blueButton[:,:,0].argmin()][0])
    rightmostB = tuple(blueButton[blueButton[:,:,0].argmax()][0])
    topmostB = tuple(blueButton[blueButton[:,:,1].argmin()][0])
    bottommostB = tuple(blueButton[blueButton[:,:,1].argmax()][0])

    #cv2.circle(img,(50, 300),2,(0,0,255),30)

    #cv2.imshow('circle test', img)
    #cv2.waitKey(0)
    #The + and - 50 is to give error for dilated bright button

    minColsGreen = leftmostG[0] - 50
    maxColsGreen = rightmostG[0] + 50
    minRowsGreen = topmostG[1] - 50
    maxRowsGreen = bottommostG[1] + 50

    minColsBlue = leftmostB[0] - 50
    print 'this is minColsBlue'
    print minColsBlue
    maxColsBlue = rightmostB[0] + 50
    print 'this is maxColsBlue'
    print maxColsBlue
    minRowsBlue = topmostB[1] - 50
    print 'this is minRowsBlue'
    print minRowsBlue
    maxRowsBlue = bottommostB[1] + 50
    print 'this is maxRowsBlue'
    print maxRowsBlue

    meanColsGreen = int((leftmostB[0] + rightmostB[0])/2)
    meanRowsGreen = int((topmostB[1] + bottommostB[1])/2)
    meanColsBlue = int((leftmostG[0] + rightmostG[0])/2)
    meanRowsBlue = int((topmostG[1] + bottommostG[1])/2)

    #meanXL = np.mean(xPointsL)
    #meanYL = np.mean(yPointsL)
    #meanXR = np.mean(xPointsR)
    #meanYR = np.mean(yPointsR)

    return bestCnts, buttonCnts, meanColsBlue, meanRowsBlue, meanColsGreen, meanRowsGreen, minColsGreen, maxColsGreen, minRowsGreen, maxRowsGreen, minColsBlue, maxColsBlue, minRowsBlue, maxRowsBlue
    #print meanxPointsL
    
 


