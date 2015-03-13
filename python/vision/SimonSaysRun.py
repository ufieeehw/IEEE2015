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


    #get x and y values for each button
    xPointsL = []
    yPointsL = []
    for points in pixelpointsL:
        xPointsL.append(points[0])
        yPointsL.append(points[1])

    xPointsR = []
    yPointsR = []
    for points in pixelpointsR:
        xPointsR.append(points[0])
        yPointsR.append(points[1])


    meanXPointsL = np.mean(xPointsL)
    meanYPointsL = np.mean(yPointsL)
    meanXPointsR = np.mean(xPointsR)
    meanYPointsR = np.mean(yPointsR)
    
    minXPointL = min(xPointsL)
    maxXPointL = max(xPointsL)
    minYPointL = min(yPointsL)
    maxYPointL = max(yPointsL)

    minXPointR = min(xPointsR)
    maxXPointR = max(xPointsR)
    minYPointR = min(yPointsR)
    maxYPointR = max(yPointsR)

    return meanXPointsR, meanYPointsR, meanXPointsL, meanYPointsL, minXPointL, maxXPointL, minYPointL, maxYPointL, minXPointR, maxXPointR, minYPointR, maxYPointR
    #print meanxPointsL
    '''
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

    xG,yG,wG,hG = cv2.boundingRect(greenButton)
    
    print xG
    print yG
    print wG
    print hG
    xB,yB,wB,hB = cv2.boundingRect(blueButton)

    #ellipseAverage = ellipseG
    ellipseB = list(ellipseB)
    ellipseG = list(ellipseG)

    #gets major and minor axis
    g = ellipseG[1]
    b = ellipseB[1]

    
    ellipseGX = []
    ellipseGX = list(ellipseGX)
    print(type(ellipseGX))
    for rows in ellipseG:
        ellipseGX.append(int(rows[0]))
    print ellipseGX

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
    '''

def getLitUpButton(img):
    
    #below ranges should work for all of the lights on ranges
    lower_on = np.array([0, 0, 245])
    upper_on = np.array([255, 255, 255])

    brightButton = cv2.inRange(img, lower_on, upper_on)

    brightButton = cv2.resize(brightButton, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('lit up button', brightButton)
    cv2.waitKey(0)

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

    
    meanXPoints = np.mean(xPoints)
    meanYPoints = np.mean(yPoints)

   
    return meanXPoints, meanYPoints, closing
 
#input for this is the mean point value for the green, blue, and lit up space
#this is to check and see what color is lit up
#last input should be the BINARY <-----
#1 is yellow
#2 is green
#3 is red
#4 is nothing
def findColor(brightX, brightY, greenX, greenY, blueX, blueY, minXPointL, maxXPointL, minYPointL, maxYPointL, minXPointR, maxXPointR, minYPointR, maxYPointR):
    averageRow = (greenX + blueX)/2

    if(brightX > averageRow and brightY > greenY and brightY < blueY):
        return 1 #  we have a yellow button lighting up
    elif(brightX < averageRow and brightY > greenY and brightY < blueY):
        return 3  #  we have a red button lighting up
    elif(brightX < maxXPointL and brightX > minXPointL and brightY < maxYPointL and brightY > minYPointL):
        return 2
    elif(brightX < maxXPointR and brightX > minXPointR and brightY < maxYPointR and brightY > minYPointR):
        return 4
    else:
        return -1



        




def makeHSV(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return hsv_img


####HEY SLEEP TEST OUT THAT THE COLORS READ IN RIGHT!#############3
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = cv2.imread('Images/ss6.JPG')
newimg = makeHSV(newimg)
getStandardState(newimg)
getLitUpButton(newimg)
colors = []


    

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
