import cv2
import numpy as np

def get_card_loc(img):
    kernelg = np.ones((4,4), np.uint8)
    #seems to be extremely less effective, just grouping old code together
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(grayscale)
    ret,thresh3 = cv2.threshold(equ,247,255,cv2.THRESH_TRUNC)
    ret,thresh4 = cv2.threshold(thresh3,130,255,cv2.THRESH_TOZERO)
    cv2.imshow('thresh3', thresh3)
    cv2.waitKey(0)
    gray = cv2.adaptiveThreshold(grayscale,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,47,13)
    gray = cv2.erode(gray, kernelg)
    gray = cv2.dilate(gray, kernelg)
    cv2.imshow('adaptive thresh', gray)
    cv2.waitKey(0)
    # Used for morphology 
    kernel = np.ones((150, 150),np.uint8)
   
    # Color range is currently set up for red only, we need to have it account for blue too
    # Converts input to HSV format, sets up color range of cards
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower_red = np.array([110, 100, 100])
    upper_red = np.array([130, 255, 255])
    
    # Masks out all colors not in range
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    """
    # Shows images for testing
    cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
    cv2.namedWindow('imgs', cv2.WINDOW_NORMAL)
    cv2.imshow('imgs', img)
    cv2.imshow('mask', mask)
    """

    cv2.imshow('binary', mask)
    cv2.waitKey(0)
    # The morphology removes the design on the card so it is just a white rectangle
    #bImg = cv2.morphologyEx(bImg, cv2.MORPH_CLOSE, kernel)
     
    # These contour the card which is used to be fitted with a rectangle
    ret, thresh = cv2.threshold(bImg, 127, 255, 0)

    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #approx_area = (-486526 * height) + 163108

    print 'this is length of contours'
    print len(contours)
    for cur in contours:
        cv2.drawContours(img, [cur], -1, (0,255,0), 10)

    cv2.imshow('img', img)
    cv2.waitKey(0)

    cardBox = []
    for current in contours:
        area = cv2.contourArea(current)
        print 'this is area'
        print area
        if area > 80000 and area < 9000000: #will need to be adjusted !!!!!
            cardBox.append(current)


    boundingBox = cardBox[0]
    cv2.drawContours(img, boundingBox, -1, (0,255,0), 10)
    
    # Fits the binary image contours with a rectangle
    (xCenter,yCenter),(MA,ma),ang = cv2.minAreaRect(boundingBox)
    
    cv2.circle(img,(int(x), int(y)),2,(0,0,255),30)
    
    img = cv2.resize(img, (0,0), fx=0.3, fy=0.3) 
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''
    #OUTPUT PRINTS
    print '\n(x,y): '
    print (x,y)
    print '\n(MA, ma): '
    print (MA, ma)
    print '\nangle: '
    print ang
    '''
    
    # Outputs the center coordinate, minor and MAJOR axes, and angle of rotation.
    return (xCenter,yCenter), (MA, ma), ang

colorimage = cv2.imread('heights/18cm.jpg') 
get_card_loc(colorimage)