import cv2
import numpy as np

def getCardLoc( img ):
    # Will be changed to take video input
    #img = cv2.imread(imgIn, cv2.IMREAD_COLOR)
    
    # Used for morphology 
    temp = np.array(0)
    kernel = np.ones((150,150),np.uint8)
   
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
    
    # bImg is the binary image isolating the card
    bImg = cv2.bitwise_or(mask, temp)
    
    # The morphology removes the design on the card so it is just a white rectangle
    bImg = cv2.morphologyEx(bImg, cv2.MORPH_CLOSE, kernel)
    
    # Testing outputs
    #cv2.namedWindow('bImgF', cv2.WINDOW_NORMAL)
    #cv2.imshow('bImgF', bImg)
    
    # These contour the card which is used to be fitted with a rectangle
    ret, thresh = cv2.threshold(bImg, 127, 255, 0)
    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cardBox = []
    for current in contours:
        area = cv2.contourArea(current)
        print area
        if area > 80000 and area < 9000000: #will need to be adjusted !!!!!
            cardBox.append(current)


    boundingBox = cardBox[0]
    cv2.drawContours(img, boundingBox, -1, (0,255,0), 10)
    
    
    
    
    # Fits the binary image contours with a rectangle
    (x,y),(MA,ma),ang = cv2.minAreaRect(boundingBox)
    
    cv2.circle(img,(int(x), int(y)),2,(0,0,255),30)
    

    img = cv2.resize(img, (0,0), fx=0.3, fy=0.3) 
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #OUTPUT PRINTS
    print '\n(x,y): '
    print (x,y)
    print '\n(MA, ma): '
    print (MA, ma)
    print '\nangle: '
    print ang
    
    
    # Outputs the center coordinate, minor and MAJOR axes, and angle of rotation.
    return (x,y), (MA, ma), ang

colorimage = cv2.imread('Images/color_in-9.jpg') 
getCardLoc(colorimage)