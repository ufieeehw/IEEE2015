import cv2
import numpy as np


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
