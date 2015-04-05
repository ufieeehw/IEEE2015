import cv2
import numpy as np


def get_lit_button(img):
    #img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
    #below ranges should work for all of the lights on ranges
    lower_on = np.array([0, 80, 140])
    upper_on = np.array([255, 255, 255])

    bright_button = cv2.inRange(img, lower_on, upper_on)

    cv2.imshow('img', bright_button)
    cv2.waitKey(0)

    kernel = np.ones((8, 8), np.uint8)
    #kernel for eroding
    kernel2 = np.ones((8, 8), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((4, 4), np.uint8)
    eroded = cv2.erode(bright_button, kernel2)
    dilated = cv2.dilate(eroded, kernel3)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel, iterations=10)
    cv2.imshow('closing', closing)
    cv2.waitKey(0)
    #to be used later
    #cont_img = closing.copy()

    ############################################Start of Contour Manipulation######################
    #get minimum contour
    contours, hierarchy = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #filtering contours to find the green and blue button
  
    #best_ctn should ideally be length of one at this point
    lit_button = contours[0]

    leftmost = tuple(lit_button[lit_button[:, :, 0].argmin()][0])

    rightmost = tuple(lit_button[lit_button[:, :, 0].argmax()][0])

    topmost = tuple(lit_button[lit_button[:, :, 1].argmin()][0])

    bottommost = tuple(lit_button[lit_button[:, :, 1].argmax()][0])

    #Testing
    cv2.circle(img, (leftmost), 2, (0, 0, 255), 30)
    cv2.circle(img, (rightmost), 2, (0, 0, 255), 30)
    cv2.circle(img, (topmost), 2, (0, 0, 255), 30)
    cv2.circle(img, (bottommost), 2, (0, 0, 255), 30)
    #cv2.circle(img, (365, 934), 2, (0, 255, 0), 30)
    #img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow('points', img)
    cv2.waitKey(0)

    mean_cols = int((leftmost[0] + rightmost[0]) / 2)

    mean_rows = int((topmost[1] + bottommost[1]) / 2)
   
    return mean_cols, mean_rows, closing
img = cv2.imread('ti/ss3.JPG')
img = cv2.resize(img,None,fx=.2, fy=.2,)
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
get_lit_button(img)
#mc, mr, clsing = get_lit_button(img)
#print mc, mr