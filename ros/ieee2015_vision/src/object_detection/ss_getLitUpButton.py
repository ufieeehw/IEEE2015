import cv2
import numpy as np


def get_lit_button(img):
    #img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
    #below ranges should work for all of the lights on ranges
    lower_on = np.array([26, 180, 215])
    upper_on = np.array([255, 255, 255])

    brightButton = cv2.inRange(img, lower_on, upper_on)

    cv2.imshow('img', brightButton)
    cv2.waitKey(0)

    print 'we got here'

    kernel = np.ones((8, 8), np.uint8)
    #kernel for eroding
    kernel2 = np.ones((8, 8), np.uint8)
    #kernel for dilating
    kernel3 = np.ones((4, 4), np.uint8)
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
    litButton = bestCtn[0]

    leftmost = tuple(litButton[litButton[:, :, 0].argmin()][0])
    print 'leftmost birhgt cbutton'
    print leftmost

    rightmost = tuple(litButton[litButton[:, :, 0].argmax()][0])
    print 'rightmost bright button'
    print rightmost

    topmost = tuple(litButton[litButton[:, :, 1].argmin()][0])
    print 'topmost bright button'
    print topmost
    bottommost = tuple(litButton[litButton[:, :, 1].argmax()][0])
    print 'bottommost bight button'
    print bottommost

    cv2.circle(img, (leftmost), 2, (0, 0, 255), 30)
    cv2.circle(img, (rightmost), 2, (0, 0, 255), 30)
    cv2.circle(img, (topmost), 2, (0, 0, 255), 30)
    cv2.circle(img, (bottommost), 2, (0, 0, 255), 30)
    #cv2.circle(img, (365, 934), 2, (0, 255, 0), 30)
    img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow('points', img)
    cv2.waitKey(0)

    meanCols = int((leftmost[0] + rightmost[0]) / 2)
    print 'mean cols bright'
    print meanCols
    meanRows = int((topmost[1] + bottommost[1]) / 2)
    print 'mean rows bright'
    print meanRows

    brightCenterPoint = (meanCols, meanRows)

    return brightCenterPoint, closing
