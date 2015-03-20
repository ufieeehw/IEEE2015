import cv2
import numpy as np

def getStartingButtonCoord(contours):
    #get the individual button identitites
    bottomB = contours[0]
    topB = contours[1]

    #get topmost point of top button and bottommost point of bottom button 
    #these two will be points we aim to press, or in between them can be arranged
    topmost = tuple(topB[topB[:,:,1].argmin()][0])
    bottommost = tuple(bottomB[bottomB[:,:,1].argmax()][0])

    return topmost, bottommost