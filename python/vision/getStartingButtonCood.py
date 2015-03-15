import cv2
import numpy as np

def getStartingButtonCoord(contours):
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