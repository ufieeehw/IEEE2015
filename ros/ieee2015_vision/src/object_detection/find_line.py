import cv2
import numpy as np
import math
 
def find_line(img):
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(grayscale,50,150,apertureSize = 3)
    
    lines = cv2.HoughLines(edges,1,np.pi/180,100)

    rho, theta = lines[0][2]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
     
     
    cv2.line(img, (x1, y1), (x2, y2), (255,0,0), 2)

    #Determine which which direction and by how much the image should be rotated
    if (theta > 0 and theta < np.pi/4.0):
        angleOfRotation = theta
 
    elif (theta > np.pi/4.0 and theta < np.pi * (3/4.0)):
        angleOfRotation = (theta - np.pi/2.0)
 
    elif (theta > np.pi * (3/4.0) and theta < np.pi * (5/4.0)):
        angleOfRotation = (theta - np.pi/1.0)
 
    elif (theta > np.pi * (5/4.0) and theta < np.pi * (7/4.0)):
        angleOfRotation = (theta - np.pi * (3/2.0))
 
    elif (theta > np.pi * (7/4.0)):
        angleOfRotation = (theta - np.pi * 2.0)
 
    else:
        angleOfRotation = 0
 
    return angleOfRotation*(180/np.pi)
