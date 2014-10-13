import cv2
import numpy as np

# Will be changed to take video input
img = cv2.imread('/home/will/Desktop/Card Images/Card Images/IMG_4048.JPG', cv2.IMREAD_COLOR)

# Used for morphology 
temp = np.array(0)
kernel = np.ones((150,150),np.uint8)

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

# These contour the card, this might not be necessary
ret, thresh = cv2.threshold(bImg, 127, 255, 0)
contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (0,255,0), 3)

cnt = contours[0]


cv2.waitKey(0)
cv2.destroyAllWindows()

# Fits the binary image contours with a rectangle
(x,y),(MA,ma),ang = cv2.minAreaRect(cnt)

"""
# I was trying to get the contours to be drawn onto the image
rect = cv2.minAreaRect(cnt)
box = cv2.cv.BoxPoints(rect)
box = np.int0(box)
cv2.drawContours(img, [box],0,(0,255,255),20)
cv2.namedWindow('box', cv2.WINDOW_NORMAL)
cv2.imshow('box', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""

# Outputs the center coordinate, minor and MAJOR axes, and angle of rotation.
print '\n(x,y): '
print (x,y)
print '\n(MA, ma): '
print (MA, ma)
print '\nangle: '
print ang