import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = cv2.imread('lib/firstmove1.JPG')
cv2.namedWindow('image')

hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
hsvimg = cv2.resize(hsvimg, (500, 500))
# create trackbars for color change
cv2.createTrackbar('H1','image',0,255,nothing)
cv2.createTrackbar('S1','image',0,255,nothing)
cv2.createTrackbar('V1','image',0,255,nothing)
cv2.createTrackbar('H2','image',0,255,nothing)
cv2.createTrackbar('S2','image',0,255,nothing)
cv2.createTrackbar('V2','image',0,255,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

while(1):
    

    # get current positions of four trackbars
    h1 = cv2.getTrackbarPos('H1','image')
    s1 = cv2.getTrackbarPos('S1','image')
    v1 = cv2.getTrackbarPos('V1','image')
    s = cv2.getTrackbarPos(switch,'image')
    h2 = cv2.getTrackbarPos('H2','image')
    s2 = cv2.getTrackbarPos('S2','image')
    v2 = cv2.getTrackbarPos('V2','image')

    lower_value = np.array([h1, s1, v1], np.uint8)
    upper_value = np.array([h2, s2, v2], np.uint8)

    image = cv2.inRange(hsvimg, lower_value, upper_value)

    cv2.imshow('image', image)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    #if s == 0:
    #    image[:] = 0
    #else:
    #    image[:] = [h1,s1, v1]


cv2.destroyAllWindows()
