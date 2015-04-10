import cv2
import numpy as np

img = cv2.imread('closeTest/closeetch3.jpg')
small = cv2.imread('rectangle.png', 0)
grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 47, 7)
#gray = cv2.erode(gray, kernelg)
cv2.imshow('adaptive thresh', gray)
cv2.waitKey(0)
kernel2 = np.ones((8, 5), np.uint8)
gray = cv2.erode(gray, kernel2)


cv2.imshow('adaptive thresh', gray)
cv2.waitKey(0)

contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

contours_small,hierarchy_small = cv2.findContours(small.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnt1 = contours[0]

squares = []
for cnt in contours:
	ret = cv2.matchShapes(cnt1,cnt,1,0.0)
	area = cv2.contourArea(cnt)
	print ret
	if area > 20000: #area value is fishy need for straight across par
t		squares.append(cnt)
	

for i in squares:
	cv2.drawContours(img, [i], 0, (255, 255, 255), 10)


small_square = squares[0]

###GET HEIGHTS

boxpoints = cv2.minAreaRect(small_square)
               # rect = ((center_x,center_y),(width,height),angle)
points = cv2.cv.BoxPoints(boxpoints)         # Find four vertices of rectangle from above rect
points = np.int0(np.around(points))
cv2.drawContours(img,[points],0,(0,0,255),2)

cv2.imshow('img with contours', img)
cv2.waitKey(0)


cv2.imshow('orgi', img)
cv2.waitKey(0)

angle = boxpoints[2]
#angle = -90 - angle
print angle