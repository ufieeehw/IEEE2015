import cv2
import numpy as np

img1 = cv2.imread('Images/template3.png')
gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
#img1 = cv2.resize(img1, (0,0), fx=0.3, fy=0.3) 
img2 = cv2.imread('Images/Set1Dark/sdark2.JPG')
gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
img2 = cv2.resize(img2, (0,0), fx=0.3, fy=0.3) 
gray2 = cv2.resize(gray2, (0,0), fx=0.3, fy=0.3) 

thresh = cv2.adaptiveThreshold(gray1,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,3,2)


#flip black and white pixels
thresh = (255-thresh)


kernel = np.ones((3, 3))
#kernel for eroding
kernel2 = np.ones((4,4), np.uint8)
#kernel for dilating
kernel3 = np.ones((5, 5), np.uint8)
eroded = cv2.erode(thresh, kernel2)
thresh = cv2.dilate(eroded, kernel3)

cv2.imshow('sup', thresh)
cv2.waitKey(0)

blur = cv2.GaussianBlur(gray2,(3,3),0)
ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

#th3 = cv2.morphologyEx(eroded, cv2.MORPH_TOPHAT, kernel)

cv2.imshow('sup', th3)
cv2.waitKey(0)


thresh2 = cv2.adaptiveThreshold(gray2,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,37,8)

#flip black and white pixels
thresh2 = (255-thresh2)

kernel = np.ones((3, 3))
#kernel for eroding
kernel2 = np.ones((4,4), np.uint8)
#kernel for dilating
kernel3 = np.ones((5, 5), np.uint8)
eroded = cv2.erode(thresh2, kernel2)
thresh2 = cv2.dilate(eroded, kernel3)

cv2.imshow('sup', thresh)
cv2.waitKey(0)


cv2.imshow('sup', thresh2)
cv2.waitKey(0)

contours,hierarchy = cv2.findContours(thresh,2,1)


templateCtn = []
for bae in contours:
    area = cv2.contourArea(bae)
    print area
    if area > 8000 and area < 200000:
        templateCtn.append(bae)
        cv2.drawContours(img1, [bae], 0, (0, 255, 255), 100)


contours,hierarchy = cv2.findContours(th3,2,1)

wholeToy = []
for bae2 in contours:
    area = cv2.contourArea(bae2)
    print area
    if area > 20000 and area < 600000:
        wholeToy.append(bae2)
        cv2.drawContours(img2, [bae2], 0, (255, 255, 0), 4)

buttonCtn = []
for bae3 in contours:
    area = cv2.contourArea(bae3)
    print area
    if area > 10000 and area < 40000:
        buttonCtn.append(bae3)
        cv2.drawContours(img2, [bae3], 0, (0, 255, 0), 4)


    #closing = cv2.resize(closing, (0,0), fx=0.5, fy=0.5) 
cv2.imshow('sup', img2)
cv2.waitKey(0)

ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
print ret