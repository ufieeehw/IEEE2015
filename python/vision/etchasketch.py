import cv2
import numpy as np
import cv2.cv as cv


image = cv2.imread("eas.png")

grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

hsv_etchasketch = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
hsv_etchasketch = cv2.resize(hsv_etchasketch, (300,250))


lower_white = np.array([70, 0, 170], np.uint8)  #estimated values until testing
upper_white = np.array([179, 20, 255], np.uint8) #estimated values until testing
# Searching the HSV image to extract the white colors

binary_img = cv2.inRange(hsv_etchasketch, lower_white, upper_white)
small_bin = cv2.resize(binary_img, (300, 250))



grayscale = cv2.resize(grayscale, (300, 250))

ret,thresh = cv2.threshold(grayscale,175,255,0)

cv2.imshow('binary', thresh)
cv2.waitKey(0)
cv2.destroyAllWindows


contours_k, hierarchy_k = cv2.findContours(thresh, 1, 2)


contours_knobs, hierarchy_knobs = cv2.findContours(small_bin, 1, 2)

cnt = contours_k[0] #Detected contours. Each contour is stored as a vector of points.
M = cv2.moments(cnt)
    #area = cv2.contourArea(cnt)
cx_coord= int(M['m10']/M['m00'])
cy_coord = int(M['m01']/M['m00'])

print cx_coord
print cy_coord

cv2.circle(grayscale, (cx_coord, cy_coord), 50, (100, 255, 0), thickness=1, lineType=8, shift=0)
cv2.drawContours(grayscale,[cnt],0,(255,0,0),-1)
        


cv2.imshow('image w/ contours', grayscale)
cv2.waitKey(0)
cv2.destroyAllWindows()




'''
cv2.imshow("circles", circles)
cv2.waitKey(0)
cv2.destroyAllWindows()


if circles is not None:
	circles = np.uint16(np.around(circles))
	for i in circles[0,:]:
	# draw the outer circle
		cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
		# draw the center of the circle
		cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
else:
	print "You screwed up"

small = cv2.resize(image, (300, 250))
cv2.imshow('detected circles', small)
cv2.waitKey(0)
cv2.destroyAllWindows()

'''









'''

threshold_img = cv2.adaptiveThreshold(grayscale,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)

height, width = threshold_img.shape







new_thresh = cv2.resize(threshold_img, (300, 250))

cv2.imshow('hello', new_thresh)
cv2.waitKey(0)
cv2.destroyAllWindows
#extracted_white = cv2.inRange(hsveye, lower_white, upper_white)

kernel = np.ones((3,3), np.uint8) #square of 2x2 pixels to use for dilating
kernel2 = np.ones((2,2), np.uint8) #square of 1x1 pixels to use for eroding
eroded_image = cv2.erode(threshold_img,kernel2,iterations = 1)
cv2.imshow("cleaned up image", eroded_image)
cv2.waitKey(0)

dilated_image = cv2.dilate(eroded_image, kernel, iterations = 1)
cv2.imshow("dilated image", dilated_image)
cv2.waitKey(0)

#cv2.imshow("dilated course", course_image_binary)
#cv2.waitKey(0)
#cv2.destroyAllWindows
#finds countours of threshold image 

contours,h = cv2.findContours(threshold_img,1,2)
cnt = contours[0]



ellipse = cv2.fitEllipse(cnt)
cv2.ellipse(image,ellipse,(0,255,0),2)
cv2.imshow = ('ellipse', ellipse)
cv2.waitKey(0)
cv2.destroyAllWindows

#Goes through the contours image and draws green pixels where it finds rectangles
#on top of the original color image

#rectanglecounter = 0
for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    
    if len(approx)==4:
        cv2.drawContours(image,[cnt],0,(0,255,0),-1)





#Creates an image called threshold, which is the threshold of the grayscale image

#finds countours of threshold image 

'''