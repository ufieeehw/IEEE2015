import cv2
import numpy as np
import math

'''
filename = 'lib/chessanglesmall.jpg'
img = cv2.imread(filename)
#newimage = cv2.resize(img,(640,480))
gray = cv2.imread(filename, 0)
'''

#rotates the image at a given angle value
def rotateImage(theta, img):
	rows,cols, ret = img.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),theta,1)
	dst = cv2.warpAffine(img,M,(cols,rows))
	print "type is: " , type(dst)
	return dst


