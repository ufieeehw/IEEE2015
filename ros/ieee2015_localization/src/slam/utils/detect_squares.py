#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot

def detect_squares(image):
	_, white_all = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
	#cv2.imshow('window', white_all1)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()

	contours, _ = cv2.findContours(white_all, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(image, contours, -1, (0,255,0), 3)

	_, white_squares = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
	#cv2.imshow('window', white_squares)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()

	return white_squares

def main():
	# define path to image
	path = '/home/peaches/Pictures/down_view.png'
	# load image
	img = cv2.imread(path)
	# convert image to grayscale
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# detect white squares
	white_squares = detect_squares(img)
	cv2.imshow('window', white_squares)
	cv2.waitKey(0)
	cv2.destroyAllWindows()	

if __name__ == "__main__":
    main()