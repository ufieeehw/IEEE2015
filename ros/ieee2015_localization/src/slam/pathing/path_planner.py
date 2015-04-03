#!/usr/bin/env python
import cv2
import numpy as np
from itertools import product
from time import time
from skfmm import distance
from matplotlib import pyplot

def plan_path(start_position, goal_position, image):
	# make an image of ones
	img_ones = np.ones_like(image[:, :, 2])
	# find the white lines
	white_lines = image[:, :, 2] > 100
	img_ones[white_lines] = 0
	gdt_initial = distance(img_ones)
	safe_space = distance(img_ones) > 5
	img_ones[safe_space] = 0
	gdt_safe = distance(img_ones)
	#pyplot.imshow(gdt_safe, interpolation = 'nearest')
	#pyplot.show()
	
	target_contour = np.ones_like(image[:, :, 2])
	target_contour[goal_position] = 0
	masked_array = np.ma.MaskedArray(target_contour, safe_space)
	 
	gdt_final = distance(masked_array)
	
	#pyplot.imshow(gdt_final, interpolation = 'nearest')
	#pyplot.show()
	
	current = start_position

	while (current != goal_position):
		temp = current
		for i in range(-1, 2):
			for j in range(-1, 2):
				temp_arr = np.array(temp)
				survey_arr = temp_arr + np.array([i, j])
				survey = (survey_arr[0], survey_arr[1])
				if (gdt_final[survey] < gdt_final[current]):
					current = survey
		cv2.line(image, (temp[1], temp[0]), (current[1], current[0]), (0,255,0), 2)
	return image

def main():
	# define path to image
	img_path = '/home/peach/catkin_ws/src/basic_vision/src/aerialImageCropped.jpg'
	# load image
	img = cv2.imread(img_path)
	
	# define start position (y, x) 
	start = (375, 120)
	# define goal position (y, x) 
	# goal = (18, 187)
	goal = (86, 134)
	
	planned_path = plan_path(start, goal, img)
	pyplot.imshow(planned_path, interpolation = 'nearest')
	pyplot.show()
	
if __name__ == "__main__":
	main()
