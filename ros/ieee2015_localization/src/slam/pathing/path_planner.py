#!/usr/bin/env python
'''
    Authors:
    Brandon Peterson
    Jacob Panikulam
'''
import cv2
import numpy as np
import math
from itertools import product
from time import time
from skfmm import distance
from matplotlib import pyplot

def plan_path(start_position, goal_position, image):
    image = cv2.resize(image, (250, 250))
    
    # pyplot.imshow(image)
    # pyplot.show()

    # Make an image of ones
    img_ones = np.ones_like(image[:, :, 2])

    # Find the white lines
    white_lines = image[:, :, 2] > 200
    # cv2.imshow("whites", np.uint8(white_lines) * 255)
    # cv2.waitKey(0)

    img_ones[white_lines] = 0

    gdt_initial = distance(img_ones)
    # allow leeway when following lines
    safe_space = distance(img_ones) > 10

    img_ones[safe_space] = 0
    gdt_safe = distance(img_ones)

    # pyplot.imshow(gdt_safe, interpolation = 'nearest')
    # pyplot.show()
    
    target_contour = np.ones_like(image[:, :, 2])
    target_contour[goal_position] = 0
    masked_array = np.ma.MaskedArray(target_contour, safe_space)
     
    gdt_final = distance(masked_array)
    
    pyplot.imshow(gdt_final, interpolation = 'nearest')
    #pyplot.show()
    
    current = np.array(start_position)

    path = []
    prev_cost = np.inf
    best_cost = np.inf
    _next = current
    while(not np.allclose(_next, np.array(goal_position))):
        current = _next
        #if best_cost == prev_cost:
        #    return None # Failed to find path
            
        prev_cost = best_cost
        for x in [-1, 0, 1]:
            for y in [-1, 0, 1]:
                xy = np.array([x, y])

                try:
                    summed = current +  xy
                    cost = gdt_final[summed[0], summed[1]]

                except(IndexError):
                    continue

                if cost < best_cost:
                    best_cost = cost
                    _next = current + xy

        path.append((_next[1], _next[0]))
        cv2.line(image, (_next[1], _next[0]), (current[1], current[0]), (0, 255, 0), 2)

    pyplot.imshow(image, interpolation = 'nearest')
    #pyplot.show()

    return path

def get_corners(path):
    corners_all = []
    for i in range(len(path)-2):  
        if (slope(path[i], path[i+1]) != slope(path[i+1], path[i+2])):            
            corners_all.append(path[i])
    #print "ALL CORNERS:"
    #print corners_all
    
    corners_simplified = []
    for i in range(len(corners_all)):
    	c1 = np.array(corners_all[i-1])
    	c2 = np.array(corners_all[i])
        if ((np.abs(c1[0] - c2[0]) < 2)  and (np.abs(c1[1] - c2[1]) > 5) or 
            (np.abs(c1[0] - c2[0]) > 5)  and (np.abs(c1[1] - c2[1]) < 2) or
            (np.abs(c1[0] - c2[0]) > 1)  and (np.abs(c1[1] - c2[1]) > 1)):
                corners_simplified.append(corners_all[i])
    #print "NECESSARY CORNERS:"
    #print corners_simplified
    return corners_simplified
    
def slope((x1, y1), (x2, y2)):
    if (x1 == x2):
    	return 2
    else:
    	return (y2 - y1) / (x2 - x1)

def get_direction_vectors(corners):
    direction_vectors = []

    for i in range(len(corners)-1):
        p1 = corners[i]
        p2 = corners[i+1]
        #print "Start Point:", p1, 
        #print "; End Point:", p2

        # get vector distance between two consecutive points
        distance = [p2[0]-p1[0], p2[1]-p1[1]]
        #print "Distance Vector:", distance

        # normalize distance vector to unit vector
        norm = math.sqrt(distance[0]**2 + distance[1]**2)
        direction = [distance[0] / norm, distance[1] / norm]
        #print "Direction Unit Vector:", direction
        #print "--------------------------------------------------------------------"
        direction_vectors.append(direction)
    return direction_vectors


def main():
    # define path to image
    img_path = 'down_view.png'
    # load image
    img = cv2.imread(img_path)
    
    # define start position (y, x) 
    start = (244, 106)
    # define goal position (y, x) 
    goal = (108, 178)
    
    tic = time()
    planned_path = plan_path(start, goal, img)
    corners = get_corners(planned_path)
    direction_vectors = get_direction_vectors(corners)
    # print direction unit vectors
    for i in range(len(direction_vectors)):
            print direction_vectors[i]
    toc = time() - tic   
    print '{} secs'.format(toc)
    
    
if __name__ == "__main__":
    main()
