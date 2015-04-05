#!/usr/bin/env python
import cv2
import numpy as np
from itertools import product
from time import time
from skfmm import distance
from matplotlib import pyplot

def plan_path(start_position, goal_position, image):
    tic = time()
    image = cv2.resize(image, (250, 250))

    # Make an image of ones
    img_ones = np.ones_like(image[:, :, 2])

    # Find the white lines
    white_lines = image[:, :, 2] > 200
    # cv2.imshow("whites", np.uint8(white_lines) * 255)
    # cv2.waitKey(0)

    img_ones[white_lines] = 0

    gdt_initial = distance(img_ones)
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
    pyplot.show()
    
    current = np.array(start_position)

    path = []
    prev_cost = np.inf
    best_cost = np.inf
    _next = current
    while(not np.allclose(_next, np.array(goal_position))):
        current = _next
        if best_cost == prev_cost:
            return None # Failed to find path
            
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
    pyplot.show()

    toc = time() - tic
    print '{} secs'.format(toc)

    return path

def main():
    # define path to image
    img_path = 'down_view.png'
    # load image
    img = cv2.imread(img_path)
    
    # define start position (y, x) 
    start = (244, 106)
    # define goal position (y, x) 
    goal = (108, 178)
    
    planned_path = plan_path(start, goal, img)
    print planned_path
    
if __name__ == "__main__":
    main()