import numpy as np
import cv2

################ROBOT IS BLUE PLAYER######################


def threshold_for_contours(img_from_array):

    hsvimg = cv2.cvtColor(img_from_array, cv2.COLOR_BGR2HSV)

    #print type(hsvimg)
    lower_blue = np.array([108, 115, 74], np.uint8)
    #estimated values until testing
    upper_blue = np.array([155, 201, 215], np.uint8)
    #estimated values until testing

    #threshold_for_contours boundaries to only get orange colors
    lower_orange = np.array([0, 100, 148], np.uint8)
    #estimated values until testing
    upper_orange = np.array([91, 231, 224], np.uint8)
    #estimated values until testing

    #heres where it is actually threshold_for_contourse
    #using the boundaries to get specified colors
    #print 'Type:', hsvimg.dtype, 'Other bullshit:', hsvimg.shape
    blue = cv2.inRange(hsvimg, lower_blue, upper_blue)
    orange = cv2.inRange(hsvimg, lower_orange, upper_orange)
    #blue and orange are binary threshholded images
    #if the image initially had a blue piece,
    #blue will have values of zero and 1
    # while orange will just 0's and vice versa
    contours_blue, hierarchy_blue = cv2.findContours(blue, 1, 2)
    contours_orange, hierarchy_orange = cv2.findContours(orange, 1, 2)
    #for some reason contours only has 1 element in it at the most
    return contours_blue, contours_orange

'''
def determine_color_and_piece_existence(Bcontours, Ocontours):
    global bluepiece
    global piece_exists
    global contours_blue
    global contours_orange
    if (len(Ocontours) == 0 and len(Bcontours) > 0):
        bluepiece = True
        return bluepiece, piece_exists
    elif (len(Bcontours) == 0 and len(Ocontours) > 0):
        bluepiece = False
        return bluepiece, piece_exists
    elif (len(Ocontours) == 0 and len(Bcontours) == 0):
        piece_exists = False
        return bluepiece, piece_exists
    else:
        return bluepiece, piece_exists
 '''


def determine_color_and_piece_existence(Bcontours, Ocontours):
    global square_occupant  # 0 is empty, 1 is orange, 2 is blue
    #piece_exists = False
    global contours_blue
    global contours_orange

    if (len(Ocontours) == 0 and len(Bcontours) > 0):
        square_occupant = 2
        return square_occupant

    elif (len(Bcontours) == 0 and len(Ocontours) > 0):
        square_occupant = 1
        return square_occupant

    elif (len(Ocontours) == 0 and len(Bcontours) == 0):
        square_occupant = 0
        return square_occupant

    else:
        square_occupant = 0
        return square_occupant

'''
def determine_center( bluepiece, Bcontours, Ocontours):

    if(bluepiece == 6 and piece_exists == True):
        cnt = Bcontours[0]
        #Detected contours. stored as array of detected contours
        M = cv2.moments(cnt)
            #area = cv2.contourArea(cnt)
        cx_coord= int(M['m10']/M['m00'])
        cy_coord = int(M['m01']/M['m00'])
        return cx_coord, cy_coord

    elif(bluepiece == 7 and piece_exists == True):
        cnt = Ocontours[0]
        #Detected contours. Each contour is stored as a vector of points.
        M = cv2.moments(cnt)
            #area = cv2.contourArea(cnt)
        cx_coord = int(M['m10']/M['m00'])
        cy_coord = int(M['m01']/M['m00'])
        return cx_coord, cy_coord

    else:
        cx_coord = 0
        cy_coord = 0
        return cx_coord, cy_coord
'''


def main(array_of_images):
    occupancy_grid = []
    array_of_coordinates = []
    global square_occupant
    square_occupant = 0
    x_coordinate = 0
    y_coordinate = 0
    global contours_blue
    contours_blue = []
    global contours_orange
    contours_orange = []
    array_of_cropped_squares = []

    array_of_cropped_squares = array_of_images

    for x in range(0, 64):
        contours_blue, contours_orange = threshold_for_contours(array_of_cropped_squares[x])

        square_occupant = determine_color_and_piece_existence(contours_blue, contours_orange)

        #x_coordinate, y_coordinate =  determine_center(bluepiece, contours_blue, contours_orange)

        array_of_coordinates.append([x_coordinate, y_coordinate])

        occupancy_grid.append(square_occupant)

    return occupancy_grid, array_of_coordinates
