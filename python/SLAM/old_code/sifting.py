import numpy as np
from copy import deepcopy
import cv2

def findBlue(piece):
	rows, cols, dim = piece.shape
	#this matrix is set up with row values in row1 and col vals in row2
	# but this gets flipped when generating the output array
	matrix = np.matrix([[0, rows-1, rows-1,      0, 0, 0],
					    [0,      0, cols-1, cols-1, 0, 0]])
	for n in range(0,rows-1):
		if piece[n,0,0] == 255:
			matrix[0,0]=n-2
			break

	for n in range(0,rows-1):
		if piece[n,cols-1,0] == 255:
			matrix[0,3] = n-2
			break

	for n in range(0,rows-1):
		if piece[n,np.floor(rows/2),0] == 255:
			border = n-2
			break

	for n in range(0,cols-1):
		if piece[border,n,0] != 255:
			matrix[0,5] = border
			matrix[1,5] = n+2
			break

	for n in range(matrix[1,5] + 5,cols-1):
		if piece[border,n,0] == 255:
			matrix[0,4] = border
			matrix[1,4] = n-2
			break
	points = np.array( [ [matrix[1,0], matrix[0,0]],
						 [matrix[1,1], matrix[0,1]],
						 [matrix[1,2], matrix[0,2]],
						 [matrix[1,3], matrix[0,3]],
						 [matrix[1,4], matrix[0,4]],
						 [matrix[1,5], matrix[0,5]],
						 [matrix[1,0], matrix[0,0]]] )
	return points


def fenceCorners(dst,points):
	cv2.fillPoly(dst,[points],0)
	return dst

def harrisCorner(pic):
	#convert to grayscale so we can process
	gray_pic = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
	gray_pic = np.float32(gray_pic)

	#do the harris corners algorithm
	corners = cv2.cornerHarris(gray_pic, 2, 3, 0.04)
	return cv2.dilate(corners,None)

def maskBlue(piece,piece_corners):
	points = findBlue(piece)
	cv2.fillPoly(piece_corners,[points],0)
	return piece_corners

def maskTemplate(template,template_corners):
	rows,cols,dim = template.shape
	#have to do this in two parts since it is a donut
	top_points = np.array( [ [0     ,  0],
						     [cols-1,  0],
						     [cols-1,155],
						     [0     ,155]])
	bottom_points = np.array(  [ [433   ,   155],
						 		 [433   ,   575],
						 		 [639   ,   575],
						 		 [639   ,   155],
						 		 [cols-1,   155],
						 		 [cols-1,rows-1],
						 		 [0     ,rows-1],
						 		 [0     ,   155]])
	cv2.fillPoly(template_corners,[top_points],0)
	cv2.fillPoly(template_corners,[bottom_points],0)
	return template_corners


#set filename and read it into an opencv object
template_location = 'template.jpg'
piece_location = 'scaledxform.png'
template = cv2.imread(template_location)
piece = cv2.imread(piece_location)

#get corners of grayscale piece
piece_corners = harrisCorner(piece)
template_corners = harrisCorner(template)

#find the blue region of the original picture
piece_corners = maskBlue(piece, piece_corners)
template_corners = maskTemplate(template, template_corners)

#plop on the points
piece[piece_corners>0.01*piece_corners.max()] = [0,0,255]
template[template_corners>0.01*template_corners.max()] = [0,0,255]

#write to file
cv2.imwrite('piececorners.png',piece)
cv2.imwrite('templatecorners.png',template)

while(1):
    cv2.imshow("piece",piece)
    cv2.imshow("template",template)


    if cv2.waitKey(20) & 0xFF == 27:
        break
cv2.destroyAllWindows()
