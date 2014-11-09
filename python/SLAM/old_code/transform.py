import numpy as np
from copy import deepcopy
import cv2

# mouse callback function
def get_point(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),1,(255,0,0),-1)
        print x,y

#Calculate the transform matrix
def getForwardMatrix(orig,guess):
	#scale and move image around so that your location is 400,800 and 
	#  you are sufficiently zoomed out
	new = tune_output_square(.4,[-160,170],guess)
	#calculate transform matrix
	return cv2.getPerspectiveTransform(orig,new)

#scale the transformed image so its dimensions are equal to the template
#find the current size of a feature in px and then state the desired size of this feature
#in the given template, a square is 53px (so we put in 53 for desired)
def scaleImage(imgx,current,desired):
	factor = desired/current
	print factor
	rows,cols,ch = imgx.shape
	scaledimg = np.zeros((np.floor(rows*factor) , np.floor(cols*factor) , 3) , np.uint8)
	scaledimg = cv2.resize(imgx,(int(rows*factor),int(cols*factor)),interpolation = cv2.INTER_AREA)
	return scaledimg


#quick fix for playing with the output position of the calibration
#  quadrilateral such that we are zoomed out correctly
def tune_output_square(scale,translate,points):
	newpoints = deepcopy(points)
	avg_x = (points[0][0] + points[1][0] + points[2][0] + points[3][0])/4
	avg_y = (points[0][1] + points[1][1] + points[2][1] + points[3][1])/4
	for n in range(0,4):
		newpoints[n][0] = avg_x - (avg_x - points[n][0])*scale + translate[0]
		newpoints[n][1] = avg_y - (avg_y - points[n][1])*scale + translate[1]
	return newpoints

#for helping me find the corners of the square..
#  This just outputs the px values on a horizontal line on the image
#  and then paints this line green for reference
def sample_lines():
	for n in range(493,794):
	 	print (img[481,n])
	 	img[481,n] = (0,255,0)
	 	print n
	print "NEEEEEXT\n\n\n\n\n\n\n"
	for n in range(486,698):
	 	print (img[457,n])
	 	img[457,n] = (0,255,0)
	 	print n

#calculate where the cone of vision has a vertex (your location)
#  and draw lines on the image to help visualize this
def find_vertex(M,imgx):
	#initialize boundary points
	points = np.matrix([[0,   0    , 800, 800],
					    [600, 800  , 600, 800],
					    [1, 1    , 1  , 1  ]])
	#calculate transformed points (in homogeneous coordinates)
	homopoints = M * points
	#convert homogeneous points to xy
	newpoints = np.zeros((2,4))
	for i in range(0,2):
		for j in range(0,4):
			newpoints[i,j] = homopoints[i,j]/homopoints[2,j]
	#calculate line parameters for both boundaries
	m1 = (newpoints[1,1] - newpoints[1,0]) / (newpoints[0,1] - newpoints[0,0])
	m2 = (newpoints[1,3] - newpoints[1,2]) / (newpoints[0,3] - newpoints[0,2])
	b1 = newpoints[1,0] - m1*newpoints[0,0]
	b2 = newpoints[1,2] - m2*newpoints[0,2]
	#calculate intersection point of these lines
	xint = (b1-b2)/(m2-m1)
	yint = (m1*xint+b1)
	cv2.line(imgx, (int(newpoints[0,0]),int(newpoints[1,0])) , (int(xint),int(yint)) , (0,0,255),1)
	cv2.line(imgx, (int(newpoints[0,2]),int(newpoints[1,2])) , (int(xint),int(yint)) , (0,0,255),1)
	return xint, yint





#set filename and read it into an opencv object
img_location = 'course.jpeg'
img = cv2.imread(img_location)

#if the following gives an error, you are probably screwing up the filename
rows,cols,ch = img.shape
print rows, cols, ch

#set up clicking behavior on the raw image
cv2.namedWindow('xformed')
cv2.setMouseCallback('xformed',get_point)
#cv2.waitKey(0)

#sample_lines()

#calculate xform matrix
orig = np.float32([[556,481],[509,457],[783,481],[669,457]])
#assuming square is 227px big
guess = np.float32([[556,481],[556,254],[783,481],[783,254]])
M = getForwardMatrix(orig,guess)

#remap original image by applying transform matrix
imgx = cv2.warpPerspective(img,M,(800,800), flags = 1, borderMode = 0, borderValue = (255,0,0))

#resize this image
scaledimgx = scaleImage(imgx,np.floor(.4*227) ,53)

#figure out if the vertex is in the right location and draw helpful lines
xint, yint = find_vertex(M,imgx)
print xint, yint

#write imageb
cv2.imwrite('xformed.png',imgx)
cv2.imwrite('scaledxform.png',scaledimgx)

while(1):
    cv2.imshow('image',img)
    cv2.imshow('xformed',imgx)
    cv2.imshow('scaledxform',scaledimgx)


    if cv2.waitKey(20) & 0xFF == 27:
        break
cv2.destroyAllWindows()