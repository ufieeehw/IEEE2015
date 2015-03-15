import cv2
 
#1 is yellow
#2 is green
#3 is red
#4 is blue
#-1 error as always
def getColorButtonCoord(colorArray, minXL, maxXL, minYL, maxYL, 
     		minXR, maxXR, minYR, maxYR):
	for currentColor in colorArray:
			xCoord = 0
			yCoord = 0
			if currentColor == 1:  #yellow
				#get row coord
				if(maxXL > maxXR):
					xCoord = maxXL + 300  #arbitrary pixel value
					
				else:
					xCoord = maxXR + 300

				#get col coord
				yCoord = (maxYL + minYR)/2

			elif currentColor == 2:  #green
				#get row coord
				xCoord = (minXL + maxXL)/2
				
				#get col coord
				yCoord = (maxYL + minYR)/2

			elif currentColor == 3:  #red
				#get row coord
				if(minXL > minXR):
					xCoord = minXL - 300  #arbitrary pixel value
					
				else:
					xCoord = minXR - 300
					
				#get col coord
				yCoord = (maxYL + minYR)/2

			else:  #blue
				xCoord = (minXR + maxXR)/2
				
				#get col coord
				yCoord = (maxYR + minYR)/2

	return xCoord, yCoord