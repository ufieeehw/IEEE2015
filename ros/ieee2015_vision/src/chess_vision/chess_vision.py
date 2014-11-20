import Rotate
import baseLine
import findCoordinates
import Squares
import edgechesshc
import FindPiece

##############This is handling Chess's vision################
#only method that directly deal with image processing will be called from here
#imported files associated with the image processing must also be included

#Vision just needs to return a new_occupancy_grid and the coordinates
#the rest will be taken care of in mission planner based off of the current architecture


def get_Occupancy_and_Coordinates(img):
		########################################################
		#this part needs to take in an image when Mission Planner tells it to
		#ergo it needs to be changed, or does mission planenr pass in a
		#ready to go image?
		#
		#filename = 'img'
		#img = cv2.imread(filename)
		#gray = cv2.imread(filename, 0)
		########################################################

		#Get the angle of rotation needed and rotate
		#rotated image is really the thing we need because it will be the best final image to use
		theta = baseLine.findAngle(img)
		rotatedImage = Rotate.rotateImage(theta)

		#Find all the lines of the squares
		imgWithEdges = edgechesshc.addEdges(rotatedImage)

		#Get all the coordinates of the keypoints
		coordinates = findCoordinates.getCoordinates(imgWithEdges)

		#Pass coordinates on to Squares
		squares = Squares.populateSquares(coordinates)
		
		#believe this step is no longer necessary because of the architecture
		#Load squares with their starting state
		#Squares.loadPieces(squares, pieceTypeArray)

		squaresCropped = Squares.getSquaresCropped(squares, rotatedImage)

		new_occupancy_grid, array_of_coordinates = FindPiece.main(squaresCropped)

		
		return new_occupancy_grid, array_of_coordinates


def giveForsythEdwardsNotation(board_state, w_or_b_turn):
		###############I know 100% there is a way to consolidate this but for right now this is what I'm going to do###################3
		###Initialize row stuff for final concatenated string
		row1 = []
		row2 = []
		row3 = []
		row4 = []
		row5 = []
		row6 = []
		row7 = []
		row8 = []

		string1 = ""
		string2 = ""
		string3 = ""
		string4 = ""
		string5 = ""
		string6 = ""
		string7 = ""
		string8 = ""

		counter1 = 0
		counter2 = 0
		counter3 = 0
		counter4 = 0
		counter5 = 0
		counter6 = 0
		counter7 = 0
		counter8 = 0

		for i in range(0, 8):
		    row1.append(pieceTypeArray[i])

		for i in range(8, 16):
		    row2.append(pieceTypeArray[i])

		for i in range(16, 24):
		    row3.append(pieceTypeArray[i])

		for i in range(24, 32):
		    row4.append(pieceTypeArray[i])

		for i in range(32, 40):
		    row5.append(pieceTypeArray[i])

		for i in range(40, 48):
		    row6.append(pieceTypeArray[i])

		for i in range(48, 56):
		    row7.append(pieceTypeArray[i])

		for i in range(56, 64):
		    row8.append(pieceTypeArray[i])

		##############converting e to integer needed#####################3

		for i in range(0, 8):
		    if(row1[i] == 'e'):
		        counter1 = counter1 + 1
		        row1[i] = ''
		        if(i == 7):
		            row1[i] = str(counter1)
		    else:
		        if(counter1 != 0):
		            row1[i - 1] = str(counter1)
		        counter1 = 0

		for i in range(0, 8):
		    if(row2[i] == 'e'):
		        counter2 = counter2 + 1
		        row2[i] = ''
		        if(i == 7):
		            row2[i] = str(counter2)
		    else:
		        if(counter2 != 0):
		            row2[i - 1] = str(counter2)
		        counter2 = 0

		for i in range(0, 8):
		    if(row3[i] == 'e'):
		        counter3 = counter3 + 1
		        row3[i] = ''
		        if(i == 7):
		            row3[i] = str(counter3)
		    else:
		        if(counter3 != 0):
		            row3[i - 1] = str(counter3)
		        counter3 = 0

		for i in range(0, 8):
		    if(row4[i] == 'e'):
		        counter4 = counter4 + 1
		        row4[i] = ''
		        if(i == 7):
		            row4[i] = str(counter4)
		    else:
		        if(counter4 != 0):
		            row4[i - 1] = str(counter4)
		        counter4 = 0

		for i in range(0, 8):
		    if(row5[i] == 'e'):
		        counter5 = counter5 + 1
		        row5[i] = ''
		        if(i == 7):
		            row5[i] = str(counter5)
		    else:
		        if(counter5 != 0):
		            row5[i - 1] = str(counter5)
		        counter5 = 0

		for i in range(0, 8):
		    if(row6[i] == 'e'):
		        counter6 = counter6 + 1
		        row6[i] = ''
		        if(i == 7):
		            row6[i] = str(counter6)
		    else:
		        if(counter6 != 0):
		            row6[i - 1] = str(counter6)
		        counter6 = 0

		for i in range(0, 8):
		    if(row7[i] == 'e'):
		        counter7 = counter7 + 1
		        row7[i] = ''
		        if(i == 7):
		            row7[i] = str(counter7)
		    else:
		        if(counter7 != 0):
		            row7[i - 1] = str(counter7)
		        counter7 = 0

		for i in range(0, 8):
		    if(row8[i] == 'e'):
		        counter8 = counter8 + 1
		        row8[i] = ''
		        if(i == 7):
		            row8[i] = str(counter8)
		    else:
		        if(counter8 != 0):
		            row8[i - 1] = str(counter8)
		        counter8 = 0

		string1 = ''.join(row1)
		string2 = ''.join(row2)
		string3 = ''.join(row3)
		string4 = ''.join(row4)
		string5 = ''.join(row5)
		string6 = ''.join(row6)
		string7 = ''.join(row7)
		string8 = ''.join(row8)

		print string5

		backslash = "/"
		combined__board_string = string1 + backslash + string2 + backslash + string3 + backslash + string4 + backslash + string5 + backslash + string6 + backslash + string7 + backslash + string8

		finalFE = combined__board_string + " " + w_or_b_turn
		print finalFE
		return finalFE

		##########################################################End of creating board part of FE Notation###################################################
