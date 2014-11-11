import cv2
import numpy as np
import math


class Squares:
	#Initialize square with it's boundries
	def __init__(self, topLeft, bottomRight):
		self.topLeft = topLeft
		self.bottomRight = bottomRight
		self.hasPiece = False
		self.pieceType = 'e'

		# took this out because w_or_b_turn is an overall field, not important to an individual square self.w_or_b_turn = 'w'

	#Checks to see if a given coordinate is within it's bounds
	def cropSquaresHelper(self, newImage):
		x = self.topLeft[0]
		y = self.topLeft[1]

		deltaX = self.bottomRight[0] - x
		deltaY = self.bottomRight[1] - y
		
		crop_img = newImage[y: y + deltaY, x:x + deltaX]
		return crop_img

## figure out what to do if a piece is taken
## a piece is only taken if the square of interest changes colors
def findDifferencesArrays(oldBool, newBool, pieceTypeArray, oldPieceColor_Array, newPieceColor_Array):
	missingPiece = ''
	indexOfFound = -1

	#WHOLE POINT OF THIS IS TO KEEP AN UPDATED PIECE TYPE ARRAY
	for i in range(0, 64):
		if (oldBool[i] == True and newBool[i] == False): #This shoul work no matter what because a kill move or just moving to an empty square fits this requirement
			missingPiece=pieceType[i]
			pieceTypeArray[i]='e'
		if ((oldBool[i] == False and newBool[i] == True) and (oldPieceColor_Array[i] == 8 and newPieceColor_Array[i] != 8)):
			indexOfFound = i
		#There are only two possible circumstances in which a piece is killed, orange where blue once was and blue where orange once was
		elif ((oldBool[i] == True and newBool[i] == True) and (oldPieceColor_Array[i] ==  6 and newPieceColor_Array[i] == 7)): #6 is blue and 7 is orange
			#this elif takes care of the case that an orange is now where a blue piece used to be
			indexOfFound == i #we need to update the pieceTypeArray with the piece that the piececolorarraysis
		elif ((oldBool[i] == True and newBool[i] == True) and (oldPieceColor_Array[i] ==  7 and newPieceColor_Array[i] == 6)):
			#this elif takes care of the case that a blue piece is now where an orange piece was
			indexOfFound == i
		pieceTypeArray[indexOfFound] = missingPiece

	return newBool, pieceType



def loadPieces(squares, booleanArray, pieceTypeArray):
	for i in range(0, 64):
		squares[i].hasPiece = booleanArray[i]			
		squares[i].pieceType = pieceTypeArray[i]
	
def initSquares(coordinates):
	allSquares = []
	j=0
	justHappened = True
	for i in range(0, 64):
		tempSquare = Squares(coordinates[j], coordinates[j + 10])
		allSquares.append(tempSquare)
		if (i - (j-i)) % 7 == 0 and (justHappened == False):
			j += 2
			justHappened = True
		else:
			j += 1
			justHappened = False

	return allSquares

def cropSquares(squares, newImage):
	croppedSquares = []
	for i in range(0, 64):
		croppedSquares.append(squares[i].cropSquaresHelper(newImage))
	
	return croppedSquares

def printCroppedSquares(croppedArray):
	print len(croppedArray)
	for i in range(0, len(croppedArray)):
		windowName = "Window%d" % (i)			
		cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
		cv2.imshow(windowName, croppedArray[i])

'''
#Testing only
def centerOfSquare(allSquares):
	
	filename = 'lib/rotated.jpg'
	img = cv2.imread(filename)

	for i in range(0, 64):
		k = int((allSquares[i].bottomRight[0] + allSquares[i].topLeft[0]) * .5)
		j = int((allSquares[i].bottomRight[1] + allSquares[i].topLeft[1]) * .5)
		img[j][k] = [0, 0, 255]

	cv2.namedWindow('dots', cv2.WINDOW_NORMAL)
	cv2.imshow('dots',img)
'''
		

def populateSquares(coordinates):
	squares = initSquares(coordinates)
	return squares

	#centerOfSquare(squares)
def getSquaresCropped(squares, newImage):
	squaresCropped = cropSquares(squares, newImage)
	return squaresCropped
	#printCroppedSquares(squaresCropped)

def giveForsythEdwardsNotation(pieceTypeArray, w_or_b_turn):
#need final product to look like this: rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1
	
################################################################FOR MAKING THE BOARD PART OF FORSYTH EDWARDS##################################
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

	for i in range(0,8):
		row1.append(pieceTypeArray[i])

	for i in range(8, 16):
		row2.append(pieceTypeArray[i])

	for i in range(16, 24):
		row3.append(pieceTypeArray[i])

	for i in range(24,32):
		row4.append(pieceTypeArray[i])

	for i in range(32,40):
		row5.append(pieceTypeArray[i])

	for i in range(40,48):
		row6.append(pieceTypeArray[i])

	for i in range(48,56):
		row7.append(pieceTypeArray[i])

	for i in range(56,64):
		row8.append(pieceTypeArray[i])


	##############converting e to integer needed#####################3

	for i in range(0,8):
		if(row1[i] == 'e'):
			counter1 = counter1 + 1
			row1[i] = ''
			if(i == 7):
				row1[i]=str(counter1)
		else:
			if(counter1 != 0):
				row1[i-1] = str(counter1)
			counter1 = 0

	for i in range(0,8):
		if(row2[i] == 'e'):
			counter2 = counter2 + 1
			row2[i] = ''
			if(i == 7):
				row2[i]=str(counter2)
		else:
			if(counter2 != 0):
				row2[i-1] = str(counter2)
			counter2 = 0

	for i in range(0,8):
		if(row3[i] == 'e'):
			counter3 = counter3 + 1
			row3[i] = ''
			if(i == 7):
				row3[i]=str(counter3)
		else:
			if(counter3 != 0):
				row3[i-1] = str(counter3)
			counter3 = 0

	for i in range(0,8):
		if(row4[i] == 'e'):
			counter4 = counter4 + 1
			row4[i] = ''
			if(i == 7):
				row4[i]=str(counter4)
		else:
			if(counter4 != 0):
				row4[i-1] = str(counter4)
			counter4 = 0

	for i in range(0,8):
		if(row5[i] == 'e'):
			counter5 = counter5 + 1
			row5[i] = ''
			if(i == 7):
				row5[i]=str(counter5)
		else:
			if(counter5 != 0):
				row[i-1] = str(counter5)
			counter5 = 0

	for i in range(0,8):
		if(row6[i] == 'e'):
			counter6 = counter6 + 1
			row6[i] = ''
			if(i == 7):
				row6[i]=str(counter6)
		else:
			if(counter6 != 0):
				row[i-1] = str(counter6)
			counter6 = 0


	for i in range(0,8):
		if(row7[i] == 'e'):
			counter7 = counter7 + 1
			row7[i] = ''
			if(i == 7):
				row7[i]=str(counter7)
		else:
			if(counter7 != 0):
				row7[i-1] = str(counter7)
			counter7 = 0

	for i in range(0,8):
		if(row8[i] == 'e'):
			counter8 = counter8 + 1
			row8[i] = ''
			if(i == 7):
				row8[i]=str(counter8)
		else:
			if(counter8 != 0):
				row[i-1] = str(counter8)
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