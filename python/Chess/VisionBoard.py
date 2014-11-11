import cv2
import numpy as np 
import math
import Rotate
import baseLine
import findCoordinates
import Squares
import edgechesshc
import FindPiece
import chess_ai

'''
##### anything labeled current is going to be replaced eventually with the new imag
currentBool_board = [True, True, True, True,True, True, True, True,
				True, True, True, True,True, True, True, True, 
				False, False, False, False, False, False, False, False,
				False, False, False, False,False, False, False, False,
				False, False, False, False,False, False, False, False,
				False, False, False, False,False, False, False, False,
				True, True, True, True,True, True, True, True,
				True, True, True, True,True, True, True, True]


currentPieceType_board = ['r','n','b','q','k','b','n','r',
				  		  'p','p','p','p','p','p','p','p',
				  	      'e','e','e','e','e','e','e','e',
				  		  'e','e','e','e','e','e','e','e',
				  	      'e','e','e','e','e','e','e','e',
				  		  'e','e','e','e','e','e','e','e',
				  		  'P','P','P','P','P','P','P','P',
				  		  'R','N','B','Q','K','B','N','R']
'''


finalFE = ""
#w_or_b_turn = 'w' #needs to change after every iteration of this call
#ROBOTCOLOR = True ########### WILL NEVER CHANGE BECAUSE WE ARE THE BLUE PLAYER

class VisionBoard:
	def __init__(self):
		self.currentBool_board = [True, True, True, True,True, True, True, True,
								 True, True, True, True,True, True, True, True, 
								False, False, False, False, False, False, False, False,
								False, False, False, False,False, False, False, False,
								False, False, False, False,False, False, False, False,
								False, False, False, False,False, False, False, False,
								True, True, True, True,True, True, True, True,
								True, True, True, True,True, True, True, True] #needed for comparison
		self.newBool_board = []#
		self.currentPieceType_board  = ['r','n','b','q','k','b','n','r',
				  		  				'p','p','p','p','p','p','p','p',
				  	      				'e','e','e','e','e','e','e','e',
				  		  				'e','e','e','e','e','e','e','e',
				  	      				'e','e','e','e','e','e','e','e',
				  		  				'e','e','e','e','e','e','e','e',
				  		  				'P','P','P','P','P','P','P','P',
				  		  				'R','N','B','Q','K','B','N','R']
		#self.newPieceType_board = newPieceType_board dont think we need one because we just update the current
		self.ROBOTCOLOR = True
		self.w_or_b_turn = 'w'


	def showImage(self, windowName, imageName):
		cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
		cv2.imshow(windowName, imageName)

	#everytime we need to use a new image we must run it through this method to initalize a proper square
	def makeSquareObjects(self):
		#######################initialize##############################
		#Take in image file
		'''
		take in file img to initialize
		then run it throught the same 
		'''
		#somehow filename needs to take in a new image work through tomorrow
		filename = 'lib/chessanglesmall.jpg'
		img = cv2.imread(filename)
		gray = cv2.imread(filename, 0)

		#Get the angle of rotation needed and rotate
		#rotated image is really the thing we need because it will be the best final image to use
		theta = baseLine.findAngle(filename)
		rotatedImage = Rotate.rotateImage(theta)

		#Save the rotated image
		cv2.imwrite('lib/rotated.jpg', rotatedImage)

		#Find all the lines of the squares
		imgWithEdges = edgechesshc.addEdges('lib/rotated.jpg')

		#Get all the coordinates of the keypoints
		coordinates = findCoordinates.getCoordinates(imgWithEdges)

		#Pass coordinates on to Squares

		squares = Squares.populateSquares(coordinates)
		
		#Load squares with their starting state
		Squares.loadPieces(squares, self.currentBool_board, self.currentPieceType_board)

		squaresCropped = Squares.getSquaresCropped(squares, rotatedImage)

		arrayOfBooleans, centerOfPiece, array_of_colors = FindPiece.main(squaresCropped)

		self.newBool_board = arrayOfBooleans
		self.newPieceColor_Array = array_of_colors
		
		return squares
		########################repeatedProcess############################
		
	def makeSquareObjects(self, img):
		#######################initialize##############################
		#Take in image file
		'''
		take in file img to initialize
		then run it throught the same 
		'''
		#somehow filename needs to take in a new image work through tomorrow
		filename = 'img'
		img = cv2.imread(filename)
		gray = cv2.imread(filename, 0)

		#Get the angle of rotation needed and rotate
		#rotated image is really the thing we need because it will be the best final image to use
		theta = baseLine.findAngle(filename)
		rotatedImage = Rotate.rotateImage(theta)

		#Save the rotated image
		cv2.imwrite('lib/rotated.jpg', rotatedImage)

		#Find all the lines of the squares
		imgWithEdges = edgechesshc.addEdges('lib/rotated.jpg')

		#Get all the coordinates of the keypoints
		coordinates = findCoordinates.getCoordinates(imgWithEdges)

		#Pass coordinates on to Squares

		squares = Squares.populateSquares(coordinates)
		
		#Load squares with their starting state
		Squares.loadPieces(squares, self.currentBool_board, self.currentPieceType_board)

		squaresCropped = Squares.getSquaresCropped(squares, rotatedImage)

		arrayOfBooleans, centerOfPiece, array_of_colors = FindPiece.main(squaresCropped)

		self.newBool_board = arrayOfBooleans
		self.newPieceColor_Array = array_of_colors
		
		return squares	
		#at the end of all repeated processes we need to shift all the "new"
		#stuff into the "old" stuff we are using for reference and comparison
		#Find where the peices lie on the board
##########################################WHOLE POINT OF THIS IS TO KEEP AN UPDATED PIECE TYPE ARRAY################################################
	def findDifferencesArrays(oldBool, newBool, pieceTypeArray, oldPieceColor_Array, newPieceColor_Array):
		missingPiece = ''
		indexOfFound = -1

		#WHOLE POINT OF THIS IS TO KEEP AN UPDATED PIECE TYPE ARRAY
		for i in range(0, 64):
			if (oldBool[i] == True and newBool[i] == False): #This shoul work no matter what because a kill move or just moving to an empty square fits this requirement
				missingPiece=pieceType[i]
				self.currentPieceType_board[i]='e'
			if ((oldBool[i] == False and newBool[i] == True) and (oldPieceColor_Array[i] == 8 and newPieceColor_Array[i] != 8)):
				indexOfFound = i
			#There are only two possible circumstances in which a piece is killed, orange where blue once was and blue where orange once was
			elif ((oldBool[i] == True and newBool[i] == True) and (oldPieceColor_Array[i] ==  6 and newPieceColor_Array[i] == 7)): #6 is blue and 7 is orange
				#this elif takes care of the case that an orange is now where a blue piece used to be
				indexOfFound == i #we need to update the pieceTypeArray with the piece that the piececolorarraysis
			elif ((oldBool[i] == True and newBool[i] == True) and (oldPieceColor_Array[i] ==  7 and newPieceColor_Array[i] == 6)):
				#this elif takes care of the case that a blue piece is now where an orange piece was
				indexOfFound == i
			self.currentPieceType_board[indexOfFound] = missingPiece

	
		#replace with updated image

		#the arrayOfBooleans is T or F if there is a piece in a square
		#centerofpeice is an array of coordinates for the cetner of each piece (catch is it has to be perfectly centered on top of the piece)


		#print arrayOfBooleans
		#print centerOfPiece
		#print array_of_colors

		#print pieceTypeArray
		#Manage old and new images first
		#pieceTypeArray = findDifferencesArrays(oldBool, newBool, pieceType)
		
		####################################finalProcess to start up again####################################3
		####We have to come up with a way to determine who moved, white or balck so that we can send it to AI
		###########we will have to constantly update piecetype array as well and boolAray
	def makeFE(self, currentPieceType_board, w_or_b_turn):
		finalFE = Squares.giveForsythEdwardsNotation(currentPieceType_board, w_or_b_turn)
		return finalFE

	def changePlayers(self, w_or_b_turn):
		if (w_or_b_turn == 'w'):
			self.w_or_b_turn == 'b'
		elif(w_or_b_turn == 'b'):
			self.w_or_b_turn == 'w'
	
def makeMove(finalFE):
	ai_move = chess_ai.get_chess_move(finalFE, self.ROBOTCOLOR)  





'''

		print ai_move
		############################Testing##############################
		#Show all the important images
		showImage('original', img)
		showImage('rotated', rotatedImage)
		showImage('image with edges', imgWithEdges)


		#Show all the cropped images
		#Squares.printCroppedSquares(squaresCropped)

		cv2.waitKey(0)
		cv2.destroyAllWindows()

'''
	

