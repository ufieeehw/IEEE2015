import cv2
import Rotate
import baseLine
import findCoordinates
import Squares
import edgechesshc
import FindPiece
import chess_ai


ROBOTCOLOR = True
finalFE = ""
#w_or_b_turn = 'w' #needs to change after every iteration of this call
#ROBOTCOLOR = True ########### WILL NEVER CHANGE BECAUSE WE ARE THE BLUE PLAYER

class VisionBoard:
	def __init__(self):
		######################FOR THIS TO WORK, ORANGE IS GOING TO HAVE TO START AT TOP OF IMAGE#########################
		self.old_piece_attribute = []
		#old piece attribute will be what we move new piece attribute into after we've run through everything

		self.new_piece_attribute = [1, 1, 1, 1, 1, 1, 1, 1,
							   		1, 1, 1, 1, 1, 1, 1, 1,
							   		0, 0, 0, 0, 0, 0, 0, 0,
							   		0, 0, 0, 0, 0, 0, 0, 0,
							   		0, 0, 0, 0, 0, 0, 0, 0,
							   		0, 0, 0, 0, 0, 0, 0, 0,
							   		2, 2, 2, 2, 2, 2, 2, 2,
							   		2, 2, 2, 2, 2, 2, 2, 2] 
		#new piece attribute is what we are goingt o have the taken in image values saved in, for initializing we need it ot be the new_piece_attribute


		self.pieceTypeArray = ['r','n','b','q','k','b','n','r',
						  		'p','p','p','p','p','p','p','p',
						  		'e','e','e','e','e','e','e','e',
						  		'e','e','e','e','e','e','e','e',
						  		'e','e','e','e','e','e','e','e',
						  		'e','e','e','e','e','e','e','e',
						  		'P','P','P','P','P','P','P','P',
						  		'R','N','B','Q','K','B','N','R']

		self.finalFE = ""
		self.w_or_b_turn = 'w'
		self.ROBOTCOLOR = True


	def showImage(self, windowName, imageName):
		cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
		cv2.imshow(windowName, imageName)

	#everytime we need to use a new image we must run it through this method to initalize a proper square
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
		Squares.loadPieces(squares, self.pieceTypeArray)

		squaresCropped = Squares.getSquaresCropped(squares, rotatedImage)

		self.new_piece_attribute, array_of_coordinates = FindPiece.main(squaresCropped)

		
		return squares	
		#at the end of all repeated processes we need to shift all the "new"
		#stuff into the "old" stuff we are using for reference and comparison
		#Find where the peices lie on the board
##########################################WHOLE POINT OF THIS IS TO KEEP AN UPDATED PIECE TYPE ARRAY################################################
	def findDifferencesArrays(old_piece_attribute, new_piece_attribute, pieceTypeArray):
		missingPiece = ''
		indexOfFound = -1

		#WHOLE POINT OF THIS IS TO KEEP AN UPDATED PIECE TYPE ARRAY

		for i in range(0, 64):
			#this first if statement should be called no matter what 
			#for a piece to move or to make a kill
			#the piece must orignally be in a square and then 
			#no longer be in that same square
			if ((old_piece_attribute[i] == 1 or old_piece_attribute[i] == 2) and new_piece_attribute[i] == 0):
				#this call tells us if a piece is no longer where it once was
				#hence we have found the "missing piece" 
				missingPiece = pieceTypeArray[i]
				#then we need to set the pieceTypeArray back to empty since 
				#we have already identified it as empty
				pieceTypeArray[i] ='e'

			#this elif is the case in which a square becomes occupied
			elif (old_piece_attribute[i] == 0 and (new_piece_attribute[i] == 1 or new_piece_attribute[i] == 2)):
				#we know that we have found our missing piece
				#we need to eventually replace our pieceTypeArray
				#with our missing piece so we need the index of where it moved
				indexOfFound = i

			#FOR THE NEXT TWO ELIFS, THESE ARE THE CASES IN WHICH A PIECE IS KILLED
			#TO IDENTIFY OUR FOUND PIECE WE SPECIFICALLY NEED TO KNOW WHAT COLOR

			#this elif statement is saying that a blue piece killed an orange piece
			#we have found a change, hence where our missing piece moved to 
			elif (old_piece_attribute[i] == 1 and new_piece_attribute[i] == 2): 
				indexOfFound = i 

			#this elif is if an orange killed a blue piece
			elif (old_piece_attribute[i] == 2 and new_piece_attribute[i] == 1):
				indexOfFound = i
				
			pieceTypeArray[indexOfFound] = missingPiece

		return pieceTypeArray

	
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
	ai_move = chess_ai.get_chess_move(finalFE, ROBOTCOLOR)  
