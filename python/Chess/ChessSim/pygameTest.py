import sys, pygame
import os
from pygame.locals import *

pieceType_Board = ['r','n','b','q','k','b','n','r',
					'p','p','p','p','p','p','p','p',
					'e','e','e','e','e','e','e','e',
					'e','e','e','e','e','e','e','e',
				   	'e','e','e','e','e','e','e','e',
				   	'e','e','e','e','e','e','e','e',
				   	'P','P','P','P','P','P','P','P',
				   	'R','N','B','Q','K','B','N','R']

def printBoardAndPieces(list):
	board = pygame.image.load("Chessboard.png")

	screen.blit(pygame.transform.scale(board, (800,800)), (0,0))
	#draws the screen
	pygame.display.flip()
	x = 0
	y = 0
	for element in list:
		
		#GET CHESS PIECES
		if element != 'e':
			#WHITE PIECES
			if element == 'r':
				piece = pygame.image.load("wRook.png")
			if element == 'n':
				piece = pygame.image.load("wKnight.png")
			if element == 'b':
				piece = pygame.image.load("wBiship.png")
			if element == "q":
				piece = pygame.image.load("wQueen.png")
			if element == 'k':
				piece = pygame.image.load("wKing.png")
			if element == 'p':
				piece = pygame.image.load("wPawn.png")
		

			#BLACK PIECES
			if element == 'R':
				piece = pygame.image.load("bRook.png")
			if element == 'N':
				piece = pygame.image.load("bKnight.png")
			if element == 'B':
				piece = pygame.image.load("bBiship.png")
			if element == 'Q':
				piece = pygame.image.load("bQueen.png")
			if element == 'K':
				piece = pygame.image.load("bKing.png")
			if element == 'P':
				piece = pygame.image.load("bPawn.png")

			#prints image to screen and resizes it
			screen.blit(pygame.transform.scale(piece, (100,100)), (x,y))

		#COORDINATE CONTROL
		if x == 700:
			#reset x
			x = 0
			y += 100
		else:	
			x += 100

		pygame.display.flip()

	return



pygame.init()

size = width, height = 800, 800

screen = pygame.display.set_mode((size), HWSURFACE|DOUBLEBUF|RESIZABLE)
#load image into board

printBoardAndPieces(pieceType_Board)


#keeps screen open
while(1):
	for event in pygame.event.get():
		if event.type == pygame.QUIT:sys.exit()



	'''
	while x < width:
		y = 0;
		while Y < length:
			#screen.blit(piece, (x,y))
			y+=100
		x+=100
	'''	