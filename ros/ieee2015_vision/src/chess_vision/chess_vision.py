from . import Rotate
from . import baseLine
from . import findCoordinates
from . import Squares
from . import edgechesshc
from . import FindPiece
import cv2

##############This is handling Chess's vision################
#only method that directly deal with image processing will be called from here
#imported files associated with the image processing must also be included

#Vision just needs to return a new_occupancy_grid and the coordinates
#the rest will be taken care of in mission planner based off of the current architecture
global x
global y


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
        img = cv2.resize(img, (640, 480))
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
        #Squares.loadPieces(squares, board_state)

        squaresCropped = Squares.getSquaresCropped(squares, rotatedImage)

        new_occupancy_grid, array_of_coordinates = FindPiece.main(squaresCropped)

        return new_occupancy_grid, array_of_coordinates


def giveForsythEdwardsNotation(board_state, w_or_b_turn):
    x = 0
    y = 8
    rows = []

    #this for loop stores the board state in 8 rows of 8 elements
    for i in range(0, 8):
        rows.append(board_state[x:y])
        x = x + 8
        y = y + 8

    #this for loop takes care of representing empty square with numbers
    counter = 0
    for j in range(0, 8):
        if (rows[j] == 'e'):
            counter = counter + 1
            rows[j] = ' '
            if(j == 7):
                rows[j] = str(counter)
        else:
            if(counter != 0):
                rows[j - 1] = str(counter)
            counter = 0

    #this for loop just makes a string for each row to pass to chess ai
    string = []
    for k in range(0, 8):
        string.append("".join(rows[k]))

    backslash = "/"
    #this is just the way I thought of to combine everything into one huge string
    combined__board_string = string[0] + backslash + string[1] + backslash + string[2] + backslash + string[3] + backslash + string[4] + backslash + string[5] + backslash + string[6] + backslash + string[7]

    #this last steps adds in whose turn it is so that ai knows
    finalFE = combined__board_string + " " + w_or_b_turn
    print finalFE
    return finalFE


        ##########################################################End of creating board part of FE Notation###################################################

def findDifferencesArrays(old_occupancy_grid, new_occupancy_grid, board_state):
    missingPiece = ''
    indexOfFound = -1

    for i in range(0, 64):
        #this first if statement should be called no matter what
        #for a piece to move or to make a kill
        #the piece must orignally be in a square and then
        #no longer be in that same square
        if ((old_occupancy_grid[i] == 1 or old_occupancy_grid[i] == 2) and new_occupancy_grid[i] == 0):
            #this call tells us if a piece is no longer where it once was
            #hence we have found the "missing piece"
            missingPiece = board_state[i]
            #then we need to set the board_state back to empty since
            #we have already identified it as empty
            board_state[i] = 'e'

        #this elif is the case in which a square becomes occupied
        elif (old_occupancy_grid[i] == 0 and (new_occupancy_grid[i] == 1 or new_occupancy_grid[i] == 2)):
            #we know that we have found our missing piece
            #we need to eventually replace our board_state
            #with our missing piece so we need the index of where it moved
            indexOfFound = i

        #FOR THE NEXT TWO ELIFS, THESE ARE THE CASES IN WHICH A PIECE IS KILLED
        #TO IDENTIFY OUR FOUND PIECE WE SPECIFICALLY NEED TO KNOW WHAT COLOR

        #this elif statement is saying that a blue piece killed an orange piece
        #we have found a change, hence where our missing piece moved to
        elif (old_occupancy_grid[i] == 1 and new_occupancy_grid[i] == 2):
            indexOfFound = i

        #this elif is if an orange killed a blue piece
        elif (old_occupancy_grid[i] == 2 and new_occupancy_grid[i] == 1):
            indexOfFound = i

        board_state[indexOfFound] = missingPiece

    return board_state


def changePlayer(w_or_b_turn):
    if (w_or_b_turn == 'w'):
        w_or_b_turn = 'b'
        return w_or_b_turn
    elif (w_or_b_turn == 'b'):
        w_or_b_turn = 'w'
        return w_or_b_turn
    else:
        print 'yeah there is a big problem....ya derp'
