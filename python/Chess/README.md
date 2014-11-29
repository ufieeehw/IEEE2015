ROBOT CHESS:

Overview - Following files are for playing computer chess. The rough steps to make this happen are as follows:

*Board information is initialized with an occupancy grid containing ones and zeros
*Board state is initialized with each piece represented by a character stored in an array of length 64
	-this is how we will know which piece is what
	-will never be replaced entirely, just elements will be changed to represent changing board
	-how AI will know what the board looks like
*Take in the image and resize it to 640x480 
	-at the moment this is the necessary size, will improve upon later
*Rotate the image so that the Orange pieces are at the top of the image 
	-obviously, the pieces will eventually not be all on one side
	-the idea is to have the same side of the board at the top of the image
*Segment the board so that each square is an individual object 
	-Objects stored as one big array
*Process each square object and determine if there is a piece contained within it
	-The numeric system is as follows:
		- 0 is assigned if piece is empty
		- 1 is assigned if orange is detected
		- 2 is assigned if blue is detected
*the array of numeric values is then compared to the previous array of numeric values stored in memory
*the difference between the two arrays is then accounted for in the board state
*board state is passed to AI
(AI then tells what needs to be moved and the arm moves to the board square)
(following happens once arm has moved to indicated board square)
*Takes in the image of the square from directly above the piece
*Determines the center of the piece
	-this is so that the arm has a more precise position in which to grab the piece
(Arm moves peice to designated square)
*take in one more image to make sure board state matches AI decision

END OF COMPUTER VISION NEEDS

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TO RUN AND TEST CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

**To run in conjunction with ROS see ReadMe in ros file

To test images and see if a correct Forsyth Edwards notation is being returned, run everything out of the file main.py
Simply replace the file name on line 52 and run it

Chess_vision.py is the file to be used in conjunction with ROS, but main is for testing outside of the scope of ROS
