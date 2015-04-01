import cv2
import ss_get_lit_button
import ss_find_button
import ss_get_axis_points
#import ss_findColor
#import ss_getStandardState

######################The following are fields for Simon
#this list holds the colors in order
#different integers represent the different numbers
#1 is blue/right
#2 is red/up
#3 is green/left
#4 is yellow/down
#-1 error as always
#colors played holds the number for what button has lit up
colors_played = []
#this array hold the current coordinates that need to be pushed for each button
#changes based of current image/if toy moves
push_array = []

#following variable define the coordinates for where the robot must push
up_button = (0, 0)
left_button = (0, 0)
right_button = (0, 0)
down_button = (0, 0)

def get_std_coordinates(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	#anything with Cent in variable name is the midpoint
	#major and minor refer to the axis
	#quarter refers to quarter point, may be unecessary in future
	#dst is rotated Image
	#x, y is the center point of object
	#angle is the angle of the object's orientation
	angle, points, x, y, p1CentMajor, p2CentMajor, p1CentMinor, p2CentMinor, quarterMin1, quarterMin2, quarterMaj1, quarterMaj2 = ss_get_axis_points.get_axis_points(img)


#call this when it lights up
def add_color(img):
	mean_cols, mean_rows, closing = ss_get_lit_button.get_lit_button(img)

	detected_button = ss_find_button.find_button(mean_cols, mean_rows, img)

	colors_played.append(detected_button)

	print colors_played

	tempPoint = (mean_cols, mean_rows)

	push_array.append(tempPoint)


def set_button_locations(img):
	###THis needs to find places to push buttons, using center circle i guess

img = cv2.imread('Images/Set3/snorm9.JPG')
img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
add_color(img)