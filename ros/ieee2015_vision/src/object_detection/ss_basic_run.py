import cv2
import ss_get_lit_button
import ss_find_button
import ss_get_axis_points

#this is the simplified version of sion says for the one button thing
#will be improved and other more robust is being worked on but this ill get us going

#hold numeric value correspoinding to the color that lit up
colors_played = []
#holds pixel coordinate value of button
coord_colors = []
#this array hold the current coordinates that need to be pushed for each button
#changes based of current image/if toy moves
angle = 0
point = []
center_of_button = []

def get_std_fields(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	#anything with Cent in variable name is the midpoint
	#major and minor refer to the axis
	#quarter refers to quarter point, may be unecessary in future
	#dst is rotated Image
	#x, y is the center point of object
	#angle is the angle of the object's orientation
	angle, points, center_of_button = ss_get_axis_points.get_axis_points(img)
	center_of_button.append(get_center_circle(img, points))

#gets our color coordinate for lit up button
def add_color(img):
	#detects the lit button coord center
	mean_cols, mean_rows, closing = ss_get_lit_button.get_lit_button(img)
	
	#using angle relative to minor axis finds which button was pushed
	detected_button = ss_find_button.find_button(img, mean_cols, mean_rows)
	
	#populating arrays
	colors_played.append(detected_button)
	tempPoint = (mean_cols, mean_rows)
	coord_colors.append(tempPoint)

#returns the current coordinate of colors to push, the color array
def get_color_coord():
	return coord_colors

img = cv2.imread('Images/Set3/snorm4.JPG')
add_color(img)