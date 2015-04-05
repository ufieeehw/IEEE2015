import cv2
import ss_get_axis_points_test
import ss_get_lit_button_test

def test_point_finder(img, height):
	angle, points, goodcircle = ss_get_axis_points_test.get_axis_points(img, height)
	centerpoint_bright = ss_get_lit_button_test.get_lit_button(img)

img = cv2.imread('ti/22hss.jpg')
test_point_finder(img, .22)