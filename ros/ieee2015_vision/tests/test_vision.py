#!/usr/bin/python
import rospy
from object_detection import rubix
from object_detection import detect_card
from object_detection import etchaSketch_detect
from object_detection import ss_get_axis_points
from object_detection import ss_get_lit_button
from ros_image_tools import Image_Subscriber
import cv2

'''How to use this:

roslaunch ieee2015_launch base_camera.launch
rosrun ieee2015_vision test_vision.py


Remove: waitKey(0) --> Blocking
Remove: Prints, use rospy.loginfo or rospy.logwarn
Remove: Things that run when imported
Remove: Extraneous imshow
Remove: Uses of cv instead of cv2 (dependency issue)

'''


def image_callback(image):
    cv2.imshow("Input Image", image)
    # find_rubix(image, 0.3)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('test_vision')
    imager = Image_Subscriber('/robot/base_camera/image_raw', image_callback)
    rospy.spin()