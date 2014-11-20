#!/usr/bin/python
import rospy
import cv2, cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
'''ros_image_pass
This is a module that has an image publisher and an image reciever object that does shit in ROS
'''

class image_publisher:
    def __init__(self, topic_name="Camera"):

        self.im_pub = rospy.Publisher(topic_name, Image)
        self.bridge = CvBridge()    
    
    def send_message(self, cv_image):
        try:
            image_message = self.bridge.cv_to_imgmsg(cv.fromarray(cv_image), "bgr8")#,desired_encoding="passthrough")
            self.im_pub.publish(image_message)
        except CvBridgeError, e:
            print e

class image_subscriber(object):
    def __init__(self, topic_name="Camera"):
        self.im_sub = rospy.Subscriber(topic_name, Image, convert)
        self.bridge = CvBridge()
        self.image = None

    def convert(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data,"bgr8")
            self.image = cv_image
        except CvBridgeError, e:
            print e
