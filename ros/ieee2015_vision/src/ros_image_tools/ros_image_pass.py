#!/usr/bin/python
import rospy
import cv2, cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
'''ros_image_pass
This is a module that has an image publisher and an image reciever object that does shit in ROS
'''

class Image_Publisher(object):
    def __init__(self, topic_name="Camera"):

        self.im_pub = rospy.Publisher(topic_name, Image, queue_size=5)
        self.bridge = CvBridge()    
    
    def publish(self, cv_image):
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")#,desired_encoding="passthrough")
            self.im_pub.publish(image_message)
        except CvBridgeError, e:
            print e


class Image_Subscriber(object):
    def __init__(self, topic_name="Camera", callback=None):
        '''Assumes topic of type "sensor_msgs/Image"'''
        self.im_sub = rospy.Subscriber(topic_name, Image, self.convert, queue_size=5)
        self.bridge = CvBridge()
        self.image = None
        self.callback = callback

    def convert(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.callback(self.image)
        except CvBridgeError, e:
            print e
