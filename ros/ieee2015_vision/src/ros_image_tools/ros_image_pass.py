#!/usr/bin/python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
'''ros_image_pass
This is a module that has an image publisher and an image reciever object that does shit in ROS
'''


class Image_Publisher(object):
    def __init__(self, topic="camera", encoding="bgr8", queue_size=1):
        '''Image Publisher -> Image_Publisher('/camera')'''
        self.im_pub = rospy.Publisher(topic, Image, queue_size=queue_size)
        self.bridge = CvBridge()    
        self.encoding = encoding
    
    def publish(self, cv_image):
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, self.encoding)#,desired_encoding="passthrough")
            self.im_pub.publish(image_message)
        except CvBridgeError, e:
            print e


class Image_Subscriber(object):
    def __init__(self, topic="camera", callback=None, encoding="bgr8", queue_size=1):
        '''Image_Subscriber('/camera', callback_function)
        Will call `callback_function` on each image every time a new image is published on `topic`
        Assumes topic of type "sensor_msgs/Image"'''
        self.encoding = encoding
        self.im_sub = rospy.Subscriber(topic, Image, self.convert, queue_size=queue_size)
        self.bridge = CvBridge()
        self.image = None
        self.callback = callback

    def convert(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
            self.callback(self.image)
        except CvBridgeError, e:
            print e
