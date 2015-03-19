#!/usr/bin/env python
import cv2
import rospy
import roslib
roslib.load_manifest('ieee2015_vision')
from ros_image_tools import Image_Subscriber
roslib.load_manifest('ieee2015_localization')
from slam.registration import similarity

class Localization(object):
    def __init__(self):
        rospy.init_node('localization')
        self.image_sub = Image_Subscriber('camera', self.image_cb)
        self.image = None

    def image_cb(self, image):
        image = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (150, 150))
        if self.image is None:
            self.image = image
            return

        cv2.imshow('Image!', self.image)
        print self.image.shape, image.shape
        matched, scale, angle, (t0, t1) = similarity(self.image, image)
        print 'Changes; scale: {}, angle: {}, t0: {}, t1: {}'.format(scale, angle, t0, t1)
        cv2.imshow('Other!', matched)
        cv2.waitKey(1)

if __name__ == '__main__':
    localization = Localization()
    rospy.spin()