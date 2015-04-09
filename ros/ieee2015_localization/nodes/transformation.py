#!/usr/bin/env python
from ros_image_tools import Image_Subscriber, Image_Publisher
from slam import Transform
import numpy as np
import rospy
import cv2
from matplotlib import pyplot
from time import time


class Test_Transform(object):
    def __init__(self):
        rospy.init_node('test_transform')
        height = 0.085 # meters
        angle = -0.4
        # view_points = Transform.get_view_points(angle, height)

        ''' Using our dict of known calibrations'''
        self.im_sub = Image_Subscriber('/robot/base_camera/image_rect', self.image_callback, queue_size=1)
        self.view_pub = Image_Publisher('/robot/base_camera/down_view', encoding="8UC1", queue_size=1)

        # sq_size = 10
        # map_coordinates = np.float32([
        #     [320 + sq_size, 240 + sq_size],
        #     [320 + sq_size, 240 - sq_size],
        #     [320 - sq_size, 240 - sq_size],
        #     [320 - sq_size, 240 + sq_size],
        # ])

        self.view = Transform.views['20_half_max']
        self.transformer = Transform(view=self.view, sq_size=10)

    def image_callback(self, rected):
        rected = rected[:, :, 2]
        # bounds = self.view['bounds']
        xformed = self.transformer.transform_image(rected, (1000, 1000))
        cv2.imshow("rected", rected)
        cv2.imshow("xformed", xformed)
        # fuct = xformed[310-(280 - 135):310, 135:280]
        # fuct = xformed[378 - (296 - 156):378, 156:296]
        fuct = xformed[194:345, 100:290]
        
        cv2.imshow("fuct", fuct)

        key_press = cv2.waitKey(1)
        if key_press & 0xFF == ord('q'):
            exit()
        elif key_press & 0xFF == ord('f'):
            pyplot.imshow(xformed)
            pyplot.show()

        self.view_pub.publish(xformed)


if __name__ == '__main__':
    tt = Test_Transform()
    rospy.spin()