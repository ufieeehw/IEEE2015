#!/usr/bin/env python
from ros_image_tools import Image_Subscriber, Image_Publisher
from slam import Transform
import numpy as np
import rospy
import cv2
from matplotlib import pyplot
from time import time

def nothing(x):
    pass

DEBUG = True

if DEBUG:
    rospy.logwarn('Transformation in debug mode')
    cv2.namedWindow('image')
    # create trackbars for color change
    cv2.createTrackbar('top_dist', 'image', 40, 80, nothing)
    cv2.createTrackbar('bot_dist', 'image', 40, 80, nothing)


class Test_Transform(object):
    def __init__(self):
        rospy.init_node('test_transform')
        height = 0.085 # meters
        angle = -0.4
        # view_points = Transform.get_view_points(angle, height)

        ''' Using our dict of known calibrations'''
        self.im_sub = Image_Subscriber('/robot/base_camera/image_rect', self.image_callback, queue_size=1)
        self.view_pub = Image_Publisher('/robot/base_camera/down_view', encoding="8UC1", queue_size=1)

        self.view = Transform.views['20_half_max']
        self.transformer = Transform(view=self.view, sq_size=10)
        self.im_num = 0

    def update(self):
        top = cv2.getTrackbarPos('top_dist', 'image') - 40
        bot = cv2.getTrackbarPos('bot_dist', 'image') - 40

        # self.view = Transform.views['20_half_max']
        self.view = {
            'view_points': np.float32([
                (440 - bot, 213),
                (390 - top, 165),
                (210 + top, 165),
                (170 + bot, 213),
            ]),
            'frame': {
                'x': (95, 557),
                'y': (0, 484),
            }
        }
        self.transformer = Transform(view=self.view, sq_size=30)

    def image_callback(self, rected):
        # self.update()
        # for point in self.view_['view_points']

        rected = rected[:, :, 2]

        # bounds = self.view['bounds']
        xformed = self.transformer.transform_image(rected, (1000, 1000))

        if DEBUG:
            cv2.imshow("rected", rected)
            cv2.imshow("xformed", xformed)
        # xform_cropped = xformed[310-(280 - 135):310, 135:280]
        # xform_cropped = xformed[378 - (296 - 156):378, 156:296]
        xform_cropped = xformed[194:345, 100:290]
        
        # cv2.imshow("xform_cropped", xform_cropped)

        key_press = cv2.waitKey(1)
        if key_press & 0xFF == ord('q'):
            exit()
        elif key_press & 0xFF == ord('f'):
            pyplot.imshow(xformed)
            pyplot.show()
        elif key_press & 0xFF == ord('p'):
            cv2.imwrite('rected{}.png'.format(self.im_num), rected)
            cv2.imwrite('xformed{}.png'.format(self.im_num), xformed)
            self.im_num += 1

        self.view_pub.publish(xformed)


if __name__ == '__main__':
    tt = Test_Transform()
    rospy.spin()