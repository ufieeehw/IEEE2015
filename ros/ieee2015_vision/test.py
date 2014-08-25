'''Todo: Write some goddamn unit tests'''
import unittest
import sys
from rectangles import RectangleFinder
PKG = 'test_roslaunch'

class Test_Rectangle_Finder(unittest.TestCase):
    def setUp(self):
        pass

    def test_find_rectangles(self):
        # Test that we detect everything

        # Test that there's the right number of detections

        # Test that we're withing 50 pixels for each rectangle

        pass

    def test_transform_rectangles(self):
        # Test that the transformed dimensions are ~right

        # Test that the result is actually what we expect
        # How to test his?
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_rectangle_finder', Test_Rectangle_Finder)

