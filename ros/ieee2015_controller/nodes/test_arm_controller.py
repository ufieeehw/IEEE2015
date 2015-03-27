#!/usr/bin/python
import unittest
import numpy as np
from arm_controller import SCARA_Controller


'''Test if arm_controller is correctly outputting bad and good input'''
'''More test should be added, but I don't know all values that should be tested'''
'''test_correct may not work, checks if arm_controller returns an accurate value'''


class armTest(unittest.TestCase):

    def setUp(self):
        # Lengths and offsets from arm_controller 
        self.shoulder_length, self.elbow_length  = 0.148, 0.160
        self.base_height = 0
        self.shoulder_angle_offset = 0.55 - np.pi/2
        self.elbow_angle_offset = np.pi / 3

    def test_is_valid_one(self):
        # Checks that the published movments are good
        self.assertIsNotNone(SCARA.solve_angles((.1, .1)))

    def test_not_valid_one(self):
        # Checks that the controller retuns none if the coordinanets are bad
        self.assertIsNone(SCARA.solve_angles((-.5, -15)))

    def test_correct(self):
        # Checks to see if the calculated angles are accurate
        shoulder_angle, elbow_angle = SCARA.solve_angles((.1, .1))
        np.testing.assert_almost_equal((.1, .1), self.find_cord(shoulder_angle, elbow_angle)) 

    def find_cord(self, shoulder, elbow):
        # Forward kinematics
        shoulder_angle = shoulder + self.shoulder_angle_offset
        elbow_angle = np.pi - (elbow + self.elbow_angle_offset)
        result_x = self.shoulder_length * np.cos(shoulder_angle) + self.elbow_length * np.sin(elbow_angle)
        result_y = self.base_height + self.shoulder_length * np.sin(shoulder_angle) + self.elbow_length * np.cos(elbow_angle)
        return result_x, result_y


if __name__ == '__main__':
    import rostest
    SCARA = SCARA_Controller()
    rostest.rosrun('test_controller', 'test_arm_controller', armTest)
    unittest.main('''exit=False''')
    print SRA.solve_angles((-.5, -15))

