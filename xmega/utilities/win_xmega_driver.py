from __future__ import division # Make all division floating point division
'''How to use:
    go to ieee2015_xmega_driver and run
    python setup.py install
'''


# Misc
import os
import argparse
import time

import xmega_driver

class Xmega_Driver(object):
    def __init__(self, types_path, verbosity):
        '''Xmega driver -- handles the serial interface with the Xmega128A1U for the IEEE 2015 robot.
        This is the windows version that does not require or use ROS.
        It is purely for testing message types

        TODO:
            - Force message 'type' declarations (int, float, etc) in types.h so we can test 
                that it is the right type, and do our special casting
        '''
        self.serial_proxy = xmega_driver.Serial_Proxy(
            port='/COM1', 
            baud_rate=256000, 
            verbose=verbosity,
        )

        # Bind types.h info for the serial proxy
        self.serial_proxy.bind_types(types_path)

        # Bind callbacks for messages FROM the Xmega
        self.serial_proxy.bind_callback('vector_error', self.got_xmega_error)
        self.serial_proxy.bind_callback('message_error', self.got_xmega_error)
        self.serial_proxy.bind_callback('buffer_error', self.got_xmega_error)
        self.serial_proxy.bind_callback('imu_data', self.got_imu_reading)

        # Start message
        self.send_start_msg()

        # Relinquish control of the program - this is an infinite loop
        self.serial_proxy.run_serial_loop()

        self.start_user_loop()


    def send_start_msg(self):
        '''Send the start message that tells the XMega the robot is ready to begin working'''
        self.serial_proxy.add_message('start')

    def got_keepalive(self, msg_data=None):
        print 'Got keep alive: ', msg_data

    def got_xmega_error(self, msg_data):
        print "Got error,", msg_data

    def got_test(self, msg_data):
        print "Recieved test!"
        if msg_data is not None:
            print "Data:", msg_data

    def got_imu_reading(self, msg_data):
        print 'Got IMU message:', msg_data
        for character in msg_data:
            item_1 = ord(character)

    def start_user_loop(self):
        while(True):
            '''Files worth looking at:
                types.h
                serial_proxy.py in ieee2015_xmega_driver
            '''
            add_message('poll_imu')
            # ord(character) -> number from ascii
            # hex(number) -> hex value in string form
            time.sleep(0.1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(usage='hello', description='sadsad')
    parser.add_argument('-v', '--verbose', action='store_true',
                      help='Set verbosity of the xmega driver')
    args = parser.parse_args()
    this_file_path = os.path.dirname(os.path.realpath(__file__))
    verbosity = args.verbose

    types_path = os.path.join(this_file_path, '..', 'types.h')
    print 'Types Path:', types_path
    print 'Verbosity:', verbosity
    driver = Xmega_Driver(types_path, verbosity)
