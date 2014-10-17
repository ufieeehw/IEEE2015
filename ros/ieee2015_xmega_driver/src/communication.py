#!/usr/bin/python
from __future__ import division # Make all division floating point division

# Serial
import serial
import threading
import numpy

# Math
import numpy as np

# Ros
import rospy
import tf.transformations as tf_trans
import tf

# Ros Msgs
from std_msgs.msg import Header, Float32, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from ieee2015_xmega_driver.msg import XMega_Message


class Communicator(object):
    def __init__(self, port, baud_rate, verbose=True):
        self.verbose = verbose

        rospy.init_node('Xmega_Connector')
        
        # Messages being sent from ROS to the XMega
        self.poll_msg_sub = rospy.Subscriber('robot/send_xmega_poll_msg', String, self.got_poll_msg)
        self.data_msg_sub = rospy.Subscriber('robot/send_xmega_data_msg', XMega_Message, self.got_data_msg)

        # Messages being recieved from the XMega, sent to ROS
        self.accel_data_pub = rospy.Publisher('robot/receive_xmega_data', String, queue_size=1)

        self.serial = serial.Serial(port, baud_rate)
        # self.serial.setTimeout(1000)
        # Defines which action function to call on the received data
        self.action_dict = {
        }
        self.type_lengths = {
        }

        # Defines the relationship between poll_message names and the hex name
        self.poll_messages = {
            'example_poll_msg': '0F'
        }
        self.data_messages = {
        }
        # First two bits
        self.type_defs = {
            0b00000000: 0,
            0b01000000: 1,
            0b10000000: 2,
            # 0b11000000: None,
        }

    def err_log(self, *args):
        '''Print the inputs as a list if class verbosity is on
        '''
        if self.verbose:
            print args

    def got_poll_msg(self, msg):
        print "data", msg.data
        self.write_packet(msg.data)

    def got_data_msg(self, msg):
        self.write_packet(msg.type, msg.data)

    def read_packets(self):
        '''read_packets
        Function:
            Permanently loops, waiting for serial messages
        TODO:
            Add ros-based message send capability
        Notes:
            This does not handle desyncs

        '''
        type_length = 1  # Bytes
        length_length = 1  # Bytes

        while True:
            # message_length = 0 # Bytes (Defaulted, indicated by message type)
            msg_type = ord(self.serial.read(type_length))
            msg_byte_type = msg_type & 0b11000000

            self.err_log('Recieving message of type', msg_type)

            # Message of known length
            if msg_byte_type in self.type_defs.keys():
                msg_length = self.type_defs[msg_byte_type]
                if msg_length > 0:
                    msg_data = None
                else:
                    msg_data = self.serial.read(msg_length)

                action_function = self.action_dict[msg_type]
                action_function(msg_data)

            # N-Byte
            elif msg_type in self.action_dict.keys():
                action_function = self.action_dict[msg_type]

                self.err_log('Recognized type as', action_function.__name__)
                msg_length = self.serial.read(length_length)
                msg_data = self.serial.read(msg_length)
                self.err_log("Message content:", msg_data)

                action_function(msg_data)

            else:
                self.err_log('Did not recognize type', msg_type)

    def write_packet(self, _type, data=None):
        '''type is _type because "type" is a python protected name
        '''
        if _type in self.poll_messages.keys():
            self.err_log("Write type recognized as a polling message")
            write_data = self.poll_messages[_type]
            print write_data
            self.serial.write(str(unichr(write_data)))
        elif _type in self.data_messages.keys():
            self.err_log("Write type recognized as a data message")
        else:
            self.err_log("Write type not recognized")


class IEEE_Communicator(Communicator):
    def __init__(self, port='/dev/ttyUSB0', baud_rate=256000):
        '''
        '''
        super(self.__class__, self).__init__(port, baud_rate)

        # For messages of known length, get the length from this table
        # -> Josh, we should consider always sending message length
        self.type_lengths.update({
            '00': 0,
            '11': 25,
        })
        # Determine which action function to call on the received data
        self.action_dict.update({
            # C0 + $num_bytes + $message
            0xF0: self.got_xmega_error,
            0xC0: self.towbot_nunchuck_echo,
            0x40: self.got_test,
        })
        self.poll_messages.update({
            'poll_imu': 0x01,
        })
        self.data_messages.update({
            'debug': 0x40,
        })

    def got_xmega_error(self, msg_data):
        self.err_log("Got error,", msg_data)

    def got_test(self, data):
        print "Recieved test!"

    def towbot_nunchuck_echo(self, msg_data):
        '''Towbot nunchuck demo
        TODO: Rosify this
        1 byte: Stick X (80 is middle)
        2 byte: Stick Y (80 is middle)
        3 byte: Acc X
        4 byte: Acc Y
        5 byte: Acc Z
        6 byte: last 2 bits are C and Z buttons
        '''
        msg_format = ['Stick X', 'Stick Y', 'Acc X', 'Acc Y', 'Acc Z', 'Bullshit']
        for character, meaning in zip(msg_data, msg_format):
            print meaning + ':', character
        self.err_log("We're up in this shit towbot nunchuck!")

        self.write_packet('init_towbot_poll')
        return


if __name__=='__main__':
    Comms = IEEE_Communicator(port='/dev/ttyUSB0')
    Comms.read_packets()
    rospy.spin()