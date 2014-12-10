#!/usr/bin/python
## Input
import argparse
## Display
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pygame
import time
import threading
## Math
import numpy as np
## Ros
import rospy
from tf import transformations as tf_trans
## Data Structures
from collections import deque, OrderedDict
## Ros Msgs
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from sensor_msgs.msg import Imu

lock = threading.Lock()

def thread_lock(function_to_lock):
    '''thread_lock(function) -> locked function 
    Thread locking decorator

        If you use this as a decorator for a function, it will apply a threading lock during the execution of that function,

        Which guarantees that no ROS callbacks can change the state of data while it is executing. This
            is critical to make sure that a new message being sent doesn't cause a weird serial interruption
    '''
    def locked_function(*args, **kwargs):
        # Get threading lock
        lock.acquire()
        # Call the decorated function as normal with its input arguments
        result = function_to_lock(*args, **kwargs)
        # Release the lock
        lock.release()
        # Return, pretending the function hasn't changed at all
        return result
    # Return the function with locking added
    return locked_function


class IMU_Visualizer(object):
    _storage_length = 100
    def __init__(self, topic, name='Plot Visualization', plot_items=None):
        node = rospy.init_node('arm_visualizer', anonymous=True)

        self.plot_name = name
        self.start_time = rospy.Time.now()
        self.times = deque()
        self.name = name

        # Check if nobody is plotting anything
        if plot_items is None:
            return
        self.plot_items = plot_items
        self.plot_dict = OrderedDict()
        for item in self.plot_items:
            self.plot_dict[item] = deque()

        self.state_sub = rospy.Subscriber(topic, Imu, self.got_state)

        # Set up and start the plot
        fig = plt.figure()
        self.ax = fig.add_subplot(1, 1, 1)
        ani = animation.FuncAnimation(fig, self.draw, interval=100)
        plt.show()

    def set_plotter(self, fig, ax):
        self.fig = fig
        self.ax = ax        

    def get_sub_item(self, msg, attribute):
        if '.' in attribute:
            return reduce(lambda a, b: getattr(a, b), attribute.split('.'), msg)
        else:
            return getattr(msg, attribute)

    @thread_lock
    def got_state(self, msg):
        # Rolling queue for time
        if len(self.times) > self._storage_length:
            self.times.popleft()
        self.times.append(msg.header.stamp.to_time())

        # Rolling queue for everything else
        for item_name, item_queue in self.plot_dict.items():
            if len(item_queue) > self._storage_length:
                item_queue.popleft()
            # This is equivalent to doing msg.item_name, except item_name is a string
            # CRITICAL!
            item_value = self.get_sub_item(msg, item_name)
            item_queue.append(item_value)

    @thread_lock
    def draw(self, i):
        # x = np.array(self.times) - self.start_time.to_time()
        x = np.array(self.plot_dict['angular_velocity.x'], np.float64)
        if len(x) < 10:
            return
        self.ax.clear()
        self.ax.grid(color='r', which='major', axis='both', linestyle='--', linewidth=1)
        # for item_name, item_queue in self.plot_dict.items():
            # y = np.array(item_queue)
            # self.ax.plot(x, y, '-', label=item_name)
        y = np.array(self.plot_dict['angular_velocity.y'], np.float64)
        self.ax.plot(x, y, '-', label='angular velocity')
        self.ax.set_autoscale_on(False)
        self.ax.axis([-34000, 34000, -34000, 34000])
        # self.ax.legend()
        plt.title(self.name)


if __name__ == '__main__':
    usage_msg = 'Do a live plot of the arm behaviors'
    desc_msg = "Default plot items: linear acceleration and velocity"

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument('--topic', dest='topic',
                      help='Choose a topic to listen to')
    parser.add_argument('--name', dest='name',
                      help='Choose the title of the plot')

    args = parser.parse_args(rospy.myargv()[1:])
    name = args.name
    topic = args.topic
    if name is None:
        name = 'Magnetometer XY'
    IV = IMU_Visualizer(topic, name=name,
        plot_items=[
            'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z',
            'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z'
        ])

