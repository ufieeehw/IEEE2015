''' IN PROGRESS'''
import numpy as np
from vispy import app

from vispy.geometry import create_cube
from vispy.util.transforms import perspective, translate, rotate, scale
from vispy.gloo import (Program, VertexBuffer, IndexBuffer, Texture2D, clear,
                        FrameBuffer, RenderBuffer, set_viewport, set_state, set_clear_color)

import rospy
import roslib
roslib.load_manifest('ieee2015_vision')
from ros_image_tools import Image_Publisher
import os

camera_vertex = """
uniform mat4 view;
uniform mat4 projection;
attribute vec3 position;
void main()
{
    gl_Position = projection * view * model * vec4(position,1.0);
}
"""

camera_fragment = """
varying vec2 v_texcoord;
void main()
{
    float r = texture2D(texture, v_texcoord).r;
    gl_FragColor = texture2D(texture, v_texcoord);
}
"""

class Camera(object):
    def __init__(self, topic='/camera', view=np.eye(4), projection=np.eye(4)):
        self.im_pub = Image_Publisher(topic=topic)
        self.view = view
        self.projection = projection
        self.fbo = FrameBuffer

    def set_projection(self, projection):
        '''Projection matrix'''
        self.projection = projection

    def set_view(self, view):
        '''View matrix'''
        self.view = view

    def draw(self):
        self.im_pub.publish(frame)