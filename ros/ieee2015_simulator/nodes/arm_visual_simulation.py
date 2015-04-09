#!/usr/bin/env python
'''Simulated view node for testing SLAM'''

import numpy as np
from vispy import app

from vispy.geometry import create_cube
from vispy.util.transforms import perspective, translate, rotate, scale, zrotate, yrotate, xrotate
from vispy.gloo import (Program, VertexBuffer, IndexBuffer, Texture2D, clear,
                        FrameBuffer, RenderBuffer, set_viewport, set_state, set_clear_color)

import rospy
import roslib
roslib.load_manifest('ieee2015_vision')
from ros_image_tools import Image_Publisher
import cv2
import os

cube_vertex = """
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
attribute vec3 position;
attribute vec2 texcoord;
varying vec2 v_texcoord;
void main()
{
    gl_Position = projection * view * model * vec4(position, 1.0);
    v_texcoord = texcoord;
}
"""

cube_fragment = """
uniform sampler2D texture;
varying vec2 v_texcoord;
void main()
{
    gl_FragColor = texture2D(texture, v_texcoord);
}
"""

quad_vertex = """
attribute vec2 position;
attribute vec2 texcoord;
varying vec2 v_texcoord;
void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    v_texcoord = texcoord;
}
"""

quad_fragment = """
uniform sampler2D texture;
varying vec2 v_texcoord;
void main()
{
    gl_FragColor = texture2D(texture, v_texcoord).bgra;

}
"""

# Needs to become the Rubix cube
def rubixTexture(grid_num=8, grid_size=32):
    row_even = grid_num // 2 * [0, 1]
    row_odd = grid_num // 2 * [1, 0]
    Z = np.row_stack(grid_num // 2 * (row_even, row_odd)).astype(np.uint8)
    return 255 * Z.repeat(grid_size, axis=0).repeat(grid_size, axis=1)

fpath = os.path.dirname(os.path.realpath(__file__))
img_path = os.path.join(fpath, "..", "data")


class Canvas(app.Canvas):

    def __init__(self):
        app.Canvas.__init__(self, title='Framebuffer post-processing',
                            keys='interactive', size=(640, 640))
        rospy.init_node('simulated_arm_view')
        self.img_pub = Image_Publisher('/sim/arm_camera/image_rect', encoding='8UC1')

    def on_initialize(self, event):
        # Build cube data
        # --------------------------------------

        texcoord = [(0, 0), (0, 1), (1, 0), (1, 1)]
        vertices = [(-1, -1, 0), (-1, +1, 0), (+1, -1, 0), (+1, +1, 0)]

        vertices = VertexBuffer(vertices)

        camera_pitch = 0.0 # Degrees
        self.rotate = [camera_pitch, 0, 0]
        self.translate = [0, 0, -3]

        # Build program
        # --------------------------------------
        view = np.eye(4, dtype=np.float32)
        model = np.eye(4, dtype=np.float32)
        scale(model, 1, 1, 1)
        self.phi = 0

        self.cube = Program(cube_vertex, cube_fragment)
        self.cube['position'] = vertices
        # 4640 x 2256
        imtex = cv2.imread(os.path.join(img_path, 'rubixFront.jpg')) 
        self.cube['texcoord'] = texcoord
        self.cube["texture"] = np.uint8(np.clip(imtex + np.random.randint(-60, 20, size=imtex.shape), 0, 255)) + 5
        self.cube["texture"].interpolation = 'linear'
        self.cube['model'] = model
        self.cube['view'] = view


        color = Texture2D((640, 640, 3), interpolation='linear')
        self.framebuffer = FrameBuffer(color, RenderBuffer((640, 640)))

        self.quad = Program(quad_vertex, quad_fragment)
        self.quad['texcoord'] = [(0, 0), (0, 1), (1, 0), (1, 1)]

        self.quad['position'] = [(-1, -1), (-1, +1), (+1, -1), (+1, +1)]

        self.quad['texture'] = color

        self.objects = [self.cube]

        # OpenGL and Timer initalization
        # --------------------------------------
        set_state(clear_color=(.3, .3, .35, 1), depth_test=True)
        self.timer = app.Timer('auto', connect=self.on_timer, start=True)
        self.pub_timer = app.Timer(0.1, connect=self.send_ros_img, start=True)
        self._set_projection(self.size)

    def send_ros_img(self, event):
        # Read frame buffer, get rid of alpha channel
        img = self.framebuffer.read()[:, :, 2]
        self.img_pub.publish(img)

    def on_draw(self, event):

        # Image rect
        with self.framebuffer:
            set_clear_color('gray')
            clear(color=True, depth=True)
            set_state(depth_test=True)
            self.cube.draw('triangle_strip')

        set_clear_color('pink')
        clear(color=True)
        set_state(depth_test=True)
        self.quad.draw('triangle_strip')

    def on_resize(self, event):
        self._set_projection(event.size)

    def _set_projection(self, size):
        width, height = self.size
        set_viewport(0, 0, width, height)
        projection = perspective(100.0, width / float(height), 1.0, 20.0)
        self.cube['projection'] = projection

    def on_timer(self, event):

        model = np.eye(4, dtype=np.float32)
        #scale(model, 1, 1, 1)
        self.cube['model'] = model

        self.view = np.eye(4)
        xrotate(self.view, self.rotate[0])
        yrotate(self.view, self.rotate[1])
        zrotate(self.view, self.rotate[2])
        translate(self.view, *self.translate)

        self.cube['view'] = self.view

        self.update()

    
    def on_key_press(self, event):
        """Controls -
        a(A) - move left
        d(D) - move right
        w(W) - move up
        s(S) - move down"""
        

        if(event.text.lower() == 'p'):
            print(self.view)

        elif(event.text.lower() == 'd'):
            self.translate[0] += -0.1
        elif(event.text.lower() == 'a'):
            self.translate[0] += 0.1

        elif(event.text.lower() == 'w'):
            self.translate[1] += -0.1
        elif(event.text.lower() == 's'):
            self.translate[1] += 0.1

        elif(event.text.lower() == 'q'):
            self.rotate[2] += -1
        elif(event.text.lower() == 'e'):
            self.rotate[2] += 1

        elif(event.text.lower() == 'z'):
            self.translate[2] += -0.1
        elif(event.text.lower() == 'x'):
            self.translate[2] += 0.1


        #translate(self.view, self.translate)
        #rotate(self.view, self)


if __name__ == '__main__':
    c = Canvas()
    c.show()
    c.app.run()