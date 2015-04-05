'''Authors: 
    Matt Feldman
    Brandon Peterson
    Jacob Panikulam
'''
import cv2
import numpy as np
from copy import deepcopy

class Transform(object):

    views = {
        '20_8.5': { # Degrees_height in cm
            'view_points': np.float32([
                (620, 220),
                (482, 147),
                (175, 146),
                (58,  220),
            ])
        },
        '20_27': {
            'view_points': np.float32([
                (502, 428),
                (430, 288),
                (177, 288),
                (100, 430),
            ]),
            'bounds': 600,
            'frame': {
                # (98, 0),
                # (458, 0),
                # (98, 258),
                # (458, 258),

                # x
                # 'x': (98, 458),
                'x': (118, 538),
                # 'y': (0, 260),
                'y': (0, 320),

            }

        }
    }
    cam_x = 640
    cam_y = 480

    def __init__(self, view, sq_size=50):
        
        map_coordinates = np.float32([
            [320 + sq_size, 240 + sq_size],
            [320 + sq_size, 240 - sq_size],
            [320 - sq_size, 240 - sq_size],
            [320 - sq_size, 240 + sq_size],
        ])

        view_points = view['view_points']
        self._perspective_matrix = self._get_perspective_matrix(view_points, map_coordinates)
        # self.bird_dims = self._get_bird_dims()
        self.crop = view['frame']

    @classmethod
    def get_view_points(self, angle, height):
        width = 0.3
        z_offset = -0.2

        view_points = [
            (616, 425, 1),
            (469, 350, 1),
            (169, 356, 1),
            (23,  430, 1),
        ]
        points = []
        for pt in view_points:
            pt = np.matrix(pt).T # Append a 1!

            back_projected = (np.matrix(self.c920_cam).I * pt).A1
            print ' one point', back_projected
            points.append(back_projected)


        objectPoints = map(lambda pt: np.array([pt], np.float32), points)

        rvec = angle * np.array((1.0, 0.0, 0.0), np.float32)
        tvec = np.array((0.0, 0.0, 0.0), np.float32)


        distCoeffs = np.array([0.169985, -0.401052, 0.005058, -0.00463, 0.0], np.float32)
        # distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        view_coordinates = []
        for i in range(4):
            img_point, _ = cv2.projectPoints(objectPoints[i], rvec, tvec, self.c920_cam, distCoeffs)
            view_coordinates.append(img_point[0][0])
        return np.float32(view_coordinates)

    @classmethod
    def _get_bird_dims(self):
        # Get bird_x from calculating where top-right corner maps to
        corner = np.dot(
            self._perspective_matrix, 
            [[self.cam_x],
             [0],
             [1]]
         )
        # Extract the new corner location by dividing by homogeneous coord
        bird_x = corner[0] / corner[2]
        # Get bird_y from calculating where bottom left corner maps to
        corner = np.dot(
            self._perspective_matrix, 
            [[0],
             [self.cam_y],
             [1]]
         )
        # Extract the new corner location by dividing by homogeneous coord
        bird_y = corner[1] / corner[2]
        return (bird_x, bird_y)

    @classmethod
    def _get_perspective_matrix(self, orig, guess):
        # Scale and move image around so that your location is equal to what
        # you defined in _robot_coordinates
        new = guess
        # Calculate the actual 3x3 matrix
        self._perspective_matrix = cv2.getPerspectiveTransform(orig, new)
        return self._perspective_matrix

    def transform_image(self, image, (bird_x, bird_y)): 
        #remap original image by applying transform matrix
        # print "Bird's Eye Coords ({} {})".format(bird_x, bird_y)
        imgx = cv2.warpPerspective(image, self._perspective_matrix,
                                   (bird_x, bird_y), flags=1, 
                                   borderMode=0, borderValue=(0, 0, 0))


        crop = self.crop
        # return imgx
        new_img = imgx[crop['y'][0]: crop['y'][1], crop['x'][0]: crop['x'][1]]
        return new_img


if __name__ == '__main__':
    import os
    from matplotlib import pyplot
    from time import time
    fpath = os.path.dirname(os.path.realpath(__file__))


    # Rubik's test
    # height = 0.086 # meters
    # height = height - 0.014 # Height of chess
    height = 0.085 # meters
    # height = height - 0.007 # Height of chess

    # angle = -0.0
    # angle = np.radians(20)
    angle = -0.4
    # view_points = Transform.get_view_points(angle, height)

    # map_coordinates = np.float32([[650, 650], [650, 350], [350, 350], [350, 650]])

    ''' Using our dict of known calibrations'''


    sq_size = 50
    map_coordinates = np.float32([
        [320 + sq_size, 240 + sq_size],
        [320 + sq_size, 240 - sq_size],
        [320 - sq_size, 240 - sq_size],
        [320 - sq_size, 240 + sq_size],
    ])

    view_points = Transform.views[20]['view_points']
    print view_points
    Transform._get_perspective_matrix(view_points, map_coordinates)
    (bird_dims) = Transform._get_bird_dims()

    cap = cv2.VideoCapture(1)
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here

        # Display the resulting frame
        key_press = cv2.waitKey(1)
        if key_press & 0xFF == ord('q'):
            break
        elif key_press & 0xFF == ord('f'):
            pyplot.imshow(frame)
            pyplot.show()

        tic = time()
        rected = frame
        # rected = Transform.rectify_image(frame)
        # cv2.imshow("Rectified", rected)

        xformed = cv2.resize(Transform.transform_image(rected, (800, 500)), (500, 500))
        toc = time() - tic
        print 'Rectification took {} seconds'.format(toc)

        print xformed.shape

        cv2.imshow("xformed", xformed)
