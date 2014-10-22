import numpy as np
from copy import deepcopy
from numpy.linalg import inv
import cv2


class VisualCortex:

    
    ####################################
    #####  1. LIST OF PROPERTIES   #####
    ####################################
    # Map image of the course
    full_map = None;
    # Dimensions of full course
    full_map_x = 2000
    full_map_y = 2000
    # Robot's position and rotation
    position = None;
    rotation = None;
    # Dimensions of images that come from camera feed
    cam_x = None;
    cam_y = None;
    # Dimensions of camera images after perspec transform (bird's eye dims)
    bird_x = None;
    bird_y = None;

    # Some useful private variables
    _affine_matrix = None;
    _map_features = None;
    _view_features = None;
    _transformed_view = None;
    _perspective_matrix = None;
    #position on a perspec transformed img that corresponds to robot's loc
    _robot_coordinates = [[400],
                          [800],
                          [1]
                         ];


    ####################################
    #####      2. INITIALIZER      #####
    ####################################
    """Initialize a VisualCortex object. 

    This sets the full_map property to a blank map of mxn dimensions,
    the position and rotation to (x,y) and 0 on this full map

    This also takes in a set of four coordinates from the camera's
    perspective image and four coordinates of where these should map to on 
    the bird's eye image and calculates the perspective transform matrix.

    It then uses this transform matrix to determine how large of an image
    we need to fit transformed images.
    """
    def __init__(self, view_coordinates, map_coordinates, img):
        self.full_map= np.ones((self.full_map_x, self.full_map_y), np.uint8);
        self.position = [1500, 2500];
        self.rotation = 0;

        # Read off the dimensions of images we are getting from camera
        self._initialize_cam_dimensions(img)

        # Get the perspec transform matrix
        self._perspective_matrix = self._get_perspective_matrix(
                                        view_coordinates, map_coordinates);
        # Now that we have a matrix, get the bird's eye image dimensions
        self._get_bird_dims()
        # Transform the initial image to bird's eye
        imgx = self.transform_image(img)
        # And paste it on to the full_map using a basic translation matrix
        self._initialize_affine_matrix()
        self.stitch(imgx)


    ####################################
    #####     3. PUBLIC METHODS    #####
    ####################################

    """Takes in the next input image and does feature detection, mapping,
    localization, and stitching to the full_map in one shot
    """
    def SLAM(self, image):
        # Run perspective transform on image
        imgx = self.transform_image(image)
        # Extract features from this image
        kp1, des1 = self.feature_detect(imgx)
        kp1, des1 = self._apply_roi(kp1, des1)
        # TODO: Use a database rather than re-ORBing on the full map
        # Extract features from the full_map
        kp2, des2 = self.feature_detect(self.full_map)
        # Match features between imgx and full_map
        M, matches = self.feature_match(kp1, kp2, des1, des2)
        self._draw_matches(imgx,self.full_map,kp1,kp2,matches,[0,1,2,3])
        # Use matches to get the affine transform
        self.get_affine_matrix(kp1, kp2, matches)
        # Use this affine matrix to update robot position
        self.localize()
        # Use this affine matrix to update full_map
        self.stitch(imgx)
        return;

    """Takes an image from the camera's video feed and transforms it into a 
    bird's eye view, returning this transformed image
    """
    def transform_image(self, image): 
        #remap original image by applying transform matrix
        imgx = cv2.warpPerspective(image, self._perspective_matrix,
                                   (self.bird_x,self.bird_y), flags = 1, 
                                   borderMode = 0, borderValue = (0,0,0))
        # TODO: See if there is a better way to get rid of pixelation that
        # comes from the interpolation of transformed pixels
        # Sharpen the image
        imgx = cv2.bilateralFilter(imgx,9,75,75)
        return imgx;

    """Find the features/corners of interest of an input image, 
    which can be either the bird's eye image or full_map, and returns the 
    keypoints array and respective descriptors matrix
    """    
    def feature_detect(self, image):
        # Initialize detector
        orb = cv2.ORB()
        # Find the keypoints and descriptors with SIFT
        kp, des = orb.detectAndCompute(image,None)
        return kp, des;

    """Given a set of two keypoint arrays and their respective descriptor 
    matrices, return the array of match objects after masking
    and the resulting homography matrix from BruteForce matching
    """
    def feature_match(self, kp1, kp2, des1, des2):
        # Create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # Match descriptors and get the matches array
        matches = bf.match(des1,des2)
        # Sort them in the order of their distance.
        # TODO: change the lambda to a def, as specified by PEP8
        matches = sorted(matches, key = lambda x: x.distance)

        # Optionally store all the good matches as per Lowe's ratio test.
        good = []
        for m in matches:
            #if m.distance < 0.7*n.distance:
                good.append(m)

        # Make sure we found matches, and then use the matches objects to 
        # extract lists of points that correspond to the matches and
        # determine the mask array based on locations.  Use this mask to
        # filter the full list of matches
        if len(good)>0:
            query_pts = np.float32(
                [ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            train_pts = np.float32(
                [ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            # Run regression on the matches to filter outliers
            M, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC,20.0)
            matches_mask = mask.ravel().tolist()
            # Use this mask immediately to get the good matches
            good = self._filter_matches(good, matches_mask)
        else:
                print "Not enough matches are found - %d/%d" % (len(good), 10)
                matches_mask = None
                M = None
        return M, good

    """Using the matches and mask between the full_map and perspec transformed
    image, determine the affine transformation that will map the smaller
    onto the bigger.
    """
    def get_affine_matrix(self, kp1, kp2, matches):
        # Indices of which points in "matches" to use
        point1 = 0;
        point2 = 1;
        point3 = 2;
        # Gather the coordinates in train img that we go TO
        trainPts = np.float32([ kp2[matches[point1].trainIdx].pt,
                                kp2[matches[point2].trainIdx].pt,
                                kp2[matches[point3].trainIdx].pt
                              ]
                             )
        # Gather the coordinates in query that we come FROM
        queryPts = np.float32([ kp1[matches[point1].queryIdx].pt,
                                kp1[matches[point2].queryIdx].pt,
                                kp1[matches[point3].queryIdx].pt
                              ]
                             )
        # Transform the query image
        self._affine_matrix = cv2.getAffineTransform(queryPts,trainPts);
        return None;
        
    """Apply the latest _affine_matrix to the _robot_coordinates so that we
    can update the robot's location in the full_map
    """
    def localize(self):
        self.position = np.dot(self._affine_matrix, self._robot_coordinates)
        # TODO: Figure out how to extract rotation from the affine matrix
        self.rotation = 0;
        return None;

    """ Update the full_map with the latest image, using the
    most up-to-date _affine_matrix"""
    def stitch(self, imgx):
        # Transform imgx into something the size of the full map using affine
        rows, cols = self.full_map.shape
        fullimgx = cv2.warpAffine(imgx, self._affine_matrix, (cols,rows))
        fullimg2 = self.full_map
        # Add the images together
        self.full_map = fullimgx/2 + self.full_map/2;
        # Clean up the "ghosting" on the resulting image
        dump, self.full_map = cv2.threshold(self.full_map, 50, 255,
            cv2.THRESH_BINARY)
        #superimposed = cv2.resize(superimposed,(0,0), fx=.25, fy=.25)
        return None;


    ####################################
    #####    4. PRIVATE METHODS    #####
    ####################################

    # Read off the dimensions of an image
    def _initialize_cam_dimensions(self, image):
        self.cam_y, self.cam_x = image.shape
        return

    # Calculate the perspective transform matrix during initialization
    def _get_perspective_matrix(self, orig, guess):
        # Scale and move image around so that your location is equal to what
        # you defined in _robot_coordinates
        new = self._tune_output_square(.5,[-285.7, -368],guess)
        # Calculate the actual 3x3 matrix
        return cv2.getPerspectiveTransform(orig, new)

    # For playing with the output position of the perspective transformed img
    # This lets you scale and translate the dst quadrilateral
    def _tune_output_square(self, scale,translate,points):
        newpoints = deepcopy(points)
        avg_x = (points[0][0] + points[1][0] + points[2][0] + points[3][0])/4
        avg_y = (points[0][1] + points[1][1] + points[2][1] + points[3][1])/4
        for n in range(0,4):
            newpoints[n][0] = avg_x - (avg_x - points[n][0])*scale + translate[0]
            newpoints[n][1] = avg_y - (avg_y - points[n][1])*scale + translate[1]
        return newpoints

    def _initialize_affine_matrix(self):
        self._affine_matrix = np.float32([[1, 0, (self.full_map_x-self.bird_x)/2],
                                          [0, 1, (self.full_map_y-self.bird_y)/2]
                                         ]
                                        )
        return;

    # Figure out the dimensions of the image that the perspective transform
    # maps to.
    def _get_bird_dims(self):
        # Get bird_x from calculating where top-right corner maps to
        corner = np.dot(self._perspective_matrix, 
                        [[self.cam_x],
                         [0],
                         [1]
                        ])
        # Extract the new corner location by dividing by homogeneous coord
        self.bird_x = corner[0] / corner[2]
        # Get bird_y from calculating where bottom left corner maps to
        corner = np.dot(self._perspective_matrix, 
                        [[0],
                         [self.cam_y],
                         [1]
                        ])
        # Extract the new corner location by dividing by homogeneous coord
        self.bird_y = corner[1] / corner[2]
        return

    # Applies a range-of-interest mask over the kp and des because
    # we don't want the corners of the transformed trapezoid to show 
    # up as features
    def _apply_roi(self,kp,des):
        # TODO: These coordinates are static, so we don't need to keep
        # recalculating every time we run _apply_roi
        # Figure out where the corners of the perspective image map to in the
        # bird's eye image
        c = 5     # Number of pixels to cushion the border with
        perspec_corners = [[c, self.cam_x - c, self.cam_x - c, c             ],
                           [c, c             , self.cam_y - c, self.cam_y - c],
                           [1, 1             , 1             , 1             ]
                          ]
        bird_homo_corners = np.dot(self._perspective_matrix, perspec_corners)
        # Initialize the output corners
        bird_corners = [[1, 1, 1, 1],
                        [1, 1, 1, 1]]
        # Populate the above matrix
        for col in range(0,4):
            for row in range(0,2):
                bird_corners[row][col] = bird_homo_corners[row][col] / bird_homo_corners[2][col]

        # TODO: Figure out a better way to check if points are inside polygon
        # TODO: Make the following chunk more readable
        # Check each kp to see if it is in the polygon defined by bird_corners
        # by checking if its x-position falls between the diagonal lines
        # and its y-position is between the horizontal lines
        include = []
        for i in range(0,len(kp)):
            keep = 0
            if ((kp[i].pt[1] > bird_corners[1][0]) and (kp[i].pt[1] < bird_corners[1][2])):
                # Calculate left bound at the given y-position
                delta = ((bird_corners[1][3] - kp[i].pt[1])/(bird_corners[1][3] - bird_corners[1][0])) * (bird_corners[0][3] - bird_corners[0][0])
                left_x = bird_corners[0][3] - delta
                if (kp[i].pt[0] > left_x):
                    # Calculate right bound at the given y-position
                    delta = ((bird_corners[1][2] - kp[i].pt[1])/(bird_corners[1][2] - bird_corners[1][1])) * (bird_corners[0][1] - bird_corners[0][2])
                    right_x = bird_corners[0][2] + delta
                    if (kp[i].pt[0] < right_x):
                        keep = 1;
            if (keep == 1):
                include.append(i)

        kp1 = []
        des1 = []

        for i in include:
            kp1.append(kp[i])
            des1.append(des[i])
        des1 = np.array(des1)

        return kp1, des1

    # Useful tool for drawing matches between two images
    def _draw_matches(self, img1, img2, kp1, kp2, matches, numberofpoints):
        # Get image dimensions        
        rows1, cols1 = img1.shape;
        rows2, cols2 = img2.shape;
        # Get ready to add it to the train image
        fullimg = np.zeros((max(rows1,rows2),cols1 + cols2 + 5,3), np.uint8);
        # Stick the two images together on full image (offset img2)
        for x in range(0,rows2-1):
            for y in range(0,cols2-1):
                fullimg[x,y+cols1+5] = img2[x,y];
        for x in range(0,rows1-1):
            for y in range(0,cols1-1):
                fullimg[x,y] = img1[x,y];

        # Plot the keypoint matches
        for i in range(0,len(matches)):
            # Get indices of matched points in the kp arrays
            index1 = matches[i].queryIdx;
            index2 = matches[i].trainIdx;
            # Extract coordinates of these keypoints
            feature1 = kp1[index1].pt;
            feature2 = kp2[index2].pt;
            # Offset feature2
            feature1 = tuple([int(feature1[0]), int(feature1[1])]);
            feature2 = tuple([int(feature2[0] + cols1 + 5), int(feature2[1])]);
            # Put dots on these features
            cv2.circle(fullimg, feature1, 2, (0,255,0), -1);
            cv2.circle(fullimg, feature2, 2, (0,255,0), -1);
            # Connect the dots with a line
            cv2.line(fullimg,feature1,feature2,(0,255,0),1)

        while(1):
            cv2.imshow('full',cv2.resize(fullimg,(0,0),fx=.3,fy=.3));

            if cv2.waitKey(20) & 0xFF == 27:
                break;
        cv2.destroyAllWindows()
        return;

    # Given the mask generated from BF, use it to filter out the
    # outliers from matches list
    def _filter_matches(self, good, matches_mask):
        removeAt = []; 
        count = 0;
        for i in matches_mask:
            if i == 0:
                removeAt.append(count)
            count = count + 1
        for i in reversed(removeAt):
            good.pop(i)
        return good


    ####################################
    #####     5. DEBUG/TESTING     #####
    ####################################

# Got these points from "cap6"
view_coordinates = np.float32([[922,220],[688,27],[276,27],[7,218]])
# Assuming square is 227px big
map_coordinates = np.float32([[650,650],[650,350],[350,350],[350,650]])

# Set filename and read it into an opencv object
img_location = 'cap1.jpg'
img = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
# Create a new VC object
VC = VisualCortex(view_coordinates,map_coordinates,img);

img_location = 'cap2.jpg'
img = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
VC.SLAM(img)


while(1):
    cv2.imshow('full',cv2.resize(VC.full_map,(0,0),fx=.5,fy=.5));

    if cv2.waitKey(20) & 0xFF == 27:
        break;
cv2.destroyAllWindows()
