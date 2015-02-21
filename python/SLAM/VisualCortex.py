import numpy as np
from copy import deepcopy
from numpy.linalg import inv
import cv2
import random
import math

#Areas of concern:
# too many detected point
# images after transformation are bad
#indistinguishable transform
#much more distortion
    #how do we combat that
    #camera angle at fault
    
# non-properties
class VisualCortex:

    
    ####################################
    #####  1. LIST OF PROPERTIES   #####
    ####################################
    # Map image of the course
    full_map = None;
    # Dimensions of full course
    full_map_x = 2000
    full_map_y = 2000
    full_map_mask = None;
    # Robot's position and rotation
    position = None;
    rotation = None;
    # Dimensions of images that come from camera feed
    cam_x = None;
    cam_y = None;
    # Dimensions of camera images after perspec transform (bird's eye dims)
    bird_x = None;
    bird_y = None;
    bird_mask = None;
    trapezoid = None;
    # Features and descriptors of full map
    _map_kp = None;
    _map_descriptors = None;

    # Some useful private variables
    _view_features = None;
    _transformed_view = None;
    _perspective_matrix = None;
    #position on a perspec transformed img that corresponds to robot's loc
    _robot_coordinates = [400, 800, 1];
    _perspec_corners = None


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
        self.full_map_mask = np.zeros((self.full_map_x, self.full_map_y), np.uint8)
        # TODO: This is not actually the starting position... This is the center of
        #    where we place the first image on the black map.  We can then extract
        #    the actual starting position by running "self.localize" on the affine
        #    matrix given by self._initialize_affine_matrix below
        self.position = [self.full_map_x/2, self.full_map_y/2];
        self.rotation = 0;

        # Read off the dimensions of images we are getting from camera
        self._initialize_cam_dimensions(img)

        # Get the perspec transform matrix
        # TODO: Get a more accurate transform
        self._perspective_matrix = self._get_perspective_matrix(view_coordinates, map_coordinates);
        # Now that we have a matrix, get the bird's eye image dimensions
        self._get_bird_dims()
        self._calculate_perspec_corners()
        self._get_bird_mask()
        # Transform the initial image to bird's eye
        imgx = self.transform_image(img)

        # And paste it on to the full_map using a basic translation matrix
        affine_matrix = self._initialize_affine_matrix()
        self.stitch(imgx, affine_matrix)



    ####################################
    #####     3. PUBLIC METHODS    #####
    ####################################

    def SLAM(self, image):
        """Takes in the next input image and does feature detection, mapping,
        localization, and stitching to the full_map
        """
        # Run perspective transform on image
        imgx = self.transform_image(image)

        # Extract features from this image
        kp1, des1 = self.feature_detect(imgx, "new")

        #self._draw_features(self.full_map, self._map_kp, None, "points")
        self._draw_features(imgx, kp1, None, "points")
        self._draw_features(self.full_map, self._map_kp, None, "points")
        # Match features between imgx and full_map
        M, matches, q, t = self.feature_match(kp1, des1)
        #self._draw_features(imgx, kp1, None, "array")
        #self._draw_features(self.full_map, self._map_kp, None, "array")
        self._draw_matches(imgx, self.full_map, kp1, self._map_kp, matches)

        # Use this affine matrix to update robot position
        # TODO: Make position/rotation a property
        self.localize(M)
        # Use this affine matrix to update full_map
        self.stitch(imgx, M)


        return;

    """Takes an image from the camera's video feed and transforms it into a 
    bird's eye view, returning this transformed image
    """
    def transform_image(self, image): 
        #remap original image by applying transform matrix
        imgx = cv2.warpPerspective(image, self._perspective_matrix,
                                   (self.bird_x,self.bird_y), flags = 1, 
                                   borderMode = 0, borderValue = (0,0,0))
        # Clean up the image
        imgx = self.clean_image(imgx)

        return imgx;

    """Find the features/corners of interest of an input image, 
    which can be either the bird's eye image or full_map, and returns the 
    keypoints array and respective descriptors matrix
    """    
    def feature_detect(self, image, typ):
        # Initialize detector
        orb = cv2.ORB()
        # Find the keypoints and descriptors with ORB
        if typ == "new":
            kp, des = orb.detectAndCompute(image, self.bird_mask)
            return kp, des;
        else:
            self._map_kp, self._map_descriptors = orb.detectAndCompute(self.full_map, self.full_map_mask)
            return

    """Given a set of two keypoint arrays and their respective descriptor 
    matrices, return the array of match objects after masking
    and the resulting homography matrix from BruteForce matching
    """
    def feature_match(self, kp1, des1):
        # Create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        # Match descriptors and get the matches array
        matches = bf.match(des1, self._map_descriptors)

        # Sort them in the order of their distance.
        # TODO: change the lambda to a def, as specified by PEP8
        matches = sorted(matches, key = lambda x: x.distance)

        # Optionally store all the good matches as per Lowe's ratio test.
        good_matches = []
        for m in matches:
            #if m.distance < 0.7*n.distance:
                good_matches.append(m)

        # Make sure we found matches, and then use the matches objects to 
        # extract lists of points that correspond to the matches and
        # determine the mask array based on locations.  Use this mask to
        # filter the full list of matches
        if len(good_matches)>0:
            # TODO: 'splain the following 4 lines of code
            query_pts = np.float32(
                [ kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1, 1, 2)
            train_pts = np.float32(
                [ self._map_kp[m.trainIdx].pt for m in good_matches ]).reshape(-1, 1, 2)
            # Run regression on the matches to filter outliers
            M, mask, successrate, q, t = self.estimateAffine(query_pts, train_pts, 5.0)

            # Use this mask immediately to get the good_matches matches
            good_matches = self._filter_matches(good_matches, mask)
        else:
                print "Not enough matches are found - %d/%d" % (len(good_matches), 10)
                matches_mask = None
                M = None
        return M, good_matches, q, t

    def estimateAffine(self, query_pts, train_pts, threshold):
        """This is a 2D version of estimateAffine3D (a built in function) that implements RANSAC to Figure
        out the best affine transformation between two sets of matched points
        """
        # Get length of dataset
        length = len(query_pts)

        # Sanity check
        if ((length != len(train_pts)) or (length < 3)):
            print "ERROR: Match list dims do not match, or not enough matches to estimate"

        # Initialize affine matrix, weight, points, and history
        M = [[1, 0, 0],
             [0, 1, 0]]
        best_cost = 0
        chosen = [0,0,0]
        history = [[0,0,0]]
        query = []
        train = []
        query_chosen = [[0, 0],
                        [0, 0],
                        [0, 0]]
        train_chosen = [[0, 0],
                        [0, 0],
                        [0, 0]]   

        # Redo query_ and train_pts because they have a really stupid shape and datatype
        for i in range(0, length):
            query.append([query_pts[i][0][0], query_pts[i][0][1] ])
            train.append([train_pts[i][0][0], train_pts[i][0][1] ])

        # TODO: Figure out a good number to use here.  Currently goes for half of the possible combinations (nCr / 2)
        # TODO: Figure out how small a number of matches we need to try every combo
        montecarlo_runs = math.factorial(length)/math.factorial(length-3)/math.factorial(3)/2
        # TODO: Use statistics to calculate the expected number of iterations to find a unique set
        TIMEOUT = montecarlo_runs
        run_count = 0
        # RANSAC to find the best affine transform
        while (run_count < montecarlo_runs):
            # DATAPATH for cost (see page 15: https://courses.engr.illinois.edu/cs498dh/fa2011/lectures/Lecture%2018%20-%20Photo%20Stitching%20-%20CP%20Fall%202011.pdf)
            # Select three random points that we haven't already tested
            # TODO: Figure out a way to recognize [1,2,3] is already in history, even if the actual entry is
            #       [1,3,2] or some different order.  Otherwise, remove the history check completely.
            #       I'm not sure if we need it at all
            time = 0
            while (chosen in history):
                # Check for TIMEOUT
                time = time + 1
                if (time > TIMEOUT):
                    break
                # Reset the pool
                pool = range(0, length)
                # Choose the three random points
                for i in range(0, 3):
                    chosen[i] = random.choice(pool)
                    pool.remove(chosen[i])
            # Append them to our history if they are new
            history.append(deepcopy(chosen))

            # Get coordinates from the random indices selected above (note that ****_pts is a really funky data type....)
            for i in range(0,3):
                query_chosen[i] = query[chosen[i]]
                train_chosen[i] = train[chosen[i]]

            # Find affine transform between the points
            M = cv2.getAffineTransform(np.array(query_chosen), np.array(train_chosen))

            # Apply affine
            homoquery = []
            for i in range(0, length):
                homoquery.append([query[i][0], query[i][1], 1])
            mapped = np.dot(homoquery, M.T)

            # Get distance between points in "mapped" and "train"
            cost = 0
            net_distance = 0
            compare = pow(threshold, 2)
            mask = []
            for i in range(0, length):
                # Apply pythagorean theorem
                distance = pow(mapped[i][0] - train[i][0], 2) + pow(mapped[i][1] - train[i][1], 2)
                # Compare the distance with our threshold
                if (distance < compare): 
                    # Note that "mask" 2D array in opencv's stupid "findHomography," but I fixed that so we don't need to unravel
                    mask.append(1)
                    net_distance = net_distance + distance
                    cost = cost + 1
                else:
                    mask.append(0)

            # Update M, mask, and cost if this cost is better then previous max
            # TODO: Figure out if comparing net inlier distance is the best way to choose a winner when costs are equal
            #       .... Or should we go by distance only???  This isn't how findHomography works but it may be better for
            #       our purposes
            if (cost > best_cost):
                # TODO: Figure out which of these actually need the deepcopy. I'm tired of getting
                #       burnt by it so I just made them all deepcopies
                best_cost = deepcopy(cost)
                best_net_distance = deepcopy(net_distance)
                best_M = deepcopy(M)
                best_mask = deepcopy(mask)
                best_query = deepcopy(query_chosen)
                best_train = deepcopy(train_chosen)
            elif (cost == best_cost):
                if (net_distance < best_net_distance):
                    # TODO: Figure out which of these actually need the deepcopy. I'm tired of getting
                    #       burnt by it so I just made them all deepcopies
                    best_cost = deepcopy(cost)
                    best_net_distance = deepcopy(best_net_distance)
                    best_M = deepcopy(M)
                    best_mask = deepcopy(mask)
                    best_query = deepcopy(query_chosen)
                    best_train = deepcopy(train_chosen)

            # Increment the iteration count
            run_count = run_count + 1

        # Return the affine matrix, mask array, and tuple of (inliers, totalmatches)
        return best_M, best_mask, (best_cost,length), best_query, best_train        
        
    def localize(self, affine_matrix):
        """Apply the latest _affine_matrix to the _robot_coordinates so that we
        can update the robot's location in the full_map
        """
        self.position = np.dot(affine_matrix, self._robot_coordinates)
        bearing = np.dot(affine_matrix, np.array([[self._robot_coordinates[0]],
                                                  [self._robot_coordinates[1] - self.bird_y/2],
                                                  [1]
                                                 ]))
        # Note that delta_y seems flipped.  This is because x coordinates go left to right (normal)
        # but y coordinates go top to bottom (flipped). As Bill Nye would say, consider the following:
        #                        O  (bigger x, smaller y)
        #               |       /
        #               |      /
        #               |     /
        #               |ang /
        #               |   / 
        #               |  /
        #               | / (smaller x, bigger y)
        #                O
        delta_x = bearing[0] - self.position[0]
        delta_y = self.position[1] - bearing[1]
        # Gives rotation in radians East of North
        self.rotation = np.arctan(delta_y/delta_x)

    def stitch(self, imgx, matrix):
        """ Update the full_map with the latest image, using the
        most up-to-date _affine_matrix
        """
        # Decide if we are doing affine or perspective xform based on the matrix shape
        if (matrix.shape == (2,3)): # Affine
            # Transform imgx into something the size of the full map using affine
            fullimgx = cv2.warpAffine(imgx, matrix, (self.full_map_x, self.full_map_y))

        elif (matrix.shape == (3,3) and matrix[2][2] == 1): # Perspective
            # Transform imgx into something the size of the full map using perspective
            fullimgx = cv2.warpPerspective(imgx, matrix, (self.full_map_x, self.full_map_y), flags = 1, 
                                           borderMode = 0, borderValue = (0,0,0))
        else:
            print "ERROR: Unknown transform matrix.  Cannot stitch to full map"
            fullimgx = np.ones((self.full_map_x, self.full_map_y), np.uint8);  

        fullimg2 = self.full_map
        # Add the images together
        # TODO: Use "blip" or something to add the images together
        self.full_map = fullimgx/2 + self.full_map/2;
        # Clean up the image
        self.full_map = self.clean_image(self.full_map)
        # Update the roi
        self._update_full_mask(matrix)
        # Update kp des dictionary
        self.feature_detect(None, "full map")

        #superimposed = cv2.resize(superimposed,(0,0), fx=.25, fy=.25)
        return None

    def clean_image(self, img):
        """ Clean up a transformed or stitched image by threshholding it and then
        blurring it to remove pixelation and salt/pepper
        """
        thresh = 50
        dump, img = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY)
        kernel = 10
        img = cv2.blur(img, (kernel, kernel))
        return img

    def display_map(self):
        """ Handle the while loop that shows an image, to clean up code a bit since we use this so often
        """
        while(1):
            cv2.imshow('full',cv2.resize(self.full_map,(0,0),fx=.5,fy=.5));

            if cv2.waitKey(20) & 0xFF == 27:
                break;
        cv2.destroyAllWindows()
        return;

    ####################################
    #####    4. PRIVATE METHODS    #####
    ####################################

    # Put green dots on each of the features
    def _draw_features(self, image, kp, bird_corners, intype):
        if intype == "points":
            # Turn this image into color so we see green dots
            color_img = cv2.cvtColor(image , cv2.COLOR_GRAY2RGB)
            # Plot the keypoints
            for i in range(0,len(kp)):
                # Extract coordinates of these keypoints
                feature1 = kp[i].pt;
                # Offset feature2
                feature1 = tuple([int(feature1[0]), int(feature1[1])]);
                # Put dots on these features
                cv2.circle(color_img, feature1, 2, (0,0,255), -1);

            if (bird_corners != None):
                cv2.line(color_img, tuple([int(bird_corners[0][0]),int(bird_corners[1][0])]),
                                    tuple([int(bird_corners[0][1]),int(bird_corners[1][1])]),
                                    (255,0,0))
                cv2.line(color_img, tuple([int(bird_corners[0][1]),int(bird_corners[1][1])]),
                                    tuple([int(bird_corners[0][2]),int(bird_corners[1][2])]),
                                    (255,0,0))
                cv2.line(color_img, tuple([int(bird_corners[0][2]),int(bird_corners[1][2])]),
                                    tuple([int(bird_corners[0][3]),int(bird_corners[1][3])]),
                                    (255,0,0))
                cv2.line(color_img, tuple([int(bird_corners[0][3]),int(bird_corners[1][3])]),
                                    tuple([int(bird_corners[0][0]),int(bird_corners[1][0])]),
                                    (255,0,0))
            while(1):
                cv2.imshow('full',color_img);

                if cv2.waitKey(20) & 0xFF == 27:
                    break;
            cv2.destroyAllWindows()

        elif intype == "array":
            # Turn this image into color so we see green dots
            color_img = cv2.cvtColor(image , cv2.COLOR_GRAY2RGB)
            # Plot the keypoints
            for i in range(0,len(kp)):
                # Offset feature2
                feature1 = tuple([int(kp[i][0]), int(kp[i][1])]);
                # Put dots on these features
                cv2.circle(color_img, feature1, 2, (0,0,255), -1);

            while(1):
                cv2.imshow('full',color_img);

                if cv2.waitKey(20) & 0xFF == 27:
                    break;
            cv2.destroyAllWindows()


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
        return np.float32([[1, 0, self.position[0] - self.bird_x/2],
                           [0, 1, self.position[1] - self.bird_y/2]
                          ])
    
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
        self.bird_mask = np.zeros((self.bird_y, self.bird_x), np.uint8)
        return

        #created this method in response to the todo comment
    def _calculate_perspec_corners(self):
        # Figure out where the corners of the perspective image map to in the
        # bird's eye image
        c = 0     # Number of pixels to cushion the border with
        self._perspec_corners = [[c, self.cam_x - c, self.cam_x - c, c             ],
                                 [c, c             , self.cam_y - c, self.cam_y - c],
                                 [1, 1             , 1             , 1             ]
                                ]
        return                         

    def _update_full_mask(self, matrix):
        '''Update the binary mask that goes over the full map for feature detection'''
        new_trapezoid = np.dot(matrix, np.array(self.trapezoid))
        mask_corners =  np.array([ [new_trapezoid[0][0], new_trapezoid[1][0]],
                                   [new_trapezoid[0][1], new_trapezoid[1][1]],
                                   [new_trapezoid[0][2], new_trapezoid[1][2]],
                                   [new_trapezoid[0][3], new_trapezoid[1][3]]], np.int32)
        # If you fill with 1 instead of 255, it doesn't detect features properly
        cv2.fillPoly(self.full_map_mask, [mask_corners], 255)
        return




    # Applies a range-of-interest mask over the kp and des because
    # we don't want the corners of the transformed trapezoid to show 
    # up as features
    def _get_bird_mask(self):
        bird_homo_corners = np.dot(self._perspective_matrix, self._perspec_corners)
        # Initialize the output corners
        bird_corners = [[1, 1, 1, 1],
                        [1, 1, 1, 1]]
        # Populate the above matrix
        for col in range(0,4):
            for row in range(0,2):
                bird_corners[row][col] = bird_homo_corners[row][col] / bird_homo_corners[2][col]
        c = 10
        cushion = [[c, -c, -c, c],
                   [c, c, -c, -c]
                  ]
        bird_corners = np.add(bird_corners, cushion)
        self.trapezoid = bird_corners
        self.trapezoid = np.vstack([self.trapezoid, [1,1, 1, 1]])

        mask_corners = np.array([ [bird_corners[0][0], bird_corners[1][0]],
                                  [bird_corners[0][1], bird_corners[1][1]],
                                  [bird_corners[0][2], bird_corners[1][2]],
                                  [bird_corners[0][3], bird_corners[1][3]]], np.int32)

        # Fill with 255 instead of 1, or else it doesn't detect all of the features properly
        cv2.fillPoly(self.bird_mask, [mask_corners], 255)
        return

    # Useful tool for drawing matches between two images
    def _draw_matches(self, img1, img2, kp1, kp2, matches):
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
            cv2.imshow('full',cv2.resize(fullimg,(0,0),fx=.5,fy=.5));

            if cv2.waitKey(20) & 0xFF == 27:
                break;
        cv2.destroyAllWindows()
        return;

    # Given the mask generated from BF, use it to filter out the
    # outliers from matches list
    def _filter_matches(self, matches, matches_mask):
        matches_with_mask = zip(matches,matches_mask)
        good = [item[0] for item in matches_with_mask if item[1] == 1]
        return good


    ####################################
    #####     5. DEBUG/TESTING     #####
    ####################################

# Got these points from "cap6"
view_coordinates = np.float32([[922,220],[688,27],[276,27],[7,218]])
# Assuming square is 227px big
map_coordinates = np.float32([[650,650],[650,350],[350,350],[350,650]])

# Set filename and read it into an opencv object

img_location = 'test_images/cap1.jpg'
img = cv2.cvtColor(cv2.imread(img_location), cv2.COLOR_BGR2GRAY)
# Create a new VC object
VC = VisualCortex(view_coordinates,map_coordinates,img);

img_location = 'test_images/cap2.jpg'
img = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
VC.SLAM(img)
VC.display_map()


img_location = 'test_images/cap3.jpg'
img = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
VC.SLAM(img)
VC.display_map()
'''
img_location = 'SLAMPics/c6.jpg'
img = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
VC.SLAM(img)
VC.display_map()
'''