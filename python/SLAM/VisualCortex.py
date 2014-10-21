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
    # Dimensions of images that come from camera feed (please update)
    cam_x = 500;
    cam_y = 500;
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

        # Get the perspec transform matrix
        self._perspective_matrix = self._get_perspective_matrix(
        								view_coordinates, map_coordinates);
        # Now that we have a matrix, get the bird's eye image dimensions
        self._get_bird_dims(img.shape)
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
            M, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC,8.0)
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
        point1 = 1;
        point2 = 2;
        point3 = 3;
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

    # TODO: This method is probably obsolete...
    # Scale the transformed image so its dimensions are equal to the template
    # find the current size of a feature in px and then state the desired size 
    # of this feature in the given template, a square is 53px (so we 
    # put in 53 for desired)
    def _scale_image(self, imgx,current,desired):
        factor = desired/current
        print factor
        rows,cols,ch = imgx.shape
        scaledimg = np.zeros((np.floor(rows*factor), 
        	np.floor(cols*factor) , 3) , np.uint8)
        scaledimg = cv2.resize(imgx,(int(rows*factor),
        	int(cols*factor)), interpolation=cv2.INTER_AREA)
        return scaledimg

    # Figure out the dimensions of the image that the perspective transform
    # maps to.
    def _get_bird_dims(self,shape):
    	# Get bird_x from calculating where top-right corner maps to
    	corner = np.dot(self._perspective_matrix, 
    					[[shape[1]],
    					 [0],
    					 [1]
    					])
    	# Extract the new corner location by dividing by homogeneous coord
    	self.bird_x = corner[0] / corner[2]
    	# Get bird_y from calculating where bottom left corner maps to
    	corner = np.dot(self._perspective_matrix, 
    					[[0],
    					 [shape[0]],
    					 [1]
    					])
    	# Extract the new corner location by dividing by homogeneous coord
    	self.bird_y = corner[1] / corner[2]
    	return

    # Applies a range-of-interest mask over the kp and des because
    # we don't want the corners of the transformed trapezoid to show 
    # up as features
    def _apply_roi(self,kp,des):
    	return kp, des

    # TODO: This method is probably obsolete...
    # Calculate where the cone of vision has a vertex (your location)
    # and draw lines on the image to help visualize this
    def _find_vertex(self,M,imgx):
        # Initialize boundary points
        points = np.matrix([[0,   0    , 800, 800],
                            [600, 800  , 600, 800],
                            [1, 1    , 1  , 1  ]])
        # Calculate transformed points (in homogeneous coordinates)
        homopoints = M * points
        # Convert homogeneous points to xy
        newpoints = np.zeros((2,4))
        for i in range(0,2):
            for j in range(0,4):
                newpoints[i,j] = homopoints[i,j]/homopoints[2,j]
        # Calculate line parameters for both boundaries
        m1 = (newpoints[1,1]-newpoints[1,0]) / (newpoints[0,1]-newpoints[0,0])
        m2 = (newpoints[1,3]-newpoints[1,2]) / (newpoints[0,3]-newpoints[0,2])
        b1 = newpoints[1,0] - m1*newpoints[0,0]
        b2 = newpoints[1,2] - m2*newpoints[0,2]
        # Calculate intersection point of these lines
        xint = (b1-b2)/(m2-m1)
        yint = (m1*xint+b1)
        cv2.line(imgx, (int(newpoints[0,0]),int(newpoints[1,0])),
        	(int(xint),int(yint)) , (0,0,255),1)
        cv2.line(imgx, (int(newpoints[0,2]),int(newpoints[1,2])),
        	(int(xint),int(yint)) , (0,0,255),1)
        print "here is vertex"
        print xint, yint
        return xint, yint

    # TODO: This method is probably obsolete because I moved its guts to the
    # public section
    # This should display the two images superimposed on top of each other 
    # as if they were stitched, using 3 random points
    def _superimpose_images1(self, img1, img2, kp1, kp2, matches, points):
        # Indices of which points in "matches" to use
        point1 = points[0];
        point2 = points[1];
        point3 = points[2];
        # Pull out the indices that we matched up from the train
        _train_indices = [matches[point1].trainIdx, matches[point2].trainIdx, 
        	matches[point3].trainIdx];
        # Pull out the indices that we matched up from the query
        _query_indices = [matches[point1].queryIdx, matches[point2].queryIdx, 
        	matches[point3].queryIdx];

        # Gather the coordinates in train that we go TO
        trainPts = np.float32([ kp2[_train_indices[0]].pt,
                                kp2[_train_indices[1]].pt,
                                kp2[_train_indices[2]].pt]);
        # Gather the coordinates in query that we come FROM
        queryPts = np.float32([ kp1[_query_indices[0]].pt,
                                kp1[_query_indices[1]].pt,
                                kp1[_query_indices[2]].pt]);
        # Transform the query image
        M = cv2.getAffineTransform(queryPts,trainPts);
        rows1,cols1 = img1.shape;
        rows2,cols2 = img2.shape;
        rows = max(rows1,rows2)
        cols = max(cols1,cols2)
        fullimg1 = cv2.warpAffine(img1,M,(cols,rows));

        # Get ready to add it to the train image
        fullimg2 = np.zeros((rows,cols), np.uint8);
        for x in range(0,img2.shape[0]):
            for y in range(0,img2.shape[1]):
                fullimg2[x,y] = img2[x,y];

        superimposed = fullimg1/2 + fullimg2/2;
        dump,superimposed = cv2.threshold(superimposed, 50, 255,
        	cv2.THRESH_BINARY)
        #superimposed = cv2.resize(superimposed,(0,0), fx=.25, fy=.25)
        
        while(1):
            cv2.imshow('both',superimposed);

            if cv2.waitKey(20) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

        frameLoc = [[400],[800],[1]];
        robotLoc = np.dot(M,frameLoc);
        self.full_map = superimposed
        print "Robot's position is:"
        print robotLoc
        return robotLoc;

    # TODO: I'm pretty sure this method is extremely obsolete
    # This should display the two images superimposed on top of each other 
    # as if they were stitched, using the BF M
    def _superimpose_images(self,img1,img2,M):
        h,w = img1.shape
        #M = inv(M)
        fullimg2 = cv2.warpPerspective(img1,M,(w,h), flags=1, borderMode=0,
        							   borderValue=(255,0,0))
        superimposed = fullimg2/2 + img1/2;
        while(1):
            cv2.imshow('img1',img1);
            cv2.imshow('fullimg2',fullimg2);
            cv2.imshow('both',superimposed);

            if cv2.waitKey(20) & 0xFF == 27:
                break
        cv2.destroyAllWindows()
        return;

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
        for i in numberofpoints:
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
            cv2.imshow('full',fullimg);

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

    # For testing the feature_detect and feature_match methods
    def test_feature_map(self, smallimg, points, drawmatch):
        # Load images
        #img1 = cv2.imread('picasso3.jpg',0) # queryImage
        #img2 = cv2.imread('picasso2.jpg',0) # trainImage
        img2 = self.full_map
        img1 = smallimg

        # Get orb output
        kp1, des1 = VC.feature_detect(img1,None);
        kp2, des2 = VC.feature_detect(img2,None);

        # Get matches matrix
        M, matches_mask, good = VC.feature_match(kp1,kp2,des1,des2);

        newgood = self._filter_matches(good,matches_mask)

        # Plot both images and then superimpose them on each other by 
        # matching three points
        #self._superimpose_images(img1,img2,M);
        self._superimpose_images1(img1,img2,kp1,kp2,newgood,points)
        # Plot both images side by side and draw lines between matched points
        if (drawmatch == 1):
            self._draw_matches(img1,img2,kp1,kp2,newgood,points);

        return;

    # Some code that I was using to test the perspective transform, 
    # put in a method to avoid clutter
    def test_transform_image(self, _perspective_matrix):
        # Set filename and read it into an opencv object
        img_location = 'course.jpeg'
        img = cv2.imread(img_location)

        # If the following gives an error, you are probably screwing up the filename
        rows,cols,ch = img.shape
        print rows, cols, ch

        # Remap original image by applying transform matrix
        imgx = cv2.warpPerspective(img,_perspective_matrix,(800,800), flags=1,
        						   borderMode=0, borderValue=(255,0,0))

        # Figure out if the vertex is in the right location 
        # and draw helpful lines
        xint, yint = self._find_vertex(_perspective_matrix,imgx)
        print xint, yint

        # Resize this image
        scaledimgx = self._scale_image(imgx,np.floor(.4*227) ,53)

        # Write imageb
        cv2.imwrite('xformed.png',imgx)
        cv2.imwrite('scaledxform.png',scaledimgx)

        while(1):
            cv2.imshow('image',img)
            cv2.imshow('scaledxform',scaledimgx)

            if cv2.waitKey(20) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

        return;


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

points = [8,9,17]


points = [14,16,7]

points = [10,11,12]
