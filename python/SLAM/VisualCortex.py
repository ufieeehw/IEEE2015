import numpy as np
from copy import deepcopy
from numpy.linalg import inv
import cv2


class VisualCortex:
	####################################
	#####  1. LIST OF PROPERTIES   #####
	####################################
	FullMap = None;
	Position = None;
	Rotation = None;
	CourseBounds = None;
	_mapFeatures = None;
	_viewFeatures = None;
	_transformedView = None;
	_perspectiveMatrix = None;


    ####################################
    #####      2. INITIALIZER      #####
    ####################################
    #initialization for integrating with GNC
	def __init__(self, viewCoordinates, mapCoordinates):
		#initialize the map and coordinates to be published to other nodes
		self.FullMap= 0;   #make this actually initialize to a black map...
		self.Position = [0 , 0];
		self.Rotation = 0;
		self.CourseBounds = [[ -1 , -1 , 1 ,  1 ],
							 [ -1 ,  1 , 1 , -1 ]]; #how should we initialize this?

		#get the transform matrix
		M = self.getForwardMatrix(viewCoordinates , mapCoordinates);
		self._perspectiveMatrix = M;



	####################################
	#####     3. PUBLIC METHODS    #####
	####################################
	# Takes an image from the camera's video feed and transforms it into a bird's eye view
	def Transform_Image(self, image , _perspectiveMatrix): 
		#remap original image by applying transform matrix
		imgx = cv2.warpPerspective(image,_perspectiveMatrix,(800,800), flags = 1, borderMode = 0, borderValue = (255,0,0))
		return imgx;

	#Finds the features/corners of interest of an input image, which can be either the _transformedView, FullMap, or raw camera image
	def Feature_Detect(self, image , mask ):
		# Initialize detector
		orb = cv2.ORB()
		# find the keypoints and descriptors with SIFT
		kp, des = orb.detectAndCompute(image,None)
		return kp, des;

	def Feature_Match(self, kp1, kp2, des1, des2):
		# create BFMatcher object
		bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

		# Match descriptors.
		matches = bf.match(des1,des2)
		# Sort them in the order of their distance.
		matches = sorted(matches, key = lambda x:x.distance)

		#end of orginial return matches;
		
		# store all the good matches as per Lowe's ratio test.
		good = []
		for m in matches:
			#if m.distance < 0.7*n.distance: #good wiTH THis cutoff ratio?
				good.append(m)

		if len(good)>0:
			src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			matchesMask = mask.ravel().tolist()

			#h,w = img1.shape
			#pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
			#dst = cv2.perspectiveTransform(pts,M)

			#img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

		else:
				print "Not enough matches are found - %d/%d" % (len(good), 10)
				matchesMask = None
				M = None
    	#im going to assume Matt's stuff takes care of this
    	#draw_params = dict(matchColor = (0,255,0), # draw matches in green color
         #          singlePointColor = None,
          #         matchesMask = matchesMask, # draw only inliers
           #        flags = 2)

		#img3 = drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
		return M, matchesMask, good
		##############################need to return something of matches type##################################
		#plt.imshow(img3, 'gray'),plt.show()


	#use SIFT/SURF/ORB/other for determining the Rotation and Position of the robot
	def Localize(self,  _viewFeatures, _mapFeatures):
		return None;

	#updates the FullMap based on new information discovered about the field, anchored at the position and rotation calculated by Localize
	def Stitch(self, Position, Rotation):
		return None;


	####################################
	#####    4. PRIVATE METHODS    #####
	####################################

	# Add any kind of private methods here that support the above public methods.
	# Be sure to include a detailed summary of what the method does


	#Calculate the transform matrix
	#Calculate the transform matrix
	def getForwardMatrix(self, orig,guess):
		#scale and move image around so that your location is 400,800 and 
		#  you are sufficiently zoomed out
		new = self.tune_output_square(.4,[-160,170],guess)
		#calculate transform matrix
		return cv2.getPerspectiveTransform(orig,new)


	#scale the transformed image so its dimensions are equal to the template
	#find the current size of a feature in px and then state the desired size of this feature
	#in the given template, a square is 53px (so we put in 53 for desired)
	def scaleImage(self, imgx,current,desired):
		factor = desired/current
		print factor
		rows,cols,ch = imgx.shape
		scaledimg = np.zeros((np.floor(rows*factor) , np.floor(cols*factor) , 3) , np.uint8)
		scaledimg = cv2.resize(imgx,(int(rows*factor),int(cols*factor)),interpolation = cv2.INTER_AREA)
		return scaledimg


	#quick fix for playing with the output position of the calibration
	#  quadrilateral such that we are zoomed out correctly
	def tune_output_square(self, scale,translate,points):
		newpoints = deepcopy(points)
		avg_x = (points[0][0] + points[1][0] + points[2][0] + points[3][0])/4
		avg_y = (points[0][1] + points[1][1] + points[2][1] + points[3][1])/4
		for n in range(0,4):
			newpoints[n][0] = avg_x - (avg_x - points[n][0])*scale + translate[0]
			newpoints[n][1] = avg_y - (avg_y - points[n][1])*scale + translate[1]
		return newpoints

	#calculate where the cone of vision has a vertex (your location)
	#  and draw lines on the image to help visualize this
	def find_vertex(self,M,imgx):
		#initialize boundary points
		points = np.matrix([[0,   0    , 800, 800],
						    [600, 800  , 600, 800],
						    [1, 1    , 1  , 1  ]])
		#calculate transformed points (in homogeneous coordinates)
		homopoints = M * points
		#convert homogeneous points to xy
		newpoints = np.zeros((2,4))
		for i in range(0,2):
			for j in range(0,4):
				newpoints[i,j] = homopoints[i,j]/homopoints[2,j]
		#calculate line parameters for both boundaries
		m1 = (newpoints[1,1] - newpoints[1,0]) / (newpoints[0,1] - newpoints[0,0])
		m2 = (newpoints[1,3] - newpoints[1,2]) / (newpoints[0,3] - newpoints[0,2])
		b1 = newpoints[1,0] - m1*newpoints[0,0]
		b2 = newpoints[1,2] - m2*newpoints[0,2]
		#calculate intersection point of these lines
		xint = (b1-b2)/(m2-m1)
		yint = (m1*xint+b1)
		cv2.line(imgx, (int(newpoints[0,0]),int(newpoints[1,0])) , (int(xint),int(yint)) , (0,0,255),1)
		cv2.line(imgx, (int(newpoints[0,2]),int(newpoints[1,2])) , (int(xint),int(yint)) , (0,0,255),1)
		return xint, yint


	#scale the transformed image so its dimensions are equal to the template
	#find the current size of a feature in px and then state the desired size of this feature
	#in the given template, a square is 53px (so we put in 53 for desired)
	def scaleImage(self, imgx,current,desired):
		factor = desired/current
		print factor
		rows,cols,ch = imgx.shape
		scaledimg = np.zeros((np.floor(rows*factor) , np.floor(cols*factor) , 3) , np.uint8)
		scaledimg = cv2.resize(imgx,(int(rows*factor),int(cols*factor)),interpolation = cv2.INTER_AREA)
		return scaledimg


	#some code that I was using to test the perspective transform, put in a method to avoid clutter
	def test_transform_image(self, _perspectiveMatrix):
		#set filename and read it into an opencv object
		img_location = 'course.jpeg'
		img = cv2.imread(img_location)

		#if the following gives an error, you are probably screwing up the filename
		rows,cols,ch = img.shape
		print rows, cols, ch

		#remap original image by applying transform matrix
		imgx = cv2.warpPerspective(img,_perspectiveMatrix,(800,800), flags = 1, borderMode = 0, borderValue = (255,0,0))

		#figure out if the vertex is in the right location and draw helpful lines
		xint, yint = self.find_vertex(_perspectiveMatrix,imgx)
		print xint, yint

		#resize this image
		scaledimgx = self.scaleImage(imgx,np.floor(.4*227) ,53)

		#write imageb
		cv2.imwrite('xformed.png',imgx)
		cv2.imwrite('scaledxform.png',scaledimgx)

		while(1):
		    cv2.imshow('image',img)
		    cv2.imshow('scaledxform',scaledimgx)

		    if cv2.waitKey(20) & 0xFF == 27:
		        break
		cv2.destroyAllWindows()

		return;

	#this should display the two images superimposed on top of each other as if they were stitched, using 3 random points
	def superimposeImages1(self,img1,img2,kp1,kp2,matches):
		# indices of which points in "matches" to use
		point1 = 0;
		point2 = 1;
		point3 = 2;
		#pull out the indices that we matched up from the train
		trainIndices = [matches[point1].trainIdx, matches[point2].trainIdx, matches[point3].trainIdx];
		#pull out the indices that we matched up from the query
		queryIndices = [matches[point1].queryIdx, matches[point2].queryIdx, matches[point3].queryIdx];

		#gather the coordinates in train that we go TO
		trainPts = np.float32([ kp2[trainIndices[0]].pt,
								kp2[trainIndices[1]].pt,
								kp2[trainIndices[2]].pt]);
		#gather the coordinates in query that we come FROM
		queryPts = np.float32([ kp1[queryIndices[0]].pt,
								kp1[queryIndices[1]].pt,
								kp1[queryIndices[2]].pt]);
		#transform the query image
		M = cv2.getAffineTransform(queryPts,trainPts);
		print M
		rows1,cols1 = img1.shape;
		rows2,cols2 = img2.shape;
		rows = max(rows1,rows2)
		cols = max(cols1,cols2)
		fullimg1 = cv2.warpAffine(img1,M,(cols,rows));


		#get ready to add it to the train image
		fullimg2 = np.zeros((rows,cols), np.uint8);
		for x in range(0,img2.shape[0]):
			for y in range(0,img2.shape[1]):
				fullimg2[x,y] = img2[x,y];

		superimposed = fullimg1/2 + fullimg2/2;



		while(1):
		    cv2.imshow('query',fullimg2);
		    cv2.imshow('train',fullimg1);
		    cv2.imshow('both',superimposed);

		    if cv2.waitKey(20) & 0xFF == 27:
		        break
		cv2.destroyAllWindows()

		return;



	#this should display the two images superimposed on top of each other as if they were stitched, using the BF M
	def superimposeImages(self,img1,img2,M):
		h,w = img1.shape
		#M = inv(M)
		fullimg2 = cv2.warpPerspective(img1,M,(w,h), flags = 1, borderMode = 0, borderValue = (255,0,0))



		superimposed = fullimg2/2 + img1/2;



		while(1):
		    cv2.imshow('img1',img1);
		    cv2.imshow('fullimg2',fullimg2);
		    cv2.imshow('both',superimposed);

		    if cv2.waitKey(20) & 0xFF == 27:
		        break
		cv2.destroyAllWindows()

		return;


	#draw the two images and lines between the features that map
	def drawMatches(self, img1, img2, kp1, kp2, matches, numberofpoints):
		#get image dimensions
		rows1, cols1 = img1.shape;
		rows2, cols2 = img2.shape;
		#get ready to add it to the train image
		fullimg = np.zeros((max(rows1,rows2),cols1 + cols2 + 5,3), np.uint8);
		#stick the two images together on full image (offset img2)
		for x in range(0,rows2-1):
			for y in range(0,cols2-1):
				fullimg[x,y+cols1+5] = img2[x,y];
		for x in range(0,rows1-1):
			for y in range(0,cols1-1):
				fullimg[x,y] = img1[x,y];

		#plot the keypoint matches
		for i in range(0,numberofpoints):
			#get indices of matched points in the kp arrays
			index1 = matches[i].queryIdx;
			index2 = matches[i].trainIdx;
			#extract coordinates of these keypoints
			feature1 = kp1[index1].pt;
			feature2 = kp2[index2].pt;
			#offset feature2
			feature1 = tuple([int(feature1[0]), int(feature1[1])]);
			feature2 = tuple([int(feature2[0] + cols1 + 5), int(feature2[1])]);
			#put dots on these features
			cv2.circle(fullimg, feature1, 2, (0,255,0), -1);
			cv2.circle(fullimg, feature2, 2, (0,255,0), -1);
			#connect the dots with a line
			cv2.line(fullimg,feature1,feature2,(0,255,0),1)

		while(1):
			cv2.imshow('full',fullimg);

			if cv2.waitKey(20) & 0xFF == 27:
				break;
		cv2.destroyAllWindows()
		return;

	# given the mask generated from BF, use it to filter out the outliers from matches list
	def filterMatches(self, good, matchesMask):
		removeAt = []; 
		count = 0;
		for i in matchesMask:
			if i == 0:
				removeAt.append(count)
			count = count + 1
		for i in reversed(removeAt):
			good.pop(i)
		return good


	#for testing the feature_detect and feature_match methods
	def test_feature_map(self,smallimg,bigimg):
		#load images
		# SMALL IMAGE MUST BE IMG2!!!!!!!!!!!!
		#img1 = cv2.imread('Guyinfield.jpg',0) # queryImage
		#img2 = cv2.imread('field.jpg',0) # trainImage
		img1 = bigimg
		img2 = smallimg

		#get orb output
		# I SWITCHED THE INDICES HERE.  BE CAREFUL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		kp1, des1 = VC.Feature_Detect(img1,None);
		kp2, des2 = VC.Feature_Detect(img2,None);

		#get matches matrix
		M, matchesMask, good = VC.Feature_Match(kp1,kp2,des1,des2);

		newgood = self.filterMatches(good,matchesMask)

		#plot both images and then superimpose them on each other by matching three points
		#self.superimposeImages(img1,img2,M);
		self.superimposeImages1(img1,img2,kp1,kp2,newgood)
		#plot both images side by side and draw lines between matched points
		self.drawMatches(img1,img2,kp1,kp2,newgood,3);


		return;












	####################################
	#####     5. DEBUG/TESTING     #####
	####################################

	# Put the main program here that uses the above class(es) to test

#calculate xform matrix
viewCoordinates = np.float32([[556,481],[509,457],[783,481],[669,457]])
#assuming square is 227px big
mapCoordinates = np.float32([[556,481],[556,254],[783,481],[783,254]])

#Create a new VC object and initialize it with the info needed to calculate transform matrix
VC = VisualCortex(viewCoordinates,mapCoordinates);
#VC.test_transform_image(VC._perspectiveMatrix);

#read world map
img_location = 'template.jpg'
worldmap = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
print worldmap.shape

#set filename and read it into an opencv object
img_location = 'course.jpg'
img = cv2.cvtColor(cv2.imread(img_location) , cv2.COLOR_BGR2GRAY)
print img.shape
#transform the camera view
imgx = VC.Transform_Image(img,VC._perspectiveMatrix);

#Use orb to find the features and descriptors (ignore ROI for now)
VC.test_feature_map(imgx,worldmap);








#ORB stuff
