#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;


void findLine(Mat* src, Vec4f* line){
	Mat points;
	findNonZero(*src, points);
	//printf("Col: %d\nRow: %d\n",points.cols,points.rows);
	fitLine(points, *line, CV_DIST_L2, 0, .01, .01);
}

void displayImage(Mat* img){
	// Create a window
	char* window_name = "Rubix Line";
  namedWindow(window_name, CV_WINDOW_NORMAL );

	// Show the window
  imshow(window_name, *img);

  // Wait until user exit program by pressing a key
  waitKey(0);
}

// Function found online, mostly unchanged.
void CannyThreshold(Mat* src, Mat* dst, int low, int high)
{
	Mat detected_edges;

  /// Reduce noise with a kernel 3x3
  blur( *src, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, low, high, 3);

  /// Using Canny's output as a mask, we display our result
  *dst = Scalar::all(0);

  src->copyTo( *dst, detected_edges);
}


/** @function main */
int main( int argc, char** argv )
{
  // Load an image
  Mat src = imread( argv[1] );

	// exit if image did not load properly
  if( !src.data )
  { return -1; }

	// Variables and such
	Mat src_gray, edges;
	Vec4f line_def;

  // Create a matrix of the same type and size as src (for dst)
  edges.create( src.size(), src.type() );

  // Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  // Get the Edges
  CannyThreshold(&src_gray, &edges, 70, 280);

	// Get the Line
	findLine(&edges, &line_def);

	// Superimpose line on image
	Point p1 = Point(line_def[2], line_def[3]);
	Point p2 = Point(line_def[2]+200*line_def[0], line_def[3]+200*line_def[1]);
	line(src, p1, p2, cvScalar(0,0,255,0), 8);

	//display the edges (for debug)
	displayImage(&edges);

	//print the angle of the cube
	double angle = -(atan2(line_def[1],line_def[0]) + M_PI / 4);
	angle = (angle < -M_PI / 4)? angle + M_PI / 2 : angle;
	double degrees = angle * 180 / M_PI;
	printf("Angle: %1.3frad, %1.3fdeg\n",angle,degrees);


	displayImage(&src);

  return 0;
 }
