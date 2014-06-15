/*  program will attempt to find the tiles of a rubix cube by
 *  searching for each color until it's sure it's found the center.
 */
 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define DEBUG 0

using namespace cv;
void displayImage(Mat img);
vector<Vec4i> getLines(Mat img, char color);
//void getTiles(vector<Vec4i> lines, vector<Point2f> *tiles);
void filterColor(Mat src, Mat* dst, char color);

/** @function main */
int main( int argc, char** argv )
{
  //Load an image
  Mat src = imread(argv[1]);

	//exit if image did not load properly
  if(!src.data){ 
		printf("Usage: ./line2 <image> <color>\n");	
		return -1;
	}

	if(DEBUG & 1) displayImage(src);	//show the input (Debug lvl 1)
  
  Mat img_color, hsv_img, img_blur; //containers
	cvtColor(src, hsv_img, CV_BGR2HSV); //convert to HSV
	filterColor(hsv_img, &img_color, 'a');  //filter to the base color
	medianBlur(img_color, img_blur, 5); //blur the image
  
  if(DEBUG & 1) displayImage(img_blur);
  
  vector<vector<Point> > contours;
  findContours(img_blur, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  
  if(DEBUG & 2){
    Mat output = src.clone();
    drawContours(output, contours, -1, Scalar(0,0,255), 10);
    displayImage(output);
  }
  
  vector<RotatedRect> squares;
  for(int i=0;i<contours.size();++i)
    squares.push_back(minAreaRect(contours[i]));
  
  if(DEBUG & 2){ 
    printf("Number of rectangles: %d\n",squares.size());
    
    Mat output = src.clone();
    for(int j=0;j<squares.size();++j){
      Point2f vertices[4];
      squares[j].points(vertices);
      for (int i = 0; i < 4; i++) line(output, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 10);
    }
    displayImage(output);
  }
  
  double angle, x, y;
  for(int i=0;i<squares.size();++i){
    angle = (angle*i + squares[i].angle/180*M_PI)/(i+1);  //have to convert to radians
    x = (x*i + squares[i].center.x)/(i+1);
    y = (y*i + squares[i].center.y)/(i+1);
  }
  
  //draw test line
  if(DEBUG & 1){
    Mat output = src.clone();
    line(output, Point(x,y), Point(x+(1000)*cos(angle),y+(1000)*sin(angle)), Scalar(0,0,255), 10);
    displayImage(output);  
  }
  
  //move origin from upper left to center of image
  x = x - (img_blur.rows / 2);
  y = (img_blur.cols / 2) - y;
  
  printf("Rubix Cube Center: (%.0f, %.0f)\nRubix Cube Angle:%.2f\n",x,y,angle);
  
  /*vector<vector<Point> > squares;
  for(int i=0;i<contours.size();++i){
    if(4 == contours[i].size()){
      vector<Point> temp;
      approxPolyDP(contours[i], temp, 10, true);
      squares.push_back(temp);
    }
  }
  
  if(DEBUG & 1){
    Mat output = src.clone();
    drawContours(output, squares, -1, Scalar(0,0,255), 10);
    displayImage(output);
  }
  
  if(squares.size() < 2){
    printf("Error: Not enough squares\n");
    return -1;
  }*/
  
  return 0;
}

void displayImage(Mat img){
	// Create a window
	char* window_name = "line2";
  namedWindow(window_name, CV_WINDOW_NORMAL );

	// Show the window
  imshow(window_name, img);

  // Wait until user exit program by pressing a key
  waitKey(0);
}

void filterColor(Mat src, Mat* dst, char color){
	//filter the correct color
	switch(color){	
		case 'g':
			inRange(src, Scalar(40,150,80), Scalar(80,255,255), *dst);
			return;
		case 'y':
			inRange(src, Scalar(20,150,80), Scalar(35,255,255), *dst);
			return;
		case 'o':
			inRange(src, Scalar(3,150,100), Scalar(20,255,255), *dst);
			return;
		case 'b':
			inRange(src, Scalar(100,150,80), Scalar(140,255,255), *dst);
			return;
		case 'w':
			inRange(src, Scalar(0,0,180), Scalar(180,25,255), *dst);
			return;
    case 'a':
      inRange(src, Scalar(0,150,100), Scalar(180,255,255), *dst);  //filter for all colros (not white or black)
      return;
		case 'r':	Mat color1, color2;
			/*inRange(src, Scalar(0,180,25), Scalar(3,255,255), color1);
			inRange(src, Scalar(160,180,25), Scalar(180,255,255), color2);
			add(color1,color2,*dst);*/
			inRange(src, Scalar(160,150,80), Scalar(180,255,255), *dst);
			return;
	}
}
