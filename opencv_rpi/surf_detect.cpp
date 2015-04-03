#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <exception>

using namespace cv;

/** @function readme */
void readme()
{ 
	std::cout << " Usage: ./detect <object>" << std::endl; 
}

void processImage(SURF& detector, std::vector<KeyPoint> keypoints_object, Mat& descriptors_object, Mat& img_object, Mat& img_scene)
{
	//-- Step 1: Detect the keypoints using ORB Detector
	std::vector<KeyPoint> keypoints_scene;
	detector.detect( img_scene, keypoints_scene );

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_scene;
	detector.compute( img_scene, keypoints_scene, descriptors_scene );
	descriptors_scene.convertTo(descriptors_scene, CV_32F);
	if(descriptors_scene.empty())
	{
		//throw std::runtime_error("Missing Scene Descriptors");
		imshow( "Camera", img_scene );
		return;
	}

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_object, descriptors_scene, matches );

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors_object.rows; i++ )
	{ double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist );
	printf("-- Min dist : %f \n", min_dist );

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	for( int i = 0; i < descriptors_object.rows; i++ )
	{
		// Ratio Test
		if( matches[i].distance < 3.0*min_dist )
		{ 
			good_matches.push_back( matches[i]); 
		}
	}

	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for( unsigned int i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	}

	Mat H = findHomography( obj, scene, CV_RANSAC );

	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
	obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
	std::vector<Point2f> scene_corners(4);

	perspectiveTransform( obj_corners, scene_corners, H);

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
	line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

	//-- Show detected matches
	imshow( "Camera", img_matches );
}

#define WIDTH 160
#define HEIGHT 120

/** @function main */
int main( int argc, char** argv )
{
	if( argc != 2 )
	{ readme(); return -1; }

	Mat img_object = imread( argv[1]);

	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		std::cout << "Cannot open video camera" << std::endl;
		return 1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	SURF detector;

	//-- Step 1: Detect the keypoints using ORB Detector
	std::vector<KeyPoint> keypoints_object;
	detector.detect( img_object, keypoints_object );

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_object;
	detector.compute( img_object, keypoints_object, descriptors_object );
	//descriptors_object.convertTo(descriptors_object, CV_32F);

	if(descriptors_object.empty())
	{
		throw std::runtime_error("Missing Object Descriptors");
	}

	while(true)
	{
		Mat img_scene;
		cap >> img_scene;
		//cvtColor(img_scene, img_scene, CV_RGB2GRAY, 1);

		if( !img_object.data || !img_scene.data )
		{ 
			std::cout<< " --(!) Error reading images " << std::endl; 
			return -1; 
		}

		processImage(detector, keypoints_object, descriptors_object, img_object, img_scene);

		if(waitKey(50) >= 0)
			break;
	}

	/*
	// Test Video Camera
	while(true)
	{
	Mat img_scene;
	cap >> img_scene;
	cvtColor(img_scene, img_scene, CV_RGB2GRAY);
	imshow( "test", img_scene );

	if(waitKey(50) >= 0)
	break;
	}
	 */
	return 0;
}


