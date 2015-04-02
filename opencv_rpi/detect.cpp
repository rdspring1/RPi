#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

/** @function readme */
void readme()
{ 
	std::cout << " Usage: ./detect <object>" << std::endl; 
}

void processImage(ORB& detector, std::vector<KeyPoint> keypoints_object, Mat& descriptors_object, Mat& img_object, Mat& img_scene)
{
	//-- Step 1: Detect the keypoints using ORB Detector
	std::vector<KeyPoint> keypoints_scene;
	detector.detect( img_scene, keypoints_scene );

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_scene;
	detector.compute( img_scene, keypoints_scene, descriptors_scene );
	descriptors_scene.convertTo(descriptors_scene, CV_32F);

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
	{ if( matches[i].distance < 2.5*min_dist )
		{ good_matches.push_back( matches[i]); }
	}

	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Show detected matches
	imshow( "Good Matches & Object detection", img_matches );
}

/** @function main */
int main( int argc, char** argv )
{
	if( argc != 2 )
	{ readme(); return -1; }

	//Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	Mat img_object = imread( argv[1]);

	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		std::cout << "Cannot open video camera" << std::endl;
		return 1;
	}

	ORB detector;

	//-- Step 1: Detect the keypoints using ORB Detector
	std::vector<KeyPoint> keypoints_object;
	detector.detect( img_object, keypoints_object );

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_object;
	detector.compute( img_object, keypoints_object, descriptors_object );
	descriptors_object.convertTo(descriptors_object, CV_32F);

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


