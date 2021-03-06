#include <stdio.h>
#include <iostream>
#include <exception>
#include <vector>
#include <list>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

const int POINTS = 500;
const unsigned MAX_MATCH_COUNT = 10;
const unsigned MIN_MATCH_COUNT = 2;
const double MAX_THRESHOLD = 10;
const double MATCH_THRESHOLD = 1;
std::list<bool> match_list(MAX_THRESHOLD, false);
int match_count = 0;

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

	// m1 - main match / m2 - closest neighbor
	//std::vector< std::vector< DMatch > > matches;
	//matcher.knnMatch( descriptors_object, descriptors_scene, matches, k );

	std::sort(matches.begin(), matches.end(), 
			[](const DMatch& l, const DMatch& r) -> bool
			{
			return l.distance < r.distance;
			});

	//-- Quick calculation of max and min distances between keypoints
	//double max_dist = matches[matches.size()-1].distance; 
	//printf("-- Max dist : %f \n", max_dist );

	double min_dist = std::min(200.0f, matches[0].distance); 
	//printf("-- Min dist : %f \n", min_dist );

	/*
	   double average = 0;
	   for( int i = 0; i < descriptors_object.rows; i++ )
	   { 
	   average += matches[i].distance;
	   }
	   average /= descriptors_object.rows;
	   printf("-- Avg dist : %f \n", average);

	   double sd = 0;
	   for( int i = 0; i < descriptors_object.rows; i++ )
	   { 
	   sd += pow((matches[i].distance - average), 2.0f);
	   }
	   sd /= descriptors_object.rows;
	   printf("-- Avg dist : %f \n", sd );
	 */

	//-- Draw only "good" matches - top N matches
	std::vector< DMatch > good_matches;
	for( unsigned i = 0; i < matches.size() && i < MAX_MATCH_COUNT; ++i )
	{
		if(matches[i].distance < 1.15 * min_dist)
		{
			good_matches.push_back(matches[i]); 
		}
	}

	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	if(match_list.front())
	{
		--match_count;
	}	
	match_list.pop_front();

	if(good_matches.size() > MIN_MATCH_COUNT)
	{
		match_list.push_back(true);
		++match_count;

		//std::cout << "-- matches : " << good_matches.size() << std::endl;
		std::vector<Point2f> scene;
		for( unsigned i = 0; i < good_matches.size(); i++ )
		{
			//-- Get the keypoints from the good matches
			scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
		}

		std::vector<Point2f> hull;
		convexHull(scene, hull);

		for(unsigned i = 0; i < hull.size()-1; ++i)
		{
			line( img_matches, hull[i] + Point2f( img_object.cols, 0), hull[i+1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
		}
			line( img_matches, hull[hull.size()-1] + Point2f( img_object.cols, 0), hull[0] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
	}
	else
	{
		match_list.push_back(false);
	}

	if(match_count >= MATCH_THRESHOLD)
	{	
		std::cout << "MATCH DETECTED: " << match_count << std::endl;
	}
	else
	{
		std::cout << "NO MATCH: " << match_count << std::endl;
	}

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

	//Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	Mat img_object = imread( argv[1]);

	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		std::cout << "Cannot open video camera" << std::endl;
		return 1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	ORB detector(POINTS);

	//-- Step 1: Detect the keypoints using ORB Detector
	std::vector<KeyPoint> keypoints_object;
	detector.detect( img_object, keypoints_object );

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_object;
	detector.compute( img_object, keypoints_object, descriptors_object );
	descriptors_object.convertTo(descriptors_object, CV_32F);

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

	return 0;
}


