// Local
#include "message.h"

// STL
#include <stdio.h>
#include <iostream>
#include <exception>
#include <vector>
#include <list>
#include <algorithm>

// OpenCV
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

class detect
{
public:
	detect(ORB& detector, std::vector<KeyPoint>& keypoints_object, Mat& descriptors_object, Mat& img_object)
	: detector_(detector), keypoints_object_(keypoints_object), descriptors_object_(descriptors_object), img_object_(img_object)
	{}

	bool processImage(Mat& img_scene, std::vector< msg >& good_matches, std::vector< msg >& neighbor_matches)
	{
		//-- Step 1: Detect the keypoints using ORB Detector
		std::vector<KeyPoint> keypoints_scene;
		detector_.detect( img_scene, keypoints_scene );

		//-- Step 2: Calculate descriptors (feature vectors)
		Mat descriptors_scene;
		detector_.compute( img_scene, keypoints_scene, descriptors_scene );
		descriptors_scene.convertTo(descriptors_scene, CV_32F);
		if(descriptors_scene.empty())
		{
			//throw std::runtime_error("Missing Scene Descriptors");
			imshow( "Camera", img_scene );
			return false;
		}

		//-- Step 3: Matching descriptor vectors using FLANN matcher
		FlannBasedMatcher matcher;
		std::vector< DMatch > matches;
		matcher.match( descriptors_object_, descriptors_scene, matches );

		// m1 - main match / m2 - closest neighbor
		//std::vector< std::vector< DMatch > > matches;
		//matcher.knnMatch( descriptors_object, descriptors_scene, matches, k );

		std::sort(matches.begin(), matches.end(), 
				[](const DMatch& l, const DMatch& r) -> bool
				{
				return l.distance < r.distance;
				});

		//-- Quick calculation of max, min, avg, sd distances between keypoints
		//double max_dist = matches[matches.size()-1].distance; 
		//printf("-- Max dist : %f \n", max_dist );

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
		double min_dist = std::min(200.0f, matches[0].distance); 
		//printf("-- Min dist : %f \n", min_dist );

		//-- Draw only local "good" matches - top N matches
		std::vector< DMatch > local_matches;
		for( unsigned i = 0; i < matches.size() && local_matches.size() < MAX_MATCH_COUNT; ++i )
		{
			if(matches[i].distance < 1.15 * min_dist)
			{
				local_matches.push_back(matches[i]); 
			}
		}

		Mat img_matches;
		drawMatches( img_object_, keypoints_object_, img_scene, keypoints_scene,
				local_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		// Determine if object is detected
		if(match_list.front())
		{
			--match_count;
		}	
		match_list.pop_front();

		if((local_matches.size() + neighbor_matches.size()) > MIN_MATCH_COUNT)
		{
			match_list.push_back(true);
			++match_count;

			std::cout << "-- matches : " << local_matches.size() << " " << neighbor_matches.size() << std::endl;
			std::vector<Point2f> scene;
			for( unsigned i = 0; i < local_matches.size(); ++i )
			{
				//-- Get the keypoints from the good matches
				scene.push_back( keypoints_scene[ local_matches[i].trainIdx ].pt );
				good_matches.push_back( make_msg(local_matches[i].trainIdx, 
					keypoints_scene[ local_matches[i].trainIdx ].pt,
					local_matches[i].distance) );
					
			}

			for( unsigned i = 0; i < neighbor_matches.size(); ++i )
			{
				//-- Get the keypoints from the good matches
				scene.push_back( neighbor_matches[i].scene );
			}

			std::vector<Point2f> hull;
			convexHull(scene, hull);

			for(unsigned i = 0; i < hull.size()-1; ++i)
			{
				line( img_matches, hull[i] + Point2f( img_object_.cols, 0), hull[i+1] + Point2f( img_object_.cols, 0), Scalar(0, 255, 0), 4 );
			}
				line( img_matches, hull[hull.size()-1] + Point2f( img_object_.cols, 0), hull[0] + Point2f( img_object_.cols, 0), Scalar(0, 255, 0), 4 );
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
		return true;
	}

private:
	ORB detector_;
	std::vector<KeyPoint> keypoints_object_;
	Mat descriptors_object_;
	Mat img_object_;
};

