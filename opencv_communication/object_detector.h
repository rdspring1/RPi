#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

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

class ObjectDetector
{
	public:
		ObjectDetector(FeatureDetector& detector, DescriptorExtractor& extractor, std::vector<KeyPoint>& keypoints_object, Mat& descriptors_object, Mat& img_object)
			: detector_(detector), extractor_(extractor), keypoints_object_(keypoints_object), descriptors_object_(descriptors_object), img_object_(img_object)
		{}

		bool processImage(Mat& img_scene, std::vector< DMatch >& good_matches)
		{
			//-- Step 1: Detect the keypoints using ORB Detector
			std::vector<KeyPoint> keypoints_scene;
			detector_.detect( img_scene, keypoints_scene );

			//-- Step 2: Calculate descriptors (feature vectors)
			Mat descriptors_scene;
			extractor_.compute( img_scene, keypoints_scene, descriptors_scene );
			descriptors_scene.convertTo(descriptors_scene, CV_32F);
			if(descriptors_scene.empty())
			{
				imshow( "Camera", img_scene );
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

			//-- Draw only "good" matches - top N matches
			for( unsigned i = 0; i < matches.size() && good_matches.size() < MAX_MATCH_COUNT; ++i )
			{
				if(matches[i].distance < 1.15 * min_dist)
				{
					good_matches.push_back(matches[i]); 
				}
			}

			/* Visual Debug Information - Start */
			// Draw matches between object and scene
			Mat img_matches;
			drawMatches( img_object_, keypoints_object_, img_scene, keypoints_scene,
					good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
					vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

			std::vector<Point2f> scene;
			for( unsigned i = 0; i < good_matches.size(); ++i )
			{
				//-- Get the keypoints from the good matches
				scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
			}

			std::vector<Point2f> hull;
			convexHull(scene, hull);

			for(unsigned i = 0; i < hull.size()-1; ++i)
			{
				line( img_matches, hull[i] + Point2f( img_object_.cols, 0), hull[i+1] + Point2f( img_object_.cols, 0), Scalar(0, 255, 0), 4 );
			}
			line( img_matches, hull[hull.size()-1] + Point2f( img_object_.cols, 0), hull[0] + Point2f( img_object_.cols, 0), Scalar(0, 255, 0), 4 );

			//-- Show detected matches
			imshow( "Camera", img_matches );
			/* Visual Debug Information - End */


			if(good_matches.size() > MIN_MATCH_COUNT)
			{
				return true;
			}
			return false;
		}

	private:
		const unsigned MAX_MATCH_COUNT = 10;
		const unsigned MIN_MATCH_COUNT = 2;

		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;
		std::vector<KeyPoint> keypoints_object_;
		Mat descriptors_object_;
		Mat img_object_;
};
#endif /* OBJECT_DETECTOR_H */
