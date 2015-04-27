#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "object_library.h"

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
		ObjectDetector(FeatureDetector& detector, DescriptorExtractor& extractor, const char* filepath)
			: lib_(detector, extractor, filepath), detector_(detector), extractor_(extractor) 
		{}

		ObjectDetector(FeatureDetector& detector, DescriptorExtractor& extractor, ObjectLibrary& lib)
			: lib_(std::move(lib)), detector_(detector), extractor_(extractor) 
		{}

		void processScene(Mat& img_scene)
		{
			//-- Step 0: Store scene image
			img_scene_ = img_scene;

			//-- Step 1: Detect the keypoints using ORB Detector
			keypoints_scene_.clear();
			detector_.detect( img_scene, keypoints_scene_ );

			//-- Step 2: Calculate descriptors (feature vectors)
			descriptors_scene_.release();
			extractor_.compute( img_scene, keypoints_scene_, descriptors_scene_ );
			descriptors_scene_.convertTo(descriptors_scene_, CV_32F);
		}

		void debugImage(unsigned object_idx, std::vector<DMatch>& good_matches)
		{
			Mat& img_object = lib_.images[object_idx];
			std::vector<KeyPoint>& keypoints_object = lib_.keypoints_objects[object_idx];

			/* Visual Debug Information - Start */
			// Draw matches between object and scene
			Mat img_matches;
			drawMatches( img_object, keypoints_object, img_scene_, keypoints_scene_,
					good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
					vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

			if(good_matches.size() > MIN_CONVEX_HULL)
			{
				std::vector<Point2f> scene;
				for( unsigned i = 0; i < good_matches.size(); ++i )
				{
					//-- Get the keypoints from the good matches
					scene.push_back( keypoints_scene_[ good_matches[i].trainIdx ].pt );
				}

				std::vector<Point2f> hull;
				convexHull(scene, hull);

				for(unsigned i = 0; i < hull.size()-1; ++i)
				{
					line( img_matches, 
							hull[i] + Point2f( img_object.cols, 0), 
							hull[i+1] + Point2f( img_object.cols, 0), 
							Scalar(0, 255, 0), 4 );
				}
				line( img_matches, 
						hull[hull.size()-1] + Point2f( img_object.cols, 0), 
						hull[0] + Point2f( img_object.cols, 0), 
						Scalar(0, 255, 0), 4 );

			}
			//-- Show detected matches
			imshow( "object", img_matches );
			/* Visual Debug Information - End */
		}

		bool processObject(unsigned object_idx, std::vector<DMatch>& good_matches)
		{
			Mat& descriptors_object = lib_.descriptors_objects[object_idx];

			//-- Step 3: Matching descriptor vectors using FLANN matcher
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match( descriptors_object, descriptors_scene_, matches );

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

			if(good_matches.size() > MIN_MATCH_COUNT)
			{
				return true;
			}
			return false;
		}

		ObjectLibrary lib_;
	private:
		const unsigned MIN_CONVEX_HULL = 3;
		const unsigned MAX_MATCH_COUNT = 10;
		const unsigned MIN_MATCH_COUNT = 2;

		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;

		std::vector<KeyPoint> keypoints_scene_;
		Mat descriptors_scene_;
		Mat img_scene_;
};
#endif /* OBJECT_DETECTOR_H */
