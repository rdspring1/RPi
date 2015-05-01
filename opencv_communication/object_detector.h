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
		}

		bool processObject(unsigned object_idx, std::vector<DMatch>& good_matches)
		{
			// Object Reference
			Mat& descriptors_object = lib_.images[object_idx].descriptors;
			std::vector<KeyPoint>& keypoints_object = lib_.images[object_idx].keypoints;

			Mat results;
			Mat dists;
			std::vector<std::vector<cv::DMatch> > matches;
			if(descriptors_object.type() == CV_8U)
			{
				// Binary Descriptors - ORB
				cv::BFMatcher matcher(cv::NORM_HAMMING);
				matcher.knnMatch(descriptors_object, descriptors_scene_, matches, k);
			}
			else
			{
				// Float Descriptors - SIFT, SURF
				cv::BFMatcher matcher(cv::NORM_L2);
				matcher.knnMatch(descriptors_object, descriptors_scene_, matches, k);
			}

			std::vector<int> pt_index;
			std::vector<cv::Point2f> mpts_1, mpts_2;

			// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
			// Check if this descriptor matches with those of the objects
			for(unsigned idx = 0; idx < matches.size(); ++idx)
			{
				// Apply NNDR
				if(matches.at(idx).size() == 2 &&
						matches.at(idx).at(0).distance <= nndrRatio * matches.at(idx).at(1).distance)
				{
					//printf("q=%d dist1=%f dist2=%f\n", matches.at(idx).at(0).queryIdx, 
					//	matches.at(idx).at(0).distance, matches.at(idx).at(1).distance);
					pt_index.push_back(idx);
					mpts_1.push_back(keypoints_object.at(matches.at(idx).at(0).queryIdx).pt);
					mpts_2.push_back(keypoints_scene_.at(matches.at(idx).at(0).trainIdx).pt);
				}
			}

			unsigned inliers = 0;
			unsigned outliers = 0;
			std::vector<uchar> outlier_mask;
			if(pt_index.size() >= MIN_MATCH_COUNT)
			{
				cv::Mat H = findHomography(mpts_1,
						mpts_2,
						cv::RANSAC,
						1.0,
						outlier_mask);

				for(unsigned int idx = 0; idx < mpts_1.size(); ++idx)
				{
					if(outlier_mask.at(idx))
					{
						++inliers;
						good_matches.push_back(matches.at(pt_index[idx]).at(0));
					}
					else
					{
						++outliers;
					}
				}

				if(inliers >= MIN_MATCH_COUNT)
				{
					std::cout << "INLIERS: "  << inliers << " OUTLIERS: " << outliers << std::endl;
					return true;
				}
			}
			return false;
		}

		void debugImage(std::string name, unsigned object_idx, std::vector<DMatch>& good_matches)
		{
			Mat& img_object = lib_.images[object_idx].image;
			std::vector<KeyPoint>& keypoints_object = lib_.images[object_idx].keypoints;

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
			imshow( name, img_matches );
			/* Visual Debug Information - End */
		}

		ObjectLibrary lib_;
	private:
		const double nndrRatio = 0.8f;
		const unsigned k = 2;
		const unsigned MIN_CONVEX_HULL = 3;
		const unsigned MIN_MATCH_COUNT = 14;

		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;

		std::vector<KeyPoint> keypoints_scene_;
		Mat descriptors_scene_;
		Mat img_scene_;
};
#endif /* OBJECT_DETECTOR_H */
