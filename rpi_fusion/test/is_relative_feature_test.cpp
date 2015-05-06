#include "is_relative_features.h"

#include <iostream>
#include <string>
#include <exception>
#include <stdlib.h> 

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

const unsigned MIN_MATCH_COUNT = 14;
const int POINTS = 1500;

std::vector<cv::KeyPoint> extract_keypoints(ImageData& img, ISRelativeFeatures::Features matches, bool object = true)
{
	std::vector<cv::KeyPoint> keypoints;
	for(DMatch& match : matches)
	{
		if(object)
		{
			keypoints.push_back(img.keypoints.at(match.queryIdx));
		}
		else
		{
			keypoints.push_back(img.keypoints.at(match.trainIdx));
		}
	}
	return keypoints;
}

/** @function main */
int main( int argc, char** argv )
{
	// Collect Arguments
	if( argc != 4 )
	{ 
		std::cout << " Usage: ./rpi_fusion <object_folder> <scene1> <scene2>" << std::endl; 
		return -1; 
	}

	Mat img_scene = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	if( !img_scene.data )
	{ 
		throw std::runtime_error("Error Reading Image"); 
	}

	Mat neighbor_image = imread(argv[3], CV_LOAD_IMAGE_GRAYSCALE);
	if( !img_scene.data )
	{ 
		throw std::runtime_error("Error Reading Image"); 
	}

	// Detection Algorithm
	ORB detector(POINTS);
	ORB extractor(POINTS);
	ObjectDetector d(detector, extractor, argv[1]);

	// Information Fusion Algorithm
	ImageData local_scene = d.processScene(img_scene);
	ImageData neighbor_scene = d.processScene(neighbor_image);

	int image_idx = d.lib_.object_img_idx.front();
	ImageData& object = d.lib_.images.at(image_idx);

	ISRelativeFeatures::Features local_matches;
	ISRelativeFeatures::Features hinted_matches;
	ISRelativeFeatures::Features neighbor_matches;

	d.processObject(local_scene, image_idx, local_matches);
	d.processObject(neighbor_scene, image_idx, neighbor_matches);
	ISRelativeFeatures::process_neighbor_msg(object, local_scene, local_matches, neighbor_matches, hinted_matches);

	// Compute Homography
	// Update result for the object
	std::vector<Point2f> mpts_1;
	std::vector<Point2f> mpts_2;
	mpts_1.reserve(local_matches.size() + hinted_matches.size());
	mpts_2.reserve(local_matches.size() + hinted_matches.size());

	for(DMatch& match : local_matches)
	{
		mpts_1.push_back(object.keypoints.at(match.queryIdx).pt);
		mpts_2.push_back(local_scene.keypoints.at(match.trainIdx).pt);
	}

	for(DMatch& match : hinted_matches)
	{
		mpts_1.push_back(object.keypoints.at(match.queryIdx).pt);
		mpts_2.push_back(local_scene.keypoints.at(match.trainIdx).pt);
	}

	std::vector<size_t> inliers = d.computeHomography(mpts_1, mpts_2, MIN_MATCH_COUNT);

	// Object Matches
	cv::Mat object_keypoint_image;
	std::vector<cv::KeyPoint> object_local_keypoints = extract_keypoints(object, local_matches);
	std::vector<cv::KeyPoint> object_hinted_keypoints = extract_keypoints(object, hinted_matches);
	cv::drawKeypoints(object.image, object_local_keypoints, object_keypoint_image, cv::Scalar(255,0,0));
	cv::drawKeypoints(object_keypoint_image, object_hinted_keypoints, object_keypoint_image, cv::Scalar(0,0,255));
	cv::imshow("object", object_keypoint_image);

	// Scene Matches
	cv::Mat scene_keypoint_image;
	std::vector<cv::KeyPoint> scene_local_keypoints = extract_keypoints(local_scene, local_matches, false);
	std::vector<cv::KeyPoint> scene_hinted_keypoints = extract_keypoints(local_scene, hinted_matches, false);
	cv::drawKeypoints(local_scene.image, scene_local_keypoints, scene_keypoint_image, cv::Scalar(255,0,0));
	cv::drawKeypoints(scene_keypoint_image, scene_hinted_keypoints, scene_keypoint_image, cv::Scalar(0,0,255));
	cv::imshow("scene", scene_keypoint_image);

	bool detect = inliers.size() >= MIN_MATCH_COUNT;
	std::cout << "Hinted Features: " << hinted_matches.size() << std::endl;
	std::cout << "Detection: " << detect << std::endl; 

	while(true)
	{
		if(waitKey(50) >= 0)
		{
			break;
		}
	}
	return 0;
}
