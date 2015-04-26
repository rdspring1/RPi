#include "basic_image_detection.h"
#include "image_sharing.h"
#include "prob_object.h"

#include <iostream>
#include <string>
#include <exception>

const int POINTS = 3000;
const int WIDTH = 320;
const int HEIGHT = 240;

/** @function main */
int main( int argc, char** argv )
{
	// Collect Arguments
	if( argc != 3 )
	{ 
		std::cout << " Usage: ./opencv_communication <object_folder> <ip_address>" << std::endl; 
		return -1; 
	}

	// Construct OpenCV video camera interface
	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		std::cout << "Cannot open video camera" << std::endl;
		return 1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	// Initialize object feature
	//Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	Mat img_object = imread( argv[1]);
	if( !img_object.data )
	{ 
		std::cout<< " --(!) Error reading images " << std::endl; 
		return -1; 
	}

	ORB detector(POINTS);
	ORB extractor(POINTS);

	//-- Step 1: Detect the keypoints using ORB Detector
	std::vector<KeyPoint> keypoints_object;
	detector.detect( img_object, keypoints_object );

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_object;
	extractor.compute( img_object, keypoints_object, descriptors_object );
	descriptors_object.convertTo(descriptors_object, CV_32F);

	if(descriptors_object.empty())
	{
		throw std::runtime_error("Missing Object Descriptors");
	}

	// Detection Algorithm
	ObjectDetector d(detector, extractor, keypoints_object, descriptors_object, img_object);

	// Information Fusion Algorithm
	//BasicImageDetection* ib = new BasicImageDetection(d);
	//ImageSharing* ib = new ImageSharing(d, argv[2]);
	ProbObject* ib = new ProbObject(d, argv[2]);

	while(true)
	{
		Mat img_scene;
		cap >> img_scene;
		cvtColor(img_scene, img_scene, CV_RGB2GRAY, 1);

		if( !img_scene.data )
		{ 
			std::cout<< " --(!) Error reading images " << std::endl; 
			return -1; 
		}

		try
		{
			ib->detect(img_scene);
		}
		catch (std::exception ex)
		{
			std::cout << ex.what() << std::endl;
		}

		if(waitKey(50) >= 0)
		{
			break;
		}
	}

	return 0;
}
