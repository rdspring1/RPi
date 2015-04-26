#include "object_library.h"
#include "fps_avg.h"

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

	ORB detector(POINTS);
	ORB extractor(POINTS);

	// Import Object Library
	ObjectLibrary lib(detector, extractor, argv[1]);

	// Detection Algorithm
	ObjectDetector d(detector, extractor, lib);

	// Information Fusion Algorithm
	//BasicImageDetection* ib = new BasicImageDetection(d);
	ImageSharing* ib = new ImageSharing(d, argv[2]);
	//ProbObject* ib = new ProbObject(d, argv[2]);

	FpsAvg fps(5);
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
			fps.update();
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
