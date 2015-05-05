#include "benchmark.h"
#include "object_library.h"
#include "fps_avg.h"

#include "basic_image_detection.h"
#include "image_sharing.h"
#include "prob_object.h"
#include "prob_subobject.h"

#include <iostream>
#include <string>
#include <exception>
#include <stdlib.h> 

const long TEST_DURATION = 60;
const int POINTS = 1500;
const int WIDTH = 320;
const int HEIGHT = 240;

/** @function main */
int main( int argc, char** argv )
{
	// Collect Arguments
	if( argc != 4 )
	{ 
		std::cout << " Usage: ./opencv_communication <object_folder> <robot_id> <ip_address>" << std::endl; 
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

	// Detection Algorithm
	ObjectDetector d(detector, extractor, argv[1]);

	// Information Fusion Algorithm
	//BasicImageDetection* basic = new BasicImageDetection(d);

	{
		ImageSharing* is = new ImageSharing(d, atoi(argv[2]), argv[3]);
		Benchmark b(cap, *is, TEST_DURATION);	
		b.run();
		delete is;
	}
	{
		ProbObject* po = new ProbObject(d, atoi(argv[2]), argv[3]);
		Benchmark b(cap, *po, TEST_DURATION);	
		b.run();
		delete po;
	}
	{
		ProbSubObject* pso = new ProbSubObject(d, atoi(argv[2]), argv[3]);
		Benchmark b(cap, *pso, TEST_DURATION);	
		b.run();
		delete pso;
	}

	return 0;
}
