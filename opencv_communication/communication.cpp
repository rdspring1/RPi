#include "udp_sender.h"
#include "udp_receiver.h"
#include "detect.h"
#include <iostream>
#include <string>

const std::string SEND = "SEND";

#define WIDTH 160
#define HEIGHT 120

/** @function readme */
void readme()
{ 
	std::cout << " Usage: ./opencv_communication <object> <ip_address>" << std::endl; 
}

/** @function main */
int main( int argc, char** argv )
{
	const short port = 13;
	boost::asio::io_service ioservice;
	udp_receiver* receiver = NULL;
	udp_sender* sender = NULL;
	try
	{
		receiver = new udp_receiver(ioservice, port);
		sender = new udp_sender(ioservice, boost::asio::ip::address::from_string(argv[1]), port);
		ioservice.run();
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	if( argc != 3 )
	{ 
		readme(); 
		return -1; 
	}

	//Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	Mat img_object = imread( argv[1]);
	if( !img_object.data )
	{ 
		std::cout<< " --(!) Error reading images " << std::endl; 
		return -1; 
	}


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

	detect d(detector, keypoints_object, descriptors_object, img_object);
	while(true)
	{
		Mat img_scene;
		cap >> img_scene;
		//cvtColor(img_scene, img_scene, CV_RGB2GRAY, 1);

		if( !img_scene.data )
		{ 
			std::cout<< " --(!) Error reading images " << std::endl; 
			return -1; 
		}

		d.processImage(img_scene);

		if(waitKey(50) >= 0)
		{
			break;
		}
	}

	return 0;
}
