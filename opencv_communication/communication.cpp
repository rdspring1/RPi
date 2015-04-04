#include "udp_sender.h"
#include "udp_receiver.h"
#include "detect.h"
#include "message.h"
#include <iostream>
#include <string>

const short port = 13;

const int WIDTH = 160;
const int HEIGHT = 120;

/** @function readme */
void readme()
{ 
	std::cout << " Usage: ./opencv_communication <object> <ip_address>" << std::endl; 
}

/** @function main */
int main( int argc, char** argv )
{
	// Collect Arguments
	if( argc != 3 )
	{ 
		readme(); 
		return -1; 
	}

	// Setup Bi-Direction Communication
	boost::asio::io_service ios_send;
	boost::asio::io_service ios_receive;
	udp_receiver<msg>* receiver = NULL;
	udp_sender<msg>* sender = NULL;
	try
	{
		receiver = new udp_receiver<msg>(ios_receive, port);
		sender = new udp_sender<msg>(ios_send, boost::asio::ip::address::from_string(argv[1]), port);
		ios_receive.run();
		ios_send.run();
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
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

	// Detection Algorithm
	std::vector<msg> neighbor_matches;
	std::vector<msg> good_matches;
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

		receiver->async_receive_msgs(neighbor_matches);
		bool success = d.processImage(img_scene, good_matches, neighbor_matches);
		if(success)
		{
			// Broadcast good features to neighbors	
			sender->async_send_msgs(good_matches);
			good_matches.clear();
		}

		if(waitKey(50) >= 0)
		{
			break;
		}
	}

	return 0;
}
