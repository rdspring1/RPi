#include "image_sharing.h"

IReport ImageSharing::detect(Mat& img_scene)
{
	ImageData scene = processScene(img_scene);
	for(unsigned idx = 0; idx < num_objects(); ++idx)
	{
		std::vector< DMatch > local_matches;
		object_tracker[idx].update(processObject(scene, object_library().object_img_idx[idx], local_matches));
		debugImage(scene, object_library().object_img_idx[idx], local_matches);
	}

	UdpReceiver<image_msg>::MessageList neighbor_msgs;
	receiver->async_receive_msgs(neighbor_msgs);
	std::vector<double> likelihood = process_neighbor_msgs(neighbor_msgs);

	// send image to neighbors
	// convert image into image_msg
	if(!(image_count_ % MSG_RATE))
	{
		std::vector<unsigned char> outImg;
		cv::imencode(PNG, img_scene, outImg);
		sender->async_send_msg(make_msg(img_scene.type(), img_scene.rows, img_scene.cols, outImg.size(), outImg));
		//std::cout << "Image Size: " << sizeof(img_scene.data) * img_scene.rows * img_scene.cols << std::endl;
		//std::cout << "Compressed Image Size: " << outImg.size() << std::endl;
	}
	++image_count_;

	// Update Bayesian Probability Measure
	// If greater than a certain level of probability, return true
	// Model distribution - Binomial / Bernoulli 
	IReport ir;	
	for(unsigned idx = 0; idx < object_tracker.size(); ++idx)
	{
		double belief = object_tracker[idx].avg();
		bool found = (belief >= THRESHOLD);
		if(!found)
		{
			likelihood[idx] += belief;
			likelihood[idx] /= (neighbor_msgs.size() + 1);
			belief = likelihood[idx];
			found = (belief >= THRESHOLD);
		}

		ir.objects.push_back(found);
		ir.object_confidence.push_back(belief);
	}
	return ir;
}

std::vector<double> ImageSharing::process_neighbor_msgs(UdpReceiver<image_msg>::MessageList& neighbor_msgs)
{
	std::vector<double> likelihood(num_objects());
	//std::cout << "msgs received: " << neighbor_msgs.size() << std::endl;
	for(auto& value : neighbor_msgs)
	{
		std::vector<boost::asio::mutable_buffer>& msg = value.second;
		image_msg* header = boost::asio::buffer_cast<image_msg*>(msg.front());
		unsigned char* body = boost::asio::buffer_cast<unsigned char*>(msg.back());

		// Convert image_msg back into image
		cv::Mat img_buf = cv::Mat(header->rows, header->cols, header->type, body);
		cv::Mat neighbor_image = cv::imdecode(img_buf, CV_LOAD_IMAGE_GRAYSCALE);
		//cv::imshow("neighbor", neighbor_image);

		// Process neighbor image
		// Increment found counter if object is successfully detected
		ImageData neighbor_scene = processScene(neighbor_image);
		for(unsigned idx = 0; idx < num_objects(); ++idx)
		{
			std::vector< DMatch > neighbor_matches;
			likelihood[idx] += (double) processObject(neighbor_scene, object_library().object_img_idx[idx], neighbor_matches);
		}
	}
	return likelihood;
}

void ImageSharing::create_buffer(std::vector<boost::asio::mutable_buffer>& data)
{
	data.push_back(boost::asio::buffer((image_msg*) malloc(sizeof(image_msg)), sizeof(image_msg)));
	data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
}

std::vector<boost::asio::const_buffer> ImageSharing::make_msg(int t, int r, int c, unsigned size, std::vector<unsigned char>& image)
{
	image_msg* msg = new image_msg();
	msg->robot_id = robot_id_;
	msg->type = t;
	msg->rows = r;
	msg->cols = c;
	msg->size = size + sizeof(image_msg);
	msg->timestamp = returnTimestamp(); 
	std::vector<boost::asio::const_buffer> buffer;
	buffer.push_back(boost::asio::buffer(msg, sizeof(image_msg)));
	buffer.push_back(boost::asio::buffer(image));
	return buffer;
} 
