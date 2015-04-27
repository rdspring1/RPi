#ifndef IMAGE_SHARING_H
#define IMAGE_SHARING_H

#include "fusion_base.h"
#include "image_formats.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>

struct image_msg
{
	unsigned message_id;
	unsigned robot_id;
	int type;
	int rows;
	int cols;
	unsigned size;
}; 

class ImageSharing : public FusionBase<image_msg>
{
	public:
		ImageSharing(ObjectDetector& d, string ip_address)
			: FusionBase<image_msg>(d, ip_address, create_buffer_fptr(boost::bind(&ImageSharing::create_buffer, _1 ))),
			  belief(num_objects(), 0.5) {}

		virtual IReport detect(Mat& img_scene)
		{
			processScene(img_scene);
			std::vector<int> found(num_objects(), 0);

			for(unsigned idx = 0; idx < num_objects(); ++idx)
			{
				std::vector< DMatch > local_matches;
				found[idx] += (int) processObject(object_library().object_idx[idx], local_matches);
				debugImage(object_library().object_idx[idx], local_matches);
			}

			std::list< std::vector<boost::asio::mutable_buffer> > neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);
			//std::cout << "msgs received: " << neighbor_msgs.size() << std::endl;
			for(std::vector<boost::asio::mutable_buffer>& msg : neighbor_msgs)
			{
				image_msg* header = boost::asio::buffer_cast<image_msg*>(msg.front());
				unsigned char* body = boost::asio::buffer_cast<unsigned char*>(msg.back());

				// Convert image_msg back into image
				cv::Mat img_buf = cv::Mat(header->rows, header->cols, header->type, body);
				cv::Mat neighbor_image = cv::imdecode(img_buf, CV_LOAD_IMAGE_GRAYSCALE);

				// Process neighbor image
				// Increment found counter if object is successfully detected
				for(unsigned idx = 0; idx < num_objects(); ++idx)
				{
					std::vector< DMatch > neighbor_matches;
					found[idx] += (int) processObject(object_library().object_idx[idx], neighbor_matches);
				}
			}

			// convert image into image_msg
			if(!(image_count_ % MSG_RATE))
			{
				std::vector<unsigned char> outImg;
				cv::imencode(PNG, img_scene, outImg);
				//std::cout << "Image Size: " << sizeof(img_scene.data) * img_scene.rows * img_scene.cols << std::endl;
				//std::cout << "Compressed Image Size: " << outImg.size() << std::endl;

				// send image to neighbors
				sender->async_send_msg(make_msg(img_scene.type(), img_scene.rows, img_scene.cols, outImg.size(), outImg));
			}
			++image_count_;

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			IReport ir;	
			for(unsigned idx = 0; idx < belief.size(); ++idx)
			{
				belief[idx] *= (found[idx] / (neighbor_msgs.size() + 1));
				ir.objects.push_back((belief[idx] >= THRESHOLD));
				ir.object_confidence.push_back(belief[idx]);
				ir.images.push_back((belief[idx] >= THRESHOLD));
				ir.image_confidence.push_back(belief[idx]);
			}
			return ir;
		}

		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data)
		{
			data.push_back(boost::asio::buffer((image_msg*) malloc(sizeof(image_msg)), sizeof(image_msg)));
			data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
		}

	private:
		std::vector<boost::asio::const_buffer> make_msg(int t, int r, int c, unsigned size, std::vector<unsigned char>& image)
		{
			image_msg* msg = new image_msg();
			msg->message_id = 0;
			msg->robot_id = 0;
			msg->type = t;
			msg->rows = r;
			msg->cols = c;
			msg->size = size + sizeof(image_msg);
			std::vector<boost::asio::const_buffer> buffer;
			buffer.push_back(boost::asio::buffer(msg, sizeof(image_msg)));
			buffer.push_back(boost::asio::buffer(image));
			return buffer;
		} 

		const static unsigned BODY_SIZE = 66560; // 65 KB buffer
		const unsigned MSG_RATE = 20;
		const double THRESHOLD = 0.7;
		std::vector<double> belief;
		unsigned image_count_ = 0;

};
#endif /* IMAGE_SHARING_H */
