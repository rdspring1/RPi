#ifndef PROB_OBJECT_H
#define PROB_OBJECT_H

#include "fusion_base.h"
#include "image_formats.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>

struct pobj_msg
{
	unsigned message_id;
	unsigned robot_id;
	unsigned num_objects;
	unsigned size;
}; 

class ProbObject : public FusionBase<pobj_msg>
{
	public:
		ProbObject(ObjectDetector& d, string ip_address) 
			: FusionBase<pobj_msg>(d, ip_address, create_buffer_fptr(boost::bind(&ProbObject::create_buffer, _1 ))) {}

		virtual bool detect(Mat& img_scene)
		{
			// # of times the object is detected
			int found = 0;

			std::vector< DMatch > local_matches;
			bool success = processImage(img_scene, local_matches);
			found += (int) success;

			std::list< std::vector<boost::asio::mutable_buffer> > neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);
			//std::cout << "msgs received: " << neighbor_msgs.size() << std::endl;
			for(std::vector<boost::asio::mutable_buffer>& msg : neighbor_msgs)
			{
				pobj_msg* header = boost::asio::buffer_cast<pobj_msg*>(msg.front());
				bool* body = boost::asio::buffer_cast<bool*>(msg.back());

				// Process neighbor message
				for(unsigned idx = 0; idx < header->num_objects; ++idx)
				{
					found += (int) body[idx];
				}
			}

			// convert image into pobj_msg
			if(!(image_count_ % MSG_RATE))
			{
				std::vector<unsigned char> out;
				out.push_back( (unsigned char) success);
				// send image to neighbors
				sender->async_send_msg(make_msg(out));
			}
			++image_count_;

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			belief *= (found / (neighbor_msgs.size() + 1));
			return (belief >= THRESHOLD);
		}

		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data)
		{
			data.push_back(boost::asio::buffer((pobj_msg*) malloc(sizeof(pobj_msg)), sizeof(pobj_msg)));
			data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
		}

	private:
		const static unsigned BODY_SIZE = 2; // Number of objects
		const unsigned MSG_RATE = 5;
		const double THRESHOLD = 0.7;
		double belief = 0.5;
		unsigned image_count_ = 0;

		std::vector<boost::asio::const_buffer> make_msg(std::vector<unsigned char>& objects)
		{
			pobj_msg* msg = new pobj_msg();
			msg->message_id = 0;
			msg->robot_id = 0;
			msg->num_objects = objects.size();
			msg->size = sizeof(bool)*objects.size() + sizeof(pobj_msg);
			std::vector<boost::asio::const_buffer> buffer;
			buffer.push_back(boost::asio::buffer(msg, sizeof(pobj_msg)));
			buffer.push_back(boost::asio::buffer(objects));
			return buffer;
		} 
};
#endif /* PROB_OBJECT_H */
