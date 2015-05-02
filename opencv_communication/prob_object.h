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
	unsigned robot_id;
	unsigned num_objects;
	unsigned size;
	double timestamp;
}; 

class ProbObject : public FusionBase<pobj_msg>
{
	public:
		ProbObject(ObjectDetector& d, unsigned robot_id, string ip_address) 
			: FusionBase<pobj_msg>(d, robot_id, ip_address, create_buffer_fptr(boost::bind(&ProbObject::create_buffer, _1 ))),
			object_tracker(num_objects()) {}

		virtual IReport detect(Mat& img_scene)
		{
			// # of times the object is detected
			ImageData scene = processScene(img_scene);
			for(unsigned idx = 0; idx < num_objects(); ++idx)
			{
				std::vector< DMatch > local_matches;
				object_tracker[idx].update(processObject(scene, object_library().object_img_idx[idx], local_matches));
			}

			UdpReceiver<pobj_msg>::MessageList neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);
			std::vector<double> likelihood = process_neighbor_msgs(neighbor_msgs);

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			IReport ir;
			std::vector<unsigned char> out(num_objects());
			for(unsigned idx = 0; idx < object_tracker.size(); ++idx)
			{
				double belief = object_tracker[idx].avg();
				bool found = (belief >= THRESHOLD);
				out[idx] = (unsigned char) found;
				if(!found)
				{
					likelihood[idx] += belief;
					likelihood[idx] /= (neighbor_msgs.size() + 1);
					belief = likelihood[idx];
					found = (belief >= THRESHOLD);
				}

				ir.objects.push_back(found);
				ir.object_confidence.push_back(belief);
				ir.images.push_back(found);
				ir.image_confidence.push_back(belief);
			}

			// convert image into pobj_msg
			if(!(image_count_ % MSG_RATE))
			{
				// send image to neighbors
				sender->async_send_msg(make_msg(out));
			}
			++image_count_;
			return ir;
		}

		std::vector<double> process_neighbor_msgs(UdpReceiver<pobj_msg>::MessageList neighbor_msgs)
		{
			std::vector<double> likelihood(num_objects());
			//std::cout << "msgs received: " << neighbor_msgs.size() << std::endl;
			for(auto& data : neighbor_msgs)
			{
				std::vector<boost::asio::mutable_buffer>& msg = data.second;
				pobj_msg* header = boost::asio::buffer_cast<pobj_msg*>(msg.front());
				bool* body = boost::asio::buffer_cast<bool*>(msg.back());

				// Process neighbor message
				for(unsigned idx = 0; idx < header->num_objects; ++idx)
				{
					likelihood[idx] += (double) body[idx];
				}
			}
			return likelihood;
		}

		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data)
		{
			data.push_back(boost::asio::buffer((pobj_msg*) malloc(sizeof(pobj_msg)), sizeof(pobj_msg)));
			data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
		}

	private:
		const static unsigned BODY_SIZE = 2; // Number of objects
		const unsigned MSG_RATE = 5;
		const double THRESHOLD = 0.75;
		unsigned image_count_ = 0;
		std::vector<Mavg> object_tracker;

		std::vector<boost::asio::const_buffer> make_msg(std::vector<unsigned char>& objects)
		{
			pobj_msg* msg = new pobj_msg();
			msg->robot_id = robot_id_;
			msg->num_objects = objects.size();
			msg->size = sizeof(bool)*objects.size() + sizeof(pobj_msg);
			msg->timestamp = returnTimestamp(); 
			std::vector<boost::asio::const_buffer> buffer;
			buffer.push_back(boost::asio::buffer(msg, sizeof(pobj_msg)));
			buffer.push_back(boost::asio::buffer(objects));
			return buffer;
		} 
};
#endif /* PROB_OBJECT_H */
