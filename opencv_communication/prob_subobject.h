#ifndef PROB_SUBOBJECT_H
#define PROB_SUBOBJECT_H

#include "fusion_base.h"
#include "image_formats.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>

class ProbSubObject : public FusionBase<pobj_msg>
{
	public:
		ProbSubObject(ObjectDetector& d, string ip_address) 
			: FusionBase<pobj_msg>(d, ip_address, create_buffer_fptr(boost::bind(&ProbObject::create_buffer, _1 ))),
			image_tracker(num_images()) {}

		virtual IReport detect(Mat& img_scene)
		{
			// # of times the object is detected
			ImageData scene = processScene(img_scene);
			for(unsigned idx = 0; idx < num_images(); ++idx)
			{
				std::vector< DMatch > local_matches;
				image_tracker[idx].update(processObject(scene, idx, local_matches));
			}

			std::list< std::vector<boost::asio::mutable_buffer> > neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);
			std::vector<double> likelihood = process_neighbor_msgs(neighbor_msgs);

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			IReport ir;
			std::vector<unsigned char> out(num_images());
			for(unsigned idx = 0; idx < image_tracker.size(); ++idx)
			{
				double belief = image_tracker[idx].avg();
				bool found = (belief >= THRESHOLD);
				out[idx] = (unsigned char) found;
				if(!found)
				{
					likelihood[idx] += belief;
					likelihood[idx] /= (neighbor_msgs.size() + 1);
					belief = likelihood[idx];
					found = (belief >= THRESHOLD);
				}
	
				ir.images.push_back(found);
				ir.image_confidence.push_back(belief);
			}

			for(unsigned idx = 0; idx < num_objects(); ++idx)
			{
				const unsigned LIMIT = (idx == num_objects()-1) 
					? num_images() : object_library().object_idx[idx+1] ;
				//const double SIZE = LIMIT - object_library().object_idx[idx]; 

				double belief = 0;
				for(unsigned jdx = object_library().object_idx[idx]; jdx < LIMIT; ++jdx)
				{
					belief += ir.image_confidence[jdx];
				}

				//belief /= ((neighbor_msgs.size()+1) * SIZE);
				ir.objects.push_back((belief >= THRESHOLD));
				ir.object_confidence.push_back(belief);
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

		std::vector<double> process_neighbor_msgs(std::list< std::vector<boost::asio::mutable_buffer> >& neighbor_msgs)
		{
			std::vector<double> likelihood(num_images());
			//std::cout << "msgs received: " << neighbor_msgs.size() << std::endl;
			for(std::vector<boost::asio::mutable_buffer>& msg : neighbor_msgs)
			{
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
		std::vector<Mavg> image_tracker;

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
#endif /* PROB_SUBOBJECT_H */
