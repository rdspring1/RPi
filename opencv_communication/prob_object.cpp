#include "prob_object.h"

IReport ProbObject::detect(Mat& img_scene)
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

std::vector<double> ProbObject::process_neighbor_msgs(UdpReceiver<pobj_msg>::MessageList neighbor_msgs)
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

void ProbObject::create_buffer(std::vector<boost::asio::mutable_buffer>& data)
{
	data.push_back(boost::asio::buffer((pobj_msg*) malloc(sizeof(pobj_msg)), sizeof(pobj_msg)));
	data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
}

std::vector<boost::asio::const_buffer> ProbObject::make_msg(std::vector<unsigned char>& objects)
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

