#ifndef PROB_OBJECT_H
#define PROB_OBJECT_H

#include "image_formats.h"
#include "fusion_base.h"
#include "mavg.h"

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
		ProbObject(ObjectDetector& d, unsigned id, string ip) 
			: FusionBase<pobj_msg>(d, id, ip, create_buffer_fptr(boost::bind(&ProbObject::create_buffer, this, _1 ))),
			object_tracker(num_objects()) {}

		virtual IReport detect(Mat& img_scene);
	private:
		std::vector<double> process_neighbor_msgs(UdpReceiver<pobj_msg>::MessageList neighbor_msgs);
		void create_buffer(std::vector<boost::asio::mutable_buffer>& data);
		std::vector<boost::asio::const_buffer> make_msg(std::vector<unsigned char>& objects);

		const unsigned MSG_RATE = 5;
		const double THRESHOLD = 0.50;
		unsigned image_count_ = 0;
		std::vector<Mavg> object_tracker;
};
#endif /* PROB_OBJECT_H */
