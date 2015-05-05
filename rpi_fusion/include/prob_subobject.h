#ifndef PROB_SUBOBJECT_H
#define PROB_SUBOBJECT_H

#include "fusion_base.h"
#include "image_formats.h"
#include "prob_object.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>

class ProbSubObject : public FusionBase<pobj_msg>
{
	public:
		ProbSubObject(ObjectDetector& d, unsigned robot_id, string ip_address) 
			: FusionBase<pobj_msg>(d, robot_id, ip_address, create_buffer_fptr(boost::bind(&ProbObject::create_buffer, _1 ))),
			image_tracker(num_images()) {}

		virtual IReport detect(Mat& img_scene);
		std::vector<double> process_neighbor_msgs(UdpReceiver<pobj_msg>::MessageList& neighbor_msgs);
		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data);

	private:
		std::vector<boost::asio::const_buffer> make_msg(std::vector<unsigned char>& objects);

		const static unsigned BODY_SIZE = 2; // Number of objects
		const unsigned MSG_RATE = 5;
		const double THRESHOLD = 0.50;
		unsigned image_count_ = 0;
		std::vector<Mavg> image_tracker;
};
#endif /* PROB_SUBOBJECT_H */
