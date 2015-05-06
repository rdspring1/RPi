#ifndef PROB_SUBOBJECT_H
#define PROB_SUBOBJECT_H

#include "image_formats.h"
#include "fusion_base.h"
#include "prob_object.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>

class ProbSubObject : public FusionBase<pobj_msg>
{
	public:
		ProbSubObject(ObjectDetector& d, unsigned id, string ip) 
			: FusionBase<pobj_msg>(d, id, ip, create_buffer_fptr(boost::bind(&ProbSubObject::create_buffer, this, _1 ))),
			image_tracker(num_images()) {}

		virtual IReport detect(Mat& img_scene);
	private:
		std::vector<double> process_neighbor_msgs(UdpReceiver<pobj_msg>::MessageList& neighbor_msgs);
		void create_buffer(std::vector<boost::asio::mutable_buffer>& data);
		std::vector<boost::asio::const_buffer> make_msg(std::vector<unsigned char>& objects);

		const unsigned MSG_RATE = 5;
		const double THRESHOLD = 0.50;
		unsigned image_count_ = 0;
		std::vector<Mavg> image_tracker;
};
#endif /* PROB_SUBOBJECT_H */
