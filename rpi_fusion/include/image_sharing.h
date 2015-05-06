#ifndef IMAGE_SHARING_H
#define IMAGE_SHARING_H

#include "image_formats.h"
#include "fusion_base.h"
#include "mavg.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

struct image_msg
{
	unsigned robot_id;
	int type;
	int rows;
	int cols;
	unsigned size;
	double timestamp;
}; 

class ImageSharing : public FusionBase<image_msg>
{
	public:
		ImageSharing(ObjectDetector& d, unsigned id, string ip)
			: FusionBase<image_msg>(d, id, ip, create_buffer_fptr(boost::bind(&ImageSharing::create_buffer, _1 ))),
			  object_tracker(num_objects()) {}

		virtual IReport detect(Mat& img_scene);
	private:
		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data);
		std::vector<double> process_neighbor_msgs(UdpReceiver<image_msg>::MessageList& neighbor_msgs);
		std::vector<boost::asio::const_buffer> make_msg(int t, int r, int c, unsigned size, std::vector<unsigned char>& image);

		const static unsigned BODY_SIZE = 66560; // 65 KB buffer
		const unsigned MSG_RATE = 10;
		const double THRESHOLD = 0.50;
		unsigned image_count_ = 0;
		std::vector<Mavg> object_tracker;
};
#endif /* IMAGE_SHARING_H */
