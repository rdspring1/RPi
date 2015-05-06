#ifndef IS_RELATIVE_FEATURES_H
#define IS_RELATIVE_FEATURES_H

#include "image_formats.h"
#include "fusion_base.h"
#include "image_sharing.h"
#include "mavg.h"

#include <cfloat>
#include <utility>
#include <stdio.h>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class ISRelativeFeatures : public FusionBase<image_msg>
{
	public:
		typedef std::vector< DMatch > Features;

		ISRelativeFeatures(ObjectDetector& d, unsigned id, string ip_address)
			: FusionBase<image_msg>(d, id, ip_address, create_buffer_fptr(boost::bind(&ISRelativeFeatures::create_buffer, _1 ))), 
			object_tracker(num_objects()) {}

		virtual IReport detect(Mat& img_scene);
		static void process_neighbor_msg(const ImageData& obj, const ImageData& local_scene, 
				const Features& local_matches, const Features& neighbor_matches, Features& hinted_matches);
	private:
		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data);
		std::vector<boost::asio::const_buffer> make_msg(int t, int r, int c, unsigned size, std::vector<unsigned char>& image);

		constexpr static double NNDR_RATIO = 2.0;
		const unsigned MIN_MATCH_COUNT = 14;
		const static unsigned BODY_SIZE = 66560; // 65 KB buffer
		const unsigned MSG_RATE = 20;
		const double THRESHOLD = 0.50;
		unsigned image_count_ = 0;
		std::vector<Mavg> object_tracker;
};
#endif /* IS_RELATIVE_FEATURES_H */
