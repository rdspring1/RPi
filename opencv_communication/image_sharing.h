#include "fusion_base.h"

#include <opencv2/opencv.hpp>

struct image_msg
{
	unsigned message_id;
	unsigned robot_id;
	int type;
	int rows;
	int cols;
	unsigned char* image_buffer;
}; 

image_msg make_msg(unsigned mid, unsigned rid, int t, int r, int c, unsigned char* buf)
{
	image_msg m = {mid, rid, t, r, c, buf};
	return m;
} 

class ImageSharing : public FusionBase<image_msg>
{
	public:
		ImageSharing(ObjectDetector& d, string ip_address) : FusionBase<image_msg>(d, ip_address) {}

		virtual bool detect(Mat& img_scene)
		{
			int found = 0;

			std::vector< DMatch > local_matches;
			bool success = processImage(img_scene, local_matches);

			if(success)
			{
				++found;
			}

			std::vector<image_msg> neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);
			//for(image_msg& img : neighbor_msgs)
			//{
				// TODO convert image_msg back into image
				// TODO process neighbor image
				// TODO increment found counter if object is successfully detected
			//}
			
			// TODO convert image into image_msg
			// TODO send image to neighbors
			// sender->async_send_msgs();

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			return true;
		}
};
