#ifndef IMAGE_SHARING_H
#define IMAGE_SHARING_H

#include "fusion_base.h"
#include "image_formats.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

struct image_msg
{
	unsigned message_id;
	unsigned robot_id;
	int type;
	int rows;
	int cols;
	unsigned size;
}; 

class ImageSharing : public FusionBase<image_msg>
{
	public:
		ImageSharing(ObjectDetector& d, string ip_address) : FusionBase<image_msg>(d, ip_address) {}

		virtual bool detect(Mat& img_scene)
		{
			int found = 0;

			std::vector< DMatch > local_matches;
			found += (int) processImage(img_scene, local_matches);

			std::vector< std::pair<image_msg, unsigned char*> > neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);
			//std::cout << "msgs received: " << neighbor_msgs.size() << std::endl;
			for(std::pair<image_msg, unsigned char*>& img : neighbor_msgs)
			{
				// Convert image_msg back into image
				cv::Mat img_buf = cv::Mat(img.first.rows, img.first.cols, img.first.type, img.second);
				cv::Mat neighbor_image = cv::imdecode(img_buf, CV_LOAD_IMAGE_GRAYSCALE);
				if(!neighbor_image.empty())
				{
					imshow( "Neighbor", neighbor_image );
				}

				// Process neighbor image
				// Increment found counter if object is successfully detected
				std::vector< DMatch > neighbor_matches;
				found += (int) processImage(neighbor_image, neighbor_matches);
			}

			// convert image into image_msg
			if(!(image_count_ % MSG_RATE))
			{
				std::vector<unsigned char> outImg;
				cv::imencode(PNG, img_scene, outImg);
				//std::cout << "Image Size: " << sizeof(img_scene.data) * img_scene.rows * img_scene.cols << std::endl;
				//std::cout << "Compressed Image Size: " << outImg.size() << std::endl;

				// send image to neighbors
				sender->async_send_msg(make_msg(img_scene.type(), img_scene.rows, img_scene.cols, outImg.size()), outImg);
			}

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			belief *= (found / (neighbor_msgs.size() + 1));
			return (belief >= THRESHOLD);
		}
	private:
		const unsigned MSG_RATE = 10;
		unsigned image_count_ = 0;
		const double THRESHOLD = 0.7;
		double belief = 0.5;

		image_msg make_msg(int t, int r, int c, unsigned size)
		{
			image_msg m = {0, 0, t, r, c, size};
			return m;
		} 
};
#endif /* IMAGE_SHARING_H */
