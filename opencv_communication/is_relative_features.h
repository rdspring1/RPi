#ifndef IS_RELATIVE_FEATURES_H
#define IS_RELATIVE_FEATURES_H

#include "fusion_base.h"
#include "image_formats.h"

#include <utility>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>

struct image_msg
{
	unsigned message_id;
	unsigned robot_id;
	int type;
	int rows;
	int cols;
	unsigned size;
};

class ISRelativeFeatures : public FusionBase<image_msg>
{
	public:
		typedef std::vector< DMatch > Features;

		ImageSharing(ObjectDetector& d, string ip_address)
			: FusionBase<image_msg>(d, ip_address, create_buffer_fptr(boost::bind(&ImageSharing::create_buffer, _1 ))),
			object_tracker(num_objects()) {}

		virtual IReport detect(Mat& img_scene)
		{
			std::list< std::vector<boost::asio::mutable_buffer> > neighbor_msgs;
			receiver->async_receive_msgs(neighbor_msgs);

			ImageData local_scene = processScene(img_scene);
			for(unsigned idx = 0; idx < num_objects(); ++idx)
			{
				int image_idx = object_library().object_img_idx[idx];
				Features local_matches;
				processObject(local_scene, image_idx, local_matches);

				for(std::vector<boost::asio::mutable_buffer>& msg : neighbor_msgs)
				{
					image_msg* header = boost::asio::buffer_cast<image_msg*>(msg.front());
					unsigned char* body = boost::asio::buffer_cast<unsigned char*>(msg.back());

					// Convert image_msg back into image
					cv::Mat img_buf = cv::Mat(header->rows, header->cols, header->type, body);
					cv::Mat neighbor_image = cv::imdecode(img_buf, CV_LOAD_IMAGE_GRAYSCALE);
					ImageData neighbor_scene = processScene(neighbor_image);

					// Process neighbor image
					Features neighbor_matches;
					processObject(neighbor_scene, image_idx, neighbor_matches);

					process_neighbor_msg(img(image_idx), local_scene, neighbor_scene, local_matches, neighbor_matches);

					// TODO Compute Homography and Threshold
					// TODO Update result for the object
				}
				debugImage(local_scene, object_library().object_img_idx[idx], local_matches);
			}

			// convert image into image_msg
			if(!(image_count_ % MSG_RATE))
			{
				std::vector<unsigned char> outImg;
				cv::imencode(PNG, img_scene, outImg);
				// send image to neighbors
				sender->async_send_msg(make_msg(img_scene.type(), img_scene.rows, img_scene.cols, outImg.size(), outImg));

				//std::cout << "Image Size: " << sizeof(img_scene.data) * img_scene.rows * img_scene.cols << std::endl;
				//std::cout << "Compressed Image Size: " << outImg.size() << std::endl;
			}
			++image_count_;

			// Update Bayesian Probability Measure
			// If greater than a certain level of probability, return true
			// Model distribution - Binomial / Bernoulli 
			IReport ir;	
			for(unsigned idx = 0; idx < object_tracker.size(); ++idx)
			{
				double belief = object_tracker[idx].avg();
				//std::cout << "belief: " << belief << std::endl;
				bool found = (belief >= THRESHOLD);

				ir.objects.push_back(found);
				ir.object_confidence.push_back(belief);
				ir.images.push_back(found);
				ir.image_confidence.push_back(belief);
			}
			return ir;
		}

		void process_neighbor_msg(ImageData& obj, ImageData& local_scene, ImageData& neighbor_scene, 
				Features& local_matches, Features& neighbor_matches)
		{
			const size_t SIZE = local_matches.size();
			for(size_t idx = 0; idx < SIZE; ++idx)
			{
				// TODO Find nearest neighbor match
				//DMatch nn = nearest_neighbor(local_matches[idx], neighbor_matches);

				// TODO Compute Relative Position (B -> A)
				
				// TODO Find nearest keypoint in local scene (B -> A)

				// TODO Compute Relative Position (A -> B)
				
				// TODO Find nearest keypoint in neighbor scene (A -> B)

				// TODO Determine distance between guess and object keypoint passes NNDR

				// TODO Add new match to local set
			}
		}

		static void create_buffer(std::vector<boost::asio::mutable_buffer>& data)
		{
			data.push_back(boost::asio::buffer((image_msg*) malloc(sizeof(image_msg)), sizeof(image_msg)));
			data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
		}

	private:
		std::vector<boost::asio::const_buffer> make_msg(int t, int r, int c, unsigned size, std::vector<unsigned char>& image)
		{
			image_msg* msg = new image_msg();
			msg->message_id = 0;
			msg->robot_id = 0;
			msg->type = t;
			msg->rows = r;
			msg->cols = c;
			msg->size = size + sizeof(image_msg);
			std::vector<boost::asio::const_buffer> buffer;
			buffer.push_back(boost::asio::buffer(msg, sizeof(image_msg)));
			buffer.push_back(boost::asio::buffer(image));
			return buffer;
		} 

		const static unsigned BODY_SIZE = 66560; // 65 KB buffer
		const unsigned MSG_RATE = 20;
		const double THRESHOLD = 0.75;
		unsigned image_count_ = 0;
		std::vector<Mavg> object_tracker;
};
#endif /* IS_RELATIVE_FEATURES_H */
