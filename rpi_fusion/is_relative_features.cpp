#include "is_relative_features.h"

IReport ISRelativeFeatures::detect(Mat& img_scene)
{
	UdpReceiver<image_msg>::MessageList neighbor_msgs;
	receiver->async_receive_msgs(neighbor_msgs);

	ImageData local_scene = processScene(img_scene);
	for(unsigned idx = 0; idx < num_objects(); ++idx)
	{
		int image_idx = object_library().object_img_idx[idx];
		ImageData& object = img_data(image_idx);

		Features local_matches;
		processObject(local_scene, image_idx, local_matches);

		Features hinted_matches;
		for(auto& value : neighbor_msgs)
		{
			std::vector<boost::asio::mutable_buffer> msg = value.second;
			image_msg* header = boost::asio::buffer_cast<image_msg*>(msg.front());
			unsigned char* body = boost::asio::buffer_cast<unsigned char*>(msg.back());

			// Convert image_msg back into image
			cv::Mat img_buf = cv::Mat(header->rows, header->cols, header->type, body);
			cv::Mat neighbor_image = cv::imdecode(img_buf, CV_LOAD_IMAGE_GRAYSCALE);
			ImageData neighbor_scene = processScene(neighbor_image);

			// Process neighbor image
			Features neighbor_matches;
			processObject(neighbor_scene, image_idx, neighbor_matches);
			process_neighbor_msg(object, local_scene, local_matches, neighbor_matches, hinted_matches);
		}

		// Compute Homography
		// Update result for the object
		std::vector<Point2f> mpts_1;
		std::vector<Point2f> mpts_2;
		mpts_1.reserve(local_matches.size() + hinted_matches.size());
		mpts_2.reserve(local_matches.size() + hinted_matches.size());

		for(DMatch& match : local_matches)
		{
			mpts_1.push_back(object.keypoints.at(match.queryIdx).pt);
			mpts_2.push_back(local_scene.keypoints.at(match.trainIdx).pt);
		}

		for(DMatch& match : hinted_matches)
		{
			mpts_1.push_back(object.keypoints.at(match.queryIdx).pt);
			mpts_2.push_back(local_scene.keypoints.at(match.trainIdx).pt);
		}

		std::vector<size_t> inliers = computeHomography(mpts_1, mpts_2, MIN_MATCH_COUNT);
		object_tracker[idx].update((inliers.size() >= MIN_MATCH_COUNT));
		debugImage(local_scene, object_library().object_img_idx[idx], local_matches);
	}

	// convert image into image_msg
	if(!(image_count_ % MSG_RATE))
	{
		std::vector<unsigned char> outImg;
		cv::imencode(PNG, img_scene, outImg);
		// send image to neighbors
		sender->async_send_msg(make_msg(img_scene.type(), img_scene.rows, img_scene.cols, outImg.size(), outImg));
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

void ISRelativeFeatures::process_neighbor_msg(const ImageData& obj, const ImageData& local_scene, 
	const Features& local_matches, const Features& neighbor_matches, Features& hinted_matches)
{
	for(const cv::DMatch& local_match : local_matches)
	{
		// Find nearest neighbor match in object image - Euclidean Distance
		const cv::Point2f& local_obj = obj.keypoints.at(local_match.queryIdx).pt;
		const cv::DMatch* nn_idx = NULL;
		{
			double minDist = DBL_MAX;
			for(const cv::DMatch& match : neighbor_matches)
			{
				double dist = cv::norm(local_obj - obj.keypoints.at(match.queryIdx).pt);
				if(dist < minDist)
				{
					minDist = dist;
					nn_idx = &match;
				}
			}
		} 

		// Compute Relative Position in local scene (B -> A)
		const cv::Point2f& nn_obj = obj.keypoints.at(nn_idx->queryIdx).pt;
		cv::Point2f diff = nn_obj - local_obj;
		const cv::Point2f& local_scene_pt = local_scene.keypoints.at(local_match.trainIdx).pt; 
		cv::Point2f BA_pt = local_scene_pt + diff;

		// Out of Bounds Check for relative mapping point - BA_pt
		std::vector<bool> outBounds(4);
		outBounds[0] = (BA_pt.x < 0);
		outBounds[1] = (BA_pt.y < 0);
		outBounds[3] = (BA_pt.x >= local_scene.image.cols);
		outBounds[2] = (BA_pt.y >= local_scene.image.rows);
		if(std::count(outBounds.begin(), outBounds.end(), true) > 0)
		{
			continue;
		}

		// Find nearest keypoint in local scene other than local scene point (B -> A)
		int BA_scene_idx = 0;
		{
			double minDist = DBL_MAX;
			int idx = 0;
			for(const cv::KeyPoint& kp : local_scene.keypoints)
			{
				if(kp.pt != local_scene_pt)
				{
					double dist = cv::norm(BA_pt - kp.pt);
					if(dist < minDist)
					{
						minDist = dist;
						BA_scene_idx = idx;
					}
				}
				++idx;
			}
		}

		// Determine if the distance between scene and object descriptors passes NNDR
		double hint_distance = 0;
		if(obj.descriptors.type() == CV_8U)
		{
			// Binary Descriptors - ORB
			hint_distance = cv::norm(obj.descriptors.row(nn_idx->queryIdx), local_scene.descriptors.row(BA_scene_idx), NORM_HAMMING);
		}
		else
		{
			// Float Descriptors - SIFT, SURF
			hint_distance = cv::norm(obj.descriptors.row(nn_idx->queryIdx), local_scene.descriptors.row(BA_scene_idx), NORM_L2);
		}

		if(hint_distance <= NNDR_RATIO * local_match.distance)
		{
			printf("q=%d dist1=%f dist2=%f\n", nn_idx->queryIdx, hint_distance, local_match.distance);
			hinted_matches.push_back(cv::DMatch(nn_idx->queryIdx, BA_scene_idx, hint_distance));
		}
	}
}

void ISRelativeFeatures::create_buffer(std::vector<boost::asio::mutable_buffer>& data)
{
	data.push_back(boost::asio::buffer((image_msg*) malloc(sizeof(image_msg)), sizeof(image_msg)));
	data.push_back(boost::asio::buffer((unsigned char*) malloc(BODY_SIZE), BODY_SIZE));
}

std::vector<boost::asio::const_buffer> ISRelativeFeatures::make_msg(int t, int r, int c, 
	unsigned size, std::vector<unsigned char>& image)
{
	image_msg* msg = new image_msg();
	msg->robot_id = robot_id_;
	msg->type = t;
	msg->rows = r;
	msg->cols = c;
	msg->size = size + sizeof(image_msg);
	msg->timestamp = returnTimestamp(); 
	std::vector<boost::asio::const_buffer> buffer;
	buffer.push_back(boost::asio::buffer(msg, sizeof(image_msg)));
	buffer.push_back(boost::asio::buffer(image));
	return buffer;
} 

