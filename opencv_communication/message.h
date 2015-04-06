#ifndef MESSAGE_H
#define MESSAGE_H

#include <opencv2/opencv.hpp>

struct msg
{
	unsigned message_id;
	unsigned object_id;
	int object_feature_idx;
	cv::Point2f scene;
	float distance;
}; 

msg make_msg(int obj, cv::Point2f& sce, float dist)
{
	msg m = {0, 0, obj, sce, dist};
	return m;
} 

#endif /* MESSAGE_H */
