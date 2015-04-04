#ifndef MESSAGE_H
#define MESSAGE_H

#include <opencv2/opencv.hpp>

struct msg
{
	unsigned long message_id;
	cv::Point2f object;
	cv::Point2f scene;
	float distance;
}; 

msg make_msg(cv::Point2f& obj, cv::Point2f& sce, float dist)
{
	msg m = {0, obj, sce, dist};
	return m;
} 

#endif /* MESSAGE_H */
