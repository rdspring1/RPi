#ifndef BASIC_IMAGE_DETECTIOM_H
#define BASIC_IMAGE_DETECTION_H

#include "interpreter_base.h"
#include "mavg.h"

#include <list>

class BasicImageDetection : public InterpreterBase
{
	public:
		BasicImageDetection(ObjectDetector& d) : InterpreterBase(d), object_tracker(num_objects()) {}

		virtual IReport detect(Mat& img_scene);
	private:
		const double MATCH_THRESHOLD = 1;
		std::vector<Mavg> object_tracker;
};

#endif /* BASIC_IMAGE_DETECTION_H */
