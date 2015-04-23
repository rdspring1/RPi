#ifndef INTERPRETER_BASE_H
#define INTERPRETER_BASE_H

#include "object_detector.h"

class InterpreterBase
{
	public:
		InterpreterBase(ObjectDetector& d) : d_(d) {}

		virtual bool detect(Mat& img_scene) = 0;

		bool processImage(Mat& img_scene, std::vector< DMatch >& good_matches)
		{
			return d_.processImage(img_scene, good_matches);
		}
	private:
		ObjectDetector& d_;
};
#endif /* INTERPRETER_BASE_H */
