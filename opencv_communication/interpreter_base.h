#ifndef INTERPRETER_BASE_H
#define INTERPRETER_BASE_H

#include "object_detector.h"

class InterpreterBase
{
	public:
		InterpreterBase(ObjectDetector& d) : d_(d) {}

		virtual std::vector<bool> detect(Mat& img_scene) = 0;

		unsigned num_objects() const
		{
			return d_.lib_.object_idx.size();
		}

		unsigned num_images() const
		{
			return d_.lib_.image_names.size();
		}

		const ObjectLibrary& object_library() const
		{
			return d_.lib_;
		}

		bool processObject(unsigned idx, std::vector< DMatch >& good_matches)
		{
			return d_.processObject(idx, good_matches);
		}

		void processScene(cv::Mat img_scene)
		{
			d_.processScene(img_scene);
		}

		void debugImage(unsigned idx, std::vector< DMatch >& good_matches)
		{
			d_.debugImage(idx, good_matches);
		}
	private:
		ObjectDetector& d_;
};
#endif /* INTERPRETER_BASE_H */
