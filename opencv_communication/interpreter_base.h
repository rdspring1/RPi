#ifndef INTERPRETER_BASE_H
#define INTERPRETER_BASE_H

#include "object_detector.h"

struct IReport
{
	std::vector<bool> objects;
	std::vector<bool> images;
	std::vector<double> object_confidence;
	std::vector<double> image_confidence;
};

class InterpreterBase
{
	public:
		InterpreterBase(ObjectDetector& d) : d_(d) {}

		virtual IReport detect(Mat& img_scene) = 0;

		unsigned num_objects() const
		{
			return d_.lib_.object_idx.size();
		}

		unsigned num_images() const
		{
			return d_.lib_.image_names.size();
		}

		std::string obj_name(unsigned idx) const
		{
			return d_.lib_.object_names.at(idx);
		}

		std::string img_name(unsigned idx) const
		{
			return d_.lib_.image_names.at(idx);
		}

		const ObjectLibrary& object_library() const
		{
			return d_.lib_;
		}

		void processScene(cv::Mat img_scene)
		{
			d_.processScene(img_scene);
		}

		bool processObject(unsigned idx, std::vector< DMatch >& good_matches)
		{
			return d_.processObject(idx, good_matches);
		}

		void debugImage(unsigned idx, std::vector< DMatch >& good_matches)
		{
			d_.debugImage(idx, good_matches);
		}
	private:
		ObjectDetector& d_;
};
#endif /* INTERPRETER_BASE_H */
