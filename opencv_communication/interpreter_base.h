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
			return d_.lib_.images.size();
		}

		std::string obj_name(unsigned idx) const
		{
			return d_.lib_.object_names.at(idx);
		}

		std::string img_name(unsigned idx) const
		{
			return d_.lib_.images.at(idx).name;
		}

		ImageData img(unsigned idx)
		{
			return d_.lib_.images.at(idx);
		}

		const ObjectLibrary& object_library() const
		{
			return d_.lib_;
		}

		ImageData processScene(cv::Mat img_scene)
		{
			return d_.processScene(img_scene);
		}

		bool processObject(ImageData& scene, unsigned idx, std::vector< DMatch >& good_matches)
		{
			return d_.processObject(scene, idx, good_matches);
		}

		void debugImage(ImageData& scene, unsigned idx, std::vector< DMatch >& good_matches)
		{
			d_.debugImage(img_name(idx), scene, idx, good_matches);
		}
	private:
		ObjectDetector& d_;
};
#endif /* INTERPRETER_BASE_H */
