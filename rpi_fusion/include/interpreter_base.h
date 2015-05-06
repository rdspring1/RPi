#ifndef INTERPRETER_BASE_H
#define INTERPRETER_BASE_H

#include "object_detector.h"

#include <time.h>

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
		InterpreterBase(ObjectDetector& d) : d_(d) 
		{
			struct tm bt = {0};
			bt.tm_hour = 0;
			bt.tm_min = 0;
			bt.tm_sec = 0;
			bt.tm_year = 100;
			bt.tm_mon = 0;
			bt.tm_mday = 1;
			basetime = mktime(&bt);
		}

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

		ImageData& img_data(unsigned idx)
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

		std::vector<size_t> computeHomography(std::vector<Point2f>& mpts_1, std::vector<Point2f>& mpts_2, const size_t threshold)
		{
			return d_.computeHomography(mpts_1, mpts_2, threshold);
		}

		void debugImage(ImageData& scene, unsigned idx, std::vector< DMatch >& good_matches)
		{
			d_.debugImage(img_name(idx), scene, idx, good_matches);
		}

		double returnTimestamp() const
		{
			time_t current_time;
			time(&current_time);
			return difftime(current_time, basetime);
		}

		virtual ~InterpreterBase() {}
	private:
		ObjectDetector& d_;
		time_t basetime;
};
#endif /* INTERPRETER_BASE_H */
