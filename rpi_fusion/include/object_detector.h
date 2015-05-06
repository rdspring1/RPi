#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "object_library.h"

// STL
#include <stdio.h>
#include <iostream>
#include <exception>
#include <vector>
#include <list>
#include <algorithm>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

class ObjectDetector
{
	public:
		ObjectDetector(FeatureDetector& detector, DescriptorExtractor& extractor, const char* filepath)
			: lib_(detector, extractor, filepath), detector_(detector), extractor_(extractor) 
		{}

		ObjectDetector(FeatureDetector& detector, DescriptorExtractor& extractor, ObjectLibrary& lib)
			: lib_(std::move(lib)), detector_(detector), extractor_(extractor) 
		{}

		ImageData processScene(Mat& img_scene)
		{
			return lib_.processImage(img_scene);
		}

		bool processObject(ImageData& scene, unsigned object_idx, std::vector<DMatch>& good_matches);
		std::vector<size_t> computeHomography(std::vector<Point2f>& mpts_1, std::vector<Point2f>& mpts_2, const size_t threshold);
		void debugImage(std::string name, ImageData& scene, unsigned object_idx, std::vector<DMatch>& good_matches);

		ObjectLibrary lib_;
	private:
		const double nndrRatio = 0.8f;
		const unsigned k = 2;
		const unsigned MIN_CONVEX_HULL = 3;
		const unsigned MIN_MATCH_COUNT = 14;

		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;
};
#endif /* OBJECT_DETECTOR_H */
