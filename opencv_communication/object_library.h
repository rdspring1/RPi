#ifndef OBJECT_LIBRARY_H
#define OBJECT_LIBRARY_H

#include <string>
#include <vector>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

using namespace cv;
using namespace boost::filesystem;

struct ImageData
{
	std::string name;
	cv::Mat image;
	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
};

class ObjectLibrary
{
	public:
		ObjectLibrary(FeatureDetector& detector, DescriptorExtractor& extractor) :
			detector_(detector), extractor_(extractor) {}

		ObjectLibrary(FeatureDetector& detector, DescriptorExtractor& extractor, const char* filepath) :
			detector_(detector), extractor_(extractor)
	{
		processFolder(filepath);
	}

		ImageData processImage(Mat& img);
		void processFolder(const char* filepath);

		// the value for each object is its start index in the bag of images
		// Images: Box - 6 sides | Hexagon Cylinder - 8
		// Start Index: Box - 0 | Hexagon Cylinder - 6
		std::vector< int > object_idx;
		std::vector< std::string > object_names;
		std::vector< int > object_img_idx;
		std::vector< ImageData > images;

	private:
		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;
};

#endif /* OBJECT_LIBRARY_H */
