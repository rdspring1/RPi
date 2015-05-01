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

		void processFolder(const char* filepath)
		{
			// Create object library
			path library_path(filepath);
			if(is_directory(library_path))
			{
				recursive_directory_iterator iter(library_path);
				recursive_directory_iterator end;
				while(iter != end)
				{
					if(is_directory(iter->path()))
					{
						object_idx.push_back(images.size());
						object_names.push_back(iter->path().filename().generic_string());
						//std::cout << "Object: " <<  object_names.back() << " " << object_idx.back() << std::endl;
					}
					else
					{
						ImageData img;
						img.name = iter->path().filename().generic_string();
						//std::cout << "Image: " << img.name << std::endl;

						// Initialize object feature
						Mat img_object = imread(iter->path().generic_string(), CV_LOAD_IMAGE_COLOR);
						if( !img_object.data )
						{ 
							throw std::runtime_error("Error Reading Image"); 
						}
						img.image = img_object;

						//-- Step 1: Detect the keypoints using ORB Detector
						detector_.detect( img_object, img.keypoints );

						//-- Step 2: Calculate descriptors (feature vectors)
						extractor_.compute( img_object, img.keypoints, img.descriptors );
						if(img.descriptors.empty())
						{
							throw std::runtime_error("Missing Object Descriptors");
						}

						//-- Step 3: Add to object library
						images.push_back(std::move(img));
					}
					++iter;
				}
			}
			//std::cout << "Num Objects: " << object_idx.size() << std::endl;
		}

		// the value for each object is its start index in the bag of images
		// Images: Box - 6 sides | Hexagon Cylinder - 8
		// Start Index: Box - 0 | Hexagon Cylinder - 6
		std::vector< int > object_idx;
		std::vector< std::string > object_names;
		std::vector< ImageData > images;

	private:
		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;
};

#endif /* OBJECT_LIBRARY_H */
