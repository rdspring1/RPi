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
						object_idx.push_back(image_names.size());
						object_names.push_back(iter->path().filename().generic_string());
						//std::cout << "Object: " <<  object_names.back() << " " << object_idx.back() << std::endl;
					}
					else
					{
						image_names.push_back(iter->path().filename().generic_string());
						//std::cout << "Image: " << image_names.back() << std::endl;

						// Initialize object feature
						Mat img_object = imread(iter->path().generic_string(), CV_LOAD_IMAGE_COLOR);
						if( !img_object.data )
						{ 
							throw std::runtime_error("Error Reading Image"); 
						}
						images.push_back(img_object);

						//-- Step 1: Detect the keypoints using ORB Detector
						std::vector<KeyPoint> keypoints_object;
						detector_.detect( img_object, keypoints_object );

						//-- Step 2: Calculate descriptors (feature vectors)
						Mat descriptors_object;
						extractor_.compute( img_object, keypoints_object, descriptors_object );
						descriptors_object.convertTo(descriptors_object, CV_32F);

						if(descriptors_object.empty())
						{
							throw std::runtime_error("Missing Object Descriptors");
						}

						//-- Step 3: Add to object library
						keypoints_objects.push_back(std::move(keypoints_object));
						descriptors_objects.push_back(std::move(descriptors_object));
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

		std::vector< std::string > image_names;
		std::vector< cv::Mat > images;
		std::vector< cv::Mat > descriptors_objects;
		std::vector< std::vector<cv::KeyPoint> > keypoints_objects;

	private:
		FeatureDetector& detector_;
		DescriptorExtractor& extractor_;
};

#endif /* OBJECT_LIBRARY_H */
