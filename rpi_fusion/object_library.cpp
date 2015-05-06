#include "object_library.h"

ImageData ObjectLibrary::processImage(Mat& img)
{
	ImageData data;

	//-- Step 0: Store scene image
	data.image = img;

	//-- Step 1: Detect the keypoints using ORB Detector
	detector_.detect( data.image, data.keypoints );

	//-- Step 2: Calculate descriptors (feature vectors)
	extractor_.compute( data.image, data.keypoints, data.descriptors );
	if(data.descriptors.empty())
	{
		throw std::runtime_error("Missing Object Descriptors");
	}
	return data;
}

void ObjectLibrary::processFolder(const char* filepath)
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

				// Handle Symlink Directories
				if(is_symlink(iter->path()))
				{
					iter = recursive_directory_iterator(canonical(read_symlink(iter->path()), library_path));
				}
			}
			else
			{
				// Initialize object feature
				Mat img_object = imread(iter->path().generic_string(), CV_LOAD_IMAGE_COLOR);
				if( !img_object.data )
				{ 
					throw std::runtime_error("Error Reading Image"); 
				}
				ImageData img = processImage(img_object);
				img.name = iter->path().stem().generic_string();
				//std::cout << "Object: " << object_names.back() << " Image: " << img.name << std::endl;

				if(img.name == object_names.back())
				{
					object_img_idx.push_back(images.size());
					//std::cout << object_names.back() << " " << images.size() << std::endl;
				}

				//-- Step 3: Add to object library
				images.push_back(std::move(img));
			}
			++iter;
		}
	}
	//std::cout << "Num Objects: " << object_idx.size() << std::endl;
}


