#include "basic_image_detection.h"

IReport BasicImageDetection::detect(Mat& img_scene)
{
	ImageData scene = processScene(img_scene);
	for(unsigned idx = 0; idx < num_objects(); ++idx)
	{
		std::vector< DMatch > local_matches;
		object_tracker[idx].update(processObject(scene, object_library().object_img_idx[idx], local_matches));
		debugImage(scene, object_library().object_img_idx[idx], local_matches);
	}

	IReport ir;
	for(unsigned idx = 0; idx < num_objects(); ++idx)
	{
		if(object_tracker[idx].count() >= MATCH_THRESHOLD)
		{	
			//std::cout << "MATCH DETECTED: " << match_count << std::endl;
			ir.objects.push_back(true);
			ir.object_confidence.push_back(1);
			ir.images.push_back(true);
			ir.image_confidence.push_back(1);
		}
		else
		{
			//std::cout << "NO MATCH DETECTED: " << match_count << std::endl;
			ir.objects.push_back(false);
			ir.object_confidence.push_back(0);
			ir.images.push_back(false);
			ir.image_confidence.push_back(0);
		}
	}
	return ir;
}
