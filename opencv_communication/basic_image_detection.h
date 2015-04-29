#ifndef BASIC_IMAGE_DETECTIOM_H
#define BASIC_IMAGE_DETECTION_H

#include "interpreter_base.h"
#include "mavg.h"

#include <list>

class BasicImageDetection : public InterpreterBase
{
	public:
		BasicImageDetection(ObjectDetector& d) : InterpreterBase(d), object_tracker(num_objects(), Mavg(MAX_THRESHOLD)) {}

		virtual IReport detect(Mat& img_scene)
		{
			processScene(img_scene);
			for(unsigned idx = 0; idx < num_objects(); ++idx)
			{
				std::vector< DMatch > local_matches;
				object_tracker[idx].update(processObject(object_library().object_idx[idx], local_matches));
				debugImage(object_library().object_idx[idx], local_matches);
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
	private:
		const double MAX_THRESHOLD = 10;
		const double MATCH_THRESHOLD = 1;
		std::vector<Mavg> object_tracker;
};

#endif /* BASIC_IMAGE_DETECTION_H */
