#ifndef BASIC_IMAGE_DETECTIOM_H
#define BASIC_IMAGE_DETECTION_H

#include "interpreter_base.h"

#include <list>

class BasicImageDetection : public InterpreterBase
{
	public:
		BasicImageDetection(ObjectDetector& d) : InterpreterBase(d), match_list(MAX_THRESHOLD, false) {}

		virtual IReport detect(Mat& img_scene)
		{
			processScene(img_scene);
			std::vector< DMatch > local_matches;
			bool success = processObject(0, local_matches);
			if(success)
			{
				debugImage(0, local_matches);
			}

			// Determine if object is detected
			if(match_list.front())
			{
				--match_count;
			}	
			match_list.pop_front();

			if(success)
			{
				++match_count;
				//std::cout << "-- matches : " << local_matches.size() << std::endl;
			}
			match_list.push_back(success);

			IReport ir;
			if(match_count >= MATCH_THRESHOLD)
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
			return ir;
		}
	private:
		const double MAX_THRESHOLD = 10;
		const double MATCH_THRESHOLD = 1;
		std::list<bool> match_list;
		int match_count = 0;
};

#endif /* BASIC_IMAGE_DETECTION_H */
