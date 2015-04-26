#ifndef BASIC_IMAGE_DETECTIOM_H
#define BASIC_IMAGE_DETECTION_H

#include "interpreter_base.h"

#include <list>

class BasicImageDetection : public InterpreterBase
{
	public:
		BasicImageDetection(ObjectDetector& d) : InterpreterBase(d), match_list(MAX_THRESHOLD, false) {}

		virtual std::vector<bool> detect(Mat& img_scene)
		{
			processScene(img_scene);
			std::vector< DMatch > local_matches;
			bool success = processObject(0, local_matches);
			debugImage(0, local_matches);

			// Determine if object is detected
			if(match_list.front())
			{
				--match_count;
			}	
			match_list.pop_front();

			if(success)
			{
				++match_count;
				std::cout << "-- matches : " << local_matches.size() << std::endl;
			}
			match_list.push_back(success);

			if(match_count >= MATCH_THRESHOLD)
			{	
				std::cout << "MATCH DETECTED: " << match_count << std::endl;
				return std::vector<bool>(1, true);
			}

			std::cout << "NO MATCH DETECTED: " << match_count << std::endl;
			return std::vector<bool>(1, false);
		}
	private:
		const double MAX_THRESHOLD = 10;
		const double MATCH_THRESHOLD = 1;
		std::list<bool> match_list;
		int match_count = 0;
};

#endif /* BASIC_IMAGE_DETECTION_H */
