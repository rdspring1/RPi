#include "object_detector.h"

bool ObjectDetector::processObject(ImageData& scene, unsigned object_idx, std::vector<DMatch>& good_matches)
{
	// Object Reference
	Mat& descriptors_object = lib_.images[object_idx].descriptors;
	std::vector<KeyPoint>& keypoints_object = lib_.images[object_idx].keypoints;

	Mat results;
	Mat dists;
	std::vector< std::vector<cv::DMatch> > matches;
	if(descriptors_object.type() == CV_8U)
	{
		// Binary Descriptors - ORB
		cv::BFMatcher matcher(cv::NORM_HAMMING);
		matcher.knnMatch(descriptors_object, scene.descriptors, matches, k);
	}
	else
	{
		// Float Descriptors - SIFT, SURF
		cv::BFMatcher matcher(cv::NORM_L2);
		matcher.knnMatch(descriptors_object, scene.descriptors, matches, k);
	}

	std::vector<int> pt_index;
	std::vector<cv::Point2f> mpts_1, mpts_2;

	// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
	// Check if this descriptor matches with those of the objects
	for(unsigned idx = 0; idx < matches.size(); ++idx)
	{
		// Apply NNDR
		if(matches.at(idx).size() == 2 &&
				matches.at(idx).at(0).distance <= nndrRatio * matches.at(idx).at(1).distance)
		{
			//printf("q=%d dist1=%f dist2=%f\n", matches.at(idx).at(0).queryIdx, 
			//	matches.at(idx).at(0).distance, matches.at(idx).at(1).distance);
			pt_index.push_back(idx);
			mpts_1.push_back(keypoints_object.at(matches.at(idx).at(0).queryIdx).pt);
			mpts_2.push_back(scene.keypoints.at(matches.at(idx).at(0).trainIdx).pt);
		}
	}

	std::vector<size_t> inliers = computeHomography(mpts_1, mpts_2, MIN_MATCH_COUNT);
	for(size_t& idx : inliers)
	{
		good_matches.push_back(matches.at(pt_index[idx]).at(0));
	}

	if(inliers.size() >= MIN_MATCH_COUNT)
	{
		//std::cout << "INLIERS: "  << inliers << " OUTLIERS: " << outliers << std::endl;
		return true;
	}
	return false;
}

std::vector<size_t> ObjectDetector::computeHomography(std::vector<Point2f>& mpts_1, 
		std::vector<Point2f>& mpts_2, const size_t threshold)
{
	unsigned inliers = 0;
	unsigned outliers = 0;
	std::vector<uchar> outlier_mask;
	std::vector<size_t> good_matches;
	if(mpts_1.size() >= threshold)
	{
		cv::Mat H = findHomography(mpts_1,
				mpts_2,
				cv::RANSAC,
				1.0,
				outlier_mask);

		for(size_t idx = 0; idx < mpts_1.size(); ++idx)
		{
			if(outlier_mask.at(idx))
			{
				++inliers;
				good_matches.push_back(idx);
			}
			else
			{
				++outliers;
			}
		}
	}
	return good_matches;
}

void ObjectDetector::debugImage(std::string name, ImageData& scene, unsigned object_idx, std::vector<DMatch>& good_matches)
{
	Mat& img_object = lib_.images[object_idx].image;
	std::vector<KeyPoint>& keypoints_object = lib_.images[object_idx].keypoints;

	/* Visual Debug Information - Start */
	// Draw matches between object and scene
	Mat img_matches;
	drawMatches( img_object, keypoints_object, scene.image, scene.keypoints,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	if(good_matches.size() > MIN_CONVEX_HULL)
	{
		std::vector<Point2f> scene_pts;
		for( unsigned i = 0; i < good_matches.size(); ++i )
		{
			//-- Get the keypoints from the good matches
			scene_pts.push_back( scene.keypoints[ good_matches[i].trainIdx ].pt );
		}

		std::vector<Point2f> hull;
		convexHull(scene_pts, hull);

		for(unsigned i = 0; i < hull.size()-1; ++i)
		{
			line( img_matches, 
					hull[i] + Point2f( img_object.cols, 0), 
					hull[i+1] + Point2f( img_object.cols, 0), 
					Scalar(0, 255, 0), 4 );
		}
		line( img_matches, 
				hull[hull.size()-1] + Point2f( img_object.cols, 0), 
				hull[0] + Point2f( img_object.cols, 0), 
				Scalar(0, 255, 0), 4 );

	}
	//-- Show detected matches
	imshow( name, img_matches );
	/* Visual Debug Information - End */
}
