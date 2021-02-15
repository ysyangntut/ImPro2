#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

int enhancedCorrelationWithReference
(InputArray image, InputArray templ,
	double ref_x, double ref_y,
	double init_x, double init_y, double init_rot,
	vector<double> &  dispAndRot,
	int motionType = cv::MOTION_EUCLIDEAN,
	cv::TermCriteria criteria =
	cv::TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 0.001)
); 

