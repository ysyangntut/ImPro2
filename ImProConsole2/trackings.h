#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std; 

//! calcTMatchRotPyr() allows user to use pyramid template match with 
// optional rotation tracking (which calls matchTemplateWithRotPyr()) 
// through an interface similar to calcOpticalFlowLKPyr()
/*!
*/
int calcTMatchRotPyr(
	cv::InputArray prevImg,			// previous image (i.e., image containing templates)
	cv::InputArray nextImg,			// current image (i.e., searched image)
	cv::InputArray prevPts,			// tracking points in previous image (i.e., center points of templates)
	cv::InputOutputArray nextPts,	// tracking points in current image (Input: initial guess)
	cv::OutputArray status,			// 1:found, 0:not found 
	cv::OutputArray err,			// 
	vector<cv::Size> winSize,		// window size (i.e., template size)
	std::vector<cv::Point2f> ptsWinRatios,	// vector<n, Point2f(.5f,.5f)> means points are at window center. vector<n, Point2f(.0f,.0f)> means points are at upper-left corner of window, etc.
	std::vector<float> search_x,                // search from (guessedPoint[i].x - search_x[i] / 2) to (guessedPoint[i].x + search_x[i] / 2)
	std::vector<float> prcx,                    // demand precision of x. Caution: smaller prcx leads to much longer computing time. Normally prcx hould be >= 0.05. 
	std::vector<float> search_y,                // search from (guessedPoint[i].y - search_y[i] / 2) to (guessedPoint[i].y + search_y[i] / 2)
	std::vector<float> prcy,                    // demand precision of y. Caution: smaller prcy leads to much longer computing time. Normally prcy should be >= 0.05.
	std::vector<float> search_r,                // search from (rot_deg[i] - search_r[i] / 2) to (rot_deg[i] + search_r[i] / 2)
	std::vector<float> prcr,                    // demand precision of rotation. Caution: smaller prcr leads to much longer computing time. Normally prcr hould be >= 1.
	vector<float> & rot_deg,                    // rotaion of each point (in degrees). (Input: initial guess) 
	int method = cv::TM_CCORR_NORMED,
	double _init_prec_x = -1, double _init_prec_y = -1, double _init_prec_rot = -1);

//! calcEcc() allows user to use enhanced correlation coefficient method with 
// different motion types
// through an interface similar to calcOpticalFlowLKPyr()
/*!
*/
//void calcEcc(cv::InputArray prevImg, cv::InputArray nextImg,
//	cv::InputArray prevPts, cv::InputOutputArray nextPts,
//	cv::OutputArray status, cv::OutputArray err,
//	cv::Size winSize,
//	int motionType,
//	vector<cv::Mat> & warpMatrices, // warp matrix of each points
//	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 0.001));


