#include "trackings.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "matchTemplateWithRotPyr.h"
#include "impro_util.h"

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
	std::vector<float> search_x,    // search from (guessedPoint[i].x - search_x[i] / 2) to (guessedPoint[i].x + search_x[i] / 2)
	std::vector<float> prcx,        // demand precision of x. Caution: smaller prcx leads to much longer computing time. Normally prcx hould be >= 0.05. 
	std::vector<float> search_y,    // search from (guessedPoint[i].y - search_y[i] / 2) to (guessedPoint[i].y + search_y[i] / 2)
	std::vector<float> prcy,        // demand precision of y. Caution: smaller prcy leads to much longer computing time. Normally prcy should be >= 0.05.
	std::vector<float> search_r,    // search from (rot_deg[i] - search_r[i] / 2) to (rot_deg[i] + search_r[i] / 2)
	std::vector<float> prcr,        // demand precision of rotation. Caution: smaller prcr leads to much longer computing time. Normally prcr hould be >= 1.
	vector<float> & rot_deg,        // rotaion of each point (in degrees). (Input: initial guess) 
	int method,
	double _init_prec_x, double _init_prec_y, double _init_prec_rot)
{
	bool debug = false; 
	// Variables
	int n; // Number of tracking points
	int ret; // return value
	cv::Mat prevPtsMat, nextPtsMat; 
	// check arguments
	if (prevImg.getMat().cols <= 0 || prevImg.getMat().rows <= 0) 
	{
		cerr << "Error: calcTMatchRotPyr(): prevImg size: " <<
			prevImg.getMat().rows << " X " << prevImg.getMat().cols << endl;
		return -1;
	}
	if (nextImg.getMat().cols <= 0 || nextImg.getMat().rows <= 0)
	{
		cerr << "Error: calcTMatchRotPyr(): nextImg size: " <<
			nextImg.getMat().rows << " X " << nextImg.getMat().cols << endl;
		return -1;
	}
	prevPts.getMat().copyTo(prevPtsMat);  // prevPtsMat = prevPts.getMat(); 
	prevPtsMat.convertTo(prevPtsMat, CV_32FC2); 
	if (prevPtsMat.cols > prevPtsMat.rows) prevPtsMat = prevPtsMat.t();

	n = prevPts.getMat().cols * prevPts.getMat().rows;
	if (n <= 0) {
		cerr << "Error: calcTMatchRotPyr(): prevPts size: " <<
			prevPts.getMat().rows << " X " << prevPts.getMat().cols << endl;
		return -1;
	}
	prevPtsMat = prevPtsMat.reshape(0, n).clone(); // make prevPtsMat one-column
	if (nextPts.getMat().cols * nextPts.getMat().rows != n)
	{
		prevPts.copyTo(nextPts);
	}
	nextPts.getMat().copyTo(nextPtsMat); 
	nextPtsMat.convertTo(nextPtsMat, CV_32FC2);
	if (nextPtsMat.cols > nextPtsMat.rows) nextPtsMat = nextPtsMat.t();

	status.getMat() = cv::Mat::ones(n, 1, CV_8U);
	err.getMat() = cv::Mat::zeros(n, 1, CV_32F);
	if (rot_deg.size() != n) 
		rot_deg.resize(n, 0.0); 

	// if winSize is not large enough, resize it, and fill it with the original last element 
	if (winSize.size() < n) {
		int ori_size = (int) winSize.size();
		winSize.resize(n); 
		for (int i = ori_size; i < winSize.size(); i++)
			winSize[i] = winSize[ori_size - 1];
	}
	if (debug) cout << "prevPts:\n" << prevPts.getMat() << endl;
	if (debug) cout << "prevPtsMat:\n" << prevPtsMat << endl;

	// run tracking
	for (int iPoint = 0; iPoint < n; iPoint++)
	{
		// define template
		cv::Point2f refPoint;
		refPoint.x = ptsWinRatios[iPoint].x * (winSize[iPoint].width - 1);
		refPoint.y = ptsWinRatios[iPoint].y * (winSize[iPoint].height - 1);
		cv::Rect rect = getTmpltRectFromImageSizeWithPreferredRef(
			prevImg.getMat().size(),
			prevPtsMat.at<cv::Point2f>(iPoint, 0),
			winSize[iPoint],
			refPoint);
		cv::Mat imgTmplt;
		prevImg.getMat()(rect).copyTo(imgTmplt);
		// run template match
		vector<double> result(10, 0.0);
		if (debug) cout << nextPtsMat.at<cv::Point2f>(iPoint, 0).x << endl;
		if (debug) cout << nextPtsMat.at<cv::Point2f>(iPoint, 0).y << endl;
		ret = matchTemplateWithRotPyr(
			nextImg,
			imgTmplt,
			refPoint.x,
			refPoint.y,
			nextPtsMat.at<cv::Point2f>(iPoint, 0).x - search_x[iPoint] / 2,
			nextPtsMat.at<cv::Point2f>(iPoint, 0).x + search_x[iPoint] / 2,
			prcx[iPoint],
			nextPtsMat.at<cv::Point2f>(iPoint, 0).y - search_y[iPoint] / 2,
			nextPtsMat.at<cv::Point2f>(iPoint, 0).y + search_y[iPoint] / 2,
			prcy[iPoint],
			rot_deg[iPoint] - search_r[iPoint] / 2,
			rot_deg[iPoint] - search_r[iPoint] / 2,
			prcr[iPoint],
			result,
			method, _init_prec_x, _init_prec_y, _init_prec_rot
		);
		nextPtsMat.at<cv::Point2f>(iPoint, 0).x = (float) result[0];
		nextPtsMat.at<cv::Point2f>(iPoint, 0).y = (float) result[1];
		rot_deg[iPoint] = (float) result[2];
	}

	if (debug) {
		cv::Mat prevImgMat = prevImg.getMat().clone();
		cv::Mat nextImgMat = nextImg.getMat().clone();
		drawPointsOnImage(prevImgMat, prevPtsMat, "square", 16, 4, cv::Scalar(255)); 
		drawPointsOnImage(nextImgMat, nextPtsMat, "square", 16, 4, cv::Scalar(255));
		imshow_resize("Prev", prevImgMat, 0.5); 
		imshow_resize("Curr", nextImgMat, 0.5);
//		cv::waitKey(0); 
	}


	// data copy back
	if (prevPts.getMat().cols > prevPts.getMat().rows)
		nextPtsMat = nextPtsMat.t(); 
	nextPtsMat.copyTo(nextPts); 
	
	return 0 ;
}

////! calcEcc() allows user to use enhanced correlation coefficient method with
//// different motion types
//// through an interface similar to calcOpticalFlowLKPyr()
///*!
//*/
//void calcEcc(cv::InputArray prevImg, cv::InputArray nextImg,
//	cv::InputArray prevPts, cv::InputOutputArray nextPts,
//	cv::OutputArray status, cv::OutputArray err,
//	cv::Size winSize,
//	int motionType,
//	vector<cv::Mat> & warpMatrices, // warp matrix of each points
//	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 0.001))
//{

//	return;
//}


