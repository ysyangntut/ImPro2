#include <vector>
#include <string>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "impro_util.h"
#include "enhancedCorrelationWithReference.h"

// 
// int enhancedCorrelationWithReference
//                                   (InputArray image, InputArray templ, 
//                                    double ref_x,    double ref_y, 
//                                    double init_x,   double init_y, double init_rot,  
//                                    vector<double> &  dispAndRot,
//                                    int motionType = cv::MOTION_EUCLIDEAN, 
//                                    cv::TermCriteria criteria = 
//                                    cv::TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 0.001)
//                                   );
// 
// Description: 
//   enhancedCorrelationWithReference() runs enchanced correlation coefficient method
//   considering ux, uy, and rotation. 
// 
// Parameters: 
// 
//   InputArray              image 
//     Image where the search is running. (It must be 8-bit or 32-bit floating-point.?)
// 
//   InputArray              templ 
//     Searched template. It must be not greater than the source image and have the same data type.
//     
//   double                  ref_x, ref_y
//     reference point of the template (in pixel, upper-left is 0.0, 0.0) 
//
//   double                  init_x, init_y, init_r
//     initially guessed position of reference point
//
//   vector<double>        & result
//     searched most matched location and rotation (x, y, rotataion in degree)
//       result[0]:  px in pixel
//       result[1]:  py in pixel
//       result[2]:  rotation in degree
//       result[3]:  best matched value 
//       result[4]:  total cpu time
//   
//   int                     return value
//      0: done successfully
//     -1: unsuccessfully
// 
//   Developer: Yuan-Sen Yang
//   Date: 2017/12/19 - initial implementation
// 

int enhancedCorrelationWithReference
(InputArray image, InputArray templ,
	double ref_x, double ref_y,
	double init_x, double init_y, double init_rot,
	vector<double> &  dispAndRot,
	int motionType,
	cv::TermCriteria criteria
)
{
	cv::Mat warpMat, initWarp33(3, 3, CV_32F), warp33(3, 3, CV_32F);
	vector<double> tmpres(10);

	float rx = (float)ref_x; 
	float ry = (float)ref_y; 
	float dx = (float)init_x;
	float dy = (float)init_y;
	float rz = (float)init_rot * (3.14159265359f / 180.f);
	double Ux, Uy, Rz;

	/// ----------------------------------------------------------------------------
	double ticCpus = getCpusTime();
	
	cv::Mat tmp1 = (Mat_<float>(3, 3) << 1, 0, -rx, 0, 1, -ry, 0, 0, 1);
	cv::Mat tmp2 = (Mat_<float>(3, 3) << cos(rz), sin(rz), 0, -sin(rz), cos(rz), 0, 0, 0, 1);
	cv::Mat tmp3 = (Mat_<float>(3, 3) << 1, 0, dx, 0, 1, dy, 0, 0, 1);

	initWarp33 = tmp3 * tmp2 * tmp1;

	if (motionType == cv::MOTION_TRANSLATION ||
		motionType == cv::MOTION_EUCLIDEAN ||
		motionType == cv::MOTION_AFFINE) {
		warpMat = cv::Mat::zeros(2, 3, CV_32F);
		cv::Mat(initWarp33, cv::Rect(0, 0, 3, 2)).copyTo(warpMat);
	}
	else /* if (motionType == cv::MOTION_HOMOGRAPHY) */ {
		warpMat = cv::Mat::eye(3, 3, CV_32F);
		initWarp33.copyTo(warpMat);
	}

	double coef;
	try {
		cv::Mat templ_gray, image_gray;
		if (templ.channels() != 1)
			cv::cvtColor(templ, templ_gray, cv::COLOR_BGR2GRAY);
		else
			templ_gray = templ.getMat(); 
		if (image.channels() != 1)
			cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
		else
			image_gray = image.getMat();
		coef = cv::findTransformECC(templ_gray, image_gray, warpMat, motionType, criteria);
	}
	catch (...)
	{
		return -1; 
	}

	if (warpMat.rows == 3)
		warp33 = warpMat;
	else
		warp33 = (Mat_<float>(3, 3) <<
			warpMat.at<float>(0, 0), warpMat.at<float>(0, 1), warpMat.at<float>(0, 2),
			warpMat.at<float>(1, 0), warpMat.at<float>(1, 1), warpMat.at<float>(1, 2),
			0, 0, 1);

	cv::Mat tmp4 = (Mat_<float>(3, 3) <<
		warp33.at<float>(0, 0), warp33.at<float>(0, 1), 0,
		warp33.at<float>(1, 0), warp33.at<float>(1, 1), 0,
		0, 0, 1);
	
	cv::Mat tmp5 = warp33 * tmp1.inv() * tmp2.inv();

	Ux = tmp5.at<float>(0, 2) / tmp5.at<float>(2, 2);
	Uy = tmp5.at<float>(1, 2) / tmp5.at<float>(2, 2);
	Rz = atan2(warp33.at<float>(1, 0), warp33.at<float>(0, 0)) * 180 / 3.1415926;

	double tocCpus = getCpusTime() - ticCpus;

	if (dispAndRot.size() < 5) dispAndRot.resize(5); 
	dispAndRot[0] = Ux;
	dispAndRot[1] = Uy;
	dispAndRot[2] = Rz;
	dispAndRot[3] = coef;
	dispAndRot[4] = tocCpus;
	return 0; 
}