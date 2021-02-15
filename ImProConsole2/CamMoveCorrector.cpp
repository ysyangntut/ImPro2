#include <opencv2/opencv.hpp>
#include "CamMoveCorrector.h"
#include "impro_util.h"
#include "pickAPoint.h"



int CamMoveCorrector::pickInitFixedPoint(int numFixedPoints)
{
	// check 
	if (numFixedPoints <= 0)
	{
		cout << "Enter number of fixed points: " << endl;
		numFixedPoints = readIntFromIstream(std::cin);
	}
	if (numFixedPoints <= 0)
	{
		cout << "Number of fixed point should be >= 0 if you want to assign.\n";
		this->fixedPoints.clear(); 
		return -1;
	}
	this->fixedPoints.resize(numFixedPoints);
	// Get points
	// get the first image to be the background for user to pick fixed reference points
	if (this->imgFixed.rows <= 0 || this->imgFixed.cols <= 0)
	{
		cerr << "You need to specified fixed image (imgFixed) before defining fixed points.\n";
		return -1;
	}
	for (int i = 0; i < numFixedPoints; i++)
	{
		cv::Point2f p;
        char buf[1000]; snprintf(buf, 1000, "Pick fixed point %d/%d (1-based)", i + 1, numFixedPoints);
		pickAPoint(buf, imgFixed, p);
		this->fixedPoints[i] = p;
	}
	return 0;
}

int CamMoveCorrector::correctImgMoved()
{
	cv::Mat imgOri, imgCamMoveCorrected;
	// Check if there are fixed point
	if (this->fixedPoints.size() > 0)
	{
		cv::Mat imgFixedSobel; 
		cv::Mat imgMovedSobel;
		// correct image: 
		// compare imgFixed and imgOri
		vector<cv::Point2f> fixedPointsMoved = this->fixedPoints; 
		vector<uchar> optStatus(this->fixedPoints.size());
		vector<float> optError(this->fixedPoints.size());
		imgFixedSobel = sobel_xy(imgFixed); 
		imgMovedSobel = sobel_xy(imgMoved);
		cout << imgFixedSobel.type() << endl;
		cout << imgMovedSobel.type() << endl;
		cout << imgFixedSobel.channels() << endl;
		cout << imgMovedSobel.channels() << endl;
		cv::calcOpticalFlowPyrLK(imgFixedSobel, imgMovedSobel,
			this->fixedPoints, // points which are supposed to be fixed
			fixedPointsMoved,
			optStatus,
			optError,
			cv::Size(61, 61),
			3 /* max level */,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50 /* ecc max count */, 0.001 /* eps */)) ; 
			
			
			// find homography
		cv::Mat hMat = cv::findHomography(fixedPointsMoved, this->fixedPoints, cv::noArray(), cv::RHO); 
		// simplify hMat
		//double c = 0.5 * (hMat.at<double>(0, 0) + hMat.at<double>(0, 0)); 
		//double s = 0.5 * (hMat.at<double>(1, 0) - hMat.at<double>(0, 1));
		//double theta = atan(s / c); 
		//hMat.at<double>(0, 0) = cos(theta); hMat.at<double>(1, 1) = cos(theta);
		//hMat.at<double>(1, 0) = sin(theta); hMat.at<double>(1, 0) = -sin(theta);
		//hMat.at<double>(2, 0) = 0.0; hMat.at<double>(2, 1) = 0.0; hMat.at<double>(2, 2) = 1.0;

//		cv::Mat hMat = cv::estimateRigidTransform(fixedPointsMoved, this->fixedPoints, false); 
		// warp image 
		cv::warpPerspective(this->imgMoved, this->imgMoved, hMat, imgOri.size());
		return true;
	}
	return 0; 
}




//  cv::VideoCapture c(0);
//  CamMoveCorrector cmc;
//  
//  cv::Mat imgCurr, imgCrrt;
//  int count = 0;
//  while (true)
//  {
//  	if (c.isOpened()) {
//  		c.grab();
//  		c.retrieve(imgCurr);
//  	}
//  
//  	// cam move correction
//  	if (cmc.imgFixed.rows > 0 && cmc.fixedPoints.size() > 0)
//  	{
//  		// correction
//  		cmc.imgMoved = imgCurr.clone();
//  		cmc.correctImgMoved();
//  		imgCrrt = cmc.imgMoved;
//  		cv::imshow("Ccorrected", imgCrrt);
//  	}
//  
//  	cv::imshow("Un-corrected", imgCurr);
//  	int ikey = cv::waitKey(30);
//  	if (ikey == 27) break;
//  	if (ikey == (int) 'i')
//  	{
//  		cmc.imgFixed = imgCurr.clone();
//  		cmc.pickInitFixedPoint(16);
//  	}
//  	count++;
//  }
//  c.release();
//  return 0;
