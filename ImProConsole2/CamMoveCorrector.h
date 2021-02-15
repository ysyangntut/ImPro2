#pragma once
#include <opencv2/opencv.hpp>

// CamMoveCorrector a;
// a.imgFixed = imgInit;  // assign initial (unmoved) image
// a.pickInitFixedPoint(6); //pick fixed points from ui, Or assign fixedPoints redirectly
// a.fixedPoints.resize(6); a.fixedPoints[0] = ... ...; 
// a.imgMoved = imgCurr.clone(); 
// a.correctImgMoved(); 
// imgCurr = a.imgMoved.clone(); 

class CamMoveCorrector
{
public:
	int pickInitFixedPoint(int numFixedPoints = 0);

	int correctImgMoved(); // This function updates imgMoved, making it unmoved (which movedPoints moves to fixedPoints)

	std::vector<cv::Point2f> fixedPoints;
	std::vector<cv::Point2f> movedPoints;

	cv::Mat imgFixed;
	cv::Mat imgMoved; 
};


