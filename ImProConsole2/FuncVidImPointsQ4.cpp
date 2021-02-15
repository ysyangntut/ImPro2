#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <ctime>

#include "FileSeq.h"
#include "Points2fHistoryData.h"
#include "impro_util.h"
#include "pickAPoint.h"

using namespace std;

// The format of the vidTrack() output xml file is in the following format: 
// ---------------------------------------------------------------
//  < ? xml version = "1.0" ? >
//  <opencv_storage>
//  < videoWidth>0 < / videoWidth >
//  < videoHeight>0 < / videoHeight >
//  < nFrames>180 < / nFrames >
//  < fps>2.9969999999999999e+01 < / fps >
//  < nxPoint>192 < / nxPoint >
//  < nyPoint>108 < / nyPoint >
//  < winSize>
//  32 32 < / winSize >
//  <imgPoints0 type_id = "opencv-matrix">
//  < rows>1 < / rows >
//  < cols>20736 < / cols >
//  <dt>"2f"< / dt>
//  < data>
//  9.50000000e+00 9.50000000e+00 2.95000000e+01 9.50000000e+00
//  4.95000000e+01 9.50000000e+00 6.95000000e+01 9.50000000e+00
//  8.95000000e+01 9.50000000e+00 1.09500000e+02 9.50000000e+00
//  1.29500000e+02 9.50000000e+00 1.49500000e+02 9.50000000e+00
//  1.69500000e+02 9.50000000e+00 1.89500000e+02 9.50000000e+00
//  2.09500000e+02 9.50000000e+00 2.29500000e+02 9.50000000e+00
//  2.49500000e+02 9.50000000e+00 2.69500000e+02 9.50000000e+00
//  2.89500000e+02 9.50000000e+00 3.09500000e+02 9.50000000e+00
// (etc.) 
// -------------------------------------------------------------- -

int FuncVidImPointsQ4(int argc, char** argv)
{
	// Declare all variables
	std::string fnameDensePoints, fnameInitImg; 
	cv::FileStorage fsDensePoints;
	int videoWidth = -1, videoHeight = -1, nFrames = -1, nxPoint = -1, nyPoint = -1;
	float fps = -1;
	cv::Size winSize = cv::Size(0, 0); 
	cv::Mat imgPoints;
	char buf[1000]; 
	cv::Mat imgInit; 
	int nxQPoint, nyQPoint; 
	std::vector<cv::Point2f> q4Points(4); // four vertices
	std::vector<cv::Point2f> qPoints; // many interpolated points in the Q4.
	int nQ4; 
	std::vector<cv::Point2f> qPointsAllQ4; 

	// Read full path of xml output of vidTrack()
	printf("# Enter full file path of the xml output of vidTrack():\n");
	fnameDensePoints = readStringFromIstream(std::cin); 
	fsDensePoints.open(fnameDensePoints, cv::FileStorage::READ); 
	if (fsDensePoints.isOpened() == false) {
		std::cerr << "Error: Cannot open file: " << fnameDensePoints << std::endl;
		return -1;
	}

	// Print basic information of the xml file
	fsDensePoints["videoWidth"] >> videoWidth; 
	fsDensePoints["videoHeight"] >> videoHeight;
	fsDensePoints["nFrames"] >> nFrames;
	fsDensePoints["fps"] >> fps;
	fsDensePoints["nxPoint"] >> nxPoint;
	fsDensePoints["nyPoint"] >> nyPoint;

	printf("videoWidth: %d \n", videoWidth);
	printf("videoHeight: %d \n", videoHeight);
	printf("nFrames: %d \n", nFrames);
	printf("fps: %f \n", fps);
	printf("nxPoint: %d \n", nxPoint);
	printf("nyPoint: %d \n", nyPoint);

	// Pick four points of a quad.
	printf("# Enter full file path of the initial image: \n"); 
	fnameInitImg = readStringFromIstream(std::cin); 
	imgInit = cv::imread(fnameInitImg); 
	if (imgInit.rows <= 0 || imgInit.cols <= 0) {
		std::cerr << "Error: Cannot open image file: " << fnameInitImg << std::endl;
		return -1;
	}

	printf("# Enter number of Q4 you want to pick: \n"); 
	nQ4 = readIntFromIstream(std::cin); 
	for (int i = 0; i < nQ4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			snprintf(buf, 1000, "Pick points %d/4 of quad %d/%d.", j + 1, i + 1, nQ4);
			pickAPoint(std::string(buf), imgInit, q4Points[j]);
		}

		// Ask how many points are to interpolated in this q4. 
		printf("# Enter nxQPoint nyQPoint of the Q4: (nxQPoint is number of points to interpolate between picked P1 and P2, and nyQPoint is between P2 and P3.\n");
		nxQPoint = readIntFromIstream(std::cin);
		nyQPoint = readIntFromIstream(std::cin);

		// Interpolation (generating qPoints)
		qPoints.resize((size_t)nxQPoint * nyQPoint);
		qPoints = interpQ4(q4Points, nxQPoint, nyQPoint);

		// append to qPointsAllQ4
		qPointsAllQ4.insert(qPointsAllQ4.end(), qPoints.begin(), qPoints.end()); 	
	}

	cout << qPointsAllQ4 << endl;

	return 0; 
}
