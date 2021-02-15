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

int FuncVidOptflowToVelocity(int argc, char** argv)
{

	// Declare all variables
	FileSeq fsq, voFsq;
	std::string fnameDensePoints, fnameInitImg;
	std::string fnameIntrinsic, fnameExtrinsic; 
	cv::FileStorage fsDensePoints, fsIntrinsic, fsExtrinsic; 
	cv::Mat cmat, dvec, rvec, tvec; 
	int videoWidth = -1, videoHeight = -1, nFrames = -1, nxPoint = -1, nyPoint = -1;
	float fps = -1;
	cv::Size winSize = cv::Size(0, 0); 
	cv::Mat imgPointsPrev, imgPointsCurr;
	cv::Mat imgVelCurr; 
	cv::Mat realVelCurr; 
	char buf[1000]; 
	cv::Mat imgInit; 
	int nxQPoint, nyQPoint; 
	std::vector<cv::Point2f> q4Points(4); // four vertices
	std::vector<cv::Point2f> qPoints; // many interpolated points in the Q4.
	int nQ4; 
	std::vector<cv::Point2f> qPointsAllQ4; 
	cv::Point2f refImgPoint, refImgPointDown;
	cv::Point3d ref3dPoint, ref3dPointDown; 
	cv::Vec2f downImgVec;

	// Read image file sequence fseq
	printf("# Enter full file path of all images:\n");
	fsq.setDirFilesByConsole(std::cin);

	// Enter intrinsic file 
	printf("# Enter full path of camera intrinsic file (xml): \n");
	fnameIntrinsic = readStringFromCin();
	fsIntrinsic.open(fnameIntrinsic, cv::FileStorage::READ); 
	if (fsIntrinsic.isOpened() == false) {
		std::cerr << "Error: Cannot read camera intrinsic file " << fnameIntrinsic << "\n";
		return -1; 
	}
	fsIntrinsic["cameraMatrix"] >> cmat; 
	if (cmat.rows != 3 || cmat.cols != 3) {
		std::cerr << "Error: Cannot get camera matrix from file " << fnameIntrinsic << "\n"; 
		return -1; 
	}
	fsIntrinsic["distortionVector"] >> dvec;
	if (dvec.rows != 1 && dvec.cols != 1) {
		std::cerr << "Error: Cannot get distortion vector file " << fnameIntrinsic << "\n";
		return -1;
	}

	// Enter extrinsic file 
	printf("# Enter full path of camera extrinsic file (xml): \n");
	fnameExtrinsic = readStringFromCin();
	fsExtrinsic.open(fnameExtrinsic, cv::FileStorage::READ);
	if (fsExtrinsic.isOpened() == false) {
		std::cerr << "Error: Cannot read camera extrinsic file " << fnameExtrinsic << "\n";
		return -1;
	}
	fsExtrinsic["rvec"] >> rvec;
	if (rvec.rows * rvec.cols != 3) {
		std::cerr << "Error: Cannot get rvec from file " << fnameExtrinsic << "\n";
		return -1;
	}
	fsExtrinsic["tvec"] >> tvec;
	if (rvec.rows * rvec.cols != 3) {
		std::cerr << "Error: Cannot get tvec vector file " << fnameExtrinsic << "\n";
		return -1;
	}

	// Enter image coord. and world coord. of a refernece point
	printf("# Enter world coord. of a reference point which image-world relationship represents everything in the image (xw yw zw): \n");
	ref3dPoint.x = readDoubleFromIstream(std::cin);
	ref3dPoint.y = readDoubleFromIstream(std::cin);
	ref3dPoint.z = readDoubleFromIstream(std::cin);

	// Calculate downward image vector
	cv::Point3d downVec(0., 0., -1.); 
	ref3dPointDown = ref3dPoint + downVec;
	vector<cv::Point3f> objPointsVec(2);
	vector<cv::Point2f> imgPointsVec(2);
	objPointsVec[0] = cv::Point3f((float) ref3dPoint.x, (float) ref3dPoint.y, (float) ref3dPoint.z);
	objPointsVec[1] = cv::Point3f((float) ref3dPointDown.x, (float) ref3dPointDown.y, (float) ref3dPointDown.z);
	cv::projectPoints(objPointsVec, rvec, tvec, cmat, dvec, imgPointsVec);
	refImgPoint = imgPointsVec[0];
	refImgPointDown = imgPointsVec[1];
	downImgVec[0] = refImgPointDown.x - refImgPoint.x;
	downImgVec[1] = refImgPointDown.y - refImgPoint.y;
	downImgVec /= sqrt(downVec.x * downVec.x + downVec.y * downVec.y + downVec.z * downVec.z);
	std::cout << "Downward img vec: " << downImgVec << "\n";





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
