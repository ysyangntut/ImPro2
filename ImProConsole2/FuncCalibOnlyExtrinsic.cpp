#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include "impro_util.h"
#include "IntrinsicCalibrator.h"
#include "FileSeq.h"

#include "Points2fHistoryData.h"
#include "Points3dHistoryData.h"

int FuncCalibOnlyExtrinsic(int argc, char ** argv)
{
	IntrinsicCalibrator calC1;
	string fnameIntrinsic; 
	cout << "Input the intrinsic parameter file (xml format):\n";
	fnameIntrinsic = readStringFromIstream(std::cin); 
    cout << "The string is \n: " << fnameIntrinsic << endl;
//    cv::FileStorage ifs(fnameIntrinsic, cv::FileStorage::READ);
//    cout << "The file isopen check: " << ifs.isOpened() << endl;
//    cv::Mat x;
//    ifs["cameraMatrix"] >> x;
//    cout << x << endl;


    calC1.readFromFsFile(fnameIntrinsic);

	cout << "Cam cmat: \n" << calC1.cameraMatrix() << endl;
	cout << "Cam dvec: \n" << calC1.distortionVector() << endl;

	// Step 10: Left extrinsic calibration with user coordinate
	// 
	Points2fHistoryData pointsUserDefined;
	Points3dHistoryData points3dUserDefined;
	// image points of known points (file calfuimpx1) --> pointsUserDefined
	{
		cout << "How do you want to define the image points of "
			"known points for extrinsic parameters?\n";
		pointsUserDefined.readThruUserInteraction(1 /* nStep */, -1 /* nPoint unknown */);
	}
	// 3d points of known points (file calfu3dpx1) --> points3dUserDefined
	{
		cout << "How do you want to define the 3d points of "
			"known points for extrinsic parameters?\n";
		points3dUserDefined.readThruUserInteraction(1 /* nStep */, pointsUserDefined.nPoint());
	}
	//// extrinsic for left cam
	cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 1 for extrinsic calibration wrt user coord.
	cv::Mat tvec1 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 1 for extrinsic calibration wrt user coord.
	cv::Mat rvec133 = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat R44C1 = cv::Mat::eye(4, 4, CV_64F); // R matrix (4 by 4) of camera 1 (wrt user-defined coord.)
	cv::Mat R33C1(R44C1(cv::Rect(0, 0, 3, 3))); // R matrix (3 by 3) of stereo calibration of camera 1 (wrt user-defined coord.)
	cv::Mat T31C1(R44C1(cv::Rect(3, 0, 1, 3))); // T vector (3 by 1) of stereo calibration of camera 1 (wrt user-defined coord.)
	cv::solvePnP(points3dUserDefined.getVecVec()[0], pointsUserDefined.getVecVec()[0],
		calC1.cameraMatrix(), calC1.distortionVector(), rvec1, tvec1);
	// convert to rvec/tvec of the other cam
	cv::Rodrigues(rvec1, rvec133);
	rvec133.copyTo(R44C1(cv::Rect(0, 0, 3, 3)));
	tvec1.copyTo(R44C1(cv::Rect(3, 0, 1, 3)));

	cout << "R44: \n" << R44C1 << endl;
	cv::Mat R44_inv = R44C1.inv(); 
	cout << "Camera posotion:\n" << R44_inv(cv::Rect(3, 0, 1, 3)) << endl;
	cout << "Camera view direction:\n" << R44_inv(cv::Rect(2, 0, 1, 3)) << endl;

	// append to file 
	cv::FileStorage fsCalFx1(fnameIntrinsic, cv::FileStorage::APPEND);
	fsCalFx1 << "rvec" << rvec1;
	fsCalFx1 << "tvec" << R44C1(cv::Rect(3, 0, 1, 3));
	fsCalFx1 << "R4" << R44C1;
	fsCalFx1 << "R4inv" << R44C1.inv();
	fsCalFx1.release();
	// output to matlab script
	//int userDefinedPhotoId = calC1.numValidPhotos();
	//calC1.defineUserPoints(userDefinedPhotoId,
	//	pointsUserDefined.getVecVec()[0],
	//	points3dUserDefined.getVecVec()[0], calC1.imageSize());
	//calC1.solvePnp(userDefinedPhotoId);
	//calC1.writeToFsFile(calfx1);
	//calC1.writeToMscript(calfpm1);



	return 0; 
}
