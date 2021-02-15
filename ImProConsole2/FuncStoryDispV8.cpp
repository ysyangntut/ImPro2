#ifdef __unix__                    /* __unix__ is usually defined by compilers targeting Unix systems */
#include "pch.h"
#elif defined(_WIN32) || defined(WIN32)     /* _Win32 is usually defined by compilers targeting 32 or 64 bit Windows systems */
#define _CRT_SECURE_NO_WARNINGS     // Make Microsoft Visual Studio quiet on localtime() and many other functions
#else
#include "pch.h"
#endif

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <ctime>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "improDraw.h"
#include "trackings.h"
#include "RollingPlot.h"

using namespace std;
using namespace cv;

cv::Point2f selectROI_zoom(cv::Mat img, int nLevel);

int estimateStoryDispV4(
	cv::Mat cmat,			         // camera matrix
	cv::Mat dvec,			         // distortion vector
	cv::Mat rvec,			         // rotational vector (3, 1, CV_64F)
	cv::Mat tvec,			         // translational vector (3, 1, CV_64F)
	cv::InputArray refImgPoints,     // reference points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray trkImgPoints,     // tracking points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray refObjPoints,     // reference points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::InputArray trkObjPoints,     // tracking points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::InputArray newRefImgPoints,  // moved reference points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray newTrkImgPoints,  // moved tracking points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::Point3d    tortionCenter,    // tortional center point in world coord.
	cv::Mat& camRot,                // camera rotational movement (3, 1, CV_64F)
	cv::Mat& newDisp,               // displacement of story (ux, uy, tortion) (3, 1, CV_64F)
	cv::Mat& newTrkObjPoints,       // moved tracking points in world coord. vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::Mat& projNewRefImgPoints,   // new projection reference points in image coord.
	cv::Mat& projNewTrkImgPoints    // new projection tracking points in image coord.
);
#pragma once 

const float r2d = 180.f / 3.1415926536f;

int FuncStoryDispV8(int argc, char** argv)
{
	int num_fixed_points, num_track_points;
	vector<cv::Point2f> fixedPoints2f, trackPoints2f;
	vector<cv::Point3d> fixedPoints3d, trackPoints3d;
	cv::Point3d tortionalCenter;
	string fnameImgInit("");
	cv::Mat imgInit;
	int trackMethod; // tracking method. 1:Template match (with rotation and pyramid), 2:Ecc, 3:Optical flow (sparse, LK Pyr.)
	int trackUpdateFreq; // tracking template updating frequency. 0:Not updating (using initial template). 1:Update every frame. 2:Update every other frame. Etc.
	int trackPredictionMethod; // tracking prediction method. 0:Previous point. 1:Linear approx. 2:2nd-order approx.
	int winSize; // window size of template size
	int iiStep = 0; // the Step to output to the big table

	string fnameBigTable;
	string fDirImgSource;
	std::vector<RollingPlot> plots{
		RollingPlot(400, 600, 100, -100, "Ux", 60,
			vector<float>{-100, -10, -1, -.1f, 0, .1f, 1, 10, 100},
			vector<float>{-80, -60, -40, -20, -8, -6, -4, -2, -.8f, -.6f, -.4f, -.2f,
				-.08f, -.06f, -.04f, -.02f, .02f, .04f, .06f, .08f,
				.1f, .2f, .4f, .6f, .8f, 2, 4, 6, 8, 20, 40, 60, 80},
			vector<string>{"-100 mm", "-10 mm", "-1 mm", "-0.1 mm", "0", "0.1 mm", "1 mm", "10 mm", "100 mm"},
			RollingPlot_Scale_SymLog, .01f, -.01f),
		RollingPlot(400, 600, 100, -100, "Uy", 60,
			vector<float>{-100, -10, -1, -.1f, 0, .1f, 1, 10, 100},
			vector<float>{-80, -60, -40, -20, -8, -6, -4, -2, -.8f, -.6f, -.4f, -.2f,
				-.08f, -.06f, -.04f, -.02f, .02f, .04f, .06f, .08f,
				.1f, .2f, .4f, .6f, .8f, 2, 4, 6, 8, 20, 40, 60, 80},
			vector<string>{"-100 mm", "-10 mm", "-1 mm", "-0.1 mm", "0", "0.1 mm", "1 mm", "10 mm", "100 mm"},
			RollingPlot_Scale_SymLog, .01f, -.01f),
		RollingPlot(400, 600, 10.f, -10.f, "Torsion", 60,
			vector<float>{-10, -1, -.1f, 0, .1f, 1, 10},
			vector<float>{-8, -6, -4, -2, -.8f, -.6f, -.4f, -.2f, -.08f, -.06f, -.04f, -.02f,
				.02f, .04f, .06f, .08f, .2f, .4f, .6f, .8f, 2, 4, 6, 8},
			vector<string>{"-10 deg", "-1 deg", "-0.1 deg", "0", "0.1 deg", "1 deg", "10 deg"},
			RollingPlot_Scale_SymLog, .01f, -.01f),
		RollingPlot(300, 600, 10.f / r2d, -10.f / r2d, "CamRot X", 60,
			vector<float>{-10 / r2d, -1 / r2d, -.1f / r2d, 0, .1f / r2d, 1 / r2d, 10 / r2d},
			vector<float>{-8 / r2d, -6 / r2d, -4 / r2d, -2 / r2d,
				-.8f / r2d, -.6f / r2d, -.4f / r2d, -.2f / r2d,
				-.08f / r2d, -.06f / r2d, -.04f / r2d, -.02f / r2d,
				.02f / r2d, .04f / r2d, .06f / r2d, .08f / r2d,
				.2f / r2d, .4f / r2d, .6f / r2d, .8f / r2d, 2 / r2d, 4 / r2d, 6 / r2d, 8 / r2d},
			vector<string>{"-10 deg", "-1 deg", "-0.1 deg", "0", "0.1 deg", "1 deg", "10 deg"},
			RollingPlot_Scale_SymLog, .01f / r2d, -.01f / r2d),
		RollingPlot(300, 600, 10.f / r2d, -10.f / r2d, "CamRot Y", 60,
			vector<float>{-10 / r2d, -1 / r2d, -.1f / r2d, 0, .1f / r2d, 1 / r2d, 10 / r2d},
			vector<float>{-8 / r2d, -6 / r2d, -4 / r2d, -2 / r2d,
				-.8f / r2d, -.6f / r2d, -.4f / r2d, -.2f / r2d,
				-.08f / r2d, -.06f / r2d, -.04f / r2d, -.02f / r2d,
				.02f / r2d, .04f / r2d, .06f / r2d, .08f / r2d,
				.2f / r2d, .4f / r2d, .6f / r2d, .8f / r2d, 2 / r2d, 4 / r2d, 6 / r2d, 8 / r2d},
			vector<string>{"-10 deg", "-1 deg", "-0.1 deg", "0", "0.1 deg", "1 deg", "10 deg"},
			RollingPlot_Scale_SymLog, .01f / r2d, -.01f / r2d),
		RollingPlot(300, 600, 10.f / r2d, -10.f / r2d, "CamRot Z", 60,
			vector<float>{-10 / r2d, -1 / r2d, -.1f / r2d, 0, .1f / r2d, 1 / r2d, 10 / r2d},
			vector<float>{-8 / r2d, -6 / r2d, -4 / r2d, -2 / r2d,
				-.8f / r2d, -.6f / r2d, -.4f / r2d, -.2f / r2d,
				-.08f / r2d, -.06f / r2d, -.04f / r2d, -.02f / r2d,
				.02f / r2d, .04f / r2d, .06f / r2d, .08f / r2d,
				.2f / r2d, .4f / r2d, .6f / r2d, .8f / r2d, 2 / r2d, 4 / r2d, 6 / r2d, 8 / r2d},
			vector<string>{"-10 deg", "-1 deg", "-0.1 deg", "0", "0.1 deg", "1 deg", "10 deg"},
			RollingPlot_Scale_SymLog, .01f / r2d, -.01f / r2d),
	};

	// Step 1: Get video source:
	cv::VideoCapture vid;
	std::string vidName;

	// Read full directory of image source 
	std::cout << "# Enter full directory of image source:\n";
	fDirImgSource = readStringFromIstream(std::cin);
	fDirImgSource = appendSlashOrBackslashAfterDirectoryIfNecessary(fDirImgSource);

	// Ready to output big table file
	FILE* fBigTable;
	int ferr;
	std::cout << "# Enter full path of the big table file:\n";
	fnameBigTable = readStringFromIstream(std::cin);
	ferr = fopen_s(&fBigTable, fnameBigTable.c_str(), "w");
	std::printf("# Big table file is at: %s\n", fnameBigTable.c_str()); std::cout.flush();

	//eric get intial image

	std::cout << "# Input full path of cam initial photo (or VideoCapture0 for camera 0, and so on):\n";
	fnameImgInit = readStringLineFromIstream(std::cin);
	imgInit = cv::imread(fnameImgInit, cv::IMREAD_GRAYSCALE);
	imshow_resize("Initial image", imgInit, (double)1024. / imgInit.cols);
	cv::waitKey(1000);
	cv::destroyWindow("Initial image");

	// Step 1: Get camera parameters

	cv::Mat cmat, dvec, rvec, tvec, r3, r4, r4inv, campos, camdir;
	std::cout << "# Enter camera intrinsic file:\n";
	std::string fnameIntrinsic = readStringLineFromCin();
	cv::FileStorage ifs(fnameIntrinsic, cv::FileStorage::READ);
	ifs["cameraMatrix"] >> cmat;
	ifs["distortionVector"] >> dvec;
	std::cout << "# Cmat:\n" << cmat << endl << "# Dvec:\n" << dvec << endl;
	ifs.release();
	std::cout << "# Enter camera extrinsic file:\n";
	string fnameExtrinsic = readStringLineFromCin();
	ifs.open(fnameExtrinsic, cv::FileStorage::READ);
	//	ifs["rvec"] >> rvec;
	//	ifs["tvec"] >> tvec;
	ifs["R4"] >> r4;
	if (r4.rows <= 0 || r4.cols <= 0)
		ifs["r4"] >> r4;

	std::cout << "r4: \n" << r4 << endl;

	r4(cv::Rect(0, 0, 3, 3)).copyTo(r3);
	r4(cv::Rect(3, 0, 1, 3)).copyTo(tvec);
	cv::Rodrigues(r3, rvec);
	r4inv = r4.inv();
	r4inv(cv::Rect(3, 0, 1, 3)).copyTo(campos);
	r4inv(cv::Rect(2, 0, 1, 3)).copyTo(camdir);
	std::cout << "rvec:\n" << rvec << "\ntvec:\n" << tvec << endl;
	std::cout << "camPos:\n" << campos << "\ncamdir:\n" << camdir << endl;

	// output to big table
	if (fBigTable) fprintf(fBigTable, "Fx: , %25.17e\n", cmat.at<double>(0, 0));
	if (fBigTable) fprintf(fBigTable, "Fy: , %25.17e\n", cmat.at<double>(1, 1));
	if (fBigTable) fprintf(fBigTable, "Cx: , %25.17e\n", cmat.at<double>(0, 2));
	if (fBigTable) fprintf(fBigTable, "Cy: , %25.17e\n", cmat.at<double>(1, 2));
	if (fBigTable) fprintf(fBigTable, "K1: , %25.17e\n", dvec.at<double>(0, 0));
	if (fBigTable) if (dvec.cols > 1) fprintf(fBigTable, "K2: , %25.17e\n", dvec.at<double>(0, 1));
	if (fBigTable) if (dvec.cols > 2) fprintf(fBigTable, "P1: , %25.17e\n", dvec.at<double>(0, 2));
	if (fBigTable) if (dvec.cols > 3) fprintf(fBigTable, "P2: , %25.17e\n", dvec.at<double>(0, 3));
	if (fBigTable) if (dvec.cols > 4) fprintf(fBigTable, "K3: , %25.17e\n", dvec.at<double>(0, 4));
	if (fBigTable) if (dvec.cols > 5) fprintf(fBigTable, "K4: , %25.17e\n", dvec.at<double>(0, 5));
	if (fBigTable) if (dvec.cols > 6) fprintf(fBigTable, "K5: , %25.17e\n", dvec.at<double>(0, 6));
	if (fBigTable) if (dvec.cols > 7) fprintf(fBigTable, "K6: , %25.17e\n", dvec.at<double>(0, 7));
	if (fBigTable) fprintf(fBigTable, "rx: , %25.17e\n", ((double*)rvec.data)[0]);
	if (fBigTable) fprintf(fBigTable, "ry: , %25.17e\n", ((double*)rvec.data)[1]);
	if (fBigTable) fprintf(fBigTable, "rz: , %25.17e\n", ((double*)rvec.data)[2]);
	if (fBigTable) fprintf(fBigTable, "tx: , %25.17e\n", ((double*)tvec.data)[0]);
	if (fBigTable) fprintf(fBigTable, "ty: , %25.17e\n", ((double*)tvec.data)[1]);
	if (fBigTable) fprintf(fBigTable, "tz: , %25.17e\n", ((double*)tvec.data)[2]);

	// Step 2: Ask user to define reference points (supposed to be fixed) both 2D (image) and 3D (world).
	std::cout << "# Enter number of reference points (fixed points): \n";
	num_fixed_points = readIntFromIstream(std::cin);
	fixedPoints2f.resize(num_fixed_points);
	fixedPoints3d.resize(num_fixed_points);
	for (int i = 0; i < num_fixed_points; i++)
	{
		std::cout << "# Enter 0 for manual picking. Enter 1 for manual input. \n";
		int inputMode = readIntFromIstream(std::cin);
		if (inputMode == 0) {
			while (imgInit.rows <= 0 || imgInit.cols <= 0)
			{
				std::cout << "# Initial image is empty. \n";
				std::cout << "# Enter initial image full file name: \n";
				fnameImgInit = readStringLineFromCin();
				imgInit = cv::imread(fnameImgInit);
			}
			fixedPoints2f[i] = selectROI_zoom(imgInit, 3);
			std::cout << "# You picked " << fixedPoints2f[i].x << " " << fixedPoints2f[i].y << endl;
			std::cout << "# Define reference points " << i << " (in world 3d. For example:  xw yw zw):\n";
		}
		else {
			std::cout << "# Define reference points " << i << " (in image 2d and world 3d. For example:  xi yi xw yw zw):\n";
			fixedPoints2f[i].x = (float)readDoubleFromIstream(std::cin);
			fixedPoints2f[i].y = (float)readDoubleFromIstream(std::cin);
		}
		fixedPoints3d[i].x = readDoubleFromIstream(std::cin);
		fixedPoints3d[i].y = readDoubleFromIstream(std::cin);
		fixedPoints3d[i].z = readDoubleFromIstream(std::cin);
	}

	// output to big table
	if (fBigTable) fprintf(fBigTable, "numRefPoints: , %4d\n", num_fixed_points);
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "refPoints2f[ %4d ]: , %16.9e , %16.9e , "
			"refPoints3d[ %4d ]: , %25.17e , %25.17e , %25.17e\n",
			i, fixedPoints2f[i].x, fixedPoints2f[i].y,
			i, fixedPoints3d[i].x, fixedPoints3d[i].y, fixedPoints3d[i].z);
	}

	// Step 3: Ask user to define tracking points (supposed to move within constrained surface) both 2D (image) and 3D (world).
	std::cout << "# Enter number of tracking points (movable points): \n";
	num_track_points = readIntFromIstream(std::cin);
	trackPoints2f.resize(num_track_points);
	trackPoints3d.resize(num_track_points);
	for (int i = 0; i < num_track_points; i++)
	{
		std::cout << "# Enter 0 for manual picking. Enter 1 for manual input. \n";
		int inputMode = readIntFromIstream(std::cin);
		if (inputMode == 0) {
			while (imgInit.rows <= 0 || imgInit.cols <= 0)
			{
				std::cout << "# Initial image is empty. \n";
				std::cout << "# Enter initial image full file name: \n";
				fnameImgInit = readStringLineFromCin();
				imgInit = cv::imread(fnameImgInit);
			}
			trackPoints2f[i] = selectROI_zoom(imgInit, 3);
			std::cout << "# You picked " << trackPoints2f[i].x << " " << trackPoints2f[i].y << endl;
			std::cout << "# Define tracking points " << i << " (in world 3d. For example:  xw yw zw):\n";
		}
		else {
			std::cout << "# Define tracking points " << i << " (in image 2d and world 3d. For example:  xi yi xw yw zw):\n";
			trackPoints2f[i].x = (float)readDoubleFromIstream(std::cin);
			trackPoints2f[i].y = (float)readDoubleFromIstream(std::cin);
		}
		trackPoints3d[i].x = readDoubleFromIstream(std::cin);
		trackPoints3d[i].y = readDoubleFromIstream(std::cin);
		trackPoints3d[i].z = readDoubleFromIstream(std::cin);
	}

	// Determine which is higher (reference points are higher or tracking points are higher)
	double zFixed = 0., zTrack = 0.;
	for (int i = 0; i < num_fixed_points; i++)
	{
		zFixed += fixedPoints3d[i].z;
	}
	zFixed /= num_fixed_points;

	for (int i = 0; i < num_track_points; i++)
	{
		zTrack += trackPoints3d[i].z;
	}
	zTrack /= num_track_points;

	// output to big table
	if (fBigTable) fprintf(fBigTable, "numTrkPoints: , %4d\n", num_fixed_points);
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "trackPoints2f[ %4d ]: , %16.9e , %16.9e , "
			"trackPoints3d[ %4d ]: , %25.17e , %25.17e , %25.17e\n",
			i, trackPoints2f[i].x, trackPoints2f[i].y,
			i, trackPoints3d[i].x, trackPoints3d[i].y, trackPoints3d[i].z);
	}

	std::cout << "# Define tortional center (xc yc zc): \n";
	tortionalCenter.x = readDoubleFromIstream(std::cin);
	tortionalCenter.y = readDoubleFromIstream(std::cin);
	tortionalCenter.z = readDoubleFromIstream(std::cin);

	// output to big table
	if (fBigTable) fprintf(fBigTable, "tortionalCenter: , %25.17e , %25.17e , %25.17e\n",
		tortionalCenter.x, tortionalCenter.y, tortionalCenter.z);

	cv::Mat matFixedPoints2f(fixedPoints2f);
	cv::Mat matFixedPoints3d(fixedPoints3d);
	cv::Mat matTrackPoints2f(trackPoints2f);
	cv::Mat matTrackPoints3d(trackPoints3d);

	std::cout << "Fixed points 2f: \n" << fixedPoints2f << endl;
	std::cout << "Fixed points 3d: \n" << fixedPoints3d << endl;
	std::cout << "Tracking points 2f: \n" << trackPoints2f << endl;
	std::cout << "Tracking points 3d: \n" << trackPoints3d << endl;

	// Step 4: Select tracking method and options
	//   4.1: Tracking method: default, template match
	//   4.2: Target updating frequency (default 1, update each frame, 0 for not-updated and using initial template)
	//   4.3: Prediction method (default: 2-nd order)
	//
	std::cout << "# Select tracking method: (3)Optical flow: (Methods 1 and 2 are disabled) \n";
	trackMethod = readIntFromIstream(std::cin, 3, 3);
	std::cout << "# Your selection is " << trackMethod << endl;
	std::cout << "# Target updating frequency: (0)Not updating (N)Updating every N frames: \n";
	trackUpdateFreq = readIntFromIstream(std::cin, 0, 99999999);
	std::cout << "# Your selection is " << trackUpdateFreq << endl;
	std::cout << "# Prediction method: (0)Previous point, (1)Linear approx, (2)2nd-order approx.:\n";
	trackPredictionMethod = readIntFromIstream(std::cin, 0, 2);
	std::cout << "# Your selection is " << trackPredictionMethod << endl;
	int defaultWinSize = std::min(imgInit.cols, imgInit.rows) / 50;
	std::cout << "# Window size (or template size) (0 for 1/50 of min(image width, height), " << defaultWinSize << "):\n";
	winSize = readIntFromIstream(std::cin, 0, std::min(imgInit.cols, imgInit.rows) - 1);
	if (winSize <= 0) winSize = defaultWinSize;
	std::cout << "Window size is " << winSize << endl;

	// output to big table
	if (fBigTable) fprintf(fBigTable, "trackMethod: , %d\n", trackMethod);
	if (fBigTable) fprintf(fBigTable, "trackUpdateFreq: , %d\n", trackUpdateFreq);
	if (fBigTable) fprintf(fBigTable, "trackPredictionMethod: , %d\n", trackPredictionMethod);
	if (fBigTable) fprintf(fBigTable, "winSize: , %d\n", winSize);

	// Step 5: start the tracking loop
	cv::Mat imgCurr, imgTmpl;

	vector<cv::Point2f> fixedPoints2f_Curr = fixedPoints2f;
	vector<cv::Point2f> fixedPoints2f_Prev = fixedPoints2f;
	vector<cv::Point2f> fixedPoints2f_Pre2 = fixedPoints2f;
	vector<cv::Point2f> fixedPoints2f_Pre3 = fixedPoints2f;
	vector<cv::Point2f> fixedPoints2f_Tmpl = fixedPoints2f;

	vector<cv::Point2f> trackPoints2f_Curr = trackPoints2f;
	vector<cv::Point2f> trackPoints2f_Prev = trackPoints2f;
	vector<cv::Point2f> trackPoints2f_Pre2 = trackPoints2f;
	vector<cv::Point2f> trackPoints2f_Pre3 = trackPoints2f;
	vector<cv::Point2f> trackPoints2f_Tmpl = trackPoints2f;

	// output to big table (header line)
	if (fBigTable) fprintf(fBigTable, " Step , "); // 8 chars (including ending space)
	if (fBigTable) fprintf(fBigTable, "           Time , "); // YYYYMMDDhhmmsssss (18 chars including ending space)
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "   RefPnt2f[%d].x, ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "   RefPnt2f[%d].y, ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "   TrkPnt2f[%d].x, ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "   TrkPnt2f[%d].y, ", i); // 17 chars
	}
	if (fBigTable) fprintf(fBigTable, "             FloorDisp.ux, "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "             FloorDisp.uy, "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "        FloorDisp.tortion, "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.rx, "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.ry, "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.rz, "); // 26 chars

	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "            CalTrk3d[%d].x, ", i); // 26 chars
		if (fBigTable) fprintf(fBigTable, "            CalTrk3d[%d].y, ", i); // 26 chars
		if (fBigTable) fprintf(fBigTable, "            CalTrk3d[%d].z, ", i); // 26 chars
	}
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "   PrjRef2f[%d].x, ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "   PrjRef2f[%d].y, ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "   PrjTrk2f[%d].x, ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "   PrjTrk2f[%d].y, ", i); // 17 chars
	}
	if (fBigTable) fprintf(fBigTable, "\n");

	//----eric photo frequence

	struct tm user_input_time;
	struct tm user_next_input_time;
	struct tm user_input_finish_time;//user input end time
	struct tm user_next_finish_time;//=used user input end time           
	char buf[1000];
	std::memset(&user_input_time, 0, sizeof(tm));
	int fre_type = 0;
	int track_frequence = 0;
	//-------------------------------------------------------------------------------------
	int breakpoint = 0;

	std::cout << "# Enter the analysis start time:" << endl;
	std::cout << "# year = \n";
	user_input_time.tm_year = readIntFromCin();

	std::cout << "# month = \n";
	user_input_time.tm_mon = readIntFromCin();

	std::cout << "# day = \n";
	user_input_time.tm_mday = readIntFromCin();

	std::cout << "# hour =\n";
	user_input_time.tm_hour = readIntFromCin();
	std::cout << "# min =\n";
	user_input_time.tm_min = readIntFromCin();
	std::cout << "# sec = \n";
	user_input_time.tm_sec = readIntFromCin();

	std::cout << "# The analysis stat time is: " << user_input_time.tm_year << "/" << user_input_time.tm_mon << "/" <<
		user_input_time.tm_mday << " " << user_input_time.tm_hour << ":" << user_input_time.tm_min << ":" << user_input_time.tm_sec << endl;
	//-----------------------------------------------------------------------------------
	//finish time
	std::cout << "# Enter the analysis finish time" << endl;
	std::cout << "# year =\n";
	user_input_finish_time.tm_year = readIntFromCin();

	std::cout << "month =\n";
	user_input_finish_time.tm_mon = readIntFromCin();

	std::cout << "day =\n";
	user_input_finish_time.tm_mday = readIntFromCin();

	std::cout << "hour =";
	user_input_finish_time.tm_hour = readIntFromCin();
	std::cout << "min =\n";
	user_input_finish_time.tm_min = readIntFromCin();
	std::cout << "sec = \n";
	user_input_finish_time.tm_sec = readIntFromCin();

	std::cout << "# The analysis finish time: " << user_input_finish_time.tm_year << "/" << user_input_finish_time.tm_mon << "/" <<
		user_input_finish_time.tm_mday << " " << user_input_finish_time.tm_hour << ":" << user_input_finish_time.tm_min << ":" << user_input_finish_time.tm_sec << endl;

	//-------------------------
	std::cout << "#  Tracking frequence type: (1) each image frames, (2) per sec, (3) per min, (4) per hour (5) per day (6) per month (7) per year:\n";
	fre_type = readIntFromIstream(std::cin, 1, 7);

	if (fre_type == 1) {
		track_frequence = 1;
		std::cout << "# You set tracking frequency to: Each image frame.\n";
	}

	if (fre_type == 2) {
		std::cout << "# How many seconds of images do you want to analyze between each analysis step?" << endl;
		track_frequence = readIntFromCin(1, 59);
	}
	if (fre_type == 3) {
		std::cout << "# How many minutes of images do you want to analyze between each analysis step?" << endl;
		track_frequence = readIntFromCin(1, 59);
		//each_fram_sec += track_frequence * 60;
	}

	if (fre_type == 4) {
		std::cout << "How many hours of images do you want to analyze between each analysis step?" << endl;
		track_frequence = readIntFromCin(1, 23);
		//each_fram_sec += track_frequence * 3600;
	}
	if (fre_type == 5) {
		std::cout << "How many days of images do you want to analyze between each analysis step?" << endl;
		track_frequence = readIntFromCin(1, 31);
		//each_fram_sec += track_frequence * 86400;
	}

	if (fre_type == 6) {
		std::cout << "How many month of images do you want to analyze bewteen between each analysis step?" << endl;
		track_frequence = readIntFromCin(1, 11);
		//each_fram_sec += track_frequence * 2629743;//(30.44 คั)
	}
	if (fre_type == 7) {
		std::cout << "How many year of images do you want to analyze between each analysis step?" << endl;
		track_frequence = readIntFromCin(1, 99);
		//each_fram_sec += track_frequence * 31556736;;//(365.24 คั)
	}
	//--------------------------------------------------------------------------
	user_next_input_time = user_input_time;
	user_next_finish_time = user_input_finish_time;

	user_next_input_time.tm_year -= 1900;
	user_next_input_time.tm_mon--;
	user_next_finish_time.tm_year -= 1900;
	user_next_finish_time.tm_mon--;

	int stepnumber = 0;
	for (size_t iStep = 0; true; iStep++)
	{
		int br = 0;

		time_t user_next_input_time_t = mktime(&user_next_input_time);//timestamp(xxxxxxxx
		time_t user_next_finish_time_t = mktime(&user_next_finish_time);//timestamp(xxxxxxxx

		time_t t = user_next_input_time_t;
		tm* ltm = localtime(&t);
		//std::cout << ltm << endl;

		if (user_next_input_time_t > user_next_finish_time_t) {
			std::cout << "# Analysis Finished. " << endl;
			std::cout << "# The last file name is " << buf << endl;
			break;
		}

		for (int i = 0; true; i++) // i is the frame number, which normally is between 0 and 29 if FPS is 30.
		{

			if (br == 1)break;

			if (fre_type == 1) {
				//	snprintf(buf, 1000, "H:\\pic\\344_10\\%04d%02d%02d%02d%02d%02d_%03d.jpg",
				std::snprintf(buf, 1000, "%s%04d%02d%02d%02d%02d%02d_%03d.jpg", fDirImgSource.c_str(),
					1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, i);
			}

			else if (fre_type != 1) {
				//				snprintf(buf, 1000, "H:\\pic\\344_10\\%04d%02d%02d%02d%02d%02d_%03d.jpg",
				std::snprintf(buf, 1000, "%s%04d%02d%02d%02d%02d%02d_%03d.jpg", fDirImgSource.c_str(),
					1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, 0);
				if (fre_type == 2)user_next_input_time.tm_sec += track_frequence;
				if (fre_type == 3)user_next_input_time.tm_min += track_frequence;
				if (fre_type == 4)user_next_input_time.tm_hour += track_frequence;
				if (fre_type == 5)user_next_input_time.tm_mday += track_frequence;
				if (fre_type == 6)user_next_input_time.tm_mon += track_frequence;
				if (fre_type == 7)user_next_input_time.tm_year += track_frequence;

				br = 1;//br=breakpoint
			}

			while (true) {
				time_t now = time(0);
				if (now > t) break;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			std::cout << buf << endl;
			if (false)
			{   // no idea what these code is for. It seems redundent. 
				cv::Mat img = cv::imread(buf, cv::IMREAD_GRAYSCALE);
				time_t now = time(0);

				while (true) {
					time_t now = time(0);
					if (now > t) break;
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}

				if (!img.data && fre_type == 1 && now > t) {
					std::cout << "# Could not open or find the image" << endl;
					user_next_input_time.tm_sec += 1;
					break;
				}
				if (!img.data && fre_type == 2 && now > t) {
					std::cout << "# Could not open or find the image" << endl;
					user_next_input_time.tm_sec += track_frequence;
					break;
				}
				if (!img.data && fre_type == 3 && now > t) {
					std::cout << "# Could not open or find the image" << endl;
					user_next_input_time.tm_min += track_frequence;
					break;
				}
				if (!img.data && fre_type == 4 && now > t) {
					std::cout << "# Could not open or find the image" << endl;
					user_next_input_time.tm_hour += track_frequence;
				}
				if (!img.data && fre_type == 5 && now > t) {
					std::cout << "# Could not open or find the image" << endl;
					user_next_input_time.tm_mday += track_frequence;
					break;
				}
				if (!img.data && fre_type == 6 && now > t) {
					std::cout << "Could not open or find the image" << endl;
					user_next_input_time.tm_mon += track_frequence;
					break;
				}
				if (!img.data && fre_type == 7 && now > t) {
					std::cout << "Could not open or find the image" << endl;
					user_next_input_time.tm_year += track_frequence;
					break;
				}
			}

			//			cv::Mat imgCurr = cv::imread(buf, cv::IMREAD_GRAYSCALE);

						// Try to read image from file
			cv::Mat imgCurr;
			if (fre_type == 1) {
				// if frequency type is 1 (analyzing every frame), it must be offline, 
				// and we assume pictures have been existed, so no waiting is needed. 
				// so, if it fails to read image, we say there is no more image in this second, 
				// for example, 20200430235959_029 (the 29th frame in this second), 
				// and there is no _030 (no 30th frame), 
				// and we will try next second. 
				imgCurr = cv::imread(buf, cv::IMREAD_GRAYSCALE);
				if (imgCurr.rows <= 0 || imgCurr.cols <= 0) {
					user_next_input_time.tm_sec += 1;
					break;
				}
			}
			else {
				// if frequency type is not 1, for example, each frame per second or each 
				// frame per minute, this analysis could be online, and the image could  
				// appear later, and we should be patient and wait for a short time, 
				// and try enough times before giving up. 
				for (int iTryReadingImage = 0; iTryReadingImage < 3600; iTryReadingImage++)
				{
					imgCurr = cv::imread(buf, cv::IMREAD_GRAYSCALE);
					if (imgCurr.rows > 0 && imgCurr.cols > 0)
						break;
					else
					{
						// if it fails to get image, we wait for a while
						std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					}
				}  // end of many trials
				cerr << "# This program has been waiting for the file: " << buf
					<< " for too long (maybe an hour). Check the file to see if there "
					<< " is anything wrong.\n";
			} // end of check if frequency type is 1 or not. 

			if (imgCurr.rows <= 0 || imgCurr.cols <= 0)
			{
				std::cout << "Failed to load this picture: " << buf << endl;
				//system("pause");
				break;
			}

			if (imgCurr.channels() > 1)
				cv::cvtColor(imgCurr, imgCurr, cv::COLOR_BGR2GRAY);
			imgCurr = sobel_xy(imgCurr.clone());
			if (iStep == 0) imgTmpl = imgCurr.clone();

			// prediction
			if (iStep == 0)
			{
				// trackPoints2f_Curr = trackPoint2f;
			}
			else if (iStep == 1 || trackPredictionMethod <= 0)
			{
				fixedPoints2f_Curr = fixedPoints2f_Prev;
				trackPoints2f_Curr = trackPoints2f_Prev;
			}
			else if (iStep == 2 || trackPredictionMethod <= 1)
			{
				for (int iPoint = 0; iPoint < trackPoints2f_Curr.size(); iPoint++)
				{
					trackPoints2f_Curr[iPoint].x = 2 * trackPoints2f_Prev[iPoint].x - trackPoints2f_Pre2[iPoint].x;
					trackPoints2f_Curr[iPoint].y = 2 * trackPoints2f_Prev[iPoint].y - trackPoints2f_Pre2[iPoint].y;
				}
				for (int iPoint = 0; iPoint < fixedPoints2f_Curr.size(); iPoint++)
				{
					fixedPoints2f_Curr[iPoint].x = 2 * fixedPoints2f_Prev[iPoint].x - fixedPoints2f_Pre2[iPoint].x;
					fixedPoints2f_Curr[iPoint].y = 2 * fixedPoints2f_Prev[iPoint].y - fixedPoints2f_Pre2[iPoint].y;
				}
			}
			else {
				for (int iPoint = 0; iPoint < trackPoints2f_Curr.size(); iPoint++)
				{
					trackPoints2f_Curr[iPoint].x = 1 * trackPoints2f_Pre3[iPoint].x
						- 3 * trackPoints2f_Pre2[iPoint].x
						+ 3 * trackPoints2f_Prev[iPoint].x;
					trackPoints2f_Curr[iPoint].y = 1 * trackPoints2f_Pre3[iPoint].y
						- 3 * trackPoints2f_Pre2[iPoint].y
						+ 3 * trackPoints2f_Prev[iPoint].y;
				}
				for (int iPoint = 0; iPoint < fixedPoints2f_Curr.size(); iPoint++)
				{
					fixedPoints2f_Curr[iPoint].x = 1 * fixedPoints2f_Pre3[iPoint].x
						- 3 * fixedPoints2f_Pre2[iPoint].x
						+ 3 * fixedPoints2f_Prev[iPoint].x;
					fixedPoints2f_Curr[iPoint].y = 1 * fixedPoints2f_Pre3[iPoint].y
						- 3 * fixedPoints2f_Pre2[iPoint].y
						+ 3 * fixedPoints2f_Prev[iPoint].y;
				}
			}

			// Step 6:      track reference points (fixed points)
			vector<uchar> optFlow_status(trackPoints2f_Curr.size());
			vector<float> optFlow_error(trackPoints2f_Curr.size());
			int maxLevel = 3;
			if (trackMethod == 1)
			{
				int n = (int)fixedPoints2f_Curr.size();
				float search_x = 10, search_y = 10, search_r = 0.0;
				vector<float> rot_deg(n, 0.0);
				calcTMatchRotPyr(imgTmpl, imgCurr, fixedPoints2f_Tmpl, fixedPoints2f_Curr,
					optFlow_status,
					optFlow_error,
					vector<cv::Size>(n, cv::Size(winSize, winSize)),
					vector<cv::Point2f>(n, cv::Point2f(0.5f, 0.5f)),
					vector<float>(n, search_x),
					vector<float>(n, 0.05f),
					vector<float>(n, search_y),
					vector<float>(n, 0.05f),
					vector<float>(n, 0.0f),
					vector<float>(n, 0.0f),
					rot_deg);
			}

			if (trackMethod == 3)
				cv::calcOpticalFlowPyrLK(imgTmpl, imgCurr, fixedPoints2f_Tmpl, fixedPoints2f_Curr,
					optFlow_status,
					optFlow_error,
					cv::Size(winSize, winSize),
					maxLevel,
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50 /* ecc max count */, 0.001 /* eps */),
					cv::OPTFLOW_USE_INITIAL_FLOW
				);

			// Step 7:		track tracking points (tracking points)

			if (trackMethod == 1)
			{
				int n = (int)trackPoints2f_Curr.size();
				float search_x = 10, search_y = 10, search_r = 0.0;
				vector<float> rot_deg(n, 0.0);
				calcTMatchRotPyr(imgTmpl, imgCurr, trackPoints2f_Tmpl, trackPoints2f_Curr,
					optFlow_status,
					optFlow_error,
					vector<cv::Size>(n, cv::Size(winSize, winSize)),
					vector<cv::Point2f>(n, cv::Point2f(0.5f, 0.5f)),
					vector<float>(n, search_x),
					vector<float>(n, 0.05f),
					vector<float>(n, search_y),
					vector<float>(n, 0.05f),
					vector<float>(n, 0.0f),
					vector<float>(n, 0.0f),
					rot_deg);
			}
			if (trackMethod == 3)
				cv::calcOpticalFlowPyrLK(imgTmpl, imgCurr, trackPoints2f_Tmpl, trackPoints2f_Curr,
					optFlow_status,
					optFlow_error,
					cv::Size(winSize, winSize),
					maxLevel,
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50 /* ecc max count */, 0.001 /* eps */),
					cv::OPTFLOW_USE_INITIAL_FLOW
				);

			int ikey = cv::waitKey(1);
			if (ikey == 27 || ikey == 32)
				break;

			// Step 8:      estimate story motion (ux, uy, torsion) according to
			//              Input: cmat, dvec, rvec, tvec, refPoints2f/3d, trackPoints2f/3d --> ux, uy, torsion
			//                    iteratively guess  (ux, uy, tortion) with (cmat, dvec, rvec, tvec, refPoints3d, trackPoints3d)
			//                              --> fixed refPoints2f and moved trackPoints3d --> (with drvec) --> new refPoints2f and new trackPoint2f
			//                              --> error of (new refPoint2f - actual refPoints2f, and new trackPoints2f - actual trackPoints2f)
			//                              --> adjust ux, uy, tortion, drvec
			//                                  ux, uy, torsion, drvec                     --> new   refPoints2f (v0) and new trackPoint2f (v0)
			//                                  ux + dux (e.g., 0.001), uy, torsion, drvec --> new   refPoints2f (v1) and new trackPoint2f (v1)
			cv::Mat camRot, newDisp, newTrkObjPoints, projNewRefImgPoints, projNewTrkImgPoints;
			newTrkObjPoints = cv::Mat::zeros(num_track_points, 1, CV_64FC3);
			int retVal =
				estimateStoryDispV4(
					cmat,			         // camera matrix
					dvec,			         // distortion vector
					rvec,			         // rotational vector (3, 1, CV_64F)
					tvec,			         // translational vector (3, 1, CV_64F)
					fixedPoints2f,     // reference points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
					trackPoints2f,     // tracking points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
					fixedPoints3d,     // reference points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
					trackPoints3d,     // tracking points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
					fixedPoints2f_Curr,  // moved reference points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
					trackPoints2f_Curr,  // moved tracking points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
					tortionalCenter,    // tortional center point in world coord.
					camRot,                // camera rotational movement (3, 1, CV_64F)
					newDisp,               // displacement of story (ux, uy, tortion) (3, 1, CV_64F)
					newTrkObjPoints,       // moved tracking points in world coord. vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
					projNewRefImgPoints,   // new projection reference points in image coord.
					projNewTrkImgPoints    // new projection tracking points in image coord.
				);

			// if fixed points (reference points) is higher, newDisp *= (-1)
			if (zFixed > zTrack)
				newDisp *= -1.;

			// draw image tracked points and projected points
			cv::Mat imgCurrDraw = imgCurr.clone();
			drawPointsOnImage(imgCurrDraw, cv::Mat(fixedPoints2f_Curr), "square", winSize, winSize / 4, cv::Scalar(255), 0.5, -1, 5);
			drawPointsOnImage(imgCurrDraw, cv::Mat(trackPoints2f_Curr), "X", winSize, winSize / 4, cv::Scalar(255), 0.5, -1, 5);
			drawPointsOnImage(imgCurrDraw, projNewRefImgPoints, "+", winSize, winSize / 4, cv::Scalar(255), 0.5, -1, 5);
			drawPointsOnImage(imgCurrDraw, projNewTrkImgPoints, "+", winSize, winSize / 4, cv::Scalar(255), 0.5, -1, 5);
			imshow_resize("Tracked", imgCurrDraw, 1.5);

			stepnumber += 1;
			struct tm* p2;
			time_t t2 = time(0);
			p2 = localtime(&t);
			std::printf("Floor disp: %16.4f %16.4f    Tortion (degrees): %16.4f      ",
				newDisp.at<double>(0, 0), newDisp.at<double>(1, 0), newDisp.at<double>(2, 0) /*  * 180. / 3.14159265358979323846 */);
			//std::cout << buf << endl;
			ofstream outputfile;
			char str[1000];
			//			snprintf(str, 1000, "H:\\pic\\data.txt");
			std::snprintf(str, 1000, "%sdata.txt", directoryOfFullPathFile(fnameBigTable).c_str());
			outputfile.open(str, ios::app);
			outputfile << stepnumber << "\t" << "time:\t" << 1900 + p2->tm_year << 1 + p2->tm_mon << p2->tm_mday << p2->tm_hour << p2->tm_min << p2->tm_sec << "\t" << 1 << "\t" << "Floor disp:\t" << newDisp.at<double>(0, 0) << "\t\t" << newDisp.at<double>(1, 0) << "\t" << "Tortion (degrees):" << "\t" << newDisp.at<double>(2, 0)
				<< endl;
			outputfile.close();

			// plot
			ikey = plots[0].addDataAndPlot((float)newDisp.at<double>(0));
			ikey = plots[1].addDataAndPlot((float)newDisp.at<double>(1));
			ikey = plots[2].addDataAndPlot((float)newDisp.at<double>(2));
			ikey = plots[3].addDataAndPlot((float)camRot.at<double>(0));
			ikey = plots[4].addDataAndPlot((float)camRot.at<double>(1));
			ikey = plots[5].addDataAndPlot((float)camRot.at<double>(2));

			// updating tmplt if necessary
			if (trackUpdateFreq > 0)
			{
				if (iStep % trackUpdateFreq == 0)
				{
					imgTmpl = imgCurr.clone();
					fixedPoints2f_Tmpl = fixedPoints2f_Curr;
					trackPoints2f_Tmpl = trackPoints2f_Curr;
				}
			}

			// updating
			fixedPoints2f_Pre3 = fixedPoints2f_Pre2;
			fixedPoints2f_Pre2 = fixedPoints2f_Prev;
			fixedPoints2f_Prev = fixedPoints2f_Curr;

			trackPoints2f_Pre3 = trackPoints2f_Pre2;
			trackPoints2f_Pre2 = trackPoints2f_Prev;
			trackPoints2f_Prev = trackPoints2f_Curr;

			// output to bigTable
			if (fBigTable) fprintf(fBigTable, "%7d, ", iiStep++);
			if (fBigTable) fprintf(fBigTable, "%04d%02d%02d%02d%02d%02d%03d, ", 1900 + p2->tm_year, 1 + p2->tm_mon, p2->tm_mday,
				p2->tm_hour, p2->tm_min, p2->tm_sec, i);  // i is the frame number 
			for (int i = 0; i < num_fixed_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", fixedPoints2f_Curr[i].x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", fixedPoints2f_Curr[i].y); // 17 chars
			}
			for (int i = 0; i < num_track_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", trackPoints2f_Curr[i].x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", trackPoints2f_Curr[i].y); // 17 chars
			}
			if (fBigTable) fprintf(fBigTable, "%25.17e, ", newDisp.at<double>(0, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e, ", newDisp.at<double>(1, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e, ", newDisp.at<double>(2, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e, ", camRot.at<double>(0, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e, ", camRot.at<double>(1, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e, ", camRot.at<double>(2, 0)); // 26 chars

			for (int i = 0; i < num_track_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%25.17e, ", newTrkObjPoints.at<cv::Point3d>(i, 0).x); // 26 chars
				if (fBigTable) fprintf(fBigTable, "%25.17e, ", newTrkObjPoints.at<cv::Point3d>(i, 0).y); // 26 chars
				if (fBigTable) fprintf(fBigTable, "%25.17e, ", newTrkObjPoints.at<cv::Point3d>(i, 0).z); // 26 chars
			}

			for (int i = 0; i < num_fixed_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", projNewRefImgPoints.at<cv::Point2f>(i, 0).x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", projNewRefImgPoints.at<cv::Point2f>(i, 0).y); // 17 chars
			}
			for (int i = 0; i < num_track_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", projNewTrkImgPoints.at<cv::Point2f>(i, 0).x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e, ", projNewTrkImgPoints.at<cv::Point2f>(i, 0).y); // 17 chars
			}

			if (fBigTable) fprintf(fBigTable, "\n");

		}//end of 31
	}// end of tracking loop

	if (fBigTable) fclose(fBigTable);

	cv::waitKey(0);
	cv::destroyAllWindows();
	return 0;

}
