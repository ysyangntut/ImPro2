#define _CRT_SECURE_NO_WARNINGS     // Make Microsoft Visual Studio quiet on localtime() and many other functions
//#include "pch.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <thread>
#include <chrono>


#include "impro_util.h"
#include "trackings.h"




using namespace std;
using namespace cv;


cv::Point2f selectROI_zoom(cv::Mat img, int nLevel);

cv::Point2f test_selectROI_zoom();

int test_estimateStoryDispV4();
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
	cv::Mat & camRot,                // camera rotational movement (3, 1, CV_64F)
	cv::Mat & newDisp,               // displacement of story (ux, uy, tortion) (3, 1, CV_64F)
	cv::Mat & newTrkObjPoints,       // moved tracking points in world coord. vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::Mat & projNewRefImgPoints,   // new projection reference points in image coord.
	cv::Mat & projNewTrkImgPoints    // new projection tracking points in image coord.
);
#pragma once 


int FuncStoryDispV6(int argc, char ** argv)
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
//    string fnameBigTable("./bigTable.txt");
	string fnameBigTable;





	//while (true) {
	//struct tm *p;
	//time_t t = time(0);
	//p = localtime(&t);
	//std::cout << p->tm_sec << endl;
	//if (p->tm_sec == 0) {
	//	std::cout << p->tm_sec << endl;
	//	break;
	//}
	//
	//
	//else if(p->tm_sec == 30)break;
	//  
	//}
	//std::cout << "good" << endl;
	// Step 1: Get video source:
	cv::VideoCapture vid;
	std::string vidName;
	//while (true) {
	//	cout << "# Enter video sour	ce (the string for cv::VideoCapture): \n";
	//	vidName = readStringFromCin();
	//	if (vidName.length() > 1)
	//		vid.open(vidName);
	//	else
	//		vid.open(vidName[0] - '0');
	//	if (vid.isOpened() == true) break;
	//	cerr << "# Cannot open " << vidName << ". Try again.\n";
	//}
	//// print video properties
	//cout << "CAP_PROP_FRAME_WIDTH : " << vid.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
	//cout << "CAP_PROP_FRAME_HEIGHT : " << vid.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
	//cout << "CAP_PROP_FPS : " << vid.get(cv::CAP_PROP_FPS) << endl;
	//cout << "CAP_PROP_FRAME_COUNT : " << vid.get(cv::CAP_PROP_FRAME_COUNT) << endl;
	//
	//// set camera resolution
	//while (vidName.length() <= 1) {
	//	// turn off the camera
	//	if (vid.isOpened() == true)
	//		//vid.release();
	//	// ask user to input resolution
	//		cout << "# Enter video resolution (width height):  \n";
	//	int vw = readIntFromIstream(std::cin);
	//	int vh = readIntFromIstream(std::cin);
	//
	//	// try to setup the resolution
	//	vid.set(cv::CAP_PROP_FRAME_WIDTH, (double)vw);
	//	vid.set(cv::CAP_PROP_FRAME_HEIGHT, (double)vh);
	//	// check if it works
	//	if (vid.isOpened() == false)
	//		if (vidName.length() > 1)
	//			vid.open(vidName);
	//		else
	//			vid.open(vidName[0] - '0');
	//	// print video properties
	//	cout << "CAP_PROP_FRAME_WIDTH : " << vid.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
	//	cout << "CAP_PROP_FRAME_HEIGHT : " << vid.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
	//	cout << "CAP_PROP_FPS : " << vid.get(cv::CAP_PROP_FPS) << endl;
	//	cout << "CAP_PROP_FRAME_COUNT : " << vid.get(cv::CAP_PROP_FRAME_COUNT) << endl;
	//	// ask user if it is okay
	//	cout << "# Does it work well? (0: re-set again. 1: go ahead.): \n";
	//	int go_ahead = readIntFromIstream(std::cin);
	//	if (go_ahead == 0) continue;
	//	else break;
	//}

	// Ready to output big table file
	FILE* fBigTable;
	int ferr;
	//fnameBigTable = extFilenameRemoved(vidName) + std::string(".txt");//set bigTable dir
	fnameBigTable = extFilenameRemoved(vidName) + std::string(".txt");//set bigTable dir
	cout << "bulidbigtable" << endl;
	ferr = fopen_s(&fBigTable, fnameBigTable.c_str(), "w");
	printf("Big table file is at: %s\n", fnameBigTable.c_str()); std::cout.flush();

	// output to bigTable
	//if (fBigTable) fprintf(fBigTable, "Video_source: %s\n", vidName.c_str());
	//if (fBigTable) fprintf(fBigTable, "CAP_PROP_FRAME_WIDTH:  %12.4f\n", vid.get(cv::CAP_PROP_FRAME_WIDTH));
	//if (fBigTable) fprintf(fBigTable, "CAP_PROP_FRAME_HEIGHT: %12.4f\n", vid.get(cv::CAP_PROP_FRAME_HEIGHT));
	//if (fBigTable) fprintf(fBigTable, "CAP_PROP_FPS: %12.4f\n", vid.get(cv::CAP_PROP_FPS));
	//if (fBigTable) fprintf(fBigTable, "CAP_PROP_FRAME_COUNT: %12.4f\n", vid.get(cv::CAP_PROP_FRAME_COUNT));


	// get initial image
	//vid >> imgInit;
	//if (imgInit.channels() > 1)
	//	cv::cvtColor(imgInit, imgInit, cv::COLOR_BGR2GRAY);
	//imshow_resize("Initial image", imgInit, (double)1024. / imgInit.cols);
	//cv::waitKey(1000);
	//cv::destroyWindow("Initial image");

	//eric get intial image

	std::cout << "Input full path of  cam initial photo (or VideoCapture0 for camera 0, and so on):\n";
	fnameImgInit = readStringLineFromIstream(std::cin);
	imgInit = cv::imread(fnameImgInit, cv::IMREAD_GRAYSCALE);
	//if (imgInit.channels() > 1)
	   // cv::cvtColor(imgInit, imgInit, cv::COLOR_BGR2GRAY);
	imshow_resize("Initial image", imgInit, (double)1024. / imgInit.cols);
	cv::waitKey(1000);
	cv::destroyWindow("Initial image");

	// Step 1: Get camera parameters

	cv::Mat cmat, dvec, rvec, tvec, r3, r4, r4inv, campos, camdir;
	//	string fnameIntrinsic = "/home/pi/Downloads/raspi_cam1_intrinsic.xml";
	//	string fnameExtrinsic = "/home/pi/Downloads/0503_rasberryCam1_8Points_extrinsic.xml";
	cout << "# Enter camera intrinsic file:\n";
	string fnameIntrinsic = readStringLineFromCin();
	cv::FileStorage ifs(fnameIntrinsic, cv::FileStorage::READ);
	ifs["cameraMatrix"] >> cmat;
	ifs["distortionVector"] >> dvec;
	cout << "Cmat:\n" << cmat << endl << "Dvec:\n" << dvec << endl;
	ifs.release();
	cout << "# Enter camera extrinsic file:\n";
	string fnameExtrinsic = readStringLineFromCin();
	ifs.open(fnameExtrinsic, cv::FileStorage::READ);
	//	ifs["rvec"] >> rvec;
	//	ifs["tvec"] >> tvec;
	ifs["R4"] >> r4;
	if (r4.rows <= 0 || r4.cols <= 0)
		ifs["r4"] >> r4;

	cout << "r4: \n" << r4 << endl;

	r4(cv::Rect(0, 0, 3, 3)).copyTo(r3);
	r4(cv::Rect(3, 0, 1, 3)).copyTo(tvec);
	cv::Rodrigues(r3, rvec);
	r4inv = r4.inv();
	r4inv(cv::Rect(3, 0, 1, 3)).copyTo(campos);
	r4inv(cv::Rect(2, 0, 1, 3)).copyTo(camdir);
	cout << "rvec:\n" << rvec << "\ntvec:\n" << tvec << endl;
	cout << "camPos:\n" << campos << "\ncamdir:\n" << camdir << endl;

	// output to big table
	if (fBigTable) fprintf(fBigTable, "Fx: %25.17e\n", cmat.at<double>(0, 0));
	if (fBigTable) fprintf(fBigTable, "Fy: %25.17e\n", cmat.at<double>(1, 1));
	if (fBigTable) fprintf(fBigTable, "Cx: %25.17e\n", cmat.at<double>(0, 2));
	if (fBigTable) fprintf(fBigTable, "Cy: %25.17e\n", cmat.at<double>(1, 2));
	if (fBigTable) fprintf(fBigTable, "K1: %25.17e\n", dvec.at<double>(0, 0));
	if (fBigTable) if (dvec.cols > 1) fprintf(fBigTable, "K2: %25.17e\n", dvec.at<double>(0, 1));
	if (fBigTable) if (dvec.cols > 2) fprintf(fBigTable, "P1: %25.17e\n", dvec.at<double>(0, 2));
	if (fBigTable) if (dvec.cols > 3) fprintf(fBigTable, "P2: %25.17e\n", dvec.at<double>(0, 3));
	if (fBigTable) if (dvec.cols > 4) fprintf(fBigTable, "K3: %25.17e\n", dvec.at<double>(0, 4));
	if (fBigTable) if (dvec.cols > 5) fprintf(fBigTable, "K4: %25.17e\n", dvec.at<double>(0, 5));
	if (fBigTable) if (dvec.cols > 6) fprintf(fBigTable, "K5: %25.17e\n", dvec.at<double>(0, 6));
	if (fBigTable) if (dvec.cols > 7) fprintf(fBigTable, "K6: %25.17e\n", dvec.at<double>(0, 7));
	if (fBigTable) fprintf(fBigTable, "rx: %25.17e\n", ((double*)rvec.data)[0]);
	if (fBigTable) fprintf(fBigTable, "ry: %25.17e\n", ((double*)rvec.data)[1]);
	if (fBigTable) fprintf(fBigTable, "rz: %25.17e\n", ((double*)rvec.data)[2]);
	if (fBigTable) fprintf(fBigTable, "tx: %25.17e\n", ((double*)tvec.data)[0]);
	if (fBigTable) fprintf(fBigTable, "ty: %25.17e\n", ((double*)tvec.data)[1]);
	if (fBigTable) fprintf(fBigTable, "tz: %25.17e\n", ((double*)tvec.data)[2]);

	// Step 2: Ask user to define reference points (supposed to be fixed) both 2D (image) and 3D (world).
	cout << "# Enter number of reference points (fixed points): \n";
	num_fixed_points = readIntFromIstream(std::cin);
	fixedPoints2f.resize(num_fixed_points);
	fixedPoints3d.resize(num_fixed_points);
	for (int i = 0; i < num_fixed_points; i++)
	{
		cout << "# Enter 0 for manual picking. Enter 1 for manual input. \n";
		int inputMode = readIntFromIstream(std::cin);
		if (inputMode == 0) {
			while (imgInit.rows <= 0 || imgInit.cols <= 0)
			{
				cout << "# Initial image is empty. \n";
				cout << "# Enter initial image full file name: \n";
				fnameImgInit = readStringLineFromCin();
				imgInit = cv::imread(fnameImgInit);
			}
			fixedPoints2f[i] = selectROI_zoom(imgInit, 3);
			cout << "# You picked " << fixedPoints2f[i].x << " " << fixedPoints2f[i].y << endl;
			cout << "# Define reference points " << i << " (in world 3d. For example:  xw yw zw):\n";
		}
		else {
			cout << "# Define reference points " << i << " (in image 2d and world 3d. For example:  xi yi xw yw zw):\n";
			fixedPoints2f[i].x = (float)readDoubleFromIstream(std::cin);
			fixedPoints2f[i].y = (float)readDoubleFromIstream(std::cin);
		}
		fixedPoints3d[i].x = readDoubleFromIstream(std::cin);
		fixedPoints3d[i].y = readDoubleFromIstream(std::cin);
		fixedPoints3d[i].z = readDoubleFromIstream(std::cin);
	}

	// output to big table
	if (fBigTable) fprintf(fBigTable, "numRefPoints: %4d\n", num_fixed_points);
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "refPoints2f[ %4d ]: %16.9e %16.9e   refPoints3d[ %4d ]: %25.17e %25.17e %25.17e\n",
			i, fixedPoints2f[i].x, fixedPoints2f[i].y,
			i, fixedPoints3d[i].x, fixedPoints3d[i].y, fixedPoints3d[i].z);
	}

	// Step 3: Ask user to define tracking points (supposed to move within constrained surface) both 2D (image) and 3D (world).
	cout << "# Enter number of tracking points (movable points): \n";
	num_track_points = readIntFromIstream(std::cin);
	trackPoints2f.resize(num_track_points);
	trackPoints3d.resize(num_track_points);
	for (int i = 0; i < num_track_points; i++)
	{
		cout << "# Enter 0 for manual picking. Enter 1 for manual input. \n";
		int inputMode = readIntFromIstream(std::cin);
		if (inputMode == 0) {
			while (imgInit.rows <= 0 || imgInit.cols <= 0)
			{
				cout << "# Initial image is empty. \n";
				cout << "# Enter initial image full file name: \n";
				fnameImgInit = readStringLineFromCin();
				imgInit = cv::imread(fnameImgInit);
			}
			trackPoints2f[i] = selectROI_zoom(imgInit, 3);
			cout << "# You picked " << trackPoints2f[i].x << " " << trackPoints2f[i].y << endl;
			cout << "# Define tracking points " << i << " (in world 3d. For example:  xw yw zw):\n";
		}
		else {
			cout << "# Define tracking points " << i << " (in image 2d and world 3d. For example:  xi yi xw yw zw):\n";
			trackPoints2f[i].x = (float)readDoubleFromIstream(std::cin);
			trackPoints2f[i].y = (float)readDoubleFromIstream(std::cin);
		}
		trackPoints3d[i].x = readDoubleFromIstream(std::cin);
		trackPoints3d[i].y = readDoubleFromIstream(std::cin);
		trackPoints3d[i].z = readDoubleFromIstream(std::cin);
	}

	// output to big table
	if (fBigTable) fprintf(fBigTable, "numTrkPoints: %4d\n", num_fixed_points);
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "trackPoints2f[ %4d ]: %16.9e %16.9e   trackPoints3d[ %4d ]: %25.17e %25.17e %25.17e\n",
			i, trackPoints2f[i].x, trackPoints2f[i].y,
			i, trackPoints3d[i].x, trackPoints3d[i].y, trackPoints3d[i].z);
	}

	cout << "# Define tortional center (xc yc zc): \n";
	tortionalCenter.x = readDoubleFromIstream(std::cin);
	tortionalCenter.y = readDoubleFromIstream(std::cin);
	tortionalCenter.z = readDoubleFromIstream(std::cin);

	// output to big table
	if (fBigTable) fprintf(fBigTable, "tortionalCenter: %25.17e %25.17e %25.17e\n",
		tortionalCenter.x, tortionalCenter.y, tortionalCenter.z);


	cv::Mat matFixedPoints2f(fixedPoints2f);
	cv::Mat matFixedPoints3d(fixedPoints3d);
	cv::Mat matTrackPoints2f(trackPoints2f);
	cv::Mat matTrackPoints3d(trackPoints3d);

	cout << "Fixed points 2f: \n" << fixedPoints2f << endl;
	cout << "Fixed points 3d: \n" << fixedPoints3d << endl;
	cout << "Tracking points 2f: \n" << trackPoints2f << endl;
	cout << "Tracking points 3d: \n" << trackPoints3d << endl;

	// Step 4: Select tracking method and options
	//   4.1: Tracking method: default, template match
	//   4.2: Target updating frequency (default 1, update each frame, 0 for not-updated and using initial template)
	//   4.3: Prediction method (default: 2-nd order)
	//
	cout << "# Select tracking method: (1)Template match, (2)Ecc, (3)Optical flow:\n";
	trackMethod = readIntFromIstream(std::cin, 1, 3);
	cout << "# Your selection is " << trackMethod << endl;
	cout << "# Target updating frequency: (0)Not updating (N)Updating every N frames: \n";
	trackUpdateFreq = readIntFromIstream(std::cin, 0, 99999999);
	cout << "# Your selection is " << trackUpdateFreq << endl;
	cout << "# Prediction method: (0)Previous point, (1)Linear approx, (2)2nd-order approx.:\n";
	trackPredictionMethod = readIntFromIstream(std::cin, 0, 2);
	cout << "# Your selection is " << trackPredictionMethod << endl;
	int defaultWinSize = std::min(imgInit.cols, imgInit.rows) / 50;
	cout << "# Window size (or template size) (0 for 1/50 of min(image width, height), " << defaultWinSize << "):\n";
	winSize = readIntFromIstream(std::cin, 0, std::min(imgInit.cols, imgInit.rows) - 1);
	if (winSize <= 0) winSize = defaultWinSize;
	cout << "Window size is " << winSize << endl;

	// output to big table
	if (fBigTable) fprintf(fBigTable, "trackMethod: %d\n", trackMethod);
	if (fBigTable) fprintf(fBigTable, "trackUpdateFreq: %d\n", trackUpdateFreq);
	if (fBigTable) fprintf(fBigTable, "trackPredictionMethod: %d\n", trackPredictionMethod);
	if (fBigTable) fprintf(fBigTable, "winSize: %d\n", winSize);

	// Step 5: start the tracking loop
	cv::Mat imgCurr, imgTmpl;
	//	cv::VideoCapture vid("v4l2src ! video/x-raw,format=NV12,width=1640,height=1232 ! videoconvert ! appsink");
	//if (vid.isOpened() == false) {
	//	cerr << "Cannot open camera.\n";
	//	return -1;
	//}
	//else
	//{
	//	printf("Camera is opened.\n");
	//	while (true)
	//	{
	//		vid >> imgCurr;
	//		if (imgCurr.rows <= 0 || imgCurr.cols <= 0) break;
	//		if (imgCurr.channels() > 1)
	//			cv::cvtColor(imgCurr, imgCurr, cv::COLOR_BGR2GRAY);
	//		// plot points on image
//  //         drawPointsOnImage(imgCurr, matFixedPoints2f, "o", 16, 8);
//  //         drawPointsOnImage(imgCurr, matTrackPoints2f, "X",  4, 2);
	//		imshow_resize("Camera", imgCurr, (double) 1024. / imgCurr.cols);
	//		//			cv::imshow("Camera", imgCurr);
	//		int ikey = cv::waitKey(3);
	//		if (ikey == 27 || ikey == 32)
	//			break;
	//	}
	//	//        vid.release();
	//	cv::destroyWindow("Camera");
	//	//        return 0;
	//}

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
	if (fBigTable) fprintf(fBigTable, "   Step "); // 8 chars (including ending space)
	if (fBigTable) fprintf(fBigTable, "             Time "); // YYYYMMDDhhmmsssss (18 chars including ending space)
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "RefPnt2f[%d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "RefPnt2f[%d].y ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "TrkPnt2f[%d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "TrkPnt2f[%d].y ", i); // 17 chars
	}
	if (fBigTable) fprintf(fBigTable, "             FloorDisp.ux "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "             FloorDisp.uy "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "        FloorDisp.tortion "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.rx "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.ry "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.rz "); // 26 chars



	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "         CalTrk3d[%d].x ", i); // 26 chars
		if (fBigTable) fprintf(fBigTable, "         CalTrk3d[%d].y ", i); // 26 chars
		if (fBigTable) fprintf(fBigTable, "         CalTrk3d[%d].z ", i); // 26 chars
	}
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "PrjRef2f[%d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "PrjRef2f[%d].y ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "PrjTrk2f[%d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "PrjTrk2f[%d].y ", i); // 17 chars
	}
	if (fBigTable) fprintf(fBigTable, "\n");

	//----eric photo frequence

	struct tm user_input_time;
	struct tm user_next_input_time;
	struct tm user_input_finish_time;//user input end time
	struct tm user_next_finish_time;//=used user input end time           
	char buf[1000]; //  , startpic[1000];
	memset(&user_input_time, 0, sizeof(tm));
	int fre_type = 0;
	int track_frequence = 0;
	//-------------------------------------------------------------------------------------
	int breakpoint = 0;
	//while (true && breakpoint != 1) {

	cout << "enter analysis start time" << endl;
	cout << "year = ";
	cin >> user_input_time.tm_year;

	cout << "month = ";
	cin >> user_input_time.tm_mon;

	cout << "day =";
	cin >> user_input_time.tm_mday;

	cout << "hour =";
	cin >> user_input_time.tm_hour;
	cout << "min =";
	cin >> user_input_time.tm_min;
	cout << "sec = \n";
	cin >> user_input_time.tm_sec;


	cout << "the analysis stat time: " << user_input_time.tm_year << "/" << user_input_time.tm_mon << "/" <<
		user_input_time.tm_mday << " " << user_input_time.tm_hour << ":" << user_input_time.tm_min << ":" << user_input_time.tm_sec << endl;
	//-----------------------------------------------------------------------------------
	//finish time
	cout << " enter analysis finish time" << endl;
	cout << "year = ";
	cin >> user_input_finish_time.tm_year;

	cout << "month = ";
	cin >> user_input_finish_time.tm_mon;

	cout << "day =";
	cin >> user_input_finish_time.tm_mday;

	cout << "hour =";
	cin >> user_input_finish_time.tm_hour;
	cout << "min =";
	cin >> user_input_finish_time.tm_min;
	cout << "sec = \n";
	cin >> user_input_finish_time.tm_sec;


	cout << "the analysis finish time: " << user_input_finish_time.tm_year << "/" << user_input_finish_time.tm_mon << "/" <<
		user_input_finish_time.tm_mday << " " << user_input_finish_time.tm_hour << ":" << user_input_finish_time.tm_min << ":" << user_input_finish_time.tm_sec << endl;

	//-------------------------
	cout << "#  tracking frequence type: (1)each  frames (2) sec (3) min (4) hour (5) day (6) month (7)year :\n";
	fre_type = readIntFromIstream(std::cin, 1, 6);






	if (fre_type == 1) {
		track_frequence = 1;
		cout << "you chouse each frames" << track_frequence << endl;
	}

	if (fre_type == 2) {
		cout << "How many sec between each frame?" << endl;
		cin >> track_frequence;

	}
	if (fre_type == 3) {
		cout << "How many minutes between each frame?" << endl;
		cin >> track_frequence;
		//each_fram_sec += track_frequence * 60;
	}

	if (fre_type == 4) {
		cout << "How many hours between each frame?" << endl;
		cin >> track_frequence;
		//each_fram_sec += track_frequence * 3600;
	}
	if (fre_type == 5) {
		cout << "How many days between each frame?" << endl;
		cin >> track_frequence;
		//each_fram_sec += track_frequence * 86400;
	}

	if (fre_type == 6) {
		cout << "How many month between each frame?" << endl;
		cin >> track_frequence;
		//each_fram_sec += track_frequence * 2629743;//(30.44 คั)
	}
	if (fre_type == 7) {
		cout << "How many year between each frame?" << endl;
		cin >> track_frequence;
		//each_fram_sec += track_frequence * 31556736;;//(365.24 คั)
	}
	//--------------------------------------------------------------------------
	user_next_input_time = user_input_time;
	user_next_finish_time = user_input_finish_time;



	user_next_input_time.tm_year -= 1900;
	user_next_input_time.tm_mon--;
	user_next_finish_time.tm_year -= 1900;
	user_next_finish_time.tm_mon--;

	//snprintf(startpic, 1000, "H:\\pic\\%04d%02d%02d%02d%02d%02d_%03d.jpg",
	//	1900 + user_next_input_time.tm_year, 1 + user_next_input_time.tm_mon, user_next_input_time.tm_mday, user_next_input_time.tm_hour,
	//	user_next_input_time.tm_min, user_next_input_time.tm_sec, 0);
	//cv::Mat trypic = imread(startpic, cv::IMREAD_GRAYSCALE);
	//
	//if (trypic.data != 0)break;
	//
	//else
	//{  
	//	
	//	for (int loop = 0; loop < 3 ; loop++) {
	//		if (fre_type == 2)user_next_input_time.tm_sec += track_frequence;
	//		if (fre_type == 3)user_next_input_time.tm_min += track_frequence;
	//		if (fre_type == 4)user_next_input_time.tm_hour += track_frequence;
	//		if (fre_type == 5)user_next_input_time.tm_mday += track_frequence;
	//		if (fre_type == 6)user_next_input_time.tm_mon += track_frequence;
	//		if (fre_type == 7)user_next_input_time.tm_year += track_frequence;
	//		snprintf(startpic, 1000, "H:\\pic\\%04d%02d%02d%02d%02d%02d_%03d.jpg",
	//			1900 + user_next_input_time.tm_year, 1 + user_next_input_time.tm_mon, user_next_input_time.tm_mday, user_next_input_time.tm_hour,
	//			user_next_input_time.tm_min, user_next_input_time.tm_sec, 0);
	//		cv::Mat trypic = imread(startpic, cv::IMREAD_GRAYSCALE);
	//
	//		if (trypic.data != 0) {
	//			 breakpoint = 1;
	//			break;
	//		}
	//		
	//	}
	//
	//}

	//cv::imshow("test", trypic);
	//cv::waitKey(5000);
	//cv::destroyWindow("test");

//}



	//ut << "great" << endl;



	//cout << endl << mktime(&user_input_time) << endl;


	int stepnumber = 0;


	for (size_t iStep = 0; true; iStep++)
	{
		int br = 0;

		int a = (int) mktime(&user_next_input_time);//timestamp(xxxxxxxx
		int b = (int) mktime(&user_next_finish_time);//timestamp(xxxxxxxx

		//cout <<"a"<<a<< endl;
		//cout << "b" << b << endl;
		time_t t = a;
		tm * ltm = localtime(&t);
		//cout << ltm << endl;

		if (a > b) {
			cout << "finish analysis" << endl;
			cout << "the last file name : " << buf << endl;
			//cout << "real finish time" << user_next_finish_time.tm_year << "/" << user_next_finish_time.tm_mon << "/" <<
				//user_next_finish_time.tm_mday << " " << user_next_finish_time.tm_hour << ":" << user_next_finish_time.tm_min << 
				//":" << user_next_finish_time.tm_sec << endl;
			break;
		}

		for (int i = 0; true; i++)
		{

			if (br == 1)break;

			if (fre_type == 1) {
				snprintf(buf, 1000, "H:\\pic\\344_10\\%04d%02d%02d%02d%02d%02d_%03d.jpg",
					1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, i);
			}

			else if (fre_type != 1) {
				snprintf(buf, 1000, "H:\\pic\\344_10\\%04d%02d%02d%02d%02d%02d_%03d.jpg",
					1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, 0);
				if (fre_type == 2)user_next_input_time.tm_sec += track_frequence;
				if (fre_type == 3)user_next_input_time.tm_min += track_frequence;
				if (fre_type == 4)user_next_input_time.tm_hour += track_frequence;
				if (fre_type == 5)user_next_input_time.tm_mday += track_frequence;
				if (fre_type == 6)user_next_input_time.tm_mon += track_frequence;
				if (fre_type == 7)user_next_input_time.tm_year += track_frequence;

				br = 1;//br=breakpoint
			}

			cout << buf << endl;
			cv::Mat img = cv::imread(buf, cv::IMREAD_GRAYSCALE);
			time_t now = time(0);
			//cv::imshow("test", img);
		//cv::waitKey(1000);
		//cv::destroyWindow("test");

			while (true) {
				time_t now = time(0);
				if (now > t) break;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}


			if (!img.data && fre_type == 1 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_sec += 1;
				break;
			}
			if (!img.data && fre_type == 2 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_sec += track_frequence;
				break;
			}
			if (!img.data && fre_type == 3 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_min += track_frequence;
				break;
			}
			if (!img.data && fre_type == 4 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_hour += track_frequence;
			}
			if (!img.data && fre_type == 5 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_mday += track_frequence;
				break;
			}
			if (!img.data && fre_type == 6 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_mon += track_frequence;
				break;
			}
			if (!img.data && fre_type == 7 && now > t) {
				cout << "Could not open or find the image" << endl;
				user_next_input_time.tm_year += track_frequence;
				break;
			}






			cv::Mat imgCurr = cv::imread(buf, cv::IMREAD_GRAYSCALE);


			if (imgCurr.rows <= 0 || imgCurr.cols <= 0)
			{
				cout << "no picture" << endl;
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

		   // cout << trackPoints2f_Tmpl[0] <<"\t"<<trackPoints2f_Tmpl[1]<<"\t"<<trackPoints2f_Tmpl[2]<<"\t"<<trackPoints2f_Tmpl[3]
		   //      <<"\t"<<trackPoints2f_Tmpl[4]<<"\t"<<trackPoints2f_Tmpl[5]<<"\t"<<trackPoints2f_Tmpl[6];

		   // cout << trackPoints2f_Tmpl[0] << " c" << trackPoints2f_Curr[0]<<"d";
			//cout << trackPoints2f_Curr[0] << "b";
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

			//cout << trackPoints2f_Curr[0] << "c" << endl;

			cv::Mat imgCurrDraw = imgCurr.clone();
			drawPointsOnImage(imgCurrDraw, cv::Mat(fixedPoints2f_Curr), "square", winSize, winSize / 4, cv::Scalar(255), 0.5, -1, 5);
			drawPointsOnImage(imgCurrDraw, cv::Mat(trackPoints2f_Curr), "X", winSize, winSize / 4, cv::Scalar(255), 0.5, -1, 5);
			imshow_resize("Tracked", imgCurrDraw, 0.66);
			int ikey = cv::waitKey(30);
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
			stepnumber += 1;
			struct tm *p2;
			time_t t2 = time(0);
			p2 = localtime(&t);
			printf("Floor disp: %16.4f %16.4f    Tortion (degrees): %16.4f      ",
				newDisp.at<double>(0, 0), newDisp.at<double>(1, 0), newDisp.at<double>(2, 0) /*  * 180. / 3.14159265358979323846 */);
			//cout << buf << endl;
			ofstream outputfile;
			char str[1000];
			snprintf(str, 1000, "H:\\pic\\data.txt");
			outputfile.open(str, ios::app);
			outputfile << stepnumber << "\t" << "time:\t" << 1900 + p2->tm_year << 1 + p2->tm_mon << p2->tm_mday << p2->tm_hour << p2->tm_min << p2->tm_sec << "\t" << 1 << "\t" << "Floor disp:\t" << newDisp.at<double>(0, 0) << "\t\t" << newDisp.at<double>(1, 0) << "\t" << "Tortion (degrees):" << "\t" << newDisp.at<double>(2, 0)
				<< endl;

			//   outputfile<<iStep<<"\t"<<"time:\t"<<1900+p->tm_year<<1+p->tm_mon<<p->tm_mday<<p->tm_hour<<p->tm_min<<p->tm_sec<<"\t"
			//            <<"Floor disp:\t"<<  newDisp.at<double>(0, 0)<<"\t"
			//            << newDisp.at<double>(1, 0)<<"\t"
			//            <<"Tortion (degrees):"<<"\t"
			//            <<newDisp.at<double>(2, 0)
			//            <<endl;
		   //snprintf(filename, 1000, " %04d%02d%02d%02d%02d%02d_%05d.jpg",1900+p->tm_year, 1+p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec, ++count);


		// plot
			bool plot = true;
			float camRotPlotMax[3] = { 0.1f, 0.1f, 0.1f };
			float camRotPlotMin[3] = { -.1f, -.1f, -.1f };
			float newDispPlotMax[3] = { 10.f, 10.f, 10.f };
			float newDispPlotMin[3] = { -10.f, -10.f, -10.f };
			int plotWidth = 1000;
			int plotHeight = 400;
			static std::vector<cv::Mat> plotCamRot(3, cv::Mat(plotHeight, plotWidth, CV_8UC3, cv::Scalar(255, 255, 255)));
			static std::vector<cv::Mat> plotNewDisp(3, cv::Mat(plotHeight, plotWidth, CV_8UC3, cv::Scalar(255, 255, 255)));
			std::vector<std::string> plotCamRotName = { "CamRot X", "CamRot Y", "CamRot Z" };
			std::vector<std::string> plotNewDispName = { "Disp X", "Disp Y", "Disp Z" };
			char buf[1000];
			for (int i = 0; i < 3; i++) {
				snprintf(buf, 1000, ".   Max:%6.2f  Min:%6.2f", camRotPlotMax[i], camRotPlotMin[i]);
				plotCamRotName[i] += std::string(buf);
				snprintf(buf, 1000, ".   Max:%6.2f  Min:%6.2f", newDispPlotMax[i], newDispPlotMin[i]);
				plotNewDispName[i] += std::string(buf);
			}

			if (plot == true)
			{
				static int iPlot = 0;
				// Plot 3 camRot plots
				for (int i = 0; i < 3; i++)
				{
					// clear pixel of this iPlot 
					for (int clearX = 0; clearX < 5; clearX++) {
						if (iPlot + clearX < plotWidth)
						{
							if (clearX == 5 - 1)
								cv::line(plotCamRot[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(128, 128, 128), 1, 8);
							else
								cv::line(plotCamRot[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(255, 255, 255), 1, 8);
						}
					}
					// plot axis
					float y = 0.0f;
					float maxY = camRotPlotMax[i];
					float minY = camRotPlotMin[i];
					int iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
					if (iy < 0) iy = 0;
					if (iy >= plotHeight) iy = plotHeight - 1;
					cv::line(plotCamRot[i], cv::Point(0, iy), cv::Point(plotWidth - 1, iy), cv::Scalar(128, 128, 128), 1, 8);
					// plot data point
					y = (float)camRot.at<double>(i);
					iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
					if (iy < 0) iy = 0;
					if (iy >= plotHeight) iy = plotHeight - 1;
					cv::line(plotCamRot[i], cv::Point(iPlot, iy), cv::Point(iPlot, iy), cv::Scalar(0, 0, 0), 1, 8);
				}
				// Plot 3 newDisp plots
				for (int i = 0; i < 3; i++)
				{
					// clear pixel of this iPlot 
					for (int clearX = 0; clearX < 5; clearX++) {
						if (iPlot + clearX < plotWidth)
						{
							if (clearX == 5 - 1)
								cv::line(plotNewDisp[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(128, 128, 128), 1, 8);
							else
								cv::line(plotNewDisp[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(255, 255, 255), 1, 8);
						}
					}
					// plot axis
					float y = 0.0f;
					float maxY = newDispPlotMax[i];
					float minY = newDispPlotMin[i];
					int iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
					if (iy < 0) iy = 0;
					if (iy >= plotHeight) iy = plotHeight - 1;
					cv::line(plotNewDisp[i], cv::Point(0, iy), cv::Point(plotWidth - 1, iy), cv::Scalar(128, 128, 128), 1, 8);
					// plot data point
					y = (float)newDisp.at<double>(i);
					iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
					if (iy < 0) iy = 0;
					if (iy >= plotHeight) iy = plotHeight - 1;
					cv::line(plotNewDisp[i], cv::Point(iPlot, iy), cv::Point(iPlot, iy), cv::Scalar(0, 0, 0), 1, 8);
				}
				// show image
				for (int i = 0; i < 3; i++)
					cv::imshow(plotCamRotName[i], plotCamRot[i]);
				for (int i = 0; i < 3; i++)
					cv::imshow(plotNewDispName[i], plotNewDisp[i]);
				cv::waitKey(2);

				iPlot++;
				iPlot = iPlot % plotWidth;
			} // end of if plot == true	


			outputfile.close();
			//
			//if (fre_type == 1 && abs(newDisp.at<double>(0, 0)) > 0.025) {
			//	//each_fram_sec = 2;
			//
			////	cout << "hhhhhhhhhhhhhhh" << endl;
			//	//cout << each_fram_sec << endl;
			//	//system("pause");
			//	//continue;
			//}
			//
			//if (fre_type == 1 && abs(newDisp.at<double>(0, 0)) < 0.025) {
			//	//each_fram_sec = 1;
			//
			//	//cout << "aaaa" << endl;
			//	//cout << each_fram_sec << endl;
			//	//system("pause");
			//	//continue;
			//}



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
			if (fBigTable) fprintf(fBigTable, "%7d ", (int)iStep);
			if (fBigTable) fprintf(fBigTable, "%04d%02d%02d%02d%02d%02d%03d ", 1900 + p2->tm_year, 1 + p2->tm_mon, p2->tm_mday,
				p2->tm_hour, p2->tm_min, p2->tm_sec, 0);
			for (int i = 0; i < num_fixed_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e ", fixedPoints2f_Curr[i].x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e ", fixedPoints2f_Curr[i].y); // 17 chars
			}
			for (int i = 0; i < num_track_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e ", trackPoints2f_Curr[i].x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e ", trackPoints2f_Curr[i].y); // 17 chars
			}
			if (fBigTable) fprintf(fBigTable, "%25.17e ", newDisp.at<double>(0, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e ", newDisp.at<double>(1, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e ", newDisp.at<double>(2, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e ", camRot.at<double>(0, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e ", camRot.at<double>(1, 0)); // 26 chars
			if (fBigTable) fprintf(fBigTable, "%25.17e ", camRot.at<double>(2, 0)); // 26 chars
			for (int i = 0; i < num_fixed_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e ", projNewRefImgPoints.at<cv::Point2f>(i, 0).x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e ", projNewRefImgPoints.at<cv::Point2f>(i, 0).y); // 17 chars
			}
			for (int i = 0; i < num_track_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%16.9e ", projNewTrkImgPoints.at<cv::Point2f>(i, 0).x); // 17 chars
				if (fBigTable) fprintf(fBigTable, "%16.9e ", projNewTrkImgPoints.at<cv::Point2f>(i, 0).y); // 17 chars
			}

			for (int i = 0; i < num_track_points; i++) {
				if (fBigTable) fprintf(fBigTable, "%25.17e ", newTrkObjPoints.at<cv::Point3d>(i, 0).x); // 26 chars
				if (fBigTable) fprintf(fBigTable, "%25.17e ", newTrkObjPoints.at<cv::Point3d>(i, 0).y); // 26 chars
				if (fBigTable) fprintf(fBigTable, "%25.17e ", newTrkObjPoints.at<cv::Point3d>(i, 0).z); // 26 chars
			}
			if (fBigTable) fprintf(fBigTable, "\n");

		}//end of 31
	}// end of tracking loop

	if (fBigTable) fclose(fBigTable);

	cv::destroyAllWindows();
	return 0;

}