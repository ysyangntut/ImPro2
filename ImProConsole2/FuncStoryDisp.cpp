#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "trackings.h"

using namespace std;

cv::Point2f selectROI_zoom(cv::Mat img, int nLevel)
{
	cv::Point2f pick = cv::Point2f(0.f, 0.f);
	cv::Mat imgSelected = img;
	cv::Rect roi;
	float fac = 4.0f;
	float fac_x = 1.0f, fac_y = 1.0f;
	float ori_x = 0.0f, ori_y = 0.0f;
	for (int iLevel = 0; iLevel < nLevel; iLevel++)
	{
		roi = cv::selectROI(imgSelected, true, false);
		pick.x = ori_x + (roi.x + roi.width * 0.5f) / fac_x - .5f;
		pick.y = ori_y + (roi.y + roi.height * 0.5f) / fac_y - .5f;
		imgSelected(roi).copyTo(imgSelected);
		cv::resize(imgSelected, imgSelected, cv::Size(0, 0), fac, fac, cv::INTER_NEAREST);
		ori_x += roi.x / fac_x;
		ori_y += roi.y / fac_y;
		fac_x *= fac;
		fac_y *= fac;
	}
	return pick;
}

cv::Point2f test_selectROI_zoom()
{
	cv::Mat img = cv::Mat::zeros(300, 300, CV_8U); 
	cv::Mat iwhite = cv::Mat::ones(150, 150, CV_8U) * 200; 
	iwhite.copyTo(img(cv::Rect(0, 0, 150, 150)));
	iwhite.copyTo(img(cv::Rect(150, 150, 150, 150)));
	cv::Point2f pick = selectROI_zoom(img, 3);
	cout << pick << endl;
	return pick;
}

int test_estimateStoryDisp(); 
int estimateStoryDisp(
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

int FuncStoryDisp(int argc, char ** argv)
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

	// Step 1: Get video source: 
	cv::VideoCapture vid;
	while (true) {
		cout << "# Enter video source (the string for cv::VideoCapture): \n";
		string vidName = readStringFromCin();
		vid.open(vidName);
		if (vid.isOpened() == true) break;
		cerr << "# Cannot open " << vidName << ". Try again.\n";
	}
	// print video properties
	cout << "CAP_PROP_FRAME_WIDTH : " << vid.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
	cout << "CAP_PROP_FRAME_HEIGHT : " << vid.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "CAP_PROP_FPS : " << vid.get(cv::CAP_PROP_FPS) << endl;
	cout << "CAP_PROP_FRAME_COUNT : " << vid.get(cv::CAP_PROP_FRAME_COUNT) << endl;

	// get initial image
	vid >> imgInit; 
	if (imgInit.channels() > 1) 
		cv::cvtColor(imgInit, imgInit, cv::COLOR_BGR2GRAY); 
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

	// Step 3: Ask user to define tracking points (supposed to move within constrained surface) both 2D (image) and 3D (world).
	cout << "# Enter number of tracking points (movable points)): \n";
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

	cout << "# Define tortional center (xc yc zc): \n";
	tortionalCenter.x = readDoubleFromIstream(std::cin);
	tortionalCenter.y = readDoubleFromIstream(std::cin);
	tortionalCenter.z = readDoubleFromIstream(std::cin);
	
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

	// Step 5: start the tracking loop
	cv::Mat imgCurr, imgTmpl;
//	cv::VideoCapture vid("v4l2src ! video/x-raw,format=NV12,width=1640,height=1232 ! videoconvert ! appsink");
	if (vid.isOpened() == false) {
		cerr << "Cannot open camera.\n";
		return -1;
	}
	else
	{
		printf("Camera is opened.\n");
		while (true)
		{
			vid >> imgCurr;
			if (imgCurr.rows <= 0 || imgCurr.cols <= 0) break;
			if (imgCurr.channels() > 1)
				cv::cvtColor(imgCurr, imgCurr, cv::COLOR_BGR2GRAY);
			// plot points on image
//           drawPointsOnImage(imgCurr, matFixedPoints2f, "o", 16, 8);
//           drawPointsOnImage(imgCurr, matTrackPoints2f, "X",  4, 2);
			imshow_resize("Camera", imgCurr, (double) 1024. / imgCurr.cols); 
//			cv::imshow("Camera", imgCurr);
			int ikey = cv::waitKey(3);
			if (ikey == 27 || ikey == 32)
				break;
		}
		//        vid.release();
		cv::destroyWindow("Camera");
		//        return 0;
	}

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

	vid.set(cv::CAP_PROP_POS_FRAMES, 0.0); 
	for (size_t iStep = 0; true; iStep++)
	{
		// get image
		vid >> imgCurr;
		if (imgCurr.rows <= 0 || imgCurr.cols <= 0) break;
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
			int n = (int) fixedPoints2f_Curr.size(); 
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
		cout << trackPoints2f_Tmpl[0] << " " << trackPoints2f_Curr[0];
		if (trackMethod == 1)
		{
			int n = (int) trackPoints2f_Curr.size();
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

		cout << trackPoints2f_Curr[0] << endl; 

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
		int retVal = 
		estimateStoryDisp(
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

		printf("Floor disp: %16.4f %16.4f    Tortion (degrees): %16.4f \n",
			newDisp.at<double>(0, 0), newDisp.at<double>(1, 0), newDisp.at<double>(2, 0) /*  * 180. / 3.14159265358979323846 */ ); 

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
	} // end of tracking loop

	cv::destroyAllWindows();
	return 0;
}