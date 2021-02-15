#define _CRT_SECURE_NO_WARNINGS

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

cv::Point2f selectROI_zoom(cv::Mat img, int nLevel);

cv::Point2f test_selectROI_zoom();

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
	cv::Mat& camRot,                // camera rotational movement (3, 1, CV_64F)
	cv::Mat& newDisp,               // displacement of story (ux, uy, tortion) (3, 1, CV_64F)
	cv::Mat& newTrkObjPoints,       // moved tracking points in world coord. vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::Mat& projNewRefImgPoints,   // new projection reference points in image coord.
	cv::Mat& projNewTrkImgPoints    // new projection tracking points in image coord.
);

int FuncStoryDispV3(int argc, char** argv)
{
	// video source, resolution
	std::string vidName;
	cv::VideoCapture vid;
	int vw = 0, vh = 0; 
	// camera parameters
	string fnameIntrinsic, fnameExtrinsic;
	cv::FileStorage ifs;
	cv::Mat cmat, dvec, rvec, tvec, r3, r4, r4inv, campos, camdir;
	// reference points & tracking points
	int num_fixed_points, num_track_points;
	vector<cv::Point2f> fixedPoints2f, trackPoints2f;
	vector<cv::Point3d> fixedPoints3d, trackPoints3d;
	cv::Mat matFixedPoints2f, matFixedPoints3d, matTrackPoints2f, matTrackPoints3d;
	cv::Point3d tortionalCenter(0, 0, 0);
	// tracking parameters
	int trackMethod; // tracking method. 1:Template match (with rotation and pyramid), 2:Ecc, 3:Optical flow (sparse, LK Pyr.)
	int trackUpdateFreq; // tracking template updating frequency. 0:Not updating (using initial template). 1:Update every frame. 2:Update every other frame. Etc.
	int trackPredictionMethod; // tracking prediction method. 0:Previous point. 1:Linear approx. 2:2nd-order approx.
	int winSize; // window size of template size 
	// files input/output
	string projectPath("/home/pi/Desktop/picture/");
	projectPath = appendSlashOrBackslashAfterDirectoryIfNecessary(projectPath);
	string fnameBigTable(projectPath + "bigTable.txt");
	string fnameCfg(projectPath + "storyDispV3.cfg"); 
	cv::FileStorage fsCfg; 
	bool fsCfgDataOk = false, useCfgData = false;
	FILE* fBigTable;

	// Load data from config file (optionally)
	fsCfg.open(fnameCfg, cv::FileStorage::READ); 
	if (fsCfg.isOpened()) {
		fsCfgDataOk = true;
		// load config data
		fsCfg["vidName"] >> vidName; 
		fsCfg["videoWidth"] >> vw;
		fsCfg["videoHeight"] >> vh;
		fsCfg["cmat"] >> cmat;
		fsCfg["dvec"] >> dvec; 
		fsCfg["rvec"] >> rvec;
		fsCfg["tvec"] >> tvec; 
		fsCfg["refPoints2f"] >> fixedPoints2f;
		fsCfg["refPoints3d"] >> fixedPoints3d;
		fsCfg["trkPoints2f"] >> trackPoints2f;
		fsCfg["trkPoints3d"] >> trackPoints3d;
		fsCfg["tortionalCenter"] >> tortionalCenter;
		fsCfg["trackMethod"] >> trackMethod;
		fsCfg["trackUpdateFreq"] >> trackUpdateFreq;
		fsCfg["trackPredictionMethod"] >> trackPredictionMethod;
		fsCfg["winSize"] >> winSize; 
		fsCfg.release();
		// data check
		if (vidName.length() <= 1) fsCfgDataOk = false;
		if (vw <= 1) fsCfgDataOk = false;
		if (vh <= 1) fsCfgDataOk = false;
		if (cmat.rows != 3 || cmat.cols != 3) fsCfgDataOk = false;
		if (dvec.rows != 1 && dvec.cols != 1) fsCfgDataOk = false;
		if (dvec.rows * dvec.cols < 4) fsCfgDataOk = false;
		if (rvec.rows == 1 && rvec.cols == 3) rvec = rvec.t(); // transpose to 3x1
		if (rvec.rows == 3 && rvec.cols == 3) cv::Rodrigues(rvec, rvec); // convert to 3x1
		if (rvec.rows != 3 || rvec.cols != 1) fsCfgDataOk = false;
		if (tvec.rows == 1 && tvec.cols == 3) tvec = tvec.t(); // transpose to 3x1
		if (tvec.rows != 3 || tvec.cols != 1) fsCfgDataOk = false;
		r4 = cv::Mat::eye(4, 4, CV_64F); 
		cv::Rodrigues(rvec, r4(cv::Rect(0, 0, 3, 3)));
		tvec.copyTo(r4(cv::Rect(3, 0, 1, 3)));
		if (fixedPoints2f.size() < 1) fsCfgDataOk = false;
		if (fixedPoints3d.size() < 1) fsCfgDataOk = false;
		if (trackPoints2f.size() < 1) fsCfgDataOk = false;
		if (trackPoints3d.size() < 1) fsCfgDataOk = false;
	}
	if (fsCfgDataOk) {
		std::cout << "# The default values defined in " << fnameCfg << "\nis:\n";
		std::cout << "# vidName:\n" << vidName << endl;
		std::cout << "# videoWidth:\n" << vw << endl;
		std::cout << "# videoHeight:\n" << vh << endl;
		std::cout << "# cmat:\n" << cmat << endl;
		std::cout << "# dvec:\n" << dvec << endl;
		std::cout << "# rvec:\n" << rvec << endl;
		std::cout << "# tvec:\n" << tvec << endl;
		std::cout << "# refPoints2f:\n" << fixedPoints2f << endl;
		std::cout << "# refPoints3d:\n" << fixedPoints3d << endl;
		std::cout << "# trkPoints2f:\n" << trackPoints2f << endl;
		std::cout << "# trkPoints3d:\n" << trackPoints3d << endl;
		std::cout << "# trackMethod:\n" << trackMethod << endl;
		std::cout << "# trackUpdateFreq:\n" << trackUpdateFreq << endl;
		std::cout << "# TrackPredictionMethod:\n" << trackPredictionMethod << endl;
		std::cout << "# winSize:\n" << winSize << endl;
		std::cout << "# Do you want to use the above values/settings? (0: No, else: Yes)\n";
		useCfgData = (readIntFromCin() != 0); 
	}
	if (fsCfgDataOk == false || useCfgData == false) 
	{
		vidName = std::string(""); 
		vw = -1;
		vh = -1;
		cmat.release();
		dvec.release();
		rvec.release();
		tvec.release();
		fixedPoints2f.clear();
		fixedPoints3d.clear();
		num_fixed_points = 0;
		trackPoints2f.clear();
		trackPoints3d.clear();
		num_track_points = 0;
		tortionalCenter = cv::Point3d(0, 0, 0);
		trackMethod = -1;
		trackUpdateFreq = -1;
		trackPredictionMethod = -1;
		winSize = -1;
	}

	string fnameImgInit("");
	cv::Mat imgInit;

	// Step 0: Ready to output file
	int ferr;
	ferr = fopen_s(&fBigTable, fnameBigTable.c_str(), "w");
	if (fBigTable == NULL || ferr != 0) {
		printf("#Cannot open file: %s \n", fnameBigTable.c_str());
		fBigTable = NULL;
	}

	// Step 1: Get video source: 
	while (vidName.length() <= 1) {
		std::cout << "# Enter video source (the string for cv::VideoCapture): \n";
		vidName = readStringFromCin();
		if (vidName.length() > 1)
			vid.open(vidName);
		else
			vid.open(vidName[0] - '0');
		if (vid.isOpened() == true) break;
		cerr << "# Cannot open " << vidName << ". Try again.\n";
	}
	// print video properties
	std::cout << "CAP_PROP_FRAME_WIDTH : " << vid.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
	std::cout << "CAP_PROP_FRAME_HEIGHT : " << vid.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
	std::cout << "CAP_PROP_FPS : " << vid.get(cv::CAP_PROP_FPS) << endl;
	std::cout << "CAP_PROP_FRAME_COUNT : " << vid.get(cv::CAP_PROP_FRAME_COUNT) << endl;

	// set camera resolution
	while (vw <= 1 || vh <= 1) {
		// turn off the camera
		if (vid.isOpened() == true)
			//vid.release();
		// ask user to input resolution
			std::cout << "# Enter video resolution (width height):  \n";
		vw = readIntFromIstream(std::cin);
		vh = readIntFromIstream(std::cin);

		// try to setup the resolution
		vid.set(cv::CAP_PROP_FRAME_WIDTH, (double)vw);
		vid.set(cv::CAP_PROP_FRAME_HEIGHT, (double)vh);
		// check if it works
		if (vid.isOpened() == false)
			if (vidName.length() > 1)
				vid.open(vidName);
			else
				vid.open(vidName[0] - '0');
		// print video properties
		std::cout << "CAP_PROP_FRAME_WIDTH : " << vid.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
		std::cout << "CAP_PROP_FRAME_HEIGHT : " << vid.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
		std::cout << "CAP_PROP_FPS : " << vid.get(cv::CAP_PROP_FPS) << endl;
		std::cout << "CAP_PROP_FRAME_COUNT : " << vid.get(cv::CAP_PROP_FRAME_COUNT) << endl;
		// ask user if it is okay
		std::cout << "# Does it work well? (0: re-set again. 1: go ahead.): \n";
		int go_ahead = readIntFromIstream(std::cin);
		if (go_ahead == 0) continue;
		else break;
	}
	// output to bigTable
	if (fBigTable) fprintf(fBigTable, "Video_source: %s\n", vidName.c_str());
	if (fBigTable) fprintf(fBigTable, "CAP_PROP_FRAME_WIDTH:  %12.4f\n", vid.get(cv::CAP_PROP_FRAME_WIDTH));
	if (fBigTable) fprintf(fBigTable, "CAP_PROP_FRAME_HEIGHT: %12.4f\n", vid.get(cv::CAP_PROP_FRAME_HEIGHT));
	if (fBigTable) fprintf(fBigTable, "CAP_PROP_FPS: %12.4f\n", vid.get(cv::CAP_PROP_FPS));
	if (fBigTable) fprintf(fBigTable, "CAP_PROP_FRAME_COUNT: %12.4f\n", vid.get(cv::CAP_PROP_FRAME_COUNT));

	// get initial image
	vid >> imgInit;
	if (imgInit.channels() > 1)
		cv::cvtColor(imgInit, imgInit, cv::COLOR_BGR2GRAY);
	imshow_resize("Initial image", imgInit, (double)1024. / imgInit.cols);
	cv::waitKey(1000);
	cv::destroyWindow("Initial image");

	// Step 1: Get camera parameters

	//	string fnameIntrinsic = "/home/pi/Downloads/raspi_cam1_intrinsic.xml";
	//	string fnameExtrinsic = "/home/pi/Downloads/0503_rasberryCam1_8Points_extrinsic.xml";
	while (cmat.rows != 3 || cmat.cols != 3) {
		std::cout << "# Enter camera intrinsic file:\n";
		fnameIntrinsic = readStringLineFromCin();
		ifs.open(fnameIntrinsic, cv::FileStorage::READ);
		ifs["cameraMatrix"] >> cmat;
		ifs["distortionVector"] >> dvec;
		std::cout << "Cmat:\n" << cmat << endl << "Dvec:\n" << dvec << endl;
		ifs.release();
	}
	while (rvec.rows != 3 || rvec.cols != 1) {
		std::cout << "# Enter camera extrinsic file:\n";
		fnameExtrinsic = readStringLineFromCin();
		ifs.open(fnameExtrinsic, cv::FileStorage::READ);
		//	ifs["rvec"] >> rvec;
		//	ifs["tvec"] >> tvec;
		ifs["R4"] >> r4;
		if (r4.rows <= 0 || r4.cols <= 0)
			ifs["r4"] >> r4;
		std::cout << "r4: \n" << r4 << endl;
	}
	r4(cv::Rect(0, 0, 3, 3)).copyTo(r3);
	r4(cv::Rect(3, 0, 1, 3)).copyTo(tvec);
	cv::Rodrigues(r3, rvec);
	r4inv = r4.inv();
	r4inv(cv::Rect(3, 0, 1, 3)).copyTo(campos);
	r4inv(cv::Rect(2, 0, 1, 3)).copyTo(camdir);
	std::cout << "rvec:\n" << rvec << "\ntvec:\n" << tvec << endl;
	std::cout << "camPos:\n" << campos << "\ncamdir:\n" << camdir << endl;
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
	while (num_fixed_points <= 0 || 
		fixedPoints2f.size() != num_fixed_points || 
		fixedPoints3d.size() != num_fixed_points) 
	{
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
	} // end of while fixPoints2f/3d is not ready.

	// output to big table
	if (fBigTable) fprintf(fBigTable, "numRefPoints: %4d\n", num_fixed_points);
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "refPoints2f[ %4d ]: %16.9e %16.9e   refPoints3d[ %4d ]: %25.17e %25.17e %25.17e\n",
			i, fixedPoints2f[i].x, fixedPoints2f[i].y,
			i, fixedPoints3d[i].x, fixedPoints3d[i].y, fixedPoints3d[i].z);
	}

	// Step 3: Ask user to define tracking points (supposed to move within constrained surface) both 2D (image) and 3D (world).
	while (num_track_points <= 0 ||
		trackPoints2f.size() != num_track_points ||
		trackPoints3d.size() != num_track_points)
	{
		std::cout << "# Enter number of tracking points (movable points)): \n";
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
	} // end of while trackPoints2f/3d is not ready.

	// output to big table
	if (fBigTable) fprintf(fBigTable, "numTrkPoints: %4d\n", num_fixed_points);
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "trackPoints2f[ %4d ]: %16.9e %16.9e   trackPoints3d[ %4d ]: %25.17e %25.17e %25.17e\n",
			i, trackPoints2f[i].x, trackPoints2f[i].y,
			i, trackPoints3d[i].x, trackPoints3d[i].y, trackPoints3d[i].z);
	}

	if (useCfgData == false)
	{
		std::cout << "# Define tortional center (xc yc zc): \n";
		tortionalCenter.x = readDoubleFromIstream(std::cin);
		tortionalCenter.y = readDoubleFromIstream(std::cin);
		tortionalCenter.z = readDoubleFromIstream(std::cin);
	}

	// output to big table
	if (fBigTable) fprintf(fBigTable, "tortionalCenter: %25.17e %25.17e %25.17e\n",
		tortionalCenter.x, tortionalCenter.y, tortionalCenter.z);

	matFixedPoints2f = cv::Mat(fixedPoints2f);
	matFixedPoints3d = cv::Mat(fixedPoints3d);
	matTrackPoints2f = cv::Mat(trackPoints2f);
	matTrackPoints3d = cv::Mat(trackPoints3d);

	std::cout << "Fixed points 2f: \n" << fixedPoints2f << endl;
	std::cout << "Fixed points 3d: \n" << fixedPoints3d << endl;
	std::cout << "Tracking points 2f: \n" << trackPoints2f << endl;
	std::cout << "Tracking points 3d: \n" << trackPoints3d << endl;

	// Step 4: Select tracking method and options
	//   4.1: Tracking method: default, template match
	//   4.2: Target updating frequency (default 1, update each frame, 0 for not-updated and using initial template)
	//   4.3: Prediction method (default: 2-nd order)
	//
	while (trackMethod <= 0 || trackMethod >= 4) {
		std::cout << "# Select tracking method: (1)Template match, (2)Ecc, (3)Optical flow:\n";
		trackMethod = readIntFromIstream(std::cin, 1, 3);
		std::cout << "# Your selection is " << trackMethod << endl;
	}
	while (trackUpdateFreq < 0) {
		std::cout << "# Target updating frequency: (0)Not updating (N)Updating every N frames: \n";
		trackUpdateFreq = readIntFromIstream(std::cin, 0, 99999999);
		std::cout << "# Your selection is " << trackUpdateFreq << endl;
	}
	while (trackPredictionMethod < 0 || trackPredictionMethod >= 3)
	{
		std::cout << "# Prediction method: (0)Previous point, (1)Linear approx, (2)2nd-order approx.:\n";
		trackPredictionMethod = readIntFromIstream(std::cin, 0, 2);
		std::cout << "# Your selection is " << trackPredictionMethod << endl;
	}
	while (winSize <= 0)
	{
		int defaultWinSize = std::min(imgInit.cols, imgInit.rows) / 50;
		std::cout << "# Window size (or template size) (0 for 1/50 of min(image width, height), " << defaultWinSize << "):\n";
		winSize = readIntFromIstream(std::cin, 0, std::min(imgInit.cols, imgInit.rows) - 1);
		if (winSize <= 0) winSize = defaultWinSize;
		std::cout << "Window size is " << winSize << endl;
	}

	// write to config file
	if (useCfgData == false)
	{
		if (fsCfg.isOpened()) fsCfg.release();
		fsCfg.open(fnameCfg, cv::FileStorage::WRITE);
		fsCfg << "vidName" << vidName;
		fsCfg << "videoWidth" << vw;
		fsCfg << "videoHeight" << vh;
		fsCfg << "cmat" << cmat;
		fsCfg << "dvec" << dvec;
		fsCfg << "rvec" << rvec;
		fsCfg << "tvec" << tvec;
		fsCfg << "refPoints2f" << fixedPoints2f;
		fsCfg << "refPoints3d" << fixedPoints3d;
		fsCfg << "trkPoints2f" << trackPoints2f;
		fsCfg << "trkPoints3d" << trackPoints3d;
		fsCfg << "tortionalCenter" << tortionalCenter;
		fsCfg << "trackMethod" << trackMethod;
		fsCfg << "trackUpdateFreq" << trackUpdateFreq;
		fsCfg << "trackPreditionMethod" << trackPredictionMethod;
		fsCfg << "winSize" << winSize;
		fsCfg.release();
	}

	// output to big table
	if (fBigTable) fprintf(fBigTable, "trackMethod: %d\n", trackMethod);
	if (fBigTable) fprintf(fBigTable, "trackUpdateFreq: %d\n", trackUpdateFreq);
	if (fBigTable) fprintf(fBigTable, "trackPredictionMethod: %d\n", trackPredictionMethod);
	if (fBigTable) fprintf(fBigTable, "winSize: %d\n", winSize);

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
			imshow_resize("Camera", imgCurr, (double)1024. / imgCurr.cols);
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

	// output to big table (header line)
	if (fBigTable) fprintf(fBigTable, "   Step "); // 8 chars (including ending space)
	if (fBigTable) fprintf(fBigTable, "             Time "); // YYYYMMDDhhmmsssss (18 chars including ending space)
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "RefPnt2f[%4d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "RefPnt2f[%4d].y ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "TrkPnt2f[%4d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "TrkPnt2f[%4d].y ", i); // 17 chars
	}
	if (fBigTable) fprintf(fBigTable, "             FloorDisp.ux "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "             FloorDisp.uy "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "        FloorDisp.tortion "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.rx "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.ry "); // 26 chars
	if (fBigTable) fprintf(fBigTable, "                camRot.rz "); // 26 chars
	for (int i = 0; i < num_fixed_points; i++) {
		if (fBigTable) fprintf(fBigTable, "PrjRef2f[%4d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "PrjRef2f[%4d].y ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "PrjTrk2f[%4d].x ", i); // 17 chars
		if (fBigTable) fprintf(fBigTable, "PrjTrk2f[%4d].y ", i); // 17 chars
	}
	for (int i = 0; i < num_track_points; i++) {
		if (fBigTable) fprintf(fBigTable, "         CalTrk3d[%4d].x ", i); // 26 chars
		if (fBigTable) fprintf(fBigTable, "         CalTrk3d[%4d].y ", i); // 26 chars
		if (fBigTable) fprintf(fBigTable, "         CalTrk3d[%4d].z ", i); // 26 chars
	}
	if (fBigTable) fprintf(fBigTable, "\n");


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

	   // std::cout << trackPoints2f_Tmpl[0] <<"\t"<<trackPoints2f_Tmpl[1]<<"\t"<<trackPoints2f_Tmpl[2]<<"\t"<<trackPoints2f_Tmpl[3]
	   //      <<"\t"<<trackPoints2f_Tmpl[4]<<"\t"<<trackPoints2f_Tmpl[5]<<"\t"<<trackPoints2f_Tmpl[6];

	   // std::cout << trackPoints2f_Tmpl[0] << " c" << trackPoints2f_Curr[0]<<"d";
		std::cout << trackPoints2f_Curr[0] << "b";
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

		std::cout << trackPoints2f_Curr[0] << "c" << endl;

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

		struct tm* p;
		time_t t = time(0);
		p = localtime(&t);
		printf("Floor disp: %16.4f %16.4f    Tortion (degrees): %16.4f      ",
			newDisp.at<double>(0, 0), newDisp.at<double>(1, 0), newDisp.at<double>(2, 0) /*  * 180. / 3.14159265358979323846 */);
		ofstream outputfile;
		char str[100];
		sprintf(str, "/home/pi/Desktop/picture/data.txt");
		outputfile.open(str, ios::app);
		outputfile << iStep << "\t" << "time:\t" << 1900 + p->tm_year << 1 + p->tm_mon << p->tm_mday << p->tm_hour << p->tm_min << "\t" << p->tm_sec << "\t" << "Floor disp:\t" << newDisp.at<double>(0, 0) << "\t\t" << newDisp.at<double>(1, 0) << "\t" << "Tortion (degrees):" << "\t" << newDisp.at<double>(2, 0)
			<< endl;
		//   outputfile<<iStep<<"\t"<<"time:\t"<<1900+p->tm_year<<1+p->tm_mon<<p->tm_mday<<p->tm_hour<<p->tm_min<<p->tm_sec<<"\t"
		//            <<"Floor disp:\t"<<  newDisp.at<double>(0, 0)<<"\t"
		//            << newDisp.at<double>(1, 0)<<"\t"
		//            <<"Tortion (degrees):"<<"\t"
		//            <<newDisp.at<double>(2, 0)
		//            <<endl;
	   //sprintf(filename," %04d%02d%02d%02d%02d%02d_%05d.jpg",1900+p->tm_year, 1+p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec, ++count);




		outputfile.close();

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
		if (fBigTable) fprintf(fBigTable, "%7d ", (int) iStep);
		if (fBigTable) fprintf(fBigTable, "%04d%02d%02d%02d%02d%02d%03d ", 1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday,
			p->tm_hour, p->tm_min, p->tm_sec, 0);
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

	} // end of tracking loop

	if (fBigTable) fclose(fBigTable);

	cv::destroyAllWindows();
	return 0;
}
