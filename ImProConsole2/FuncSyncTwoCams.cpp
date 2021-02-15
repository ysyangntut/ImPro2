#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "sync.h"

using namespace std; 

// Given two files of histories of image points positions, taken by different cameras, 
// with a selected point at each file (which supposed to be the same point in the real world)
// this function estimates the time lag of the second file. 

// General steps:
// Step 1: Get data from user (two files, point numbers, guessed lag, search range (tried lag), position and range of correlation.)
// Step 2: Estimate the lag of camera 2 by using cross correlation
// Step 3: 

const cv::String keys =
"{help h usage ?     |      | print this message   }"
"{ifphist   ifphist  |      | input file of points history of camera 1.}"
"{ifphist2  ifphist2 |      | input file of points history of camera 2.}"
"{point1    point1   |      | point number (1-based) to match in camera 1}"
"{point2    point2   |      | point number (1-based) to match in camera 2. Supposed to be the same point with point1.}"
"{type      type     |      | (1) x in image, (2) y in image, (3) sqrt(x^2+y^2), (4) sqrt(vx^2+vy^2). (<=0 for default, sqrt(vx^2+vy^2)}"
"{guessLag  guesslag |      | guessed time lag of camera 2, unit of time step. Positive means camera 2 starts later. (<=0 for default, zero)}"
"{searcRng  searcRng |      | search range of time step. E.g., 1 means from (guessLag - 1) to (guessLag + 1). (<=0 for default, 0.05 of time length)}"
"{winCenter winCenter|      | center step (0-based) of cross correlation range. Normally at a peak of camera 1. (<=0 for default, center of time series)}"
"{winSize   winSize  |      | window size of cross correlation. (<=0 for default, half of time length)}"
"{precsn    precsn   |      | precision of time lag (<=0 for default, 0.01}"
"{ofphist2  ofphist2 |      | output file of points history of camera 2.}"; 

int FuncSyncTwoCams(int argc, char ** argv) {
	string ifphist1;     // input file (xml) of points history of camera 1. (vector<vector<Point2f>>) 
	string ifphist2;	 // input file (xml) of points history of camera 2. (vector<vector<Point2f>>)
	string ofphist2;	 // output file (xml) of new synchronized points history of camera 2. (vector<vector<Point2f>>)
	int    point1 = -1; // point number (1-based) to match in camera 1.
	int    point2 = -1; // point number (1-based) to match in camera 2. Supposed to be the same point with point1. 
	int    type = -1;   //  (1) x in image, (2) y in image, (3) sqrt(x^2+y^2), (4) sqrt(vx^2+vy^2). (<=0 for default, sqrt(vx^2+vy^2)
	int    guessLag = -1; // guessed time lag of camera 2, unit of time step. Positive means camera 2 starts later.
	int    searcRng = -1; // search range of time step. E.g., 1 means from (guessLag - 1) to (guessLag + 1).
	int    winCenter = -1; // center step (0-based) of cross correlation range. Normally at a peak of camera 1.
	int    winSize = -1;   // window size of cross correlation.
	float  precsn = -1;    // precision of time step when doing cross correlation

	int nStep1; // number of steps of cam 1 file
	int nPoint1; // number of points of cam 1 file
	int nStep2; // number of steps of cam 2 file
	int nPoint2; // number of points of cam 2 file
	vector<vector<cv::Point2f> > xi1; // image points history of camera 1
	vector<vector<cv::Point2f> > xi2; // image points history of camera 2
	vector<vector<cv::Point2f> > xic; // synchronized image points history of camera 2

	cv::CommandLineParser parser(argc, argv, keys);

	if (parser.has("help") || argc <= 1) {
		parser.printMessage();
//		std::cout << "\nUsage: \n";
	}

	if (parser.has("ifphist"))
		ifphist1 = parser.get<string>("ifphist"); 
	if (ifphist1.length() <= 0) {
		cout << "Input file (xml) of points history of camera 1 (format: vector<vector<Point2f>>, dim[nSteps][nPoints]) ('g' for gui file dialog):\n";
		ifphist1 = readStringLineFromCin();
		if (ifphist1.length() == 1 && ifphist1[0] == 'g')
			ifphist1 = uigetfile(); 
		cout << ifphist1 << endl;
	}
	cv::FileStorage ifsPhist1(ifphist1, cv::FileStorage::READ);
	ifsPhist1["numSteps"] >> nStep1; 
	ifsPhist1["numPoints"] >> nPoint1;
	cout << "Got " << nStep1 << " steps of " << nPoint1 << " points from camera 1 file.\n"; cout.flush(); 
	ifsPhist1["VecVecPoint2f"] >> xi1;

	if (parser.has("ifphist2"))
		ifphist2 = parser.get<string>("ifphist2");
	if (ifphist2.length() <= 0) {
		cout << "Input file (xml) of points history of camera 2 (format: vector<vector<Point2f>>, dim[nSteps][nPoints]) ('g' for gui file dialog):\n";
		ifphist2 = readStringLineFromCin();
		if (ifphist2.length() == 1 && ifphist2[0] == 'g')
			ifphist2 = uigetfile();
		cout << ifphist2 << endl;
	}
	cv::FileStorage ifsPhist2(ifphist2, cv::FileStorage::READ);
	ifsPhist2["numSteps"] >> nStep2;
	ifsPhist2["numPoints"] >> nPoint2;
	cout << "Got " << nStep2 << " steps of " << nPoint2 << " points from camera 2 file.\n"; cout.flush();
	ifsPhist2["VecVecPoint2f"] >> xi2;

	//
	if (parser.has("point1"))
		point1 = parser.get<int>("point1");
	else {
		cout << "Point number (1-based) to match in camera 1:\n";
		point1 = readIntFromCin();
	}
	if (parser.has("point2"))
		point2 = parser.get<int>("point2");
	else {
		cout << "Point number (1-based) to match in camera 2:\n";
		point2 = readIntFromCin();
	}
	if (parser.has("type"))
		type = parser.get<int>("type");
	else {
		cout << "Sync type: (1) x in image, (2) y in image, (3) sqrt(x^2+y^2), (4) sqrt(vx^2+vy^2).(<=0 for default sqrt(vx^2+vy^2):\n";
		type = (int)readIntFromCin();
	}
	if (parser.has("guessLag"))
		guessLag = parser.get<int>("guessLag");
	else {
		cout << "Guessed time lag of camera 2, unit of time step. Positive means camera 2 starts later:\n";
		guessLag = (int)readIntFromCin(); 
	}
	if (parser.has("searcRng"))
		searcRng = parser.get<int>("searcRng"); 
	else {
		cout << "Search range of time step. E.g., 1 means from (guessLag - 1) to (guessLag + 1):\n";
		searcRng = (int)readDoubleFromCin();
	}
	if (parser.has("winCenter"))
		winCenter = parser.get<int>("winCenter");
	else {
		cout << "Center step (0-based) of cross correlation range. Normally at a peak of camera 1:\n";
		winCenter = readIntFromCin();
	}
	if (parser.has("winSize"))
		winSize = parser.get<int>("winSize");
	else {
		cout << "Window size of cross correlation:\n";
		winSize = readIntFromCin();
	}
	if (parser.has("precsn"))
		precsn = parser.get<float>("precsn"); 
	else {
		cout << "Precision of time step when doing cross correlation:\n";
		precsn = (float) readDoubleFromCin();
	}

	// ask if user wants to apply the synchronization
	int applySync = 1;
	if (parser.has("ofphist2") == false) {
		cout << "Do you want to apply the synchronization by generating new file of camera 2 (0:no, else:yes):\n";
		applySync = (int)readIntFromCin();
	}

	// ask file
	if (parser.has("ofphist2") == false)
		ofphist2 = parser.get<string>("ofphist2");
	if (ofphist2.length() <= 0) {
		cout << "Output file (xml) of new synchronized points history of camera 2 (format: vector<vector<Point2f>>, dim[nSteps][nPoints]) ('g' for gui file dialog):\n";
		ofphist2 = readStringLineFromCin();
		if (ofphist2.length() == 1 && ofphist2[0] == 'g')
			ofphist2 = uiputfile();
		cout << ofphist2 << endl;
	}

	// start finding time lag and doing synchronization 

	float lag, coef; 
	cv::Mat series1; // time series of a point of camera 1
	cv::Mat series2; // time series of a point of camera 2
	cv::Mat s2sync;  // synchronized series 2
	cv::Mat xcorr_x, xcorr_y; // cross correlation function (x:lag-axis, y:coef-axis) 
	if (type == 1) { // sync type: (1) x in image,
		// extract data from points history
		series1 = cv::Mat(1, nStep1, CV_32F); // time series of a point of camera 1, (1, nStep, CV_32F) 
		series2 = cv::Mat(1, nStep2, CV_32F); // time series of a point of camera 2, (1, nStep, CV_32F) 
		s2sync = cv::Mat(1, nStep2, CV_32F);  // synchronized series 2
		for (int i = 0; i < nStep1; i++)
			series1.at<float>(0, i) = xi1[i][point1 - 1].x;
		for (int i = 0; i < nStep2; i++)
			series2.at<float>(0, i) = xi2[i][point2 - 1].x;
		// run sync.
		coef = syncTwoSeries(series1, series2, lag, s2sync, xcorr_x, xcorr_y, 
			guessLag, searcRng, winCenter, winSize, precsn, false, 
			directoryOfFullPathFile(ofphist2));
	}
	if (type == 2) { // sync type: (1) y in image,
		// extract data from points history
		series1 = cv::Mat(1, nStep1, CV_32F); // time series of a point of camera 1, (1, nStep, CV_32F) 
		series2 = cv::Mat(1, nStep2, CV_32F); // time series of a point of camera 2, (1, nStep, CV_32F) 
		s2sync = cv::Mat(1, nStep2, CV_32F);  // synchronized series 2
		for (int i = 0; i < nStep1; i++)
			series1.at<float>(0, i) = xi1[i][point1 - 1].y;
		for (int i = 0; i < nStep2; i++)
			series2.at<float>(0, i) = xi2[i][point2 - 1].y;
		// run sync.
		coef = syncTwoSeries(series1, series2, lag, s2sync, xcorr_x, xcorr_y, 
			guessLag, searcRng, winCenter, winSize, precsn, false,
			directoryOfFullPathFile(ofphist2));
	}
	if (type == 3) { // sync type: (3) sqrt(x^2+y^2),
		// extract data from points history
		series1 = cv::Mat(1, nStep1, CV_32FC2); // time series of a point of camera 1, (1, nStep, CV_32F) 
		series2 = cv::Mat(1, nStep2, CV_32FC2); // time series of a point of camera 2, (1, nStep, CV_32F) 
		s2sync = cv::Mat(1, nStep2, CV_32FC2);  // synchronized series 2
		for (int i = 0; i < nStep1; i++)
			series1.at<cv::Point2f>(0, i) = xi1[i][point1 - 1];
		for (int i = 0; i < nStep2; i++)
			series2.at<cv::Point2f>(0, i) = xi2[i][point2 - 1];
		// run sync.
		coef = syncTwoVector2dSeries(series1, series2, lag, s2sync, xcorr_x, xcorr_y,
			guessLag, searcRng, winCenter, winSize, precsn, 
			directoryOfFullPathFile(ofphist2));

	}
	if (type == 4 || type <= 0) { // sync type: (4) sqrt(vx^2+vy^2).
		// extract data from points history
		series1 = cv::Mat(1, nStep1, CV_32FC2); // time series of a point of camera 1, (1, nStep, CV_32F) 
		series2 = cv::Mat(1, nStep2, CV_32FC2); // time series of a point of camera 2, (1, nStep, CV_32F) 
		s2sync = cv::Mat(1, nStep2, CV_32FC2);  // synchronized series 2
		for (int i = 0; i < nStep1; i++)
			series1.at<cv::Point2f>(0, i) = xi1[i][point1 - 1];
		for (int i = 0; i < nStep2; i++)
			series2.at<cv::Point2f>(0, i) = xi2[i][point2 - 1];
		// run sync.
		coef = syncTwoVector2dSeriesByVelocity(series1, series2, lag, s2sync, xcorr_x, xcorr_y,
			guessLag, searcRng, winCenter, winSize, precsn, 
			directoryOfFullPathFile(ofphist2));
	}

	cout << "Sync: Time lag of camera 2 is " << lag << " steps." << endl;
	cout << "Matched coefficient is " << coef << endl;


	// apply synchronization 
	if (applySync == 0)
		return 0; 

	cv::FileStorage ofsPhist2(ofphist2, cv::FileStorage::WRITE);
	ofsPhist2 << "numSteps" << nStep2;
	ofsPhist2 << "numPoints" << nPoint2;
	xic = xi2;
	for (int iPoint = 0; iPoint < nPoint2; iPoint++) {
		cv::Mat t0(1, nStep2, CV_32F); 
		cv::Mat tc(1, nStep2, CV_32F); 
		// synchronize channel x
		for (int iStep = 0; iStep < nStep2; iStep++) 
			t0.at<float>(0, iStep) = xi2[iStep][iPoint].x;
		applySynchronization(t0, tc, lag);
		for (int iStep = 0; iStep < nStep2; iStep++)
			xic[iStep][iPoint].x = tc.at<float>(0, iStep);
		// synchronize channel y
		for (int iStep = 0; iStep < nStep2; iStep++)
			t0.at<float>(0, iStep) = xi2[iStep][iPoint].y;
		applySynchronization(t0, tc, lag);
		for (int iStep = 0; iStep < nStep2; iStep++)
			xic[iStep][iPoint].y = tc.at<float>(0, iStep);
	}
	ofsPhist2 << "VecVecPoint2f" << xic;
	ofsPhist2.release();

	return 0;
}