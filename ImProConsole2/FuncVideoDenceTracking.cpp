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

using namespace std;

// Step 1: Read file sequence fseq
// Step 2: Set nyPoint, nxPoint, winSize, (and nImg if necessary)
// Step 3: Set output file (binary boFile, .xml.zip xoFile)
// Step 4: Start tracking (--> xyDat.at<Point2f>(iImg, iPoint).x/y, --> success.at<uint8>(iImg, iPoint))
// Step 5: Save file to boFile, xoFile
// Step 6: Visualization

int FuncVideoDenseTracking(int argc, char ** argv)
{
	// Variables
	FileSeq fsq, voFsq; 
	int nImg; 
	int nyPoint, nxPoint;
	cv::Size imgSize, winSize; 
	int optfMaxLevel;
	std::string uoFilename, voFilename;
	cv::FileStorage uoFile, voFile;
	cv::Mat imgInit;
	cv::Mat imgPrev; 
	cv::Mat imgCurr; 
	cv::Mat posInit; // initial positions of points. In Mat(nyPoint,nxPoint,CV_32FC2), iStepInit = 0
	cv::Mat posPrev; // previous positions of points. In Mat(nyPoint,nxPoint,CV_32FC2), iStepPrev = iStep - k, where k could be 1, 2, 3 ...
	cv::Mat posCurr; // current positions of points. In Mat(nyPoint,nxPoint,CV_32FC2), iStepCurr = iStep
	cv::Mat posPrv1; // the one before posCurr. In Mat(nyPoint,nxPoint,CV_32FC2), iStepPrv1 = iStep - 1
	cv::Mat posPrv2; // the one before posPrv1. In Mat(nyPoint,nxPoint,CV_32FC2), iStepPrv2 = iStep - 2
	cv::Mat posPrv3; // the one before posPrv2. In Mat(nyPoint,nxPoint,CV_32FC2), iStepPrv3 = iStep - 3
	cv::Mat velCurr;


	char buf[1000]; 
	float colormapMax;

	// Step 1: Read file sequence fseq
	fsq.setDirFilesByConsole(std::cin);
	printf("# There are %d images.\n", fsq.num_files());
	imgInit = cv::imread(fsq.fullPathOfFile(0), cv::IMREAD_GRAYSCALE); 
	if (imgInit.cols <= 0 || imgInit.rows <= 0)
	{
		printf("# The first image %s does not exist yet.\n", fsq.fullPathOfFile(0).c_str()); 
	}
	else {
		imgSize.width = imgInit.cols;
		imgSize.height = imgInit.rows;
		printf("# Image size is: %d (height) x %d (width).\n", imgInit.rows, imgInit.cols);
	}

	// Step 2: Set nyPoint, nxPoint, winSize, (and nImg if necessary)
	printf("# Enter number of tracking points along width (nxPoint) and height (nyPoint):\n");
	nxPoint = readIntFromIstream(std::cin);
	nyPoint = readIntFromIstream(std::cin);
	printf("# Enter window size (wx wy) (%d %d for non-overlapping):\n",
		(int) (imgInit.cols / nxPoint + .5), (int)(imgInit.rows / nyPoint + .5) );
	winSize.width = readIntFromIstream(std::cin);
	winSize.height = readIntFromIstream(std::cin);
	printf("# Enter maximum level for optical flow (default: 3):\n");
	optfMaxLevel = readIntFromIstream(std::cin);
	if (optfMaxLevel < 0) optfMaxLevel = 3;
	printf("# Enter velocity (pixels per frame time) for maximum colormap:\n");
	colormapMax = (float) readDoubleFromIstream(std::cin, 0.0, 1000);

	// Step 3: Set output file (binary boFile, .xml.zip xoFile)
	printf("# Enter full path of optical point output file (image coord., in pixels) (e.g., c:/temp/vidTracking_imgxy.xml.gz ):\n");
	uoFilename = readStringLineFromIstream(std::cin);
	printf("# Enter full path of optical velocity output file (delta image coord., pixels per frame) (e.g., c:/temp/vidTracking_vel.xml.gz ):\n");
	voFilename = readStringLineFromIstream(std::cin);
	printf("# Enter output file sequence of colormap images:\n");
	voFsq.setDirFilesByConsole();

	// Step 4: Start tracking (--> xyDat.at<Point2f>(iImg, iPoint).x/y, --> success.at<uint8>(iImg, iPoint))
	//  4.1  allocate memory
	nImg = fsq.num_files();
	posInit = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2); 
	posPrev = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2);
	posPrv1 = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2);
	posPrv2 = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2);
	posPrv3 = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2);
	posCurr = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2);
	velCurr = cv::Mat::zeros(1, nyPoint * nxPoint, CV_32FC2);
	//  4.2  initial points locations
	for (int i = 0; i < nyPoint; i++)
		for (int j = 0; j < nxPoint; j++)
		{
			posInit.at<cv::Point2f>(0, i * nxPoint + j).x = (float)(-0.5 + imgSize.width * (0.5 + j) / nxPoint);
			posInit.at<cv::Point2f>(0, i * nxPoint + j).y = (float)(-0.5 + imgSize.height * (0.5 + i) / nyPoint);
		}
	// 4.3  tracking settings
	cv::Mat prevImg, nextImg; 
	vector<uchar> optfStatus((size_t)(nyPoint * nxPoint), 0); 
	vector<float> optfErr((size_t)(nyPoint * nxPoint), 0.0f);

	// 4.4  start tracking (running iStep loop)
	//  4.4.1 open xml(.gz) files and write initial data 
	//  Prints to uoFile (image points of each point each frame)
	uoFile.open(uoFilename, cv::FileStorage::WRITE);
	if (uoFile.isOpened())
		printf("# XML file %s is opened successfully.\n", uoFilename.c_str());
	else
		printf("# Warning: XML file %s is not opened.\n", uoFilename.c_str());
	if (uoFile.isOpened()) uoFile << "videoWidth" << imgInit.cols;
	if (uoFile.isOpened()) uoFile << "videoHeight" << imgInit.rows;
	if (uoFile.isOpened()) uoFile << "nFrames" << nImg;
	if (uoFile.isOpened()) uoFile << "fps" << 29.97;
	if (uoFile.isOpened()) uoFile << "nxPoint" << nxPoint;
	if (uoFile.isOpened()) uoFile << "nyPoint" << nyPoint;
	if (uoFile.isOpened()) uoFile << "winSize" << winSize;
	snprintf(buf, 1000, "imgPoints%d", 0); 
	if (uoFile.isOpened()) uoFile << buf << posInit;
	//  Prints to voFile (increment of image points of each point each frame, i.e., image velocity of each point each frame)
	voFile.open(voFilename, cv::FileStorage::WRITE);
	if (voFile.isOpened())
		printf("XML file %s is opened successfully.\n", voFilename.c_str());
	else
		printf("Warning: XML file %s is not opened.\n", voFilename.c_str());
	if (voFile.isOpened()) voFile << "videoWidth" << imgInit.cols;
	if (voFile.isOpened()) voFile << "videoHeight" << imgInit.rows;
	if (voFile.isOpened()) voFile << "nFrames" << nImg;
	if (voFile.isOpened()) voFile << "fps" << 29.97;
	if (voFile.isOpened()) voFile << "nxPoint" << nxPoint;
	if (voFile.isOpened()) voFile << "nyPoint" << nyPoint;
	if (voFile.isOpened()) voFile << "winSize" << winSize;
	snprintf(buf, 1000, "imgVelocities%d", 0);
	if (voFile.isOpened()) voFile << buf << velCurr; // at this step, velCurr are zeros. 
	//  4.4.2 start the time loop 
	for (int iStep = 1; iStep < nImg; iStep++)
	{
		// 4.5 estimate the position --> posCurr
		if (iStep == 1) { // 0-order. Copy from initial step
			posInit.copyTo(posPrev);
			posInit.copyTo(posCurr);
			posInit.copyTo(posPrv1);
			posInit.copyTo(posPrv2);
			posInit.copyTo(posPrv3);
			if (imgInit.cols <= 0 || imgInit.rows <= 0)
				imgInit = cv::imread(fsq.fullPathOfFile(0), cv::IMREAD_GRAYSCALE);
		}
		else if (iStep == 2) { // 1st order. Linear extrapolation
			posCurr = 2 * posPrv1 - posPrv2; 
		}
		else if (iStep >= 3) { // 2nd order extrapolation 
			posCurr = posPrv1 - 3 * posPrv2 + 3 * posPrv3; 
		}

		// 4.6 Define which step (frame) is the previous one for optical flow 
		int iStepPrev = std::max(iStep - 1, 0);
		int iStepCurr = iStep; 

		// 4.7 Read images
		imgPrev = cv::imread(fsq.fullPathOfFile(iStepPrev), cv::IMREAD_GRAYSCALE);
		imgCurr = cv::imread(fsq.fullPathOfFile(iStepCurr), cv::IMREAD_GRAYSCALE);

		// 4.8 Show images
//		cv::destroyAllWindows(); 
		//imshow_resize("Prev", imgPrev, 0.25);
		//imshow_resize("Curr", imgCurr, 0.25);
		printf("Prev is step %d. Curr is step %d\n", iStepPrev, iStepCurr); 
		cv::waitKey(10);

		// 4.9 Run optical flow 
//		std::vector<cv::Point2f> vecPosPrev((cv::Point2f*) posPrev.data, (cv::Point2f*) posPrev.data + nyPoint * nxPoint); 
//		std::vector<cv::Point2f> vecPosCurr((cv::Point2f*) posCurr.data, (cv::Point2f*) posCurr.data + nyPoint * nxPoint);
		cv::calcOpticalFlowPyrLK(imgPrev, imgCurr, posPrev, posCurr,
			optfStatus, optfErr,
			winSize,
			optfMaxLevel); 

		// 4.10 Show result
		// calculate absolute velocity (pixels per frame time)
		for (int i = 0; i < nxPoint * nyPoint; i++) {
			velCurr.at<cv::Point2f>(0, i) = posCurr.at<cv::Point2f>(0, i) - posPrev.at<cv::Point2f>(0, i);
		}
		cv::Mat imgDrawCurr = imgCurr.clone();
		cv::Mat imgColormap = cv::Mat(imgCurr.size(), CV_8UC3); 
		cv::cvtColor(imgDrawCurr, imgDrawCurr, cv::COLOR_GRAY2BGR); 
		//drawPointsOnImage(imgDrawCurr, posCurr,
		//	std::string("o"),
		//	16 /* size */,
		//	4 /* thickness */,
		//	cv::Scalar(0), /* color */
		//	0.5f, /* transparency */
		//	-1, /* put text */
		//	0 /* shift */
		//);
		for (int i = 0; i < nyPoint; i++) {
			for (int j = 0; j < nxPoint; j++) {
				int ij = i * nxPoint + j;
				int dw = imgCurr.cols / nxPoint;
				int dh = imgCurr.rows / nyPoint; 
				float velx = velCurr.at<cv::Point2f>(ij).x; 
				float vely = velCurr.at<cv::Point2f>(ij).y;
				float vel = sqrt(velx * velx + vely * vely); 
				uchar graylevel = (uchar) (255 * (vel / colormapMax));
				cv::Point p1(j * dw, i * dh); 
				cv::Point p2((j + 1) * dw - 1, (i + 1) * dw - 1); 
				cv::rectangle(imgColormap, p1, p2,
					cv::Scalar(graylevel, graylevel, graylevel),
					cv::FILLED, cv::LineTypes::LINE_8, 0 /*shift*/); 
			}
		}
		cv::applyColorMap(imgColormap, imgColormap, cv::COLORMAP_JET); 
		imgDrawCurr = 0.2 * imgDrawCurr + 0.8 * imgColormap;
		imshow_resize("Curr", imgDrawCurr, 0.25);
		cv::waitKey(10);

		// write to colormap image
		cv::imwrite(voFsq.fullPathOfFile(iStep), imgDrawCurr);

		// Step 5: Save file to boFile, xoFile
		snprintf(buf, 1000, "imgPoints%d", iStep);
		if (uoFile.isOpened()) uoFile << buf << posCurr;
		snprintf(buf, 1000, "imgVelocities%d", iStep);
		if (voFile.isOpened()) voFile << buf << velCurr;

		// Step 6: Visualization
	}
	// 
	if (uoFile.isOpened()) uoFile.release(); 
	if (voFile.isOpened()) voFile.release();

	return 0;
}