#include <iostream>
#include <fstream>
#include <string>

#include <thread>
#include <chrono>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "impro_util.h"
#include "FileSeq.h"

using namespace std;

// Check roi validation
cv::Rect checkRoiValidation(cv::Rect roi, cv::Rect fullRoi, int minRoiSize = 1, int maxRoiSize = 8192);

//! FuncImshowOnline() reads an image file list and displays images of

int FuncImshowOnline(int argc, char** argv)
{
	// Variables
	FileSeq fsq;  // file sequence 
	int wtimeBeforeNext = 1000; // waiting time before reading next image (ms)
	float slowFactor = 1.0f;    // >1 for slow motion. negative to pause.
	int timeOfLastSuccessRead = (int)time(0);  // time (sec) that last image was successfully read.
	int iImg, nImg, ikey;
	char buf[1000];
	int togglePause = 1;
	int toggleSobel = 0;
	int imgHasSobel = 0;
	int toggleFeature = 0; 
	int imgHasFeature = 0; 
	int toggleInfo = 1;
	int maxRoiSize = 8192;
	int minRoiSize = 4; 
	double imshowSizeFactor = 1.1; 
	double roiZoomFactor = 1.1;
	double roiMoveRatio = 0.1; 
	cv::Rect roi(-1, -1, -1, -1); // the ROI to show, adjusted by z, Z, Ctrl-z, arrow keys
	cv::Size initImshowWinSize = cv::Size(1024, 576);
	cv::Size imshowWinSize = initImshowWinSize; 
	int needReadImage = 1;

	// Step 1: ask to input the file lists
	while (true) {
		fsq.setDirFilesByConsole();
		nImg = fsq.num_files();
		if (nImg < 0) {
			printf("# Error: You gave an empty file list. Try again.\n");
			continue;
		}
		printf("# %d files in the list, starting from \n# %s\n", nImg, fsq.fullPathOfFile(0).c_str());
		break;
	}

	// Step 2: ask to input waiting time before reading next image
	printf("# Waiting time before reading next image (by default 1000 ms):\n");
	wtimeBeforeNext = readIntFromCin(1);

	// Start the while loop
	iImg = 0;
	cv::Mat thisImg;
	while (true)
	{
		// try to read an image (iImg) if image is empty
		if (needReadImage >= 1 || thisImg.cols <= 0 || thisImg.rows <= 0) {
			thisImg = cv::imread(fsq.fullPathOfFile(iImg));
			if (roi.x < 0) roi = cv::Rect(0, 0, thisImg.cols, thisImg.rows); 
			// if read, show information and wait for a short time
			if (thisImg.cols >= 1 && thisImg.rows >= 1) {
				printf("# Successfully read image %d with size of %dx%d.\n",
					iImg, thisImg.rows, thisImg.cols);
				// refresh variables such as imgHasXXX mark 
				imgHasSobel = 0;
				imgHasFeature = 0; 
				needReadImage = 0;
				// wait
				cv::waitKeyEx(wtimeBeforeNext);
			}
		}

		// Sobel processing
		if (toggleSobel >= 1 && imgHasSobel == 0 && thisImg.cols >= 1 && thisImg.rows >= 1)
		{
			cv::Mat gray, grad, grad_x, grad_y, abs_grad_x, abs_grad_y;
			int ddepth = CV_16S;
			int dx = 1, dy = 1;
			int ksize = -1;
			int scale = 1;
			int delta = 0;
			cv::cvtColor(thisImg, gray, cv::COLOR_BGR2GRAY);
			cv::Sobel(gray, grad_x, ddepth, dx, 0, ksize, scale, delta, cv::BORDER_DEFAULT);
			cv::Sobel(gray, grad_y, ddepth, 0, dy, ksize, scale, delta, cv::BORDER_DEFAULT);
			cv::convertScaleAbs(grad_x, abs_grad_x);
			cv::convertScaleAbs(grad_y, abs_grad_y);
			cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, gray);
			printf("Image %d has done Sobel.\n", iImg);
			thisImg = gray;
			imgHasSobel = 1;
		}

		// Feature processing
		if (toggleFeature >= 1 && imgHasFeature == 0 && thisImg.cols >= 1 && thisImg.rows >= 1)
		{
			//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
//			cv::Ptr<cv::ORB> detector = cv::ORB::create();
			cv::Ptr<cv::BRISK> detector = cv::BRISK::create(100);
			std::vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors; 
			detector->detectAndCompute(thisImg, cv::noArray(), keypoints, descriptors); 
			cv::drawKeypoints(thisImg, keypoints, thisImg, cv::Scalar(255)); 
//			for (int k = 0; k < keypoints.size(); k++) {
//				int x = (int)(keypoints[k].pt.x + 0.5); 
//				int y = (int)(keypoints[k].pt.y + 0.5);
//				cv::Rect rect(x - 1, y - 1, 2, 2);
//				cv::rectangle(thisImg, rect, cv::Scalar(0), 1);
//			}
			imgHasFeature = 1; 
			cout << "# " << detector->getDefaultName() << " got " << keypoints.size() << " points.\n";
		}
		// 
		// show image (thisImg) 
		int winnameOption = 2;
		if (winnameOption == 1) {
			// winnameOption == 1: Shows image number and full file path 
			// A new window name changes window location, not good for image updating
			if (iImg >= 1) cv::destroyWindow(buf);
			snprintf(buf, 1000, "Image %d: %s", iImg, fsq.fullPathOfFile(iImg).c_str());
			cv::imshow(buf, thisImg);
			imshow_resize("imshow online", thisImg(roi), imshowWinSize);
		}
		else if (winnameOption == 2) {
			// winnameOption == 2: Keeps window name unchanged, changes buf instead. 
			snprintf(buf, 1000, "Image %d: %s . ", iImg, fsq.fullPathOfFile(iImg).c_str());
			//			for (int k = 0; k < 99; k++) printf("\b");
			//			cout << buf; 
			imshow_resize("imshow online", thisImg(roi), imshowWinSize);
		}

		ikey = cv::waitKeyEx(wtimeBeforeNext);
		if (ikey == (int)'A' || ikey == (int)'a')
		{
			// handles user key A or a for
		} 
		else if (ikey == (int)'B' || ikey == (int)'b')
		{
			// handles user key B or b for
		} 
		else if (ikey == (int)'C' || ikey == (int)'c')
		{
			// handles user key C or c for
		} 
		else if (ikey == (int)'D' || ikey == (int)'d')
		{
			// handles user key D or d for
		}
		else if (ikey == (int)'E' || ikey == (int)'e')
		{
			// handles user key E or e for
		}
		else if (ikey == (int)'F' || ikey == (int)'f')
		{
			// handles user key F or f for turn feature detection on 
			toggleFeature = 1 - toggleFeature;
			printf("# Feature toggle: %d.\n", toggleFeature);
		}
		else if (ikey == (int)'G' || ikey == (int)'g')
		{
			// handles user key G or g for
		} 
		else if (ikey == (int)'H' || ikey == (int)'h')
		{
			// handles user key H or h for
		} 
		else if (ikey == (int)'i' || ikey == (int)'I') 
		{
			// handles user key I or i for toggling information printing 
			toggleInfo = 1 - toggleInfo;
			printf("Information toggle: %d.\n", toggleInfo);
			if (toggleInfo >= 1) {
				printf("# Current image: %d.\n", iImg);
				printf("# Current image path: %s.\n", fsq.fullPathOfFile(iImg).c_str());
				printf("# Current Sobel toggle: %d\n", toggleSobel);
				printf("# Current Feature toggle: %d\n", toggleFeature);
				printf("# Current Pause toggle: %d\n", togglePause);
			}
		}  
		else if (ikey == (int)'J' || ikey == (int)'j')
		{
			// handles user key J or j for
		} 
		else if (ikey == (int)'K' || ikey == (int)'k')
		{
			// handles user key K or k for
		} 
		else if (ikey == (int)'L' || ikey == (int)'l')
		{
			// handles user key L or l for
		}
		else if (ikey == (int)'M' || ikey == (int)'m')		
		{
			// handles user key M or m for
		}
		else if (ikey == (int)'N' || ikey == (int)'n')
		{
			// handles user key N or n for
		}
		else if (ikey == (int)'O' || ikey == (int)'o')
		{
			// handles user key O or o for
		}
		else if (ikey == (int)'P' || ikey == (int)'p')
		{
			// handles user key P or p for
			togglePause = 1 - togglePause;
			printf("# Pause toggle: %d.\n", togglePause);
		}
		else if (ikey == (int)'Q' || ikey == (int)'q')
		{
			// handles user key O or o for
		}
		else if (ikey == (int)'R' || ikey == (int)'r')
		{
			// handles user key R or r for
			needReadImage = 1;
		}
		else if (ikey == (int)'S' || ikey == (int)'s')
		{
			// handles user key S or s for turn Sobel on 
			toggleSobel = 1 - toggleSobel;
			printf("# Sobel toggle: %d.\n", toggleSobel);
		}
		else if (ikey == (int)'T' || ikey == (int)'t')
		{
			// handles user key T or t for
		}
		else if (ikey == (int)'U' || ikey == (int)'u')
		{
			// handles user key O or o for
		}
		else if (ikey == (int)'V' || ikey == (int)'v')
		{
			// handles user key V or v for
		}
		else if (ikey == (int)'W')
		{
			// handles user key W to increase the imshow window size
			imshowWinSize.width = (int)(imshowWinSize.width * imshowSizeFactor + 0.5);
			imshowWinSize.height = (int)(imshowWinSize.height * imshowSizeFactor + 0.5);
		}
		else if (ikey == (int)'w')
		{
			// handles user key w to decrease the imshow window size 
			imshowWinSize.width = (int)(imshowWinSize.width / imshowSizeFactor + 0.5);
			imshowWinSize.height = (int)(imshowWinSize.height / imshowSizeFactor + 0.5);
		}
		else if (ikey == (int)(23)) // Ctrl-w
		{
			// handles user key Ctrl-w to reset the imshow window size
			imshowWinSize = initImshowWinSize;
		}
		else if (ikey == (int)'X' || ikey == (int)'x')
		{
			// handles user key X or x for
		}
		else if (ikey == (int)'Y' || ikey == (int)'y')
		{
			// handles user key Y or y for
		}
		else if (ikey == (int)'Z')
		{
			// handles user key Z to adjust the ROI (zoom-in)
			if (roi.width >= minRoiSize && roi.height >= minRoiSize) {
				cv::Rect trialRoi = roi; 
				float roiCx = roi.x + roi.width * 0.5f;
				float roiCy = roi.y + roi.height * 0.5f;
				trialRoi.width = (int) (roi.width / roiZoomFactor + 0.5); 
				trialRoi.height = (int) (roi.height / roiZoomFactor + 0.5);
				trialRoi.x = (int)(roiCx - trialRoi.width * 0.5 + 0.5);
				trialRoi.y = (int)(roiCy - trialRoi.height * 0.5 + 0.5);
				// Check ROI validation
				roi = checkRoiValidation(trialRoi, cv::Rect(0, 0, thisImg.cols, thisImg.rows),
					minRoiSize, maxRoiSize); 
				cout << "# Updated ROI: " << roi << endl;
			}
			else {
				printf("# Warning: ROI size cannot be smaller than %d pixels.\n", minRoiSize); 
			}
		}
		else if (ikey == (int)'z')
		{
			// handles user key z to adjust the ROI (zoom-out)
			if (roi.width <= maxRoiSize && roi.height <= maxRoiSize) {
				cv::Rect trialRoi = roi;
				float roiCx = roi.x + roi.width * 0.5f;
				float roiCy = roi.y + roi.height * 0.5f;
				trialRoi.width = (int)(roi.width * roiZoomFactor + 0.5);
				trialRoi.height = (int)(roi.height * roiZoomFactor + 0.5);
				trialRoi.x = (int)(roiCx - trialRoi.width * 0.5 + 0.5);
				trialRoi.y = (int)(roiCy - trialRoi.height * 0.5 + 0.5);
				// Check ROI validation
				roi = checkRoiValidation(trialRoi, cv::Rect(0, 0, thisImg.cols, thisImg.rows),
					minRoiSize, maxRoiSize);
				cout << "# Updated ROI: " << roi << endl;
			}
			else {
				printf("# Warning: ROI size cannot be larger than %d pixels.\n", maxRoiSize);
			}
		}
		else if (ikey == (int) (26)) /* Ctrl-z */
		{
			// handles user key Ctrl-z to reset roi
			roi = cv::Rect(0, 0, thisImg.cols, thisImg.rows); 
			cout << "# Updated ROI: " << roi << endl;
		}
		else if (ikey == 0x260000)
		{
			// handles user arrow UP
			cv::Rect trialRoi = roi;
			trialRoi.y -= (int) (trialRoi.height * roiMoveRatio + .5); 
			if (trialRoi.y < 0) trialRoi.y = 0; 
			roi = checkRoiValidation(trialRoi, cv::Rect(0, 0, thisImg.cols, thisImg.rows),
				minRoiSize, maxRoiSize);
			cout << "# Updated ROI: " << roi << endl;
		}
		else if (ikey == 0x280000)
		{
			// handles user arrow DOWN
			cv::Rect trialRoi = roi;
			trialRoi.y += (int)(trialRoi.height * roiMoveRatio + .5);
			if (trialRoi.y + trialRoi.height > thisImg.rows) 
				trialRoi.y = thisImg.rows - trialRoi.height;
			roi = checkRoiValidation(trialRoi, cv::Rect(0, 0, thisImg.cols, thisImg.rows),
				minRoiSize, maxRoiSize);
			cout << "# Updated ROI: " << roi << endl;
		}
		else if (ikey == 0x250000)
		{
			// handles user arrow LEFT
			cv::Rect trialRoi = roi;
			trialRoi.x -= (int)(trialRoi.width * roiMoveRatio + .5);
			if (trialRoi.x < 0) trialRoi.x = 0;
			roi = checkRoiValidation(trialRoi, cv::Rect(0, 0, thisImg.cols, thisImg.rows),
				minRoiSize, maxRoiSize);
			cout << "# Updated ROI: " << roi << endl;
		}
		else if (ikey == 0x270000)
		{
			// handles user arrow RIGHT
			cv::Rect trialRoi = roi;
			trialRoi.x += (int)(trialRoi.width * roiMoveRatio + .5);
			if (trialRoi.x + trialRoi.width > thisImg.cols)
				trialRoi.x = thisImg.cols - trialRoi.width;
			roi = checkRoiValidation(trialRoi, cv::Rect(0, 0, thisImg.cols, thisImg.rows),
				minRoiSize, maxRoiSize);
			cout << "# Updated ROI: " << roi << endl;
		}
		else if (ikey == 0x700000)
		{
			printf("# Arrow F1 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x710000)
		{
			printf("# Arrow F2 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x720000)
		{
			// handles F3
			printf("# Arrow F3 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x730000)
		{
			// handles F4
			printf("# Arrow F4 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x740000)
		{
			// handles F5
			printf("# Arrow F5 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x750000)
		{
			// handles F6
			printf("# Arrow F6 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x760000)
		{
			// handles F7
			printf("# Arrow F7 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x770000)
		{
			// handles F8
			printf("# Arrow F8 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x780000)
		{
			// handles F9
			printf("# Arrow F9 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x790000)
		{
			// handles F10
			printf("# Arrow F10 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == 0x7A0000)
		{
			// handles F11
			printf("# Arrow F11 is not defined yet.\n"); cout.flush();
		}
		else if (ikey == -1)
		{


		}
		else 
		{
			printf("# Unknown key %5d (%04X)\n", ikey, ikey); cout.flush();
		}




		// try next image
		if (togglePause == 0 && iImg + 1 < fsq.num_files()) {
			cv::Mat trialNextImg = cv::imread(fsq.fullPathOfFile(iImg + 1));
			if (roi.x < 0) roi = cv::Rect(0, 0, thisImg.cols, thisImg.rows);
			if (trialNextImg.cols > 0 && trialNextImg.rows > 0) {
				thisImg = trialNextImg.clone();
				iImg += 1;
				// refresh variables such as imgHasXXX mark 
				imgHasSobel = 0;
				imgHasFeature = 0; 
				needReadImage = 0;
				// print info
				if (toggleInfo >= 1) {
					printf("# Successfully read image %d with size of %dx%d.\n",
						iImg, thisImg.rows, thisImg.cols);
				}
			}
			else
			{
				if (toggleInfo >= 1) {
					printf("# Waiting for next image %d.\n", iImg + 1);
				}
			} // if (trialNextImg.cols > 0 && trialNextImg.rows > 0) 
		}  // if (togglePause == 0 && iImg + 1 < fsq.num_files())
	}

	cv::destroyWindow(buf);
	return 0;
}


cv::Rect checkRoiValidation(cv::Rect roi, cv::Rect fullRoi, int minRoiSize, int maxRoiSize)
{
	int nCorrectionRoi = 10;
	cv::Rect trialRoi = roi; 
	while (trialRoi.x < fullRoi.x || 
		trialRoi.y < fullRoi.y ||
		trialRoi.width < minRoiSize || 
		trialRoi.height < minRoiSize ||
		trialRoi.width > maxRoiSize || 
		trialRoi.height > maxRoiSize ||
		trialRoi.width + trialRoi.x > fullRoi.width + fullRoi.x ||
		trialRoi.height + trialRoi.y > fullRoi.height + fullRoi.y) 
	{

		if (trialRoi.x < fullRoi.x) trialRoi.x = fullRoi.x;
		if (trialRoi.y < fullRoi.y) trialRoi.y = fullRoi.y;
		if (trialRoi.width < minRoiSize) {
			trialRoi.width = minRoiSize;
			trialRoi.height = (int)(trialRoi.width * roi.height * 1.0 / roi.width + 0.5);
		}
		if (trialRoi.height < minRoiSize) {
			trialRoi.height = minRoiSize;
			trialRoi.width = (int)(trialRoi.height * roi.width * 1.0 / roi.height + 0.5);
		}
		if (trialRoi.width + trialRoi.x > fullRoi.width + fullRoi.x) {
			trialRoi.width = fullRoi.width + fullRoi.x - trialRoi.x;
			trialRoi.height = (int)(trialRoi.width * roi.height * 1.0 / roi.width + 0.5);
		}
		if (trialRoi.height + trialRoi.y > fullRoi.height + fullRoi.y) {
			trialRoi.height = fullRoi.height + fullRoi.y - trialRoi.y;
			trialRoi.width = (int)(trialRoi.height * roi.width * 1.0 / roi.height + 0.5);
		}
		// nCorrectionRoi --
		nCorrectionRoi--; 
		if (nCorrectionRoi < 0) {
			cout << "# Warning: ROI adjustment cannot fulfill the criteria.: Trying " 
				<< trialRoi << " but restored to " << roi << endl;
			trialRoi = roi; 
			break;
		}
	}
	return trialRoi; 
}