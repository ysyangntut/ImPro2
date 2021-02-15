#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "FileSeq.h" 
#include "impro_util.h"

using namespace std;

int FuncCamMoveCorrection(int argc, char** argv)
{
	int nRefPoints; 
	int nStep; 
	FileSeq fsqI; // input file sequence
	FileSeq fsqO; // output file sequence
	string fnameTrackedPoints; 
	vector<vector<cv::Point2f> >  vecvecTrackedPoints; 
	vector<int> refPointsIds;
	vector<vector<cv::Point2f> >  refVecvecTrackedPoints; 

	cout << "Input photos list: \n";
	fsqI.setDirFilesByConsole(); 
	cout << "Output photos list: \n";
	fsqO.setDirFilesByConsole();

	cout << "Enter the tracked points (format: compact.xml, VecVecPoint2f): "; 
	fnameTrackedPoints = readStringLineFromCin(); 
	cv::FileStorage ifs(fnameTrackedPoints, cv::FileStorage::READ);
	ifs["VecVecPoint2f"] >> vecvecTrackedPoints;
	nStep = (int) vecvecTrackedPoints.size(); 
	refVecvecTrackedPoints.resize(nStep);

	cout << "How many reference points are tracked?\n";
	nRefPoints = readIntFromCin(); 
	if (nRefPoints < 3) {
		cerr << "Number of reference points have to be >= 3.\n";
		return -1; 
	}
	refPointsIds.resize(nRefPoints);
	for (int iStep = 0; iStep < nStep; iStep++)
		refVecvecTrackedPoints[iStep].resize(nRefPoints);

	cout << "Input reference points ID one by one (1-based): \n"; 
	for (int iPoint = 0; iPoint < nRefPoints; iPoint++)
		refPointsIds[iPoint] = readIntFromCin() - 1; // from user's 1-base to array 0-base

	// copy all-point data (vecvecTrackedPoints) to selected reference points (refVecvecTrackedPoints)
	for (int iStep = 0; iStep < nStep; iStep++)
		for (int iPoint = 0; iPoint < nRefPoints; iPoint++)
			refVecvecTrackedPoints[iStep][iPoint] = vecvecTrackedPoints[iStep][refPointsIds[iPoint]];

	// generate new photos
	for (int iStep = 0; iStep < nStep; iStep++) {
		cv::Mat warp = cv::Mat::eye(3, 3, CV_32F); 
		cv::Mat oriImg = cv::imread(fsqI.fullPathOfFile(iStep));
		if (oriImg.rows == 0 || oriImg.cols == 0) {
			cerr << "Warning: Cannot read file " << fsqI.fullPathOfFile(iStep) << endl;
			continue; 
		}
		cv::Mat newImg; 
		if (nRefPoints < 4) {
			cv::Mat affine = cv::getAffineTransform(
				refVecvecTrackedPoints[iStep], refVecvecTrackedPoints[0]);
			affine.convertTo(affine, CV_32F); 
			warp.at<float>(0, 0) = affine.at<float>(0, 0);
			warp.at<float>(0, 1) = affine.at<float>(0, 1);
			warp.at<float>(0, 2) = affine.at<float>(0, 2);
			warp.at<float>(1, 0) = affine.at<float>(1, 0);
			warp.at<float>(1, 1) = affine.at<float>(1, 1);
			warp.at<float>(1, 2) = affine.at<float>(1, 2);
		}
		else {
			warp = cv::findHomography(
				refVecvecTrackedPoints[iStep], refVecvecTrackedPoints[0]);
			warp.convertTo(warp, CV_32F);
		}
		cv::warpPerspective(oriImg, newImg, warp, oriImg.size(), cv::INTER_LANCZOS4);
		cv::imwrite(fsqO.fullPathOfFile(iStep), newImg); 
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "Step " << iStep << " conversion completed.";
	}
	cout << endl;
	return 0;
}