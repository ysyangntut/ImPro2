#include "impro_util.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include <Windows.h> // for OPENFILENAME, GetOpenFileName
#include <shlobj_core.h>
#include <atlstr.h>
#endif 

#include "matchTemplateWithRotPyr.h"
#include "enhancedCorrelationWithReference.h"

using namespace std;
using namespace cv; 

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#else
int fopen_s(FILE **f, const char *name, const char *mode) {
    int ret = 0;
    assert(f);
    *f = fopen(name, mode);
    /* Can't be sure about 1-to-1 mapping of errno and MS' errno_t */
    if (!*f)
        ret = -1;
    return ret;
}
#endif



bool findChessboardCornersSubpix(cv::Mat image, Size patternSize, std::vector<Point2f> & corners,
	int flags, TermCriteria criteria)
{
	// variables
	bool bres;
	if (image.channels() != 1) cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	bres = cv::findChessboardCorners(image, patternSize, corners, flags);
	if (bres == false)
		return false;
	// subpix
	cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(0, 0), criteria);

	// check ordering. Convert to left-to-right ordering. If not, reverse points.
	if (corners[0].x > corners[corners.size() - 1].x)
		std::reverse(corners.begin(), corners.end());
	return bres;
}

bool findChessboardCornersSubpix(cv::Mat image, cv::Size patternSize, cv::Mat & corners, int flags, cv::TermCriteria criteria)
{
	// variables
	bool bres;
	if (image.channels() != 1) cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	bres = cv::findChessboardCorners(image, patternSize, corners, flags);
	if (bres == false)
		return false;
	// subpix
	cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(0, 0), criteria);

	// check ordering. Convert to left-to-right ordering. If not, reverse points.
	if (corners.at<cv::Point2f>(0, 0).x > corners.at<cv::Point2f>(corners.rows - 1, 0).x)
	{
		cv::Mat clone; corners.copyTo(clone);
		for (int i = 0; i < corners.rows; i++)
			corners.at<cv::Point2f>(i, 0) = clone.at<cv::Point2f>(corners.rows - i - 1, 0);
	}
	return bres;
}

std::vector<cv::Point3f> create3DChessboardCorners(cv::Size bsize, float squareSize_w, float squareSize_h)
{
    std::vector<cv::Point3f> corners3d;
	for (int i = 0; i < bsize.height; i++)
		for (int j = 0; j < bsize.width; j++)
			corners3d.push_back(cv::Point3f(float(j*squareSize_w),
				float(i*squareSize_h), 0));
	return corners3d;
}

cv::Mat create3DChessboardCornersMat(cv::Size bsize, float squareSize_w, float squareSize_h)
{
	cv::Mat corners3d(bsize, CV_32FC2);
	for (int i = 0; i < bsize.height; i++)
		for (int j = 0; j < bsize.width; j++)
			corners3d.at<cv::Point3f>(i, j) = cv::Point3f(float(j*squareSize_w),
				float(i*squareSize_h), 0);
	return corners3d;
}

double calibrateCameraFromChessboardImages(
    const std::vector<cv::Mat> & imgs,
	cv::Size imageSize,
	cv::Size bsize,
	float squareSize_w, float squareSize_h,
	cv::Mat & cmat,
	cv::Mat & dmat,
    std::vector<cv::Mat> & rmats,
    std::vector<cv::Mat> & tmats,
	cv::TermCriteria criteria,
	int flags // flag of calibrateCamera() 
)
{
	// variables
	int nimg;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<std::vector<Point3f> > objectPoints;
	bool bres;
	// find number of pairs of the calibration photos
	nimg = (int)imgs.size();
	if (nimg <= 0) {
		//		cerr << "calibrateCameraChessboard() error: No photos input.\n";
		return -1;
	}
	// Convert to gray level image
	corners.resize(nimg);
	for (int iimg = 0; iimg < nimg; iimg++)
	{
		cv::Mat img_m;
		if (imgs[iimg].channels() == 1)
			img_m = imgs[iimg];
		else if (imgs[iimg].channels() == 3)
			cv::cvtColor(imgs[iimg], img_m, cv::COLOR_BGR2GRAY);
		// Find chessboard corner
		bres = cv::findChessboardCorners(img_m, bsize, corners[iimg]);
		if (bres == false) {
			//			cerr << "Calibration error: Cannot find chessboard corners in calib image.\n";
			return -1;
		}
		cv::cornerSubPix(img_m, corners[iimg], cv::Size(5, 5), cv::Size(0, 0), criteria);
		// check ordering. Convert to left-to-right ordering. If not, reverse points.
		if (corners[iimg][0].x > corners[iimg][corners[iimg].size() - 1].x)
			std::reverse(corners[iimg].begin(), corners[iimg].end());
	}
	// generate object points
    objectPoints = std::vector<std::vector<Point3f> >(nimg, create3DChessboardCorners(bsize, squareSize_w, squareSize_h));

	// stereo calib
	if (cmat.cols != 3 || cmat.rows != 3 ||
		(flags & cv::CALIB_USE_INTRINSIC_GUESS) == false) {
		cmat = initCameraMatrix2D(objectPoints, corners, imageSize, 0);
		dmat = Mat::zeros(1, 8, CV_32F);
	}
	double rms = calibrateCamera(objectPoints, corners, imageSize,
		cmat, dmat,
		rmats, tmats,
		flags,
		//		CALIB_FIX_ASPECT_RATIO +
		//		CALIB_ZERO_TANGENT_DIST +
		//      CALIB_USE_INTRINSIC_GUESS +
		//      CALIB_SAME_FOCAL_LENGTH +
		//      CALIB_RATIONAL_MODEL +
		//      CALIB_FIX_K2 + CALIB_FIX_K3 + CALIB_FIX_K5 + CALIB_FIX_K6,
		//		CALIB_FIX_K3 + CALIB_FIX_K6,
		//		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		criteria);

	return rms;
}


double stereoCalibrateChessboard(
    const std::vector<cv::Mat> & imgs_L,
    const std::vector<cv::Mat> & imgs_R,
	cv::Size imageSize,
	cv::Size bsize,
	float squareSize_w, float squareSize_h,
	cv::Mat & cmat_L,
	cv::Mat & cmat_R,
	cv::Mat & dmat_L,
	cv::Mat & dmat_R,
	cv::Mat & rmat,
	cv::Mat & tmat,
	cv::Mat & emat,
	cv::Mat & fmat,
	cv::TermCriteria criteria,
	int flags
)
{
	// variables
	int npair;
    std::vector<std::vector<cv::Point2f> > corners_L, corners_R;
    std::vector<std::vector<Point3f> > objectPoints;
	bool bres;
	// find number of pairs of the calibration photos
	npair = (int)imgs_L.size();
	if (npair <= 0) {
		//		cerr << "stereoCalibrateChessboard() error: No photos input.\n";
		return -1;
	}
    if (npair != (int) imgs_R.size()) {
		//		cerr << "stereoCalibrateChessboard() error: Left and right photos should have the same number of photos.\n";
		return -1;
	}
	// Convert to gray level image
	corners_L.resize(npair);
	corners_R.resize(npair);
	for (int ipair = 0; ipair < npair; ipair++)
	{
		cv::Mat img_Lm, img_Rm;
		if (imgs_L[ipair].channels() == 1)
			img_Lm = imgs_L[ipair];
		else if (imgs_L[ipair].channels() == 3)
			cv::cvtColor(imgs_L[ipair], img_Lm, cv::COLOR_BGR2GRAY);
		if (imgs_R[ipair].channels() == 1)
			img_Rm = imgs_R[ipair];
		else if (imgs_R[ipair].channels() == 3)
			cv::cvtColor(imgs_R[ipair], img_Rm, cv::COLOR_BGR2GRAY);
		// Find chessboard color
		//   left
		bres = cv::findChessboardCorners(img_Lm, bsize, corners_L[ipair]);
		if (bres == false) {
			//			cerr << "Calibration error: Cannot find chessboard corners in left image.\n";
			return -1;
		}
		cv::cornerSubPix(img_Lm, corners_L[ipair], cv::Size(5, 5), cv::Size(0, 0), criteria);
		//   right
		bres = cv::findChessboardCorners(img_Rm, bsize, corners_R[ipair]);
		if (bres == false) {
			//			cerr << "Calibration error: Cannot find chessboard corners in right image.\n";
			return -1;
		}
		cv::cornerSubPix(img_Rm, corners_R[ipair], cv::Size(5, 5), cv::Size(0, 0), criteria);

		// check ordering. Convert to left-to-right ordering. If not, reverse points.
		if (corners_L[ipair][0].x > corners_L[ipair][corners_L[ipair].size() - 1].x)
			std::reverse(corners_L[ipair].begin(), corners_L[ipair].end());
		if (corners_R[ipair][0].x > corners_R[ipair][corners_R[ipair].size() - 1].x)
			std::reverse(corners_R[ipair].begin(), corners_R[ipair].end());
	}
	// generate object points
	objectPoints.resize(npair);
	for (int i = 0; i < npair; i++)
	{
		for (int j = 0; j < bsize.height; j++)
			for (int k = 0; k < bsize.width; k++)
				objectPoints[i].push_back(cv::Point3f(k*squareSize_w, j*squareSize_h, 0));
	}

	// stereo calib
#define _INIT_CAMERA_MATRIX_2D_
#ifdef _INIT_CAMERA_MATRIX_2D_
	cmat_L = initCameraMatrix2D(objectPoints, corners_L, imageSize, 0);
	cmat_R = initCameraMatrix2D(objectPoints, corners_R, imageSize, 0);
	dmat_L = Mat::zeros(1, 8, CV_32F);
	dmat_R = Mat::zeros(1, 8, CV_32F);
#else
    std::vector<cv::Mat> rmats, tmats;
	double rms_L = calibrateCamera(objectPoints, corners_L, imageSize,
		cmat_L, dmat_L, rmats, tmats); 
	double rms_R = calibrateCamera(objectPoints, corners_R, imageSize,
		cmat_R, dmat_R, rmats, tmats);
#endif
	double rms = stereoCalibrate(objectPoints, corners_L, corners_R,
		cmat_L, dmat_L,
		cmat_R, dmat_R,
		imageSize, rmat, tmat, emat, fmat,
		//		CALIB_FIX_ASPECT_RATIO +
		//		CALIB_ZERO_TANGENT_DIST +
		CALIB_USE_INTRINSIC_GUESS +
//		CALIB_SAME_FOCAL_LENGTH +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K2 + CALIB_FIX_K3 + CALIB_FIX_K5 + CALIB_FIX_K6,
		//		CALIB_FIX_K3 + CALIB_FIX_K6,
		//		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		criteria);

	return rms;
}

cv::Mat Rvec2R4(const cv::Mat & rvec, const cv::Mat & tvec)
{
	cv::Mat R3, R4, T;
	// rvec is a column vector
	if ((rvec.cols == 1 && rvec.rows == 3 && tvec.cols == 1 && tvec.rows == 3))
	{
		R4 = cv::Mat::eye(4, 4, rvec.type());
		R3 = R4(Rect(0, 0, 3, 3));
		T = R4(Rect(3, 0, 1, 3));
		Rodrigues(rvec, R3);
		tvec.copyTo(T);
	}
	if ((rvec.cols == 3 && rvec.rows == 3 && tvec.cols == 1 && tvec.rows == 3))
	{
		R4 = cv::Mat::eye(4, 4, rvec.type());
		R3 = R4(Rect(0, 0, 3, 3));
		T = R4(Rect(3, 0, 1, 3));
		rvec.copyTo(R3);
		tvec.copyTo(T);
	}
	return R4;
}

int getTmpltFromImage(const cv::Mat & img, const cv::Point2f & pnt, const cv::Size & tmpltSize, cv::Mat & tmplt, cv::Point2f & ref)
{
	int ret = 0;
	cv::Rect tmplt_rect;
	// tmplt size
	tmplt_rect.width = tmpltSize.width;
	tmplt_rect.height = tmpltSize.height;
	// tmplt upper-left position
	tmplt_rect.x = (int)(pnt.x - tmpltSize.width / 2 + 0.5);
	if (tmplt_rect.x < 0)
		tmplt_rect.x = 0;
	tmplt_rect.y = (int)(pnt.y - tmpltSize.height / 2 + 0.5);
	if (tmplt_rect.y < 0)
		tmplt_rect.y = 0;
	// check tmplt lower-right position
	if (tmplt_rect.x + tmpltSize.width > img.cols)
		tmplt_rect.x = img.cols - tmpltSize.width;
	if (tmplt_rect.y + tmpltSize.height > img.rows)
		tmplt_rect.y = img.rows - tmpltSize.height;
	// crop by copying
	if (tmpltSize.width <= img.cols && tmpltSize.height <= img.rows) {
		tmplt = cv::Mat::zeros(tmpltSize, img.type());
		img(tmplt_rect).copyTo(tmplt);
		ref.x = pnt.x - tmplt_rect.x;
		ref.y = pnt.y - tmplt_rect.y;
//		cout << "Rect: \n" << tmplt_rect << endl;
//		cout << "Ref: \n" << ref << endl;
	}
	else {
		img.copyTo(tmplt);
		ref = pnt;
		return 0;
	}
	return ret;
}

cv::Rect getTmpltRectFromImageSize(
	const cv::Size      &    imgSize,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Point2f   &    ref)
{
	cv::Rect tmplt_rect;
	// tmplt size
	tmplt_rect.width = tmpltSize.width;
	tmplt_rect.height = tmpltSize.height;
	// tmplt upper-left position
	tmplt_rect.x = (int)(pnt.x - tmpltSize.width / 2 + 0.5);
	if (tmplt_rect.x < 0)
		tmplt_rect.x = 0;
	tmplt_rect.y = (int)(pnt.y - tmpltSize.height / 2 + 0.5);
	if (tmplt_rect.y < 0)
		tmplt_rect.y = 0;
	// check tmplt lower-right position
	if (tmplt_rect.x + tmpltSize.width > imgSize.width)
		tmplt_rect.x = imgSize.width - tmpltSize.width;
	if (tmplt_rect.y + tmpltSize.height > imgSize.height)
		tmplt_rect.y = imgSize.height - tmpltSize.height;
	// crop by copying
	if (tmpltSize.width <= imgSize.width && tmpltSize.height <= imgSize.height) {
		ref.x = pnt.x - tmplt_rect.x;
		ref.y = pnt.y - tmplt_rect.y;
	}
	else {
		ref = pnt;
		tmplt_rect = cv::Rect(0, 0, imgSize.width, imgSize.height);
	}
	return tmplt_rect;
}

cv::Rect getTmpltRectFromImageSizeWithPreferredRef(
	const cv::Size      &    imgSize,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Point2f   &    ref)
{
	cv::Rect tmplt_rect;
	// tmplt size
	tmplt_rect.width = tmpltSize.width;
	tmplt_rect.height = tmpltSize.height;
	// tmplt upper-left position
	tmplt_rect.x = (int)(pnt.x - ref.x);
	if (tmplt_rect.x < 0)
		tmplt_rect.x = 0;
	tmplt_rect.y = (int)(pnt.y - ref.y);
	if (tmplt_rect.y < 0)
		tmplt_rect.y = 0;
	// check tmplt lower-right position
	if (tmplt_rect.x + tmpltSize.width > imgSize.width)
		tmplt_rect.x = imgSize.width - tmpltSize.width;
	if (tmplt_rect.y + tmpltSize.height > imgSize.height)
		tmplt_rect.y = imgSize.height - tmpltSize.height;
	// crop by copying
	if (tmpltSize.width <= imgSize.width && tmpltSize.height <= imgSize.height) {
		ref.x = pnt.x - tmplt_rect.x;
		ref.y = pnt.y - tmplt_rect.y;
	}
	else {
		ref = pnt;
		tmplt_rect = cv::Rect(0, 0, imgSize.width, imgSize.height);
	}
	return tmplt_rect;
}

cv::Rect getTmpltRectFromImage(
	const cv::Mat       &    img,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Point2f   &    ref)
{
	cv::Rect tmplt_rect;
	// tmplt size
	tmplt_rect.width = tmpltSize.width;
	tmplt_rect.height = tmpltSize.height;
	// tmplt upper-left position
	tmplt_rect.x = (int)(pnt.x - tmpltSize.width / 2 + 0.5);
	if (tmplt_rect.x < 0)
		tmplt_rect.x = 0;
	tmplt_rect.y = (int)(pnt.y - tmpltSize.height / 2 + 0.5);
	if (tmplt_rect.y < 0)
		tmplt_rect.y = 0;
	// check tmplt lower-right position
	if (tmplt_rect.x + tmpltSize.width > img.cols)
		tmplt_rect.x = img.cols - tmpltSize.width;
	if (tmplt_rect.y + tmpltSize.height > img.rows)
		tmplt_rect.y = img.rows - tmpltSize.height;
	// crop by copying
	if (tmpltSize.width <= img.cols && tmpltSize.height <= img.rows) {
		ref.x = pnt.x - tmplt_rect.x;
		ref.y = pnt.y - tmplt_rect.y;
//		cout << "Rect: \n" << tmplt_rect << endl;
//		cout << "Ref: \n" << ref << endl;
	}
	else {
//		img.copyTo(tmplt);
		ref = pnt;
		tmplt_rect = cv::Rect(0, 0, img.cols, img.rows); 
	}
	return tmplt_rect;
}



//  Windows
#ifdef _WIN32
#include <Windows.h>
double getWallTime() {
	LARGE_INTEGER time, freq;
	if (!QueryPerformanceFrequency(&freq)) {
		//  Handle error
		return 0;
	}
	if (!QueryPerformanceCounter(&time)) {
		//  Handle error
		return 0;
	}
	return (double)time.QuadPart / freq.QuadPart;
}
double getCpusTime() {
	FILETIME a, b, c, d;
	if (GetProcessTimes(GetCurrentProcess(), &a, &b, &c, &d) != 0) {
		//  Returns total user time.
		//  Can be tweaked to include kernel times as well.
		return
			(double)(d.dwLowDateTime |
			((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
	}
	else {
		//  Handle error
		return 0;
	}
}

//  Posix/Linux
#else
#include <time.h>
#include <sys/time.h>
double getWallTime() {
	struct timeval time;
	if (gettimeofday(&time, 0)) {
		//  Handle error
		return 0;
	}
	return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
double getCpusTime() {
	return (double)clock() / CLOCKS_PER_SEC;
}
#endif

std::string appendSubstringBeforeLastDot(std::string fname, std::string substr)
{
	int pos = (int)fname.find_last_of("."); 
	if (pos < 0) // no dot (i.e., no extension) in the file name
		return fname + substr; 
	return fname.substr(0, pos) + substr + fname.substr(pos);
}

int mtm_ecc(cv::InputArray _imgSrch, cv::InputArray _imgInit, cv::Point2f tPoint, 
    cv::Size tSize, std::vector<double>& result, cv::Point2f tGuess, float rotGuess,
	float xMin, float xMax, float yMin, float yMax, float rotMin, float rotMax, 
	cv::Size largeWinSize)
{
	double totalCpusTime = getCpusTime();
	double totalWallTime = getWallTime();
	// Check 
	if (_imgSrch.getMat().rows <= 0 || _imgSrch.getMat().cols <= 0 || 
		_imgInit.getMat().rows <= 0 || _imgInit.getMat().cols <= 0 || 
		tPoint.x < 0. || tPoint.y < 0. || 
		tPoint.x > _imgInit.getMat().cols || tPoint.y > _imgInit.getMat().rows)
	{
		return -1; 
	}
	// Default values
	if (result.size() < 12)
		result.resize(12, 0.0); 
	if (isnan(tGuess.x) || isnan(tGuess.y))
		tGuess = tPoint;
	if (isnan(rotGuess))
		rotGuess = 0.0f; 
	if (isnan(xMin))
		xMin = tGuess.x - 2 * tSize.width;
	if (isnan(xMax))
		xMax = tGuess.x + 2 * tSize.width;
	if (isnan(yMin))
		yMin = tGuess.y - 2 * tSize.height;
	if (isnan(yMax))
		yMax = tGuess.y + 2 * tSize.height;
	if (rotMin == 0.0f || isnan(rotMin))
		rotMin = 0.0f;
	if (rotMax == 0.0f || isnan(rotMax))
		rotMax = 0.0f;
	if (largeWinSize.width <= 0 || largeWinSize.height <= 0)
		largeWinSize = cv::Size(2 * tSize.width, 2 * tSize.height); 

	// Making sure images are gray scaled. 
	cv::Mat imgSrch_gray, imgInit_gray; 
	if (_imgSrch.channels() != 1)
		cv::cvtColor(_imgSrch, imgSrch_gray, cv::COLOR_BGR2GRAY);
	else
		imgSrch_gray = _imgSrch.getMat();
	if (_imgInit.channels() != 1)
		cv::cvtColor(_imgInit, imgInit_gray, cv::COLOR_BGR2GRAY);
	else
		imgInit_gray = _imgInit.getMat();

	// Estimate large-window movement 
	//  The precision sets to 1/4 of template size, assuring
	//  enough precision that is near template, avoiding possible 
	//  similar pattern around. 
    std::vector<double> largeWinResult(8, 0.0);
	cv::Mat largeWinResultMat(largeWinResult); 
	cv::Point2f refLargeWinPoint; 
	cv::Rect largeWinRect = getTmpltRectFromImage(
		imgInit_gray, tPoint, largeWinSize, refLargeWinPoint);
	double largeWinCpusTime = getCpusTime(); 
	double largeWinWallTime = getWallTime();
	float largeWin_xMin = xMin;
	float largeWin_xMax = xMax;
	float largeWin_yMin = yMin;
	float largeWin_yMax = yMax;
	float largeWin_xPcn = tSize.width * 0.25f;
	float largeWin_yPcn = tSize.height * 0.25f;
	float largeWin_rMin = rotMin;
	float largeWin_rMax = rotMax;
	float largeWin_rPcn = (float) (rotMax - rotMin) / 4.f;
	matchTemplateWithRotPyr(imgSrch_gray, imgInit_gray(largeWinRect),
		refLargeWinPoint.x, refLargeWinPoint.y,
		largeWin_xMin, largeWin_xMax, largeWin_xPcn, 
		largeWin_yMin, largeWin_yMax, largeWin_yPcn,
		largeWin_rMin, largeWin_rMax, largeWin_rPcn,
		largeWinResult); 
	largeWinCpusTime = getCpusTime() - largeWinCpusTime;
	largeWinWallTime = getWallTime() - largeWinWallTime;
//	cout << "Large-win:\n" << largeWinResultMat << endl;
	result[6] = largeWinCpusTime;
	result[7] = largeWinWallTime;
	// refined match
	double smallWinCpusTime = getCpusTime();
	double smallWinWallTime = getWallTime();
    std::vector<double> smallWinResult(8, 0.0);
	cv::Mat smallWinResultMat(smallWinResult);
	cv::Point2f refSmallWinPoint;
	cv::Rect smallWinRect = getTmpltRectFromImage(
		imgInit_gray, tPoint, tSize, refSmallWinPoint);	
	float smallWin_xMin = (float) largeWinResult[0] - 2 * largeWin_xPcn;
	float smallWin_xMax = (float) largeWinResult[0] + 2 * largeWin_xPcn;
	float smallWin_yMin = (float) largeWinResult[1] - 2 * largeWin_yPcn;
	float smallWin_yMax = (float) largeWinResult[1] + 2 * largeWin_yPcn;
	float smallWin_xPcn = (float) 1.f;
	float smallWin_yPcn = (float) 1.f;
	float smallWin_rMin = (float) largeWinResult[2] - 2 * largeWin_rPcn;
	float smallWin_rMax = (float) largeWinResult[2] + 2 * largeWin_rPcn;
	float smallWin_rPcn = 5.f;
	matchTemplateWithRotPyr(imgSrch_gray, imgInit_gray(smallWinRect),
		refSmallWinPoint.x, refSmallWinPoint.y,
		smallWin_xMin, smallWin_xMax, smallWin_xPcn,
		smallWin_yMin, smallWin_yMax, smallWin_yPcn,
		smallWin_rMin, smallWin_rMax, smallWin_rPcn,
		smallWinResult);
	smallWinCpusTime = getCpusTime() - smallWinCpusTime;
	smallWinWallTime = getWallTime() - smallWinWallTime;
//	cout << "Small-win:\n" << smallWinResultMat << endl;
	result[8] = smallWinCpusTime;
	result[9] = smallWinWallTime;

	// ecc 
	double eccCpusTime = getCpusTime();
	double eccWallTime = getWallTime();
    std::vector<double> eccResult(8, 0.0);
	cv::Mat eccResultMat(eccResult);
	cv::Point2f refEccPoint;
	cv::Rect eccRect = getTmpltRectFromImage(
		imgInit_gray, tPoint, tSize, refEccPoint);
	float ecc_guessX = (float) smallWinResult[0];
	float ecc_guessY = (float) smallWinResult[1];
	float ecc_guessR = (float) smallWinResult[2]; 
	int ecc_motionType; 
	if (abs(rotMax - rotMin) < 1e-3)
		ecc_motionType = cv::MOTION_TRANSLATION;
	else
		ecc_motionType = cv::MOTION_EUCLIDEAN;
	enhancedCorrelationWithReference(imgSrch_gray, imgInit_gray(eccRect),
		refEccPoint.x, refEccPoint.y, ecc_guessX, ecc_guessY, ecc_guessR,
		eccResult, ecc_motionType, 
		cv::TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01));
	eccCpusTime = getCpusTime() - eccCpusTime;
	eccWallTime = getWallTime() - eccWallTime;
//	cout << "Ecc:\n" << eccResultMat << endl;
	result[10] = eccCpusTime;
	result[11] = eccWallTime;

	totalCpusTime = getCpusTime() - totalCpusTime;
	totalWallTime = getWallTime() - totalWallTime;
	result[4] = totalCpusTime;
	result[5] = totalWallTime;

	result[0] = eccResult[0];
	result[1] = eccResult[1];
	result[2] = eccResult[2];
	result[3] = eccResult[3];
	
	cv::Mat resultMat(result);
//	cout << "Total result: \n" << resultMat << endl;

	//		cv::Point3f initGuess;
	return 0;
}

//int mtm_opf(cv::InputArray _imgSrch, cv::InputArray _imgInit, cv::Point2f tPoint,
//	cv::Size tSize, std::vector<double>& result, cv::Point2f tGuess, float rotGuess,
//	float xMin, float xMax, float yMin, float yMax, float rotMin, float rotMax,
//	cv::Size largeWinSize)
//{
//	double totalCpusTime = getCpusTime();
//	double totalWallTime = getWallTime();
//	// Check 
//	if (_imgSrch.getMat().rows <= 0 || _imgSrch.getMat().cols <= 0 ||
//		_imgInit.getMat().rows <= 0 || _imgInit.getMat().cols <= 0 ||
//		tPoint.x < 0. || tPoint.y < 0. ||
//		tPoint.x > _imgInit.getMat().cols || tPoint.y > _imgInit.getMat().rows)
//	{
//		return -1;
//	}
//	// Default values
//	if (result.size() < 12)
//		result.resize(12, 0.0);
//	if (isnan(tGuess.x) || isnan(tGuess.y))
//		tGuess = tPoint;
//	if (isnan(rotGuess))
//		rotGuess = 0.0f;
//	if (isnan(xMin))
//		xMin = tGuess.x - 2 * tSize.width;
//	if (isnan(xMax))
//		xMax = tGuess.x + 2 * tSize.width;
//	if (isnan(yMin))
//		yMin = tGuess.y - 2 * tSize.height;
//	if (isnan(yMax))
//		yMax = tGuess.y + 2 * tSize.height;
//	if (rotMin == 0.0f || isnan(rotMin))
//		rotMin = 0.0f;
//	if (rotMax == 0.0f || isnan(rotMax))
//		rotMax = 0.0f;
//	if (largeWinSize.width <= 0 || largeWinSize.height <= 0)
//		largeWinSize = cv::Size(2 * tSize.width, 2 * tSize.height);
//
//	// Making sure images are gray scaled. 
//	cv::Mat imgSrch_gray, imgInit_gray;
//	if (_imgSrch.channels() != 1)
//		cv::cvtColor(_imgSrch, imgSrch_gray, cv::COLOR_BGR2GRAY);
//	else
//		imgSrch_gray = _imgSrch.getMat();
//	if (_imgInit.channels() != 1)
//		cv::cvtColor(_imgInit, imgInit_gray, cv::COLOR_BGR2GRAY);
//	else
//		imgInit_gray = _imgInit.getMat();
//
//	// Estimate large-window movement 
//	//  The precision sets to 1/4 of template size, assuring
//	//  enough precision that is near template, avoiding possible 
//	//  similar pattern around. 
//	std::vector<double> largeWinResult(8, 0.0);
//	cv::Mat largeWinResultMat(largeWinResult);
//	cv::Point2f refLargeWinPoint;
//	cv::Rect largeWinRect = getTmpltRectFromImage(
//		imgInit_gray, tPoint, largeWinSize, refLargeWinPoint);
//	double largeWinCpusTime = getCpusTime();
//	double largeWinWallTime = getWallTime();
//	float largeWin_xMin = xMin;
//	float largeWin_xMax = xMax;
//	float largeWin_yMin = yMin;
//	float largeWin_yMax = yMax;
//	float largeWin_xPcn = tSize.width * 0.25f;
//	float largeWin_yPcn = tSize.height * 0.25f;
//	float largeWin_rMin = rotMin;
//	float largeWin_rMax = rotMax;
//	float largeWin_rPcn = (float)(rotMax - rotMin) / 4.f;
//	matchTemplateWithRotPyr(imgSrch_gray, imgInit_gray(largeWinRect),
//		refLargeWinPoint.x, refLargeWinPoint.y,
//		largeWin_xMin, largeWin_xMax, largeWin_xPcn,
//		largeWin_yMin, largeWin_yMax, largeWin_yPcn,
//		largeWin_rMin, largeWin_rMax, largeWin_rPcn,
//		largeWinResult);
//	largeWinCpusTime = getCpusTime() - largeWinCpusTime;
//	largeWinWallTime = getWallTime() - largeWinWallTime;
//	//	cout << "Large-win:\n" << largeWinResultMat << endl;
//	result[6] = largeWinCpusTime;
//	result[7] = largeWinWallTime;
//	// refined match
//	double smallWinCpusTime = getCpusTime();
//	double smallWinWallTime = getWallTime();
//	std::vector<double> smallWinResult(8, 0.0);
//	cv::Mat smallWinResultMat(smallWinResult);
//	cv::Point2f refSmallWinPoint;
//	cv::Rect smallWinRect = getTmpltRectFromImage(
//		imgInit_gray, tPoint, tSize, refSmallWinPoint);
//	float smallWin_xMin = (float)largeWinResult[0] - 2 * largeWin_xPcn;
//	float smallWin_xMax = (float)largeWinResult[0] + 2 * largeWin_xPcn;
//	float smallWin_yMin = (float)largeWinResult[1] - 2 * largeWin_yPcn;
//	float smallWin_yMax = (float)largeWinResult[1] + 2 * largeWin_yPcn;
//	float smallWin_xPcn = (float) 1.f;
//	float smallWin_yPcn = (float) 1.f;
//	float smallWin_rMin = (float)largeWinResult[2] - 2 * largeWin_rPcn;
//	float smallWin_rMax = (float)largeWinResult[2] + 2 * largeWin_rPcn;
//	float smallWin_rPcn = 5.f;
//	matchTemplateWithRotPyr(imgSrch_gray, imgInit_gray(smallWinRect),
//		refSmallWinPoint.x, refSmallWinPoint.y,
//		smallWin_xMin, smallWin_xMax, smallWin_xPcn,
//		smallWin_yMin, smallWin_yMax, smallWin_yPcn,
//		smallWin_rMin, smallWin_rMax, smallWin_rPcn,
//		smallWinResult);
//	smallWinCpusTime = getCpusTime() - smallWinCpusTime;
//	smallWinWallTime = getWallTime() - smallWinWallTime;
//	//	cout << "Small-win:\n" << smallWinResultMat << endl;
//	result[8] = smallWinCpusTime;
//	result[9] = smallWinWallTime;
//
//	// optical flow 
//	double opfCpusTime = getCpusTime();
//	double opfWallTime = getWallTime();
//	std::vector<double> opfResult(4, 0.0);
//	cv::Mat opfResultMat(opfResult);
//	cv::Point2f refOpfPoint;
//	cv::Rect opfRect = getTmpltRectFromImage(
//		imgInit_gray, tPoint, 
//		cv::Size((int)(tSize.width * 1.5), (int)(tSize.height)), 
//		refOpfPoint);
//	float opf_guessX = (float)smallWinResult[0];
//	float opf_guessY = (float)smallWinResult[1];
//	float opf_guessR = (float)smallWinResult[2];
//	int opf_maxLevel = 1; 
//	std::vector<cv::Point2f> tPointVec(1); tPointVec[0] = tPoint; // 1-point vector
//	std::vector<cv::Point2f> tPointAfterVec(1);
//	tPointAfterVec[0].x = opf_guessX; // initial guess 
//	tPointAfterVec[0].y = opf_guessY; // initial guess 
//	std::vector<uchar> opf_status(1);
//	std::vector<float> opf_err(1);
//	if (abs(opf_guessR) > 1e-3) { // has a rotation
//		cv::Mat rotTmplt = imgInit_gray(opfRect);
//		cv::Mat noRotBackup; // backup the template before rotation
//		rotTmplt.copyTo(noRotBackup);
//		cv::Mat r33 = getRotationMatrix2D(refOpfPoint, (double)opf_guessR, 1);
//		cv::warpAffine(noRotBackup, rotTmplt, r33, rotTmplt.size(), cv::INTER_CUBIC);
//		cv::calcOpticalFlowPyrLK(imgInit_gray, imgSrch_gray,
//			tPointVec, tPointAfterVec, opf_status, opf_err,
//			tSize, opf_maxLevel,
//			cv::TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01),
//			cv::OPTFLOW_USE_INITIAL_FLOW); 
//		noRotBackup.copyTo(rotTmplt); // recover the imgInit_gray from being locally rotated
//	}
//	else { // has no rotation
//		cv::calcOpticalFlowPyrLK(imgInit_gray, imgSrch_gray,
//			tPointVec, tPointAfterVec, opf_status, opf_err,
//			tSize, opf_maxLevel,
//			cv::TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01),
//			cv::OPTFLOW_USE_INITIAL_FLOW);
//	}
//	opfResult[0] = tPointAfterVec[0].x;
//	opfResult[1] = tPointAfterVec[0].y;
//	opfResult[2] = opf_guessR;
//	opfResult[3] = 1. - opf_err[0]; 
//	opfCpusTime = getCpusTime() - opfCpusTime;
//	opfWallTime = getWallTime() - opfWallTime;
//	result[10] = opfCpusTime;
//	result[11] = opfWallTime;
//	totalCpusTime = getCpusTime() - totalCpusTime;
//	totalWallTime = getWallTime() - totalWallTime;
//	result[4] = totalCpusTime;
//	result[5] = totalWallTime;
//
//	result[0] = opfResult[0];
//	result[1] = opfResult[1];
//	result[2] = opfResult[2];
//	result[3] = opfResult[3];
//
//	return 0;
//}

int mtm_opfs(cv::Mat imgInit, cv::Mat imgSrch,
    const std::vector<cv::Point2f> & tPointsInit,
    std::vector<cv::Point3f> & tPointsSrch,
    const std::vector<float> & maxMove,
    std::vector<uchar> & status,
    std::vector<float> & error,
    std::vector<float> & timing,
	cv::Size winSize,
	int maxLevel,
	cv::TermCriteria criteria,
	int flags)
{
	double totalCpusTime = getCpusTime();
	double totalWallTime = getWallTime();
	// Check 
	if (imgSrch.rows <= 0 || imgSrch.cols <= 0 ||
		imgInit.rows <= 0 || imgInit.cols <= 0)
	{
		return -1; // imgInit or imgSrch is empty
	}
	if (timing.size() < 8)
		timing.resize(8, 0.f);
	if (tPointsSrch.size() < tPointsInit.size()) {
		int tPointsSrchOriSize = (int)tPointsSrch.size();
		tPointsSrch.resize(tPointsInit.size());
        for (int i = tPointsSrchOriSize; i < (int) tPointsInit.size(); i++) {
			tPointsSrch[i].x = tPointsInit[i].x;
			tPointsSrch[i].y = tPointsInit[i].y;
			tPointsSrch[i].z = 0.f;
		}
	}
	if (status.size() < tPointsInit.size())
		status.resize(tPointsInit.size());
	if (error.size() < tPointsInit.size())
		error.resize(tPointsInit.size());

	// clone tPointsInit to tPointsInitValid (points of nanf to remove)
	// clone tPointsSrch to tPointsSrchValid 
    // build std::vector<int> mappingValid
	// E.g., If tPointsInit[3].x or .y is nan, then
	//       tPointsInitValid[3] = tPointsInitValid[4], 
	//       o2n[4] = 3, n2o[3] = 4
    std::vector<int> o2n, n2o;
    std::vector<cv::Point2f> tPointsInitValid;
    std::vector<cv::Point3f> tPointsSrchValid;
	points2fVecValid(tPointsInit, tPointsInitValid, o2n, n2o,
		0.f, (float) imgInit.cols, 0.f, (float) imgInit.rows);
	tPointsSrchValid.resize(tPointsInitValid.size()); 
    std::vector<uchar> statusValid(tPointsInitValid.size(), 0);
    std::vector<float> errorValid(tPointsInitValid.size(), -1.f);
    for (int i = 0; i < (int) tPointsInitValid.size(); i++) {
		tPointsInitValid[i] = tPointsInit[n2o[i]];
		tPointsSrchValid[i] = tPointsSrch[n2o[i]];
	}
//	std::vector<int> mapv(tPointsInit.size(), -1);
//	std::vector<int> invm(tPointsInit.size(), -1);
//	int nValid = 0;
//	for (int i = 0; i < tPointsInit.size(); i++) {
//		if (isnan(tPointsInit[i].x) == false &&
//			isnan(tPointsInit[i].y) == false && 
//			tPointsInit[i].x >= 0 && 
//			tPointsInit[i].y >= 0 &&
//			tPointsInit[i].x < imgInit.cols && 
//			tPointsInit[i].y < imgInit.rows) {
//			mapv[i] = nValid;
//			invm[nValid] = i;
//			nValid++;
//		}
//	}
//	if (nValid == 0) return -2; 
//
//	// build vectors for function calcOpticalFlowPyr() 
//	std::vector<cv::Point2f> tPointsInitValid(nValid);
//	std::vector<cv::Point3f> tPointsSrchValid(nValid);
//	std::vector<uchar> statusValid(nValid, 0);
//	std::vector<float> errorValid(nValid, -1.f);
//	for (int i = 0; i < tPointsInitValid.size(); i++) {
//		tPointsInitValid[i] = tPointsInit[invm[i]];
//		tPointsSrchValid[i] = tPointsSrch[invm[i]];
//	}

	// Default values

	// Initialize tPointsSrchValid if flags OPTFLOW_USE_INITIAL_FLOW is off
	if ((flags & OPTFLOW_USE_INITIAL_FLOW) == 0) {
        for (int i = 0; i < (int) tPointsInitValid.size(); i++) {
			tPointsSrchValid[i].x = tPointsInitValid[i].x;
			tPointsSrchValid[i].y = tPointsInitValid[i].y;
			tPointsSrchValid[i].z = 0.f;
		}
	}

	// Making sure images are gray scaled. 
	cv::Mat imgSrch_gray, imgInit_gray;
	if (imgSrch.channels() != 1)
		cv::cvtColor(imgSrch, imgSrch_gray, cv::COLOR_BGR2GRAY);
	else
		imgSrch_gray = imgSrch;
	if (imgInit.channels() != 1)
		cv::cvtColor(imgInit, imgInit_gray, cv::COLOR_BGR2GRAY); // imgInit_gray is a clone
	else if (maxMove[2] > 1e-6) // if rotation is possible, clone imgInit to imgInit_gray
		imgInit.copyTo(imgInit_gray);
	else
		imgInit_gray = imgInit; 

	// Estimate large-window movement 
	//  The precision sets to 1/4 of template size, assuring
	//  enough precision that is near template, avoiding possible 
	//  similar pattern around. 
	double largeWinCpusTime = getCpusTime();
	double largeWinWallTime = getWallTime();
	int largeWinScale = (int)(std::pow(2, maxLevel) + 0.1f); // largeWinScale = 2^maxLevel 
	cv::Size largeWinSize(winSize.width * largeWinScale, winSize.height * largeWinScale); 
    std::vector<double> largeWinResult(8, 0.0);
	cv::Mat largeWinResultMat(largeWinResult);
    for (int iPoint = 0; iPoint < (int) tPointsInitValid.size(); iPoint++) {
		cv::Point2f refLargeWinPoint;
		cv::Rect largeWinRect = getTmpltRectFromImage(
			imgInit_gray, tPointsInitValid[iPoint], largeWinSize, refLargeWinPoint);
		float largeWin_xMin = tPointsSrchValid[iPoint].x - maxMove[0];
		float largeWin_xMax = tPointsSrchValid[iPoint].x + maxMove[0];
		float largeWin_yMin = tPointsSrchValid[iPoint].y - maxMove[1];
		float largeWin_yMax = tPointsSrchValid[iPoint].y + maxMove[1];
		float largeWin_xPcn = winSize.width * 0.25f;
		float largeWin_yPcn = winSize.height * 0.25f;
		float largeWin_rMin = tPointsSrchValid[iPoint].z - maxMove[2];
		float largeWin_rMax = tPointsSrchValid[iPoint].z + maxMove[2];
		float largeWin_rPcn = (float)(maxMove[2]) / 2.f;
		matchTemplateWithRotPyr(imgSrch_gray, imgInit_gray(largeWinRect),
			refLargeWinPoint.x, refLargeWinPoint.y,
			largeWin_xMin, largeWin_xMax, largeWin_xPcn,
			largeWin_yMin, largeWin_yMax, largeWin_yPcn,
			largeWin_rMin, largeWin_rMax, largeWin_rPcn,
			largeWinResult);
		tPointsSrchValid[iPoint].x = (float) largeWinResult[0];
		tPointsSrchValid[iPoint].y = (float) largeWinResult[1];
		if (maxMove[2] > 1e-6)
			tPointsSrchValid[iPoint].z = (float) largeWinResult[2];
	}
	largeWinCpusTime = getCpusTime() - largeWinCpusTime;
	largeWinWallTime = getWallTime() - largeWinWallTime;
	//	cout << "Large-win:\n" << largeWinResultMat << endl;
	timing[2] = (float) largeWinCpusTime;
	timing[3] = (float) largeWinWallTime;

	// refined match. assuming tPointsSrch are close to final result (less than 1/2 of window size)
	double smallWinCpusTime = getCpusTime();
	double smallWinWallTime = getWallTime();
    std::vector<double> smallWinResult(8, 0.0);
	cv::Mat smallWinResultMat(smallWinResult);
    for (int iPoint = 0; iPoint < (int) tPointsInitValid.size(); iPoint++) {
		cv::Point2f refSmallWinPoint;
		cv::Rect smallWinRect = getTmpltRectFromImage(
			imgInit_gray, tPointsInitValid[iPoint], winSize, refSmallWinPoint);
		float smallWin_xMin = tPointsSrchValid[iPoint].x - winSize.width / 2;
		float smallWin_xMax = tPointsSrchValid[iPoint].x + winSize.width / 2;
		float smallWin_yMin = tPointsSrchValid[iPoint].y - winSize.height / 2;
		float smallWin_yMax = tPointsSrchValid[iPoint].y + winSize.height / 2;
		float smallWin_xPcn = winSize.width * 0.25f;
		float smallWin_yPcn = winSize.height * 0.25f;
		float smallWin_rMin = tPointsSrch[iPoint].z - maxMove[2];
		float smallWin_rMax = tPointsSrch[iPoint].z + maxMove[2];
		float smallWin_rPcn = (float) 1.0f;
		matchTemplateWithRotPyr(imgSrch_gray, imgInit_gray(smallWinRect),
			refSmallWinPoint.x, refSmallWinPoint.y,
			smallWin_xMin, smallWin_xMax, smallWin_xPcn,
			smallWin_yMin, smallWin_yMax, smallWin_yPcn,
			smallWin_rMin, smallWin_rMax, smallWin_rPcn,
			smallWinResult);
		tPointsSrchValid[iPoint].x = (float) smallWinResult[0];
		tPointsSrchValid[iPoint].y = (float) smallWinResult[1];
		if (maxMove[2] > 1e-6)
			tPointsSrchValid[iPoint].z = (float) largeWinResult[2];
	}
	smallWinCpusTime = getCpusTime() - smallWinCpusTime;
	smallWinWallTime = getWallTime() - smallWinWallTime;
	//	cout << "Small-win:\n" << smallWinResultMat << endl;
	timing[4] = (float)smallWinCpusTime;
	timing[5] = (float)smallWinWallTime;

	// optical flow 
	double opfCpusTime = getCpusTime();
	double opfWallTime = getWallTime();
    std::vector<cv::Point2f> refOpfPoint;
	// Find max winSize factor induced by rotations and new (larger) winSize
	float maxWinFactorByRot = 1.f; 
    for (int iPoint = 0; iPoint < (int) tPointsInitValid.size(); iPoint++) {
		float iFactor;
		if (abs(tPointsSrchValid[iPoint].z) < 1e-6)
			iFactor = 1.f;
		else 
			iFactor = 1.f + 0.415f * 
			(float) sin(2 * tPointsSrchValid[iPoint].z * 3.1416 / 180.f);
		if (maxWinFactorByRot < iFactor)
			maxWinFactorByRot = iFactor; 
	}
	cv::Size rotWinSize;
	if (maxWinFactorByRot == 1.f)
		rotWinSize = winSize;
	else {
		rotWinSize.width = (int)(winSize.width * maxWinFactorByRot + .5f);
		rotWinSize.height = (int)(winSize.height * maxWinFactorByRot + .5f);
	}

	// rotate sub-image of targets in imgInit_gray if necessary
    for (int iPoint = 0; iPoint < (int) tPointsInitValid.size(); iPoint++) {
		if (abs(tPointsSrchValid[iPoint].z) > 1e-3) {
			cv::Point2f refOpfPoint;
			cv::Rect opfRect = getTmpltRectFromImage(
				imgInit_gray, tPointsInitValid[iPoint],
				rotWinSize,
				refOpfPoint);
			cv::Mat rotTmplt = imgInit_gray(opfRect);
			cv::Mat r33 = getRotationMatrix2D(refOpfPoint, 
				(double)tPointsSrchValid[iPoint].z, 1);
			cv::warpAffine(rotTmplt, rotTmplt, r33, 
				rotTmplt.size(), cv::INTER_CUBIC);
		}
	}
	// copy tPointsSrch.x/y to tPointsSrch2f.x/y (as tPointsSrch has additional .z for rotation) 
    std::vector<cv::Point2f> tPointsSrchValid2f(tPointsSrchValid.size());
    for (int iPoint = 0; iPoint < (int) tPointsSrchValid.size(); iPoint++) {
		tPointsSrchValid2f[iPoint].x = tPointsSrchValid[iPoint].x;
		tPointsSrchValid2f[iPoint].y = tPointsSrchValid[iPoint].y;
	}
	// optical flow 
	cv::calcOpticalFlowPyrLK(imgInit_gray, imgSrch_gray,
		tPointsInitValid, tPointsSrchValid2f, statusValid, errorValid,
		rotWinSize, maxLevel,
		criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    for (int iPoint = 0; iPoint < (int) tPointsSrchValid.size(); iPoint++) {
		tPointsSrchValid[iPoint].x = tPointsSrchValid2f[iPoint].x;
		tPointsSrchValid[iPoint].y = tPointsSrchValid2f[iPoint].y;
	}
	// copy tPointsSrchValid to valid tPointsSrch
    for (int iPoint = 0; iPoint < (int) tPointsSrchValid.size(); iPoint++) {
		tPointsSrch[n2o[iPoint]] = tPointsSrchValid[iPoint];
		status[n2o[iPoint]] = statusValid[iPoint];
		error[n2o[iPoint]] = errorValid[iPoint];
	}

	opfCpusTime = getCpusTime() - opfCpusTime;
	opfWallTime = getWallTime() - opfWallTime;
	timing[6] = (float)opfCpusTime;
	timing[7] = (float)opfWallTime;
	totalCpusTime = getCpusTime() - totalCpusTime;
	totalWallTime = getWallTime() - totalWallTime;
	timing[0] = (float)totalCpusTime;
	timing[1] = (float)totalWallTime;
	return 0;
}

int points2fVecValid(const std::vector<cv::Point2f>& oldVec,
    std::vector<cv::Point2f>& newVec,
    std::vector<int>& o2n, std::vector<int>& n2o,
	float xMin, float xMax, float yMin, float yMax)
{
	o2n.resize(oldVec.size(), -1);
	n2o.resize(oldVec.size(), -1);
	int nValid = 0;
    for (int i = 0; i < (int) oldVec.size(); i++) {
		if (isnan(oldVec[i].x) == false &&
			isnan(oldVec[i].y) == false &&
			oldVec[i].x >= xMin &&
			oldVec[i].y >= yMin &&
			oldVec[i].x <= xMax &&
			oldVec[i].y <= yMax) {
			o2n[i] = nValid;
			n2o[nValid] = i;
			nValid++;
		}
	}
	if (nValid == 0) return -1;
	newVec.resize(nValid); 
	for (int i = 0; i < nValid; i++)
		newVec[i] = oldVec[n2o[i]]; 
	return 0;
}

int points3fVecValid(const std::vector<cv::Point3f>& oldVec,
    std::vector<cv::Point3f>& newVec,
    std::vector<int>& o2n, std::vector<int>& n2o,
	float xMin, float xMax, float yMin, float yMax, 
	float zMin, float zMax)
{
	o2n.resize(oldVec.size(), -1);
	n2o.resize(oldVec.size(), -1);
	int nValid = 0;
    for (int i = 0; i < (int) oldVec.size(); i++) {
		if (isnan(oldVec[i].x) == false &&
			isnan(oldVec[i].y) == false &&
			isnan(oldVec[i].z) == false &&
			oldVec[i].x >= xMin &&
			oldVec[i].y >= yMin &&
			oldVec[i].z >= zMin &&
			oldVec[i].x <= xMax &&
			oldVec[i].y <= yMax &&
			oldVec[i].z <= zMax) {
			o2n[i] = nValid;
			n2o[nValid] = i;
			nValid++;
		}
	}
	if (nValid == 0) return -1;
	newVec.resize(nValid);
	for (int i = 0; i < nValid; i++)
		newVec[i] = oldVec[n2o[i]];
	return 0;
}

bool isValid(cv::Point2f p) {
	return (isnan(p.x) == false && isnan(p.y) == false); 
}
bool isValid(cv::Point3f p) {
	return (isnan(p.x) == false && isnan(p.y) == false
		&& isnan(p.z) == false);
}
bool isValid(const std::vector<cv::Point2f> & vp) {
    for (int i = 0; i < (int) vp.size(); i++) {
		if (isValid(vp[i]) == false)
			return false;
	}
	return true; 
}

std::string appendSlashOrBackslashAfterDirectoryIfNecessary(std::string dir)
{
	string appended = dir; 
	if (appended.length() <= 0 || 
	    appended[appended.length() - 1] == '/' || 
		appended[appended.length() - 1] == '\\') 
		return dir;

	// count '/' and '\\' in the dir string
	size_t n_slash = std::count(appended.begin(), appended.end(), '/');
	size_t n_bslash = std::count(appended.begin(), appended.end(), '\\');
	if (n_slash > n_bslash)
		appended += "/";
	else
		appended += "\\";
	return appended;
}

std::string extFilenameRemoved(std::string f)
{
	int dotPosition = -1;
	for (int i = (int) f.length() - 1; i >= 1; i--) {
		if (f[i] == '.')
			dotPosition = i;
		if (f[i] == '\\' || f[i] == '/')
			break;
	}
	if (dotPosition == -1)
		return f;
	else
		return f.substr(0, dotPosition);
}

std::string directoryOfFullPathFile(std::string f)
{
	int slashPosition = -1;
	for (int i = (int)f.length() - 1; i >= 1; i--) {
		if (f[i] == '\\' || f[i] == '/') {
			slashPosition = i;
			break; 
		}
	}
	if (slashPosition == -1)
		return std::string("");
	else
		return f.substr(0, slashPosition + 1);
}

std::string fileOfFullPathFile(std::string f)
{
	int slashPosition = -1;
	for (int i = (int)f.length() - 1; i >= 1; i--) {
		if (f[i] == '\\' || f[i] == '/') {
			slashPosition = i;
			break;
		}
	}
	if (slashPosition == -1)
		return f;
	else
		return f.substr(slashPosition + 1);
}

std::vector<cv::Point2f> interpQ4(const std::vector<cv::Point2f> & inPoints, int n12, int n23)
{
    std::vector<cv::Point2f> outPoints;
    std::vector<Point2f> objPoints(4);
	objPoints[0] = Point2f(0, 0); objPoints[1] = Point2f(1, 0);
	objPoints[2] = Point2f(1, 1); objPoints[3] = Point2f(0, 1);
	cv::Mat homoMat = findHomography(objPoints, inPoints);
	cv::Mat inManyPoints;
	cv::Mat objManyPoints(3, n12 * n23, CV_64F);  // matrix: 3 x (n12*n23)
	for (int i = 0; i < n23; i++) {
		for (int j = 0; j < n12; j++) {
			objManyPoints.at<double>(0, j + i * n12) = j / (n12 - 1.);
			objManyPoints.at<double>(1, j + i * n12) = i / (n23 - 1.);
			objManyPoints.at<double>(2, j + i * n12) = 1.;
		}
	}
	inManyPoints = homoMat * objManyPoints;
	outPoints.resize(n12 * n23);
	for (int i = 0; i < n23; i++) {
		for (int j = 0; j < n12; j++) {
			outPoints[j + i * n12].x
				= (float)inManyPoints.at<double>(0, j + i * n12) / (float)inManyPoints.at<double>(2, j + i * n12);
			outPoints[j + i * n12].y
				= (float)inManyPoints.at<double>(1, j + i * n12) / (float)inManyPoints.at<double>(2, j + i * n12);
		}
	}
	return outPoints; 
}


std::vector<cv::Point3d> interpQ43d(const std::vector<cv::Point3d>& inPoints, int n12, int n23)
{
    std::vector<cv::Point2f> p2f4(4);
	p2f4[0] = cv::Point2f(0.f, 0.f); 
	p2f4[1] = cv::Point2f(1.f, 0.f);
	p2f4[2] = cv::Point2f(1.f, 1.f);
	p2f4[3] = cv::Point2f(0.f, 1.f);
    std::vector<cv::Point2f> p2f = interpQ4(p2f4, n12, n23);
    std::vector<cv::Point3d> p3d(n12 * n23);
	for (int i = 0; i < n12 * n23; i++) {
		p3d[i] = inPoints[0] * (1.f - p2f[i].x) * (1.f - p2f[i].y)
			+ inPoints[1] * (0.f + p2f[i].x) * (1.f - p2f[i].y)
			+ inPoints[2] * (0.f + p2f[i].x) * (0.f + p2f[i].y)
			+ inPoints[3] * (1.f - p2f[i].x) * (0.f + p2f[i].y); 
	}
	return p3d; 
}




int readIntFromCin(int lb, int hb)
{
	return readIntFromIstream(cin, lb, hb); 
}

double readDoubleFromCin(double lb, double hb)
{
	return readDoubleFromIstream(cin, lb, hb);
}

string readStringFromCin()
{
	return readStringFromIstream(cin); 
}

string readStringLineFromCin()
{
	return readStringLineFromIstream(cin); 
}

int readIntFromIstream(istream & sin, int lb, int hb)
{
	int theInt;
	while (true) {
		string strbuf; sin >> strbuf;
		if (sin.eof()) return INT_MIN; // to be modified. 
		if (strbuf.length() <= 0 || strbuf[0] == '#') {
			getline(sin, strbuf); // skip the rest of this line
			continue;
		}
		try {
			theInt = stoi(strbuf);
			if (theInt < lb || theInt > hb) {
				printf("# Warning: The value should be between %d and %d. Enter again.\n", lb, hb); 
				continue;
			}
			break;
		}
		catch (...) {
			printf("# Warning: Cannot parse the input. Try again.\n");
			continue;
		}
	}
	return theInt;
}

double readDoubleFromIstream(istream & sin, double lb, double hb)
{
	double theDouble;
	while (true) {
		string strbuf; sin >> strbuf;
		if (sin.eof()) return nan("");
		if (strbuf.length() <= 0 || strbuf[0] == '#')
		{
			getline(sin, strbuf); // skip the rest of this line
			continue;
		}
		try {
			theDouble = stod(strbuf);
			if (theDouble < lb || theDouble > hb) {
				printf("# Warning: The value should be between %e and %e. Enter again.\n", lb, hb);
				continue;
			}
			break;
		}
		catch (...) {
			printf("# Warning: Cannot parse the input. Try again.\n");
			continue;
		}
	}
	return theDouble;
}

string readStringFromIstream(istream & sin)
{
	string strbuf;
	while (true) {
		sin >> strbuf;
		if (sin.eof()) return strbuf;
		if (strbuf.length() <= 0 || strbuf[0] == '#')
		{
			getline(sin, strbuf); // skip the rest of this line
			continue;
		}
		break;
	}
	return strbuf;
}

string readStringLineFromIstream(istream & sin)
{
	string strbuf;
	while (true) {
		getline(sin, strbuf);
		if (sin.eof()) return strbuf;
		if (strbuf.length() <= 0 || strbuf[0] == '#') continue;
		break;
	}
	// remove everything after '#' if any 
	int foundPunc = (int) strbuf.find('#');
	if (foundPunc > 0)
		return strbuf.substr(0, foundPunc); 
	return strbuf;
}

std::vector<std::string> readStringsFromStringLine(std::string stringLine)
{
    std::vector<std::string> strings;
	stringstream ss(stringLine);
	std::string temp;
	while (ss >> temp)
		strings.push_back(temp); 
	return strings; 
}

int preferredFourcc()
{
	int f = 0; 
	cv::VideoWriter tv;
	string fn("VideoWriterTesting.mp4"); 
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); tv.open(fn, f, 30.0, cv::Size(1920, 1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('h', '2', '6', '5'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('h', '2', '6', '4'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('m', '4', 's', '2'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('m', 'p', '4', 's'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'v', '3'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'p', 'v'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'v', '3'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'v', 'c', '1'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'v', 'p', '2'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('i', '4', '2', '0'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('i', 'y', 'u', 'v'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('m', 'p', 'g', '1'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('m', 's', 's', '1'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('m', 's', 's', '2'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('u', 'y', 'v', 'y'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'v', '1'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'v', '2'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'v', '3'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('w', 'm', 'v', 'a'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('y', 'u', 'y', '2'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('y', 'v', '1', '2'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('y', 'v', 'u', '9'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) {f = cv::VideoWriter::fourcc('y', 'v', 'y', 'u'); tv.open(fn, f, 30.0, cv::Size(1920,1080)); }
	if (!tv.isOpened()) f = 0; 
	return f;
}


//! uToCrack() calculates crack opening or sliding according to given displacement fields.
/*!
\details This function calculates crack opening or sliding according to given displacement fields.
The displacement fields are normally analyzed by template match, ECC, optical flow, or other methods.
\param u displacement field. Type:CV_32FC2 in unit of pixel. Right-ward positive. (image coordinate)
\param crack_opn crack opening field. Type:CV_32F, in unit of pixel.
\param crack_sld crack sliding field. Type:CV_32F, in unit of pixel.
\param angle assumed crack direction (0, 45, 90, 135 for specific direction, or 999 for max. of all above)
\param oper if oper (operator) is 0, crack_opn/sld = calculated value.
			if oper is 1 and crack_opn/sld is allocated in corrected sizes, crack_opn/sld = max(crack_opn/sld, calculated values)
\return 0.
*/
int uToCrack(const cv::Mat & u, cv::Mat & crack_opn, cv::Mat & crack_sld,
	int angle, int oper)
{
	// if angle == 999, pick max of angle = 0, 45, 90, and 135
	if (angle >= 999 || angle <= -999) 
	{
		uToCrack(u, crack_opn, crack_sld,   0, 0); // initialize by angle = 0
		uToCrack(u, crack_opn, crack_sld,  45, 1); // update by max( which angle = 0, which angle = 45)
		uToCrack(u, crack_opn, crack_sld,  90, 1); // update by max( current value, which angle = 90)
		uToCrack(u, crack_opn, crack_sld, 135, 1); // update by max( current value, which angle = 135)
		return 0;
	}
	// variables
	float theta; 
	// check
	if (u.rows <= 0 || u.cols <= 0 || u.type() != CV_32FC2)
	{
		cerr << "uToCrack error: Input u cannot be empty and needs to be CV_32FC2 (i.e., 13).\n";
		cerr << "  but sized " << u.rows << "-by-" << u.cols << " typed " << u.type() << endl;
		return -1;
	}
	// reallocate
	if (crack_opn.rows != u.rows || crack_opn.cols != u.cols || crack_opn.type() != CV_32F)
		crack_opn = cv::Mat::zeros(u.rows, u.cols, CV_32F) - 100; // assuming -100 is minimum 
	if (crack_sld.rows != u.rows || crack_sld.cols != u.cols || crack_sld.type() != CV_32F)
		crack_sld = cv::Mat::zeros(u.rows, u.cols, CV_32F) - 100; // assuming -100 is minimum
	// convert angle to range[0,180)
	while (angle < 0) angle += 180; 
	while (angle >= 180) angle -= 180;
	theta = (float) (angle * M_PI / 180.);
	// crack field
	cv::Point2f ua, ub, u_up, u_dn, u_lf, u_rt; 
	cv::Mat cr(2, 1, CV_32F), uab(2, 1, CV_32F); 
	float cos_theta = cos(theta), sin_theta = sin(theta); 
	cv::Mat c22(2, 2, CV_32F), c22_inv(2, 2, CV_32F);
	c22.at<float>(0, 0) = (float) cos(theta + M_PI / 2.); 
	c22.at<float>(0, 1) = (float) cos(theta); 
	c22.at<float>(1, 0) = (float) sin(theta + M_PI / 2.); 
	c22.at<float>(1, 1) = (float) sin(theta);
	c22_inv = c22.inv(); 

	for (int i = 0; i < u.rows; i++)
	{
		for (int j = 0; j < u.cols; j++)
		{
			// calculate U_up, U_down, U_left, U_right
			int I = max(1, min(u.rows - 2, i)); // I = i but must be between 1 ~ (u.rows - 2)
			int J = max(1, min(u.cols - 2, j)); // J = j but must be between 1 ~ (u.cols - 2)
			u_up = u.at<cv::Point2f>(I - 1, j);
			u_dn = u.at<cv::Point2f>(I + 1, j);
			u_lf = u.at<cv::Point2f>(i, J - 1);
			u_rt = u.at<cv::Point2f>(i, J + 1);
			// calculate UA and UB
			if (angle < 90)
			{
				ua = (-u_up * cos_theta + u_lf * sin_theta) / (cos_theta + sin_theta); 
				ub = (-u_dn * cos_theta + u_rt * sin_theta) / (cos_theta + sin_theta);
			}
			else {
				ua = (u_dn * cos_theta + u_lf * sin_theta) / (-cos_theta + sin_theta);
				ub = (u_up * cos_theta + u_rt * sin_theta) / (-cos_theta + sin_theta);
			}
			// Solve the 2x2 linear system
			uab.at<float>(0, 0) = ua.x - ub.x;
			uab.at<float>(1, 0) = ua.y - ub.y;
//			cr = c22_inv * uab; 
			cr.at<float>(0, 0) = c22_inv.at<float>(0, 0) * uab.at<float>(0, 0) + c22_inv.at<float>(0, 1) * uab.at<float>(1, 0);
			cr.at<float>(1, 0) = c22_inv.at<float>(1, 0) * uab.at<float>(0, 0) + c22_inv.at<float>(1, 1) * uab.at<float>(1, 0);
			// fill-in crack opening and sliding to Mat crack_opn and crack_sld
			if (oper == 0) {
				crack_opn.at<float>(i, j) = cr.at<float>(0, 0);
				crack_sld.at<float>(i, j) = cr.at<float>(1, 0);
			}
			else if (oper == 1) {
				crack_opn.at<float>(i, j) = max(crack_opn.at<float>(i, j), cr.at<float>(0, 0));
				crack_sld.at<float>(i, j) = max(crack_sld.at<float>(i, j), cr.at<float>(1, 0));
			}
		}
	}
	return 0;
}

//! uToCrack() calculates strain fields according to given displacement fields.
/*!
\details This function calculates strain fields according to given displacement fields.
The displacement fields are normally analyzed by template match, ECC, optical flow, or other methods.
\param u displacement field. Type:CV_32FC2 in unit of pixel. Right-ward/down-ward positive. (image coordinate)
\param exx strain field exx. Type:CV_32F, dimensionless.
\param exx strain field eyy. Type:CV_32F, dimensionless.
\param exx strain field exy. Type:CV_32F, dimensionless.
\return 0.
*/
int uToStrain(const cv::Mat & u, cv::Mat & exx, cv::Mat & eyy, cv::Mat & exy)
{
	// check
	if (u.rows <= 0 || u.cols <= 0 || u.type() != CV_32FC2)
	{
		cerr << "uToStrain error: Input u cannot be empty and needs to be CV_32FC2 (i.e., 13).\n";
		cerr << "  but sized " << u.rows << "-by-" << u.cols << " typed " << u.type() << endl;
		return -1;
	}
	// reallocate
	if (exx.rows != u.rows || exx.cols != u.cols || exx.type() != CV_32F)
		exx = cv::Mat::zeros(u.rows, u.cols, CV_32F);
	if (eyy.rows != u.rows || eyy.cols != u.cols || eyy.type() != CV_32F)
		eyy = cv::Mat::zeros(u.rows, u.cols, CV_32F);
	if (exy.rows != u.rows || exy.cols != u.cols || exy.type() != CV_32F)
		exy = cv::Mat::zeros(u.rows, u.cols, CV_32F);

	// strain field
	cv::Point2f u_up, u_dn, u_lf, u_rt;
	for (int i = 0; i < u.rows; i++)
	{
		for (int j = 0; j < u.cols; j++)
		{
			// calculate U_up, U_down, U_left, U_right
			int I = max(1, min(u.rows - 2, i)); // I = i but must be between 1 ~ (u.rows - 2)
			int J = max(1, min(u.cols - 2, j)); // J = j but must be between 1 ~ (u.cols - 2)
			u_up = u.at<cv::Point2f>(I - 1, j);
			u_dn = u.at<cv::Point2f>(I + 1, j);
			u_lf = u.at<cv::Point2f>(i, J - 1);
			u_rt = u.at<cv::Point2f>(i, J + 1);
			// calculate strain
			exx.at<float>(i, j) = (u_rt.x - u_lf.x) / 2.0f; 
			eyy.at<float>(i, j) = (u_dn.y - u_up.y) / 2.0f; 
			exy.at<float>(i, j) = (u_rt.y - u_lf.y) / 2.0f + (u_dn.x - u_up.x) / 2.0f; 
		}
	}
	return 0;
}

cv::Mat sobel_xy(const cv::Mat & src)
{
	cv::Mat src_gray;
	cv::Mat grad;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	if (!src.data)
	{
		return cv::Mat();
	}

	/// Convert it to gray
	if (src.channels() > 1)
		cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
	else
		src_gray = src;

	/// Generate grad_x and grad_y
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);

	/// Gradient Y
	//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);

	/// Total Gradient (approximate)
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
	return grad;
}

void imshow_resize(string winname, cv::Mat img, double factor)
{
	cv::Mat tmp;
	cv::resize(img, tmp, cv::Size(0, 0), factor, factor);
	cv::imshow(winname, tmp);
}

void imshow_resize(string winname, cv::Mat img, cv::Size maxWinSize)
{
	cv::Mat tmp;
	double factorX, factorY, factor;
	factorY = maxWinSize.height * 1.0 / img.rows;
	factorX = maxWinSize.width * 1.0 / img.cols;
	factor = factorY > factorX ? factorX : factorY; 
	cv::resize(img, tmp, cv::Size(0, 0), factor, factor, cv::INTER_LANCZOS4);
	cv::imshow(winname, tmp);
}

//! tryOpcvCapFocusExposureGain() allows user to find the best focus/exposure/gain values
// by trial and error interactively (thruogh keyboard)
/*!
\detail 
hotkey: 0~9 - set active camera id 
hotkey: a - increase focus value by 0.01
hotkey: z - decrease focus value by 0.01
hotkey: s - increase exposure value by 0.01
hotkey: x - decrease exposure value by 0.01
hotkey: r - trial resolution by console interaction
hotkey: t - set region of imshow (as a cv::Rect: x0 y0 width height) through console interaction
hotkey: ijkl - movement of showing zoom window
hotkey: p - save current image to file 
hotkey: b - switch sobel on/off
hotkey: o - switch continuous imshow on/off
hotkey: 
\param result adjusted value by user. [0]:cam_id, [1]:focus, [2]:exposure, [3] width, [4] height
\return 0.
*/
int tryOpcvCapFocusExposure(std::vector<double> & result)
{
    // fill special camera string here. For example, if pressing "9" is a special camera, define your capStr[9].
    int cam_id = 0;
	double focus = 0.0, exposure = 0.0;
	int width = 3840, height = 2160; 
	int zoom_w = 640, zoom_h = 480;
	int full_w = 800; // height is calculated based on actual aspect ratio
	cv::Rect imshowRect(0, 0, zoom_w, zoom_h);
	cv::VideoCapture cam;
	bool cam_opened = false;
	bool img_ok = false;
	bool sobel = false; 
	int my_waitKeyTime = 0; // 0: waiting for key, 200: refresh every 0.2 sec.
    string camStr;

    cout << "# Input initial VideoCapture (0 to 8 for normal camera index, 9 for string): \n";
    cam_id = readIntFromIstream(std::cin);
    if (cam_id == 9)
    {
        std::vector<string> camStrDb;
        camStrDb.push_back("v4l2src ! video/x-raw,format=NV12,width=1640,height=1232 ! videoconvert ! appsink");
        cout << "# Input string argument for VideoCapture: \n";
        for (int i = 0; i < (int) camStrDb.size(); i++)
        {
            printf("#  (%d) %s:\n", i, camStrDb[i].c_str());
        }
        camStr = camStrDb[readIntFromIstream(std::cin)];
        cam.open(camStr);
    }

    while (true)
	{
		cv::Mat img = cv::Mat::zeros(imshowRect.height, imshowRect.width, CV_8UC3); 
		// Set camera 
        if (cam.isOpened() == false) {
            if (cam_id >= 0 && cam_id <= 8)
                cam_opened = cam.open(cam_id, cv::CAP_DSHOW);
            // camera 9 is special case
            if (cam_id == 9)
            {
                cam_opened = cam.open(camStr);
            }
        }
		if (cam_opened == false)
		{
            printf("Failed to open camera %d.\n", cam_id);
            cout << "Trying " << "v4l2src ! video/x-raw,format=NV12,width=1640,height=1232 ! videoconvert ! appsink" << " ...\n";
            cam_opened = cam.open("v4l2src ! video/x-raw,format=NV12,width=1640,height=1232 ! videoconvert ! appsink");
            if (cam_opened == false) {
                cerr << "Still failed to open camera.\n";
            }
		}
		if (cam_opened == true)
		{
			cam.set(cv::CAP_PROP_AUTOFOCUS, 0.0); // disable autofocus
			cam.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.0); // disable auto-exposure
			cam.set(cv::CAP_PROP_FOCUS, focus); 
			cam.set(cv::CAP_PROP_EXPOSURE, exposure); 
			if (height != (int)cam.get(cv::CAP_PROP_FRAME_HEIGHT)) 
				cam.set(cv::CAP_PROP_FRAME_HEIGHT, (double)height);
			if (width != (int)cam.get(cv::CAP_PROP_FRAME_WIDTH))
				cam.set(cv::CAP_PROP_FRAME_WIDTH, (double)width);
		}
		// grab image
		if (cam_opened == true && cam.isOpened() == true)
		{
			img_ok = cam.grab(); 
			if (img_ok)
				img_ok = cam.retrieve(img);
			if (img_ok && sobel)
				img = sobel_xy(img); 
		}
		// show image
		if (img_ok == true)
		{
			cv::imshow("Zoom Camera", img(imshowRect));
			int full_h = (int)(full_w * img.rows * 1.0 / img.cols + .5); 
			cv::Mat imgSmall; 
			cv::resize(img, imgSmall, cv::Size(full_w, full_h)); 
			cv::imshow("Overall view", imgSmall);
		}
		// get info from camera (assuring properties are actual properties)
		// print information on console
		printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		printf("Camera %d: Focus:%7.2f, Exposure:%7.2f, Width:%4d, Height:%4d. Rect(%4d,%4d,%4d,%4d)",
			cam_id, 
			cam.get(cv::CAP_PROP_FOCUS), 
			cam.get(cv::CAP_PROP_EXPOSURE), 
			(int) cam.get(cv::CAP_PROP_FRAME_WIDTH),
			(int) cam.get(cv::CAP_PROP_FRAME_HEIGHT),
			imshowRect.x, imshowRect.y, imshowRect.width, imshowRect.height); 
		// waiting for user key-in
		int ukey = cv::waitKey(my_waitKeyTime); 
		if (ukey > 0) printf("\nYou pressed %d\n", ukey); 
		//   if change camera
		if (ukey >= (int) '0' && ukey <= (int) '9')
		{
			cam_id = ukey - (int)'0';
			printf("Reset camera ID to %d\n", cam_id); 
			cam.release(); 
		}
		//   if change focus
		if (ukey == (int)'a') 
		{
			focus += 1;
			printf("Set trial focus to %7.2f\n", focus);
		}
		if (ukey == (int)'z') 
		{
			focus -= 1;
			printf("Set trial focus to %7.2f\n", focus);
		}

		//   if change exposure
		if (ukey == (int)'s') 
		{
			exposure += 1;
			printf("Set trial exposure to %7.2f\n", exposure);
		}
		if (ukey == (int)'x') 
		{
			exposure -= 1;
			printf("Set trial exposure to %7.2f\n", exposure);
		}
		//   if moving zoom window
		if (ukey == (int)'i') {
			imshowRect.y -= 20; 
			if (imshowRect.y < 0) imshowRect.y = 0;
		}
		if (ukey == (int)'j') {
			imshowRect.x -= 20;
			if (imshowRect.x < 0) imshowRect.x = 0;
		}
		if (ukey == (int)'k') {
			imshowRect.y += 20;
			if (imshowRect.y > img.rows - imshowRect.height) imshowRect.y = img.rows - imshowRect.height;
		}
		if (ukey == (int)'l') {
			imshowRect.x += 20;
			if (imshowRect.x > img.cols - imshowRect.width) imshowRect.x = img.cols - imshowRect.width;
		}

		//   if change trial camera resolution
		if (ukey == (int)'r')
		{
			printf("Input your preferred resolution (current: %d %d): ", width, height);
			std::cin.clear(); 
			cin >> width >> height;
			std::cin.clear();
		}
		//   if change imshow range (rect)
		if (ukey == (int) 't')
		{
			printf("Input your preferred showing rect (current: %d %d %d %d): ", imshowRect.x, imshowRect.y, imshowRect.width, imshowRect.height);
			std::cin.clear();
			cin >> imshowRect.x >> imshowRect.y >> imshowRect.width >> imshowRect.height;
			std::cin.clear();
		}
		if (ukey == (int) 'b')
		{
			sobel = !sobel; 
			printf("Switch sobel to %d.\n", sobel); 
		}
		if (ukey == (int) 'p')
		{
			std::string fname;
			printf("Enter file name to save the current photo: ");
			fname = readStringLineFromCin();
			cv::imwrite(fname, img); 
		}
		if (ukey == (int) 'o')
		{
			if (my_waitKeyTime == 0)
				my_waitKeyTime = 200;
			else
				my_waitKeyTime = 0;
		}
		//   if ok and quit .
		if (ukey == (int)'q')
		{
			if (result.size() < 5) result.resize(5); 
			result[0] = (double)cam_id;
			result[1] = focus;
			result[2] = exposure;
			result[3] = (double)width;
			result[4] = (double)height;
			cv::destroyWindow("Zoom Camera");
			cv::destroyWindow("Overall view");
			return 0;
		}
	}
	return 0;
}

//
template <typename T>
std::vector<size_t> sort_index(const std::vector<T> &v) {
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
  return idx;
}

//
// This function is designed for brute-force trial-and-error for an n-dimensional problem.
//
// Example:
// Input:
//     nDim = 3,
//     nOptions = {2, 3, 2}
//     composition = {0, 0, 0}
// Output: (index) 0
// More details of the example
// index | composition when nDim = 3
//       |  and nOptions = {4, 3, 2}
// --------------------
//    0  | 0   0   0
//    1  | 1   0   0
//    2  | 2   0   0
//    3  | 3   0   0
//    4  | 0   1   0
//    5  | 1   1   0
//    6  | 2   1   0
//    7  | 3   1   0
//    8  | 0   2   0
//    9  | 1   2   0
//   10  | 2   2   0
//   11  | 3   2   0
//   12  | 0   0   1
//   13  | 1   0   1
//   14  | 2   0   1
//   15  | 3   0   1
//   16  | 0   1   1
//   17  | 1   1   1
//   18  | 2   1   1
//   19  | 3   1   1
//   20  | 0   2   1
//   21  | 1   2   1
//   22  | 2   2   1
//   23  | 3   2   1
int indexOfComposition(int nDim, const int* nOptions, const int* composition)
{
    int i = 0, mul = 1;
    for (int k = 0; k < nDim; k++)
    {
        i += composition[k] * mul;
        mul *= nOptions[k];
    }
    return i;
}

void compositionOfIndex(int nDim, const int* nOptions, int index, int* composition)
{
    for (int k = 0; k < nDim; k++)
    {
        composition[k] = index % nOptions[k];
        index = index / nOptions[k];
    }
}

// The same function name but using std::vector<int> rather than int*.
int indexOfComposition(const std::vector<int> & nOptions, const std::vector<int> & composition)
{
    int nDim = (int) nOptions.size();
    if (nDim > (int) composition.size()) nDim = (int) composition.size();
    return indexOfComposition(nDim, nOptions.data(), composition.data());
}
// The same function name but using std::vector<int> rather than int*.
void compositionOfIndex(const std::vector<int> & nOptions, int index, std::vector<int> & composition)
{
    int nDim = (int) nOptions.size();
    if (composition.size() < (size_t) nDim)
        composition.resize((size_t) nDim);
    compositionOfIndex(nDim, nOptions.data(), index, composition.data());
}

double normOfVector(const std::vector<double> & x)
{
    double norm = 0.0;
    for (size_t i = 0; i < x.size(); i++)
        norm += x[i] * x[i];
    norm = sqrt(norm);
    return norm;
}
void printVector(std::vector<int> x)
{
    for (int i = 0; i < (int) x.size(); i++)
        printf(" %2d ", x[i]);
}

void printVector(std::vector<double> x)
{
    for (int i = 0; i < (int) x.size(); i++)
        printf(" %10.3e ", x[i]);
}

//
int bruteForceOneLevel(int nIn, int nOut, int (*func)(const std::vector<double> &, std::vector<double> &, const void *),
                       const std::vector<double> xlowerBounds, const std::vector<double> xupperBounds,
                       const std::vector<int> nTrialx,
                       const void* misc,
                       int nBest,
                       std::vector<std::vector<double>> & optxs, std::vector<std::vector<double>> & optys, int debug)
{
    int nEval = 1;
    std::vector<double> bestNorms(nBest, -1.);
    std::vector<double> xj(nIn, 0.), yj(nOut, 0.);
    for (int i = 0; i < (int) nTrialx.size(); i++)
        nEval *= nTrialx[i];
    if (debug > 0) cout << "nEval: " << nEval << endl;
    // allocate optx and opty
    optxs.resize(nBest, std::vector<double>(nIn, 0.0));
    optys.resize(nBest, std::vector<double>(nOut, 0.0));
    // brute-force loop
    for (int i = 0; i < nEval; i++)
    {
        // by index (i), generate the composotion of arguments
        std::vector<int> composition;
        compositionOfIndex(nTrialx, i, composition);
        // generate input vector x
        for (int j = 0; j < nIn; j++)
            xj[j] = xlowerBounds[j] + (composition[j] * 1. / (nTrialx[j] - 1)) * (xupperBounds[j] - xlowerBounds[j]);
        // evaluate function
        func(xj, yj, (const void*) misc);
        // check best
        double ynorm = normOfVector(yj);
        for (int k = 0; k < nBest; k++)
        {
            if (ynorm < bestNorms[k] || bestNorms[k] < 0.)
            {
                if (debug > 0)
                {
                    cout << "Composition " << i << " is selected: ";
                    printVector(xj); cout << " "; printVector(yj); cout << "\n";
                }
                // swap ynorm <--> bestNorms[k]
                double tmp = ynorm;
                ynorm = bestNorms[k];
                bestNorms[k] = tmp;
                // swap xj <--> optxs[k]
                std::vector<double> tmpvec = xj;
                xj = optxs[k];
                optxs[k] = tmpvec;
                // swap yj <--> optys[k].
                tmpvec = yj;
                yj = optys[k];
                optys[k] = tmpvec;
                // debug
            }
        }
    }

    return 0;
}


int q4MembraneStrains(const cv::Mat& _x0, const cv::Mat& _x1, cv::Mat& strains, double poisson)
{
	// Step 0: Convert data to x0(3, 4, CV_64F) and x1(3, 4, CV_64F)
	cv::Mat x0, x1;
	// Check x0
	if (_x0.type() == CV_32F && _x0.rows == 4 && _x0.cols == 3) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x0.at<double>(i, j) = (double)_x0.at<float>(j, i);
	}
	if (_x0.type() == CV_64F && _x0.rows == 4 && _x0.cols == 3) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x0.at<double>(i, j) = (double)_x0.at<double>(j, i);
	}
	if (_x0.type() == CV_32FC3 && _x0.rows == 4 && _x0.cols == 1) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int j = 0; j < 4; j++) {
			x0.at<double>(0, j) = (double)_x0.at<cv::Point3f>(j).x;
			x0.at<double>(1, j) = (double)_x0.at<cv::Point3f>(j).y;
			x0.at<double>(2, j) = (double)_x0.at<cv::Point3f>(j).z;
		}
	}
	if (_x0.type() == CV_64FC3 && _x0.rows == 4 && _x0.cols == 1) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int j = 0; j < 4; j++) {
			x0.at<double>(0, j) = (double)_x0.at<cv::Point3d>(j).x;
			x0.at<double>(1, j) = (double)_x0.at<cv::Point3d>(j).y;
			x0.at<double>(2, j) = (double)_x0.at<cv::Point3d>(j).z;
		}
	}
	if (_x0.type() == CV_32F && _x0.rows == 3 && _x0.cols == 4) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x0.at<double>(i, j) = (double)_x0.at<float>(i, j);
	}
	if (_x0.type() == CV_64F && _x0.rows == 3 && _x0.cols == 4) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x0.at<double>(i, j) = (double)_x0.at<double>(i, j);
	}
	if (_x0.type() == CV_32FC4 && (_x0.rows * _x0.cols) == 3) {
		x0 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++) 
			for (int c = 0; c < 4; c++)
				x0.at<double>(i, c) = (double)_x0.at<cv::Vec4f>(i)[c];
	}
	if (_x0.type() == CV_64FC4 && (_x0.rows * _x0.cols) == 3) {
		x0 = cv::Mat(3, 4, CV_64F);
			for (int i = 0; i < 3; i++)
				for (int c = 0; c < 4; c++)
					x0.at<double>(i, c) = (double)_x0.at<cv::Vec4d>(i)[c];
	}
	if (x0.cols <= 0 || x0.rows <= 0) {
		fprintf(stderr, "q4MembraneStrains: Error: Check x0 data type and dimension.\n"
			"x0(%d,%d,%d)\n", _x0.rows, _x0.cols, _x0.type());
	}
	// Check x1
	if (_x1.type() == CV_32F && _x1.rows == 4 && _x1.cols == 3) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x1.at<double>(i, j) = (double)_x1.at<float>(j, i);
	}
	if (_x1.type() == CV_64F && _x1.rows == 4 && _x1.cols == 3) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x1.at<double>(i, j) = (double)_x1.at<double>(j, i);
	}
	if (_x1.type() == CV_32FC3 && _x1.rows == 4 && _x1.cols == 1) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int j = 0; j < 4; j++) {
			x1.at<double>(0, j) = (double)_x1.at<cv::Point3f>(j).x;
			x1.at<double>(1, j) = (double)_x1.at<cv::Point3f>(j).y;
			x1.at<double>(2, j) = (double)_x1.at<cv::Point3f>(j).z;
		}
	}
	if (_x1.type() == CV_64FC3 && _x1.rows == 4 && _x1.cols == 1) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int j = 0; j < 4; j++) {
			x1.at<double>(0, j) = (double)_x1.at<cv::Point3d>(j).x;
			x1.at<double>(1, j) = (double)_x1.at<cv::Point3d>(j).y;
			x1.at<double>(2, j) = (double)_x1.at<cv::Point3d>(j).z;
		}
	}
	if (_x1.type() == CV_32F && _x1.rows == 3 && _x1.cols == 4) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x1.at<double>(i, j) = (double)_x1.at<float>(i, j);
	}
	if (_x1.type() == CV_64F && _x1.rows == 3 && _x1.cols == 4) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				x1.at<double>(i, j) = (double)_x1.at<double>(i, j);
	}
	if (_x1.type() == CV_32FC4 && (_x1.rows * _x1.cols) == 3) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int c = 0; c < 4; c++)
				x1.at<double>(i, c) = (double)_x1.at<cv::Vec4f>(i)[c];
	}
	if (_x1.type() == CV_64FC4 && (_x1.rows * _x1.cols) == 3) {
		x1 = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++)
			for (int c = 0; c < 4; c++)
				x1.at<double>(i, c) = (double)_x1.at<cv::Vec4d>(i)[c];
	}
	if (x1.cols <= 0 || x1.rows <= 0) {
		fprintf(stderr, "q4MembraneStrains: Error: Check x1 data type and dimension.\n"
			"_x1(%d,%d,%d)\n", _x1.rows, _x1.cols, _x1.type());
	}

	// Step 1: determine R0 for x0 (R matrix of initial state) 
	double dx1, dy1, dz1, dx2, dy2, dz2; 
	cv::Vec3d vx, vy, vz;
	cv::Mat R0(3, 3, CV_64F), R1(3, 3, CV_64F), x0Local, x1Local;
	dx1 = x0.at<double>(0, 0) - x0.at<double>(0, 1); 
	dy1 = x0.at<double>(1, 0) - x0.at<double>(1, 1);
	dz1 = x0.at<double>(2, 0) - x0.at<double>(2, 1);
	dx2 = x0.at<double>(0, 3) - x0.at<double>(0, 2);
	dy2 = x0.at<double>(1, 3) - x0.at<double>(1, 2);
	dz2 = x0.at<double>(2, 3) - x0.at<double>(2, 2);
	vx = 0.5 * cv::Vec3d(dx1, dy1, dz1) + 0.5 * cv::Vec3d(dx2, dy2, dz2); 
	dx1 = x0.at<double>(0, 0) - x0.at<double>(0, 3);
	dy1 = x0.at<double>(1, 0) - x0.at<double>(1, 3);
	dz1 = x0.at<double>(2, 0) - x0.at<double>(2, 3);
	dx2 = x0.at<double>(0, 1) - x0.at<double>(0, 2);
	dy2 = x0.at<double>(1, 1) - x0.at<double>(1, 2);
	dz2 = x0.at<double>(2, 1) - x0.at<double>(2, 2);
	vy = 0.5 * cv::Vec3d(dx1, dy1, dz1) + 0.5 * cv::Vec3d(dx2, dy2, dz2);
	vz = vx.cross(vy); 
	vy = vz.cross(vx); 
	vx = vx / sqrt(vx[0] * vx[0] + vx[1] * vx[1] + vx[2] * vx[2]);
	vy = vy / sqrt(vy[0] * vy[0] + vy[1] * vy[1] + vy[2] * vy[2]);
	vz = vz / sqrt(vz[0] * vz[0] + vz[1] * vz[1] + vz[2] * vz[2]);
	R0.at<double>(0, 0) = vx[0]; R0.at<double>(0, 1) = vy[0]; R0.at<double>(0, 2) = vz[0];
	R0.at<double>(1, 0) = vx[1]; R0.at<double>(1, 1) = vy[1]; R0.at<double>(1, 2) = vz[1];
	R0.at<double>(2, 0) = vx[2]; R0.at<double>(2, 1) = vy[2]; R0.at<double>(2, 2) = vz[2];
	x0Local = R0.inv() * x0; 

	dx1 = x1.at<double>(0, 0) - x1.at<double>(0, 1);
	dy1 = x1.at<double>(1, 0) - x1.at<double>(1, 1);
	dz1 = x1.at<double>(2, 0) - x1.at<double>(2, 1);
	dx2 = x1.at<double>(0, 3) - x1.at<double>(0, 2);
	dy2 = x1.at<double>(1, 3) - x1.at<double>(1, 2);
	dz2 = x1.at<double>(2, 3) - x1.at<double>(2, 2);
	vx = 0.5 * cv::Vec3d(dx1, dy1, dz1) + 0.5 * cv::Vec3d(dx2, dy2, dz2);
	dx1 = x1.at<double>(0, 0) - x1.at<double>(0, 3);
	dy1 = x1.at<double>(1, 0) - x1.at<double>(1, 3);
	dz1 = x1.at<double>(2, 0) - x1.at<double>(2, 3);
	dx2 = x1.at<double>(0, 1) - x1.at<double>(0, 2);
	dy2 = x1.at<double>(1, 1) - x1.at<double>(1, 2);
	dz2 = x1.at<double>(2, 1) - x1.at<double>(2, 2);
	vy = 0.5 * cv::Vec3d(dx1, dy1, dz1) + 0.5 * cv::Vec3d(dx2, dy2, dz2);
	vz = vx.cross(vy);
	vy = vz.cross(vx);
	vx = vx / sqrt(vx[0] * vx[0] + vx[1] * vx[1] + vx[2] * vx[2]);
	vy = vy / sqrt(vy[0] * vy[0] + vy[1] * vy[1] + vy[2] * vy[2]);
	vz = vz / sqrt(vz[0] * vz[0] + vz[1] * vz[1] + vz[2] * vz[2]);
	R1.at<double>(0, 0) = vx[0]; R1.at<double>(0, 1) = vy[0]; R1.at<double>(0, 2) = vz[0];
	R1.at<double>(1, 0) = vx[1]; R1.at<double>(1, 1) = vy[1]; R1.at<double>(1, 2) = vz[1];
	R1.at<double>(2, 0) = vx[2]; R1.at<double>(2, 1) = vy[2]; R1.at<double>(2, 2) = vz[2];
	x1Local = R1.inv() * x1;
	std::cout << x1Local << endl;

	// Step 2: calculate u vector (8 by 1)
	cv::Mat uvec(8, 1, CV_64F); 
	uvec.at<double>(0) = x1Local.at<double>(0, 0) - x0Local.at<double>(0, 0);
	uvec.at<double>(1) = x1Local.at<double>(1, 0) - x0Local.at<double>(1, 0);
	uvec.at<double>(2) = x1Local.at<double>(0, 1) - x0Local.at<double>(0, 1);
	uvec.at<double>(3) = x1Local.at<double>(1, 1) - x0Local.at<double>(1, 1);
	uvec.at<double>(4) = x1Local.at<double>(0, 2) - x0Local.at<double>(0, 2);
	uvec.at<double>(5) = x1Local.at<double>(1, 2) - x0Local.at<double>(1, 2);
	uvec.at<double>(6) = x1Local.at<double>(0, 3) - x0Local.at<double>(0, 3);
	uvec.at<double>(7) = x1Local.at<double>(1, 3) - x0Local.at<double>(1, 3);
	std::cout << uvec << endl;

	// Step 3: start r loop and s loop 
	double r5[5] = { 0, .57735, -0.57735, -0.57735, 0.57735 };
	double s5[5] = { 0, .57735, 0.57735, -0.57735, -0.57735 };
	strains = cv::Mat(5, 7, CV_64F); 
	for (int i = 0; i < 5; i++) {
		double r = r5[i];
		double s = s5[i];
		// Step 4: dudxdy, dvdxdy, J
		cv::Mat dudxdy(2, 4, CV_64F); 
		cv::Mat dvdxdy(2, 4, CV_64F);
		cv::Mat J(2, 2, CV_64F); 
		double _x1, _x2, _x3, _x4, _y1, _y2, _y3, _y4;
		_x1 = x0Local.at<double>(0, 0); 
		_x2 = x0Local.at<double>(0, 1);
		_x3 = x0Local.at<double>(0, 2);
		_x4 = x0Local.at<double>(0, 3);
		_y1 = x0Local.at<double>(1, 0);
		_y2 = x0Local.at<double>(1, 1);
		_y3 = x0Local.at<double>(1, 2);
		_y4 = x0Local.at<double>(1, 3);
		J.at<double>(0, 0) = 0.25 * ((1 + s) * _x1 - (1 + s) * _x2 - (1 - s) * _x3 + (1 - s) * _x4);
		J.at<double>(1, 0) = 0.25 * ((1 + r) * _x1 + (1 - r) * _x2 - (1 - r) * _x3 - (1 + r) * _x4);
		J.at<double>(0, 1) = 0.25 * ((1 + s) * _y1 - (1 + s) * _y2 - (1 - s) * _y3 + (1 - s) * _y4);
		J.at<double>(1, 1) = 0.25 * ((1 + r) * _y1 + (1 - r) * _y2 - (1 - r) * _y3 - (1 + r) * _y4);
		std::cout << "J:\n" << J << endl;
		dudxdy.at<double>(0, 0) = (1 + s);
		dudxdy.at<double>(0, 1) = -(1 + s);
		dudxdy.at<double>(0, 2) = -(1 - s);
		dudxdy.at<double>(0, 3) = (1 - s);
		dudxdy.at<double>(1, 0) = (1 + r);
		dudxdy.at<double>(1, 1) = 1 - r;
		dudxdy.at<double>(1, 2) = -(1 - r);
		dudxdy.at<double>(1, 3) = -(1 + r);
		std::cout << "dudxdy:\n" << dudxdy << endl;
		dudxdy = 0.25 * J.inv() * dudxdy; 
		std::cout << "dudxdy:\n" << dudxdy << endl;
		dvdxdy.at<double>(0, 0) = (1 + s);
		dvdxdy.at<double>(0, 1) = -(1 + s);
		dvdxdy.at<double>(0, 2) = -(1 - s);
		dvdxdy.at<double>(0, 3) = (1 - s);
		dvdxdy.at<double>(1, 0) = (1 + r);
		dvdxdy.at<double>(1, 1) = 1 - r;
		dvdxdy.at<double>(1, 2) = -(1 - r);
		dvdxdy.at<double>(1, 3) = -(1 + r);
		std::cout << "dvdxdy:\n" << dvdxdy << endl;
		dvdxdy = 0.25 * J.inv() * dvdxdy;
		std::cout << "dvdxdy:\n" << dvdxdy << endl;
		// Step 5: Matrix B
		cv::Mat B = cv::Mat::zeros(3, 8, CV_64F); 
		B.at<double>(0, 0) = dudxdy.at<double>(0, 0);
		B.at<double>(0, 2) = dudxdy.at<double>(0, 1);
		B.at<double>(0, 4) = dudxdy.at<double>(0, 2);
		B.at<double>(0, 6) = dudxdy.at<double>(0, 3);

		B.at<double>(1, 1) = dudxdy.at<double>(1, 0);
		B.at<double>(1, 3) = dudxdy.at<double>(1, 1);
		B.at<double>(1, 5) = dudxdy.at<double>(1, 2);
		B.at<double>(1, 7) = dudxdy.at<double>(1, 3);

		B.at<double>(2, 0) = dudxdy.at<double>(1, 0);
		B.at<double>(2, 2) = dudxdy.at<double>(1, 1);
		B.at<double>(2, 4) = dudxdy.at<double>(1, 2);
		B.at<double>(2, 6) = dudxdy.at<double>(1, 3);

		B.at<double>(2, 1) = dudxdy.at<double>(0, 0);
		B.at<double>(2, 3) = dudxdy.at<double>(0, 1);
		B.at<double>(2, 5) = dudxdy.at<double>(0, 2);
		B.at<double>(2, 7) = dudxdy.at<double>(0, 3);

		// Step 6: calculate exx, eyy, exy
		cv::Mat strainxy = B * uvec;
		double exx = strainxy.at<double>(0); 
		double eyy = strainxy.at<double>(1);
		double exy = strainxy.at<double>(2);

		// Step 7: calculate e1, e2, gammas (max shear strain), eqv (equilvalent von mises strain)
		strains.at<double>(i, 0) = exx;
		strains.at<double>(i, 1) = eyy;
		strains.at<double>(i, 2) = exy;
		double gammas = sqrt((pow((exx - eyy) / 2., 2) + pow(exy / 2, 2)));
		double e1 = (exx + eyy) / 2 + gammas; // principal strain along the surface (in-plane)
		double e2 = (exx + eyy) / 2 - gammas; // principal strain along the surface (in-plane)
		double e3 = poisson* (-e1 - e2);
		strains.at<double>(i, 3) = e1; 
		strains.at<double>(i, 4) = e2;
		strains.at<double>(i, 5) = gammas;
		double _exx = +2. / 3. * e1 - 1. / 3. * e2 - 1. / 3. * e3;
		double _eyy = -1. / 3. * e1 + 2. / 3. * e2 - 1. / 3. * e3; 
		double _ezz = -1. / 3. * e1 - 1. / 3. * e2 + 2. / 3. * e3; 
		double _eeq = 2. / 3. * sqrt(1.5 * (_exx * _exx + _eyy * _eyy + _ezz * _ezz)); // along principal axes shear strains are ignored. 
		strains.at<double>(i, 6) = _eeq; 
	}

	std::cout << "strains:\n" << strains << endl;

	return 0;
}


