#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "sync.h"
#include "impro_util.h"

// float guess = 12.234;
// int searchRange = 15; // from -searchRange to +searchRange 
// int winCenter = n / 2 + 25;
// int winSize = 50;
// double fact = 100.0;
// bool oppositeDirection = true;

/*!
  \brief apply synchroization by generating a new history
  \param t0 time series. Must be 1 x N, float (CV_32F). (can be cv::Mat(1, N, CV_32F, (void*) data)
  \param tc time series. Must be 1 x N, float (CV_32F). (can be cv::Mat(1, N, CV_32F, (void*) data)
  \param lag estimated time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \return 0
*/
int applySynchronization(
	const cv::Mat & t0,
	cv::Mat & tc,
	float lag)
{
	int n = t0.cols;
	float borderValue = 0.f;
	if (borderValue > 0.f)
		borderValue = t0.at<float>(0, 0);
	else
		borderValue = t0.at<float>(0, n - 1);
	cv::Mat map1, map2;
	tc = cv::Mat(t0.size(), t0.type());
	map1 = cv::Mat::zeros(t0.size(), t0.type()) - lag;
	for (int i = 1; i < map1.cols; i++) map1.at<float>(0, i) += i;
	map2 = cv::Mat::zeros(t0.size(), t0.type());
	cv::remap(t0, tc, map1, map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, borderValue);
	return 0;
}

/*! This function synchroize two time series (find the time lag of series 2)
  \brief synchroize two time series (find the time lag of series 2)
  \param t1 time series 1. Must be 1 x N, float (CV_32F).
  \param t2 time series 2. Must be 1 x N, float (CV_32F)
  \param lag estimated time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \param t2sync synchronized t2 (1 x N, float (CV_32F))
  \param xcorr_lag  x data (lag) of cross correlation function (1 x N, float (CV_32F))
  \param xcorr_coef y data (coefficient) of cross correlation function (1 x N, float (CV_32F))
  \param guessT2Lag user initial guess of time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \param searchRange range (e.g., if 5, will search from (guessT2Lag - searchRange) to (guessT2Lag + searchRange).) (if -1, default is 5% of t1 length)
  \param winCenter matching window center (if -1, default is center of t1 series)
  \param winSize maching window size (if -1, default is half of t1 length)
  \param prec precision of matching (if 0.01, data length will be resized to 100 times longer before matching)
  \param oppoDir if true, two series are supposed to run in opposite way, when t1 goes up, t2 should go down.
  \param outPathMatlabScript path for output file of matlab script for visualization
  \return correlation coefficient
*/

float syncTwoSeries(
	const cv::Mat & t1,
	const cv::Mat & t2,
	float & lag,
	cv::Mat & t2sync,
	cv::Mat & xcorr_lag,
	cv::Mat & xcorr_coef,
	int   guessT2Lag,
	int   searchRange,
	int   winCenter,
	int   winSize,
	float prec,
	bool oppoDir,
	std::string outPathMatlabScript
)
{
	// Check

	// Default value
	int n1 = t1.cols; 
	int n2 = t2.cols; 
	if (searchRange < 0) searchRange = (int)(0.05 * n1 + 0.5); 
	if (winCenter < 0) winCenter = n1 / 2;
	if (winSize <= 0) winSize = n1 / 2; 
	if (prec <= 0) prec = 0.01f; 

	//	double minA1, maxA1;
	double minT2, maxT2, minS2, maxS2;
	//	cv::Point minS1Point, maxS1Point;
	cv::Point minT2Point, maxT2Point, minS2Point, maxS2Point;

	// output file for checking and debugging
	outPathMatlabScript = appendSlashOrBackslashAfterDirectoryIfNecessary(outPathMatlabScript);
	std::ofstream ofSync(outPathMatlabScript + "of_sync.m");

	// cross correlation range (left bound X, width W) 
	int tmpltX = winCenter - winSize / 2;
	int tmpltW = winSize;
	int searcX = (int)(winCenter - guessT2Lag - winSize / 2 - searchRange + .5f);
	int searcW = winSize + 2 * searchRange;
	if (tmpltX < 0) tmpltX = 0;
	if (tmpltW > t1.cols - tmpltX) tmpltW = t1.cols - tmpltX;
	if (searcX < 0) searcX = 0;
	if (searcW > t1.cols - searcX) searcW = t2.cols - searcX;

	// cross correlation Mat (1 * W) 
	cv::Mat tmplt; t1(cv::Rect(tmpltX, 0, tmpltW, 1)).copyTo(tmplt);
	cv::Mat searc; t2(cv::Rect(searcX, 0, searcW, 1)).copyTo(searc);

	// cross correlation Mat scaled to 0~255 and U8 format 
	if (oppoDir == true) tmplt = -tmplt;
	cv::minMaxLoc(tmplt, &minT2, &maxT2, &minT2Point, &maxT2Point);
	cv::minMaxLoc(searc, &minS2, &maxS2, &minS2Point, &maxS2Point);
	//double minT2S2 = cv::min(minT2, minS2); 
	//double maxT2S2 = cv::max(maxT2, maxS2);

	cv::Mat tmpltU8(tmplt.size(), CV_8U), searcU8(searc.size(), CV_8U);
	for (int i = 0; i < tmpltU8.cols; i++) {
		tmpltU8.at<uchar>(0, i) = (uchar)((tmplt.at<float>(0, i) - minT2) * 255. / (maxT2 - minT2 + 1e-12));
	}
	for (int i = 0; i < searcU8.cols; i++) {
		searcU8.at<uchar>(0, i) = (uchar)((searc.at<float>(0, i) - minS2) * 255. / (maxS2 - minS2 + 1e-12));
	}

	//	cv::minMaxLoc(tmpltU8, &minS2, &maxS2, &minS2Point, &maxS2Point);
	//	cv::minMaxLoc(searcU8, &minS2, &maxS2, &minS2Point, &maxS2Point);
	cv::resize(tmpltU8, tmpltU8, cv::Size(0, 0), 1. / prec, 1.0, cv::INTER_LANCZOS4);
	cv::resize(searcU8, searcU8, cv::Size(0, 0), 1. / prec, 1.0, cv::INTER_LANCZOS4);

	//  Sync by using cross correlation 
	cv::Mat tmResult;
	cv::matchTemplate(tmpltU8, searcU8, tmResult, cv::TemplateMatchModes::TM_CCORR_NORMED);
   
	// post-processing of tmResult --> lag, xcorr_lag, xcoor_coef
	double resultMin, resultMax;
	cv::Point resultMinPoint, resultMaxPoint;
	float dt = 0.f; 
	cv::minMaxLoc(tmResult, &resultMin, &resultMax, &resultMinPoint, &resultMaxPoint);
	if (resultMaxPoint.x > 0 && resultMaxPoint.x < tmResult.cols - 1) {
		float yL = tmResult.at<float>(0, resultMaxPoint.x - 1);
		float yC = tmResult.at<float>(0, resultMaxPoint.x  );
		float yR = tmResult.at<float>(0, resultMaxPoint.x + 1);
		float dYL = yC - yL;
		float dYR = yC - yR; 
		dt = 0.5f * (dYL - dYR) / (dYL + dYR);
		if (dYL + dYR < 1e-9) dt = 0.f;
	}
	lag = -(float) ((resultMaxPoint.x + dt) * prec + (searcX - tmpltX));
	xcorr_lag = cv::Mat(1, tmResult.cols, CV_32F); 
	for (int i = 0; i < xcorr_lag.cols; i++) 
		xcorr_lag.at<float>(0, i) = - (float)((i + dt) * prec + (searcX - tmpltX));
	tmResult.convertTo(xcorr_coef, CV_32F); 

	// apply synchronization
	t2.copyTo(t2sync); 
	applySynchronization(t2, t2sync, lag); 
	
	// print plot statement to matlab file so that user can plot figures by matlab 
	ofSync << "tmplt = " << tmplt << ";" << std::endl;
	ofSync << "searc = " << searc << ";" << std::endl;
	ofSync << "t1 = " << t1 << ";" << std::endl;
	ofSync << "t2 = " << t2 << ";" << std::endl;
	ofSync << "t2sync = " << t2sync << ";" << std::endl;
	ofSync << "tmpltU8 = " << tmpltU8 << ";" << std::endl;
	ofSync << "searcU8 = " << searcU8 << ";" << std::endl;
	ofSync << "lag = " << lag << ";" << std::endl;
	ofSync << "prec = " << prec << ";" << std::endl;
	ofSync << "searcX = " << searcX << ";" << std::endl;
	ofSync << "tmpltX = " << tmpltX << ";" << std::endl;
	ofSync << "tmResult = " << tmResult << ";" << std::endl;
	ofSync << "figure; plot(tmpltX:(tmpltX + size(tmplt,2) - 1), tmplt); hold on; plot(searcX:(searcX + size(searc,2) - 1), searc); grid on; legend('xi1', 'xi2'); xlabel('Frame Step'); " << std::endl;
	ofSync << "t_tmpltU8 = ((1:size(tmpltU8,2)) - 1) * prec + tmpltX;"; 
	ofSync << "t_searcU8 = ((1:size(searcU8,2)) - 1) * prec + searcX;";
	ofSync << "figure; plot(t_tmpltU8, tmpltU8); hold on; plot(t_searcU8, searcU8); grid on; legend('xi1-U8', 'xi2-U8'); xlabel('Frame Step'); " << std::endl;
	ofSync << "figure; plot(t_tmpltU8, tmpltU8); hold on; plot(t_searcU8 + lag, searcU8); grid on; legend('xi1-U8', 'xi2-U8-Sync'); xlabel('Frame Step'); " << std::endl;
	ofSync << "figure; plot(tmpltX:(tmpltX + size(tmplt,2) - 1), tmplt); hold on; plot((searcX:(searcX + size(searc,2) - 1)) + lag, searc); grid on; legend('xi1', 'xi2'); xlabel('Frame Step'); " << std::endl;
	ofSync << "figure; plot(1:size(t1,2), t1); hold on; plot(1:size(t2sync,2), t2sync); grid on; legend('Series 1', 'Series 2 sync'); xlabel('Frame Step'); " << std::endl;
	ofSync << "figure; plot((-1) * ((0:size(tmResult,2)-1) * prec + (searcX - tmpltX)), tmResult); grid on; legend('XCorr');" << std::endl;
	ofSync.close();
	// cout << "Offset: " << lag << endl;
	// cout << "Correlation: " << resultMax << endl;
	return (float) resultMax; 
}

/*!
  \brief synchroize two vectors (2f) time series (find the time lag of series 2)
  \param t1 time series 1. Must be 1 x N, float (CV_32FCx). (can be cv::Mat(1, N, CV_32FC2, (void*) data)
  \param t2 time series 2. Must be 1 x N, float (CV_32FCx). (can be cv::Mat(1, N, CV_32FC2, (void*) data)
  \param lag estimated time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \param t2sync synchronized t2 (1 x N, float (CV_32F))
  \param xcorr_lag  x data (lag) of cross correlation function (1 x N, float (CV_32F))
  \param xcorr_coef y data (coefficient) of cross correlation function (1 x N, float (CV_32F))
  \param guessT2Lag user initial guess of time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \param searchRange range (e.g., if 5, will search from (guessT2Lag - searchRange) to (guessT2Lag + searchRange).) (if -1, default is 5% of t1 length)
  \param winCenter matching window center (if -1, default is center of t1 series)
  \param winSize maching window size (if -1, default is half of t1 length)
  \param prec precision of matching (if 0.01, data length will be resized to 100 times longer before matching)
  \param oppoDir if true, two series are supposed to run in opposite way, when t1 goes up, t2 should go down.
  \param outPathMatlabScript path for output file of matlab script for visualization
  \return correlation coefficient
*/
float syncTwoVector2dSeries(
	const cv::Mat & t1,
	const cv::Mat & t2,
	float & lag,
	cv::Mat & t2sync,
	cv::Mat & xcorr_lag,
	cv::Mat & xcorr_coef,
	int   guessT2Lag,
	int   searchRange,
	int   winCenter,
	int   winSize,
	float prec,
	std::string outPathMatlabScript
)
{
	int n = t1.cols; 
	cv::Mat t1norm = cv::Mat::zeros(1, n, CV_32F);
	cv::Mat t2norm = cv::Mat::zeros(1, n, CV_32F);
	if (t1.type() == CV_32FC2) {
		for (int i = 0; i < n; i++) {
			t1norm.at<float>(0, i) = (float) cv::norm(t1.at<cv::Point2f>(0, i));
			t2norm.at<float>(0, i) = (float) cv::norm(t2.at<cv::Point2f>(0, i));
		}
	}
	if (t1.type() == CV_32FC3) {
		for (int i = 0; i < n; i++) {
			t1norm.at<float>(0, i) = (float) cv::norm(t1.at<cv::Point3f>(0, i));
			t2norm.at<float>(0, i) = (float) cv::norm(t2.at<cv::Point3f>(0, i));
		}
	}
	if (t1.type() == CV_64FC2) {
		for (int i = 0; i < n; i++) {
			t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2d>(0, i));
			t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2d>(0, i));
		}
	}
	if (t1.type() == CV_64FC3) {
		for (int i = 0; i < n; i++) {
			t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3d>(0, i));
			t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3d>(0, i));
		}
	}
	return syncTwoSeries(t1norm, t2norm, lag, t2sync, xcorr_lag, xcorr_coef, 
		guessT2Lag, searchRange, winCenter, winSize, prec, false, outPathMatlabScript);
}

/*!
  \brief synchroize two velocity (2f) time series by their velocity (find the time lag of series 2)
  \param t1 time series 1. Must be 1 x N, float (CV_32FCx). (can be cv::Mat(1, N, CV_32FC2, (void*) data)
  \param t2 time series 2. Must be 1 x N, float (CV_32FCx). (can be cv::Mat(1, N, CV_32FC2, (void*) data)
  \param lag estimated time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \param t2sync synchronized t2 (1 x N, float (CV_32F))
  \param xcorr_lag  x data (lag) of cross correlation function (1 x N, float (CV_32F))
  \param xcorr_coef y data (coefficient) of cross correlation function (1 x N, float (CV_32F))
  \param guessT2Lag user initial guess of time lag of time series 2 (lag > 0 means sensor 2 starts later)
  \param searchRange range (e.g., if 5, will search from (guessT2Lag - searchRange) to (guessT2Lag + searchRange).) (if -1, default is 5% of t1 length)
  \param winCenter matching window center (if -1, default is center of t1 series)
  \param winSize maching window size (if -1, default is half of t1 length)
  \param prec precision of matching (if 0.01, data length will be resized to 100 times longer before matching)
  \param oppoDir if true, two series are supposed to run in opposite way, when t1 goes up, t2 should go down.
  \param outPathMatlabScript path for output file of matlab script for visualization
  \return correlation coefficient
*/
float syncTwoVector2dSeriesByVelocity(
	const cv::Mat & t1,
	const cv::Mat & t2,
	float & lag,
	cv::Mat & t2sync,
	cv::Mat & xcorr_lag,
	cv::Mat & xcorr_coef,
	int   guessT2Lag,
	int   searchRange,
	int   winCenter,
	int   winSize,
	float prec,
	std::string outPathMatlabScript
)
{
	int n = t1.cols;
	cv::Mat t1norm = cv::Mat::zeros(1, n, CV_32F);
	cv::Mat t2norm = cv::Mat::zeros(1, n, CV_32F);
	if (t1.type() == CV_32FC2) {
		int i = 0; 
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2f>(0, i + 1) - t1.at<cv::Point2f>(0, i)); 
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2f>(0, i + 1) - t2.at<cv::Point2f>(0, i));
		for (i = 1; i < n - 1; i++) {
			t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2f>(0, i + 1) - t1.at<cv::Point2f>(0, i - 1)) / 2.f;
			t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2f>(0, i + 1) - t2.at<cv::Point2f>(0, i - 1)) / 2.f;
		}
		i = n - 1; 
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2f>(0, i) - t1.at<cv::Point2f>(0, i -1));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2f>(0, i) - t2.at<cv::Point2f>(0, i -1));
	}
	if (t1.type() == CV_32FC3) {
		int i = 0;
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3f>(0, i + 1) - t1.at<cv::Point3f>(0, i));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3f>(0, i + 1) - t2.at<cv::Point3f>(0, i));
		for (i = 1; i < n - 1; i++) {
			t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3f>(0, i + 1) - t1.at<cv::Point3f>(0, i - 1)) / 2.f;
			t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3f>(0, i + 1) - t2.at<cv::Point3f>(0, i - 1)) / 2.f;
		}
		i = n - 1;
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3f>(0, i) - t1.at<cv::Point3f>(0, i - 1));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3f>(0, i) - t2.at<cv::Point3f>(0, i - 1));
	}
	if (t1.type() == CV_64FC2) {
		int i = 0;
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2d>(0, i + 1) - t1.at<cv::Point2d>(0, i));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2d>(0, i + 1) - t2.at<cv::Point2d>(0, i));
		for (i = 1; i < n - 1; i++) {
			t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2d>(0, i + 1) - t1.at<cv::Point2d>(0, i - 1)) / 2.f;
			t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2d>(0, i + 1) - t2.at<cv::Point2d>(0, i - 1)) / 2.f;
		}
		i = n - 1;
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point2d>(0, i) - t1.at<cv::Point2d>(0, i - 1));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point2d>(0, i) - t2.at<cv::Point2d>(0, i - 1));
	}
	if (t1.type() == CV_64FC3) {
		int i = 0;
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3d>(0, i + 1) - t1.at<cv::Point3d>(0, i));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3d>(0, i + 1) - t2.at<cv::Point3d>(0, i));
		for (i = 1; i < n - 1; i++) {
			t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3d>(0, i + 1) - t1.at<cv::Point3d>(0, i - 1)) / 2.f;
			t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3d>(0, i + 1) - t2.at<cv::Point3d>(0, i - 1)) / 2.f;
		}
		i = n - 1;
		t1norm.at<float>(0, i) = (float)cv::norm(t1.at<cv::Point3d>(0, i) - t1.at<cv::Point3d>(0, i - 1));
		t2norm.at<float>(0, i) = (float)cv::norm(t2.at<cv::Point3d>(0, i) - t2.at<cv::Point3d>(0, i - 1));
	}
	return syncTwoSeries(t1norm, t2norm, lag, t2sync, xcorr_lag, xcorr_coef, 
		guessT2Lag, searchRange, winCenter, winSize, prec, false, outPathMatlabScript);
}

