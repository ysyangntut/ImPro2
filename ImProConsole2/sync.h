#pragma once

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std; 

//	Sample:
// 
//
// float wave(float x) { return sin(10.*2.*3.1416*x);} 
//
//int n = 1000;
//cv::Mat a1(1, n, CV_32F), a2(1, n, CV_32F);
//cv::Mat b1(1, n, CV_32FC2), b2(1, n, CV_32FC2);
//for (int i = 0; i < n; i++) {
//	a1.at<float>(0, i) = (float)wave(i * 1.0f / n);
//	b1.at<cv::Point2f>(0, i).x = (float)wave(i * 1.0f / n);
//	b1.at<cv::Point2f>(0, i).y = 0.f;
//	float lag = 1.357f;
//	float j = i + lag;
//	a2.at<float>(0, i) = (float)wave(j * 1.0f / n);
//	b2.at<cv::Point2f>(0, i).x = (float)wave(j * 1.0f / n);
//	b2.at<cv::Point2f>(0, i).y = 0.f;
//}
//
//float lag;
//float guess = 0.8f;
//int   sRange = 10;
//int   winCenter = 500;
//int   winSize = 50;
//double precision = 0.01;
//bool  oppoDir = false;
//double corra = syncTwoSeries(a1, a2, lag, guess, sRange, winCenter, winSize, precision, oppoDir);
//cout << "Time lag is " << lag << endl;
//cout << "Correlation is " << corra << endl;
//
//double corrb = syncTwoVector2dSeries(b1, b2, lag, guess, sRange, winCenter, winSize, precision);
//cout << "Time lag is " << lag << endl;
//cout << "Correlation is " << corrb << endl;

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
	float lag);


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
	int   guessT2Lag = 0,
	int   searchRange = -1,
	int   winCenter = -1,
	int   winSize = -1,
	float prec = 0.01,
	bool oppoDir = false, 
	std::string outPathMatlabScript = ""
);

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
	int   guessT2Lag = 0,
	int   searchRange = -1,
	int   winCenter = -1,
	int   winSize = -1,
	float prec = 0.01, 
	string outPathMatlabScript = ""
);

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
	int   guessT2Lag = 0.0f,
	int   searchRange = -1,
	int   winCenter = -1,
	int   winSize = -1,
	float prec = 0.01, 
	string outPathMatlabScript = ""
); 

