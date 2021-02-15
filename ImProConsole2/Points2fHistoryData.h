#pragma once
#include <opencv2/opencv.hpp>

#include "IoData.h"

class Points2fHistoryData :
	public IoData
{
public:
	Points2fHistoryData();
	Points2fHistoryData(const cv::Mat & dataToClone);
	Points2fHistoryData(const vector<cv::Mat> & dataToClone);
	Points2fHistoryData(const vector<vector<cv::Point2f> > & dataToClone);
	virtual ~Points2fHistoryData();

	// Inherited via IoData
	virtual int readFromTxt(string fileTxt) ;
	virtual int writeToTxt(string fileTxt) ;
	virtual int readFromXml(string fileXml) ;
	virtual int writeToXml(string fileXml) ;
	virtual int readThruUserInteraction(); 
	virtual int readThruUserInteraction(int nStep, int nPoint = -1) ;
	virtual int writeThruUserInteraction() ;
	virtual int writeScriptMat(string fileM) ;

	// write script
	virtual int writeScriptMatAdvanced(string fileM, string backgroundImgFile,
		bool drawPointNumber = true,
		int pointNumberBase = 1,
		int onlyData = 0,
		int iStep = -1  // -1 for all step, >= 0 for only step iStep
	);


	// get/set size
	int nStep();
	int nPoint(); 
	int resize(int nStep, int nPoint); 

	// access (read)
	cv::Point2f get(int iStep, int iPoint);
	cv::Mat & getMat();
	vector<cv::Mat> getVecMat(); 
	vector<vector<cv::Point2f> > getVecVec();
	cv::Rect getRect(int iPoint) const; 

	// access entire object
	// set theMat to this object
	int set(int iStep, int iPoint, cv::Point2f p);
	int set(int iStep, int iPoint, cv::Point2d p); // will convert to cv::Point2f
	int set(const cv::Mat & theDat);
	int set(const vector<cv::Mat> & theDat);
	int set(const vector<vector<cv::Point2f> > & dataToClone);
	int set(const vector<vector<cv::Point2d> > & dataToClone);
	int setRect(int iPoint, cv::Rect rect); 

	// append points
	int appendLine(cv::Point2f p0, cv::Point2f p1, int nPointAdd);
	int appendQ4(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, int nPoint01, int nPoint12);
	   
protected:
	cv::Mat dat;  // point iPoint at time step iStep: dat.at<cv::Point2f>(iStep, iPoint)
	vector<cv::Rect> rects; // if size is not zero, rects[iPoint] is the initial rect (template) range of point iPoint
	cv::Size imgSize; 
};

void testPoints2fHistoryData();