#pragma once
#include <opencv2/opencv.hpp>

#include "IoData.h"

class Points3dHistoryData :
	public IoData
{
public:
	Points3dHistoryData();
	Points3dHistoryData(const cv::Mat & dataToClone);
	Points3dHistoryData(const vector<cv::Mat> & dataToClone);
	Points3dHistoryData(const vector<vector<cv::Point3d> > & dataToClone);
	virtual ~Points3dHistoryData();

	// Inherited via IoData
	virtual int readFromTxt(string fileTxt);
	virtual int writeToTxt(string fileTxt);
	virtual int readFromXml(string fileXml);
	virtual int writeToXml(string fileXml);
	virtual int readThruUserInteraction();
	virtual int readThruUserInteraction(int nStep, int nPoint = -1);
	virtual int writeThruUserInteraction();
	virtual int writeScriptMat(string fileM);

	// write script
	virtual int writeScriptMatAdvanced(string fileM, 
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
	cv::Point3d get(int iStep, int iPoint);
	cv::Mat & getMat();
	vector<cv::Mat> getVecMat();
	vector<vector<cv::Point3d> > getVecVec();

	// access entire object
	// set theMat to this object
	int set(int iStep, int iPoint, cv::Point3d p);
	int set(int iStep, int iPoint, cv::Point3f p); // will convert to cv::Point3d
	int set(const cv::Mat & theDat);
	int set(const vector<cv::Mat> & theDat);
	int set(const vector<vector<cv::Point3d> > & dataToClone);
	int set(const vector<vector<cv::Point3f> > & dataToClone); // will convert to cv::Point3d

	// append points
	int appendLine(cv::Point3d p0, cv::Point3d p1, int nPointAdd);
	int appendQ4(cv::Point3d p0, cv::Point3d p1, cv::Point3d p2, cv::Point3d p3, int nPoint01, int nPoint12);

protected:
	cv::Mat dat;
};

void testPoints3dHistoryData(); 
