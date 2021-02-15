#pragma once

#include <vector>
#include <string>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>


const int RollingPlot_Scale_Linear = 1;
const int RollingPlot_Scale_Log = 2;
const int RollingPlot_Scale_SymLog = 3;

const int RollingPlot_Color_Dark = 1;
const int RollingPlot_Color_Light = 2;
const int RollingPlot_Color_DarkCyberpunk = 3;
const int RollingPlot_Color_LightCyberpunk = 4;


class RollingPlot {
public:
	RollingPlot(int height = 400, int width = 400,
		float yMax = 1.f, float yMin = -1.f,
		std::string winName = "Rolling Plot Default",
		int xTickInterval = 100,
		std::vector<float> yTicks = std::vector<float>{},
		std::vector<float> yMinorTicks = std::vector<float>{},
		std::vector<std::string> yTicksText = std::vector<std::string>{},
		int scaleType = RollingPlot_Scale_Linear,
		float ySymlogPositiveZero = 1e-6,
		float ySymlogNegativeZero = -1e-6,
		int colorStyle = 3);
	float scaleY(float y);
	int scaleYToPixel(float y);
	int addDataAndPlot(float y);
	// colors
	cv::Scalar backgroundColor;
	cv::Scalar dataLineColor;
	cv::Scalar tickLineColor;
	cv::Scalar tickTextColor;
	cv::Scalar minorTickColor;
	cv::Scalar progressLineColor;

private:
	cv::Mat imgPlot;
	int w, h;
	int yScaleType; // 1: linear scale, 2: log scale, 3: symlog
	float yMin, yMax;
	int xTickInterval; 
	float yCurrent;
	float yScaled, yScaledMax, yScaledMin;
	float ySymlogPositiveZero, ySymlogNegativeZero;
	vector<float> yTicks;
	vector<float> yMinorTicks;
	vector<std::string> yTicksText;
	int xCurrent, yPrevious;
	std::string winName;
};
