#pragma once
#include "RollingPlot.h"

RollingPlot::RollingPlot(int height, int width,
	float yMax, float yMin, 
	std::string winName,
	int xTickInterval,
	std::vector<float> yTicks,
	std::vector<float> yMinorTicks,
	std::vector<std::string> yTicksText,
	int scaleType, float ySymlogPositiveZero, float ySymlogNegativeZero, 
	int colorStyle)
{
	this->w = width;
	this->h = height;
	if (yMin > yMax) { 
		float tmp = yMin; 
		yMin = yMax; 
		yMax = tmp; 
	}
	this->yMin = yMin;
	this->yMax = yMax;
	this->winName = winName; 
	this->xTickInterval = xTickInterval; 
	this->yTicks = yTicks;
	this->yMinorTicks = yMinorTicks;
	this->yTicksText = yTicksText;
	this->xCurrent = 0;
	this->yScaleType = scaleType;
	if (this->yTicks.size() != this->yTicksText.size() && this->yTicksText.size() > 0)
		std::cout << "# Warning: RollingPlot: " << this->winName 
				  << ": Number of ticks and number of ticks text are not the same.\n";
	// only for symlog
	if (ySymlogPositiveZero < ySymlogNegativeZero) { 
		float tmp = ySymlogPositiveZero; 
		ySymlogPositiveZero = ySymlogNegativeZero; 
		ySymlogNegativeZero = tmp; 
	}
	this->ySymlogPositiveZero = ySymlogPositiveZero;
	this->ySymlogNegativeZero = ySymlogNegativeZero;

	// colors
	if (colorStyle == RollingPlot_Color_Dark) {
		this->backgroundColor = cv::Scalar(0, 0, 0);
		this->dataLineColor = cv::Scalar(255, 255, 255);
		this->tickLineColor = cv::Scalar(128, 128, 128);
		this->tickTextColor = cv::Scalar(64, 255, 64);
		this->minorTickColor = cv::Scalar(64, 64, 64);
		this->progressLineColor = cv::Scalar(128, 255, 128);
	} 
	else if (colorStyle == RollingPlot_Color_Light) {
		this->backgroundColor = cv::Scalar(255, 255, 255);
		this->dataLineColor = cv::Scalar(0, 0, 0);
		this->tickLineColor = cv::Scalar(192, 192, 192);
		this->tickTextColor = cv::Scalar(0, 64, 0);
		this->minorTickColor = cv::Scalar(224, 224, 224);
		this->progressLineColor = cv::Scalar(64, 255, 64);
	} 
	else if (colorStyle == RollingPlot_Color_DarkCyberpunk) {
		this->backgroundColor = cv::Scalar(70, 41, 33);
		//		this->dataLineColor = cv::Scalar(180, 119, 31); // blue like
		this->dataLineColor = cv::Scalar(14, 127, 255); // orange like
		this->tickLineColor = cv::Scalar(89, 52, 42);
		this->tickTextColor = cv::Scalar(230, 230, 230);
		this->minorTickColor = (this->backgroundColor + this->tickLineColor) / 2;
		this->progressLineColor = (this->dataLineColor + this->tickLineColor) / 2;
	}
	else if (colorStyle == RollingPlot_Color_LightCyberpunk) {
		this->backgroundColor = cv::Scalar(255, 255, 255);
		this->dataLineColor = cv::Scalar(180, 119, 31); // blue like
//		this->dataLineColor = cv::Scalar(14, 127, 255); // orange like
		this->tickLineColor = cv::Scalar(192, 192, 192);
		this->tickTextColor = cv::Scalar(0, 0, 0);
		this->minorTickColor = (this->backgroundColor + this->tickLineColor) / 2;
		this->progressLineColor = (this->dataLineColor + this->tickLineColor) / 2;
	}


	//
	this->imgPlot = cv::Mat(this->h, this->w, CV_8UC3, this->backgroundColor);
}

float RollingPlot::scaleY(float y)
{
	if (this->yScaleType == RollingPlot_Scale_Linear)
		return y;
	else if (this->yScaleType == RollingPlot_Scale_Log)
		return log(y);
	else if (this->yScaleType == RollingPlot_Scale_SymLog)
		if (y > 0) {
			if (y <= this->ySymlogPositiveZero)
				return 0.0f;
			else
				return log(y) - log(this->ySymlogPositiveZero);
		}
		else if (y < 0) {
			if (y >= this->ySymlogNegativeZero)
				return 0.0f;
			else
				return -log(-y) + log(-(this->ySymlogNegativeZero));
		}
		else {
			return 0.0f;
		}
	return 0.0f; // will never be here, only to avoid warning message.
}

int RollingPlot::scaleYToPixel(float y)
{
	float yScaled = this->scaleY(y);
	this->yScaledMax = this->scaleY(this->yMax);
	this->yScaledMin = this->scaleY(this->yMin);
	int yScaledPixel = (int)((this->h - 1) *
		(1.f - (yScaled - this->yScaledMin) / (this->yScaledMax - this->yScaledMin)) + .5f);
	if (yScaledPixel >= h - 1) yScaledPixel = h - 1;
	if (yScaledPixel <= 0) yScaledPixel = 0;
	return yScaledPixel;
}



int RollingPlot::addDataAndPlot(float y)
{
	// Scale y to pixel (according to its scaling type: linear, log, or symlog)
	int yScaledPixel = this->scaleYToPixel(y);

	// clear old image (draw two vertical lines)
	cv::line(imgPlot, cv::Point(xCurrent, 0), cv::Point(xCurrent, h - 1), this->backgroundColor, 1, 8, 0); 
	if (xCurrent + 1 < this->w)
		cv::line(imgPlot, cv::Point(xCurrent + 1, 0), cv::Point(xCurrent + 1, h - 1), 
			this->progressLineColor, 1, 8, 0);

	// Plot tick text
	for (size_t i = 0; i < yTicks.size(); i++) {
		int yTickPixel = this->scaleYToPixel(this->yTicks[i]);
		cv::putText(imgPlot, yTicksText[i], cv::Point(0, yTickPixel), cv::FONT_HERSHEY_PLAIN,
			1.0, this->tickTextColor, 1, 8); 
	}

	// Check if it needs an X tick line
	if (xCurrent % this->xTickInterval == 0)
		cv::line(imgPlot, cv::Point(xCurrent, 0), cv::Point(xCurrent, h - 1), 
			this->tickLineColor, 1, 8, 0);

	// Plot lines of minor ticks
	for (size_t i = 0; i < yMinorTicks.size(); i++) {
		int yMinorTickPixel = this->scaleYToPixel(this->yMinorTicks[i]);
		imgPlot.at<cv::Vec3b>(yMinorTickPixel, xCurrent)[0] = (uchar)this->minorTickColor[0];
		imgPlot.at<cv::Vec3b>(yMinorTickPixel, xCurrent)[1] = (uchar)this->minorTickColor[1];
		imgPlot.at<cv::Vec3b>(yMinorTickPixel, xCurrent)[2] = (uchar)this->minorTickColor[2];
	}

	// Plot lines of ticks
	for (size_t i = 0; i < yTicks.size(); i++) {
		int yTickPixel = this->scaleYToPixel(this->yTicks[i]);
		imgPlot.at<cv::Vec3b>(yTickPixel, xCurrent)[0] = (uchar)this->tickLineColor[0];
		imgPlot.at<cv::Vec3b>(yTickPixel, xCurrent)[1] = (uchar)this->tickLineColor[1];
		imgPlot.at<cv::Vec3b>(yTickPixel, xCurrent)[2] = (uchar)this->tickLineColor[2];
	}

	// Plot data in image (cv::Mat imgPlot)
	if (this->xCurrent != 0) {
		cv::Point p0(this->xCurrent - 1, yPrevious);
		cv::Point p1(this->xCurrent, yScaledPixel);
		cv::line(this->imgPlot, p0, p1, this->dataLineColor, 1, 8, 0);
		//		cv::line(this->imgPlot, p0 * 2, p1 * 2, this->dataLineColor, 1, 8, 1);
		//		cv::line(this->imgPlot, p0 * 4, p1 * 4, this->dataLineColor, 1, 8, 2);
	}

	// plot
	cv::namedWindow(this->winName, cv::WINDOW_AUTOSIZE);
	cv::imshow(this->winName, imgPlot);
	int ikey = cv::waitKey(1);

	// Prepare next
	this->xCurrent++;
	if (this->xCurrent >= this->w)
		this->xCurrent = 0;
	yPrevious = yScaledPixel;

	return ikey;
}
