#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "IntrinsicCalibrator.h"
#include "impro_util.h"
#include "improDraw.h"

using namespace std;

int FuncDrawGrid(int argc, char ** argv)
{
	string fnameImg, fnameImgOut;
	string fnameCalib;
	cv::Mat img;
	cv::Mat cmat, dvec, r4, rvec, tvec;

	// background image
	std::cout << "#Input full path of background image:\n";
	fnameImg = readStringLineFromIstream(std::cin);
	img = cv::imread(fnameImg);
	if (img.cols <= 0 || img.rows <= 0)
	{
		std::cerr << "#Cannot read the image.\n";
		return -1;
	}
	double imshowFactor = 600. / img.rows;
	imshow_resize("Original background", img, imshowFactor);
	cv::waitKey(500);
	cv::destroyAllWindows();

	// output file
	std::cout << "#Full path of output image:\n";
	fnameImgOut = readStringLineFromIstream(std::cin);

	// calibration file
	std::cout << "#Input full path of camera calibration file (xml):\n";
	fnameCalib = readStringLineFromIstream(std::cin);
	cv::FileStorage fs(fnameCalib, cv::FileStorage::READ);
	fs["cameraMatrix"] >> cmat;
	fs["distortionVector"] >> dvec;
	fs["R4"] >> r4;
	try
	{
		// try to read tvec ad rvec as cv::Mat
		fs["tvec"] >> tvec;
		fs["rvec"] >> rvec;
	}
	catch (...)
	{
		// if failed, read as vector<double> 
		tvec = cv::Mat(3, 1, CV_64F);
		rvec = cv::Mat(3, 1, CV_64F);
		vector<double> _tvec(3);
		vector<double> _rvec(3);
		fs["tvec"] >> _tvec;
		fs["rvec"] >> _rvec;
		for (int k = 0; k < 3; k++) tvec.at<double>(k, 0) = _tvec[k];
		for (int k = 0; k < 3; k++) rvec.at<double>(k, 0) = _rvec[k];
	}
	cv::Mat r3(3, 3, CV_64F); 
	cv::Rodrigues(rvec, r3); 
	r4 = cv::Mat::eye(4, 4, CV_64F); 
	r3.copyTo(r4(cv::Rect(0, 0, 3, 3))); 
	r4.at<double>(0, 3) = tvec.at<double>(0); 
	r4.at<double>(1, 3) = tvec.at<double>(1);
	r4.at<double>(2, 3) = tvec.at<double>(2);
	if (cmat.cols != 3 || cmat.rows != 3) {
		std::cerr << fnameCalib << " does not contain cameraMatrix. Try again.\n";
		return -1;
	}
	if (min(dvec.cols, dvec.rows) != 1 || max(dvec.cols, dvec.rows) <= 1) {
		std::cerr << fnameCalib << " dvec size should be 1x4, 1x5, or 1x8 but " << dvec.rows << "x" << dvec.cols << endl;
		std::cerr << fnameCalib << " does not contain distortionVector. Try again.\n";
		return -1;
	}
	if (r4.cols != 4 || r4.rows != 4) {
		std::cerr << fnameCalib << " does not contain R4. Try again.\n";
		return -1;
	}
	if (tvec.cols * tvec.rows != 3) {
		tvec = r4(cv::Rect(3, 0, 1, 3)).clone();
	}
	if (rvec.cols * rvec.rows != 3) {
		rvec = cv::Mat(3, 1, r4.type());
		cv::Rodrigues(rvec, r4(cv::Rect(0, 0, 3, 3)));
	}
	if (tvec.cols == 3 && tvec.rows == 1) tvec = tvec.t();
	if (rvec.cols == 3 && rvec.rows == 1) rvec = rvec.t();
	std::cout << "Camera matrix is:\n" << cmat << endl;
	std::cout << "Distortion vec is:\n" << dvec << endl;

	// read grid data
	while (true) {
		vector<float> gridData(10);
		float dotSize, x_start, x_end, dx, y_start, y_end, dy, z_start, z_end, dz;
		// read grid data
		std::cout << "# Input dotSize  x_start  x_end dx   y_start y_end dy  z_start z_end dz (or 0 to quit):\n";
		dotSize = (float)readDoubleFromIstream(std::cin); 
		if (dotSize <= 0.0f) break;
		x_start = (float)readDoubleFromIstream(std::cin);
		x_end = (float)readDoubleFromIstream(std::cin);
		dx = (float)readDoubleFromIstream(std::cin);
		y_start = (float)readDoubleFromIstream(std::cin);
		y_end = (float)readDoubleFromIstream(std::cin);
		dy = (float)readDoubleFromIstream(std::cin);
		z_start = (float)readDoubleFromIstream(std::cin);
		z_end = (float)readDoubleFromIstream(std::cin);
		dz = (float)readDoubleFromIstream(std::cin);
		// generate objPoints (a cv::Mat(nPoints, 1, CV_32FC3))
		int nx = 1, ny = 1, nz = 1; 
		if (dx != 0.0) nx = min(max((int)((x_end - x_start) / dx + 1.0), 1), 999);
		if (dy != 0.0) ny = min(max((int)((y_end - y_start) / dy + 1.0), 1), 999);
		if (dz != 0.0) nz = min(max((int)((z_end - z_start) / dz + 1.0), 1), 999);
		int nPoints = nx * ny * nz, iPoint = 0;
		cv::Mat objPoints = cv::Mat::zeros(nPoints, 1, CV_32FC3);
		for (int ix = 0; ix < nx; ix++)
		{
			for (int iy = 0; iy < ny; iy++)
			{
				for (int iz = 0; iz < nz; iz++)
				{
					objPoints.at<cv::Point3f>(iPoint, 0).x = x_start + dx * ix; 
					objPoints.at<cv::Point3f>(iPoint, 0).y = y_start + dy * iy;
					objPoints.at<cv::Point3f>(iPoint, 0).z = z_start + dz * iz;
					iPoint++; 
				}
			}
		}
		// project points to imgPoints (a cv::Mat(nPoints, 1, CV_32FC2))
		cv::Mat imgPoints = cv::Mat(nPoints, 1, CV_32FC2); 
		// project points (from objPoints to imgPoints)
		cv::projectPoints(objPoints, rvec, tvec, cmat, dvec, imgPoints); 
		// plot on image
		int thickness, radius, shift = 2;
		float alpha = 0.75f; 
		radius = (int)(dotSize + .5f); 
		thickness = max((int)(dotSize / 2), 1);
		drawPointsOnImage(img, imgPoints, std::string("o"),
			radius, thickness, 
			cv::Scalar(0, 0, 0),
			alpha, -1 /* put text */, shift);
		radius = (int)(dotSize + .5f - 2);
		thickness = max((int)(dotSize / 4), 1);
		drawPointsOnImage(img, imgPoints, std::string("o"),
			radius, thickness,
			cv::Scalar(255, 255, 255),
			alpha, -1 /* put text */, shift);
	} // end of while (each iteration --> each grid)

	// write plotted image to file
	cv::imwrite(fnameImgOut, img); 
	imshow_resize("Plotted image", img, imshowFactor);
	cv::waitKey(5000);
	cv::destroyAllWindows();
	return 0;
}


