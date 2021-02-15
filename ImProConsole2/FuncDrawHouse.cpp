#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>
#include "impro_util.h"

using namespace std;
using namespace cv;

void drawhouse(cv::Mat & img, 
	int w, int h, 
	cv::Mat cmat, 
	cv::Mat dvec, 
	cv::Mat rvec, 
	cv::Mat tvec,
	vector<Vec2i> lines, 
	vector<Point3f> p3, 
	vector<Point2f> p2)
{
	// Project points
	cv::projectPoints(p3, rvec, tvec, cmat, dvec, p2);

	// draw house
	img = cv::Mat::zeros(h, w, CV_8U) + 255;
	for (int i = 0; i < lines.size(); i++)
	{
		int pt1 = lines[i][0];
		int pt2 = lines[i][1];
		int thickness = 2;
		int lineType = 8;
		cv::line(img, p2[pt1], p2[pt2], cv::Scalar(0), thickness, lineType, 0);
	}

	// draw axis
	{
		int radius = 1;
		int thickness = 1;
		vector<Point2f> p2dot;
		vector<Point3f> p3dot;
		for (float z = 0.f; z <= 5.f; z += .02f)
			p3dot.push_back(Point3f(0, 0, z));
		cv::projectPoints(p3dot, rvec, tvec, cmat, dvec, p2dot);
		for (int i = 0; i < p2dot.size(); i++)
			cv::circle(img, p2dot[i], radius, Scalar(0), thickness);
	}
	{
		int radius = 1;
		int thickness = 1;
		vector<Point2f> p2dot;
		vector<Point3f> p3dot;
		for (float x = 0.f; x <= 7.f; x += .02f)
			p3dot.push_back(Point3f(x, 0, 0));
		cv::projectPoints(p3dot, rvec, tvec, cmat, dvec, p2dot);
		for (int i = 0; i < p2dot.size(); i++)
			cv::circle(img, p2dot[i], radius, Scalar(0), thickness);
	}


	// draw dots 
	for (int x = 0; x <= 5; x++)
	{
		int radius = 1; 
		int thickness = 1; 
		vector<Point2f> p2dot;
		vector<Point3f> p3dot;
		for (float z = -7.f; z <= 7.f; z += .2f)
			p3dot.push_back(Point3f((float)x, 0.f, z));
		cv::projectPoints(p3dot, rvec, tvec, cmat, dvec, p2dot); 
		for (int i = 0; i < p2dot.size() - 1; i++)
			cv::line(img, p2dot[i], p2dot[i + 1], Scalar(128), thickness);
//			cv::circle(img, p2dot[i], radius, Scalar(128), thickness); 
	}
	for (int z = 0; z <= 5; z++)
	{
		int radius = 1;
		int thickness = 1;
		vector<Point2f> p2dot;
		vector<Point3f> p3dot;
		for (float x = -20.f; x <= 9.f; x += .2f)
			p3dot.push_back(Point3f(x, 0.f, (float)z));
		cv::projectPoints(p3dot, rvec, tvec, cmat, dvec, p2dot);
		for (int i = 0; i < p2dot.size() - 1; i++)
			cv::line(img, p2dot[i], p2dot[i + 1], Scalar(128), thickness);
		//			cv::circle(img, p2dot[i], radius, Scalar(128), thickness);
	}

	// text
	char buf[1000];
    snprintf(buf, 1000, "fx = %6.1f", cmat.at<float>(0, 0));
	cv::putText(img, string(buf), Point(10, h - 230), 1, 2, Scalar(0));
    snprintf(buf, 1000, "fy = %6.1f", cmat.at<float>(1, 1));
	cv::putText(img, string(buf), Point(10, h - 200), 1, 2, Scalar(0));
    snprintf(buf, 1000, "cx = %6.1f", cmat.at<float>(0, 2));
	cv::putText(img, string(buf), Point(10, h - 170), 1, 2, Scalar(0));
    snprintf(buf, 1000, "cy = %6.1f", cmat.at<float>(1, 2));
	cv::putText(img, string(buf), Point(10, h - 140), 1, 2, Scalar(0));
    snprintf(buf, 1000, "k1 = %6.2f", dvec.at<float>(0, 0));
	cv::putText(img, string(buf), Point(10, h - 110), 1, 2, Scalar(0));
    snprintf(buf, 1000, "k2 = %6.2f", dvec.at<float>(0, 1));
	cv::putText(img, string(buf), Point(10, h - 80), 1, 2, Scalar(0));
    snprintf(buf, 1000, "p1 = %6.2f", dvec.at<float>(0, 2));
	cv::putText(img, string(buf), Point(10, h - 50), 1, 2, Scalar(0));
    snprintf(buf, 1000, "p2 = %6.2f", dvec.at<float>(0, 3));
	cv::putText(img, string(buf), Point(10, h - 20), 1, 2, Scalar(0));
}

int FuncDrawHouse(int argc, char ** argv)
{
	// basic variables
	int w; // image width
	int h; // image height
	cv::Mat cmat = cv::Mat::eye(3, 3, CV_32F);    // camera matrix
	cv::Mat dvec = cv::Mat::zeros(1, 4, CV_32F);  // distortion vector
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);  // rotation vector
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);  // translation vector
	cv::Mat r33 = cv::Mat::eye(3, 3, CV_32F); 
	vector<Point3f> p3;
	vector<Point2f> p2; 
	vector<Vec2i> lines; 
	cv::Mat img; 

	//// initialization
	w = 1280; 
	h = 720; 
	cmat.at<float>(0, 0) = 750.f;
	cmat.at<float>(1, 1) = 750.f;
	cmat.at<float>(0, 2) = (w - 1) / 2.f;
	cmat.at<float>(1, 2) = (h - 1) / 2.f;
	tvec.at<float>(0, 0) = -3.f;
	tvec.at<float>(1, 0) = 1.5f;
	tvec.at<float>(2, 0) = 10.f;
	r33.at<float>(0, 0) = 1.f; 
	r33.at<float>(1, 0) = 1.f;
	r33.at<float>(2, 0) = -1.f;
	r33.at<float>(0, 1) = 1.f;
	r33.at<float>(1, 1) = -1.f;
	r33.at<float>(2, 1) = 1.f;
	r33.at<float>(0, 2) = 0.f;
	r33.at<float>(1, 2) = -1.f;
	r33.at<float>(2, 2) = 0.f;

	// points and lines initialization 
	p3.push_back(cv::Point3f(0, 0, 0)); 
	p3.push_back(cv::Point3f(5, 0, 0));
	p3.push_back(cv::Point3f(5, 0, 3));
	p3.push_back(cv::Point3f(0, 0, 3));
	p3.push_back(cv::Point3f(2.5f, 0, 5));
	p3.push_back(cv::Point3f(0, 3, 0));
	p3.push_back(cv::Point3f(5, 3, 0));
	p3.push_back(cv::Point3f(5, 3, 3));
	p3.push_back(cv::Point3f(0, 3, 3));
	p3.push_back(cv::Point3f(2.5f, 3, 5));
	p3.push_back(cv::Point3f(2, 0, 1)); 
	p3.push_back(cv::Point3f(3, 0, 1));
	p3.push_back(cv::Point3f(2, 0, 2));
	p3.push_back(cv::Point3f(3, 0, 2));
	p3.push_back(cv::Point3f(5, 1, 0));
	p3.push_back(cv::Point3f(5, 2, 0));
	p3.push_back(cv::Point3f(5, 1, 2));
	p3.push_back(cv::Point3f(5, 2, 2));

	lines.push_back(Vec2i(0, 1));
	lines.push_back(Vec2i(1, 2));
	lines.push_back(Vec2i(2, 3));
	lines.push_back(Vec2i(3, 0));
	lines.push_back(Vec2i(2, 4));
	lines.push_back(Vec2i(3, 4));
//	lines.push_back(Vec2i(0+5, 1+5));
	lines.push_back(Vec2i(1+5, 2+5));
//	lines.push_back(Vec2i(2+5, 3+5));
//	lines.push_back(Vec2i(3+5, 0+5));
	lines.push_back(Vec2i(2+5, 4+5));
//	lines.push_back(Vec2i(3+5, 4+5));
//	lines.push_back(Vec2i(0, 5));
	lines.push_back(Vec2i(1, 6));
	lines.push_back(Vec2i(2, 7));
//	lines.push_back(Vec2i(3, 8));
	lines.push_back(Vec2i(4, 9));
	lines.push_back(Vec2i(10, 11));
	lines.push_back(Vec2i(12, 13));
	lines.push_back(Vec2i(11, 13));
	lines.push_back(Vec2i(10, 12));
	lines.push_back(Vec2i(10+4, 11+4));
	lines.push_back(Vec2i(12+4, 13+4));
	lines.push_back(Vec2i(11+4, 13+4));
	lines.push_back(Vec2i(10+4, 12+4));
  
	//// normalize r33
	float L1 = sqrt(pow(r33.at<float>(0, 0), 2) + pow(r33.at<float>(1, 0), 2) + pow(r33.at<float>(2, 0), 2));
	float L2 = sqrt(pow(r33.at<float>(0, 1), 2) + pow(r33.at<float>(1, 1), 2) + pow(r33.at<float>(2, 1), 2));
	float L3 = sqrt(pow(r33.at<float>(0, 2), 2) + pow(r33.at<float>(1, 2), 2) + pow(r33.at<float>(2, 2), 2));
	r33.at<float>(0, 0) = r33.at<float>(0, 0) / L1;
	r33.at<float>(1, 0) = r33.at<float>(1, 0) / L1;
	r33.at<float>(2, 0) = r33.at<float>(2, 0) / L1;
	r33.at<float>(0, 1) = r33.at<float>(0, 1) / L2;
	r33.at<float>(1, 1) = r33.at<float>(1, 1) / L2;
	r33.at<float>(2, 1) = r33.at<float>(2, 1) / L2;
	r33.at<float>(0, 2) = r33.at<float>(0, 2) / L3;
	r33.at<float>(1, 2) = r33.at<float>(1, 2) / L3;
	r33.at<float>(2, 2) = r33.at<float>(2, 2) / L3;
	cv::Rodrigues(r33, rvec); 

//	cout << "R33: \n" << r33 << endl;
//	cout << "P3: " << p3.size() << endl;

	// while
	cv::Mat cmat0 = cmat.clone(); 
	cv::Mat dvec0 = dvec.clone();
	cv::Mat rvec0 = rvec.clone();
	cv::Mat tvec0 = tvec.clone();
	while (true)
	{
		drawhouse(img, w, h, cmat, dvec, rvec, tvec, lines, p3, p2); 
		cv::imshow("House", img); 
		int ikey = cv::waitKey(100);
		if (ikey == 27) break;
		// adjusting fx
		if (ikey == 'q') cmat.at<float>(0, 0) /= .9f; 
		if (ikey == 'a') cmat.at<float>(0, 0) *= .9f;
		if (ikey == 'z') cmat.at<float>(0, 0) = cmat0.at<float>(0, 0);
		//// adjusting fy
		if (ikey == 'w') cmat.at<float>(1, 1) /= .9f;
		if (ikey == 's') cmat.at<float>(1, 1) *= .9f;
		if (ikey == 'x') cmat.at<float>(1, 1) = cmat0.at<float>(1, 1);
		// adjusting cx
		if (ikey == 'e') cmat.at<float>(0, 2) /= .9f;
		if (ikey == 'd') cmat.at<float>(0, 2) *= .9f;
		if (ikey == 'c') cmat.at<float>(0, 2) = cmat0.at<float>(0, 2);
		// adjusting cy
		if (ikey == 'r') cmat.at<float>(1, 2) /= .9f;
		if (ikey == 'f') cmat.at<float>(1, 2) *= .9f;
		if (ikey == 'v') cmat.at<float>(1, 2) = cmat0.at<float>(1, 2);
		// adjusting k1
		if (ikey == 't') dvec.at<float>(0, 0) += .01f;
		if (ikey == 'g') dvec.at<float>(0, 0) -= .01f;
		if (ikey == 'b') dvec.at<float>(0, 0) = dvec0.at<float>(0, 0);
		// adjusting k2
		if (ikey == 'y') dvec.at<float>(0, 1) += .01f;
		if (ikey == 'h') dvec.at<float>(0, 1) -= .01f;
		if (ikey == 'n') dvec.at<float>(0, 1) = dvec0.at<float>(0, 1);
		// adjusting p1
		if (ikey == 'u') dvec.at<float>(0, 2) += .02f;
		if (ikey == 'j') dvec.at<float>(0, 2) -= .02f;
		if (ikey == 'm') dvec.at<float>(0, 2) = dvec0.at<float>(0, 2);
		// adjusting p2
		if (ikey == 'i') dvec.at<float>(0, 3) += .01f;
		if (ikey == 'k') dvec.at<float>(0, 3) -= .01f;
		if (ikey == 'm') dvec.at<float>(0, 3) = dvec0.at<float>(0, 3);
		// adjusting p2
		if (ikey == 'i') dvec.at<float>(0, 3) += .01f;
		if (ikey == 'k') dvec.at<float>(0, 3) -= .01f;
		if (ikey == 'm') dvec.at<float>(0, 3) = dvec.at<float>(0, 3);


	}

	cv::destroyAllWindows();

	return 0; 
}
