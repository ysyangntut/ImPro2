#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cmath>
#include <complex>
#include "impro_util.h"
using namespace std;

double mandelbrot(cv::Mat & img, double x0, double x1, double y0, double y1)
{
	int h = img.rows, w = img.cols; 
	int itr, maxItr = 1000; 
	double conv = 4.0;
	std::complex<double> z, c;
	img = cv::Mat::zeros(h, w, CV_8U);
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			double cr = x0 + (j * 1. / w) * (x1 - x0);
			double ci = y0 + (1. - (i * 1. / h)) * (y1 - y0);
			std::complex<double> c = cr + ci * 1i;
			std::complex<double> z = 0. + 0.i;
			itr = 1;
			while (itr < maxItr && std::real(z * std::conj(z)) < conv)
			{
				z = z * z + c;
				itr++;
			}
			img.at<uchar>(i, j) = (uchar)(255 - itr * 255. / maxItr);
		}
		if (i % 50 == 0) {
			cv::imshow("Calculating", img);
			cv::waitKey(1);
		}
	}
	cv::destroyWindow("Calculating"); 
	return 0; 
}




int FuncMandelbrot(int argc, char** argv)
{
	double x0, x1, y0, y1, conv = 4.0; 
	cv::Mat img; 

	x0 = -2; x1 = 2; y0 = -1; y1 = 1; 
//	x0 = 0.25; x1 = 0.28; y0 = -0.015; y1 = 0.015; 

	printf("# Enter Mandelbrot image width and height (in pixels):\n"); 
	int w = readIntFromCin(10, 2160);
	int h = readIntFromCin(10, 3840); 

	while (true) {
		img = cv::Mat(h, w, CV_8U);
		printf("X:%25.16e ~ %25.16e.  Y:%25.16e ~ %25.16e.\n", x0, x1, y0, y1); 
		mandelbrot(img, x0, x1, y0, y1);
		cv::Rect rect = cv::selectROI(img, true, false);
		if (rect.height <= 1 || rect.width <= 1) break;
		double new_x0 = x0 + rect.x * 1.0 / w * (x1 - x0); 
		double new_x1 = x0 + (rect.x + rect.width) * 1.0 / w * (x1 - x0);
		double new_y0 = y0 + (h - rect.y - rect.height) * 1.0 / h * (y1 - y0);
		double new_y1 = y0 + (h - rect.y) * 1.0 / h * (y1 - y0);
		// ratio fits roi (image size changed) or fits original ratio (image size unchanged)
		bool ratioRoi = true;
		if (ratioRoi) {
			x0 = new_x0;
			x1 = new_x1;
			y0 = new_y0;
			y1 = new_y1;
			w = (int)(h * (x1 - x0) / (y1 - y0));
		}
		else {
			x0 = 0.5 * (new_x0 + new_x1) - (new_y1 - new_y0) * w * 0.5 / h;
			x1 = 0.5 * (new_x0 + new_x1) + (new_y1 - new_y0) * w * 0.5 / h;
			y0 = new_y0;
			y1 = new_y1;
		}

	}

	cv::destroyAllWindows();

	return 0;
}