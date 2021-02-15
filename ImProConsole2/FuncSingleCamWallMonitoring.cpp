#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "impro_util.h"

#include <opencv2/opencv.hpp>

double getMatElm(const cv::Mat& m, int i)
{
	if (m.type() == CV_8U) return m.at<unsigned char>(i);
	else if (m.type() == CV_8S) return m.at<signed char>(i);
	else if (m.type() == CV_16U) return m.at<unsigned short>(i);
	else if (m.type() == CV_16S) return m.at<signed short>(i);
	else if (m.type() == CV_32S) return m.at<signed int>(i);
	else if (m.type() == CV_32F) return m.at<float>(i);
	else if (m.type() == CV_64F) return m.at<double>(i);
	else if (m.type() == CV_8UC2 || m.type() == CV_8UC3 || m.type() == CV_8UC4)
	{
		unsigned char* p; p = m.data; 
		p = p + i; 
		return (double)(*p); 
	}
	else if (m.type() == CV_8SC2 || m.type() == CV_8SC3 || m.type() == CV_8SC4)
	{
		signed char* p; p = (signed char*) m.data;
		p = p + i;
		return (double)(*p);
	}
	else if (m.type() == CV_16UC2 || m.type() == CV_16UC3 || m.type() == CV_16UC4)
	{
		unsigned short* p; p = (unsigned short*) m.data;
		p = p + i;
		return (double)(*p);
	}
	else if (m.type() == CV_16SC2 || m.type() == CV_16SC3 || m.type() == CV_16SC4)
	{
		signed short* p; p = (signed short*) m.data;
		p = p + i;
		return (double)(*p);
	}
	else if (m.type() == CV_32SC2 || m.type() == CV_32SC3 || m.type() == CV_32SC4)
	{
		signed int* p; p = (signed int*)m.data;
		p = p + i;
		return (double)(*p);
	}
	else if (m.type() == CV_32FC2 || m.type() == CV_32FC3 || m.type() == CV_32FC4)
	{
		float* p; p = (float*)m.data;
		p = p + i;
		return (double)(*p);
	}
	else if (m.type() == CV_64FC2 || m.type() == CV_64FC3 || m.type() == CV_64FC4)
	{
		double* p; p = (double*)m.data;
		p = p + i;
		return (double)(*p);
	}
	else {
		std::cerr << "# Error: getMatElm() got a Mat with unknown type: " << m.type() << "\n";
	}
	return 0;
}
double getMatElm(const cv::Mat & m, int i, int j)
{
	if (m.type() == CV_8U) return m.at<unsigned char>(i, j);
	else if (m.type() == CV_8S) return m.at<signed char>(i, j);
	else if (m.type() == CV_16U) return m.at<unsigned short>(i, j);
	else if (m.type() == CV_16S) return m.at<signed short>(i, j);
	else if (m.type() == CV_32S) return m.at<signed int>(i, j);
	else if (m.type() == CV_32F) return m.at<float>(i, j);
	else if (m.type() == CV_64F) return m.at<double>(i, j);
	else 
	{
		std::cerr << "# Error: getMatElm() does not support multi-channel Mat access.\n";
	}
	return 0; 
}

cv::Point2f selectPoint_zoom(cv::Mat img, int nLevel)
{
	int initW = 1280;
	int initH = 800;
	cv::Point2f pick = cv::Point2f(0.f, 0.f);
	cv::Rect roi;
	double fac = 4.;
	double ori_x = 0.0, ori_y = 0.0;
	double fac_x = 1., fac_y = 1.;
	cv::Mat imgSelected = img;
	// set initial image size
	fac_x = initW * 1. / img.cols;
	fac_y = initH * 1. / img.rows;
	if (fac_x > fac_y) fac_x = fac_y;
	if (fac_y > fac_x) fac_y = fac_x;
	cv::resize(img, imgSelected, cv::Size(0, 0), fac_x, fac_y, cv::INTER_NEAREST);
	for (int iLevel = 0; iLevel < nLevel; iLevel++)
	{
		roi = cv::selectROI(imgSelected, true, false);
		// if Cancel [ESC] or [C] is pressed, it returns cv::Point2f(0.f, 0.f)
		if (roi.width <= 0 || roi.height <= 0) {
			return cv::Point2f(0.f, 0.f);
		}
		if (iLevel == 0) {
			roi.x = (int) (roi.x / fac_x);
			roi.y = (int) (roi.y / fac_y);
			roi.width = (int) (roi.width / fac_x);
			roi.height = (int) (roi.height / fac_y);
			imgSelected = img.clone();
			fac_x = 1.0;
			fac_y = 1.0;
		}
		pick.x = (float) (ori_x + (roi.x + roi.width  * 0.5) / fac_x - .5);
		pick.y = (float) (ori_y + (roi.y + roi.height * 0.5) / fac_y - .5);
		imgSelected(roi).copyTo(imgSelected);
		cv::resize(imgSelected, imgSelected, cv::Size(0, 0), fac, fac, cv::INTER_NEAREST);
		ori_x += roi.x / fac_x;
		ori_y += roi.y / fac_y;
		fac_x *= fac;
		fac_y *= fac;
	}
	return pick;
}

cv::Mat Rodrigues31(cv::Mat rvec)
{
	char buf[1000];
	if (!(
		  (rvec.cols == 1 && rvec.rows == 3) || 
		  (rvec.cols == 3 && rvec.rows == 1) ||
          (rvec.cols == 3 && rvec.rows == 3)) )
	{
		snprintf(buf, 1000, "# Error: Your rvec is %d x %d, which has to be a 1-by-3, 3-by-1, or 3-by-3. "
			"Make it correct and try again.\n", rvec.rows, rvec.cols);
		std::cerr << buf;
		return cv::Mat::zeros(3, 1, CV_64F);
	}
	// If rvec is CV_32F, convert it to CV_64F
	if (rvec.type() != CV_64F)
	{
		cv::Mat rvecClone = rvec.clone();
		rvec = cv::Mat::zeros(rvecClone.rows, rvecClone.cols, CV_64F);
		for (int i = 0; i < rvecClone.rows; i++)
		{
			for (int j = 0; j < rvecClone.cols; j++)
			{
//				rvec.at<double>(i, j) = (double)rvecClone.at<float>(i, j);
				rvec.at<double>(i, j) = (double)getMatElm(rvecClone, i, j); 
			}
		}
	}
	// if rvec is a 3 by 3, convert it to a vector (not sure cv::Rodrigues returns 1 by 3 or 3 by 1.)
	if (rvec.rows == 3 && rvec.cols == 3)
	{
		cv::Mat rvecClone = rvec.clone();
		cv::Rodrigues(rvecClone, rvec);
	}

	// if rvec is a 1 by 3, convert it to 3 by 1
	if (rvec.rows == 1 && rvec.cols == 3)
	{
		cv::Mat rvecClone = rvec.clone().t();
		rvecClone.copyTo(rvec);
	}
	// 
	return rvec; 
}
cv::Mat Rodrigues13(cv::Mat rvec)
{
	bool debug = false; 
	char buf[1000];
	if (!(
		(rvec.cols == 1 && rvec.rows == 3) ||
		(rvec.cols == 3 && rvec.rows == 1) ||
		(rvec.cols == 3 && rvec.rows == 3)))
	{
		snprintf(buf, 1000, "# Error: Your rvec is %d x %d, which has to be a 1-by-3, 3-by-1, or 3-by-3. "
			"Make it correct and try again.\n", rvec.rows, rvec.cols);
		std::cerr << buf;
		return cv::Mat::zeros(3, 1, CV_64F);
	}
	// If rvec is not CV_64F, convert it to CV_64F
	if (rvec.type() == CV_32F)
	{
		cv::Mat rvecClone = rvec.clone();
		rvec = cv::Mat::zeros(rvecClone.rows, rvecClone.cols, CV_64F);
		for (int i = 0; i < rvecClone.rows; i++)
		{
			for (int j = 0; j < rvecClone.cols; j++)
			{
				rvec.at<double>(i, j) = (double)getMatElm(rvecClone, i, j);
			}
		}
	}
	// if rvec is a 3 by 3, convert it to a vector (not sure cv::Rodrigues returns 1 by 3 or 3 by 1.)
	if (rvec.rows == 3 && rvec.cols == 3)
	{
		cv::Mat rvecClone = rvec.clone();
		if (debug) std::cout << "debug: rvecClone:\n" << rvecClone << "\n"; std::cout.flush();
		cv::Rodrigues(rvecClone, rvec);
		if (debug) std::cout << "debug: rvec:\n" << rvec << "\n"; std::cout.flush();
	}

	// if rvec is a 3 by 1, convert it to 1 by 3
	if (rvec.rows == 3 && rvec.cols == 1)
	{
		if (debug) std::cout << "debug: rvec:\n" << rvec << "\n"; std::cout.flush();
		cv::Mat rvecClone;
		rvecClone = rvec.clone().t();
		if (debug) std::cout << "debug: rvec:\n" << rvec << "\n"; std::cout.flush();
		rvecClone.copyTo(rvec);
		if (debug) std::cout << "debug: rvec:\n" << rvec << "\n"; std::cout.flush();
	}
	// 
	return rvec;
}
cv::Mat Rodrigues33(cv::Mat rvec)
{
	char buf[1000];
	if (!(
		(rvec.cols == 1 && rvec.rows == 3) ||
		(rvec.cols == 3 && rvec.rows == 1) ||
		(rvec.cols == 3 && rvec.rows == 3)))
	{
		snprintf(buf, 1000, "# Error: Your rvec is %d x %d, which has to be a 1-by-3, 3-by-1, or 3-by-3. "
			"Make it correct and try again.\n", rvec.rows, rvec.cols);
		std::cerr << buf;
		return cv::Mat::zeros(3, 1, CV_64F);
	}
	// If rvec is not CV_64F, convert it to CV_64F
	if (rvec.type() != CV_64F)
	{
		cv::Mat rvecClone = rvec.clone();
		rvec = cv::Mat::zeros(rvecClone.rows, rvecClone.cols, CV_64F);
		for (int i = 0; i < rvecClone.rows; i++)
		{
			for (int j = 0; j < rvecClone.cols; j++)
			{
				rvec.at<double>(i, j) = (double)getMatElm(rvecClone, i, j);
			}
		}
	}
	// if rvec is a 3 by 1 or 1 by 3, calls Rodrigues
	if (rvec.rows * rvec.cols == 3)
	{
		cv::Mat rvecClone = rvec.clone();
		cv::Rodrigues(rvecClone, rvec);
	}
	return rvec;
}
void testRodriguesXX()
{
	std::cout << "Testing Rodrigues Mat 33 32F -> Mat 33: \n " << Rodrigues33(cv::Mat::eye(3, 3, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 33 64F -> Mat 33: \n " << Rodrigues33(cv::Mat::eye(3, 3, CV_64F)) << "\n";
	std::cout << "Testing Rodrigues Mat 33 32F -> Mat 31: \n " << Rodrigues31(cv::Mat::eye(3, 3, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 33 64F -> Mat 31: \n " << Rodrigues31(cv::Mat::eye(3, 3, CV_64F)) << "\n";
	std::cout << "Testing Rodrigues Mat 33 32F -> Mat 13: \n " << Rodrigues13(cv::Mat::eye(3, 3, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 33 64F -> Mat 13: \n " << Rodrigues13(cv::Mat::eye(3, 3, CV_64F)) << "\n";

	std::cout << "Testing Rodrigues Mat 31 32F -> Mat 33: \n " << Rodrigues33(cv::Mat::zeros(3, 1, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 64F -> Mat 33: \n " << Rodrigues33(cv::Mat::zeros(3, 1, CV_64F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 32F -> Mat 31: \n " << Rodrigues31(cv::Mat::zeros(3, 1, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 64F -> Mat 31: \n " << Rodrigues31(cv::Mat::zeros(3, 1, CV_64F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 32F -> Mat 13: \n " << Rodrigues13(cv::Mat::zeros(3, 1, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 64F -> Mat 13: \n " << Rodrigues13(cv::Mat::zeros(3, 1, CV_64F)) << "\n";

	std::cout << "Testing Rodrigues Mat 31 32F -> Mat 33: \n " << Rodrigues33(cv::Mat::zeros(1, 3, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 64F -> Mat 33: \n " << Rodrigues33(cv::Mat::zeros(1, 3, CV_64F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 32F -> Mat 31: \n " << Rodrigues31(cv::Mat::zeros(1, 3, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 64F -> Mat 31: \n " << Rodrigues31(cv::Mat::zeros(1, 3, CV_64F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 32F -> Mat 13: \n " << Rodrigues13(cv::Mat::zeros(1, 3, CV_32F)) << "\n";
	std::cout << "Testing Rodrigues Mat 31 64F -> Mat 13: \n " << Rodrigues13(cv::Mat::zeros(1, 3, CV_64F)) << "\n";

}


// Procedures:
// Step 1: Load calibration data (intr. and extr.)
//    --> fnameCalib, cmat, dvec, rvec, tvec (all of these Mat are 64-bit) 
// Step 2: Select the initial image for picking
//    --> fnameInitImg, imgInit
// Step 3: Undistortion options
//    --> doUndistortion (true or false), undistAlpha, undistCmat, undistImgSize, 
// Step 3: Pick points
//    --> nCtrlPoints, ctrlPointsImg(nCtrlPoints, 2, CV_32F)
// Step 4: Define local coordinate by assigning x(px1->px2) and y'(py1->py2) 
// Step 5: Define polygon ROI (proi1, proi2, ...) 
// Step 6: Define fixed points (for correcting camera movement) 
// Step 7: Setup image sequence file names
//    --> fsq (a FileSeq)


// Step 1: Read camera parameters (cmat, dvec, rvec, tvec) (single cam)
//         cv::Mat cmat(3, 3, CV_64F), dvec(1, n, CV_64F) (?or (n, 1, CV_64F)), rvec(3, 1, CV_64F), tvec(3, 1, CV_64F) 
// Step 2: Read initial photo
//         cv::Mat imgInit; 
// Step 3: Ask users to enter world coordinates of four rectangular corners (Pw0, Pw1, Pw2, Pw3) (counter-clockwise)
//         vector<cv::Point3f> Pw4(4) 
// Step 4: Ask users to enter world coordinates of four expanded rectangular corners (Qw0, Qw1, Qw2, Qw3) (counter-clockwise)
//         vector<cv::Point3f> Qw4(4) 
// Step 5: Project qi4 from Qw4
//         vector<cv::Point2f> qi4(4) <== cv::projectPoints( Qw4 ) 
// Step 6: Determine pxl (length of each pixel), w, h (the rectified image size is h x w)
//         float pxl;
//         int w, h;
// Step 7: Generate Q mesh
//         cv::Mat Qmesh(h * w, 1, CV_32FC3)
// Step 8: Project Q mesh to q mesh
//         vector<cv::Point2f> qmesh(h * w, 1, CV_32FC2)
// Step 9: Generate a rectified image by cv::remap()
//         (You probably need to reshape Qmesh and qmesh from (h*w, 1) to (h, w). (Use interpolation of CUBIC, or higher order) 
//         cv::Mat imgRectf; 

int FuncSingleCamWallMonitoring(int argc, char** argv)
{
	// variables
	char buf[1000];
	std::string fnameCalib;  // full path of the calibration file
	std::string fnameInitImg; // full path of the initial image (for user to pick points)
	cv::Mat cmat, dvec, rvec, tvec; 

	// Load calibration data (intr. and extr.)
	while (true)
	{
		std::cout << "\n";
		std::cout << "# 1. Enter the full path of the calibration file: \n";
		std::cout << "#   Your file needs to be written by OpenCV cv::FileStorage and includes the following items:\n";
		std::cout << "#     cameraMatrix: a matrix of cv::Mat(3, 3, CV_64F)\n";
		std::cout << "#     distortionVector: a vector of cv::Mat(N, 1, CV_64F) or cv::Mat(1, N, CV_64F), where N >= 4.\n";
		std::cout << "#     rvec: a column vector of cv::Mat(3, 1, CV_64F).\n";
		std::cout << "#     tvec: a column vector of cv::Mat(3, 1, CV_64F).\n";
		fnameCalib = readStringLineFromIstream(std::cin);
		cv::FileStorage fsCalib(fnameCalib, cv::FileStorage::READ);

		// If calibration file cannot be opened, ask user to enter again.
		if (fsCalib.isOpened() == false)
		{
			std::cerr << "# Error: Cannot open your calibration file: "
				<< fnameCalib << ". Try to enter its full path again.\n";
			continue;
		}

		// Read cmat
		fsCalib["cameraMatrix"] >> cmat;
		if (cmat.cols != 3 || cmat.rows != 3)
		{
			snprintf(buf, 1000, "# Error: Your cameraMatrix is %d x %d, which has to be 3 by 3. "
				"Make it correct and try again.\n", cmat.rows, cmat.cols);
			std::cerr << buf;
			continue;
		}
		// If cmat is not CV_64F, convert it to CV_64F
		if (cmat.type() != CV_64F)
		{
			cv::Mat cmatClone = cmat.clone();
			cmat = cv::Mat::eye(3, 3, CV_64F);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					cmat.at<double>(i, j) = (double)cmatClone.at<double>(i, j);
				}
			}
		}
		if (cmat.at<double>(0, 0) <= 0.0 || cmat.at<double>(0, 0) > 20000.0 ||
			cmat.at<double>(1, 1) <= 0.0 || cmat.at<double>(1, 1) > 20000.0 ||
			cmat.at<double>(0, 2) <= 0.0 || cmat.at<double>(0, 2) > 20000.0 ||
			cmat.at<double>(1, 2) <= 0.0 || cmat.at<double>(1, 2) > 20000.0 ||
			abs(cmat.at<double>(0, 1)) > 1e-9 ||
			abs(cmat.at<double>(1, 0)) > 1e-9 ||
			abs(cmat.at<double>(2, 0)) > 1e-9 ||
			abs(cmat.at<double>(2, 1)) > 1e-9 ||
			abs(cmat.at<double>(2, 2) - 1.0) > 1e-9)
		{
			std::cerr << "# Warning: Your cameraMatrix is unusual. You probably would like to double check it.\n";
		}
		std::cout << "# cameraMatrix (cmat):\n" << cmat << "\n";

		// Read dvec
		fsCalib["distortionVector"] >> dvec;
		if (dvec.cols != 1 && dvec.rows != 1)
		{
			snprintf(buf, 1000, "# Error: Your distortionVector is %d x %d, which has to be a vector. "
				"Make it correct and try again.\n", dvec.rows, dvec.cols);
			std::cerr << buf;
			continue;
		}
		// Transpose it to a row vector 
		if (dvec.rows != 1)
		{
			cv::Mat dvecClone;
			dvecClone = dvec.clone().t();
			dvecClone.copyTo(dvec);
		}
		// If dvec is not CV_64F, convert it to CV_64F
		if (dvec.type() != CV_64F)
		{
			cv::Mat dvecClone = dvec.clone();
			dvec = cv::Mat::zeros(dvecClone.rows, dvecClone.cols, CV_64F);
			for (int i = 0; i < dvecClone.rows; i++)
			{
				for (int j = 0; j < dvecClone.cols; j++)
				{
					dvec.at<double>(i, j) = (double)dvecClone.at<double>(i, j);
				}
			}
		}
		std::cout << "# distortionVector (dvec):\n" << dvec << "\n";

		// Read rvec
		fsCalib["rvec"] >> rvec;
		if (rvec.cols * rvec.rows != 3)
		{
			snprintf(buf, 1000, "# Error: Your rvec is %d x %d, which has to be a 1-by-3 or 3-by-1 vector. "
				"Make it correct and try again.\n", rvec.rows, rvec.cols);
			std::cerr << buf;
			continue;
		}
		// Convert rvec to a column vector 
		if (rvec.cols != 1)
		{
			cv::Mat rvecClone = rvec.clone().t();
			rvecClone.copyTo(rvec);
		}
		// If rvec is not CV_64F, convert it to CV_64F
		if (rvec.type() != CV_64F)
		{
			cv::Mat rvecClone = rvec.clone();
			rvec = cv::Mat::zeros(rvecClone.rows, rvecClone.cols, CV_64F);
			for (int i = 0; i < rvecClone.rows; i++)
			{
				for (int j = 0; j < rvecClone.cols; j++)
				{
					rvec.at<double>(i, j) = (double)rvecClone.at<double>(i, j);
				}
			}
		}
		std::cout << "# rvec :\n" << rvec << "\n";

		// Read tvec
		fsCalib["tvec"] >> tvec;
		if (tvec.cols * tvec.rows != 3)
		{
			snprintf(buf, 1000, "# Error: Your tvec is %d x %d, which has to be a 1-by-3 or 3-by-1 vector. "
				"Make it correct and try again.\n", tvec.rows, tvec.cols);
			std::cerr << buf;
			continue;
		}
		// Convert tvec to a column vector 
		if (tvec.cols != 1)
		{
			cv::Mat tvecClone = tvec.clone().t();
			tvecClone.copyTo(tvec);
		}
		// If tvec is not CV_64F, convert it to CV_64F
		if (tvec.type() != CV_64F)
		{
			cv::Mat tvecClone = tvec.clone();
			tvec = cv::Mat::zeros(tvecClone.rows, tvecClone.cols, CV_64F);
			for (int i = 0; i < tvecClone.rows; i++)
			{
				for (int j = 0; j < tvecClone.cols; j++)
				{
					tvec.at<double>(i, j) = (double)tvecClone.at<double>(i, j);
				}
			}
		}
		std::cout << "# tvec :\n" << tvec << "\n";
		break;
	} // end of while loop of reading calibration file 



	return 0; 
}






