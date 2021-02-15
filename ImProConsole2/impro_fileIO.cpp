#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>

#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "impro_fileIO.h"



int writeMatToCsvFile(const cv::Mat& m, std::string fname, std::string cfmt)
{
	std::ofstream f(fname.c_str(), std::ofstream::out);
	char buf[1000];
	if (f.is_open() == false) {
		std::cerr << "# Warning: Cannot open file for writing: " << fname << "\n";
		return -1;
	}
	if (m.cols <= 0 || m.rows <= 0) {
		std::cerr << "# Warning: Got an empty matrix. Cannot write to " << fname << "\n";
		return -1;
	}
	// output (check type)
	if (m.type() == CV_8U) {
		if (cfmt.size() <= 1) cfmt = "%3d,";
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at<uchar>(i, j));
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_8UC2) {
		if (cfmt.size() <= 1) cfmt = "%3d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec2b>(i, j)[0],
					m.at <cv::Vec2b>(i, j)[1]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_8UC3) {
		if (cfmt.size() <= 1) cfmt = "%3d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3b>(i, j)[0],
					m.at <cv::Vec3b>(i, j)[1], m.at <cv::Vec3b>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_8UC4) {
		if (cfmt.size() <= 1) cfmt = "%3d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3b>(i, j)[0],
					m.at <cv::Vec3b>(i, j)[1], m.at <cv::Vec3b>(i, j)[2], 
					m.at <cv::Vec3b>(i, j)[3]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16U) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at<ushort>(i, j));
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16UC2) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec2w>(i, j)[0],
					m.at <cv::Vec2w>(i, j)[1]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16UC3) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3w>(i, j)[0],
					m.at <cv::Vec3w>(i, j)[1], m.at <cv::Vec3w>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16UC4) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec4w>(i, j)[0],
					m.at <cv::Vec4w>(i, j)[1], m.at <cv::Vec4w>(i, j)[2],
					m.at <cv::Vec4w>(i, j)[3]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16S) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at<short>(i, j));
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16SC2) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec2s>(i, j)[0],
					m.at <cv::Vec2s>(i, j)[1]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16SC3) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3s>(i, j)[0],
					m.at <cv::Vec3s>(i, j)[1], m.at <cv::Vec3s>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_16SC4) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec4s>(i, j)[0],
					m.at <cv::Vec4s>(i, j)[1], m.at <cv::Vec4s>(i, j)[2],
					m.at <cv::Vec4s>(i, j)[3]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32S) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at<int>(i, j));
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32SC2) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec2i>(i, j)[0],
					m.at <cv::Vec2i>(i, j)[1]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32SC3) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3i>(i, j)[0],
					m.at <cv::Vec3i>(i, j)[1], m.at <cv::Vec3i>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32SC4) {
		if (cfmt.size() <= 1) cfmt = "%d,";
		for (int i = 0; i < m.rows; i++) {
			cfmt = cfmt + cfmt + cfmt + cfmt;
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec4i>(i, j)[0],
					m.at <cv::Vec4i>(i, j)[1], m.at <cv::Vec4i>(i, j)[2],
					m.at <cv::Vec4i>(i, j)[3]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32F) {
		if (cfmt.size() <= 1) cfmt = "%15.8e,";
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at<float>(i, j));
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32FC2) {
		if (cfmt.size() <= 1) cfmt = "%15.8e,";
		cfmt = cfmt + cfmt;
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(),
					m.at <cv::Vec2f>(i, j)[0], m.at <cv::Vec2f>(i, j)[1]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32FC3) {
		if (cfmt.size() <= 1) cfmt = "%15.8e,";
		cfmt = cfmt + cfmt + cfmt;
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3f>(i, j)[0],
					m.at <cv::Vec3f>(i, j)[1], m.at <cv::Vec3f>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_32FC4) {
		if (cfmt.size() <= 1) cfmt = "%15.8e,";
		cfmt = cfmt + cfmt + cfmt + cfmt;
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec4f>(i, j)[0],
					m.at <cv::Vec4f>(i, j)[1], m.at <cv::Vec4f>(i, j)[2], 
					m.at <cv::Vec4f>(i, j)[3]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_64F) {
		if (cfmt.size() <= 1) cfmt = "%24.16e,";
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at<double>(i, j));
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_64FC2) {
		if (cfmt.size() <= 1) cfmt = "%24.16e,";
		cfmt = cfmt + cfmt;
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec2d>(i, j)[0],
					m.at <cv::Vec2d>(i, j)[1]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_64FC3) {
		if (cfmt.size() <= 1) cfmt = "%24.16e,";
		cfmt = cfmt + cfmt + cfmt;
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec3d>(i, j)[0],
					m.at <cv::Vec3d>(i, j)[1], m.at <cv::Vec3d>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else if (m.type() == CV_64FC4) {
		if (cfmt.size() <= 1) cfmt = "%24.16e,";
		cfmt = cfmt + cfmt + cfmt + cfmt;
		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				snprintf(buf, 1000, cfmt.c_str(), m.at <cv::Vec4d>(i, j)[0],
					m.at <cv::Vec4d>(i, j)[1], m.at <cv::Vec4d>(i, j)[2]);
				f << buf;
			}
			f << "\n";
		}
	}
	else {
		std::cerr << "# Warning: unknown type to write to file: " << fname << "\n";
	}
	return 0;
}
