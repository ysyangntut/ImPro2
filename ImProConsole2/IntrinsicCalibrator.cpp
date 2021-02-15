#define _CRT_SECURE_NO_WARNINGS

#include "FileSeq.h"
#include "IntrinsicCalibrator.h"

#include <vector>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

//#include "ImProUtil.h"
#include "impro_util.h"
#include "ImagePointsPicker.h"

using namespace std;
using namespace cv;

static vector<Point2f> emptyVecPoint2f;
static vector<Point3f> emptyVecPoint3f;


vector<double> proj_err_points(
	const vector<cv::Point2f> imgPoints,
	const vector<cv::Point3f> objPoints,
	const cv::Mat cmat, const cv::Mat dvec,
	const cv::Mat rvec, const cv::Mat tvec) {
	// vector err
	vector<double> err(imgPoints.size(), 0);
	// convert rvec if it is 3x3
	if (rvec.cols == 3 && rvec.rows == 3)
		cv::Rodrigues(rvec, rvec);
	// run projection
	vector<Point2f> projectedImgPoints = imgPoints;
	cv::projectPoints(objPoints,
		rvec, tvec,
		cmat, dvec, projectedImgPoints);
	// calculate error
	for (int j = 0; j < projectedImgPoints.size(); j++)
	{
		double dx = projectedImgPoints[j].x - imgPoints[j].x;
		double dy = projectedImgPoints[j].y - imgPoints[j].y;
		err[j] = sqrt(dx * dx + dy * dy);
	}
	return err;
}

vector<double> proj_err_lines(
	const vector<cv::Point2f> imgPoints,
	const cv::Mat cmat, const cv::Mat dvec) {
	// vector err
	vector<double> err(imgPoints.size(), 0);
	// run projection
	vector<Point2f> projectedImgPoints = imgPoints;
	cv::undistortPoints(imgPoints, projectedImgPoints, cmat, dvec);
	// fx, fy, cx, cy
	float fx = (float) cmat.at<double>(0, 0); 
	float fy = (float) cmat.at<double>(1, 1);
	float cx = (float) cmat.at<double>(0, 2);
	float cy = (float) cmat.at<double>(1, 2);
	// calculate error
	for (int i = 0; i < projectedImgPoints.size() / 3; i++)
	{
		int j0 = i * 3, j1 = j0 + 1, j2 = j0 + 2;
		double x0 = fx * projectedImgPoints[j0].x + cx;
		double y0 = fy * projectedImgPoints[j0].y + cy;
		double x1 = fx * projectedImgPoints[j1].x + cx;
		double y1 = fy * projectedImgPoints[j1].y + cy;
		double x2 = fx * projectedImgPoints[j2].x + cx;
		double y2 = fy * projectedImgPoints[j2].y + cy;
		double e0 = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
		double e1 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
		double e2 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
		double ex = max( max(e0, e1), e2); 
		double se = .5 * (e0 + e1 + e2); 
		double ar = sqrt(se * (se - e0) * (se - e1) * (se - e2)); 
		double ht = ar / ex; // smallest height: area/max(edge)
		err[j0] = ht / 3;
		err[j1] = ht / 3 * 2;
		err[j2] = ht / 3;
	}
	return err;
}

IntrinsicCalibrator::IntrinsicCalibrator()
{
	this->init(); 
}


IntrinsicCalibrator::~IntrinsicCalibrator()
{
	this->log("IntrinsicCalibrator stopped logging.");
}

IntrinsicCalibrator::IntrinsicCalibrator(std::string theDir)
	:IntrinsicCalibrator()
{
	this->init();
	this->imsq.setDir(theDir); 
	this->log("IntrinsicCalibrator started logging.");
}

void IntrinsicCalibrator::init()
{
	this->n_calib_imgs = 0;
	this->calib_valid_fnames.clear();
	this->calib_valid_rms.clear();
	//	this->calib_flag = cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
	this->calib_flag = cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
	this->calib_flag |= cv::CALIB_USE_INTRINSIC_GUESS; 
	this->cmat = cv::Mat::zeros(3, 3, CV_64F);
	this->dvec = cv::Mat::zeros(1, 8, CV_64F);
//	this->rmat = cv::Mat::zeros(3, 3, CV_64F);
//	this->rvec = cv::Mat::zeros(3, 1, CV_64F);
//	this->tvec = cv::Mat::zeros(3, 1, CV_64F);
	this->rmats44.clear(); 
//	this->num_corners_along_width = 7;  // assuming it is a regular chessboard 
//	this->num_corners_along_height = 7; // (8 x 8 squares --> 7 x 7 internal corners)
//	this->square_width = 50.8;       // assuming each square is 2-inch (50.8 mm) wide 
//	this->square_height = 50.8;      // assuming each square is 2-inch (50.8 mm) high 
//	this->need_calibrate = true;
	this->imgSize = cv::Size(0, 0);
	this->calib_rms = nanf("");

	// set log file name 
	// (as this function is supposed to be called in the very beginning)
	std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	char logFilenamec[100];
	snprintf(logFilenamec, 100, "log_IntrinsicCalibrator_%4d%02d%02d%02d%02d%02d.log",
		now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
		now->tm_hour, now->tm_min, now->tm_sec);
	this->logFilename = std::string(logFilenamec);
}

FileSeq & IntrinsicCalibrator::fileSeq()
{
	return this->imsq; 
}

void IntrinsicCalibrator::createTemplateInputFile()
{
	this->writeToFsFile(this->imsq.directory() + "IntrinsicCalibratorTemplateInput.xml");
	this->writeToFsFile(this->imsq.directory() + "IntrinsicCalibratorTemplateInput.yaml");
	return;
}

void IntrinsicCalibrator::setCalibrationBoard(int type, int num_corners_along_width, int num_corners_along_height,
	double square_width, double square_height)
{
//	this->board_type = type; 
//	this->num_corners_along_width = num_corners_along_width;
//	this->num_corners_along_height = num_corners_along_height;
//	this->square_width = square_width; 
//	this->square_height = square_height; 
////	this->need_calibrate = true;
//	// log
//	char strbuf[1000];
//	snprintf(strbuf, 1000, "Set board size: (%d, %d). Cell size: (%f, %f)", 
//		this->num_corners_along_width, this->num_corners_along_height, 
//		this->square_width, this->square_height);
//	this->log(strbuf); 
}

int IntrinsicCalibrator::setFileSeq(const FileSeq & _imsq)
{
	// When user resets the file sequence of images, this program also resets the 
	// calibration parameters.
//	this->need_calibrate = true;
	this->calib_objPoints.clear();
	this->calib_imgPoints.clear();
	this->calib_valid_fnames.clear();
	this->imgSize = cv::Size(0, 0);
	this->cmat = cv::Mat::zeros(3, 3, CV_64F);
	this->dvec = cv::Mat::zeros(1, 8, CV_64F);
//	this->rmat = cv::Mat::zeros(3, 3, CV_64F);
//	this->rvec = cv::Mat::zeros(3, 1, CV_64F);
//	this->tvec = cv::Mat::zeros(3, 1, CV_64F);
	this->rmats44.clear();
	this->calib_rms = nanf("");

	// But some of the parameters are not changed by calling setFileSequence.
	// For example, calib_flag, num_corners_along_width, num_corners_along_height, 

	int nfile = _imsq.num_files();
	if (nfile <= 0)
		return 0;
	this->imsq = _imsq;

	return 0;
}

int IntrinsicCalibrator::setImageSize(int w, int h)
{
	if (w > 0 && h > 0)
		this->imgSize = cv::Size(w, h); 
	else
	{
		// try to find from photo file (the first file)
		if (this->fileSeq().num_files() > 0) {
			cv::Mat img = cv::imread(this->getFileSeq().filename(0)); 
			this->imgSize = img.size(); 
		}
		else if (this->imgPoints().size() > 0) {
			// (assuming the average of image points is about at the center, 
			// and assuming calibration is not sensitive to the image size)
			cv::Size _imgSize(0, 0); 
			for (int i = 0; i < this->imgPoints()[0].size(); i++) {
				_imgSize.width += (int)(this->imgPoints()[0][i].x + 0.5f);
				_imgSize.height += (int)(this->imgPoints()[0][i].y + 0.5f);
			}
			_imgSize.width = (int)(_imgSize.width * 2 / this->imgPoints()[0].size());
			_imgSize.height = (int)(_imgSize.height * 2 / this->imgPoints()[0].size());
			this->imgSize = _imgSize; 
		}
		else {
			this->imgSize.width = 3000;   // dirty setting
			this->imgSize.height = 2000;  // dirty setting
		}
	}
	return 0;
}

cv::Size IntrinsicCalibrator::imageSize() const
{
	return this->imgSize;
}

int IntrinsicCalibrator::addCalibrationPhoto(const cv::Mat & img, const cv::Size bsize_w_h, double square_w, double square_h)
{
	// Check data
	if (img.cols <= 0 || img.rows <= 0)
		return -1;
	cv::Mat _img;
	if (img.channels() != 1)
		cvtColor(img, _img, cv::COLOR_BGR2GRAY);
	else
		_img = img;
	this->imgSize = cv::Size(img.cols, img.rows);

	vector<Point3f> tmp_objPoints(bsize_w_h.width * bsize_w_h.height);
	vector<Point2f> tmp_imgPoints(bsize_w_h.width * bsize_w_h.height);

	// Try to find the corners in the photo. If found, add this photo and corners points. 
	bool bres = findChessboardCornersSubpix(_img, Size(bsize_w_h.width, bsize_w_h.height),
		tmp_imgPoints);
	if (bres == true) {

		this->n_calib_imgs++;
		// set this valid photo name to "Not assigned" as this function does not
		// have the file name. 
		if (this->calib_valid_fnames.size() < n_calib_imgs) {
			this->calib_valid_fnames.resize(n_calib_imgs);
		}
		this->calib_valid_fnames[this->n_calib_imgs - 1] = string("Not assigned.");
		// set the object points
		calib_objPoints.push_back(
			create3DChessboardCorners(Size(bsize_w_h.width, bsize_w_h.height),
			(float)square_w, (float)square_h));
		// set the image points
		calib_imgPoints.push_back(tmp_imgPoints);
//		this->need_calibrate = true;
	}
	else {
		return -1;
	}

	// calibrate the camera and update cmat and dvec
	// this->calibrate(); 
	return 1;
}

int IntrinsicCalibrator::findCorners(int idx, cv::Size bSize, 
	float sqw, float sqh, int board_type)
{
	// check idx
	if (idx < 0 || idx >= this->imsq.num_files()) {
		char buf[1000];
		snprintf(buf, 1000, " Error: findCorners(): Index %d is out of range (%d).", idx, this->imsq.num_files());
		this->log(buf); 
		return -1;
	}
	
//	// save calibration type to board type (chess/grid/grid-unsym)
//	if (this->cal_types.size() < idx + 1)
//		this->cal_types.resize(idx + 1, -1); 
//	this->cal_types[idx] = board_type; 

	// Read file and load image
	std::string fname = this->imsq.fullPathOfFile(idx); 
	cv::Mat img = cv::imread(fname, cv::IMREAD_GRAYSCALE); 
	if (img.rows <= 0 || img.cols <= 0) {
		char buf[1000];
		snprintf(buf, 1000, " Error: findCorners(): Cannot load image from file %s.", fname.c_str());
		this->log(buf); 
		return -1;
	}

	// prepare for finding corners
//	int num_corners = this->num_corners_along_width * this->num_corners_along_height; 
	int num_corners = bSize.width * bSize.height; 
//	cv::Size bSize(this->num_corners_along_width, this->num_corners_along_height);
	vector<Point2f> tmp_imgPoints(num_corners);

	// Check find
	if (this->findingCornersResult.size() <= idx)
		this->findingCornersResult.resize((size_t)(idx + 1));

	// Try to find the corners in the photo. If found, add this photo and corners points. 
	bool bres = false;
	// Check if corners have been found before and saved to a file
	tmp_imgPoints.clear(); 
	std::string cornersFnameCheck = extFilenameRemoved(fname) + "_corners.xml";
	cv::FileStorage fsCornersCheck(cornersFnameCheck, cv::FileStorage::READ); 
	if (fsCornersCheck.isOpened()) {
		fsCornersCheck["CornersVecPoint2f"] >> tmp_imgPoints;
		if (tmp_imgPoints.size() == num_corners) bres = true;
	}
	// If corners not read from file, find them by corner finder. 
	if (bres == false) 
	{
		if (board_type == 1)
			bres = findChessboardCornersSubpix(img, bSize, tmp_imgPoints);
		else if (board_type == 2) {
			bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints);
			if (bres == 0) {
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints, CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
			}
			if (bres == 0) {
				bitwise_not(img, img);
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints);
			}
			if (bres == 0) {
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints, CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
			}
			if (bres == 0) {
				cv::resize(img, img, cv::Size(0, 0), 0.5, 0.5, cv::INTER_LANCZOS4);
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints);
				if (bres == true)
					for (int i = 0; i < tmp_imgPoints.size(); i++) {
						tmp_imgPoints[i].x *= 2.0f;
						tmp_imgPoints[i].y *= 2.0f;
					}
			}
			if (bres == 0) {
				cv::resize(img, img, cv::Size(0, 0), 0.5, 0.5, cv::INTER_LANCZOS4);
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints);
				if (bres == true)
					for (int i = 0; i < tmp_imgPoints.size(); i++) {
						tmp_imgPoints[i].x *= 4.0f;
						tmp_imgPoints[i].y *= 4.0f;
					}
			}
			if (bres == 0) {
				cv::resize(img, img, cv::Size(0, 0), 0.5, 0.5, cv::INTER_LANCZOS4);
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints);
				if (bres == true)
					for (int i = 0; i < tmp_imgPoints.size(); i++) {
						tmp_imgPoints[i].x *= 8.0f;
						tmp_imgPoints[i].y *= 8.0f;
					}
			}
			if (bres == 0) {
				cv::resize(img, img, cv::Size(0, 0), 0.5, 0.5, cv::INTER_LANCZOS4);
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints);
				if (bres == true)
					for (int i = 0; i < tmp_imgPoints.size(); i++) {
						tmp_imgPoints[i].x *= 16.f;
						tmp_imgPoints[i].y *= 16.f;
					}
			}
		}
		else if (board_type == 3) {
			bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints, CALIB_CB_ASYMMETRIC_GRID);
			if (bres == 0) {
				bres = cv::findCirclesGrid(img, bSize, tmp_imgPoints, CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
			}
		}
	}
	
	if (bres == true) {
		// set image size
		this->imgSize = img.size();

		if (this->n_calib_imgs <= idx)
			this->n_calib_imgs = idx + 1;
		// Copy corners image points to calib_imgPoints[idx]
		//   Make sure calib_imgPoints has sufficent size
		if (this->calib_imgPoints.size() <= idx)
			this->calib_imgPoints.resize((size_t)(idx + 1));
		this->calib_imgPoints[idx] = tmp_imgPoints;
		// Set result
		this->findingCornersResult[idx] = 1;
		this->log(" findCorners(): Found corners in file " + fname);
		// draw corners to file
		std::string drawFname = appendSubstringBeforeLastDot(fname, "_cornersDrawn"); 
		cv::Mat drawImg = cv::imread(fname); 
		drawChessboardCorners(drawImg, bSize, this->calib_imgPoints[idx], bres);
		cv::imwrite(drawFname, drawImg, std::vector<int>({ cv::IMWRITE_JPEG_QUALITY, 25 }));
		// save corners points
		if (bres) {
			std::string cornersFname = extFilenameRemoved(fname) + "_corners.xml";
			cv::FileStorage fsCorners(cornersFname, cv::FileStorage::WRITE);
			fsCorners << "CornersVecPoint2f" << this->calib_imgPoints[idx];
			fsCorners.release(); 
		}
		// set object points
		this->setBoardObjPoints(idx, bSize, sqw, sqh, board_type); 
		// save calibration type to board type (chess/grid/grid-unsym)
		if (this->cal_types.size() < idx + 1)   
			this->cal_types.resize(idx + 1, 0);
		this->cal_types[idx] = board_type; 
	}
	else {
		// failed to find all corners
		this->findingCornersResult[idx] = -1;
		// clear calibration type to board type (chess/grid/grid-unsym)
		if (this->cal_types.size() < idx + 1)
			this->cal_types.resize(idx + 1, 0);
		this->cal_types[idx] = 0; // set cal type to "not assigned" 
		return -1;
	}
	if (this->findingCornersResult[idx] == -1)
		return -1;
	return 0;
}

vector<int>& IntrinsicCalibrator::calTypes()
{
	return this->cal_types;
}
const vector<int>& IntrinsicCalibrator::calTypes() const
{
	return this->cal_types;
}

vector<vector<Point2f>>& IntrinsicCalibrator::imgPoints()
{
	return this->calib_imgPoints; 
}
const vector<vector<Point2f>>& IntrinsicCalibrator::imgPoints() const
{
	return this->calib_imgPoints;
}

vector<vector<Point3f>>& IntrinsicCalibrator::objPoints()
{
	return this->calib_objPoints;
}
const vector<vector<Point3f>>& IntrinsicCalibrator::objPoints() const
{
	return this->calib_objPoints;
}


int IntrinsicCalibrator::writeImgPointsToFile(int idx)
{
	std::string txtFilename, xmlFilename, ymlFilename, photoName;
	photoName = this->imsq.fullPathOfFile(idx);
	txtFilename = extFilenameRemoved(photoName) + "_xi.txt"; 
	xmlFilename = extFilenameRemoved(photoName) + "_xi.xml";
	ymlFilename = extFilenameRemoved(photoName) + "_xi.yaml";
	// txt 
	ofstream oftxt(txtFilename); 
	oftxt << this->calib_imgPoints[idx].size() << endl;
	for (int i = 0; i < this->calib_imgPoints[idx].size(); i++) {
		oftxt << this->calib_imgPoints[idx][i].x << "\t"
			<< this->calib_imgPoints[idx][i].y << endl;
	}
	oftxt.close(); 
	// xml
	cv::FileStorage fs(xmlFilename, cv::FileStorage::WRITE); 
	fs << "VecPoint2f" << this->calib_imgPoints[idx];
	fs.release(); 
	// yaml
	cv::FileStorage fs2(ymlFilename, cv::FileStorage::WRITE);
	fs2 << "VecPoint2f" << this->calib_imgPoints[idx];
	fs2.release();
	return 0;
}

int IntrinsicCalibrator::writeImgsPointsToFiles()
{
	for (int i = 0; i < this->calib_imgPoints.size(); i++)
		this->writeImgPointsToFile(i);
	return 0;
}

int IntrinsicCalibrator::writeObjPointsToFile(int idx)
{
	std::string txtFilename, xmlFilename, ymlFilename, photoName;
	photoName = this->imsq.fullPathOfFile(idx);
	txtFilename = extFilenameRemoved(photoName) + "_xxi.txt";
	xmlFilename = extFilenameRemoved(photoName) + "_xxi.xml";
	ymlFilename = extFilenameRemoved(photoName) + "_xxi.yaml";
	// txt 
	ofstream oftxt(txtFilename);
	oftxt << this->calib_imgPoints[idx].size() << endl;
	for (int i = 0; i < this->calib_objPoints[idx].size(); i++) {
		oftxt << this->calib_objPoints[idx][i].x << "\t"
			  << this->calib_objPoints[idx][i].y << "\t" 
			  << this->calib_objPoints[idx][i].z << endl;
	}
	oftxt.close();
	// xml
	cv::FileStorage fs(xmlFilename, cv::FileStorage::WRITE);
	fs << "VecPoint3f" << this->calib_objPoints[idx];
	fs.release();
	// yaml
	cv::FileStorage fs2(ymlFilename, cv::FileStorage::WRITE);
	fs2 << "VecPoint3f" << this->calib_objPoints[idx];
	fs2.release();
	return 0;
}

int IntrinsicCalibrator::writeObjsPointsToFiles()
{
	for (int i = 0; i < this->calib_objPoints.size(); i++)
		this->writeObjPointsToFile(i);
	return 0;
}

int IntrinsicCalibrator::findAllCorners(cv::Size bSize, 
	float sqw, float sqh, int board_type)
{
	// try to find corners in add all FileSequence files
	// If corners are found, add the file into calib_valid_fnames.
	int nfile = imsq.num_files();
	if (nfile <= 0)
		return 0;
	for (int i = 0; i < nfile; i++)
	{
		this->findCorners(i, bSize, sqw, sqh, board_type); 
//		// read image 
//		cv::Mat img = cv::imread(imsq.fullPathOfFile(i), IMREAD_GRAYSCALE);
//		// try to find corners. If found, add corners to calib_objPoints and calib_imgPoints.
//		int added = this->addCalibrationPhoto(img,
//			cv::Size(this->num_corners_along_width, this->num_corners_along_height),
//			this->square_width, this->square_height);
//		if (added == 1) {
//			if (this->calib_valid_fnames.size() < n_calib_imgs) {
//				this->calib_valid_fnames.resize(n_calib_imgs);
//			}
//			this->calib_valid_fnames[this->n_calib_imgs - 1] = imsq.filename(i);
////			this->need_calibrate = true;
//		}
	}
	return (int) this->calib_valid_fnames.size();
}

int IntrinsicCalibrator::setBoardObjPoints(int idx, cv::Size bSize, float sqw, float sqh,
	int board_type)
{
	if (board_type == 0) {
		this->log(" IntrinsicCalibrator:Warning: Board type is not set.\n");
		return -1;
	}
	if (this->calib_objPoints.size() < idx + 1)
		this->calib_objPoints.resize(idx + 1); 
	this->calib_objPoints[idx].clear(); 
	if (board_type == 1 || board_type == 2) {
		for (int i = 0; i < bSize.height; i++)
			for (int j = 0; j < bSize.width; j++)
				this->calib_objPoints[idx].push_back(
					Point3f(float(j * sqw), 
						float(i * sqh), 0));
	}
	else if (board_type == 3) {
		for (int i = 0; i < bSize.height; i++)
			for (int j = 0; j < bSize.width; j++)
				this->calib_objPoints[idx].push_back(
					Point3f(float((2 * j + i % 2) * sqw), 
						float(i * sqh), 0));
	}
	else {
		this->log(" IntrinsicCalibrator:Warning: Unknown board type when setting obj points.\n");
		return -1;
	}
	return 0;
}

int IntrinsicCalibrator::defineUserPoints(int idx, 
	const vector<cv::Point2f>& imgPoints, 
	const vector<cv::Point3f>& objPoints, 
	cv::Size _imgSize)
{
	if (this->calib_imgPoints.size() <= idx) {
		this->calib_imgPoints.resize(idx + 1);
		this->n_calib_imgs = (int) this->calib_imgPoints.size(); 
	}
	this->calib_imgPoints[idx] = imgPoints;
	if (this->calib_objPoints.size() <= idx)
		this->calib_objPoints.resize(idx + 1);
	this->calib_objPoints[idx] = objPoints;
	if (this->cal_types.size() <= idx)
		this->cal_types.resize(idx + 1);
	this->cal_types[idx] = 11;
	// if imgSize is not defined, use double of average points
	// (assuming the average of image points is about at the center, 
	// and assuming calibration is not sensitive to the image size)
	if (_imgSize.width <= 0 || _imgSize.height <= 0) {
		_imgSize.width = 0; _imgSize.height = 0; 
		for (int i = 0; i < imgPoints.size(); i++) {
			_imgSize.width += (int)(imgPoints[i].x + 0.5f); 
			_imgSize.height += (int)(imgPoints[i].y + 0.5f);
		}
		_imgSize.width = (int) (_imgSize.width * 2 / imgPoints.size()); 
		_imgSize.height = (int)(_imgSize.height * 2 / imgPoints.size());
	}
	this->imgSize = _imgSize; 
	return 0;
}

int IntrinsicCalibrator::defineUserPoints(int idx,
	const vector<cv::Point2f>& imgPoints,
	const vector<cv::Point3d>& objPoints,
	cv::Size _imgSize)
{
	vector<cv::Point3f> objPoints3f; 
	objPoints3f.resize(objPoints.size()); 
	for (int i = 0; i < objPoints3f.size(); i++) {
		objPoints3f[i].x = (float) objPoints[i].x; 
		objPoints3f[i].y = (float) objPoints[i].y;
		objPoints3f[i].z = (float) objPoints[i].z;
	}
	this->defineUserPoints(idx, imgPoints, objPoints3f, _imgSize); 
	return 0; 
}

int IntrinsicCalibrator::setCameraMatrix(const cv::Mat & _cmat)
{
	_cmat.copyTo(this->cmat);
	return 0;
}

int IntrinsicCalibrator::setDistortionVector(const cv::Mat & _dvec)
{
	_dvec.copyTo(this->dvec); 
	return 0;
}

int IntrinsicCalibrator::solvePnp(int valid_photo_id)
{
	cv::Mat rvec(1,1, CV_64FC3), tvec(1,1, CV_64FC3); 
	if (this->calib_objPoints.size() <= valid_photo_id || this->calib_imgPoints.size() <= valid_photo_id)
	{
		cerr << "IntrinsicCalibrator::solvePnp() does not have sufficient obj/img points.\n";
		return -1;
	}
	cv::solvePnP(this->calib_objPoints[valid_photo_id],
		this->calib_imgPoints[valid_photo_id],
		this->cmat,
		this->dvec,
		rvec, tvec); 
	// copy rvec and tvec to this->rvecs[valid_photo_id] and this->tvecs[valid_photo_id]
	if (this->rvecs.size() <= valid_photo_id)
		this->rvecs.resize(valid_photo_id + 1);
	this->rvecs[valid_photo_id] = rvec.at<Vec3d>(0, 0);
	if (this->tvecs.size() <= valid_photo_id)
		this->tvecs.resize(valid_photo_id + 1);
	this->tvecs[valid_photo_id] = tvec.at<Vec3d>(0, 0);
	// refresh project points
	this->projection_points_vecvec(); 
	return 0;
}

const vector<cv::Point2f>& IntrinsicCalibrator::imagePoints(int idx)
{
	if (idx < this->calib_imgPoints.size()) {
		return this->calib_imgPoints[idx];
	}
	else {
		return emptyVecPoint2f;
	}
}

const vector<cv::Point3f>& IntrinsicCalibrator::objectPoints(int idx)
{
	if (idx < this->calib_objPoints.size()) {
		return this->calib_objPoints[idx];
	}
	else {
		return emptyVecPoint3f;
	}
}

int IntrinsicCalibrator::setFlag_FixAspectRatio(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_ASPECT_RATIO; else this->calib_flag &= ~cv::CALIB_FIX_ASPECT_RATIO;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixFocalLength(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_FOCAL_LENGTH; else this->calib_flag &= ~cv::CALIB_FIX_FOCAL_LENGTH;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixK1(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_K1; else this->calib_flag &= ~cv::CALIB_FIX_K1;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixK2(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_K2; else this->calib_flag &= ~cv::CALIB_FIX_K2;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixK3(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_K3; else this->calib_flag &= ~cv::CALIB_FIX_K3;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixK4(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_K4; else this->calib_flag &= ~cv::CALIB_FIX_K4;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixK5(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_K5; else this->calib_flag &= ~cv::CALIB_FIX_K5;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixK6(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_K6; else this->calib_flag &= ~cv::CALIB_FIX_K6;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixPrincipalPoint(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_PRINCIPAL_POINT; else this->calib_flag &= ~cv::CALIB_FIX_PRINCIPAL_POINT;
	return 0;
}

int IntrinsicCalibrator::setFlag_RationalModel(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_RATIONAL_MODEL; else this->calib_flag &= ~cv::CALIB_RATIONAL_MODEL;
	return 0;
}

int IntrinsicCalibrator::setFlag_FixTangentDist(bool f)
{
	if (f) this->calib_flag |= cv::CALIB_FIX_TANGENT_DIST; else this->calib_flag &= ~cv::CALIB_FIX_TANGENT_DIST;
	return 0;
}

int IntrinsicCalibrator::setFlagByAsking()
{
	int flag = 0;
	vector<string> flagNames(22);
	flagNames[0] = "cv::CALIB_USE_INTRINSIC_GUESS"; 
	flagNames[1] = "cv::CALIB_FIX_ASPECT_RATIO";
	flagNames[2] = "cv::CALIB_FIX_PRINCIPAL_POINT";
	flagNames[3] = "cv::CALIB_ZERO_TANGENT_DIST";
	flagNames[4] = "cv::CALIB_FIX_FOCAL_LENGTH";
	flagNames[5] = "cv::CALIB_FIX_K1";
	flagNames[6] = "cv::CALIB_FIX_K2";
	flagNames[7] = "cv::CALIB_FIX_K3";
	flagNames[8] = "cv::CALIB_FIX_INTRINSIC";
	flagNames[9] = "cv::CALIB_SAME_FOCAL_LENGTH";
	flagNames[10] = "cv::CALIB_ZERO_DISPARITY";
	flagNames[11] = "cv::CALIB_FIX_K4";
	flagNames[12] = "cv::CALIB_FIX_K5";
	flagNames[13] = "cv::CALIB_FIX_K6";
	flagNames[14] = "cv::CALIB_RATIONAL_MODEL";
	flagNames[15] = "cv::CALIB_THIN_PRISM_MODEL";
	flagNames[16] = "cv::CALIB_FIX_S1_S2_S3_S4";
	flagNames[17] = "";
	flagNames[18] = "cv::CALIB_TILTED_MODEL";
	flagNames[19] = "cv::CALIB_FIX_TAUX_TAUY";
	flagNames[20] = "";
	flagNames[21] = "cv::CALIB_FIX_TANGENT_DIST"; 

	for (int i = 0; i < flagNames.size(); i++) {
		if (flagNames[i].length() == 0) continue;  // skip unknown flag 
		std::cout << "Set " << flagNames[i] << " (0 or 1. OpenCV default is 0): ";
		int thisFlag = 0;
		while (true) {
			string strbuf; cin >> strbuf;
			if (strbuf[0] == '#') continue;
			try {
				thisFlag = stoi(strbuf);
				break;
			}
			catch (...) {
				continue;
			}
		}
		if (thisFlag > 0) flag |= (1 << i);
	}
	"The following calibration flags are set.:\n";
	for (int i = 0; i < flagNames.size(); i++) {
		if (flagNames[i].length() == 0) continue;  // skip unknown flag 
		if (flag & (1 << i))
			cout << flagNames[i] << " is set.\n";
	}
	this->calib_flag = flag; 
	return 0;
}

int IntrinsicCalibrator::calibrate(int flagType, int flag)
{
	// check if corners are found
//	if (this->imsq.num_files() > 0 && this->calib_imgPoints.size() == 0 &&
//		this->calib_objPoints.size() == 0) {
//		this->findAllCorners();
//	}
	// remove photos with nanf points before running calibration 
	std::vector<std::vector<cv::Point2f> > imgPointsValid;
	std::vector<std::vector<cv::Point3f> > objPointsValid;
	std::vector<int> o2n(this->calib_imgPoints.size(), -1), n2o;
	int num_valid_calib_img = 0; 
	// check each calibration image 
	for (int i = 0; i < (int) this->calib_imgPoints.size(); i++) { 
		bool isValid = true;
		// if no calib_objPoints for image i, break
		if (this->cal_types.size() <= i)
			this->cal_types.resize(this->calib_imgPoints.size(), 0);
		// if calibration type is not for typical calibration (1, 2, 3, 11), skip it.
		if (this->cal_types[i] != 1 && this->cal_types[i] != 2 &&
			this->cal_types[i] != 3 && this->cal_types[i] != 11)
			continue;
		if (this->calib_objPoints.size() <= i || this->calib_imgPoints.size() <= i) {
			isValid = false; 
			this->cal_types[i] = 0; 
			continue; 
		}
		// if numbers of points are not correct, skip it.
		if (this->calib_imgPoints[i].size() <= 0 ||
			this->calib_objPoints[i].size() <= 0 || 
			this->calib_imgPoints[i].size() != this->calib_objPoints[i].size()) {
			isValid = false;
			this->cal_types[i] = 0;
			continue;
		}
		// check each point in this calibration image 
		for (int j = 0; j < this->calib_imgPoints[i].size(); j++) {
			if (isnan(this->calib_imgPoints[i][j].x) ||
				isnan(this->calib_imgPoints[i][j].y) ||
				isnan(this->calib_objPoints[i][j].x) ||
				isnan(this->calib_objPoints[i][j].y) ||
				isnan(this->calib_objPoints[i][j].z) ||
				this->calib_imgPoints[i][j].x < 0 ||
				this->calib_imgPoints[i][j].y < 0 ) {
//				this->calib_imgPoints[i][j].x > this->imgSize.width ||
//				this->calib_imgPoints[i][j].y > this->imgSize.height) {
				isValid = false;
				this->cal_types[i] = 0;
				continue;
			} // end of if invalid
		} // end of for-each point of this calibration image
		if (isValid == true) {
			if (imgPointsValid.size() <= num_valid_calib_img)
				imgPointsValid.resize(num_valid_calib_img + 1);
			imgPointsValid[num_valid_calib_img] = this->calib_imgPoints[i];
			if (objPointsValid.size() <= num_valid_calib_img)
				objPointsValid.resize(num_valid_calib_img + 1);
			objPointsValid[num_valid_calib_img] = this->calib_objPoints[i];
			if (n2o.size() <= num_valid_calib_img)
				n2o.resize(num_valid_calib_img + 1);
			n2o[num_valid_calib_img] = i;
			o2n[i] = num_valid_calib_img;
			num_valid_calib_img++;
		}
	}

//	// print valid photos names
//	cout << "Photos for calibration:\n";
//	for (int i = 0; i < num_valid_calib_img; i++) {
//		cout << this->imsq.filename(n2o[i]) << endl;
//	}

	// calibrate the camera and update cmat and dvec
	cv::Mat rvecs_dummy, tvecs_dummy;
	//   check image size
	if (this->imgSize.width <= 0 || this->imgSize.height <= 0)
		this->setImageSize(); 
	// flag
	if (flagType == 0)
		flag = this->calib_flag;
	//   set initial guess and calib_flag according to number of points
	if (flagType == 1) { // flag is determined according to number of points
		flag = 0;
		int nValidPoint = 0;
		for (int iPhoto = 0; iPhoto < imgPointsValid.size(); iPhoto++)
			nValidPoint += (int)(imgPointsValid[iPhoto].size());
		if (nValidPoint < 20) { // 20 is determined roughly. Should be adjusted in the future. 
			//		this->calib_flag |= cv::CALIB_FIX_ASPECT_RATIO;  // fx = fy
			flag |= cv::CALIB_FIX_PRINCIPAL_POINT; // fix cx, cy (to image center)
			flag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
			flag |= cv::CALIB_FIX_K2; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
			flag |= cv::CALIB_USE_INTRINSIC_GUESS;
			this->cmat = cv::Mat::eye(3, 3, CV_64F);
			this->dvec = cv::Mat::zeros(1, 8, CV_64F);
			cmat.at<double>(0, 0) = this->imgSize.width * 0.5 / tan((39.6 / 2) / 180 * 3.1416); // Focal-Len 50-mm -> View angle W:39.6/H:27.0 deg.
			cmat.at<double>(1, 1) = cmat.at<double>(0, 0);
			cmat.at<double>(0, 2) = (this->imgSize.width - 1) / 2.;
			cmat.at<double>(1, 2) = (this->imgSize.height - 1) / 2.;
		}
		else if (nValidPoint < 40) { // 40 is determined roughly. Should be adjusted in the future
			flag |= cv::CALIB_FIX_K2; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
			flag |= cv::CALIB_USE_INTRINSIC_GUESS;
			this->cmat = cv::Mat::eye(3, 3, CV_64F);
			this->dvec = cv::Mat::zeros(1, 8, CV_64F);
			cmat.at<double>(0, 0) = this->imgSize.width * 0.5 / tan((39.6 / 2) / 180 * 3.1416); // Focal-Len 50-mm -> View angle W:39.6/H:27.0 deg.
			cmat.at<double>(1, 1) = cmat.at<double>(0, 0);
			cmat.at<double>(0, 2) = (this->imgSize.width - 1) / 2.;
			cmat.at<double>(1, 2) = (this->imgSize.height - 1) / 2.;
		}
		else {
			flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
			flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
			flag |= cv::CALIB_USE_INTRINSIC_GUESS;
			this->cmat = cv::Mat::eye(3, 3, CV_64F);
			this->dvec = cv::Mat::zeros(1, 8, CV_64F);
			cmat.at<double>(0, 0) = this->imgSize.width * 0.5 / tan((39.6 / 2) / 180 * 3.1416); // Focal-Len 50-mm -> View angle W:39.6/H:27.0 deg.
			cmat.at<double>(1, 1) = cmat.at<double>(0, 0);
			cmat.at<double>(0, 2) = (this->imgSize.width - 1) / 2.;
			cmat.at<double>(1, 2) = (this->imgSize.height - 1) / 2.;
		}
	} // handling flagType == 1 
	if (flagType == 2) {
		this->cmat = cv::Mat::eye(3, 3, CV_64F);
		this->dvec = cv::Mat::zeros(1, 8, CV_64F);
		cmat.at<double>(0, 0) = this->imgSize.width * 0.5 / tan((39.6 / 2) / 180 * 3.1416); // Focal-Len 50-mm -> View angle W:39.6/H:27.0 deg.
		cmat.at<double>(1, 1) = cmat.at<double>(0, 0);
		cmat.at<double>(0, 2) = (this->imgSize.width - 1) / 2.;
		cmat.at<double>(1, 2) = (this->imgSize.height - 1) / 2.;
//		flag = flag; // doing nothing, will be optimized away by compiler. 
	}	

	double _calib_rms = cv::calibrateCamera(objPointsValid,
		imgPointsValid, this->imgSize, this->cmat, this->dvec, 
		rvecs_dummy, tvecs_dummy,
		flag);
	// save rvecs and tvecs
	for (int i = 0; i < num_valid_calib_img; i++) {
		int oi = n2o[i];
		// check size of rvecs and tvecs
		if (this->rvecs.size() <= oi) rvecs.resize(oi + 1);
		if (this->tvecs.size() <= oi) tvecs.resize(oi + 1);
		this->rvecs[oi] = rvecs_dummy.at<Vec3d>(i, 0);
		this->tvecs[oi] = tvecs_dummy.at<Vec3d>(i, 0);
	}

//	double _calib_rms = cv::calibrateCamera(this->calib_objPoints,
//		this->calib_imgPoints, this->imgSize, this->cmat, this->dvec, rvecs_dummy, tvecs_dummy,
//		this->calib_flag);

	// set root-mean-square (rms) error
	this->calib_rms = _calib_rms;

	// calculate rms of the added (last) photo
	for (int i = 0; i < num_valid_calib_img; i++) {
		// reprojection
		vector<Point2f> projectedImgPoints = this->calib_imgPoints[n2o[i]];
		cv::projectPoints(this->calib_objPoints[n2o[i]],
			rvecs_dummy.at<Vec3d>(i, 0), tvecs_dummy.at<Vec3d>(i, 0),
			this->cmat, this->dvec, projectedImgPoints);
		// calculate rms
		if (this->calib_valid_rms.size() < n2o[i] + 1)
			this->calib_valid_rms.resize(n2o[i] + 1);
		this->calib_valid_rms[n2o[i]] = 0.0;
		for (int j = 0; j < projectedImgPoints.size(); j++)
		{
			double dx = projectedImgPoints[j].x - this->calib_imgPoints[n2o[i]][j].x;
			double dy = projectedImgPoints[j].y - this->calib_imgPoints[n2o[i]][j].y;
			this->calib_valid_rms[n2o[i]] += dx * dx + dy * dy;
			if (this->projection_errs.size() <= n2o[i])
				this->projection_errs.resize(n2o[i] + 1); 
			if (this->projection_errs[n2o[i]].size() <= j)
				this->projection_errs[n2o[i]].resize(j + 1); 
			this->projection_errs[n2o[i]][j] = sqrt(dx * dx + dy * dy); 
		}
		this->calib_valid_rms[n2o[i]] = 
			sqrt(this->calib_valid_rms[n2o[i]] / projectedImgPoints.size());
	}

	// set the camera rvec and tvec
	// They are defined by the first calibrated photo (but will be recalculated 
	// if new calibration photo is added) 
//	cv::Mat r4 = cv::Mat::zeros(4, 4, CV_64F);
//	this->rvec = cv::Mat::zeros(1, 3, CV_64F);
//	rvec.at<double>(0, 0) = rvecs_dummy.at<Point3d>(0, 0).x;
//	rvec.at<double>(0, 1) = rvecs_dummy.at<Point3d>(0, 0).y;
//	rvec.at<double>(0, 2) = rvecs_dummy.at<Point3d>(0, 0).z;
//	cv::Rodrigues(rvec, rmat);
//	rmat.copyTo(r4(cv::Rect(0, 0, 3, 3)));
//	this->tvec = cv::Mat::zeros(1, 3, CV_64F);
//	tvec.at<double>(0, 0) = tvecs_dummy.at<Point3d>(0, 0).x;
//	tvec.at<double>(0, 1) = tvecs_dummy.at<Point3d>(0, 0).y;
//	tvec.at<double>(0, 2) = tvecs_dummy.at<Point3d>(0, 0).z;
//	tvec = tvec.t();
//	tvec.copyTo(r4(cv::Rect(3, 0, 1, 3)));
//	r4.at<double>(3, 3) = 1.0;
//	//	cout << r4 << endl;
//	r4 = r4.inv();
//	r4(cv::Rect(0, 0, 3, 3)).copyTo(rmat);
//	tvec = tvec.t();
//	r4(cv::Rect(3, 0, 1, 3)).copyTo(tvec);
//	Rodrigues(this->rmat, this->rvec);

//	this->need_calibrate = false; 
	this->projection_points_vecvec(); 
	return 0;
}

int IntrinsicCalibrator::calibrateByLevel(int level)
{
	int theFlag = 0; 
	if (level == 1) {
		theFlag |= cv::CALIB_FIX_ASPECT_RATIO;  // fx = fy
		theFlag |= cv::CALIB_FIX_PRINCIPAL_POINT; // fix cx, cy (to image center)
		theFlag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
		theFlag |= cv::CALIB_FIX_K2; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K3; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K4; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K5; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K6; // fix kx (to zero)
		theFlag |= cv::CALIB_USE_INTRINSIC_GUESS;
		this->calibrate(2, theFlag);
	}
	else if (level == 2) {
		theFlag |= cv::CALIB_FIX_PRINCIPAL_POINT; // fix cx, cy (to image center)
		theFlag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
		theFlag |= cv::CALIB_FIX_K2; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K3; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K4; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K5; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K6; // fix kx (to zero)
		theFlag |= cv::CALIB_USE_INTRINSIC_GUESS;
		this->calibrate(2, theFlag);
	}
	else if (level == 3) {
		theFlag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
		theFlag |= cv::CALIB_FIX_K2; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K3; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K4; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K5; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K6; // fix kx (to zero)
		theFlag |= cv::CALIB_USE_INTRINSIC_GUESS;
		this->calibrate(2, theFlag);
	}
	else if (level == 4) {
		theFlag |= cv::CALIB_FIX_K2; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K3; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K4; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K5; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K6; // fix kx (to zero)
		theFlag |= cv::CALIB_USE_INTRINSIC_GUESS;
		this->calibrate(2, theFlag);
	}
	else if (level == 5) {
		theFlag |= cv::CALIB_FIX_K3; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K4; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K5; // fix kx (to zero)
		theFlag |= cv::CALIB_FIX_K6; // fix kx (to zero)
		theFlag |= cv::CALIB_USE_INTRINSIC_GUESS;
		this->calibrate(2, theFlag);
	}
	else {
		this->setFlagByAsking();
		this->calibrate();
	}
	this->projection_points_vecvec();
	return 0;
}

int IntrinsicCalibrator::numValidPhotos() const
{
	return this->n_calib_imgs;
}

string IntrinsicCalibrator::validCalibrationFileName(int index) const
{
	if (index >= 0 && index < this->calib_valid_fnames.size())
		return this->calib_valid_fnames[index]; 
	return string();
}

double IntrinsicCalibrator::validCalibrationRms(int index) const
{
	if (index >= this->calib_valid_rms.size())
		return nan("");
	else
		return this->calib_valid_rms[index];
}

cv::Mat IntrinsicCalibrator::cameraMatrix()
{
	return this->cmat;
}

cv::Mat IntrinsicCalibrator::distortionVector()
{
	return this->dvec; 
}

vector<Vec3d> IntrinsicCalibrator::rotationVectors()
{
	return this->rvecs;
}

vector<Vec3d> IntrinsicCalibrator::translationVectors()
{
	return this->tvecs; 
}

vector<vector<cv::Point2f>> IntrinsicCalibrator::projection_points_vecvec()
{
	// resize calib_prjPoints
	this->calib_prjPoints.resize(this->calib_imgPoints.size());
	// calculate project points image by image, point by point
	for (int i = 0; i < this->calib_imgPoints.size(); i++) {
		// resize points 
		cv::Mat rvec(1, 3, CV_64FC1), tvec(1, 3, CV_64FC1);
		rvec.at<double>(0, 0) = this->rvecs[i][0];
		rvec.at<double>(0, 1) = this->rvecs[i][1];
		rvec.at<double>(0, 2) = this->rvecs[i][2];
		tvec.at<double>(0, 0) = this->tvecs[i][0];
		tvec.at<double>(0, 1) = this->tvecs[i][1];
		tvec.at<double>(0, 2) = this->tvecs[i][2];
		cv::Mat pjp;
		cv::projectPoints(this->calib_objPoints[i],
			rvec, tvec, this->cmat, this->dvec, pjp); 
		this->calib_prjPoints[i].resize(pjp.rows);
		for (int iPoint = 0; iPoint < pjp.rows; iPoint++)
			this->calib_prjPoints[i][iPoint] = pjp.at<cv::Point2f>(iPoint, 0);
	}
	return this->calib_prjPoints;
}

vector<vector<double>> IntrinsicCalibrator::projection_errors_vecvec()
{
	// resize projection_errs
	this->projection_errs.resize(this->calib_imgPoints.size()); 
	// calculate error image by image, point by point
	for (int i = 0; i < this->calib_imgPoints.size(); i++) {
		// resize points 
		this->projection_errs[i].resize(this->calib_imgPoints[i].size());
		cv::Mat rvec(1, 3, CV_64FC1), tvec(1, 3, CV_64FC1);
		rvec.at<double>(0, 0) = this->rvecs[i][0];
		rvec.at<double>(0, 1) = this->rvecs[i][1];
		rvec.at<double>(0, 2) = this->rvecs[i][2];
		tvec.at<double>(0, 0) = this->tvecs[i][0];
		tvec.at<double>(0, 1) = this->tvecs[i][1];
		tvec.at<double>(0, 2) = this->tvecs[i][2];
		// if it is point-based projection
		if (this->cal_types[i] == 1 || this->cal_types[i] == 2 ||
			this->cal_types[i] == 3 || this->cal_types[i] == 11) {
			this->projection_errs[i] = proj_err_points(
				this->calib_imgPoints[i], this->calib_objPoints[i],
				this->cmat, this->dvec, rvec, tvec); 
		} // end of if it is point based projection
		// if it is line-based projection
		if (this->cal_types[i] == 12) {
			this->projection_errs[i] = proj_err_lines(
				this->calib_imgPoints[i],
				this->cmat, this->dvec);
		} // end of if it is line based projection
	}
	return this->projection_errs;
}

vector<double> IntrinsicCalibrator::projection_errors_vec()
{
	vector<vector<double> > errvv = this->projection_errors_vecvec(); 
	vector<double> errv(errvv.size(), 0.0); 
	for (int i = 0; i < errvv.size(); i++) {
		int count = 0; 
		for (int j = 0; j < errvv[i].size(); j++) {
			if (isnan(errvv[i][j]) == false) {
				errv[i] += errvv[i][j] * errvv[i][j];
				count++;
			}
		}
		if (count > 0) {
			errv[i] /= count;
			errv[i] = sqrt(errv[i]);
		} 
	}
	return errv;
}

double IntrinsicCalibrator::projection_error()
{
	vector<vector<double> > errvv = this->projection_errors_vecvec();
	double err = 0; 
	int count = 0;
	for (int i = 0; i < errvv.size(); i++) {
		for (int j = 0; j < errvv[i].size(); j++) {
			if (isnan(errvv[i][j]) == false) {
				err += errvv[i][j] * errvv[i][j];
				count++;
			}
		}
	}
	if (count > 0) {
		err /= count;
		err = sqrt(err);
	}
	return err; 
}

int IntrinsicCalibrator::writeIntrinsicParamteresToFile(string filename) const
{
	ostream * osp; 
	ofstream of;
	if (filename.compare("cout") == 0) {
		osp = &(std::cout);
	}
	else {
		string fullfile;
		if (this->imsq.directory().length() > 1)
			fullfile = this->imsq.directory() + filename;
		else
			fullfile = filename;
		of.open(fullfile);
		if (of.is_open() == false)
			return -1;
		osp = &of; 
	}
	ostream & os = *osp; 
		
//	if (this->cmat.cols == 3 && this->cmat.rows == 3) {
		os << setw(24) << setprecision(15) << scientific << this->cmat.at<double>(0, 0) << endl;
		os << setw(24) << setprecision(15) << scientific << this->cmat.at<double>(1, 1) << endl;
		os << setw(24) << setprecision(15) << scientific << this->cmat.at<double>(0, 2) << endl;
		os << setw(24) << setprecision(15) << scientific << this->cmat.at<double>(1, 2) << endl;
//	}
//	if (this->dvec.cols > 1 && this->dvec.rows == 1) {
		for (int i = 0; i < 12; i++) {
			double value;
			if (i < this->dvec.cols)
				value = this->dvec.at<double>(0, i);
			else
				break;
			os << setw(24) << setprecision(15) << scientific << value << endl;
		}
//	}
	if(of.is_open()) 
		of.close();
	return 0;
}


int IntrinsicCalibrator::readIntrinsicParamteresToFile(string fullpathfile)
{
	ifstream ifile(fullpathfile);
	if (ifile.is_open() == false)
		return -1;
	double fx, fy, cx, cy;
	vector<double> k(4, 0.0); 
	ifile >> fx >> fy >> cx >> cy;
	for (int i = 0; i < 12; i++) {
		double val = 0.0; 
		ifile >> val;
		if (ifile.eof() && val == 0.0)
			break;
		if (i >= k.size())
			k.resize(i + 1);
		k[i] = val;
	}
	ifile.close();
	this->cmat = cv::Mat::eye(3, 3, CV_64F);
	this->cmat.at<double>(0, 0) = fx;
	this->cmat.at<double>(1, 1) = fy;
	this->cmat.at<double>(0, 2) = cx;
	this->cmat.at<double>(1, 2) = cy; 
	this->dvec = cv::Mat::zeros(1, (int) k.size(), CV_64F);
	for (int i = 0; i < k.size(); i++)
		this->dvec.at<double>(0, i) = k[i];
	return 0;
}

int IntrinsicCalibrator::writeToFsFile(string filename) const
{
	std::string fullFilename(this->imsq.directory() + filename);
	// if given filename has no directory, use file sequence directory.
	if (directoryOfFullPathFile(filename).length() <= 0)
		fullFilename = this->imsq.directory() + filename;
	else
		fullFilename = filename;
	cout << fullFilename << endl;
	FileStorage ofs(fullFilename, FileStorage::Mode::WRITE);
	// File sequence path of images
	string fileSequencePath; 
	if (imsq.directory().length() > 1)
		fileSequencePath = imsq.directory();
	else
		fileSequencePath = "C:\\folder_of_calibration_photos\\";
	ofs << "FileSequencePath" << fileSequencePath;

	// Number of squares of this board
//	ofs << "NumberOfCornersAlongWidth" << this->num_corners_along_width;
//	ofs << "NumberOfCornersAlongHeight" << this->num_corners_along_height;

	// Size of squares
//	ofs << "SquareWidth" << this->square_width;
//	ofs << "SquareHeight" << this->square_height;

	// Image size of this camera
	if (this->imgSize.width > 0 && this->imgSize.height > 0)
		ofs << "imageSize" << this->imgSize; 

	// Number of valid (corners found) photos, if not zero
	if (this->numValidPhotos() > 0) {
		ofs << "numValidPhotos" << this->numValidPhotos();
		ofs << "validCalibrationFileName" << this->calib_valid_fnames;
		ofs << "validCalibrationRms" << this->calib_valid_rms;
	}
	
	// Calibration flags
	ofs << "CALIB_FIX_ASPECT_RATIO"    << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_ASPECT_RATIO));
	ofs << "CALIB_FIX_FOCAL_LENGTH"    << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_FOCAL_LENGTH));
	ofs << "CALIB_FIX_K1"              << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_K1));
	ofs << "CALIB_FIX_K2"              << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_K2));
	ofs << "CALIB_FIX_K3"              << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_K3));
	ofs << "CALIB_FIX_K4"              << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_K4));
	ofs << "CALIB_FIX_K5"              << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_K5));
	ofs << "CALIB_FIX_K6"              << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_K6));
	ofs << "CALIB_FIX_PRINCIPAL_POINT" << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_PRINCIPAL_POINT));
	ofs << "CALIB_RATIONAL_MODEL"      << (bool)(0 != (this->calib_flag & cv::CALIB_RATIONAL_MODEL));
	ofs << "CALIB_FIX_TANGENT_DIST"    << (bool)(0 != (this->calib_flag & cv::CALIB_FIX_TANGENT_DIST));

	// Need-calibration flag
//	ofs << "needCalibration" << this->need_calibrate;

	// Camera matrix
	if (this->cmat.at<double>(2, 2) >= 1.0) {
		// matrix form
		ofs << "cameraMatrix" << this->cmat; 
		// variable form
//		ofs << "fx" << this->cmat.at<double>(0, 0);
//		ofs << "fy" << this->cmat.at<double>(1, 1);
//		ofs << "cx" << this->cmat.at<double>(0, 2);
//		ofs << "cy" << this->cmat.at<double>(1, 2);
	}
	// Distortion vector
	if (cv::norm(this->dvec) > 1e-9) {
		// vector form (supposed to be 1 x N)
		ofs << "distortionVector" << this->dvec;
//		// variable form 
//		if (dvec.cols * dvec.rows >=  1) ofs << "k1" << this->dvec.ptr<double>(0);
//		if (dvec.cols * dvec.rows >=  2) ofs << "k2" << this->dvec.ptr<double>(1);
//		if (dvec.cols * dvec.rows >=  3) ofs << "p1" << this->dvec.ptr<double>(2);
//		if (dvec.cols * dvec.rows >=  4) ofs << "p2" << this->dvec.ptr<double>(3);
//		if (dvec.cols * dvec.rows >=  5) ofs << "k3" << this->dvec.ptr<double>(4);
//		if (dvec.cols * dvec.rows >=  6) ofs << "k4" << this->dvec.ptr<double>(5);
//		if (dvec.cols * dvec.rows >=  7) ofs << "k5" << this->dvec.ptr<double>(6);
//		if (dvec.cols * dvec.rows >=  8) ofs << "k6" << this->dvec.ptr<double>(7);
//		if (dvec.cols * dvec.rows >=  9) ofs << "k7" << this->dvec.ptr<double>(8);
//		if (dvec.cols * dvec.rows >= 10) ofs << "k8" << this->dvec.ptr<double>(9);
//		if (dvec.cols * dvec.rows >= 11) ofs << "k9" << this->dvec.ptr<double>(10);
//		if (dvec.cols * dvec.rows >= 12) ofs << "k10" << this->dvec.ptr<double>(11);
	}


	// R matrix 
//	if (cv::norm(this->rmat) > 1e-9) {
//		ofs << "R-matrix" << this->rmat;
//	}
	// R vec (Rodrigues of R matrix) 
//	if (cv::norm(this->rvec) > 1e-9) {
//		ofs << "R-vector" << this->rvec;
//	}
	// T vec 
//	if (cv::norm(this->tvec) > 1e-9) {
//		ofs << "T-vector" << this->tvec;
//	}

	// Image points
	if (this->calib_imgPoints.size() > 0) {
		ofs << "ImagePoints" << this->calib_imgPoints;
	}

	// Object points
	if (this->calib_objPoints.size() > 0) {
		ofs << "ObjectPoints" << this->calib_objPoints;
	}

	// Projected points
	if (this->calib_prjPoints.size() > 0) {
		ofs << "ProjectedPoints" << this->calib_prjPoints;
	}

	// extrinsic of rvec/tvec of last (probably user defined) photo
	if (this->rvecs.size() >= 1 && this->tvecs.size() >= 1)
	{
		ofs << "rvec" << cv::Mat(this->rvecs[this->rvecs.size() - 1]);
		ofs << "tvec" << cv::Mat(this->tvecs[this->tvecs.size() - 1]); 
		cv::Mat r33(3, 3, CV_64F);
		cv::Mat r44 = cv::Mat::eye(4, 4, CV_64F);
		cv::Rodrigues(this->rvecs[this->rvecs.size() - 1], r33); 
		r33.copyTo(r44(cv::Rect(0, 0, 3, 3)));
		r44.at<double>(0, 3) = this->tvecs[this->tvecs.size() - 1][0];
		r44.at<double>(1, 3) = this->tvecs[this->tvecs.size() - 1][1];
		r44.at<double>(2, 3) = this->tvecs[this->tvecs.size() - 1][2];
		ofs << "R4" << r44;
		cv::Mat r44inv = r44.inv(); 
		ofs << "R4inv" << r44inv;
		ofs << "CamPosition" << r44inv(cv::Rect(3, 0, 1, 3));
	}

	ofs.release();
	return 0;
}

int IntrinsicCalibrator::readFromFsFile(string fullPathSetting)
{
	FileStorage ifs(fullPathSetting, FileStorage::Mode::READ);
	if (ifs.isOpened() == false)
		return -1;

	// Number of squares of this board
//	ifs["NumberOfCornersAlongWidth" ] >> this->num_corners_along_width;
//	ifs["NumberOfCornersAlongHeight"] >> this->num_corners_along_height;

	// Size of squares
//	ifs["SquareWidth" ] >> this->square_width;
//	ifs["SquareHeight"] >> this->square_height;

	// Image size
	ifs["imageSize"] >> this->imgSize;
    cout << "Image size from file is " << this->imgSize << endl;

	// File sequence path of images
	string fileSequencePath;
    // I marked the 3 lines here because they crash in Raspberry.
//	ifs["FileSequencePath"] >> fileSequencePath;
//	imsq.setFilesByExt(".JPG");
// 	this->setFileSeq(imsq);

	// Number of valid (corners found) photos, if not zero
//	ifs["numValidPhotos"] >> this->n_calib_imgs;

	// calibration flags
	bool f; 
	ifs["CALIB_FIX_ASPECT_RATIO"   ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_ASPECT_RATIO   ; else this->calib_flag &= ~cv::CALIB_FIX_ASPECT_RATIO;
	ifs["CALIB_FIX_FOCAL_LENGTH"   ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_FOCAL_LENGTH   ; else this->calib_flag &= ~cv::CALIB_FIX_FOCAL_LENGTH;
	ifs["CALIB_FIX_K1"             ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_K1             ; else this->calib_flag &= ~cv::CALIB_FIX_K1;
	ifs["CALIB_FIX_K2"             ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_K2             ; else this->calib_flag &= ~cv::CALIB_FIX_K2;
	ifs["CALIB_FIX_K3"             ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_K3             ; else this->calib_flag &= ~cv::CALIB_FIX_K3;
	ifs["CALIB_FIX_K4"             ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_K4             ; else this->calib_flag &= ~cv::CALIB_FIX_K4;
	ifs["CALIB_FIX_K5"             ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_K5             ; else this->calib_flag &= ~cv::CALIB_FIX_K5;
	ifs["CALIB_FIX_K6"             ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_K6             ; else this->calib_flag &= ~cv::CALIB_FIX_K6;
	ifs["CALIB_FIX_PRINCIPAL_POINT"] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_PRINCIPAL_POINT; else this->calib_flag &= ~cv::CALIB_FIX_PRINCIPAL_POINT;
	ifs["CALIB_RATIONAL_MODEL"     ] >> f; if (f) this->calib_flag |= cv::CALIB_RATIONAL_MODEL     ; else this->calib_flag &= ~cv::CALIB_RATIONAL_MODEL;
	ifs["CALIB_FIX_TANGENT_DIST"   ] >> f; if (f) this->calib_flag |= cv::CALIB_FIX_TANGENT_DIST   ; else this->calib_flag &= ~cv::CALIB_FIX_TANGENT_DIST;

    cout << "Calibration flag from file is " << this->calib_flag << endl;

	// Need-calibration flag
//	ifs["needCalibration"] >> this->need_calibrate;

	// dummy
	cv::Mat dummy;

	// Camera matrix
	ifs["cameraMatrix"] >> dummy;
	if (dummy.empty() == false) this->cmat = dummy;
    cout << "Camera matrix from file is " << this->cmat << endl;

	// Distortion vector
	ifs["distortionVector"] >> dummy;
	if (dummy.empty() == false) this->dvec = dummy;
    cout << "Distortion vector from file is " << this->dvec << endl;

	// R matrix 
//	ifs["R-matrix"] >> this->rmat;

	// R vector (Rodrigues of R matrix)
//	ifs["R-vector"] >> dummy;
//	if (dummy.empty() == false) this->rvec = dummy;

	// T vector
//	ifs["T-vector"] >> dummy;
//	if (dummy.empty() == false) this->tvec = dummy;

	// Image points
	vector<vector<Point2f> > imgPoints_dummy;
	ifs["ImagePoints"] >> imgPoints_dummy;
	if (imgPoints_dummy.size() > 0)
		this->calib_imgPoints = imgPoints_dummy;
    if (imgPoints_dummy.size() > 0)
        cout << "# Number of calibration points is " << this->calib_imgPoints.size() << endl;

	// Object points
	vector<vector<Point3f> > objPoints_dummy;
	ifs["ObjectPoints"] >> objPoints_dummy;
	if (objPoints_dummy.size() > 0)
		this->calib_objPoints = objPoints_dummy;

	return 0;
}

cv::Mat IntrinsicCalibrator::R4(int i) const
{
	cv::Mat r4 = cv::Mat::eye(4, 4, CV_64F); 
	if (i + 1 >= this->rvecs.size()) {
		cv::Mat r3 = r4(cv::Rect(0, 0, 3, 3));
		cv::Rodrigues(this->rvecs[i], r3);
		r4.at<double>(0, 3) = this->tvecs[i](0);
		r4.at<double>(1, 3) = this->tvecs[i](1);
		r4.at<double>(2, 3) = this->tvecs[i](2);
	}
	return r4;
}

FileSeq IntrinsicCalibrator::getFileSeq() const
{
	return this->imsq;
}

int IntrinsicCalibrator::log(std::string msg) const
{
	if (this->imsq.directory().length() <= 1) return -1;
	ofstream logFile(this->imsq.directory() + this->logFilename, std::ios_base::app);
	if (logFile.is_open()) {
		std::time_t t = std::time(0);
		std::tm* now = std::localtime(&t);
		char logTime[100];
		snprintf(logTime, 100, " (%4d-%02d-%02d %02d:%02d:%02d)",
			now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
			now->tm_hour, now->tm_min, now->tm_sec);
		logFile << msg << logTime << endl;
		logFile.close();
	}
	return 0;
}

int IntrinsicCalibrator::writeToMscript(std::string filename) const
{
	std::string fullFilename;
	// if given filename has no directory, use file sequence directory.
	if (directoryOfFullPathFile(filename).length() <= 0)
		fullFilename = this->imsq.directory() + filename;
	else
		fullFilename = filename;
	cout << fullFilename << endl;
	// open file to write 
	ofstream ofs(fullFilename); 
	if (ofs.is_open() == false) {
		cerr << "Cannot open " << fullFilename << endl;
		return -1;
	}
	// data 
	ofs << "clear all; close all;\n";
	ofs << "imgx = zeros( " << calib_prjPoints.size() << ", " << calib_prjPoints[0].size() << ");\n";
	ofs << "prjx = zeros( " << calib_prjPoints.size() << ", " << calib_prjPoints[0].size() << ");\n";
	ofs << "imgy = zeros( " << calib_prjPoints.size() << ", " << calib_prjPoints[0].size() << ");\n";
	ofs << "prjy = zeros( " << calib_prjPoints.size() << ", " << calib_prjPoints[0].size() << ");\n";
	ofs << "nStep = " << calib_prjPoints.size() << ";\n";
	ofs << "nPoint = " << calib_prjPoints[0].size() << ";\n"; 
	for (int iStep = 0; iStep < calib_prjPoints.size(); iStep++)
	{
		for (int iPoint = 0; iPoint < calib_prjPoints[iStep].size(); iPoint++)
		{
			ofs << "imgx(" << iStep + 1 << ", " << iPoint + 1 << ") = "
				<< this->calib_imgPoints[iStep][iPoint].x << "; ";
			ofs << "prjx(" << iStep + 1 << ", " << iPoint + 1 << ") = "
				<< this->calib_prjPoints[iStep][iPoint].x << "; ";
			ofs << "imgy(" << iStep + 1 << ", " << iPoint + 1 << ") = "
				<< this->calib_imgPoints[iStep][iPoint].y << "; ";
			ofs << "prjy(" << iStep + 1 << ", " << iPoint + 1 << ") = "
				<< this->calib_prjPoints[iStep][iPoint].y << "; ";
			ofs << endl;
		}
	}
	ofs << "errx = prjx - imgx;" << endl;
	ofs << "erry = prjy - imgy;" << endl;
	// plot parameters
	ofs << "nGridRefinement = " << 40 << ";" << endl;
//	ofs << "nContourLines = " << 40 << ";" << endl;
	ofs << "contourLines = [-10 -2 -1.5 -1 -0.5 0 .5 1 1.5 2 10];" << endl;
	// Plot image projected / actual points all calibration photos in one
	ofs << "hfig_all = figure('name', 'all points');\n";
	ofs << "for i = 1: nStep" << endl;
	ofs << "  plot(imgx, imgy, 'o'); hold on;" << endl;
	ofs << "  plot(prjx, prjy, '+'); hold on;" << endl;
	ofs << "end" << endl;
	ofs << "set(gca,'Ydir','reverse');" << endl;
	ofs << "legend('Image', 'Projected'); grid on;" << endl;
	// Plot image projected / actual points all calibration photos one by one
	ofs << "for i = 1: nStep" << endl;
	ofs << "  hfigi{i} = figure('name', ['photo ' num2str(i)]); " << endl;
	ofs << "  plot(imgx(i,:), imgy(i,:), 'o'); hold on;" << endl;
	ofs << "  plot(prjx(i,:), prjy(i,:), '+'); hold on;" << endl;
	ofs << "end" << endl;
	ofs << "set(gca,'Ydir','reverse');" << endl;
	ofs << "legend('Image', 'Projected'); grid on;" << endl;
	// Plot error x of all calibration photos in one
	ofs << "hfig_all_errx = figure('name', 'Err x all points');\n";
	ofs << "interpolant = scatteredInterpolant(imgx(:), imgy(:), errx(:));\n";
	ofs << "interpolant.ExtrapolationMethod = 'none';\n";
	ofs << "minx = min(imgx(:)); maxx = max(imgx(:));\n";
	ofs << "miny = min(imgy(:)); maxy = max(imgy(:));\n";
	ofs << "[xx, yy] = meshgrid(linspace(minx, maxx, nGridRefinement), linspace(miny, maxy, nGridRefinement));\n";
	ofs << "intensity_interp = interpolant(xx, yy);\n";
//	ofs << "contourf(xx, yy, intensity_interp, nContourLines); colorbar; grid on; \n";
	ofs << "contourf(xx, yy, intensity_interp, contourLines); colorbar; grid on; \n";
	ofs << "hold on; plot(prjx(:), prjy(:), '+');" << endl;
	ofs << "set(gca,'Ydir','reverse');" << endl;
	// Plot error y of all calibration photos in one
	ofs << "hfig_all_erry = figure('name', 'Err y all points');\n";
	ofs << "interpolant = scatteredInterpolant(imgx(:), imgy(:), erry(:));\n";
	ofs << "interpolant.ExtrapolationMethod = 'none';\n";
	ofs << "minx = min(imgx(:)); maxx = max(imgx(:));\n";
	ofs << "miny = min(imgy(:)); maxy = max(imgy(:));\n";
	ofs << "[xx, yy] = meshgrid(linspace(minx, maxx, nGridRefinement), linspace(miny, maxy, nGridRefinement));\n";
	ofs << "intensity_interp = interpolant(xx, yy);\n";
//	ofs << "contourf(xx, yy, intensity_interp, nContourLines); colorbar; grid on; \n";
	ofs << "contourf(xx, yy, intensity_interp, contourLines); colorbar; grid on; \n";
	ofs << "hold on; plot(prjx(:), prjy(:), '+');" << endl;
	ofs << "set(gca,'Ydir','reverse');" << endl;
	// Plot error x and y of all calibration photos one by one 
	ofs << "for i = 1: nStep" << endl;
	ofs << "  hfigi_errx{i} = figure('name', ['errx of photo ' num2str(i)]); " << endl;
	ofs << "  interpolant = scatteredInterpolant(imgx(i,:)', imgy(i,:)', errx(i,:)');\n";
	ofs << "  interpolant.ExtrapolationMethod = 'none';\n";
	ofs << "  [xx, yy] = meshgrid(linspace(minx, maxx, nGridRefinement), linspace(miny, maxy, nGridRefinement));\n";
	ofs << "  intensity_interp = interpolant(xx, yy);\n";
//	ofs << "  contourf(xx, yy, intensity_interp, nContourLines); colorbar; grid on; \n";
	ofs << "  contourf(xx, yy, intensity_interp, contourLines); colorbar; grid on; \n";
	ofs << "  hold on; plot(prjx(i,:), prjy(i,:), '+');" << endl;
	ofs << "  set(gca,'Ydir','reverse');" << endl;
	ofs << "  hfigi_erry{i} = figure('name', ['erry of photo ' num2str(i)]); " << endl;
	ofs << "  interpolant = scatteredInterpolant(imgx(i,:)', imgy(i,:)', erry(i,:)');\n";
	ofs << "  interpolant.ExtrapolationMethod = 'none';\n";
	ofs << "  [xx, yy] = meshgrid(linspace(minx, maxx, nGridRefinement), linspace(miny, maxy, nGridRefinement));\n";
	ofs << "  intensity_interp = interpolant(xx, yy);\n";
//	ofs << "  contourf(xx, yy, intensity_interp, nContourLines); colorbar; grid on; \n";
	ofs << "  contourf(xx, yy, intensity_interp, contourLines); colorbar; grid on; \n";
	ofs << "  hold on; plot(prjx(i,:), prjy(i,:), '+');" << endl;
	ofs << "  set(gca,'Ydir','reverse');" << endl;
	ofs << "end" << endl;
	
	ofs.close();
	return 0;
}

//int findExtrinsic(int, char **);
//int imagePointsPicking(int, char**);
//int trackPointsEcc(int, char**);
//int baslerFileNameRename(int argc, char** argv); 


//int IntrinsicCalibrator::sample01()
//{
//	IntrinsicCalibrator & cal = *this; 
//
//	vector<Point2f> vp(5, Point2f(1, 1)), vp_new; 
//	// user menu loop
//	while (true) {
//		int selection; 
//		// main menu
//		std::cout << "\n\n\nMain menu ---------------------------\n"; 
//		std::cout << "(0) Exit\n";
//		std::cout << "(1) Set file-list file:\n";
//		std::cout << "(2) Select files from GUI dialog:\n";
//		std::cout << "(3) Find corners:\n";
//		std::cout << "(4) Define calibration points:\n"; 
//		std::cout << "(5) Define 3-point straigt lines:\n";
//		std::cout << "(6) Calibrate by img/obj points: \n"; 
//		std::cout << "(7) Write img/obj & calib files: \n";
//		std::cout << "(8) Independent extrinsic calibration: \n";
//		std::cout << "(9) Independent points picking: \n";
//		std::cout << "(10) Independent ECC points tracking: \n";
//		std::cout << "(11) Independent rename basler camera file names: \n";
//
//		// cin >> selection; 
//		while (true) {
//			string strbuf; 
//			std::cin >> strbuf;
//			if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//			try {
//				selection = std::stoi(strbuf);
//				break;
//			}
//			catch (...) {
//				continue; 
//			}
//		}
//
//		// Selection 1: Get file sequence of calibration photos
//		if (selection == 0) {
//			return -1; 
//		}
//		if (selection == 1) {
//			std::cout << "  Enter calibration file-list file ('g' for GUI files): ";
//			std::string calPhotoListFile;
//			while (true) {
//				std::cin >> calPhotoListFile;
//				if (calPhotoListFile.length() > 0 && calPhotoListFile[0] == '#') continue;
//				break;
//			}
//			if (calPhotoListFile.size() == 1 && calPhotoListFile[0] == 'g') {
//				std::vector<std::string> calFiles = uigetfiles();
//				cal.fileSeq().setFilesByStringVec(calFiles);
//				cal.fileSeq().generateFileList("cal_filelist.txt");
//			}
//			else {
//				cal.fileSeq().setFilesByListFile(calPhotoListFile);
//			}
//			std::cout << "  Working directory: " << cal.fileSeq().directory() << endl;
//			std::cout << cal.fileSeq().num_files() << " files, starting from " << cal.fileSeq().filename(0) << endl;
//			continue; 
//		}
//		// Selection 2: Get file sequence of calibration photos by GUI
//		if (selection == 2) {
//			std::vector<std::string> calFiles = uigetfiles();
//			cal.fileSeq().setFilesByStringVec(calFiles);
//			cal.fileSeq().generateFileList("cal_filelist.txt");
//			std::cout << "  Working directory: " << cal.fileSeq().directory() << endl;
//			std::cout << cal.fileSeq().num_files() << " files, starting from " << cal.fileSeq().filename(0) << endl;
//			continue;
//		}
//		// Selection 3: Find corners 
//		if (selection == 3) {
//			// board info
//			std::cout << "  Calibration board info: \n";
//			std::cout << "    type (1 : Chessb. 2 : Grid(sym). 3 : Grid(unsym)\n";
//			std::cout << "    numbers of points(corners) along width and height (7 7 for 8x8 standard board)\n";
//			std::cout << "    size of each square along width and height:\n";
//			int nw, nh, btype; float sqw, sqh;
//			//			std::cin >> btype >> nw >> nh >> sqw >> sqh;
//			while (true) {
//				string strbuf; 	cin >> strbuf; 
//				if (strbuf[0] == '#') continue;
//				try {
//					btype = std::stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue; 
//				}
//			}
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf[0] == '#') continue;
//				try {
//					nw = std::stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf[0] == '#') continue;
//				try {
//					nh = std::stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf[0] == '#') continue;
//				try {
//					sqw = std::stof(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf[0] == '#') continue;
//				try {
//					sqh = std::stof(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			// photo number and indices
//			int num_photos_to_find_corners;
//			std::vector<int> idxs; 
//			for (int i = 0; i < cal.fileSeq().num_files(); i++) {
//				std::cout << "   " << i + 1 << ": "
//					<< cal.fileSeq().filename(i) << endl;
//			}
//			std::cout << "  How many photos do you want to find corners? ";
////			std::cin >> num_photos_to_find_corners;
//			while (true) {
//				string strbuf;  cin >> strbuf;
//				if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//				try {
//					num_photos_to_find_corners = std::stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			// if user decides that all photos need to find corners
//			if (num_photos_to_find_corners == cal.fileSeq().num_files()) {
//				idxs.resize(num_photos_to_find_corners);
//				for (int i = 0; i < num_photos_to_find_corners; i++)
//					idxs[i] = i; 
//			}
//			else { // user specifies which photos need to find corners
//				std::cout << "  Input those indices of photos: ";
//				idxs.resize(num_photos_to_find_corners);
//				for (int i = 0; i < num_photos_to_find_corners; i++) {
////					std::cin >> idxs[i];
//					while (true) {
//						string strbuf; cin >> strbuf;
//						if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//						try {
//							idxs[i] = std::stoi(strbuf);
//							break;
//						}
//						catch (...) {
//							continue;
//						}
//					}
//					idxs[i]--; // convert 1-base to 0-base
//				}
//			}
//			// finding corners
//			for (int i = 0; i < num_photos_to_find_corners; i++) {
//				int cFound = cal.findCorners(idxs[i], cv::Size(nw, nh),
//					sqw, sqh, btype);
//				if (cFound == 0) {
//					std::cout << "Photo index " << idxs[i] + 1 << ": "
//						<< cal.fileSeq().filename(idxs[i])
//						<< ". Corners found.\n";
//				}
//				else {
//					std::cout << "Photo index " << idxs[i] + 1 << ": "
//						<< cal.fileSeq().filename(idxs[i])
//						<< ". Corners NOT found.\n";
//				}
//			}
//		}
//		// Selection 4: Define calibration points 
//		if (selection == 4) {
//			// photo number and indices
//			int num_photos_to_define_points;
//			std::vector<int> idxs;
//			for (int i = 0; i < cal.fileSeq().num_files(); i++) {
//				std::cout << "   " << i + 1 << ": "
//					<< cal.fileSeq().filename(i) << endl;
//			}
//			std::cout << "  How many photos do you want to find points? ";
////			std::cin >> num_photos_to_define_points;
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//				try {
//					num_photos_to_define_points = std::stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			std::cout << "  Input those indices of photos: ";
//			idxs.resize(num_photos_to_define_points);
//			for (int i = 0; i < num_photos_to_define_points; i++) {
////				std::cin >> idxs[i];
//				while (true) {
//					string strbuf; cin >> strbuf;
//					if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//					try {
//						idxs[i] = std::stoi(strbuf);
//						break;
//					}
//					catch (...) {
//						continue; 
//					}
//				}
//				idxs[i]--; // convert 1-base to 0-base
//			}
//			// define points
//			for (int i = 0; i < num_photos_to_define_points; i++) {
//				// ask # of points
//				int num_points;
//				std::cout << "    How many points to you want to pick on "
//					<< idxs[i] + 1 << ": "
//					<< cal.fileSeq().filename(idxs[i]) << " ? (-1 for selecting txt image points filez)\n";
////				std::cin >> num_points;
//				while (true) {
//					string strbuf; cin >> strbuf;
//					if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//					try {
//						num_points = stoi(strbuf);
//						break;
//					}
//					catch (...) {
//						continue;
//					}
//				}
//
//				// pick points
//				ImagePointsPicker pp;
//				if (num_points > 0) {
//					pp.setBackgroundImageByFilename(
//						cal.fileSeq().filename(idxs[i]));
//					pp.pickPoints(num_points);
//				}
//				else {
//					string fname = uigetfile();
//					if (fname.length() > 1)
//						pp.readPointsFromTxtFile(fname); 
//					num_points = pp.num_points();
//					cout << "User readPointsFromTxtFile check: num_points: " << num_points << endl;
//				}
//				// get points data 
//				if (cal.imgPoints().size() < idxs[i] + 1)
//					cal.imgPoints().resize(idxs[i] + 1); 
//				cal.imgPoints()[idxs[i]] = pp.pointsInPoint2fVector();
//				// set image size if necessary
//				if (this->imgSize.width <= 0 || this->imgSize.height <= 0)
//					this->imgSize = pp.imgSize(); 
//				// ask user to input object points
//				std::cout << "   Input 3D positions of the " << num_points << " points:\n";
//				if (cal.objPoints().size() < idxs[i] + 1)
//					cal.objPoints().resize(idxs[i] + 1);
//				cal.objPoints()[idxs[i]].resize(num_points);
//				for (int j = 0; j < num_points; j++) {
////					std::cin >> cal.objPoints()[idxs[i]][j].x; 
////					std::cin >> cal.objPoints()[idxs[i]][j].y;
////					std::cin >> cal.objPoints()[idxs[i]][j].z;
//					while (true) {
//						string strbuf; cin >> strbuf; 
//						if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//						try {
//							cal.objPoints()[idxs[i]][j].x = stof(strbuf);
//							break;
//						}
//						catch (...) {
//							continue;
//						}
//					}
//					while (true) {
//						string strbuf; cin >> strbuf;
//						if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//						try {
//							cal.objPoints()[idxs[i]][j].y = stof(strbuf);
//							break;
//						}
//						catch (...) {
//							continue;
//						}
//					}
//					while (true) {
//						string strbuf; cin >> strbuf;
//						if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//						try {
//							cal.objPoints()[idxs[i]][j].z = stof(strbuf);
//							break;
//						}
//						catch (...) {
//							continue;
//						}
//					}
//				}
//				// set calibration type to user-defined points (11)
//				if (cal.calTypes().size() < idxs[i] + 1)
//					cal.calTypes().resize(idxs[i] + 1, 0);
//				cal.calTypes()[idxs[i]] = 11;
//			} // end of for each photo 
//		} // end of if selection 4  
//		// Selection 5: Define straight lines 
//		if (selection == 5) {
//			// photo number and indices
//			int num_photos_to_define_lines;
//			std::vector<int> idxs;
//			for (int i = 0; i < cal.fileSeq().num_files(); i++) {
//				std::cout << "   " << i + 1 << ": "
//					<< cal.fileSeq().filename(i) << endl;
//			}
//			std::cout << "  How many photos do you want to define lines? ";
////			std::cin >> num_photos_to_define_lines;
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//				try {
//					num_photos_to_define_lines = stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue; 
//				}
//			}
//			std::cout << "  Input those indices of photos: ";
//			idxs.resize(num_photos_to_define_lines);
//			for (int i = 0; i < num_photos_to_define_lines; i++) {
////				std::cin >> idxs[i];
//				while (true) {
//					string strbuf; cin >> strbuf;
//					if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//					try {
//						idxs[i] = stoi(strbuf);
//						break;
//					}
//					catch (...) {
//						continue;
//					}
//				}
//				idxs[i]--; // convert 1-base to 0-base
//			}
//			// define points
//			for (int i = 0; i < num_photos_to_define_lines; i++) {
//				// ask # of points
//				int num_lines;
//				std::cout << "    How many lines to you want to define on "
//					<< idxs[i] + 1 << ": "
//					<< cal.fileSeq().filename(idxs[i]) << " ?\n";
////				std::cin >> num_lines;
//				while (true) {
//					string strbuf; cin >> strbuf;
//					if (strbuf.length() > 0 && strbuf[0] == '#') continue;
//					try {
//						num_lines = stoi(strbuf);
//						break;
//					}
//					catch (...) {
//						continue;
//					}
//				}
//				std::cout << "Each line is defined by picking 3 points.\n";
//				// pick points
//				ImagePointsPicker pp;
//				pp.setBackgroundImageByFilename(
//					cal.fileSeq().fullPathOfFile(idxs[i]));
//				pp.pickPoints(num_lines * 3);
//				// get points data 
//				if (cal.imgPoints().size() < idxs[i] + 1)
//					cal.imgPoints().resize(idxs[i] + 1);
//				cal.imgPoints()[idxs[i]] = pp.pointsInPoint2fVector();
//				// clear object points
//				if (cal.objPoints().size() < idxs[i] + 1)
//					cal.objPoints().resize(idxs[i] + 1);
//				cal.objPoints()[idxs[i]].clear();
//				// set calibration type to 3-point straight line (12)
//				if (cal.calTypes().size() < idxs[i] + 1)
//					cal.calTypes().resize(idxs[i] + 1, 0);
//				cal.calTypes()[idxs[i]] = 12;
//			} // end of for each photo 
//		} // end of if selection 5 
//		// Selection 6: Calibrate by img/obj points
//		if (selection == 6) {
//			// ask details about flag
//			cout << "  (1) Calibrate fx (=fy) and k1.\n";
//			cout << "  (2) Calibrate fx, fy, and k1.\n";
//			cout << "  (3) Calibrate fx, fy, cx, cy, and k1.\n";
//			cout << "  (4) Calibrate fx, fy, cx, cy, k1, p1, and p2.\n";
//			cout << "  (5) Calibrate fx, fy, cx, cy, k1, k2, p1, and p2.\n";
//			cout << "  (6) Otherwise, set flags one by one. \n";
//			int calib_level;
//			int flag; 
//			while (true) {
//				string strbuf; cin >> strbuf;
//				if (strbuf[0] == '#') continue;
//				try {
//					calib_level = stoi(strbuf);
//					break;
//				}
//				catch (...) {
//					continue;
//				}
//			}
//			// set initial guess
//			if (this->imageSize().width <= 1) {
//				int w, h; 
//				cout << "Calibration photo image size width/height is " << this->imageSize().width 
//					<< " / " << this->imageSize().height << endl;
//				cout << "Please enter image size (w h): "; 
//				cin >> w >> h;
//				this->setImageSize(w, h);
//			}
//			this->cmat = cv::Mat::eye(3, 3, CV_64F);
//			this->dvec = cv::Mat::zeros(1, 8, CV_64F);
//			cmat.at<double>(0, 0) = this->imageSize().width * 0.5 / tan((39.6 / 2) / 180 * 3.1416); // Focal-Len 50-mm -> View angle W:39.6/H:27.0 deg.
//			cmat.at<double>(1, 1) = cmat.at<double>(0, 0);
//			cmat.at<double>(0, 2) = (this->imgSize.width - 1) / 2.;
//			cmat.at<double>(1, 2) = (this->imgSize.height - 1) / 2.;
//			// set flag by calib_level
//			if (calib_level == 1) {
//				flag |= cv::CALIB_FIX_ASPECT_RATIO;  // fx = fy
//				flag |= cv::CALIB_FIX_PRINCIPAL_POINT; // fix cx, cy (to image center)
//				flag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
//				flag |= cv::CALIB_FIX_K2; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
//				flag |= cv::CALIB_USE_INTRINSIC_GUESS;
//				cal.calibrate(2, flag);
//			}
//			else if (calib_level == 2) {
//				flag |= cv::CALIB_FIX_PRINCIPAL_POINT; // fix cx, cy (to image center)
//				flag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
//				flag |= cv::CALIB_FIX_K2; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
//				flag |= cv::CALIB_USE_INTRINSIC_GUESS;
//				cal.calibrate(2, flag);
//			}
//			else if (calib_level == 3) {
//				flag |= cv::CALIB_ZERO_TANGENT_DIST; // p1 = p2 = 0
//				flag |= cv::CALIB_FIX_K2; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
//				flag |= cv::CALIB_USE_INTRINSIC_GUESS;
//				cal.calibrate(2, flag);
//			}
//			else if (calib_level == 4) {
//				flag |= cv::CALIB_FIX_K2; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
//				flag |= cv::CALIB_USE_INTRINSIC_GUESS;
//				cal.calibrate(2, flag);
//			}
//			else if (calib_level == 5) {
//				flag |= cv::CALIB_FIX_K3; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K4; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K5; // fix kx (to zero)
//				flag |= cv::CALIB_FIX_K6; // fix kx (to zero)
//				flag |= cv::CALIB_USE_INTRINSIC_GUESS;
//				cal.calibrate(2, flag);
//			}
//			else  {
//				cal.setFlagByAsking();
//				cal.calibrate();
//			}
//			// Get a clone image points vector
//			cout << "C-mat: \n" << cal.cameraMatrix() << endl;
//			cout << "D-vec: \n" << cal.distortionVector() << endl;
//			// Get r4 and proj error
//			vector<vector<double> > errs = cal.projection_errors_vecvec(); 
//			cout << "R4 and Projection error of each photo: \n"; 
//			for (int i = 0; i < errs.size(); i++) {
//				// R4 matrix
//				cout << "  R4 mat:\n" << this->R4(i) << endl;
//				cout << "  R4 inv mat:\n" << this->R4(i).inv() << endl;
//				// Proj err
//				if (cal.calTypes()[i] == 1 || cal.calTypes()[i] == 2 ||
//					cal.calTypes()[i] == 3 || cal.calTypes()[i] == 11) {
//					cout << "  " << cal.fileSeq().filename(i) << ": " << cal.projection_errors_vec()[i] << " pixels\n";
//				}
//				for (int j = 0; j < errs[i].size(); j++) {
//					cout << "    " << "Img " << i + 1 << " Point " << j + 1 << " proj err: " << errs[i][j] << "\n";
//				}
//			}
//		}
//		// Selection 7: Write to files
//		if (selection == 7) {
//			string intrFilename; 
//			cout << "File name of intrinsic parameters to output (w/o directory, w/o extenion): ";
//			while (true) {
//				cin >> intrFilename;
//				if (intrFilename[0] == '#') continue;
//				break;
//			}
//			this->writeIntrinsicParamteresToFile(intrFilename + "_intrinsic.txt"); 
//			this->writeToFsFile(intrFilename + "_intrinsic.xml");
//			this->writeToFsFile(intrFilename + "_intrinsic.yaml");
//			this->writeImgsPointsToFiles();
//			this->writeObjsPointsToFiles();			
//		}
//
//		// Selection 8: Independent extrinsic calibration (SolvePnPRansac)
//		if (selection == 8) {
//			int res = findExtrinsic(0, NULL);
//		}
//
//		// Selection 9: Independent points picking 
//		if (selection == 9) {
//			int res = imagePointsPicking(0, NULL);
//		}
//
//		// Selection 10: Independent ECC points tracking 
//		if (selection == 10) {
//			int res = trackPointsEcc(0, NULL);
//		}
//
//		// Selection 11: Independent rename basler file names 
//		if (selection == 11) {
//			int res = baslerFileNameRename(0, NULL);
//		}
//
//	} // end of main menu while loop
//
//	return 0;
//}

