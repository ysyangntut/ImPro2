#define _USE_MATH_DEFINES
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "FileSeq.h"
#include "matchTemplateWithRotPyr.h"
#include "ImagePointsPicker.h"
#include "enhancedCorrelationWithReference.h"
#include "triangulatepoints2.h"
#include "impro_util.h"
#include "Points2fHistoryData.h"
#include "Points3dHistoryData.h"

// Step 1: Read camera parameters (cmat, dvec, rvec, tvec) (single cam)
//         cv::Mat cmat(3, 3, CV_64F), dvec(1, n, CV_64F) (?or (n, 1, CV_64F)), rvec(3, 1, CV_64F), tvec(3, 1, CV_64F)
// Step 2: Read initial photo
//         cv::Mat imgInit;
// Step 3: Ask users to enter world coordinates of three rectangular corners (pw0, pw1, pw2), (lower-left, upper-left, lower-right)
//         vector<cv::Point3f> pw3(3)
// Step 4: Ask users to enter world coordinates of expansion (expx, expy) (for example: 0.2 0.1 for 20% expansion along -X and +X, 10% along -Y and +Y)
//         float expx, expy;
//         further calculate qw3
//         vector<cv::Point3f> qw3(3)
// Step 5: Determine pxl (length of each pixel), w, h (the rectified image size is h x w)
//         float pxl;
//         int w, h;
// Step 6: Generate q mesh
//         cv::Mat qwmesh(h * w, 1, CV_32FC3)
// Step 7: Project qimesh from qwmesh
//         vector<cv::Point2f> qimesh(h * w, 1, CV_32FC2)
// Step 8: Generate a rectified image by cv::remap()
//         (You probably need to reshape Qmesh and qmesh from (h*w, 1) to (h, w). (Use interpolation of CUBIC, or higher order)
//         cv::Mat imgRectf;

int FuncWallSingleCam(int argc, char** argv)
{	// variables
	string fnameCalib;
	cv::FileStorage fsCalib;
	cv::Mat cmat, dvec, rvec, tvec;
	string fnameImgInit;
	cv::Mat imgInit;
	vector<cv::Point3f> pw3(3);
	vector<cv::Point2f> pi3(3);
	vector<cv::Point3f> qw3(3);
	cv::Point3f pw1plumb;
	float expx, expy;
	float pxl_01, pxl_02, pxl_default, pxl;
	int wImgRectf, hImgRectf;
	cv::Mat qwmesh, qimesh;
	cv::Mat img, imgRectf, imgSobelRectf, imgInitRectf, imgPrev, imgCurr, imgNewRectf;
	FileSeq fsqSourceImg, fsqRectfImg;
	int nCellsWidth, nCellsHeight;
	std::vector<cv::Point2f> InitprevPts, prevPts, nextPts;
	cv::Mat disp; // disp(nCellsHeight, nCellsWidth, CV_32FC2) in pixel
	cv::Mat uxyGrid; // displacement fields. (in pixel)
	cv::Mat uxyGrid_mm; // displacement fields. (in mm)
	cv::Mat strain2Dxx, strain2Dyy, strain2Dxy; // displacement fields. (in pixel)
	size_t nCells;
	Points2fHistoryData manyPoints, xyFeatures, rangeTracing, uxyGridPlot;
	Points3dHistoryData strain2DPlot, strain2DPlotTf, crack2D;
	cv::Mat
		ux_left, ux_rigt, ux_up__, ux_down,
		uy_left, uy_rigt, uy_up__, uy_down,
		xx_left, xx_rigt, xx_up__, xx_down,
		yy_left, yy_rigt, yy_up__, yy_down;	//strain
	cv::Mat
		cux_left, cux_rigt, cux_up__, cux_down,
		cuy_left, cuy_rigt, cuy_up__, cuy_down,
		uax, uay, ubx, uby, u_vec, c_vec,
		crack_opening, crack_sliding, crack_angle, max_crack;	//crack
	bool debug = true;

	// Step 1: Read camera parameters (cmat, dvec, rvec, tvec) (single cam)
	//         cv::Mat cmat(3, 3, CV_64F), dvec(1, n, CV_64F) (?or (n, 1, CV_64F)), rvec(3, 1, CV_64F), tvec(3, 1, CV_64F)
	cout << "# Enter full path of calibration file (including cameraMatrix, distortionVector, rvec (3x1), and tvec (3x1):\n";
	fnameCalib = readStringLineFromCin();
	fsCalib.open(fnameCalib, cv::FileStorage::READ);
	if (fsCalib.isOpened() == false) {
		cerr << "Error: # Cannot open file " << fnameCalib << "\n";
		return -1;
	}
	fsCalib["cameraMatrix"] >> cmat;
	fsCalib["distortionVector"] >> dvec;
	fsCalib["rvec"] >> rvec;
	fsCalib["tvec"] >> tvec;

	// Step 2: Read initial photo
	//         cv::Mat imgInit;
	while (true) {
		cout << "# Enter full path of the initial image: \n";
		fnameImgInit = readStringLineFromCin();
		imgInit = cv::imread(fnameImgInit);
		if (imgInit.cols <= 0 || imgInit.rows <= 0) {
			cerr << "# Warning: Cannot read the image file: " << fnameImgInit << "\n";
			cerr << "# Try again.\n";
			continue;
		}
		break;
	}
	imshow_resize("Initial Image", imgInit, 1024.0 / imgInit.cols);
	cv::waitKey(1000);
	cv::destroyWindow("Initial Image");

	// Step 3: Ask users to enter world coordinates of three rectangular corners (pw0, pw1, pw2), (lower-left, upper-left, lower-right)
	//         vector<cv::Point3f> pw3(3)
	cout << "# Enter three control points of the rectangular ROI:\n";
	cout << "#   Lower-left corner (pw0_x pw0_y pw0_z) (e.g., 0 0 0):\n";
	pw3[0].x = (float) readDoubleFromCin();
	pw3[0].y = (float) readDoubleFromCin();
	pw3[0].z = (float) readDoubleFromCin();
	cout << "#   Upper-left corner (pw1_x pw1_y pw1_z) (e.g., 0 200 0):\n";
	pw3[1].x = (float) readDoubleFromCin();
	pw3[1].y = (float) readDoubleFromCin();
	pw3[1].z = (float) readDoubleFromCin();
	pw1plumb = pw3[1];
	cout << "#   Lower-right corner (pw2_x pw2_y pw0_z) (e.g., 400 0 0):\n";
	pw3[2].x = (float) readDoubleFromCin();
	pw3[2].y = (float) readDoubleFromCin();
	pw3[2].z = (float) readDoubleFromCin();
	// adjustment so that P0-P1 is perpenticular to P0-P2, by adjusting P1
	// vx = (P2-P0)/|P2-P0|
	// vyp = (P1-P0)/|P1-P0|
	// vz = (vx cross vyp) / |vz|
	// vy = vz cross vx
	// updated P1 = P0 + (vy dot (P1-P0)) vy
	cv::Point3f vx = pw3[2] - pw3[0];
	vx = vx / sqrt(vx.dot(vx));
	cv::Point3f vyp = pw3[1] - pw3[0];
	vyp = vyp / sqrt(vyp.dot(vyp));
	cv::Point3f vz = vx.cross(vyp);
	vz = vz / sqrt(vz.dot(vz));
	cv::Point3f vy = vz.cross(vx);
	pw3[1] = pw3[0] + vy.dot(pw3[1] - pw3[0]) * vy;
	cout << "# The inputted P1 (p1_plumb) is adjusted to: " << pw3[1] << endl;

	// Step 4: Ask users to enter world coordinates of expansion (expx, expy) (for example: 0.2 0.1 for 20% expansion along -X and +X, 10% along -Y and +Y)
	//         float expx, expy;
	//         further calculate qw3
	//         vector<cv::Point3f> qw3(3)
	cout << "# Enter rectification expansion ratios (expx and expy), (for example: 0.2 0.1 for 20% expansion along -X and +X, 10% along -Y and +Y):\n";
	expx = (float) readDoubleFromCin();
	expy = (float) readDoubleFromCin();
	qw3[0] = pw3[0] - expx * (pw3[2] - pw3[0]) - expy * (pw3[1] - pw3[0]);
	qw3[1] = pw3[1] - expx * (pw3[2] - pw3[0]) + expy * (pw3[1] - pw3[0]);
	qw3[2] = pw3[2] + expx * (pw3[2] - pw3[0]) - expy * (pw3[1] - pw3[0]);

	// Step 5: Determine pxl (length of each pixel), w, h (the rectified image size is h x w)
	//         float pxl;
	//         int w, h;
	cv::projectPoints(pw3, rvec, tvec, cmat, dvec, pi3);
	if (debug) std::cout << "# pi3:\n" << pi3 << std::endl;
	pxl_01 = sqrt((pw3[1] - pw3[0]).dot(pw3[1] - pw3[0])) / sqrt((pi3[1] - pi3[0]).dot(pi3[1] - pi3[0]));
	pxl_02 = sqrt((pw3[2] - pw3[0]).dot(pw3[2] - pw3[0])) / sqrt((pi3[2] - pi3[0]).dot(pi3[2] - pi3[0]));
	pxl_default = std::fmin(pxl_01, pxl_02);
	cout << "# Enter pixel length of rectification image (or enter 0 or negative value for the default value: " << pxl_default << "\n";
	pxl = (float)readDoubleFromCin();
	if (pxl <= 0.0) pxl = pxl_default;
	wImgRectf = (int)(sqrt((qw3[2] - qw3[0]).dot((qw3[2] - qw3[0]))) / pxl + 0.5);
	hImgRectf = (int)(sqrt((qw3[1] - qw3[0]).dot((qw3[1] - qw3[0]))) / pxl + 0.5);

	// Step 6: Generate q mesh
	//         cv::Mat qwmesh(h * w, 1, CV_32FC3)
	int jp0 = (int)((wImgRectf - 1.) * expx / (1. + expx * 2.) + .5);
	int ip0 = (int)((hImgRectf - 1.) * (1. + expy) / (1. + expy * 2.) + .5);
	qwmesh = cv::Mat(hImgRectf * wImgRectf, 1, CV_32FC3);
	// i and j are image coordinate, which are integers.
	int k = 0;
	for (int i = 0; i < hImgRectf; i++) {
		for (int j = 0; j < wImgRectf; j++) {
			qwmesh.at<cv::Point3f>(k, 0) = pw3[0] - (i - ip0) * vy * pxl + (j - jp0) * vx * pxl;
			k = k + 1;
		}
	}

	// Step 7: Project qimesh from qwmesh
	//         vector<cv::Point2f> qimesh(h * w, 1, CV_32FC2)
	qimesh = cv::Mat(hImgRectf * wImgRectf, 1, CV_32FC2);

	cv::projectPoints(qwmesh, rvec, tvec, cmat, dvec, qimesh); 
	qimesh = qimesh.reshape(2, vector<int>{hImgRectf, wImgRectf});

	// Step 8: Generate a rectified image by cv::remap()
	//         (You probably need to reshape Qmesh and qmesh from (h*w, 1) to (h, w). (Use interpolation of CUBIC, or higher order)
	//         cv::Mat imgRectf;
	img = imgInit;
	cv::remap(img, imgRectf, qimesh, cv::noArray(), cv::INTER_CUBIC);
	cv::imshow("Rectf", imgRectf);
	cv::waitKey(1000);
//	std::string InitImgRectf = "G:\\201510_CarletonTest\\20151120\\20200803\\Control_Wall\\Analysis_2\\Rectf\\InitImgRectf.jpg";
	std::cout << "# Enter full path of initial rectified image to output:\n ";
	std::string InitImgRectf = readStringLineFromCin();
	cv::imwrite(InitImgRectf, imgRectf);
	cv::destroyWindow("Rectf");

	//vector<cv::Point3f> pw4(4);
	//pw4[0] = cv::Point3f(98, 78, -127);
	//pw4[1] = cv::Point3f(98, 219.5, -127);
	//pw4[2] = cv::Point3f(177, 78, -127);
	//pw4[3] = cv::Point3f(177, 219.5, -127);
	//vector<cv::Point2f> pi4(4);
	//cv::projectPoints(pw4, rvec, tvec, cmat, dvec, pi4);
	//cout << pi4 << endl;

	// Step 9: Start the loop
	//         (You probably need to reshape Qmesh and qmesh from (h*w, 1) to (h, w). (Use interpolation of CUBIC, or higher order)
	//         cv::Mat imgRectf;
	cout << "# Enter file sequence of source images:\n";
	fsqSourceImg.setDirFilesByConsole();
	cout << "# Enter file sequence of rectified images:\n";
	fsqRectfImg.setDirFilesByConsole();
	cout << "# Enter number of cells along width and height: \n";
	nCellsWidth = readIntFromCin(1, 1000);
	nCellsHeight = readIntFromCin(1, 1000);
	nCells = nCellsWidth * nCellsHeight;

	// define initial vector of 2d points
	prevPts.resize(nCells);
	nextPts.resize(nCells);
	disp = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC2);

	// Pick Feature Points
	std::cout << "# Do you want to define 4 points of camera by mouse picking or from file? (0:mouse picking, else:from file):\n";
	int mouseOrFile = readIntFromCin();
	ImagePointsPicker userPicked;
	userPicked.setBackgroundImageByFilename(InitImgRectf);
	if (mouseOrFile == 0) {
		userPicked.pickTemplates(4, 27, 32, 13);
		userPicked.writePointsToXmlFile((extFilenameRemoved(fsqRectfImg.fullPathOfFile(0)) + "_FourPoints.xml"));
	}
	else {
		userPicked.readPointsFromXmlFile((extFilenameRemoved(fsqRectfImg.fullPathOfFile(0)) + "_FourPoints.xml"));
	}
	userPicked.generateQ4Templates(nCellsWidth, nCellsHeight); // begin from upper-left (counterclockwise)
	prevPts = userPicked.pointsInPoint2fVector();
	InitprevPts = prevPts;
	manyPoints.set(cv::Mat::zeros(1, nCellsWidth * nCellsHeight, CV_32FC2));
	for (int i = 0; i < prevPts.size(); i++) {
		manyPoints.set(0, i, prevPts[i]);
	}
	manyPoints.writeScriptMat((extFilenameRemoved(fsqRectfImg.fullPathOfFile(0)) + "_manyPoints.m"));
	//	manyPoints.writeToXml((extFilenameRemoved(fsqRectfImg.fullPathOfFile(0)) + "_manyPoints.xml"));

	xyFeatures.set(cv::Mat::zeros(nCellsHeight, nCellsWidth, CV_32FC2));
	for (int i = 0; i < nCellsHeight; i++) {
		for (int j = 0; j < nCellsWidth; j++) {
			xyFeatures.set(i, j, manyPoints.get(0, i * nCellsWidth + j));
		}
	}
	// save Feature Points of Initial Rectified image to file
	xyFeatures.writeScriptMat((extFilenameRemoved(fsqRectfImg.fullPathOfFile(0)) + "_xyFeatures.m"));
	//	xyFeatures.writeToXml((extFilenameRemoved(fsqRectfImg.fullPathOfFile(0)) + "_xyFeatures.xml"));

		// start the loop
	for (int iStep = 0; iStep < fsqSourceImg.num_files(); iStep++)
	{
		// define the previous image
		if (iStep == 0) {
			imgRectf.copyTo(imgInitRectf);
			imgSobelRectf = sobel_xy(imgInitRectf);
			imgSobelRectf.copyTo(imgPrev);
		}
		else {

			//imgSobelRectf.copyTo(imgPrev);
			imgCurr.copyTo(imgPrev);
			prevPts = nextPts;
		}
		// Read or Wait source image
		fsqSourceImg.waitForImageFile(iStep, imgCurr);
		// Rectification by remapping
		cv::remap(imgCurr, imgRectf, qimesh, cv::noArray(), cv::INTER_CUBIC);
		imgRectf.copyTo(imgNewRectf);
		imgSobelRectf = sobel_xy(imgNewRectf);
		imgSobelRectf.copyTo(imgCurr);
		cv::imshow("Rectf", imgRectf);
		cv::imshow("imgPrev", imgPrev);
		cv::imshow("imgCurr", imgCurr);
		cv::waitKey(1);
		// save rectified image to file
		std::string fnameImgRectf = fsqRectfImg.fullPathOfFile(iStep);
		cv::imwrite(fnameImgRectf, imgRectf);

		// displacement (image tracking)
		cv::Mat optFlow_status, optFlow_error, infoTM_Acc;
		optFlow_status = cv::Mat(1, (int) nCells, CV_8UC1);
		optFlow_error = cv::Mat(1, (int) nCells, CV_32FC1);
		infoTM_Acc = cv::Mat(1, (int) nCells, CV_32FC1);
		int winSx = 21;
		int	winSy = 21;
		cv::Size winSize = cv::Size(winSx, winSy);
		int maxLevel = 3;
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 0.001);

		cv::calcOpticalFlowPyrLK(
			imgPrev,	// previous photo
			imgCurr,	// current photo
			prevPts,
			nextPts,
			optFlow_status,
			optFlow_error,
			winSize,
			maxLevel,
			criteria);

		for (int iCell = 0; iCell < nCellsHeight; iCell++) {
			for (int jCell = 0; jCell < nCellsWidth; jCell++) {
				disp.at<cv::Point2f>(iCell, jCell) =
					nextPts[(size_t)iCell * (size_t)nCellsWidth + (size_t)jCell] -
					InitprevPts[(size_t)iCell * (size_t)nCellsWidth + (size_t)jCell];
			}
		}
		infoTM_Acc = 1. - optFlow_error / (winSx * winSy) / 128;

		std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\bStep " << iStep << endl;
		printf("OptFlow\n");
		printf("%10.2f %10.2f  \n", disp.at<cv::Point2f>(0, 0).x, disp.at<cv::Point2f>(0, 0).y); // upper-left corner
		printf("%10.2f %10.2f  \n", disp.at<cv::Point2f>(0, nCellsWidth - 1).x, disp.at<cv::Point2f>(0, nCellsWidth - 1).y); // upper-right corner
		printf("%10.2f %10.2f  \n", disp.at<cv::Point2f>(nCellsHeight - 1, 0).x, disp.at<cv::Point2f>(nCellsHeight - 1, 0).y); // lower-left corner
		printf("%10.2f %10.2f  \n", disp.at<cv::Point2f>(nCellsHeight - 1, nCellsWidth - 1).x, disp.at<cv::Point2f>(nCellsHeight - 1, nCellsWidth - 1).y); // lower-right corner
//		cv::FileStorage fs_disp(extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_disp_pixel.xml", FileStorage::WRITE);
//		fs_disp << "Optflow_disp" << disp;
//		cv::FileStorage fs_status(extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_status.xml", FileStorage::WRITE);
//		fs_status << "Optflow_status" << optFlow_status;
//		cv::FileStorage fs_error(extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_error.xml", FileStorage::WRITE);
//		fs_error << "Optflow_error" << optFlow_error;
//		cv::FileStorage fs_infoTM_Acc(extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_infoTM_Acc.xml", FileStorage::WRITE);
//		fs_infoTM_Acc << "Optflow_infoTM_Acc" << infoTM_Acc;

		rangeTracing.set(cv::Mat::zeros(1, nCellsHeight * nCellsWidth, CV_32FC2));
		for (int i = 0; i < nextPts.size(); i++) {
			rangeTracing.set(0, i, nextPts[i]);
		}
		rangeTracing.writeScriptMatAdvanced((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_rangeTracing.m"), fnameImgRectf, false, 1, 1 /* only data */);
		//		rangeTracing.writeToXml((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_rangeTracing.xml"));

				// strain2Dxx_yy_xy (nCellsHeight * nCellsWidth) ¡G the strain component of the region.
				// uxyGrid (nCellsHeight * nCellsWidth)	¡G displacement field on regular grid

				// calculation of strain
				// Note: strain2D is in image coordinate (which image Y vector is downward).
				// Normally in 3D the Y is upward, so the strain2Dxy will be inverted.

		disp.copyTo(uxyGrid); // in pixel

		ux_left = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); ux_rigt = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		ux_up__ = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); ux_down = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		uy_left = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); uy_rigt = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		uy_up__ = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); uy_down = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		xx_left = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); xx_rigt = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		xx_up__ = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); xx_down = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		yy_left = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); yy_rigt = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		yy_up__ = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1); yy_down = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		strain2Dxx = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		strain2Dyy = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		strain2Dxy = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);

		for (int i = 1; i < (nCellsHeight - 1); i++) {
			for (int j = 1; j < (nCellsWidth - 1); j++) {
				ux_left.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i, j - 1).x;
				ux_rigt.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i, j + 1).x;
				uy_left.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i, j - 1).y;
				uy_rigt.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i, j + 1).y;
				ux_up__.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i - 1, j).x;
				ux_down.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i + 1, j).x;
				uy_up__.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i - 1, j).y;
				uy_down.at<float>(i, j) = uxyGrid.at<cv::Point2f>(i + 1, j).y;

				xx_left.at<float>(i, j) = xyFeatures.get(i, j - 1).x;
				xx_rigt.at<float>(i, j) = xyFeatures.get(i, j + 1).x;
				yy_left.at<float>(i, j) = xyFeatures.get(i, j - 1).y;
				yy_rigt.at<float>(i, j) = xyFeatures.get(i, j + 1).y;
				xx_up__.at<float>(i, j) = xyFeatures.get(i - 1, j).x;
				xx_down.at<float>(i, j) = xyFeatures.get(i + 1, j).x;
				yy_up__.at<float>(i, j) = xyFeatures.get(i - 1, j).y;
				yy_down.at<float>(i, j) = xyFeatures.get(i + 1, j).y;

				strain2Dxx.at<float>(i, j) = (ux_rigt.at<float>(i, j) - ux_left.at<float>(i, j)) / (xx_rigt.at<float>(i, j) - xx_left.at<float>(i, j));
				strain2Dyy.at<float>(i, j) = (uy_up__.at<float>(i, j) - uy_down.at<float>(i, j)) / (yy_up__.at<float>(i, j) - yy_down.at<float>(i, j));
				strain2Dxy.at<float>(i, j) = (ux_up__.at<float>(i, j) - ux_down.at<float>(i, j)) / (yy_up__.at<float>(i, j) - yy_down.at<float>(i, j))
					+ (uy_rigt.at<float>(i, j) - uy_left.at<float>(i, j)) / (xx_rigt.at<float>(i, j) - xx_left.at<float>(i, j));
			}
		}
		// strains at boundaries are specially calculated.
		// upper boundary
		// i = 0;
		for (int j = 1; j < (nCellsWidth - 1); j++) {
			ux_left.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0, j - 1).x;
			ux_rigt.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0, j + 1).x;
			uy_left.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0, j - 1).y;
			uy_rigt.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0, j + 1).y;
			ux_up__.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0 - 0, j).x;
			ux_down.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0 + 1, j).x;
			uy_up__.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0 - 0, j).y;
			uy_down.at<float>(0, j) = uxyGrid.at<cv::Point2f>(0 + 1, j).y;

			xx_left.at<float>(0, j) = xyFeatures.get(0, j - 1).x;
			xx_rigt.at<float>(0, j) = xyFeatures.get(0, j + 1).x;
			yy_left.at<float>(0, j) = xyFeatures.get(0, j - 1).y;
			yy_rigt.at<float>(0, j) = xyFeatures.get(0, j + 1).y;
			xx_up__.at<float>(0, j) = xyFeatures.get(0 - 0, j).x;
			xx_down.at<float>(0, j) = xyFeatures.get(0 + 1, j).x;
			yy_up__.at<float>(0, j) = xyFeatures.get(0 - 0, j).y;
			yy_down.at<float>(0, j) = xyFeatures.get(0 + 1, j).y;

			strain2Dxx.at<float>(0, j) = (ux_rigt.at<float>(0, j) - ux_left.at<float>(0, j)) / (xx_rigt.at<float>(0, j) - xx_left.at<float>(0, j));
			strain2Dyy.at<float>(0, j) = (uy_up__.at<float>(0, j) - uy_down.at<float>(0, j)) / (yy_up__.at<float>(0, j) - yy_down.at<float>(0, j));
			strain2Dxy.at<float>(0, j) = 0.5f * ((ux_up__.at<float>(0, j) - ux_down.at<float>(0, j)) / (yy_up__.at<float>(0, j) - yy_down.at<float>(0, j))
				+ (uy_rigt.at<float>(0, j) - uy_left.at<float>(0, j)) / (xx_rigt.at<float>(0, j) - xx_left.at<float>(0, j)));
		}

		// lower boundary
		// i = (nCellsHeight - 1);
		for (int j = 1; j < (nCellsWidth - 1); j++) {
			ux_left.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), j - 1).x;
			ux_rigt.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), j + 1).x;
			uy_left.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), j - 1).y;
			uy_rigt.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), j + 1).y;
			ux_up__.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) - 1, j).x;
			ux_down.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) + 0, j).x;
			uy_up__.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) - 1, j).y;
			uy_down.at<float>((nCellsHeight - 1), j) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) + 0, j).y;

			xx_left.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1), j - 1).x;
			xx_rigt.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1), j + 1).x;
			yy_left.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1), j - 1).y;
			yy_rigt.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1), j + 1).y;
			xx_up__.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1) - 1, j).x;
			xx_down.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1) + 0, j).x;
			yy_up__.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1) - 1, j).y;
			yy_down.at<float>((nCellsHeight - 1), j) = xyFeatures.get((nCellsHeight - 1) + 0, j).y;

			strain2Dxx.at<float>((nCellsHeight - 1), j) = (ux_rigt.at<float>((nCellsHeight - 1), j) - ux_left.at<float>((nCellsHeight - 1), j))
				/ (xx_rigt.at<float>((nCellsHeight - 1), j) - xx_left.at<float>((nCellsHeight - 1), j));
			strain2Dyy.at<float>((nCellsHeight - 1), j) = (uy_up__.at<float>((nCellsHeight - 1), j) - uy_down.at<float>((nCellsHeight - 1), j))
				/ (yy_up__.at<float>((nCellsHeight - 1), j) - yy_down.at<float>((nCellsHeight - 1), j));
			strain2Dxy.at<float>((nCellsHeight - 1), j) = 0.5f * ((ux_up__.at<float>((nCellsHeight - 1), j) - ux_down.at<float>((nCellsHeight - 1), j)) / (yy_up__.at<float>((nCellsHeight - 1), j) - yy_down.at<float>((nCellsHeight - 1), j))
				+ (uy_rigt.at<float>((nCellsHeight - 1), j) - uy_left.at<float>((nCellsHeight - 1), j)) / (xx_rigt.at<float>((nCellsHeight - 1), j) - xx_left.at<float>((nCellsHeight - 1), j)));
		}

		// left boundary
		// j = 0;
		for (int i = 1; i < (nCellsHeight - 1); i++) {
			ux_left.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i, 0 - 0).x;
			ux_rigt.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i, 0 + 1).x;
			uy_left.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i, 0 - 0).y;
			uy_rigt.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i, 0 + 1).y;
			ux_up__.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i - 1, 0).x;
			ux_down.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i + 1, 0).x;
			uy_up__.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i - 1, 0).y;
			uy_down.at<float>(i, 0) = uxyGrid.at<cv::Point2f>(i + 1, 0).y;

			xx_left.at<float>(i, 0) = xyFeatures.get(i, 0 - 0).x;
			xx_rigt.at<float>(i, 0) = xyFeatures.get(i, 0 + 1).x;
			yy_left.at<float>(i, 0) = xyFeatures.get(i, 0 - 0).y;
			yy_rigt.at<float>(i, 0) = xyFeatures.get(i, 0 + 1).y;
			xx_up__.at<float>(i, 0) = xyFeatures.get(i - 1, 0).x;
			xx_down.at<float>(i, 0) = xyFeatures.get(i + 1, 0).x;
			yy_up__.at<float>(i, 0) = xyFeatures.get(i - 1, 0).y;
			yy_down.at<float>(i, 0) = xyFeatures.get(i + 1, 0).y;

			strain2Dxx.at<float>(i, 0) = (ux_rigt.at<float>(i, 0) - ux_left.at<float>(i, 0)) / (xx_rigt.at<float>(i, 0) - xx_left.at<float>(i, 0));
			strain2Dyy.at<float>(i, 0) = (uy_up__.at<float>(i, 0) - uy_down.at<float>(i, 0)) / (yy_up__.at<float>(i, 0) - yy_down.at<float>(i, 0));
			strain2Dxy.at<float>(i, 0) = 0.5f * ((ux_up__.at<float>(i, 0) - ux_down.at<float>(i, 0)) / (yy_up__.at<float>(i, 0) - yy_down.at<float>(i, 0))
				+ (uy_rigt.at<float>(i, 0) - uy_left.at<float>(i, 0)) / (xx_rigt.at<float>(i, 0) - xx_left.at<float>(i, 0)));
		}

		// right boundary
		// j = (nCellsWidth - 1);
		for (int i = 1; i < (nCellsHeight - 1); i++) {
			ux_left.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i, (nCellsWidth - 1) - 1).x;
			ux_rigt.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i, (nCellsWidth - 1) + 0).x;
			uy_left.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i, (nCellsWidth - 1) - 1).y;
			uy_rigt.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i, (nCellsWidth - 1) + 0).y;
			ux_up__.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i - 1, (nCellsWidth - 1)).x;
			ux_down.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i + 1, (nCellsWidth - 1)).x;
			uy_up__.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i - 1, (nCellsWidth - 1)).y;
			uy_down.at<float>(i, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(i + 1, (nCellsWidth - 1)).y;

			xx_left.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i, (nCellsWidth - 1) - 1).x;
			xx_rigt.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i, (nCellsWidth - 1) + 0).x;
			yy_left.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i, (nCellsWidth - 1) - 1).y;
			yy_rigt.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i, (nCellsWidth - 1) + 0).y;
			xx_up__.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i - 1, (nCellsWidth - 1)).x;
			xx_down.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i + 1, (nCellsWidth - 1)).x;
			yy_up__.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i - 1, (nCellsWidth - 1)).y;
			yy_down.at<float>(i, (nCellsWidth - 1)) = xyFeatures.get(i + 1, (nCellsWidth - 1)).y;

			strain2Dxx.at<float>(i, (nCellsWidth - 1)) = (ux_rigt.at<float>(i, (nCellsWidth - 1)) - ux_left.at<float>(i, (nCellsWidth - 1)))
				/ (xx_rigt.at<float>(i, (nCellsWidth - 1)) - xx_left.at<float>(i, (nCellsWidth - 1)));

			strain2Dyy.at<float>(i, (nCellsWidth - 1)) = (uy_up__.at<float>(i, (nCellsWidth - 1)) - uy_down.at<float>(i, (nCellsWidth - 1)))
				/ (yy_up__.at<float>(i, (nCellsWidth - 1)) - yy_down.at<float>(i, (nCellsWidth - 1)));

			strain2Dxy.at<float>(i, (nCellsWidth - 1)) = 0.5f * ((ux_up__.at<float>(i, (nCellsWidth - 1)) - ux_down.at<float>(i, (nCellsWidth - 1))) / (yy_up__.at<float>(i, (nCellsWidth - 1)) - yy_down.at<float>(i, (nCellsWidth - 1)))
				+ (uy_rigt.at<float>(i, (nCellsWidth - 1)) - uy_left.at<float>(i, (nCellsWidth - 1))) / (xx_rigt.at<float>(i, (nCellsWidth - 1)) - xx_left.at<float>(i, (nCellsWidth - 1))));
		}

		// upper-left corner
		// i = 0; j = 0;
		ux_left.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0, 0 - 0).x;
		ux_rigt.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0, 0 + 1).x;
		uy_left.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0, 0 - 0).y;
		uy_rigt.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0, 0 + 1).y;
		ux_up__.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0 - 0, 0).x;
		ux_down.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0 + 1, 0).x;
		uy_up__.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0 - 0, 0).y;
		uy_down.at<float>(0, 0) = uxyGrid.at<cv::Point2f>(0 + 1, 0).y;

		xx_left.at<float>(0, 0) = xyFeatures.get(0, 0 - 0).x;
		xx_rigt.at<float>(0, 0) = xyFeatures.get(0, 0 + 1).x;
		yy_left.at<float>(0, 0) = xyFeatures.get(0, 0 - 0).y;
		yy_rigt.at<float>(0, 0) = xyFeatures.get(0, 0 + 1).y;
		xx_up__.at<float>(0, 0) = xyFeatures.get(0 - 0, 0).x;
		xx_down.at<float>(0, 0) = xyFeatures.get(0 + 1, 0).x;
		yy_up__.at<float>(0, 0) = xyFeatures.get(0 - 0, 0).y;
		yy_down.at<float>(0, 0) = xyFeatures.get(0 + 1, 0).y;

		strain2Dxx.at<float>(0, 0) = (ux_rigt.at<float>(0, 0) - ux_left.at<float>(0, 0)) / (xx_rigt.at<float>(0, 0) - xx_left.at<float>(0, 0));
		strain2Dyy.at<float>(0, 0) = (uy_up__.at<float>(0, 0) - uy_down.at<float>(0, 0)) / (yy_up__.at<float>(0, 0) - yy_down.at<float>(0, 0));
		strain2Dxy.at<float>(0, 0) = 0.5f * ((ux_up__.at<float>(0, 0) - ux_down.at<float>(0, 0)) / (yy_up__.at<float>(0, 0) - yy_down.at<float>(0, 0))
			+ (uy_rigt.at<float>(0, 0) - uy_left.at<float>(0, 0)) / (xx_rigt.at<float>(0, 0) - xx_left.at<float>(0, 0)));

		// lower-left corner
		// i = (nCellsHeight - 1); j = 0;
		ux_left.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), 0 - 0).x;
		ux_rigt.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), 0 + 1).x;
		uy_left.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), 0 - 0).y;
		uy_rigt.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), 0 + 1).y;
		ux_up__.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) - 1, 0).x;
		ux_down.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) + 0, 0).x;
		uy_up__.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) - 1, 0).y;
		uy_down.at<float>((nCellsHeight - 1), 0) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) + 0, 0).y;

		xx_left.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1), 0 - 0).x;
		xx_rigt.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1), 0 + 1).x;
		yy_left.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1), 0 - 0).y;
		yy_rigt.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1), 0 + 1).y;
		xx_up__.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1) - 1, 0).x;
		xx_down.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1) + 0, 0).x;
		yy_up__.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1) - 1, 0).y;
		yy_down.at<float>((nCellsHeight - 1), 0) = xyFeatures.get((nCellsHeight - 1) + 0, 0).y;

		strain2Dxx.at<float>((nCellsHeight - 1), 0) = (ux_rigt.at<float>((nCellsHeight - 1), 0) - ux_left.at<float>((nCellsHeight - 1), 0))
			/ (xx_rigt.at<float>((nCellsHeight - 1), 0) - xx_left.at<float>((nCellsHeight - 1), 0));

		strain2Dyy.at<float>((nCellsHeight - 1), 0) = (uy_up__.at<float>((nCellsHeight - 1), 0) - uy_down.at<float>((nCellsHeight - 1), 0))
			/ (yy_up__.at<float>((nCellsHeight - 1), 0) - yy_down.at<float>((nCellsHeight - 1), 0));

		strain2Dxy.at<float>((nCellsHeight - 1), 0) = 0.5f * ((ux_up__.at<float>((nCellsHeight - 1), 0) - ux_down.at<float>((nCellsHeight - 1), 0)) / (yy_up__.at<float>((nCellsHeight - 1), 0) - yy_down.at<float>((nCellsHeight - 1), 0))
			+ (uy_rigt.at<float>((nCellsHeight - 1), 0) - uy_left.at<float>((nCellsHeight - 1), 0)) / (xx_rigt.at<float>((nCellsHeight - 1), 0) - xx_left.at<float>((nCellsHeight - 1), 0)));

		// upper-right corner
		// i = 0; j = (nCellsWidth - 1);
		ux_left.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0, (nCellsWidth - 1) - 1).x;
		ux_rigt.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0, (nCellsWidth - 1) + 0).x;
		uy_left.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0, (nCellsWidth - 1) - 1).y;
		uy_rigt.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0, (nCellsWidth - 1) + 0).y;
		ux_up__.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0 - 0, (nCellsWidth - 1)).x;
		ux_down.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0 + 1, (nCellsWidth - 1)).x;
		uy_up__.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0 - 0, (nCellsWidth - 1)).y;
		uy_down.at<float>(0, (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>(0 + 1, (nCellsWidth - 1)).y;

		xx_left.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0, (nCellsWidth - 1) - 1).x;
		xx_rigt.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0, (nCellsWidth - 1) + 0).x;
		yy_left.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0, (nCellsWidth - 1) - 1).y;
		yy_rigt.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0, (nCellsWidth - 1) + 0).y;
		xx_up__.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0 - 0, (nCellsWidth - 1)).x;
		xx_down.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0 + 1, (nCellsWidth - 1)).x;
		yy_up__.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0 - 0, (nCellsWidth - 1)).y;
		yy_down.at<float>(0, (nCellsWidth - 1)) = xyFeatures.get(0 + 1, (nCellsWidth - 1)).y;

		strain2Dxx.at<float>(0, (nCellsWidth - 1)) = (ux_rigt.at<float>(0, (nCellsWidth - 1)) - ux_left.at<float>(0, (nCellsWidth - 1)))
			/ (xx_rigt.at<float>(0, (nCellsWidth - 1)) - xx_left.at<float>(0, (nCellsWidth - 1)));

		strain2Dyy.at<float>(0, (nCellsWidth - 1)) = (uy_up__.at<float>(0, (nCellsWidth - 1)) - uy_down.at<float>(0, (nCellsWidth - 1)))
			/ (yy_up__.at<float>(0, (nCellsWidth - 1)) - yy_down.at<float>(0, (nCellsWidth - 1)));

		strain2Dxy.at<float>(0, (nCellsWidth - 1)) = 0.5f * ((ux_up__.at<float>(0, (nCellsWidth - 1)) - ux_down.at<float>(0, (nCellsWidth - 1))) / (yy_up__.at<float>(0, (nCellsWidth - 1)) - yy_down.at<float>(0, (nCellsWidth - 1)))
			+ (uy_rigt.at<float>(0, (nCellsWidth - 1)) - uy_left.at<float>(0, (nCellsWidth - 1))) / (xx_rigt.at<float>(0, (nCellsWidth - 1)) - xx_left.at<float>(0, (nCellsWidth - 1))));

		// lower-right corner
		// i = (nCellsHeight - 1); j = (nCellsWidth - 1);
		ux_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), (nCellsWidth - 1) - 1).x;
		ux_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), (nCellsWidth - 1) + 0).x;
		uy_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), (nCellsWidth - 1) - 1).y;
		uy_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1), (nCellsWidth - 1) + 0).y;
		ux_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) - 1, (nCellsWidth - 1)).x;
		ux_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) + 0, (nCellsWidth - 1)).x;
		uy_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) - 1, (nCellsWidth - 1)).y;
		uy_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = uxyGrid.at<cv::Point2f>((nCellsHeight - 1) + 0, (nCellsWidth - 1)).y;

		xx_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1), (nCellsWidth - 1) - 1).x;
		xx_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1), (nCellsWidth - 1) + 0).x;
		yy_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1), (nCellsWidth - 1) - 1).y;
		yy_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1), (nCellsWidth - 1) + 0).y;
		xx_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1) - 1, (nCellsWidth - 1)).x;
		xx_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1) + 0, (nCellsWidth - 1)).x;
		yy_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1) - 1, (nCellsWidth - 1)).y;
		yy_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = xyFeatures.get((nCellsHeight - 1) + 0, (nCellsWidth - 1)).y;

		strain2Dxx.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = (ux_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - ux_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1)))
			/ (xx_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - xx_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1)));

		strain2Dyy.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = (uy_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - uy_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)))
			/ (yy_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - yy_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)));

		strain2Dxy.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) = 0.5f * ((ux_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - ux_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1))) / (yy_up__.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - yy_down.at<float>((nCellsHeight - 1), (nCellsWidth - 1)))
			+ (uy_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - uy_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1))) / (xx_rigt.at<float>((nCellsHeight - 1), (nCellsWidth - 1)) - xx_left.at<float>((nCellsHeight - 1), (nCellsWidth - 1))));

		strain2DPlot = cv::Mat(nCellsHeight, nCellsWidth, CV_64FC3);
		for (int i = 0; i < nCellsHeight; i++) {
			for (int j = 0; j < nCellsWidth; j++) {
				cv::Point3d strainPoint = cv::Point3d(double(strain2Dxx.at<float>(i, j)), double(strain2Dyy.at<float>(i, j)), double(strain2Dxy.at<float>(i, j)));
				strain2DPlot.set(i, j, strainPoint);
			}
		}

		strain2DPlotTf = cv::Mat(1, nCellsHeight * nCellsWidth, CV_64FC3);
		for (int i = 0; i < nCellsHeight; i++) {
			for (int j = 0; j < nCellsWidth; j++) {
				strain2DPlotTf.set(0, i * nCellsWidth + j, strain2DPlot.get(i, j));
			}
		}
		// Save strain2D_xx_yy_xy to file
		strain2DPlotTf.writeScriptMatAdvanced((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_strain2Dxx_yy_xy.m"), false, 1, 1 /* only data */);
		//		strain2DPlotTf.writeToXml((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_strain2Dxx_yy_xy.xml"));

				// Displacement field
				// Calculation of Ux & Uy (in mm)
		uxyGrid_mm = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC2);
		for (int i = 0; i < nCellsHeight; i++) {
			for (int j = 0; j < nCellsWidth; j++) {
				uxyGrid_mm.at<cv::Point2f>(i, j).x = (float(pxl)) * uxyGrid.at<cv::Point2f>(i, j).x;
				uxyGrid_mm.at<cv::Point2f>(i, j).y = (float(-pxl)) * uxyGrid.at<cv::Point2f>(i, j).y;
			}
		}
		uxyGridPlot.set(cv::Mat::zeros(1, nCellsHeight * nCellsWidth, CV_32FC2));
		for (int i = 0; i < nCellsHeight; i++) {
			for (int j = 0; j < nCellsWidth; j++) {
				uxyGridPlot.set(0, i * nCellsWidth + j, uxyGrid_mm.at<cv::Point2f>(i, j));
			}
		}
		// Save Ux_Uy to file (in mm)
		uxyGridPlot.writeScriptMatAdvanced((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_Ux_Uy.m"), fnameImgRectf, false, 1, 1 /* only data */);
		//		uxyGridPlot.writeToXml((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_Ux_Uy.xml"));

				// Crack Field
				// Calculates crack fields, including crack opening and crack sliding.
		cux_left = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);	cux_rigt = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		cux_up__ = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);	cux_down = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		cuy_left = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);	cuy_rigt = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		cuy_up__ = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);	cuy_down = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		uax = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);	uay = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		ubx = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);	uby = cv::Mat(nCellsHeight, nCellsWidth, CV_32FC1);
		u_vec = cv::Mat(2, nCellsHeight * nCellsWidth, CV_32FC1);	c_vec = cv::Mat(2, nCellsHeight * nCellsWidth, CV_32FC1);
		crack_opening = cv::Mat(1, nCellsHeight * nCellsWidth, CV_32FC1); crack_sliding = cv::Mat(1, nCellsHeight * nCellsWidth, CV_32FC1);
		crack_angle = cv::Mat(1, nCellsHeight * nCellsWidth, CV_32FC1); max_crack = cv::Mat(1, nCellsHeight * nCellsWidth, CV_32FC1);


		int theta; // theta is between 0 and 179.999...
		for (theta = 0; theta <= 135; theta += 45) { // 0 45 90 135
			theta = theta % 180;
			Mat mat_cr(2, 2, CV_32FC1);
			mat_cr.at<float>(1, 1) = (float) sin(theta * M_PI / 180);
			mat_cr.at<float>(0, 1) = (float) cos(theta * M_PI / 180);
			mat_cr.at<float>(0, 0) = (float) cos(theta * M_PI / 180 + M_PI / 2);
			mat_cr.at<float>(1, 0) = (float) sin(theta * M_PI / 180 + M_PI / 2);
			Mat inv_mat_cr = mat_cr.inv();

			// calculate cux_left, cuy_left, ..., cux_down, cuy_down
			for (int i = 0; i < nCellsHeight; i++) {
				for (int j = 0; j < nCellsWidth; j++) {
					if (j > 0) { // not left bound(normal)
						cux_left.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j - 1).x;
						cuy_left.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j - 1).y;
					}
					else {       // left bound(special case)
						cux_left.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).x;
						cuy_left.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).y;
					}
					if (j < (nCellsWidth - 1)) { // not right bound(normal)
						cux_rigt.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j + 1).x;
						cuy_rigt.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j + 1).y;
					}
					else {      // right bound(special case)
						cux_rigt.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).x;
						cuy_rigt.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).y;
					}
					if (i > 0) { // not upper bound(normal)
						cux_up__.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i - 1, j).x;
						cuy_up__.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i - 1, j).y;
					}
					else { // upper bound(special case)
						cux_up__.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).x;
						cuy_up__.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).y;
					}
					if (i < (nCellsHeight - 1)) { // not lower bound(normal)
						cux_down.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i + 1, j).x;
						cuy_down.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i + 1, j).y;
					}
					else { // lower bound(special case)
						cux_down.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).x;
						cuy_down.at<float>(i, j) = uxyGrid_mm.at<cv::Point2f>(i, j).y;
					}
				}
			}

			// Calclate uax, uay, ubx, uby
			float c = (float) cos(theta);
			float s = (float) sin(theta);
			if (theta >= 0 && theta < 90) {
				for (int i = 0; i < nCellsHeight; i++) {
					for (int j = 0; j < nCellsWidth; j++) {
						uax.at<float>(i, j) = (cux_up__.at<float>(i, j) * c + cux_left.at<float>(i, j) * s) / (c + s);
						uay.at<float>(i, j) = (cuy_up__.at<float>(i, j) * c + cuy_left.at<float>(i, j) * s) / (c + s);
						ubx.at<float>(i, j) = (cux_down.at<float>(i, j) * c + cux_rigt.at<float>(i, j) * s) / (c + s);
						uby.at<float>(i, j) = (cuy_down.at<float>(i, j) * c + cuy_rigt.at<float>(i, j) * s) / (c + s);
					}
				}
			}
			else {
				for (int i = 0; i < nCellsHeight; i++) {
					for (int j = 0; j < nCellsWidth; j++) {
						uax.at<float>(i, j) = ((-cux_down.at<float>(i, j)) * c + cux_left.at<float>(i, j) * s) / (-c + s);
						uay.at<float>(i, j) = ((-cuy_down.at<float>(i, j)) * c + cuy_left.at<float>(i, j) * s) / (-c + s);
						ubx.at<float>(i, j) = ((-cux_up__.at<float>(i, j)) * c + cux_rigt.at<float>(i, j) * s) / (-c + s);
						uby.at<float>(i, j) = ((-cuy_up__.at<float>(i, j)) * c + cuy_rigt.at<float>(i, j) * s) / (-c + s);
					}
				}
			}

			//Calculate crack opening and crack sliding
			for (int i = 0; i < nCellsHeight; i++) {
				for (int j = 0; j < nCellsWidth; j++) {
					u_vec.at<float>(0, i * nCellsWidth + j) = uax.at<float>(i, j) - ubx.at<float>(i, j);
					u_vec.at<float>(1, i * nCellsWidth + j) = uay.at<float>(i, j) - uby.at<float>(i, j);
				}
			}
			c_vec = inv_mat_cr * u_vec;  // c_vec(0) is crack_opening ; c_vec(1) is crack_sliding

			if (theta == 0) {
				for (int i = 0; i < nCellsHeight; i++) {
					for (int j = 0; j < nCellsWidth; j++) {
						max_crack.at<float>(0, i * nCellsWidth + j) = c_vec.at<float>(0, i * nCellsWidth + j);
						crack_angle = cv::Mat::zeros(size(crack_opening), CV_32FC1);
					}
				}
			}
			else { // crack_angle is the crack angle which opening is largest
				for (int i = 0; i < nCellsHeight; i++) {
					for (int j = 0; j < nCellsWidth; j++) {
						max_crack.at<float>(0, i * nCellsWidth + j) = max(c_vec.at<float>(0, i * nCellsWidth + j), max_crack.at<float>(0, i * nCellsWidth + j));
						if (c_vec.at<float>(0, i * nCellsWidth + j) == max_crack.at<float>(0, i * nCellsWidth + j))
							crack_angle.at<float>(0, i * nCellsWidth + j) = (float) theta;
					}
				}
			}
		}

		crack2D.set(cv::Mat::zeros(1, nCellsHeight * nCellsWidth, CV_64FC3));
		for (int i = 0; i < nCellsHeight; i++) {
			for (int j = 0; j < nCellsWidth; j++) {
				cv::Point3d crack =
					cv::Point3d(
						double(max_crack.at<float>(0, i * nCellsWidth + j)),
						double(c_vec.at<float>(1, i * nCellsWidth + j)),
						double(crack_angle.at<float>(0, i * nCellsWidth + j)));
				crack2D.set(0, i * nCellsWidth + j, crack);
			}
		}
		crack2D.writeScriptMatAdvanced((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_crack2D.m"), false, 1, 1 /* only data */);
		//		crack2D.writeToXml((extFilenameRemoved(fsqRectfImg.fullPathOfFile(iStep)) + "_crack2D.xml"));

	}

	cv::destroyWindow("Rectf");
	cv::destroyWindow("imgPrev");
	cv::destroyWindow("imgCurr");
	return 0;
}