#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include "impro_util.h"
#include "triangulatepoints2.h"

using namespace std;

int FuncTriangulationAllSteps(int argc, char ** argv)
{
	cv::Mat cmat1(3, 3, CV_64F);  // camera matrix of intrinsic parameters. (3,3,CV_64F)
	cv::Mat dvec1(1, 5, CV_64F);  // distortion vector of intrinsic parameters. (1,5,CV_64F)
	cv::Mat rvec1(3, 1, CV_64F);  // camera rotation vector of extrinsic parameters. (3,1,CV_64F)
	cv::Mat rmat1(3, 3, CV_64F);  // camera rotation matrix of extrinsic parameters. (3,3,CV_64F)
	cv::Mat tvec1(3, 1, CV_64F);  // camera translation vector of extrinsic parameters. (3,1,CV_64F)

	cv::Mat cmat2(3, 3, CV_64F);  // camera matrix of intrinsic parameters. (3,3,CV_64F)
	cv::Mat dvec2(1, 5, CV_64F);  // distortion vector of intrinsic parameters. (1,5,CV_64F)
	cv::Mat rvec2(3, 1, CV_64F);  // camera rotation vector of extrinsic parameters. (3,1,CV_64F)
	cv::Mat rmat2(3, 3, CV_64F);  // camera rotation matrix of extrinsic parameters. (3,3,CV_64F)
	cv::Mat tvec2(3, 1, CV_64F);  // camera translation vector of extrinsic parameters. (3,1,CV_64F)

	int nStep; // number of steps to be triangulated 
	int nPoint; // number of points to be triangulated (as maybe a part of points need to be triangulated) 
	int nStepCam1; // number of steps read from points file camera 1 
	int nStepCam2; // number of steps read from points file camera 2 
	int nPointCam1; // number of points read from points file camera 1 
	int nPointCam2;	// number of points read from points file camera 2

	cv::Mat imgPointsCam1; // image points (nPoint,1,CV_32FC2). 
	cv::Mat imgPointsCam2; // image points (nPoint,1,CV_32FC2). 
	cv::Mat triangulatedPoints; // 3d points (nStep, nPoint, CV_64FC3). 
	cv::Mat triangulatedErrors; // projection error of triangulation (unit in pixels) (sum of errors in both images)
	vector<vector<cv::Point2f> > imgPointsCam1_AllSteps;  // image points in Cam 1 of all steps
	vector<vector<cv::Point2f> > imgPointsCam2_AllSteps;	 // image points in Cam 2 of all steps

	string fnameIntrinsic1, fnameExtrinsic1;
	string fnameIntrinsic2, fnameExtrinsic2;
	string fnameImagePoints1; // full-path file name of image points (E.g., c:\path\imgPointsHist1.xml) 
	string fnameImagePoints2; // full-path file name of image points (E.g., c:\path\imgPointsHist2.xml) 
	string fnameSummary;
	vector<int> pointIdsCam1; // points ids to be triangulated in camera 1. Vector length is nPoint
	vector<int> pointIdsCam2; // points ids to be triangulated in camera 2. Vector length is nPoint

	// get camera intrinsic and extrinsic parameters of camera 1
	while (true) {
		std::cout << "  Camera 1 intrinsic file (full path, space allowed, no quotation, 'g' for gui file dialog):\n";
		fnameIntrinsic1 = readStringLineFromIstream(cin);
		if (fnameIntrinsic1.length() == 1 && fnameIntrinsic1[0] == 'g') {
			fnameIntrinsic1 = uigetfile();
			std::cout << fnameIntrinsic1 << endl;
		}
		cv::FileStorage fsIntrinsic1(fnameIntrinsic1, cv::FileStorage::READ);
		if (fsIntrinsic1.isOpened() == false) {
			cerr << "  File does not exist (" << fnameIntrinsic1 << ")\n";
			continue;
		}
		fsIntrinsic1["cameraMatrix"] >> cmat1;
		fsIntrinsic1["distortionVector"] >> dvec1;
		if (cmat1.cols != 3 || cmat1.rows != 3) {
			cerr << "  camera matrix of Cam 1 size is incorrect.\n";
			continue;
		}
		if (dvec1.rows != 1) {
			cerr << "  distortion vector of Cam 1 size is incorrect.\n";
			continue;
		}
		std::cout << "  Camera 1 extrinsic file (full path, space allowed, no quotation, 'g' for gui file dialog):\n";
		fnameExtrinsic1 = readStringLineFromIstream(cin);
		if (fnameExtrinsic1.length() == 1 && fnameExtrinsic1[0] == 'g') {
			fnameExtrinsic1 = uigetfile();
			std::cout << fnameExtrinsic1 << endl;
		}
		cv::FileStorage fsExtrinsic1(fnameExtrinsic1, cv::FileStorage::READ);
		if (fsExtrinsic1.isOpened() == false) {
			cerr << "  File does not exist (" << fnameExtrinsic1 << ")\n";
			continue;
		}
		try {
			fsExtrinsic1["rvec"] >> rvec1;
		}
		catch (...) {
			cv::Vec3d v3; 
			fsExtrinsic1["rvec"] >> v3;
			rvec1 = cv::Mat(v3); 
			if (rvec1.cols == 3 && rvec1.rows == 1)	rvec1 = rvec1.t(); 
		}
		try {
			fsExtrinsic1["tvec"] >> tvec1;
		}
		catch (...) {
			cv::Vec3d v3;
			fsExtrinsic1["tvec"] >> v3;
			tvec1 = cv::Mat(v3);
			if (tvec1.cols == 3 && tvec1.rows == 1)	tvec1 = tvec1.t();
		}
		if (rvec1.cols != 1 || rvec1.rows != 3) {
			cerr << "  rvec size of cam 1 is incorrect.\n";
			continue;
		}
		if (tvec1.cols != 1 || tvec1.rows != 3) {
			cerr << "  tvec size of cam 1 is incorrect.\n";
			continue;
		}
		cv::Rodrigues(rvec1, rmat1);
		break;
	}
	std::cout << "Camera 1 intrinsic parameters are: \n";
	std::cout << "   Cmat:\n" << cmat1 << endl << "  Dist:\n" << dvec1 << endl;
	std::cout << "Camera 1 extrinsic parameters are: \n";
	std::cout << "   Rmat:\n" << rmat1 << endl << "  Tvec:\n" << tvec1 << endl;

	// get camera intrinsic and extrinsic parameters of camera 2
	while (true) {
		std::cout << "  Camera 2 intrinsic file (full path, space allowed, no quotation, 'g' for gui file dialog):\n";
		fnameIntrinsic2 = readStringLineFromIstream(cin);
		if (fnameIntrinsic2.length() == 1 && fnameIntrinsic2[0] == 'g') {
			fnameIntrinsic2 = uigetfile();
			std::cout << fnameIntrinsic2 << endl;
		}
		cv::FileStorage fsIntrinsic2(fnameIntrinsic2, cv::FileStorage::READ);
		if (fsIntrinsic2.isOpened() == false) {
			cerr << "  File does not exist (" << fnameIntrinsic2 << ")\n";
			continue;
		}
		fsIntrinsic2["cameraMatrix"] >> cmat2;
		fsIntrinsic2["distortionVector"] >> dvec2;
		if (cmat2.cols != 3 || cmat2.rows != 3) {
			cerr << "  camera matrix of Cam 2 size is incorrect.\n";
			continue;
		}
		if (dvec2.rows != 1) {
			cerr << "  distortion vector of Cam 2 size is incorrect.\n";
			continue;
		}
		std::cout << "  Camera 2 extrinsic file (full path, space allowed, no quotation, 'g' for gui file dialog):\n";
		fnameExtrinsic2 = readStringLineFromIstream(cin);
		if (fnameExtrinsic2.length() == 1 && fnameExtrinsic2[0] == 'g') {
			fnameExtrinsic2 = uigetfile();
			std::cout << fnameExtrinsic2 << endl;
		}
		cv::FileStorage fsExtrinsic2(fnameExtrinsic2, cv::FileStorage::READ);
		if (fsExtrinsic2.isOpened() == false) {
			cerr << "  File does not exist (" << fnameExtrinsic2 << ")\n";
			continue;
		}
		try {
			fsExtrinsic2["rvec"] >> rvec2;
		}
		catch (...) {
			cv::Vec3d v3;
			fsExtrinsic2["rvec"] >> v3;
			rvec2 = cv::Mat(v3);
			if (rvec2.cols == 3 && rvec2.rows == 1)	rvec2 = rvec2.t();
		}
		try {
			fsExtrinsic2["tvec"] >> tvec2;
		}
		catch (...) {
			cv::Vec3d v3;
			fsExtrinsic2["tvec"] >> v3;
			tvec2 = cv::Mat(v3);
			if (tvec2.cols == 3 && tvec2.rows == 1)	tvec2 = tvec2.t();
		}
		if (rvec2.cols != 1 || rvec2.rows != 3) {
			cerr << "  rvec size of cam 2 is incorrect.\n";
			continue;
		}
		if (tvec2.cols != 1 || tvec2.rows != 3) {
			cerr << "  tvec size of cam 2 is incorrect.\n";
			continue;
		}
		cv::Rodrigues(rvec2, rmat2);
		break;
	}
	std::cout << "Camera 2 intrinsic parameters are: \n";
	std::cout << "   Cmat:\n" << cmat2 << endl << "  Dist:\n" << dvec2 << endl;
	std::cout << "Camera 2 extrinsic parameters are: \n";
	std::cout << "   Rmat:\n" << rmat2 << endl << "  Tvec:\n" << tvec2 << endl;

	// Get image points of camera 1
	while (true) {
		std::cout << "  Full path of image points file of cam 1 (space allowed, no quotation) (E.g., c:\\path\\imgPointsHistoryCam1.xml):\n";
		fnameImagePoints1 = readStringLineFromIstream(cin);
		cv::FileStorage fsImgPoints1(fnameImagePoints1, cv::FileStorage::READ);
		if (fsImgPoints1.isOpened() == false) {
			cerr << "  File does not exist (" << fnameImagePoints1 << ")\n";
			continue;
		}
		fsImgPoints1["numSteps"] >> nStepCam1;
		fsImgPoints1["numPoints"] >> nPointCam1;
		fsImgPoints1["VecVecPoint2f"] >> imgPointsCam1_AllSteps;
		nStep = nStepCam1;
		// check dimension of read data
		if (nStepCam1 != imgPointsCam1_AllSteps.size()) {
			cerr << "Error: Number of steps is " << nStep << " but only "
				<< imgPointsCam1_AllSteps.size() << " steps are read from file " << fnameImagePoints1 << endl;
			continue; 
		}
		if (nPointCam1 != imgPointsCam1_AllSteps[0].size()) {
			cerr << "Error: Number of points is " << nPointCam1 << " but only "
				<< imgPointsCam1_AllSteps[0].size() << " points are read from file " << fnameImagePoints1 << endl;
			continue;
		}
		fsImgPoints1.release();
		break;
	}

	// Get image points of camera 2
	while (true) {
		std::cout << "  Full path of image points file of cam 2 (space allowed, no quotation) (E.g., c:\\path\\imgPointsHistoryCam2.xml):\n";
		fnameImagePoints2 = readStringLineFromIstream(cin);
		cv::FileStorage fsImgPoints2(fnameImagePoints2, cv::FileStorage::READ);
		if (fsImgPoints2.isOpened() == false) {
			cerr << "  File does not exist (" << fnameImagePoints2 << ")\n";
			continue;
		}
		fsImgPoints2["numSteps"] >> nStepCam2;
		fsImgPoints2["numPoints"] >> nPointCam2;
		fsImgPoints2["VecVecPoint2f"] >> imgPointsCam2_AllSteps;
		// check dimension of read data
		if (nStepCam2 != imgPointsCam2_AllSteps.size()) {
			cerr << "Error: Number of steps is " << nStep << " but only "
				<< imgPointsCam2_AllSteps.size() << " steps are read from file " << fnameImagePoints2 << endl;
			continue;
		}
		if (nPointCam2 != imgPointsCam2_AllSteps[0].size()) {
			cerr << "Error: Number of points is " << nPointCam2 << " but only "
				<< imgPointsCam2_AllSteps[0].size() << " points are read from file " << fnameImagePoints2 << endl;
			continue;
		}
		fsImgPoints2.release();
		break;
	}

	// check nStep
	if (nStepCam1 != nStepCam2) {
		std::cout << "Warning: number of steps in cam 1 (" << nStepCam1
			<< " does not match with cam 2 (" << nStepCam2 << ").\n";
		std::cout << "How many steps do you want to triangulat? \n";
		nStep = readIntFromCin(); 
	}

	// ask which points are to be triangulated
	std::cout << "Camera 1 has " << nPointCam1 << " points.\n";
	std::cout << "Camera 2 has " << nPointCam2 << " points.\n";
	std::cout << "How many points do you want to triangulate? \n";
	nPoint = readIntFromCin();
	pointIdsCam1.resize(nPoint);
	pointIdsCam2.resize(nPoint);
	std::cout << "Input point index (1-based) in camera 1 to triangulate:\n";
	for (int i = 0; i < nPoint; i++)
		pointIdsCam1[i] = readIntFromCin();
	std::cout << "Input point index (1-based) in camera 2 to triangulate:\n";
	for (int i = 0; i < nPoint; i++)
		pointIdsCam2[i] = readIntFromCin();
	imgPointsCam1 = cv::Mat::zeros(nPoint, 1, CV_32FC2); 
	imgPointsCam2 = cv::Mat::zeros(nPoint, 1, CV_32FC2);

	// Output file 
	std::cout << "  Full path file of triangulation result (space allowed, no quotation) (E.g., c:\\path\\triangulatedPoints_AllSteps.xml):\n";
	fnameSummary = readStringLineFromIstream(cin);
	
	// main loop
	int64 tickCountStart = cv::getTickCount();
	for (int iStep = 0; iStep < nStep; iStep++) {
		// build cv::Mat that points to correct cv::Point2f vector
		for (int iPoint = 0; iPoint < nPoint; iPoint++) {
			imgPointsCam1.at<cv::Point2f>(iPoint, 0) = imgPointsCam1_AllSteps[iStep][pointIdsCam1[iPoint] - 1]; 
			imgPointsCam2.at<cv::Point2f>(iPoint, 0) = imgPointsCam2_AllSteps[iStep][pointIdsCam2[iPoint] - 1];
			// imgPointsCam1 = cv::Mat((int)nPoint, 1, CV_32FC2, imgPointsCam1_AllSteps[iStep].data());
			// imgPointsCam2 = cv::Mat((int)nPoint, 1, CV_32FC2, imgPointsCam2_AllSteps[iStep].data());
		}

		// triangulation 
		int triangRet; 
		cv::Mat points3d, err; 
//		std::cout << "Cmat1: \n" << cmat1 << endl;
//		std::cout << "Dvec1: \n" << dvec1 << endl;
//		std::cout << "Rmat1: \n" << rmat1 << endl;
//		std::cout << "Tvec1: \n" << tvec1 << endl;
//		std::cout << "Cmat2: \n" << cmat2 << endl;
//		std::cout << "Dvec2: \n" << dvec2 << endl;
//		std::cout << "Rmat2: \n" << rmat2 << endl;
//		std::cout << "Tvec2: \n" << tvec2 << endl;
//		std::cout << "Img1: \n"  << imgPointsCam1 << endl;
//		std::cout << "Img2: \n" << imgPointsCam2 << endl;

		triangRet = triangulatePointsGlobalCoordinate(
			cmat1, dvec1, rmat1, tvec1, 
			cmat2, dvec2, rmat2, tvec2, 
			imgPointsCam1, imgPointsCam2, 
			points3d,
			err
		);
//		std::cout << "points3d = " << points3d << endl;
//		std::cout << "err = " << err << endl;

		// copy points3d to triangulatedPoints
		if (triangulatedPoints.rows < nStep || triangulatedPoints.cols < nPoint)
			triangulatedPoints = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
		if (triangulatedErrors.rows < nStep || triangulatedErrors.cols < nPoint)
			triangulatedErrors = cv::Mat::zeros(nStep, nPoint, CV_64F);

		for (int iPoint = 0; iPoint < nPoint; iPoint++) {
			triangulatedPoints.at<cv::Point3d>(iStep, iPoint).x = points3d.at<double>(iPoint, 0);
			triangulatedPoints.at<cv::Point3d>(iStep, iPoint).y = points3d.at<double>(iPoint, 1);
			triangulatedPoints.at<cv::Point3d>(iStep, iPoint).z = points3d.at<double>(iPoint, 2);
			triangulatedErrors.at<double>(iStep, iPoint) = err.at<double>(0, iPoint);
			//			cout << points3d.at<double>(iPoint, 0) << endl;
//			cout << triangulatedPoints.at<cv::Point3d>(iStep, iPoint) << endl;
		}
		// Print info
		if (iStep % 10 == 0) {
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b"
				"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
			int64 tickCountFrameEnd = cv::getTickCount();
			float timePast = (float)((tickCountFrameEnd - tickCountStart) / cv::getTickFrequency());
			float timeTotalEstimated = timePast / ((iStep + 1) * 1.0f / nStep);
			float timeReamining = timeTotalEstimated - timePast;
			printf("Step %06d/%06d. Time remaining: %6d sec.", iStep, nStep, (int)(timeReamining + .5f));
		}

	} // end of while 

	// output summary
	cv::FileStorage ofsSummary(fnameSummary, cv::FileStorage::WRITE);
	ofsSummary << "numSteps" << nStep;
	ofsSummary << "numPoints" << nPoint;
	ofsSummary << "PointsHistory" << triangulatedPoints; 
	ofsSummary << "ProjectionErr" << triangulatedErrors; 
	ofsSummary.release();

	// output matlab script
	string fnameTriangPointsMatlab = extFilenameRemoved(fnameSummary) + ".m";
	ofstream ofMat(fnameTriangPointsMatlab);
	ofMat << "PointsHistory = " << triangulatedPoints << ";" << endl;
	ofMat << "PointsHistory = reshape(PointsHistory', [ 3 "
		<< triangulatedPoints.cols << " " // .cols is nPoint
		<< triangulatedPoints.rows << " ]);\n "; // .rows is nStep
	ofMat << "ProjectionErr = " << triangulatedErrors.t() << ";" << endl;
	for (int i = 0; i < nPoint; i++) {
		ofMat << "fps = 29.97; % modify this number by yourself.\n";
		ofMat << "figure('name', 'Disp. of Point " << i + 1 << "');\n";
		ofMat << "xt(:) = (0:" << nStep - 1 << ")/fps; ";
		ofMat << "yt(:) = PointsHistory(1, " << i + 1 << ", :) - PointsHistory(1, " << i + 1 << ", 1); \n";
		ofMat << "plot(xt,yt);\n";
		ofMat << "hold on;\n";
		ofMat << "yt(:) = PointsHistory(2, " << i + 1 << ", :) - PointsHistory(2, " << i + 1 << ", 1); \n";
		ofMat << "plot(xt,yt);\n";
		ofMat << "yt(:) = PointsHistory(3, " << i + 1 << ", :) - PointsHistory(3, " << i + 1 << ", 1); \n";
		ofMat << "plot(xt,yt);\n";
		ofMat << "grid on; xlabel('Time (sec.)'); ylabel('Triangulated position'); \n";
		ofMat << "legend('ux', 'uy', 'uz'); \n";
	}
	ofMat.close();

	return 0; 
}