#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "ImagePointsPicker.h"

#include "FileSeq.h"
#include "IntrinsicCalibrator.h"

#include "Points2fHistoryData.h"
#include "Points3dHistoryData.h"

// Step 1: Calibration info: 
//         bType:  calibration board type:  1:chessboard, 2.grid(sym), 3.grid(unsym), 
//         nx ny:  number of points along horizontal and vertical directions,
//         dx dy:  grid distances (square size) along horizontal and vertical directions. 
// Step 2: Left cam calibration photos: (FileSeq)
//         (ask user. FileSeq options: o, c, f, g, gm) 
// Step 3: Left find corners 
// Step 4: Left calibration level for options (output .m scripts)
// Step 5: Left output result 
// Step 6: Right cam calibration photos: (FileSeq)
//         (ask user. FileSeq options: o, c, f, g, gm) 
// Step 7: Right find corners 
// Step 8: Right calibration level for options (output .m scripts)
// Step 9: Right output result 
// Step 10: Stereo calibration (between cameras 1 and 2) 
// Step 11: Extrinsic calibration with user coordinate


const String keys =
"{help          h usage ? |   | print this message   }"
"{calbtype      calbt     |   | calibration board type: 1:chessboard,2:sym.grid,3:unsym.grid. }"
"{calbnx        calbnx    |   | number of corners along board x. Standard chessboard is 7. }"
"{calbny        calbny    |   | number of corners along board y. Standard chessboard is 7. }"
"{calbsx        calbsx    |   | distance between corners along board x. Standard chessboard is 57.15 mm. Unit is arbitrary but be consistent.}"
"{calbsy        calbsy    |   | distance between corners along board y. Standard chessboard is 57.15 mm. Unit is arbitrary but be consistent.}"
"{calflist1     calfs1    |   | path and file name of list of calibration photos of camera 1 (left) } "
"{calflist2     calfs2    |   | path and file name of list of calibration photos of camera 2 (right) } "
"{caliblevel1   callevel1 |   | intrinsic parameters level 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.  } "
"{caliblevel2   callevel2 |   | intrinsic parameters level 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.  } "
"{calfilexml1   calfx1    |   | path and file name intr/extr parameters of camera 1 }"
"{calfilexml2   calfx2    |   | path and file name intr/extr parameters of camera 2 }"
"{calfileimpm1  calfpm1   |   | path and file name of matlab (octave) scirpt file for camera 1 image points }"
"{calfileimpm2  calfpm2   |   | path and file name of matlab (octave) scirpt file for camera 2 image points }"
"{calextrcam    calexcam  |   | camera id (1 or 2) having known points and to be used to calculate extrinsic parameters }"
"{calfileuimpx1 calfuimpx1 |  | path and xml file name of user defined image points for camera 1 }"
"{calfileuimpx2 calfuimpx2 |  | path and xml file name of user defined image points for camera 2 }"
"{calfileu3dpx1 calfu3dpx1 |  | path and xml file name of user defined image points for camera 1 }"
"{calfileu3dpx2 calfu3dpx2 |  | path and xml file name of user defined image points for camera 2 }"

;


int FuncCalibStereoOnSite(int argc, char** argv)
{
	// Step 0: Variable declaration and parser
	//
	int calbt; // calibration board type:  1:chessboard, 2.grid(sym), 3.grid(unsym)
	int calbnx; // number of points along horizontal directions. Standard chessboard (8x8) is 7 (7x7 corners)  
	int calbny; // number of points along horizontal directions. Standard chessboard (8x8) is 7 (7x7 corners)
	double calbsx; // distances between corners along horizontal directions. Use unit you preferred (mm, m, inch, ft, ..., just be consistent). 
	double calbsy; // distances between corners along vertical directions. Use unit you preferred (mm, m, inch, ft, ..., just be consistent). 
	string calfs1; // camera 1 (left) calibration photos files list file
	string calfs2; // camera 2 (right) calibration photos files list file
	int callevel1; // calibration level. 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.
	int callevel2; // calibration level. 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.
	string calfx1; // path and file name of calibration result xml file of camera 1 
	string calfx2; // path and file name of calibration result xml file of camera 2
	string calfpm1; // path and file name of output matlab (octave) scirpt file for camera 1 image points 
	string calfpm2; // path and file name of output matlab (octave) scirpt file for camera 2 image points 
	string calfuimpx1; // path and xml file name of user defined image points for camera 1 
	string calfuimpx2; // path and xml file name of user defined image points for camera 2 
	string calfu3dpx1; // path and xml file name of user defined image points for camera 1 
	string calfu3dpx2; // path and xml file name of user defined image points for camera 2 

	FileSeq fsqCalFiles1;  // left camera calibration photos files sequence
	FileSeq fsqCalFiles2;  // right camera calibration photos  

	Points2fHistoryData calPoints1; // left camera image points (corners) for calibration
	Points2fHistoryData calPoints2; // right camera image points (corners) for calibration
	Points3dHistoryData calPoints3d1; // left camera object points (corners) for calibration
	Points3dHistoryData calPoints3d2; // right camera object points (corners) for calibration
	Points2fHistoryData calPointsPrj1; // left camera projected image points (corners) for calibration
	Points2fHistoryData calPointsPrj2; // right camera projected image points (corners) for calibration

	cv::Mat R44 = cv::Mat::eye(4, 4, CV_64F); // R matrix (4 by 4) of stereo calibration
	cv::Mat R33(R44(cv::Rect(0, 0, 3, 3))); // R matrix (3 by 3) of stereo calibration
	cv::Mat T31(R44(cv::Rect(3, 0, 1, 3))); // T vector (3 by 1) of stereo calibration 
	cv::Mat Emat, Fmat; // 
	cv::Mat R44C1 = cv::Mat::eye(4, 4, CV_64F); // R matrix (4 by 4) of camera 1 (wrt user-defined coord.)
	cv::Mat R33C1(R44C1(cv::Rect(0, 0, 3, 3))); // R matrix (3 by 3) of stereo calibration of camera 1 (wrt user-defined coord.)
	cv::Mat T31C1(R44C1(cv::Rect(3, 0, 1, 3))); // T vector (3 by 1) of stereo calibration of camera 1 (wrt user-defined coord.)
	cv::Mat R44C2 = cv::Mat::eye(4, 4, CV_64F); // R matrix (4 by 4) of camera 1 (wrt user-defined coord.)
	cv::Mat R33C2(R44C2(cv::Rect(0, 0, 3, 3))); // R matrix (3 by 3) of stereo calibration of camera 1 (wrt user-defined coord.)
	cv::Mat T31C2(R44C2(cv::Rect(3, 0, 1, 3))); // T vector (3 by 1) of stereo calibration of camera 1 (wrt user-defined coord.)

	int calexcam;  // camera id (1 or 2) for extrinsic parameters wrt user's coordinate system

	cv::CommandLineParser parser(argc, argv, keys); 

	// Step 1: Calibration board info 
	// calbt
	if (parser.has("calbt"))
		calbt = parser.get<int>("calbt"); 
	else
	{
		cout << "Input calibration board type (-calbt=): 1:chessboard,2:sym.grid,3:unsym.grid.:\n";
		calbt = readIntFromCin();
	}
	// calbnx
	if (parser.has("calbnx"))
		calbnx = parser.get<int>("calbnx");
	else
	{
		cout << "Input number of points along horizontal directions (-calbnx=). Standard chessboard (8x8) is 7 (7x7 corners):\n";
		calbnx = readIntFromCin();
	}
	// calbny
	if (parser.has("calbny"))
		calbny = parser.get<int>("calbny");
	else
	{
		cout << "Input number of points along vertical directions (-calbny=). Standard chessboard (8x8) is 7 (7x7 corners):\n";
		calbny = readIntFromCin();
	}
	// calbsx
	if (parser.has("calbsx"))
		calbsx = parser.get<int>("calbsx");
	else
	{
		cout << "Input distances between corners along horizontal directions (-calbsx=). Use unit you prefer (mm, m, inch, ft, ..., needs be consistent):\n";
		calbsx = readDoubleFromCin();
	}
	// calbsy
	if (parser.has("calbsy"))
		calbsy = parser.get<int>("calbsy");
	else
	{
		cout << "Input  distances between corners along vertical directions (-calbsy). Use unit you prefer (mm, m, inch, ft, ..., needs be consistent):\n";
		calbsy = readDoubleFromCin();
	}

	// Step 2: Left cam calibration photos 
	// calfs1
	FileSeq fsq1;
	IntrinsicCalibrator calC1;
	if (parser.has("callevel1"))
		callevel1 = parser.get<int>("callevel1");
	if (parser.has("calfs1")) {
		FileSeq fsq1; 
		calfs1 = parser.get<int>("calfs1");
		fsq1.setFilesByListFile(calfs1);
		calC1.setFileSeq(fsq1); 
	}
	else
	{
		cout << "Input camera 1 (left) calibration photos files:\n";
		fsq1.setDirFilesByConsole(); 
		calC1.setFileSeq(fsq1);
	}

	// Step 3: Find corners
	calPoints1.resize(calC1.fileSeq().num_files(), calbnx * calbny);
	calC1.setCalibrationBoard(calbt, calbnx, calbny, calbsx, calbsy); 
	for (int i = 0; i < calC1.fileSeq().num_files(); i++) {
		cout << "   " << calC1.fileSeq().filename(i) << ": ";
		int foundOK = calC1.findCorners(i, cv::Size(calbnx, calbny), (float) calbsx, (float) calbsy, calbt);
		if (foundOK == 0)
			cout << "Corners found successfully.\n";
		else
			cout << "Failed to find corners in this photo.\n";
	}


	// Step 4: calibration
	int flag1 = 0; 
	// callevel1
	if (parser.has("callevel1"))
		callevel1 = parser.get<int>("callevel1");
	else
	{
		cout << "Input calibration level for camera 1 (callevel1=). "
			"0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, "
			"4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.:\n";
		callevel1 = readIntFromCin();
	}
	if (callevel1 == 0) callevel1 = 3; // set to 3 (fx,fy,cx,cy,k1) by default 
	calC1.calibrateByLevel(callevel1);

	// Step 5: Write to xml file
	if (parser.has("calfx1")) {
		calfx1 = parser.get<string>("calfx1"); 
	}
	else {
		cout << "Enter path and file name intr/extr parameters of camera 1:\n";
		calfx1 = readStringLineFromCin();
	}
	calC1.writeToFsFile(calfx1); 

	// Step 6: Right cam calibration photos 
	// calfs2
	IntrinsicCalibrator calC2;
	if (parser.has("calfs2")) {
		FileSeq fsq2;
		calfs2 = parser.get<int>("calfs2");
		fsq2.setFilesByListFile(calfs2);
		calC2.setFileSeq(fsq2);
	}
	else
	{
		cout << "Input camera 2 (right) calibration photos files:\n";
		FileSeq fsq2;
		fsq2.setDirFilesByConsole();
		calC2.setFileSeq(fsq2);
	}

	// Step 7: Find corners
	calPoints2.resize(calC2.fileSeq().num_files(), calbnx * calbny);
	calC2.setCalibrationBoard(calbt, calbnx, calbny, calbsx, calbsy);
	for (int i = 0; i < calC2.fileSeq().num_files(); i++) {
		cout << "   " << calC2.fileSeq().filename(i) << ": ";
		int foundOK = calC2.findCorners(i, cv::Size(calbnx, calbny), (float) calbsx, (float) calbsy, calbt);
		if (foundOK == 0)
			cout << "Corners found successfully.\n";
		else
			cout << "Failed to find corners in this photo.\n";
	}
	
	// Step 8: calibration
	int flag2 = 0;
	// callevel2
	if (parser.has("callevel2"))
		callevel2 = parser.get<int>("callevel2");
	else
	{
		cout << "Input calibration level for camera 2 (callevel2=). "
			"0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, "
			"4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.:\n";
		callevel2 = readIntFromCin();
	}
	if (callevel2 == 0) callevel2 = 3; // set to 3 (fx,fy,cx,cy,k1) by default 
	calC2.calibrateByLevel(callevel2); 

	// Step 9: Write C2 intrinsic to xml file
	if (parser.has("calfx2")) {
		calfx2 = parser.get<string>("calfx2");
	}
	else {
		cout << "Enter path and file name intr/extr parameters of camera 2:\n";
		calfx2 = readStringLineFromCin();
	}
	calC2.writeToFsFile(calfx2);

	// Step 10: Stereo calibration 
	// Reminding message
	cout << "Note: You need to make sure two cameras have consistent mapping. That means: \n"
		"   1. Both cameras have the same number of calibration photos.\n"
		"   2. Each pair (C1 and C2) of calibratino photos were taken at the same time, which calibration board was at the same position.\n"
		"   3. Each detected corner of one photo is the same point of that in the other photo (and corners are the same ordering).\n"
		"   4. All calibration photos of both cameras have the same image size (width and height).\n"
		"   5. This program may give you totally wrong stereo calibration result if any if above requirement is not satisfied.\n";
	// check 
	if (calC1.numValidPhotos() != calC2.numValidPhotos()) 
	{
		cerr << "Error: For stereo calibration, C1 and C2 must have the same number of valid calibration photos.\n";
		return -1; 
	} 
	// actually more checks should be done. 
	cv::stereoCalibrate(
		calC1.objPoints(), 
		calC1.imgPoints(),
		calC2.imgPoints(),
		calC1.cameraMatrix(), calC1.distortionVector(),
		calC2.cameraMatrix(), calC2.distortionVector(), calC1.imageSize(), 
		R33, T31, Emat, Fmat); 

	cout << "R33:\n" << R33 << endl;
	cout << "T31:\n" << T31 << endl;
	cout << "R44:\n" << R44 << endl;
	   	 
	// Step 11: Extrinsic calibration with user coordinate
	// 
	if (parser.has("calexcam")) {
		calexcam = parser.get<int>("calexcam"); 
	}
	else {
		cout << "Which camera do you want to use to define user coordinate system (1 or 2):\n";
		calexcam = readIntFromCin(1, 2);
	}

	Points2fHistoryData pointsUserDefined;
	Points3dHistoryData points3dUserDefined;
	if (calexcam == 1) {
		// image points of known points (file calfuimpx1) --> pointsUserDefined
		if (parser.has("calfuimpx1")) {
			calfuimpx1 = parser.get<string>("calfuimpx1");
			pointsUserDefined.readFromXml(calfuimpx1);
		}
		else
		{
			cout << "How do you want to define the image points of "
				"known points for extrinsic parameters?\n";
			pointsUserDefined.readThruUserInteraction(1 /* nStep */, -1 /* nPoint unknown */);
		}
		// 3d points of known points (file calfu3dpx1) --> points3dUserDefined
		if (parser.has("calfu3dpx1")) {
			calfu3dpx1 = parser.get<string>("calfu3dpx1");
			points3dUserDefined.readFromXml(calfu3dpx1);
		}
		else
		{
			cout << "How do you want to define the 3d points of "
				"known points for extrinsic parameters?\n";
			points3dUserDefined.readThruUserInteraction(1 /* nStep */, pointsUserDefined.nPoint());
		}
	}
	else { // calexcam == 2
		// image points of known points (file calfuimpx2) --> pointsUserDefined
		if (parser.has("calfuimpx2")) {
			calfuimpx2 = parser.get<string>("calfuimpx2");
			pointsUserDefined.readFromXml(calfuimpx2);
		}
		else
		{
			cout << "How do you want to define the image points of "
				"known points for extrinsic parameters?\n";
			pointsUserDefined.readThruUserInteraction(1 /* nStep */, -1 /* nPoint unknown */);
		}
		// 3d points of known points (file calfu3dpx2) --> points3dUserDefined
		if (parser.has("calfu3dpx2")) {
			calfu3dpx2 = parser.get<string>("calfu3dpx2");
			points3dUserDefined.readFromXml(calfu3dpx2);
		}
		else
		{
			cout << "How do you want to define the 3d points of "
				"known points for extrinsic parameters?\n";
			points3dUserDefined.readThruUserInteraction(1 /* nStep */, pointsUserDefined.nPoint());
		}
	}

	if (calexcam == 1) {
		// estimate camera extrinsic rvec/tvec
		cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 1 for extrinsic calibration wrt user coord.
		cv::Mat tvec1 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 1 for extrinsic calibration wrt user coord.
		cv::Mat rvec33 = cv::Mat::zeros(3, 3, CV_64F); 
		cv::Mat rvec2 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 2 for extrinsic calibration wrt user coord.
		cv::solvePnP(points3dUserDefined.getVecVec()[0], pointsUserDefined.getVecVec()[0],
			calC1.cameraMatrix(), calC1.distortionVector(), rvec1, tvec1);
		// convert to rvec/tvec of the other cam
		cv::Rodrigues(rvec1, rvec33);
		rvec33.copyTo(R44C1(cv::Rect(0,0,3,3)));
		tvec1.copyTo(R44C1(cv::Rect(3, 0, 1, 3))); 
		R44C2 = R44 * R44C1;
		// check
//		cout << "R44C1: \n" << R44C1 << endl;
//		cout << "R44C2: \n" << R44C2 << endl;
//		cout << "R44C2 * R44C1^-1: \n" << R44C2 * R44C1.inv() << endl;
//		cout << "R44: \n" << R44 << endl;
		// append to file 
		cv::FileStorage fsCalFx1(calfx1, cv::FileStorage::APPEND);
		fsCalFx1 << "rvec" << rvec1;
		fsCalFx1 << "tvec" << R44C1(cv::Rect(3, 0, 1, 3));
		fsCalFx1 << "R4" << R44C1;
		fsCalFx1 << "R4inv" << R44C1.inv();
		fsCalFx1.release(); 
		cv::FileStorage fsCalFx2(calfx2, cv::FileStorage::APPEND);
		cv::Rodrigues(R44C2(cv::Rect(0, 0, 3, 3)), rvec2);
		fsCalFx2 << "rvec" << rvec2;
		fsCalFx2 << "tvec" << R44C2(cv::Rect(3, 0, 1, 3));
		fsCalFx2 << "R4" << R44C2;
		fsCalFx2 << "R4inv" << R44C2.inv();
		fsCalFx2.release();
	}
	else { // calexcam == 2 
		// estimate camera extrinsic rvec/tvec
		cv::Mat rvec2 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 1 for extrinsic calibration wrt user coord.
		cv::Mat tvec2 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 1 for extrinsic calibration wrt user coord.
		cv::Mat rvec33 = cv::Mat::zeros(3, 3, CV_64F);
		cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64F); // rvec of camera 2 for extrinsic calibration wrt user coord.
		cv::solvePnP(points3dUserDefined.getVecVec()[0], pointsUserDefined.getVecVec()[0],
			calC2.cameraMatrix(), calC2.distortionVector(), rvec2, tvec2);
		// convert to rvec/tvec of the other cam
		cv::Rodrigues(rvec2, rvec33);
		rvec33.copyTo(R44C2(cv::Rect(0, 0, 3, 3)));
		tvec2.copyTo(R44C2(cv::Rect(3, 0, 1, 3)));
		R44C1 = R44.inv() * R44C2;
		// check
//		cout << "R44C1: \n" << R44C1 << endl;
//		cout << "R44C2: \n" << R44C2 << endl;
//		cout << "R44C2 * R44C1^-1: \n" << R44C2 * R44C1.inv() << endl;
//		cout << "R44: \n" << R44 << endl;
		// append to file 
		cv::FileStorage fsCalFx1(calfx1, cv::FileStorage::APPEND);
		fsCalFx1 << "rvec" << rvec1;
		fsCalFx1 << "tvec" << R44C1(cv::Rect(3, 0, 1, 3));
		fsCalFx1 << "R4" << R44C1;
		fsCalFx1 << "R4inv" << R44C1.inv();
		fsCalFx1.release();
		cv::FileStorage fsCalFx2(calfx2, cv::FileStorage::APPEND);
		cv::Rodrigues(R44C2(cv::Rect(0, 0, 3, 3)), rvec2);
		fsCalFx2 << "rvec" << rvec2;
		fsCalFx2 << "tvec" << R44C2(cv::Rect(3, 0, 1, 3));
		fsCalFx2 << "R4" << R44C2;
		fsCalFx2 << "R4inv" << R44C2.inv();
		fsCalFx2.release();
	}

	// output to matlab script
	if (parser.has("calfpm1"))
		calfpm1 = parser.get<string>("calfpm1");
	else
	{
		cout << "Input path and file name of output matlab (octave) scirpt file "
			"for camera 1 image points  (calfpm1=):\n";
		calfpm1 = readStringLineFromCin();
	}
	calC1.writeToMscript(calfpm1); 
	if (parser.has("calfpm2"))
		calfpm2 = parser.get<string>("calfpm21");
	else
	{
		cout << "Input path and file name of output matlab (octave) scirpt file "
			"for camera 2 image points  (calfpm2=):\n";
		calfpm2 = readStringLineFromCin();
	}
	calC2.writeToMscript(calfpm2);
	return 0;
}