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
// Step 10: Left extrinsic calibration with user coordinate
// Step 11: Right extrinsic calibration with user coordinate

const String keys =
"{help          h usage ? |   | print this message   }"
"{caliblevel1   callevel1 |   | intrinsic parameters level 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.  } "
"{caliblevel2   callevel2 |   | intrinsic parameters level 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.  } "
"{calfilexml1   calfx1    |   | path and file name intr/extr parameters of camera 1 }"
"{calfilexml2   calfx2    |   | path and file name intr/extr parameters of camera 2 }"
"{calfileimpm1  calfpm1   |   | path and file name of matlab (octave) scirpt file for camera 1 image points }"
"{calfileimpm2  calfpm2   |   | path and file name of matlab (octave) scirpt file for camera 2 image points }"
"{calfileuimpx1 calfuimpx1 |  | path and xml file name of user defined image points for camera 1 }"
"{calfileuimpx2 calfuimpx2 |  | path and xml file name of user defined image points for camera 2 }"
"{calfileu3dpx1 calfu3dpx1 |  | path and xml file name of user defined image points for camera 1 }"
"{calfileu3dpx2 calfu3dpx2 |  | path and xml file name of user defined image points for camera 2 }"

;


int FuncCalibOnSiteUserPoints(int argc, char** argv)
{
	string calfuimpx1; // path and xml file name of user defined image points for camera 1 
	string calfuimpx2; // path and xml file name of user defined image points for camera 2 
	string calfu3dpx1; // path and xml file name of user defined image points for camera 1 
	string calfu3dpx2; // path and xml file name of user defined image points for camera 2 
	int callevel1; // calibration level. 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.
	int callevel2; // calibration level. 0:auto, 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.
	string calfx1; // path and file name of calibration result xml file of camera 1 
	string calfx2; // path and file name of calibration result xml file of camera 2
	string calfpm1; // path and file name of output matlab (octave) scirpt file for camera 1 image points 
	string calfpm2; // path and file name of output matlab (octave) scirpt file for camera 2 image points 

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

	IntrinsicCalibrator calC1;
	IntrinsicCalibrator calC2;

	// Step 0: Build parser 
	cv::CommandLineParser parser(argc, argv, keys);

	// Step 1: Left intrinsic/extrinsic calibration with user coordinate, all in one
	Points2fHistoryData pointsUserDefinedC1;
	Points3dHistoryData points3dUserDefinedC1;
	// image points of known points (file calfuimpx1) --> pointsUserDefined
	if (parser.has("calfuimpx1")) {
		calfuimpx1 = parser.get<string>("calfuimpx1");
		pointsUserDefinedC1.readFromXml(calfuimpx1);
	}
	else
	{
		cout << "For left cam (cam 1) extrinsic calibration, how do you want to define the image points of "
			"known points for extrinsic parameters?\n";
		pointsUserDefinedC1.readThruUserInteraction(1 /* nStep */, -1 /* nPoint unknown */);
	}
	// 3d points of known points (file calfu3dpx1) --> points3dUserDefined
	if (parser.has("calfu3dpx1")) {
		calfu3dpx1 = parser.get<string>("calfu3dpx1");
		points3dUserDefinedC1.readFromXml(calfu3dpx1);
	}
	else
	{
		cout << "How do you want to define the 3d points of "
			"known points for extrinsic parameters?\n";
		points3dUserDefinedC1.readThruUserInteraction(1 /* nStep */, pointsUserDefinedC1.nPoint());
	}
	// intrinsic/extrinsic for left cam
	calC1.defineUserPoints(0, pointsUserDefinedC1.getVecVec()[0], points3dUserDefinedC1.getVecVec()[0]);

	// Step 2: Intrinsic / extrinsic calibration all in one
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

	// Step 3: Write to xml file
	if (parser.has("calfx1")) {
		calfx1 = parser.get<string>("calfx1");
	}
	else {
		cout << "Enter path and file name intr/extr parameters of camera 1:\n";
		calfx1 = readStringLineFromCin();
	}
	calC1.writeToFsFile(calfx1);

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

	// Step 4: Right intrinsic/extrinsic calibration with user coordinate, all in one
	Points2fHistoryData pointsUserDefinedC2;
	Points3dHistoryData points3dUserDefinedC2;
	// image points of known points (file calfuimpx2) --> pointsUserDefinedC2
	if (parser.has("calfuimpx2")) {
		calfuimpx2 = parser.get<string>("calfuimpx2");
		pointsUserDefinedC2.readFromXml(calfuimpx2);
	}
	else
	{
		cout << "For right cam (cam 2) extrinsic calibration, how do you want to define the image points of "
			"known points for extrinsic parameters?\n";
		pointsUserDefinedC2.readThruUserInteraction(1 /* nStep */, -1 /* nPoint unknown */);
	}
	// 3d points of known points (file calfu3dpx2) --> points3dUserDefinedC2
	if (parser.has("calfu3dpx2")) {
		calfu3dpx2 = parser.get<string>("calfu3dpx2");
		points3dUserDefinedC2.readFromXml(calfu3dpx2);
	}
	else
	{
		cout << "How do you want to define the 3d points of "
			"known points for extrinsic parameters?\n";
		points3dUserDefinedC2.readThruUserInteraction(1 /* nStep */, pointsUserDefinedC2.nPoint());
	}
	// intrinsic/extrinsic for right cam
	calC2.defineUserPoints(0, pointsUserDefinedC2.getVecVec()[0], points3dUserDefinedC2.getVecVec()[0]);

	// Step 5: Intrinsic / extrinsic calibration all in one
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

	// Step 3: Write to xml file
	if (parser.has("calfx2")) {
		calfx2 = parser.get<string>("calfx2");
	}
	else {
		cout << "Enter path and file name intr/extr parameters of camera 2:\n";
		calfx2 = readStringLineFromCin();
	}
	calC2.writeToFsFile(calfx2);

	// output to matlab script
	if (parser.has("calfpm2"))
		calfpm2 = parser.get<string>("calfpm2");
	else
	{
		cout << "Input path and file name of output matlab (octave) scirpt file "
			"for camera 2 image points  (calfpm2=):\n";
		calfpm2 = readStringLineFromCin();
	}
	calC2.writeToMscript(calfpm2);

	return 0;
}