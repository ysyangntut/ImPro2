#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include "triangulatepoints2.h"
#include "FileSeq.h"
#include "Points2fHistoryData.h"
#include "Points3dHistoryData.h"
#include "matchTemplateWithRotPyr.h"
#include "ImagePointsPicker.h"
#include "enhancedCorrelationWithReference.h"
#include "triangulatepoints2.h"
#include "impro_util.h"

using namespace std;

// Step 1: Read left and right cameras parameters 
// Step 2: Read left and right initial photos
// Step 3: Undistort initial photos
// Step 4: Ask users to pick four points on both undistorted photos 
// Step 5: Ask users the mesh size (e.g., 40 x 30) 
// Step 6: Generate mesh points (many points) (using interpQ4()) both cameras
// Step 7: Transform undistorted many points to "distorted" many points (both cameras)
// Step 8: For each photo pair (a pair is left photo + right photo) 
// Step 9:   Wait for the photos
// Step 10:  Track many image points on both cameras
// Step 10a:     By t-match
// Step 10b:     By ECC
// Step 10c:     By optical flow 
// Step 11:  Stereo triangulation for each point of many points 
// Step 11a:     Based on t-match
// Step 11b:     Based on ECC
// Step 11c:     Based on optical flow 
// Step 12:  Output image points and triangulated points to files (including m-script files) 
// Step 12a:     Based on t-match
// Step 12b:     Based on ECC
// Step 12c:     Based on optical flow 
// Step 13: Goto Step 8 until the end of the test 



int FuncWallDisp(int argc, char ** argv)
{
	//cv::VideoCapture cam(2, cv::CAP_DSHOW);
	//cam.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	//cam.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	//cam.set(cv::CAP_PROP_AUTOFOCUS, 0); // set autofocus off
	////cam.set(cv::CAP_PROP_SETTINGS, 1);
	//double f = 0;
	//int j = 0;
	//vector<string> Str;
	//Str.push_back("10000");
	//while (true)
	//{
	//	cam.set(cv::CAP_PROP_FOCUS, f); // set focus far
	//	
	//	
	//	vector<string> StrTemp(1);
	//	StrTemp[0].assign(Str[0], 1, Str[0].length());
	//	vector<int>Myint(1);
	//	Myint[0] = stoi(StrTemp[0]) + j + 1;
	//	StrTemp[0] = Str[0];
	//	string s1 = to_string(Myint[0]);
	//	StrTemp[0].insert(StrTemp[0].length() - s1.length(), s1);
	//	StrTemp[0].assign(StrTemp[0], 0, Str[0].length());
	//	cv::Mat img;
	//	cam.grab();
	//	cam.retrieve(img); 
	//	char buf[1000];
    //	snprintf(buf, 1000, "Set FOCUS to %8.2f", f);
	//	cv::putText(img, buf, cv::Point(100, 100), cv::FONT_HERSHEY_PLAIN, 3.0, cv::Scalar(13, 23,227)); 
	//	cv::imshow("TRYING FOCUS", img); 
	//	int key1 = 0;
	//	key1=cv::waitKey(0); 
	//	if (key1 = 13)
	//	{
	//		cv::imwrite("C:\\Users\\I\\Downloads\\hahahha\\" + StrTemp[0] + ".jpg", img);
	//		f++;
	//		j++;
	//	}
	//	if (f > 50 || key1 ==27) f = 0;
	//}
	
	string leftRight[2]; leftRight[0] = "left"; leftRight[1] = "right"; 
	string fnameCamCalib[2];
	string fnameImgInit[2], fnameImgInitUndistort[2];
	string outputDirectory; 
	cv::Mat imgInitOri[2]; // initial image but are (original), not sobel processed 
	cv::Mat imgInit[2]; // initial image (step 0)
	cv::Mat imgPrev[2]; // previous image 
	cv::Mat imgCurr[2]; // current image 
	cv::Mat cmat[2], dvec[2];
	cv::Mat r4[2], tvec[2], rvec[2]; 
	cv::Mat imgInitUndistort[2]; // undistorted image of imgInit[2]
	Points2fHistoryData fourPoints[2], manyPoints[2], manyPointsDistorted[2]; // only for initial photos 
	Points2fHistoryData TMatchPoints[2], EccPoints[2], OptPoints[2]; // image points history (nStep, n12*n23*CV_32FC2)
	Points3dHistoryData TMatchPoints3d, EccPoints3d, OptPoints3d;  // 3d triangulated points history (nStep, n12*n23*CV_32FC2)
	cv::Mat TMatchPointsTriangulatedError; //   triangulated points (nStep, n12 * n23, CV_64F);
	cv::Mat EccPointsTriangulatedError;   //    triangulated points (nStep, n12 * n23, CV_64F);
	cv::Mat OptPointsTriangulatedError;   //    triangulated points (nStep, n12 * n23, CV_64F);
	cv::VideoCapture usbCam[2];           //    usb cameras (optional) 


	// Step 1: Read left and right cameras parameters
	for (int i = 0; i < 2; i++) { // Left and right 
		while (true)
		{
			std::cout << "Input full path of " << leftRight[i] << " camera calibration file (xml):\n";
			fnameCamCalib[i] = readStringLineFromIstream(std::cin);
			cv::FileStorage a(fnameCamCalib[i], cv::FileStorage::READ);
			a["cameraMatrix"] >> cmat[i];
			a["distortionVector"] >> dvec[i];
			a["R4"] >> r4[i];
			try
			{
				// try to read tvec ad rvec as cv::Mat
				a["tvec"] >> tvec[i];
				a["rvec"] >> rvec[i];
			}
			catch (...)
			{
				// if failed, read as vector<double> 
				tvec[i] = cv::Mat(3, 1, CV_64F);
				rvec[i] = cv::Mat(3, 1, CV_64F);
				vector<double> _tvec(3); 
				vector<double> _rvec(3);
				a["tvec"] >> _tvec;
				a["rvec"] >> _rvec;		
				for (int k = 0; k < 3; k++) tvec[i].at<double>(k, 0) = _tvec[k]; 
				for (int k = 0; k < 3; k++) rvec[i].at<double>(k, 0) = _rvec[k];
			}
			if (cmat[i].cols != 3 || cmat[i].rows != 3) {
				std::cout << fnameCamCalib[i] << " does not contain cameraMatrix. Try again.\n";
				continue;
			}
			if (min(dvec[i].cols, dvec[i].rows) != 1 || max(dvec[i].cols, dvec[i].rows) <= 1) {
				std::cout << fnameCamCalib[i] << " dvec size should be 1x4, 1x5, or 1x8 but " << dvec[i].rows << "x" << dvec[i].cols << endl;
				std::cout << fnameCamCalib[i] << " does not contain distortionVector. Try again.\n";
				continue;
			}
			if (r4[i].cols != 4 || r4[i].rows != 4) {
				std::cout << fnameCamCalib[i] << " does not contain R4. Try again.\n";
				continue;
			}
			if (tvec[i].cols * tvec[i].rows != 3) {
				tvec[i] = r4[i](cv::Rect(3, 0, 1, 3)).clone();
			}
			if (rvec[i].cols * rvec[i].rows != 3) {
				rvec[i] = cv::Mat(3, 1, r4[i].type());
				cv::Rodrigues(rvec[i], r4[i](cv::Rect(0, 0, 3, 3)));
			}
			if (tvec[i].cols == 3 && tvec[i].rows == 1) tvec[i] = tvec[i].t();
			if (rvec[i].cols == 3 && rvec[i].rows == 1) rvec[i] = rvec[i].t();
			break;
		}
		std::cout << "Camera matrix (" << leftRight[i] << " is:\n" << cmat[i] << endl;
		std::cout << "Distortion vec (" << leftRight[i] << " is:\n" << dvec[i] << endl;

	}

	std::cout << "Enter the output directory:\n";
	outputDirectory = readStringLineFromCin();
	outputDirectory = appendSlashOrBackslashAfterDirectoryIfNecessary(outputDirectory);

	// Step 2: Read left and right initial photos
	for (int i = 0; i < 2; i++)
	{
		while (true)
		{
			std::cout << "Input full path of " << leftRight[i] << " cam initial photo (or VideoCapture0 for camera 0, and so on):\n";
			fnameImgInit[i] = readStringLineFromIstream(std::cin); 
			if (fnameImgInit[i].substr(0,12).compare("VideoCapture") == 0) 
			{
				// if it is a VideoCapture object  
				int ncam = fnameImgInit[i][12] - '0';
				int cam_id, w, h; 
				for (cam_id = 0; cam_id < ncam; cam_id++)
				{
					char buf[1000];
//					bool retVal = usbCam[i].open(cam_id);
					bool retVal = usbCam[i].open(cam_id, cv::CAP_DSHOW);
//					bool retVal = usbCam[i].open(cam_id, cv::CAP_VFW);
					if (retVal == false) continue; 
					usbCam[i].set(cv::CAP_PROP_FRAME_WIDTH, 1920);
					usbCam[i].set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
					usbCam[i].set(cv::CAP_PROP_AUTOFOCUS, 0); // set autofocus off
					usbCam[i].set(cv::CAP_PROP_FOCUS, 0.001); // set focus far
					w = (int) usbCam[i].get(cv::CAP_PROP_FRAME_WIDTH);
					h = (int) usbCam[i].get(cv::CAP_PROP_FRAME_HEIGHT);
					bool readRetVal = usbCam[i].retrieve(imgInitOri[i]);
					if (readRetVal == true && imgInitOri[i].cols > 0 && imgInitOri[i].rows > 0)
					{
						cout << "CAP_PROP_FOCUS of " << cam_id << ": " << usbCam[i].get(cv::CAP_PROP_FOCUS) << endl;
                        snprintf(buf, 1000, "%d (%d x %d)", cam_id, w, h);
						imshow_resize("VideoCapture " + string(buf), imgInitOri[i], 0.25);
					}
					usbCam[i].release(); 
				}
				cout << "Which camera do you want for cam " << leftRight[i] << "? (Press ESC then answer) \n";
				int ikey = cv::waitKey(0);
				cam_id = readIntFromCin();
				cv::destroyAllWindows();
				usbCam[i].open(cam_id);
				usbCam[i].set(cv::CAP_PROP_FRAME_WIDTH, 1920);
				usbCam[i].set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
				usbCam[i].set(cv::CAP_PROP_AUTOFOCUS, 0); // set autofocus off
				cout << "CAP_PROP_FOCUS: " << usbCam[i].get(cv::CAP_PROP_FOCUS) << endl;
				bool readRetVal = usbCam[i].retrieve(imgInitOri[i]);
				if (readRetVal == true && imgInitOri[i].cols > 0 && imgInitOri[i].rows > 0) break;
				std::cout << fnameImgInit[i] << " cannot read valid image. Try again.\n";
			}
			else {
				// if it is a general file
				imgInitOri[i] = cv::imread(fnameImgInit[i], cv::IMREAD_GRAYSCALE);
				if (imgInitOri[i].cols > 0 && imgInitOri[i].rows > 0) break;
				std::cout << fnameImgInit[i] << " is not a valid image. Try again.\n";
			}
		} // while trying to get image (from either file or usb camera)
		imgInit[i] = sobel_xy(imgInitOri[i]);
		imshow_resize("InitImg", imgInitOri[i], 0.25); cv::waitKey(1000); cv::destroyWindow("InitImg");
		cv::imwrite(outputDirectory + "_" + fnameImgInit[i] + "_" + leftRight[i] + ".JPG", imgInitOri[i]); 
	} // loop of left (i = 0) and right (i = 1)

	// Step 3: Undistort initial photos
	for (int i = 0; i < 2; i++)
	{
		cv::undistort(imgInitOri[i], imgInitUndistort[i], cmat[i], dvec[i]); 
//		fnameImgInitUndistort[i] = extFilenameRemoved(fnameImgInit[i]) + "_undistort.JPG"; 
		fnameImgInitUndistort[i] =
			outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i]+ "_undistort.JPG";
		cv::imwrite(fnameImgInitUndistort[i], imgInitUndistort[i]);
		imshow_resize("Initial " + leftRight[i], imgInitOri[i], 0.25);
		imshow_resize("Undistorted " + leftRight[i], imgInitUndistort[i], 0.25);
	}
	cv::waitKey(0); 
	cv::destroyAllWindows();

	// Step 4: Ask users to pick four points on both undistorted photos 
	// Step 5: Ask users the mesh size (e.g., 40 x 30)
	// Step 6: Generate mesh points (many points) (using interpQ4()) both cameras
	std::cout << "You will be asked to pick four points of your wall ROI (P1,P2,P3, and P4).\n";
	std::cout << "How many points are to be interpolated along P1-P2, and P2-P3? (e.g., 30  40):\n";
	int n12 = readIntFromCin();
	int n23 = readIntFromCin(); 
	
	for (int i = 0; i < 2; i++)
	{
		std::cout << "Do you want to define 4 points of " << leftRight[i] << " camera by mouse picking or from file? (0:mouse picking, else:from file):\n"; 
		int mouseOrFile = readIntFromCin();
		ImagePointsPicker userPicked;
		userPicked.setBackgroundImageByFilename(fnameImgInitUndistort[i]);
		if (mouseOrFile == 0) {
			userPicked.pickTemplates(4, 27, 32, 13);
			userPicked.writePointsToXmlFile(
				outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i] + "_FourPoints.xml");
		}
		else {
			userPicked.readPointsFromXmlFile(
				outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i] + "_FourPoints.xml");
		}
		userPicked.generateQ4Templates(n12, n23);
		vector<cv::Point2f> vecPoints = userPicked.pointsInPoint2fVector();
		manyPoints[i].set(cv::Mat::zeros(1, n12 * n23, CV_32FC2));
		for (int j = 0; j < vecPoints.size(); j++) {
			manyPoints[i].set(0, j, vecPoints[j]);
			manyPoints[i].setRect(j, userPicked.rectsInRectVector()[j]); 
		}
		manyPoints[i].writeScriptMatAdvanced(
			outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i] + "_manyPoints.m",
			outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i] + "_undistort.JPG",
			true, 1);
		manyPoints[i].writeToTxt(
			outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i] + "_manyPoints.txt");
		manyPoints[i].writeToXml(
			outputDirectory + extFilenameRemoved(fileOfFullPathFile(fnameImgInit[i])) + "_" + leftRight[i] + "_manyPoints.xml");
	}

	// Step 7: Transform undistorted many points to "distorted" many points (both cameras)
	int nStep = 1; 
	for (int i = 0; i < 2; i++)
	{
		manyPointsDistorted[i].set(cv::Mat::zeros(nStep, n12 * n23, CV_32FC2));
		for (int iy = 0; iy < n23; iy++)
		{
			for (int ix = 0; ix < n12; ix++)
			{
				int iPoint = ix + iy * n12; 
				float xu = manyPoints[i].get(0, iPoint).x;
				float yu = manyPoints[i].get(0, iPoint).y;
				float fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6, kd;
				if (cmat[i].type() == CV_64F)
				{
					fx = (float)cmat[0].at<double>(0, 0);
					fy = (float)cmat[0].at<double>(1, 1);
					cx = (float)cmat[i].at<double>(0, 2);
					cy = (float)cmat[i].at<double>(1, 2);
				}
				else {
					fx = (float)cmat[0].at<float>(0, 0);
					fy = (float)cmat[0].at<float>(1, 1);
					cx = (float)cmat[i].at<float>(0, 2);
					cy = (float)cmat[i].at<float>(1, 2);
				}
				if (dvec->type() == CV_64F)
				{
					k1 = (float)((double*)dvec[i].data)[0];
					k2 = (float)((double*)dvec[i].data)[1];
					p1 = (float)((double*)dvec[i].data)[2];
					p2 = (float)((double*)dvec[i].data)[3];
					if (dvec[i].size[0] * dvec[i].size[1] < 5) k3 = 0.f; else k3 = (float)((double*)dvec[i].data)[4];
					if (dvec[i].size[0] * dvec[i].size[1] < 6) k4 = 0.f; else k4 = (float)((double*)dvec[i].data)[5];
					if (dvec[i].size[0] * dvec[i].size[1] < 7) k5 = 0.f; else k5 = (float)((double*)dvec[i].data)[6];
					if (dvec[i].size[0] * dvec[i].size[1] < 8) k6 = 0.f; else k6 = (float)((double*)dvec[i].data)[7];
				}
				else {
					k1 = (float)((float*)dvec[i].data)[0];
					k2 = (float)((float*)dvec[i].data)[1];
					p1 = (float)((float*)dvec[i].data)[2];
					p2 = (float)((float*)dvec[i].data)[3];
					if (dvec[i].size[0] * dvec[i].size[1] < 5) k3 = 0.f; else k3 = (float)((float*)dvec[i].data)[4];
					if (dvec[i].size[0] * dvec[i].size[1] < 6) k4 = 0.f; else k4 = (float)((float*)dvec[i].data)[5];
					if (dvec[i].size[0] * dvec[i].size[1] < 7) k5 = 0.f; else k5 = (float)((float*)dvec[i].data)[6];
					if (dvec[i].size[0] * dvec[i].size[1] < 8) k6 = 0.f; else k6 = (float)((float*)dvec[i].data)[7];
				}
				float xn = (xu - cx) / fx;
				float yn = (yu - cy) / fy;
				float r = sqrt(xn * xn + yn * yn);
				kd = (1 + k1 * r * r + k2 * pow(r, 4) + k3 * pow(r, 6)) / (1 + k4 * r * r + k5 * pow(r, 4) + k6 * pow(r, 6));
				float xd = xn * kd + 2 * p1 * xn * yn + p2 * (r * r + 2 * xn * xn);
				float yd = yn * kd + p1 * (r * r + 2 * yn * yn) + 2 * p2 * xn * yn;
				xd = fx * xd + cx;
				yd = fy * yd + cy;
				// point
				manyPointsDistorted[i].set(0, iPoint, cv::Point2f(xd, yd));
				// rect
				cv::Rect rect = manyPoints[i].getRect(iPoint);
				rect.x = (int) (rect.x - manyPoints[i].get(0, iPoint).x + manyPointsDistorted[i].get(0, iPoint).x + .5f); 
				rect.y = (int) (rect.y - manyPoints[i].get(0, iPoint).y + manyPointsDistorted[i].get(0, iPoint).y + .5f);
				manyPointsDistorted[i].setRect(iPoint, rect);
			} // end of ix
		} // end of iy

		manyPointsDistorted[i].writeScriptMatAdvanced(extFilenameRemoved(fnameImgInit[i]) + "_" + leftRight[i] + "_manyPointsDistorted.m", fnameImgInit[i], true, 1);
		manyPointsDistorted[i].writeToTxt(extFilenameRemoved(fnameImgInit[i]) + "_" + leftRight[i] + "_manyPointsDistorted.txt");
		manyPointsDistorted[i].writeToXml(extFilenameRemoved(fnameImgInit[i]) + "_" + leftRight[i] + "_manyPointsDistorted.xml");

	} // end if i (camera id) 


	// Step 8: For each photo pair (a pair is left photo + right photo) 
	// preparation for loop: file sequence
	FileSeq fsq[2]; 
	for (int i = 0; i < 2; i++)
	{
		std::cout << "Set the photos file names of the " << leftRight[i] << " camera: \n";
		fsq[i].setDirFilesByConsole(); 
	}
	if (fsq[0].num_files() != fsq[1].num_files())
	{
		std::cout << "Error: Left and Right cameras have different number of photos. \n";
		return -1; 
	}
	nStep = fsq[0].num_files();

	cout << "Enter on/off (1/0) of T-Match, ECC, Optical-Flow: (E.g., 1 0 1 for T-Match on, Ecc off, Opt-flow on.)\n";
	int tmt_on = readIntFromCin();
	int ecc_on = readIntFromCin();
	int opt_on = readIntFromCin();

	// preparation for loop: declare big data arrays
	for (int i = 0; i < 2; i++)
	{
		TMatchPoints[i].set(cv::Mat::zeros(nStep, n12 * n23, CV_32FC2));
		EccPoints[i].set(cv::Mat::zeros(nStep, n12 * n23, CV_32FC2));
		OptPoints[i].set(cv::Mat::zeros(nStep, n12 * n23, CV_32FC2));
	}
	TMatchPointsTriangulatedError = cv::Mat::zeros(nStep, n12 * n23, CV_64F);
	EccPointsTriangulatedError = cv::Mat::zeros(nStep, n12 * n23, CV_64F);
	OptPointsTriangulatedError = cv::Mat::zeros(nStep, n12 * n23, CV_64F);

	TMatchPoints3d.set(cv::Mat::zeros(nStep, n12 * n23, CV_64FC3));
	EccPoints3d.set(cv::Mat::zeros(nStep, n12 * n23, CV_64FC3));
	OptPoints3d.set(cv::Mat::zeros(nStep, n12 * n23, CV_64FC3));
	printf("Allocated memory for TMatch image points: 2 cams x (nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * 2 * sizeof(float) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for TMatch 3d points(nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * 3 * sizeof(double) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for TMatch 3d err(nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * sizeof(double) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for Ecc image points: 2 cams x (nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * 2 * sizeof(float) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for Ecc 3d points(nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * 3 * sizeof(double) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for Ecc 3d err(nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * sizeof(double) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for OptFlow image points: 2 cams x (nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * 2 * sizeof(float) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for OptFlow 3d points(nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * 3 * sizeof(double) * 1.f / (1024 * 1024 * 1024));
	printf("Allocated memory for OptFlow 3d err(nStep %d, nPoint %dx%d) (%5.1f GB)\n", nStep, n12, n23, nStep * n12 * n23 * sizeof(double) * 1.f / (1024 * 1024 * 1024));

	// preparation for loop: allocate guessed image points
	cv::Mat guessedImgPoints[2]; // guessed image points, sized 1 x (n12 x n23)
	for (int iCam = 0; iCam < 2; iCam++)
	{
		int iStep = 0; 
		guessedImgPoints[iCam] = cv::Mat(1, n12 * n23, CV_32FC2);
		//for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
		//{
		//	guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = manyPointsDistorted[iCam].get(iStep, iPoint);
		//}
	}

	// Start the major loop
	for (int iStep = 0; iStep < nStep; iStep++)
	{
//#pragma omp parallel for 
		for (int iCam = 0; iCam < 2; iCam++)
		{
			// define the previous image
			if (iStep == 0)
				imgPrev[iCam] = imgInit[iCam]; 
			else 
				imgPrev[iCam] = imgCurr[iCam];
			// guessed points
			int guessOrder = 2; // if guessOrder == 1, linear extrapolation; if guessOrder == 2, 2nd order extrapolation
			if (iStep == 0) {
				// for iStep == 0, guessed point is the user defined target points
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++) {
					guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = manyPointsDistorted[iCam].get(iStep, iPoint);
				}
			}
			else if (iStep == 1) { // for iStep == 1, guessed point is the previous points 
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++) {
					if (tmt_on > 0)
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = TMatchPoints[iCam].get(iStep - 1, iPoint); 
					else if (opt_on > 0)
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = OptPoints[iCam].get(iStep - 1, iPoint);
					else if (ecc_on > 0)
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = EccPoints[iCam].get(iStep - 1, iPoint);
					else
					{
						cerr << "You disabled all tracking methods.\n";
						return -1; 
					}
				}
			}
			else if (iStep == 2 || guessOrder <= 1) { // for iStep == 2, use linear extrapolation
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++) {
					if (tmt_on > 0)
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = 2 * TMatchPoints[iCam].get(iStep - 1, iPoint) - TMatchPoints[iCam].get(iStep - 2, iPoint);
					else if (opt_on > 0)
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = 2 * OptPoints[iCam].get(iStep - 1, iPoint) - OptPoints[iCam].get(iStep - 2, iPoint);
					else if (ecc_on > 0)
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint) = 2 * EccPoints[iCam].get(iStep - 1, iPoint) - EccPoints[iCam].get(iStep - 2, iPoint);
					else
					{
						cerr << "You disabled all tracking methods.\n";
						return -1;
					}
				}
			}
			else { // for iStep >= 3, use 2nd order extrapolation
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++) {
					cv::Point2f p0, p1, p2;
					if (tmt_on > 0) {
						p2 = TMatchPoints[iCam].get(iStep - 1, iPoint);
						p1 = TMatchPoints[iCam].get(iStep - 2, iPoint);
						p0 = TMatchPoints[iCam].get(iStep - 3, iPoint);
					}
					else if (opt_on > 0) {
						p2 = OptPoints[iCam].get(iStep - 1, iPoint);
						p1 = OptPoints[iCam].get(iStep - 2, iPoint);
						p0 = OptPoints[iCam].get(iStep - 3, iPoint);
					}
					else if (ecc_on > 0) {
						p2 = EccPoints[iCam].get(iStep - 1, iPoint);
						p1 = EccPoints[iCam].get(iStep - 2, iPoint);
						p0 = EccPoints[iCam].get(iStep - 3, iPoint);
					}
					else
					{
						cerr << "You disabled all tracking methods.\n";
						return -1;
					}
					//guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).x = 2 * p2.x - p1.x + 0.5f * powf(p2.x - 2 * p1.x + p0.x, 2);
					//guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).y = 2 * p2.y - p1.y + 0.5f * powf(p2.y - 2 * p1.y + p0.y, 2);
					guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).x = p0.x - 3 * p1.x + 3 * p2.x; 
					guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).y = p0.y - 3 * p1.y + 3 * p2.y;;
				}
			}

			// cloned templates (targets)
			// Although the templates (targets) can be sub-images of large images, the performance can be much better 
			// if the templates are small images which have their own small memory. 
			vector<cv::Mat> targetsInit(n12 * n23);
			vector<cv::Point2f> refsInit(n12 * n23); 
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				// check if the target is near image boundary and template is out of image range. 
				// If ok (i.e., not near boundary), preferredRef is not changed, and the returned rect is naturally within range.
				// If yes (template is out of range), preferredRef is adjusted, and the returned rect is within range.
				cv::Point2f targetPoint2f = manyPointsDistorted[iCam].get(0, iPoint);
				cv::Size targetTmpltSize = cv::Size(manyPointsDistorted[iCam].getRect(iPoint).width, manyPointsDistorted[iCam].getRect(iPoint).height);
				cv::Point2f preferredRef, ref;
				preferredRef.x = targetPoint2f.x - manyPointsDistorted[iCam].getRect(iPoint).x;
				preferredRef.y = targetPoint2f.y - manyPointsDistorted[iCam].getRect(iPoint).y;
				cv::Rect rect = getTmpltRectFromImageSizeWithPreferredRef(
					imgInit[iCam].size(), // full image size
					targetPoint2f, // initial position of target point 
					targetTmpltSize, // target template size
					preferredRef); 
				targetsInit[iPoint] = imgInit[iCam](rect).clone(); 
				refsInit[iPoint] = preferredRef;
			}

			// Step 9:   Wait for the photos
			fsq[iCam].waitForImageFile(iStep, imgCurr[iCam], cv::IMREAD_GRAYSCALE);
			imgCurr[iCam] = sobel_xy(imgCurr[iCam]); 
			cout << "Found file of Cam " << iCam + 1 << " Step " << iStep + 1 << ", file name " << fsq[iCam].fullPathOfFile(iStep) << endl;

			// Step 10:  Track many image points on both cameras
			// Step 10a:     By t-match
			// T-match: target tracking 
			// From guessedImgPoints[iCam]
			// To TMatchPoints[iCam]
			if (tmt_on > 0)
#pragma omp parallel for 
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
				{
					double min_x = guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).x - targetsInit[iPoint].cols; 
					double max_x = guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).x + targetsInit[iPoint].cols;
					double precision_x = .05; 
					double min_y = guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).y - targetsInit[iPoint].rows;
					double max_y = guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).y + targetsInit[iPoint].rows;
					double precision_y = 0.05;
					double min_r = 0.;
					double max_r = 0.;
					double precision_r = 1.;
					vector<double> tMatchResult(10); 

					int tmatchRet; 
					{
					tmatchRet = matchTemplateWithRotPyr(
						imgCurr[iCam],
						targetsInit[iPoint],
						refsInit[iPoint].x, refsInit[iPoint].y,
						min_x, max_x, precision_x,
						min_y, max_y, precision_y,
						min_r, max_r, precision_r,
						tMatchResult);
					float target_x = (float)tMatchResult[0];
					float target_y = (float)tMatchResult[1];
					TMatchPoints[iCam].set(iStep, iPoint, cv::Point2f(target_x, target_y));
					printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
					printf("TMatch Cam %d Point %04d. X:%7.2f Y:%7.2f ", iCam + 1, iPoint + 1, target_x, target_y);
				} // end of n12*n23 points 
			} // tmt_on 

			// Step 10:  Track many image points on both cameras
			// Step 10b:     By t-Ecc
			// From guessedImgPoints[iCam]
			// To EccPoints[iCam]
			if (ecc_on > 0)
			{
#pragma omp parallel for 
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
				{
					vector<double> eccResult(10);
					enhancedCorrelationWithReference(imgCurr[iCam], targetsInit[iPoint],
						refsInit[iPoint].x, refsInit[iPoint].y,
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).x,
						guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint).y,
						0.0 /* init_rot */,
						eccResult,
						cv::MOTION_TRANSLATION,
						cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50 /* ecc max count */, 0.01 /* eps */));
					float target_x = (float)eccResult[0];
					float target_y = (float)eccResult[1];
					EccPoints[iCam].set(iStep, iPoint, cv::Point2f(target_x, target_y));
					printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
					printf("Ecc Cam %d Point %04d. X:%7.2f Y:%7.2f ", iCam + 1, iPoint + 1, target_x, target_y);
				} // end of n12*n23 points 
			} // ecc_on 

			// Step 10:  Track many image points on both cameras
			// Step 10b:     By optical flow 
			// From guessedImgPoints[iCam]
			// To OptPoints[iCam]
			if (opt_on > 0)
			{
				vector<cv::Point2f> prevPts(n12 * n23), currPts(n12 * n23);
				vector<uchar> optFlow_status(n12 * n23);
				vector<float> optFlow_err(n12 * n23);
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
				{
					if (iStep == 0)
						prevPts[iPoint] = manyPointsDistorted[iCam].get(0, iPoint);
					else
						prevPts[iPoint] = OptPoints[iCam].get(iStep - 1, iPoint);
					currPts[iPoint] = guessedImgPoints[iCam].at<cv::Point2f>(0, iPoint); 
				}

				calcOpticalFlowPyrLK(
					imgPrev[iCam], // previous photo
					imgCurr[iCam],  // current photo
					prevPts,
					currPts,
					optFlow_status,
					optFlow_err,
					cv::Size(61, 61),
					2 /* max level */,
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50 /* ecc max count */, 0.001 /* eps */)
				);
				for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
				{
					OptPoints[iCam].set(iStep, iPoint, currPts[iPoint]);
				}
				printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
				printf("Optical flow completed.\n");
			} // opt_on

		} // end of iCam loop (left and right)

		// print results (comparison with different methods)
		int iPoint, iCam;
//		printf("Lower-Left Point (Left Cam)\n");
		printf("T-Match             ECC          OptFlow\n");
		iPoint = 0; iCam = 0; 
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y); 
//		printf("Lower-Left Point (Right Cam)\n");
//		printf("T-Match             ECC          OptFlow\n");
		iPoint = 0; iCam = 1;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);

//		printf("Lower-Right Point (Left Cam)\n");
//		printf("T-Match             ECC          OptFlow\n");
		iPoint = n12; iCam = 0;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);
//		printf("Lower-Right Point (Right Cam)\n");
//		printf("T-Match             ECC          OptFlow\n");
		iPoint = n12; iCam = 1;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);

//		printf("Upper-Right Point (Left Cam)\n");
//		printf("T-Match             ECC          OptFlow\n");
		iPoint = n12 * n23 - 1; iCam = 0;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);
		//printf("Upper-Right Point (Right Cam)\n");
		//printf("T-Match             ECC          OptFlow\n");
		iPoint = n12 * n23 - 1; iCam = 1;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);

		//printf("Upper-Left Point (Left Cam)\n");
		//printf("T-Match             ECC          OptFlow\n");
		iPoint = n12 * (n23 - 1); iCam = 0;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);
		//printf("Upper-Left Point (Right Cam)\n");
		//printf("T-Match             ECC          OptFlow\n");
		iPoint = n12 * (n23 - 1); iCam = 1;
		printf("%7.2f %7.2f  %7.2f %7.2f  %7.2f %7.2f  \n",
			TMatchPoints[iCam].get(iStep, iPoint).x, TMatchPoints[iCam].get(iStep, iPoint).y,
			EccPoints[iCam].get(iStep, iPoint).x, EccPoints[iCam].get(iStep, iPoint).y,
			OptPoints[iCam].get(iStep, iPoint).x, OptPoints[iCam].get(iStep, iPoint).y);

		// Step 11:  Stereo triangulation for each point of many points 
		cv::Mat leftCamPoints(1, n12 * n23, CV_32FC2); 
		cv::Mat rightCamPoints(1, n12 * n23, CV_32FC2);
		cv::Mat triangulatedPoints(n12 * n23, 3, CV_64F), triangulatedErrs(1, n12 * n23, CV_64F);
		// Step 11a:     Based on t-match
		if (tmt_on > 0)
		{
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				leftCamPoints.at<cv::Point2f>(0, iPoint) = TMatchPoints[0].get(iStep, iPoint);
				rightCamPoints.at<cv::Point2f>(0, iPoint) = TMatchPoints[1].get(iStep, iPoint);
			}
			triangulatePointsGlobalCoordinate(
				cmat[0], dvec[0], r4[0],
				cmat[1], dvec[1], r4[1],
				leftCamPoints, rightCamPoints,
				triangulatedPoints, triangulatedErrs);
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				cv::Point3d p;
				p.x = triangulatedPoints.at<double>(iPoint, 0);
				p.y = triangulatedPoints.at<double>(iPoint, 1);
				p.z = triangulatedPoints.at<double>(iPoint, 2);
				TMatchPoints3d.set(iStep, iPoint, p);
				TMatchPointsTriangulatedError.at<double>(iStep, iPoint) = triangulatedErrs.at<double>(0, iPoint);
			}
		} // tmt_on

		// Step 11b:     Based on ecc
		if (ecc_on)
		{
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				leftCamPoints.at<cv::Point2f>(0, iPoint) = EccPoints[0].get(iStep, iPoint);
				rightCamPoints.at<cv::Point2f>(0, iPoint) = EccPoints[1].get(iStep, iPoint);
			}
			triangulatePointsGlobalCoordinate(
				cmat[0], dvec[0], r4[0],
				cmat[1], dvec[1], r4[1],
				leftCamPoints, rightCamPoints,
				triangulatedPoints, triangulatedErrs);
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				cv::Point3d p; 
				p.x = triangulatedPoints.at<double>(iPoint, 0);
				p.y = triangulatedPoints.at<double>(iPoint, 1);
				p.z = triangulatedPoints.at<double>(iPoint, 2);
				EccPoints3d.set(iStep, iPoint, p);
				EccPointsTriangulatedError.at<double>(iStep, iPoint) = triangulatedErrs.at<double>(0, iPoint);
			}
		} // ecc_on

		// Step 11c:     Based on optical flow
		if (opt_on > 0)
		{
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				leftCamPoints.at<cv::Point2f>(0, iPoint) = OptPoints[0].get(iStep, iPoint);
				rightCamPoints.at<cv::Point2f>(0, iPoint) = OptPoints[1].get(iStep, iPoint);
			}
			triangulatePointsGlobalCoordinate(
				cmat[0], dvec[0], r4[0],
				cmat[1], dvec[1], r4[1],
				leftCamPoints, rightCamPoints,
				triangulatedPoints, triangulatedErrs);
			for (int iPoint = 0; iPoint < n12 * n23; iPoint++)
			{
				cv::Point3d p;
				p.x = triangulatedPoints.at<double>(iPoint, 0);
				p.y = triangulatedPoints.at<double>(iPoint, 1);
				p.z = triangulatedPoints.at<double>(iPoint, 2);
				OptPoints3d.set(iStep, iPoint, p);
				OptPointsTriangulatedError.at<double>(iStep, iPoint) = triangulatedErrs.at<double>(0, iPoint);
			}
		}

		// 
//		if (iStep < nStep - 1) continue;
		// Step 12:  Output image points and triangulated points to files (including m-script files) 
		// Step 12a:     Based on t-match
		// Step 12b:     Based on ECC
		// Step 12c:     Based on optical flow 

		printf("T-match(L)      Ecc(L)      Optflow(L)      T-match(R)      Ecc(R)      Optflow(R)     T-match(3D)      Ecc(3D)      Optflow(3D)      T-match(Disp)      Ecc(Disp)      Optflow(Disp)\n");
		int iPoints[] = { 0, n12 - 1, n12 * n23 - 1, n12 * (n23 - 1) };
		for (int iPointIndex = 0; iPointIndex < 4; iPointIndex++)
		{
			iPoint = iPoints[iPointIndex];
			printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n",
				TMatchPoints[0].get(iStep, iPoint).x, TMatchPoints[0].get(iStep, iPoint).y,
				EccPoints[0].get(iStep, iPoint).x, EccPoints[0].get(iStep, iPoint).y,
				OptPoints[0].get(iStep, iPoint).x, OptPoints[0].get(iStep, iPoint).y,
				TMatchPoints[1].get(iStep, iPoint).x, TMatchPoints[1].get(iStep, iPoint).y,
				EccPoints[1].get(iStep, iPoint).x, EccPoints[1].get(iStep, iPoint).y,
				OptPoints[1].get(iStep, iPoint).x, OptPoints[1].get(iStep, iPoint).y,
				TMatchPoints3d.get(iStep, iPoint).x, TMatchPoints3d.get(iStep, iPoint).y, TMatchPoints3d.get(iStep, iPoint).z,
				EccPoints3d.get(iStep, iPoint).x, EccPoints3d.get(iStep, iPoint).y, EccPoints3d.get(iStep, iPoint).z,
				OptPoints3d.get(iStep, iPoint).x, OptPoints3d.get(iStep, iPoint).y, OptPoints3d.get(iStep, iPoint).z,
				(TMatchPoints3d.get(iStep, iPoint) - TMatchPoints3d.get(0, iPoint)).x,
				(TMatchPoints3d.get(iStep, iPoint) - TMatchPoints3d.get(0, iPoint)).y,
				(TMatchPoints3d.get(iStep, iPoint) - TMatchPoints3d.get(0, iPoint)).z,
				(EccPoints3d.get(iStep, iPoint) - EccPoints3d.get(0, iPoint)).x,
				(EccPoints3d.get(iStep, iPoint) - EccPoints3d.get(0, iPoint)).y,
				(EccPoints3d.get(iStep, iPoint) - EccPoints3d.get(0, iPoint)).z,
				(OptPoints3d.get(iStep, iPoint) - OptPoints3d.get(0, iPoint)).x,
				(OptPoints3d.get(iStep, iPoint) - OptPoints3d.get(0, iPoint)).y,
				(OptPoints3d.get(iStep, iPoint) - OptPoints3d.get(0, iPoint)).z);
		} // end of loop of iPointIndex 0 ~ 3 

		// write data to files
		char buf[1000]; 
		for (int iCam = 0; iCam < 2; iCam++)
		{
            snprintf(buf, 1000, "%sTMatchImgPoints_Step%04d_Cam%01d.m",
				outputDirectory.c_str(), iStep + 1, iCam + 1); 
			if (tmt_on > 0)
				TMatchPoints[iCam].writeScriptMatAdvanced(
					buf, fnameImgInit[iCam], false, 1, 1 /* only data */, iStep);
            snprintf(buf, 1000, "%sEccImgPoints_Step%04d_Cam%01d.m",
				outputDirectory.c_str(), iStep + 1, iCam + 1);
			if (ecc_on > 0)
				EccPoints[iCam].writeScriptMatAdvanced(
					buf, fnameImgInit[iCam], false, 1, 1 /* only data */, iStep);
            snprintf(buf, 1000, "%sOptImgPoints_Step%04d_Cam%01d.m",
				outputDirectory.c_str(), iStep + 1, iCam + 1);
			if (opt_on > 0)
				OptPoints[iCam].writeScriptMatAdvanced(
					buf, fnameImgInit[iCam], false, 1, 1 /* only data */, iStep);
		}
		// output 3d points
        snprintf(buf, 1000, "%sTMatch3dPoints_Step%04d.m",
			outputDirectory.c_str(), iStep + 1);
		if (tmt_on > 0)
			TMatchPoints3d.writeScriptMatAdvanced(buf, true, 1, 1 /* only data */, iStep);
        snprintf(buf, 1000, "%sEcc3dPoints_Step%04d.m",
			outputDirectory.c_str(), iStep + 1);
		if (ecc_on > 0)
			EccPoints3d.writeScriptMatAdvanced(buf, true, 1, 1 /* only data */, iStep);
        snprintf(buf, 1000, "%sOpt3dPoints_Step%04d.m",
			outputDirectory.c_str(), iStep + 1);
		if (opt_on > 0)
			OptPoints3d.writeScriptMatAdvanced(buf, true, 1, 1 /* only data */, iStep);


		// Step 13: Goto Step 8 until the end of the test 
	}
	
	cv::destroyAllWindows(); 
	return 0; 
}
