#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "impro_fileIO.h"
#include "FileSeq.h"
#include "improStrings.h"

using std::string; 
using std::set;
using std::vector; 
using std::cout;
using std::cin; 
using std::endl; 
using std::printf; 


std::string sprintCameraIntrinsic(cv::Mat cmat, cv::Mat dvec, cv::Size imgSize = cv::Size(-1, -1)) {
	std::string str; 
	double fx, fy, cx, cy, k[14], width, height; 
//	double halfdiagSqr;
	char buf[1000];
	// convert cmat & dvec to CV_64F
	cmat.convertTo(cmat, CV_64F); 
	dvec.convertTo(dvec, CV_64F);
	// copy parameters to variables 
	fx = cmat.at<double>(0, 0);
	fy = cmat.at<double>(1, 1);
	cx = cmat.at<double>(0, 2); 
	cy = cmat.at<double>(1, 2); 
	for (int i = 0; i < 14; i++) {
		if (dvec.cols * dvec.rows > i)
			k[i] = dvec.at<double>(i);
		else
			k[i] = 0.0;
	}
	width = imgSize.width;
	height = imgSize.height; 
//	halfdiagSqr = 0.25 * (imgSize.width * imgSize.width + imgSize.height * imgSize.height); 
	// start printing. If image size is given, also print FOV and dimensionless information. 
	snprintf(buf, 1000, "# Image size (w x h): %d x %d\n", imgSize.width, imgSize.height); 
	snprintf(buf, 1000, "# fx: %24.16le", fx); 
	str += buf; 
	if (imgSize.width > 1) {
		snprintf(buf, 1000, " (FOVx: %6.2f degs.)", 2. * atan(imgSize.width * 0.5 / fx) * 180. / 3.1415926);
		str += buf;
	}
	str += std::string("\n");
	snprintf(buf, 1000, "# fy: %24.16le", fy);
	str += buf;
	if (imgSize.width > 1) {
		snprintf(buf, 1000, " (FOVy: %7.2f degs.)", 2. * atan(imgSize.height * 0.5 / fy) * 180. / 3.1415926);
		str += buf;
	}
	str += std::string("\n");
	snprintf(buf, 1000, "# cx: %24.16le", cx);
	str += buf;
	if (imgSize.width > 1) {
		snprintf(buf, 1000, " (Cx bias: %7.3f times image width.)", (cx - 0.5 * (width - 1)) / width); 
		str += buf;
	}
	str += std::string("\n");
	snprintf(buf, 1000, "# cy: %24.16le", cy);
	str += buf;
	if (imgSize.width > 1) {
		snprintf(buf, 1000, " (Cy bias: %7.3f times image height.)", (cy - 0.5 * (height - 1)) / height);
		str += buf;
	}
	str += std::string("\n");
	snprintf(buf, 1000, "# k1: %24.16le", k[0]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# k2: %24.16le", k[0]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# p1: %24.16le", k[2]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# p2: %24.16le", k[3]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# k3: %24.16le", k[4]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# k4: %24.16le", k[5]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# k5: %24.16le", k[6]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# k6: %24.16le", k[7]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# s1: %24.16le", k[8]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# s2: %24.16le", k[9]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# s3: %24.16le", k[10]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# s4: %24.16le", k[11]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# taux: %24.16le", k[12]);
	str += buf;
	str += std::string("\n");
	snprintf(buf, 1000, "# tauy: %24.16le", k[13]);
	str += buf;
	str += std::string("\n");
	return str;
}

int getCamIntrinsicFromConsole(cv::Mat& cmat, cv::Mat& dvec, cv::Size imgSize)
{
	char buf[1000];
	int uway = 0;
	std::string fnameIntrinsic;
	imgSize = cv::Size(-1, -1); 

	std::cout << "# How do you want to define camera parameters:\n";
	std::cout << "#  (1) From xml file (must have cameraMatrix for 3x3 matrix and distortionVector for a 1xN vector).\n";
	uway = readIntFromCin();

	if (uway == 1) {
		std::cout << "# Enter camera calibration file:\n";
		std::cout << "# (must have cameraMatrix for 3x3 matrix and distortionVector for a 1xN vector):\n";
		fnameIntrinsic = readStringFromCin();
		// open the file
		cv::FileStorage fs;
		while (true) {
			int waitTime = 10000; 
			fs.open(fnameIntrinsic, cv::FileStorage::READ);
			if (fs.isOpened() == true) break;
			printf("# Waiting for file %s ... (checking every %d ms.) \n", fnameIntrinsic.c_str(), waitTime); 
			std::this_thread::sleep_for(std::chrono::milliseconds(waitTime));
		}
		fs["cameraMatrix"] >> cmat;
		if (cmat.cols <= 0 || cmat.rows <= 0) fs["cmat"] >> cmat;
		if (cmat.cols <= 0 || cmat.rows <= 0) fs["camMat"] >> cmat;
		if (cmat.cols <= 0 || cmat.rows <= 0) fs["camMatrix"] >> cmat;
		if (cmat.cols != 3 || cmat.rows != 3) {
			std::cerr << "# Error: Cannot get camera matrix from " << fnameIntrinsic ;
			std::cerr << ". Camera matrix must be writen to tag cameraMatrix.\n";
		}
		fs["distortionVector"] >> dvec;
		if (dvec.cols <= 0 || dvec.rows <= 0) fs["dvec"] >> dvec;
		if (dvec.cols <= 0 || dvec.rows <= 0) fs["distortVec"] >> dvec;
		if (dvec.cols <= 0 || dvec.rows <= 0) fs["distortionVec"] >> dvec;
		if (std::min(dvec.cols, dvec.rows) != 1 || std::max(dvec.cols, dvec.rows) <= 1) {
			std::cerr << "# Error: Cannot get distortion vector from " << fnameIntrinsic;
			std::cerr << ". Distortion vector must be writen to tag distortionVector.\n";
		}
		fs["imageSize"] >> imgSize;
		if (imgSize.width <= 0) fs["imgSize"] >> imgSize;
		// turn to row vector
		dvec = dvec.t();
		dvec = dvec.reshape(1 /* # of channels */, 1 /* # of rows */).clone(); 
	}

	// print camera intrinsic 
	std::cout << sprintCameraIntrinsic(cmat, dvec, imgSize); 

	return 0;
}

int FuncUndistortOnline(int argc, char** argv)
{
	char buf[1000];
	cv::Size imgSize;     //!< image size of the camera
	cv::Mat cmat, dvec;   //!
	FileSeq fsq1, fsq2;   //!< file sequence of input and output images 
	cv::Size maxImshow;   //!< maximum window size to image show
	int waitKeyDelay;     //!< delay time for imshow waitKeyEx
    bool debug = true;

	const cv::String cmdParserKeys =
		"{help h usage ?      |                     | print this message   }"
		"{calibFileXml  calib | <none>              | calibration file that contains cameraMatrix for 3x3 matrix and distortionVector for a 1xN vector }"
		"{srcFileList     src | <none>              | file list that contains source images, each text line the file represents a full path of an image}"
		"{dstFileList     dst | <none>              | file list that contains destination images, each text line the file represents a full path of an image}"
		"{imshowMaxW imshow_w | <none>              | max width of undistort imshow() window}"
		"{imshowMaxH imshow_h | <none>              | max width of undistort imshow() window}"
		"{imshowTime imshow_t | <none>              | imshow timeout in ms         }"
		;
	cv::CommandLineParser cmdParser(argc, argv, cmdParserKeys); 
	cmdParser.about("Function undistortion online (undistonline)"); 

	if (cmdParser.has("help"))
	{
		cmdParser.printMessage();
		return 0;
	}

	std::cout << "# UndistortOnline undistorts images.\n";

	// Get intrinsic parameters
	string calibFname; 
	if (cmdParser.has("calib")) {
		// get intrinsic parameters from XML file ("cameraMatrix" and "distortionVector")
		calibFname = cmdParser.get<cv::String>("calib");
		cv::FileStorage ifs(calibFname, cv::FileStorage::READ); 
		if (ifs.isOpened()) {
			if (cmat.cols <= 0 || cmat.rows <= 0) ifs["cmat"] >> cmat; 
			if (cmat.cols <= 0 || cmat.rows <= 0) ifs["cameraMatrix"] >> cmat;
			if (dvec.cols <= 0 || dvec.rows <= 0) ifs["dvec"] >> dvec;
			if (dvec.cols <= 0 || dvec.rows <= 0) ifs["distortionVector"] >> dvec;
			if (imgSize.width <= 0 || imgSize.height <= 0) ifs["imgSize"] >> imgSize;
			if (imgSize.width <= 0 || imgSize.height <= 0) ifs["imageSize"] >> imgSize;
		}
	}
	if (cmat.cols <= 0 || cmat.rows <= 0 || dvec.cols <= 0 || dvec.rows <= 0) 
	{
		std::cout << "# Enter camera calibration file:\n";
		std::cout << "# (must have cameraMatrix for 3x3 matrix and distortionVector for a 1xN vector):\n";
		getCamIntrinsicFromConsole(cmat, dvec, imgSize);
	}

	// Get files of source images
	printf("# Getting information about source images...\n");
	string srcFname;
	if (cmdParser.has("src")) {
		// get source files from file list
		srcFname = cmdParser.get<cv::String>("src"); 
		fsq1.setFilesByListFile(srcFname);
		printf("#  %d files are set, from %s to %s under %s\n", 
			(int)(fsq1.num_files()),
			fsq1.filename(0).c_str(), 
			fsq1.filename(fsq1.num_files() - 1).c_str(), 
			fsq1.directory().c_str() ); 
	}
	if (fsq1.num_files() <= 0) {
		fsq1.setDirFilesByConsole();
		std::printf("# Source has %d images, starting from %s\n", (int) fsq1.num_files(), fsq1.fullPathOfFile(0).c_str());
		std::printf("# Now %d of them are already there.\n", fsq1.countCanRead());
	}

	// Get files of destination images
	printf("# Getting information about destination images...\n");
	string dstFname;
	if (cmdParser.has("dst")) {
		// get destination files from file list
		dstFname = cmdParser.get<cv::String>("dst");
		fsq2.setFilesByListFile(dstFname);
		printf("#  %d files are set, from %s to %s under %s\n",
			(int)fsq2.num_files(), 
			fsq2.filename(0).c_str(), 
			fsq2.filename(fsq2.num_files() - 1).c_str(),
			fsq2.directory().c_str()
		);
	}
	if (fsq2.num_files() <= 0) {
		fsq2.setDirFilesByConsole();
		std::printf("# Destination has %d images, starting from %s\n", (int) fsq2.num_files(), fsq2.fullPathOfFile(0).c_str());
		std::printf("# Now %d of them are already there.\n", fsq2.countCanRead());
	}
	if (fsq2.num_files() <= 0) {
		fsq2.setDirFilesByConsole();
		printf("# Output has %d images, starting from %s\n", (int) fsq2.num_files(), fsq2.fullPathOfFile(0).c_str());
		printf("# Now %d of them are already there.\n", fsq2.countCanRead());
	}

	// Get visualization setting
	maxImshow.width = -1; 
	maxImshow.height = -1; 
	if (cmdParser.has("imshow_w")) {
		maxImshow.width = cmdParser.get<int>("imshow_w");
	}
	if (cmdParser.has("imshow_h")) {
		maxImshow.height = cmdParser.get<int>("imshow_h");
	}
	if (maxImshow.width < 0 || maxImshow.height < 0) {
		printf("# Enter the maximum window size to display undistorted image (e.g., 800 450) (0 0 for not showing)\n");
		maxImshow.width = readIntFromCin();
		maxImshow.height = readIntFromCin();
	}

	// Get imshow time before timeout
	waitKeyDelay = -1; 
	if (cmdParser.has("imshow_t")) {
		waitKeyDelay = cmdParser.get<int>("imshow_t");
	}
	if (waitKeyDelay < 0) {
		printf("# Enter delay time (in ms) for imshow waitkey (-1 for wait forever):\n");
		waitKeyDelay = readIntFromCin();
	}

	// main loop
	printf("# Undistoring images.\n");
    printf("# The full path of the first image is %s\n", fsq1.fullPathOfFile(0).c_str());
    printf("File_name  Wait_time  Read_time  Undist_time  Write_time (ms)\n");
	for (int i = 0; i < fsq1.num_files(); i++)
	{
        cv::Mat imgIn, imgOut;
        // get file name
        std::string thisFilenamePath = fsq1.filename(i);

        // print file name in 20-space format
        //   could be "          A00000.JPG" (short file name filled with spaces)
        //   could be " ... Trimmed_A00.JPG" (long file name trimmed)
        std::string onlyFilename = splitPath(thisFilenamePath, set<char>{'\\', '/'}).back();
        if (onlyFilename.length() <= 20) {
            for (int k = 0; k < 20 - onlyFilename.length(); k++) printf(" ");
            printf("%s", onlyFilename.c_str());
        } else {
            printf(" ... %s", onlyFilename.substr(onlyFilename.length() - 15, 15).c_str());
        }
//        printf("%s  ", onlyFilename.c_str());

        if (debug) {
            printf("#   Directory is: %s \n", fsq1.directory().c_str());
            printf("#   File name is: %s \n", fsq1.filename(i).c_str());
            printf("#   Full path is: %s \n", fsq1.fullPathOfFile(i).c_str());
            printf("#   Waiting for %s ... ", fsq1.filename(i).c_str()); std::fflush(stdout);
        }

        // wait file
        /* timing */ auto ticWaiting = std::chrono::steady_clock::now();
//        fsq1.waitForFile(i, 50 /* waiting time after each check */, 86400 * 1000 /* max number of trials */);
        /* timing */ auto tocWaiting = std::chrono::steady_clock::now();
        /* timing */ double tWaiting = std::chrono::duration<double>(tocWaiting - ticWaiting).count();
        /* timing */ printf(" %8.3f", tWaiting);
        /* timing */ std::fflush(stdout);

        // read file
        /* timing */ auto ticReading = std::chrono::steady_clock::now();
        fsq1.waitForImageFile(i, imgIn);
        /* timing */ auto tocReading = std::chrono::steady_clock::now();
        /* timing */ double tReading = std::chrono::duration<double>(tocReading - ticReading).count();
        /* timing */ printf(" %8.3f", tReading);
        /* timing */ std::fflush(stdout);

        // generate undistorted image
//		printf(" Undistorting ...");
        /* timing */ auto ticUndistort = std::chrono::steady_clock::now();
		cv::undistort(imgIn, imgOut, cmat, dvec); 
        /* timing */ auto tocUndistort = std::chrono::steady_clock::now();
        /* timing */ double tUndistort = std::chrono::duration<double>(tocUndistort - ticUndistort).count();
        /* timing */ printf(" %8.3f", tUndistort);
        /* timing */ std::fflush(stdout);

        // write undistorted image
        /* timing */ auto ticImwrite = std::chrono::steady_clock::now();
        cv::imwrite(fsq2.fullPathOfFile(i), imgOut);
        /* timing */ auto tocImwrite = std::chrono::steady_clock::now();
        /* timing */ double tImwrite = std::chrono::duration<double>(tocImwrite - ticImwrite).count();
        /* timing */ printf(" %8.3f", tImwrite);
        /* timing */ std::fflush(stdout);

        /* timing */ printf("\n");
        /* timing */ std::fflush(stdout);

        // show undistorted image
		int ikey; 
		if (maxImshow.width > 0 && maxImshow.height > 0) {
			imshow_resize("Undistorted", imgOut, maxImshow);
			ikey = cv::waitKey(waitKeyDelay);
		}

		// Display information 
//		printf("# Spent %f sec. for undistorting and %f sec. for image writing.\n", tDistort, tImwrite);
	}

	return 0; 
}
