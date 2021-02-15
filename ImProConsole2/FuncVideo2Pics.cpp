#include <iostream>
#include <opencv2/opencv.hpp>
#include "impro_util.h"

using namespace std;

int FuncVideo2Pics(int argc, char** argv)
{
	string fnameVid, fnameFlist; 
	int vid_FRAME_WIDTH;
	int vid_FRAME_HEIGHT;
	double vid_FPS;
	int vid_FRAME_COUNT;

	// Get video
	cout << "Full path of video (space is allowed, 'g' for gui file dialog): ";
	fnameVid = readStringLineFromIstream(cin); 
	if (fnameVid.size() == 1 && fnameVid[0] == 'g')
		fnameVid = uigetfile(); 
	cv::VideoCapture vid(fnameVid); 
	if (vid.isOpened() == false)
	{
		cerr << "  FuncVideo2Pics cannot read video: " << fnameVid << endl;
		return -1;
	}
	vid_FRAME_WIDTH = (int)(vid.get(cv::CAP_PROP_FRAME_WIDTH) + .5);
	vid_FRAME_HEIGHT = (int)(vid.get(cv::CAP_PROP_FRAME_HEIGHT) + .5);
	vid_FPS = vid.get(cv::CAP_PROP_FPS);
	vid_FRAME_COUNT = (int)(vid.get(cv::CAP_PROP_FRAME_COUNT) + .5);

	cout << "         Width: " << vid_FRAME_WIDTH  << endl;
	cout << "        Height: " << vid_FRAME_HEIGHT << endl;
	cout << "Frame per sec.: " << vid_FPS          << endl;
	cout << "   Frame count: " << vid_FRAME_COUNT  << endl;

	// starting index and frame number
	int iStart, nFrame; 
	cout << "Starting frame (0-based) (negative value to quit): "; 
	iStart = readIntFromIstream(cin); 
	if (iStart < 0) return -1;
	cout << "Number of frames to convert (negative value means to-the-end): ";
	nFrame = readIntFromIstream(cin);
	if (nFrame < 0)
		nFrame = vid_FRAME_COUNT - iStart;

	// Picture files
	cout << "Full path file names of pictures (in C-style format including a %d, space allowed): ";
	string fnamePicsFormat = readStringLineFromIstream(cin); 

	// Read video and skip to frame iStart
	for (int i = 0; i < iStart; i++)
	{
		cv::Mat buf;
		vid >> buf; 
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "Skipping frame " << i << " ...";
	}

	// Start converting video to pictures
	char fnamePic[1000];
	for (int i = iStart; i < iStart + nFrame; i++)
	{
		if (i >= vid_FRAME_COUNT) break;
		cv::Mat buf; 
		vid >> buf; 
		if (buf.cols <= 0 || buf.rows <= 0)
		{
			cerr << "Error: Cannot read image from video object of " << fnameVid << "\n";
			cerr.flush(); 
			break;
		}
        snprintf(fnamePic, 1000, fnamePicsFormat.c_str(), i - iStart);
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "Writing frame " << i << " ...";
		cv::imwrite(fnamePic, buf); 
	}
	vid.release(); 
	return 0;
}



int FuncPics2Video(int argc, char** argv)
{
	string fnameVid, fnameFlist;
	char fnamePic[1000];

	// Picture files
	cout << "# Full path file names of pictures (in C-style format including a %d, space allowed): ";
	string fnamePicsFormat = readStringLineFromIstream(cin);

	// starting index and frame number
	int iStart, nFrame;
	cout << "# Starting frame (0-based) (negative value to quit): ";
	iStart = readIntFromIstream(cin);
	if (iStart < 0) return -1;
	cout << "# Number of frames to convert (negative value to quit): ";
	nFrame = readIntFromIstream(cin);
	if (nFrame < 0) return -1;

	// Get first image size
    snprintf(fnamePic, 1000, fnamePicsFormat.c_str(), iStart);
	cv::Mat img = cv::imread(fnamePic); 
	cv::Size imgSize = img.size(); 
	cout << "# Image size of " << fnamePic << " is " << imgSize << endl;
	if (imgSize.width <= 0 || imgSize.height <= 0) {
		cerr << "#  Error: Image size of " << fnamePic << " is zero.\n";
		return -1;
	}

	// Get video
	cout << "# Full path of video (space is allowed, 'g' for gui file dialog): ";
	fnameVid = readStringLineFromIstream(cin);
	if (fnameVid.size() == 1 && fnameVid[0] == 'g')
		fnameVid = uiputfile();
	if (fnameVid.size() == 0) return -1; 
	cout << "# Preferred frame rate (fps): ";
	double fps = readDoubleFromIstream(cin); 

	cv::VideoWriter vid(fnameVid, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, imgSize);
	if (vid.isOpened() == false)
	{
		cerr << "#   FuncPics2Video cannot open video: " << fnameVid << endl;
		return -1;
	}
	cout << "#          Width: " << imgSize.width << endl;
	cout << "#         Height: " << imgSize.height << endl;
	cout << "# Frame per sec.: " << fps << endl;

	// Start converting 
	for (int i = iStart; i < iStart + nFrame; i++)
	{
        snprintf(fnamePic, 1000, fnamePicsFormat.c_str(), i);
		img = cv::imread(fnamePic);
		if (img.cols <= 0 || img.rows <= 0) {
			cerr << "#  " << fnamePic << " does not exist. Skipping this frame.\n"; 
			continue; 
		}
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "# Writing frame " << i << " ...";
		vid << img;
	}
	vid.release();
	return 0;
}

int FuncVideo2PicsResize(int argc, char** argv)
{
	string fnameVid, fnameFlist;
	int vid_FRAME_WIDTH;
	int vid_FRAME_HEIGHT;
	double vid_FPS;
	int vid_FRAME_COUNT;
	cv::Rect roiCrop; 
	int output_vid_width, output_vid_height;

	// Get video
	cout << "# Full path of video (space is allowed, 'g' for gui file dialog): ";
	fnameVid = readStringLineFromIstream(cin);
	if (fnameVid.size() == 1 && fnameVid[0] == 'g')
		fnameVid = uigetfile();
	cv::VideoCapture vid(fnameVid);
	if (vid.isOpened() == false)
	{
		cerr << "#  FuncVideo2Pics cannot read video: " << fnameVid << endl;
		return -1;
	}
	vid_FRAME_WIDTH = (int)(vid.get(cv::CAP_PROP_FRAME_WIDTH) + .5);
	vid_FRAME_HEIGHT = (int)(vid.get(cv::CAP_PROP_FRAME_HEIGHT) + .5);
	vid_FPS = vid.get(cv::CAP_PROP_FPS);
	vid_FRAME_COUNT = (int)(vid.get(cv::CAP_PROP_FRAME_COUNT) + .5);

	cout << "#          Width: " << vid_FRAME_WIDTH << endl;
	cout << "#         Height: " << vid_FRAME_HEIGHT << endl;
	cout << "# Frame per sec.: " << vid_FPS << endl;
	cout << "#    Frame count: " << vid_FRAME_COUNT << endl;

	// starting index and frame number
	int iStart, nFrame;
	cout << "# Starting frame (0-based) (negative value to quit): ";
	iStart = readIntFromIstream(cin);
	if (iStart < 0) return -1;
	cout << "# Number of frames to convert (negative value means to-the-end): ";
	nFrame = readIntFromIstream(cin);
	if (nFrame < 0)
		nFrame = vid_FRAME_COUNT - iStart;
	cout << "# Region of interests to crop (x y w h)(or 0 0 0 0 for full size): ";
	roiCrop.x = readIntFromIstream(cin);
	roiCrop.y = readIntFromIstream(cin);
	roiCrop.width = readIntFromIstream(cin);
	roiCrop.height = readIntFromIstream(cin);
	if (roiCrop.width <= 0 || roiCrop.height <= 0) 
		roiCrop = cv::Rect(0, 0, vid_FRAME_WIDTH, vid_FRAME_HEIGHT); 
	cout << "# Preferred video width and height (w h): ";
	output_vid_width = readIntFromIstream(cin);
	output_vid_height = readIntFromIstream(cin);

	// Picture files
	cout << "# Full path file names of pictures (in C-style format including a %d, space allowed): ";
	string fnamePicsFormat = readStringLineFromIstream(cin);

	// Read video and skip to frame iStart
	for (int i = 0; i < iStart; i++)
	{
		cv::Mat buf;
		vid >> buf;
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "# Skipping frame " << i << " ...";
	}

	// Start converting video to pictures
	char fnamePic[1000];
	for (int i = iStart; i < iStart + nFrame; i++)
	{
		if (i >= vid_FRAME_COUNT) break;
		cv::Mat buf;
		vid >> buf;
        snprintf(fnamePic, 1000, fnamePicsFormat.c_str(), i - iStart);
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "# Writing frame " << i << " ...";
		cv::resize(buf(roiCrop), buf, cv::Size(output_vid_width, output_vid_height), 0.0, 0.0, cv::INTER_LANCZOS4); 
		cv::imwrite(fnamePic, buf);
	}
	vid.release();
	return 0;
}



int FuncPics2VideoResize(int argc, char** argv)
{
	string fnameVid, fnameFlist;
	char fnamePic[1000];
	cv::Rect roiCrop; 
	int output_vid_width, output_vid_height; 

	// Picture files
	cout << "# Full path file names of pictures (in C-style format including a %d, space allowed): ";
	string fnamePicsFormat = readStringLineFromIstream(cin);

	// starting index and frame number
	int iStart, nFrame;
	cout << "# Starting frame (0-based) (negative value to quit): ";
	iStart = readIntFromIstream(cin);
	if (iStart < 0) return -1;
	cout << "# Number of frames to convert (negative value to quit): ";
	nFrame = readIntFromIstream(cin);
	if (nFrame < 0) return -1;

	// Get first image size
    snprintf(fnamePic, 1000, fnamePicsFormat.c_str(), iStart);
	cv::Mat img = cv::imread(fnamePic);
	cv::Size imgSize = img.size();
	cout << "# Image size of " << fnamePic << " is " << imgSize << endl;
	if (imgSize.width <= 0 || imgSize.height <= 0) {
		cerr << "#  Error: Image size of " << fnamePic << " is zero.\n";
		return -1;
	}

	// Get video
	cout << "# Full path of video (space is allowed, 'g' for gui file dialog): ";
	fnameVid = readStringLineFromIstream(cin);
	if (fnameVid.size() == 1 && fnameVid[0] == 'g')
		fnameVid = uiputfile();
	if (fnameVid.size() == 0) return -1;
	cout << "# Preferred frame rate (fps): ";
	double fps = readDoubleFromIstream(cin);
	cout << "# Region of interests to crop (x y w h)(or 0 0 0 0 for full size): ";
	roiCrop.x = readIntFromIstream(cin);
	roiCrop.y = readIntFromIstream(cin);
	roiCrop.width = readIntFromIstream(cin);
	roiCrop.height = readIntFromIstream(cin);
	if (roiCrop.width <= 0 || roiCrop.height <= 0)
		roiCrop = cv::Rect(0, 0, imgSize.width, imgSize.height);
	cout << "# Preferred video width and height (w h): ";
	output_vid_width = readIntFromIstream(cin); 
	output_vid_height = readIntFromIstream(cin);

	cv::VideoWriter vid(fnameVid, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, 
		cv::Size(output_vid_width, output_vid_height));
	if (vid.isOpened() == false)
	{
		cerr << "#  FuncPics2Video cannot open video: " << fnameVid << endl;
		return -1;
	}
	cout << "#          Width: " << output_vid_width << endl;
	cout << "#         Height: " << output_vid_height << endl;
	cout << "# Frame per sec.: " << fps << endl;

	// Start converting 
	for (int i = iStart; i < iStart + nFrame; i++)
	{
        snprintf(fnamePic, 1000, fnamePicsFormat.c_str(), i);
		img = cv::imread(fnamePic);
		if (img.cols <= 0 || img.rows <= 0) {
			cerr << "  " << fnamePic << " does not exist. Skipping this frame.\n";
			continue;
		}
		cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		cout << "# Writing frame " << i << " ...";
		cv::resize(img(roiCrop), img, cv::Size(output_vid_width, output_vid_height), 0.0, 0.0, cv::INTER_LANCZOS4); 
//		cv::imshow("1200", img); cv::waitKey(0);
		vid << img;
	}
	vid.release();
	return 0;
}
