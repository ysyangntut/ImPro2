#include <iostream>
#include <opencv2/opencv.hpp>
#include "impro_util.h"
#include "FileSeq.h"

using namespace std;

int FuncPip(int argc, char** argv)
{
	cv::Size canvasSize;
	cv::Mat canvas;
	uchar canvasColor[3]; 
	int nPip;
	int nFrame = 0; 
	vector<FileSeq> fseqs; 
	FileSeq outFseq; 
	vector<cv::Rect> picRects; // for each picture of series i, resize picRects[i].
	vector<cv::Rect> canvasRects; // and copy to canvas at canvasRects[i].

	cout << "# Enter canvas size (width height) (in pixels):\n";
	canvasSize.width = readIntFromCin(1, 99999);
	canvasSize.height = readIntFromCin(1, 99999);
	cout << "# Enter canvas background color (in BGR format, 255 255 255 for white; 0 0 0 for black):\n";
	canvasColor[0] = readIntFromCin(0, 255); 
	canvasColor[1] = readIntFromCin(0, 255);
	canvasColor[2] = readIntFromCin(0, 255);
	canvas = cv::Mat(canvasSize.height, canvasSize.width, CV_8UC3, cv::Scalar(canvasColor[0], canvasColor[1], canvasColor[2]));
	
	cout << "# Number of picture series: \n";
	nPip = readIntFromCin(1, 99);
	fseqs.resize(nPip); 
	picRects.resize(nPip);
	canvasRects.resize(nPip); 

	for (int i = 0; i < nPip; i++)
	{
		printf("\n# Define files of picture series %d/%d:\n", i + 1, nPip); 
		fseqs[i].setDirFilesByConsole(); 
		printf("\n# Enter the local Rect of picture in series %d/%d (x y width height):\n", i + 1, nPip); 
		picRects[i].x = readIntFromCin(0, 9999); 
		picRects[i].y = readIntFromCin(0, 9999);
		picRects[i].width = readIntFromCin(0, 9999);
		picRects[i].height = readIntFromCin(0, 9999);
		printf("\n# Enter the canvas Rect that you want to put picture series %d/%d in canvas (x y width height):\n", i + 1, nPip);
		canvasRects[i].x = readIntFromCin(0, 9999);
		canvasRects[i].y = readIntFromCin(0, 9999);
		canvasRects[i].width = readIntFromCin(0, 9999);
		canvasRects[i].height = readIntFromCin(0, 9999);
		// number of frames is the maximum number of files of all file sequences. 
		if (fseqs[i].num_files() > nFrame)
			nFrame = fseqs[i].num_files(); 
	}

	printf("\n# Define output file sequence: \n");
	outFseq.setDirFilesByConsole(); 

	for (int iFrame = 0; iFrame < nFrame; iFrame++)
	{
		canvas = cv::Mat(canvasSize.height, canvasSize.width, CV_8UC3, cv::Scalar(canvasColor[0], canvasColor[1], canvasColor[2]));
		for (int i = 0; i < nPip; i++)
		{
			cv::Mat pipCanvas = canvas(canvasRects[i]);
			cv::Mat picFromFile; 
			if (iFrame <= fseqs[i].num_files())
				picFromFile = cv::imread(fseqs[i].fullPathOfFile(iFrame));
			// check if valid. If not, continue next picture
			if (picFromFile.rows < picRects[i].y + picRects[i].height)
				continue;
			if (picFromFile.cols < picRects[i].x + picRects[i].width)
				continue;
			cv::Mat picOrisize = picFromFile(picRects[i]); 
			cv::Mat picResized = cv::Mat(canvasRects[i].height, canvasRects[i].width, CV_8UC3); 
			// resize picture
			cv::resize(picOrisize, picResized, cv::Size(canvasRects[i].width, canvasRects[i].height), 0., 0., cv::INTER_LANCZOS4); 
			picResized.copyTo(pipCanvas); 
		}
		cv::imshow("combined", canvas); cv::waitKey(10); 
		cv::imwrite(outFseq.fullPathOfFile(iFrame), canvas); 
	}
	cv::destroyWindow("combined"); 
	return 0; 
}




