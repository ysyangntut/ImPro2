#include "Points2fHistoryData.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include "impro_util.h"

#include "ImagePointsPicker.h"

using namespace std;

Points2fHistoryData::Points2fHistoryData()
{
	imgSize = cv::Size(0, 0); 
}

Points2fHistoryData::Points2fHistoryData(const cv::Mat & dataToClone)
{
	this->set(dataToClone);
}

Points2fHistoryData::Points2fHistoryData(const vector<cv::Mat>& dataToClone)
{
	this->set(dataToClone);
}

Points2fHistoryData::Points2fHistoryData(const vector<vector<cv::Point2f> > & dataToClone)
{
	this->set(dataToClone);
}

Points2fHistoryData::~Points2fHistoryData()
{
}

int Points2fHistoryData::readFromTxt(string fileTxt)
{
	int nStep = -1, nPoint = -1;
	// open input stream file. Returns -1 if fails to open. 
	ifstream ifTxt(fileTxt);
	if (ifTxt.is_open() == false)
		return -1;
	// read data size. Returns -1 if size is not positive.
	nStep = readIntFromIstream(ifTxt);
	nPoint = readIntFromIstream(ifTxt);
	if (nStep <= 0 || nPoint <= 0)
		return -1;
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_32FC2); 
	// read data 
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = 0; iPoint < nPoint; iPoint++) {
			this->dat.at<cv::Point2f>(iStep, iPoint).x = (float)readDoubleFromIstream(ifTxt);
			this->dat.at<cv::Point2f>(iStep, iPoint).y = (float)readDoubleFromIstream(ifTxt);
		}
	}
	// read rects
	for (int iPoint = 0; iPoint < nPoint; iPoint++) {
		if (ifTxt.eof()) break;
		int x = readIntFromIstream(ifTxt);
		int y = readIntFromIstream(ifTxt);
		int w = readIntFromIstream(ifTxt);
		int h = readIntFromIstream(ifTxt);
		if (ifTxt.eof()) break;
		this->rects.resize(iPoint + 1);
		this->rects[iPoint] = cv::Rect(x, y, w, h);
	}

	// close file
	ifTxt.close();
	return 0;
}

int Points2fHistoryData::writeToTxt(string fileTxt)
{
	int nStep = this->dat.rows;
	int nPoint = this->dat.cols;
	if (nStep <= 0 || nPoint <= 0)
		return -1;
	// open output stream file. Returns -1 if fails to open. 
	FILE * ofTxt;  
	errno_t err = fopen_s(&ofTxt, fileTxt.c_str(), "w"); 
	if (err != 0) 
		return -1;
	// write data size. Returns -1 if size is not positive.
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    fprintf_s(ofTxt, "# Number of steps(time steps)\n %d\n", nStep);
	fprintf_s(ofTxt, "# Number of points (cv::Point2f)\n %d\n", nPoint);
#else
    fprintf(ofTxt, "# Number of steps(time steps)\n %d\n", nStep);
    fprintf(ofTxt, "# Number of points (cv::Point2f)\n %d\n", nPoint);
#endif
	// write data 
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = 0; iPoint < nPoint; iPoint++) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
            fprintf_s(ofTxt, "%16.8e\t",  this->dat.at<cv::Point2f>(iStep, iPoint).x);
			fprintf_s(ofTxt, "%16.8e \t", this->dat.at<cv::Point2f>(iStep, iPoint).y);
#else
            fprintf(ofTxt, "%16.8e\t",  this->dat.at<cv::Point2f>(iStep, iPoint).x);
            fprintf(ofTxt, "%16.8e \t", this->dat.at<cv::Point2f>(iStep, iPoint).y);
#endif
       }
	}
	// initial template rects
	if (this->rects.size() > 0)
	{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        fprintf_s(ofTxt, "# Initial template rect (cv::Rect)\n");
#else
        fprintf(ofTxt, "# Initial template rect (cv::Rect)\n");
#endif
        for (int iPoint = 0; iPoint < nPoint; iPoint++) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
            fprintf_s(ofTxt, "%d %d %d %d\n",
				this->rects[iPoint].x, this->rects[iPoint].y, this->rects[iPoint].width, this->rects[iPoint].height);
#else
            fprintf(ofTxt, "%d %d %d %d\n",
                this->rects[iPoint].x, this->rects[iPoint].y, this->rects[iPoint].width, this->rects[iPoint].height);
#endif

        }
	}
	// close file
	fclose(ofTxt); 

	return 0;
}

int Points2fHistoryData::readFromXml(string fileXml)
{
	cv::FileStorage ifs(fileXml, cv::FileStorage::READ);
	ifs["Points2fHistoryData"] >> this->dat;
//	ifs["ImageWidth"] >> this->imgSize.width; 
//	ifs["ImageHeight"] >> this->imgSize.height;
	ifs["InitTemplate"] >> this->rects;
	ifs.release();
	return 0;
}

int Points2fHistoryData::writeToXml(string fileXml)
{
	cv::FileStorage ofs(fileXml, cv::FileStorage::WRITE);
	ofs << "Points2fHistoryData" << this->dat;
//	ofs << "ImageWidth" << this->imgSize.width; 
//	ofs << "ImageHeight" << this->imgSize.height;
	if (this->rects.size() > 0)
		ofs << "InitTemplate" << this->rects; 
	ofs.release();
	return 0;
}

int Points2fHistoryData::readThruUserInteraction()
{
	return readThruUserInteraction(-1, -1); 
}

int Points2fHistoryData::readThruUserInteraction(int nStep, int nPoint)
{
	// ask user to select source: 1. txt file, 2. xml file, 3. manual input 
	int sourceType;
	cout << "  Select source type: 1.txt file, 2.xml file, 3.manual input, 4.pick by mouse:\n";
	cout << "     txt format:\n"
	        "        nStep nPoint x1 y1 x2 y2 x3 y3 ... xnPoint ynPoint\n";
	cout << "     xml format:\n"
            "        <?xml version=\"1.0\"?>\n"
            "        <opencv_storage>\n"
            "        <Points2fHistoryData type_id = \"opencv-matrix\">\n"
            "        <rows> nStep </rows>\n"
            "        <cols> nPoint </cols>\n"
            "        <dt>\"2f\"</dt>\n"
            "        <data> x1 y1 x2 y2 ... xnPoint ynPoint </data></Points2fHistoryData>\n"
            "        </opencv_storage>\n";
	sourceType = readIntFromCin();
	// Reading from txt file
	if (sourceType == 1) {
		string fname;
		cout << "    Input txt file name (full-path):\n";
		fname = readStringLineFromCin();
		int readErrNo = this->readFromTxt(fname);
		if (readErrNo != 0) {
			cerr << "  Failed to read " << fname << endl;
			return readErrNo;
		}
		printf("  Read data from %s.\n   nStep = %d, nPoint = %d\n", 
			fname.c_str(), 
			this->dat.rows, this->dat.cols);
		return readErrNo;
	}
	// Reading from xml file
	if (sourceType == 2) {
		string fname;
		cout << "    Input xml file name (full-path):\n";
		fname = readStringLineFromCin();
		int readErrNo = this->readFromXml(fname);
		if (readErrNo != 0) {
			cerr << "  Failed to read " << fname << endl;
			return readErrNo;
		}
		printf("  Read data from %s.\n   nStep = %d, nPoint = %d\n",
			fname.c_str(),
			this->dat.rows, this->dat.cols);
		return readErrNo;
	}	
	// Reading from manual input (keyboard)
	if (sourceType == 3) {
		if (nStep <= 0) {
			cout << "    Input number of Steps:\n";
			nStep = readIntFromCin(1); // min is 1
		}
		if (nPoint <= 0) {
			cout << "    Input number of Points:\n";
			nPoint = readIntFromCin(1);
		}
		if (nStep <= 0 || nPoint <= 0) {
			cerr << "   Invalid input.\n";
			return -1;
		}
		this->dat = cv::Mat::zeros(nStep, nPoint, CV_32FC2); 
		for (int iStep = 0; iStep < nStep; iStep++) {
			for (int iPoint = 0; iPoint < nPoint; iPoint++) {
				printf("  Input points(%d,%d) (x and y):\n", iStep, iPoint);
				this->dat.at<cv::Point2f>(iStep, iPoint).x = (float) readDoubleFromCin();
				this->dat.at<cv::Point2f>(iStep, iPoint).y = (float)readDoubleFromCin();
			}
		}
		// save to file(s)
		string tfname, xfname;
		cout << "  Input xml file name (full-path) to save this data (single char to skip):";
		xfname = readStringLineFromCin();
		if (xfname.length() > 3)
			this->writeToXml(xfname);
		cout << "  Input txt file name (full-path) to save this data (single char to skip):";
		tfname = readStringLineFromCin();
		if (tfname.length() > 3)
			this->writeToTxt(tfname);
	}
	// Reading from mouse picking
	if (sourceType == 4) {
		ImagePointsPicker pp;
		if (nStep <= 0) {
			cout << "    Input number of Steps:\n";
			nStep = readIntFromCin(1); // min is 1
		}
		if (nPoint <= 0) {
			cout << "    Input number of Points:\n";
			nPoint = readIntFromCin(1);
		}
		if (nStep <= 0 || nPoint <= 0) {
			cerr << "   Invalid input.\n";
			return -1;
		}
		// allocate memory
		this->dat = cv::Mat::zeros(nStep, nPoint, CV_32FC2);
		// pick points
		for (int iStep = 0; iStep < nStep; iStep++) {
			// background image
			if (nStep == 1) 
				cout << "Input (full or relative path) of background image:\n";
			else
				cout << "Input (full or relative path) of background image of Step " << iStep + 1 << " (1-based):\n";
			string fnameBg = readStringLineFromCin();
			cv::Mat bg = cv::imread(fnameBg);
			if (bg.cols <= 0 || bg.rows <= 0) {
				cerr << "Cannot read the background image " << fnameBg << endl;
				return -1;
			}
			pp.setBackgroundImageByFilename(fnameBg);
			pp.pickPoints(nPoint); 
			for (int iPoint = 0; iPoint < nPoint; iPoint++) {
				this->dat.at<cv::Point2f>(iStep, iPoint) = 
					pp.pointsInPoint2fVector()[iPoint];
			}
		}
		// save to file(s)
		string jfname, tfname, xfname;
		cout << "  Input JPG file name (full-path) to save picked points (single char to skip):";
		jfname = readStringLineFromCin();
		if (jfname.length() > 3)
			pp.writeMarkersToImageFile(jfname); 
		this->writeToXml(xfname);
		cout << "  Input xml file name (full-path) to save this data (single char to skip):";
		xfname = readStringLineFromCin();
		if (xfname.length() > 3)
			this->writeToXml(xfname);
		cout << "  Input txt file name (full-path) to save this data (single char to skip):";
		tfname = readStringLineFromCin();
		if (tfname.length() > 3)
			this->writeToTxt(tfname);
	}
	return 0;
}

int Points2fHistoryData::writeThruUserInteraction()
{
	// ask user to select destination: 1. txt file, 2. xml file
	int destType;
	cout << "  Select output type: 1.txt file, 2.xml file:\n";
	destType = readIntFromCin();
	// Writing to txt file
	if (destType == 1) {
		string fname;
		cout << "    Input txt file name (full-path):\n";
		fname = readStringLineFromCin();
		int writeErrNo = this->writeToTxt(fname);
		if (writeErrNo != 0) {
			cerr << "  Failed to write to " << fname << endl;
			return writeErrNo;
		}
		printf("  Wrote data to %s.\n   nStep = %d, nPoint = %d\n",
			fname.c_str(),
			this->dat.rows, this->dat.cols);
		return writeErrNo;
	}
	// Reading from xml file
	if (destType == 2) {
		string fname;
		cout << "    Input xml file name (full-path):\n";
		fname = readStringLineFromCin();
		int writeErrNo = this->writeToXml(fname);
		if (writeErrNo != 0) {
			cerr << "  Failed to write to " << fname << endl;
			return writeErrNo;
		}
		printf("  Wrote data to %s.\n   nStep = %d, nPoint = %d\n",
			fname.c_str(),
			this->dat.rows, this->dat.cols);
		return writeErrNo;
	}
	return 0;
}

int Points2fHistoryData::writeScriptMat(string fname)
{ 
	return writeScriptMatAdvanced(fname, "");
}

int Points2fHistoryData::writeScriptMatAdvanced(string fname, string backgroundImgFile,
	bool drawPointNumber, int pointNumberBase, int onlyData, int iStep)
{
	// check data size
	char buf[1000];
	int nStep = this->dat.rows;
	int nPoint = this->dat.cols;
	if (nStep <= 0 || nPoint <= 0)
		return -1;
	// write data to matlab script
	ofstream ofile(fname);
	if (ofile.is_open() == false)
		return -1;
	// script of loading data 
	ofile << "nStep = " << nStep << "; " << endl;
	ofile << "nPoint = " << nPoint << "; " << endl;
	if (iStep < 0) // given iStep < 0 indincates writing all steps
	{
		ofile << "p2fHist = " << this->dat.reshape(1, 1) << ";" << endl;
		ofile << "p2fHist = reshape(p2fHist, [2 nPoint nStep]); \n";
	}
	else
	{
		ofile << "if (exist('p2fHist','var') == 0 || size(p2fHist, 3) < nStep)\n\tp2fHist = zeros([2 nPoint nStep]);\nend\n"; // make sure p2fHist size is full size 
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
		{
            snprintf(buf, 1000, "p2fHist(1,%d,%d)=%f;",
				iPoint + 1, iStep + 1, this->dat.at<cv::Point2f>(iStep, iPoint).x);
			ofile << buf;
            snprintf(buf, 1000, "p2fHist(2,%d,%d)=%f;",
				iPoint + 1, iStep + 1, this->dat.at<cv::Point2f>(iStep, iPoint).y);
			ofile << buf;
		}
		ofile << endl;
	}
	ofile << "clear p2fHistx; p2fHistx(1:nPoint,1:nStep) = p2fHist(1,1:nPoint,1:nStep);\n";
	ofile << "clear p2fHisty; p2fHisty(1:nPoint,1:nStep) = p2fHist(2,1:nPoint,1:nStep);\n";
	if (onlyData == 1)
		return 0;
	// script of reading background image
	ofile << "bgfile = '" << backgroundImgFile << "';" << endl;
	ofile << "clear img; img = imread(bgfile); " << endl;
	// plot points of each step
	ofile << "for iStep = [1 nStep] " << endl;
	ofile << "  hfig = figure('name', ['Step ' num2str(iStep)]); " << endl;
	ofile << "  if (isempty(img) == false); imshow(img); hold on; end; " << endl;
	ofile << "  clear px; px(1:nPoint) = p2fHistx(1:nPoint,iStep); " << endl;
	ofile << "  clear py; py(1:nPoint) = p2fHisty(1:nPoint,iStep); " << endl;
	ofile << "  plot(px + 1, py + 1, 'o', 'LineWidth', 2); " << endl;
	ofile << "  axis('image'); set(gca, 'YDir', 'reverse'); " << endl;
	if (drawPointNumber) {
		ofile << "  pointNumberBase = " << pointNumberBase << ";" << endl;
		ofile << "  for iPoint = 1: nPoint" << endl;
		ofile << "    text(px(iPoint) + 1, py(iPoint) + 1, num2str(iPoint + pointNumberBase - 1)); " << endl;
		ofile << "  end" << endl;
	}
	ofile << "end" << endl;
	// plot relative movement (history) of each point 
	// x plot
	ofile << "hxfig = figure('name', 'Displacement history X'); " << endl;
	ofile << "clear pointLegend; iLegend = 0;" << endl;
	ofile << "for iPoint = 1:nPoint " << endl;
	ofile << "  plot(1:nStep, p2fHistx(iPoint, :) - p2fHistx(iPoint, 1), 'LineWidth', 2); " << endl;
	ofile << "  hold on; " << endl;
	ofile << "  iLegend = iLegend + 1; " << endl;
	ofile << "  pointLegend{ iLegend } = ['Point ' num2str(iPoint + pointNumberBase - 1)]; " << endl;
	ofile << "end" << endl;
	ofile << "xlabel('Time Step'); ylabel('Relative movement (pixel)'); " << endl;
	ofile << "legend(pointLegend);" << endl;
	// y plot
	ofile << "hyfig = figure('name', 'Displacement history Y'); " << endl;
	ofile << "clear pointLegend; iLegend = 0;" << endl;
	ofile << "for iPoint = 1:nPoint " << endl;
	ofile << "  plot(1:nStep, p2fHisty(iPoint, :) - p2fHisty(iPoint, 1), 'LineWidth', 2); " << endl;
	ofile << "  hold on; " << endl;
	ofile << "  iLegend = iLegend + 1; " << endl;
	ofile << "  pointLegend{ iLegend } = ['Point ' num2str(iPoint + pointNumberBase - 1)]; " << endl;
	ofile << "end" << endl;
	ofile << "xlabel('Time Step'); ylabel('Relative movement (pixel)'); " << endl;
	ofile << "legend(pointLegend);" << endl;

	return 0;
}

int Points2fHistoryData::nStep()
{
	return this->dat.rows;
}

int Points2fHistoryData::nPoint()
{
	return this->dat.cols;
}

int Points2fHistoryData::resize(int nStep, int nPoint)
{
	// allocate a new Mat
	cv::Mat tmp = cv::Mat::zeros(nStep, nPoint, CV_32FC2);
	// copy original data to the new Mat
	for (int i = 0; i < std::min(this->dat.rows, nStep); i++)
		for (int j = 0; j < std::min(this->dat.cols, nPoint); j++)
			tmp.at<cv::Point2f>(i, j) = this->dat.at<cv::Point2f>(i, j); 
	// replace original data with the new Mat
	tmp.copyTo(this->dat); 
	// click memory reallocation count
	this->memoryReallocationClick(); 
	return 0;
}

cv::Point2f Points2fHistoryData::get(int iStep, int iPoint)
{
	if (iStep >= this->dat.rows || iPoint >= this->dat.cols)
		return cv::Point2f(std::nanf(""), std::nanf(""));
	return this->dat.at<cv::Point2f>(iStep, iPoint);
}

cv::Rect Points2fHistoryData::getRect(int iPoint) const 
{
	if (this->rects.size() >= iPoint + 1)
		return rects[iPoint];
	else
		return cv::Rect(0, 0, 0, 0);
}

int Points2fHistoryData::set(int iStep, int iPoint, cv::Point2f p)
{
	if (iStep >= this->dat.rows || iPoint >= this->dat.cols)
		this->resize(
			std::max(this->dat.rows, iStep + 1), 
			std::max(this->dat.cols, iPoint + 1) ); 
	this->dat.at<cv::Point2f>(iStep, iPoint) = p;
	return 0;
}

int Points2fHistoryData::set(int iStep, int iPoint, cv::Point2d p)
{
	cv::Point2f pf((float)p.x, (float)p.y); 
	this->set(iStep, iPoint, pf); 
	return 0;
}

cv::Mat & Points2fHistoryData::getMat()
{
	return this->dat;
}

vector<cv::Mat> Points2fHistoryData::getVecMat()
{
	int nStep = this->dat.rows;
	int nPoint = this->dat.cols;
	vector<cv::Mat> vecmat(nStep);
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		vecmat[iStep] = cv::Mat(1, nPoint, CV_32FC2);
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
			vecmat[iStep].at<cv::Point2f>(0, iPoint) = this->dat.at<cv::Point2f>(iStep, iPoint);
	}
	return vecmat; 
}

vector<vector<cv::Point2f>> Points2fHistoryData::getVecVec()
{
	int nStep = this->dat.rows;
	int nPoint = this->dat.cols;
	vector<vector<cv::Point2f> > vecvec(nStep);
	for (int iStep = 0; iStep < nStep; iStep++) 
	{
		vecvec[iStep].resize(nPoint); 
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
			vecvec[iStep][iPoint] = this->dat.at<cv::Point2f>(iStep, iPoint);
	}
	return vecvec;
}

int Points2fHistoryData::set(const cv::Mat & theDat)
{
	if (theDat.type() == CV_32FC2)
		theDat.copyTo(this->dat);
	else if (theDat.type() == CV_64FC2)
		theDat.convertTo(this->dat, CV_32FC2);
	else
	{
		cerr << "  Points2fHistoryData::set(): Cannot convert data type from "
			<< theDat.type() << " to CV_32FC2.\n";
		return -1;
	}
	return 0;
}

int Points2fHistoryData::set(const vector<cv::Mat>& theDat)
{
	// check theDat data type
	if (theDat[0].type() != CV_32FC2 && theDat[0].type() != CV_64FC2)
	{
		cerr << "  Points2fHistoryData::set(): Cannot convert data type from "
			<< theDat[0].type() << " to CV_32FC2.\n";
		return -1;
	}

	// find data dimension (nStep, nPoint)
	int nStep = (int) theDat.size();
	int nPoint = 0; 
	for (int iStep = 0; iStep < nStep; iStep++)
		if (theDat[iStep].cols > nPoint) nPoint = theDat[iStep].cols; 
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_32FC2);
	// copy data
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		for (int iPoint = 0; iPoint < theDat[iStep].cols; iPoint++)
		{
			if (theDat[iStep].type() == CV_32FC2)
				this->dat.at<cv::Point2f>(iStep, iPoint) = theDat[iStep].at<cv::Point2f>(0, iPoint);
			if (theDat[iStep].type() == CV_64FC2) {
				this->dat.at<cv::Point2f>(iStep, iPoint).x = (float)theDat[iStep].at<cv::Point2d>(0, iPoint).x;
				this->dat.at<cv::Point2f>(iStep, iPoint).y = (float)theDat[iStep].at<cv::Point2d>(0, iPoint).y;
			}
		}
	}
	return 0;
}

int Points2fHistoryData::set(const vector<vector<cv::Point2f> > & theDat)
{
	// find data dimension (nStep, nPoint)
	int nStep = (int) theDat.size();
	int nPoint = 0;
	for (int iStep = 0; iStep < nStep; iStep++)
		if ((int) theDat[iStep].size() > nPoint) nPoint = (int) theDat[iStep].size();
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_32FC2);
	// copy data
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		for (int iPoint = 0; iPoint < theDat[iStep].size(); iPoint++)
		{
			this->dat.at<cv::Point2f>(iStep, iPoint) = theDat[iStep][iPoint];
		}
	}
	return 0;
}

int Points2fHistoryData::set(const vector<vector<cv::Point2d> > & theDat)
{
	// find data dimension (nStep, nPoint)
	int nStep = (int)theDat.size();
	int nPoint = 0;
	for (int iStep = 0; iStep < nStep; iStep++)
		if ((int)theDat[iStep].size() > nPoint) nPoint = (int)theDat[iStep].size();
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_32FC2);
	// copy data
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		for (int iPoint = 0; iPoint < theDat[iStep].size(); iPoint++)
		{
			this->dat.at<cv::Point2f>(iStep, iPoint).x = (float) theDat[iStep][iPoint].x;
			this->dat.at<cv::Point2f>(iStep, iPoint).y = (float) theDat[iStep][iPoint].y;
		}
	}
	return 0;
}

int Points2fHistoryData::setRect(int iPoint, cv::Rect rect)
{
	if (this->rects.size() < iPoint + 1)
		this->rects.resize(max(iPoint + 1, this->dat.cols)); 
	this->rects[iPoint] = rect; 
	return 0;
}

int Points2fHistoryData::appendLine(cv::Point2f p0, cv::Point2f p1, int nPointAdd)
{
	// reallocate data size
	int nStep = this->dat.rows;
	int nPointOld = this->dat.cols;
	int nPoint = this->dat.cols + nPointAdd;
	this->resize(nStep, nPoint);
	// set interpolated points
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = nPointOld; iPoint < nPoint; iPoint++) {
			cv::Point2f tmp = (1.f - (iPoint - nPointOld) * 1.f / nPointAdd) * p0 +
				(iPoint - nPointOld) * 1.f / nPointAdd * p1; 
			this->set(iStep, iPoint, tmp); 
		}
	}
	return 0;
}

int Points2fHistoryData::appendQ4(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, int nPoint01, int nPoint12)
{
	// reallocate data size
	int nStep = this->dat.rows;
	int nPointOld = this->dat.cols;
	int nPoint = this->dat.cols + nPoint01 * nPoint12;
	this->resize(nStep, nPoint);
	// generate q4 points (with perspective effect)
	vector<cv::Point2f> p4(4);
	p4[0] = p0; p4[1] = p1; p4[2] = p2; p4[3] = p3;
	vector<cv::Point2f> q4 = interpQ4(p4, nPoint01, nPoint12);
	// set new points
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = nPointOld; iPoint < nPoint; iPoint++) {
			this->set(iStep, iPoint, q4[iPoint - nPointOld]);
		}
	}
	return 0;
}


void testPoints2fHistoryData()
{
	// 
	Points2fHistoryData a, b;
	a.resize(3, 5);
	a.set(2, 2, cv::Point2f(3.f, 5.f));
	a.writeToTxt("d:\\temp\\a.txt");
	b.readFromTxt("d:\\temp\\a.txt");
	b.writeToTxt("d:\\temp\\b.txt");

	a.set(1, 1, cv::Point2f(9.f, 2.f));
	a.writeToXml("d:\\temp\\a.xml");
	b.readFromXml("d:\\temp\\a.xml");
	b.writeToXml("d:\\temp\\b.xml");

	a.set(0, 1, cv::Point2f(7.f, 3.f));
	vector<unsigned char> bin = a.serialize();
	b.deserialize(bin);
	b.writeToXml("d:\\temp\\b.xml");

	a.appendLine(cv::Point2f(0.f, 0.f), cv::Point2f(100.f, 1000.f), 50);

	a.appendQ4(cv::Point2f(200.f, 400.f), cv::Point2f(400.f, 200.f),
		cv::Point2f(1000, 500), cv::Point2f(500, 1000), 20, 30);

	a.writeScriptMatAdvanced("d:\\temp\\a.m",
		"E:\\ExpDataSamples\\20170809-OpeningDemo\\YangYS\\RawData\\Test1_Cam1\\T1C1_0000.JPG",
		true, 100);
}
