#include "Points3dHistoryData.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include "impro_util.h"

using namespace std;

Points3dHistoryData::Points3dHistoryData()
{
}

Points3dHistoryData::Points3dHistoryData(const cv::Mat & dataToClone)
{
	this->set(dataToClone);
}

Points3dHistoryData::Points3dHistoryData(const vector<cv::Mat>& dataToClone)
{
	this->set(dataToClone);
}

Points3dHistoryData::Points3dHistoryData(const vector<vector<cv::Point3d> > & dataToClone)
{
	this->set(dataToClone);
}

Points3dHistoryData::~Points3dHistoryData()
{
}

int Points3dHistoryData::readFromTxt(string fileTxt)
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
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
	// read data 
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = 0; iPoint < nPoint; iPoint++) {
			this->dat.at<cv::Point3d>(iStep, iPoint).x = (double)readDoubleFromIstream(ifTxt);
			this->dat.at<cv::Point3d>(iStep, iPoint).y = (double)readDoubleFromIstream(ifTxt);
			this->dat.at<cv::Point3d>(iStep, iPoint).z = (double)readDoubleFromIstream(ifTxt);
		}
	}
	// close file
	ifTxt.close();
	return 0;
}

int Points3dHistoryData::writeToTxt(string fileTxt)
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
	fprintf_s(ofTxt, "# Number of points (cv::Point3d)\n %d\n", nPoint);
#else
    fprintf(ofTxt, "# Number of steps(time steps)\n %d\n", nStep);
    fprintf(ofTxt, "# Number of points (cv::Point3d)\n %d\n", nPoint);
#endif
	// write data 
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = 0; iPoint < nPoint; iPoint++) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
            fprintf_s(ofTxt, "%24.16e\t",  this->dat.at<cv::Point3d>(iStep, iPoint).x);
			fprintf_s(ofTxt, "%24.16e\t",  this->dat.at<cv::Point3d>(iStep, iPoint).y);
			fprintf_s(ofTxt, "%24.16e \t", this->dat.at<cv::Point3d>(iStep, iPoint).z);
#else
            fprintf(ofTxt, "%24.16e\t",  this->dat.at<cv::Point3d>(iStep, iPoint).x);
            fprintf(ofTxt, "%24.16e\t",  this->dat.at<cv::Point3d>(iStep, iPoint).y);
            fprintf(ofTxt, "%24.16e \t", this->dat.at<cv::Point3d>(iStep, iPoint).z);
#endif
		}
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        fprintf_s(ofTxt, "\n");
#else
        fprintf(ofTxt, "\n");
#endif
	}
	// close file
	fclose(ofTxt);

	return 0;
}

int Points3dHistoryData::readFromXml(string fileXml)
{
	cv::FileStorage ifs(fileXml, cv::FileStorage::READ);
	ifs["Points3dHistoryData"] >> this->dat;
	ifs.release();
	return 0;
}

int Points3dHistoryData::writeToXml(string fileXml)
{
	cv::FileStorage ofs(fileXml, cv::FileStorage::WRITE);
	ofs << "Points3dHistoryData" << this->dat;
	ofs.release();
	return 0;
}

int Points3dHistoryData::readThruUserInteraction()
{
	return this->readThruUserInteraction(-1, -1); 
}
int Points3dHistoryData::readThruUserInteraction(int nStep, int nPoint)
{
	// ask user to select source: 1. txt file, 2. xml file, 3. manual input 
	int sourceType;
	cout << "  Select source type: 1.txt file, 2.xml file, 3.manual input:\n";
	cout << "     txt format:\n"
		"        nStep nPoint x1 y1 z1 x2 y2 z2 x3 y3 z3 ... xnPoint ynPoint\n";
	cout << "     xml format:\n"
		"        <?xml version=\"1.0\"?>\n"
		"        <opencv_storage>\n"
		"        <Points3dHistoryData type_id = \"opencv-matrix\">\n"
		"        <rows> nStep </rows>\n"
		"        <cols> nPoint </cols>\n"
		"        <dt>\"3d\"</dt>\n"
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
		this->dat = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
		for (int iStep = 0; iStep < nStep; iStep++) {
			for (int iPoint = 0; iPoint < nPoint; iPoint++) {
				printf("  Input points(Step %d, Point %d) (1-based) (format: x  y  z ):\n", iStep + 1, iPoint + 1);
				this->dat.at<cv::Point3d>(iStep, iPoint).x = (double)readDoubleFromCin();
				this->dat.at<cv::Point3d>(iStep, iPoint).y = (double)readDoubleFromCin();
				this->dat.at<cv::Point3d>(iStep, iPoint).z = (double)readDoubleFromCin();
			}
		}
		// save to file(s)
		string tfname, xfname;
		cout << "  Input xml file name (full-path) to save this data (single char to skip):\n";
		xfname = readStringLineFromCin();
		if (xfname.length() > 3)
			this->writeToXml(xfname);
		cout << "  Input txt file name (full-path) to save this data (single char to skip):\n";
		tfname = readStringLineFromCin();
		if (tfname.length() > 3)
			this->writeToTxt(tfname);
	}
	return 0;
}

int Points3dHistoryData::writeThruUserInteraction()
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

int Points3dHistoryData::writeScriptMat(string fname)
{
	return writeScriptMatAdvanced(fname, true, 1);
}

int Points3dHistoryData::writeScriptMatAdvanced(string fname, 
	bool drawPointNumber, int pointNumberBase, int onlyData, 
	int iStep)
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
		ofile << "p3dHist = " << this->dat.reshape(1, 1) << ";" << endl;
		ofile << "p3dHist = reshape(p3dHist, [3 nPoint nStep]); \n";
	}
	else {
		ofile << "if (exist('p3dHist','var') == 0 || size(p3dHist, 3) < nStep)\n\tp3dHist = zeros([3 nPoint nStep]);\nend\n"; // make sure p3dHist size is full size 
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
		{
            snprintf(buf, 1000, "p3dHist(1,%d,%d)=%f;",
				iPoint + 1, iStep + 1, this->dat.at<cv::Point3d>(iStep, iPoint).x);
			ofile << buf;
            snprintf(buf, 1000, "p3dHist(2,%d,%d)=%f;",
				iPoint + 1, iStep + 1, this->dat.at<cv::Point3d>(iStep, iPoint).y);
			ofile << buf;
            snprintf(buf, 1000, "p3dHist(3,%d,%d)=%f;",
				iPoint + 1, iStep + 1, this->dat.at<cv::Point3d>(iStep, iPoint).z);
			ofile << buf;
		}
		ofile << endl;
	}
	ofile << "clear p3dHistx; p3dHistx(:,:) = p3dHist(1,:,:);\n";
	ofile << "clear p3dHisty; p3dHisty(:,:) = p3dHist(2,:,:);\n";
	ofile << "clear p3dHistz; p3dHistz(:,:) = p3dHist(3,:,:);\n";
	if (onlyData == 1)
		return 0; 
	// plot points of each step
	ofile << "for iStep = [1 nStep] " << endl;
	ofile << "  hfig = figure('name', ['Step ' num2str(iStep)]);" << endl;
	ofile << "  clear px; px(:) = p3dHistx(:,iStep); " << endl;
	ofile << "  clear py; py(:) = p3dHisty(:,iStep); " << endl;
	ofile << "  clear pz; pz(:) = p3dHistz(:,iStep); " << endl;
	ofile << "  plot3(px + 1, py + 1, pz + 1, 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 2); " << endl;
	ofile << "  axis equal; grid on;" << endl;
	if (drawPointNumber) {
		ofile << "  pointNumberBase = " << pointNumberBase << ";" << endl;
		ofile << "  for iPoint = 1: nPoint" << endl;
		ofile << "    text(px(iPoint) + 1, py(iPoint) + 1, pz(iPoint) + 1, num2str(iPoint + pointNumberBase - 1)); " << endl;
		ofile << "  end" << endl;
	}
	ofile << "end" << endl;
	// plot relative movement (history) of each point 
	// x plot
	ofile << "hxfig = figure('name', 'Displacement history X'); " << endl;
	ofile << "clear pointLegend; iLegend = 0;" << endl;
	ofile << "for iPoint = 1:nPoint " << endl;
	ofile << "  plot(1:nStep, p3dHistx(iPoint, :) - p3dHistx(iPoint, 1), 'LineWidth', 2); " << endl;
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
	ofile << "  plot(1:nStep, p3dHisty(iPoint, :) - p3dHisty(iPoint, 1), 'LineWidth', 2); " << endl;
	ofile << "  hold on; " << endl;
	ofile << "  iLegend = iLegend + 1; " << endl;
	ofile << "  pointLegend{ iLegend } = ['Point ' num2str(iPoint + pointNumberBase - 1)]; " << endl;
	ofile << "end" << endl;
	ofile << "xlabel('Time Step'); ylabel('Relative movement (pixel)'); " << endl;
	ofile << "legend(pointLegend);" << endl;
	// z plot
	ofile << "hzfig = figure('name', 'Displacement history Z'); " << endl;
	ofile << "clear pointLegend; iLegend = 0;" << endl;
	ofile << "for iPoint = 1:nPoint " << endl;
	ofile << "  plot(1:nStep, p3dHistz(iPoint, :) - p3dHistz(iPoint, 1), 'LineWidth', 2); " << endl;
	ofile << "  hold on; " << endl;
	ofile << "  iLegend = iLegend + 1; " << endl;
	ofile << "  pointLegend{ iLegend } = ['Point ' num2str(iPoint + pointNumberBase - 1)]; " << endl;
	ofile << "end" << endl;
	ofile << "xlabel('Time Step'); ylabel('Relative movement (pixel)'); " << endl;
	ofile << "legend(pointLegend);" << endl;

	return 0;
}

int Points3dHistoryData::nStep()
{
	return this->dat.rows;
}

int Points3dHistoryData::nPoint()
{
	return this->dat.cols;
}

int Points3dHistoryData::resize(int nStep, int nPoint)
{
	// allocate a new Mat
	cv::Mat tmp = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
	// copy original data to the new Mat
	for (int i = 0; i < std::min(this->dat.rows, nStep); i++)
		for (int j = 0; j < std::min(this->dat.cols, nPoint); j++)
			tmp.at<cv::Point3d>(i, j) = this->dat.at<cv::Point3d>(i, j);
	// replace original data with the new Mat
	tmp.copyTo(this->dat);
	// click memory reallocation count
	this->memoryReallocationClick();
	return 0;
}

cv::Point3d Points3dHistoryData::get(int iStep, int iPoint)
{
	if (iStep >= this->dat.rows || iPoint >= this->dat.cols)
		return cv::Point3d(std::nanf(""), std::nanf(""), std::nanf(""));
	return this->dat.at<cv::Point3d>(iStep, iPoint);
}

int Points3dHistoryData::set(int iStep, int iPoint, cv::Point3d p)
{
	if (iStep >= this->dat.rows || iPoint >= this->dat.cols)
		this->resize(
			std::max(this->dat.rows, iStep + 1),
			std::max(this->dat.cols, iPoint + 1));
	this->dat.at<cv::Point3d>(iStep, iPoint) = p;
	return 0;
}

int Points3dHistoryData::set(int iStep, int iPoint, cv::Point3f p)
{
	cv::Point3d pd((double)p.x, (double)p.y, (double)p.z);
	this->set(iStep, iPoint, pd);
	return 0;
}

cv::Mat & Points3dHistoryData::getMat()
{
	return this->dat;
}

vector<cv::Mat> Points3dHistoryData::getVecMat()
{
	int nStep = this->dat.rows;
	int nPoint = this->dat.cols;
	vector<cv::Mat> vecmat(nStep);
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		vecmat[iStep] = cv::Mat(1, nPoint, CV_64FC3);
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
			vecmat[iStep].at<cv::Point3d>(0, iPoint) = this->dat.at<cv::Point3d>(iStep, iPoint);
	}
	return vecmat;
}

vector<vector<cv::Point3d>> Points3dHistoryData::getVecVec()
{
	int nStep = this->dat.rows;
	int nPoint = this->dat.cols;
	vector<vector<cv::Point3d> > vecvec(nStep);
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		vecvec[iStep].resize(nPoint);
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
			vecvec[iStep][iPoint] = this->dat.at<cv::Point3d>(iStep, iPoint);
	}
	return vecvec;
}

int Points3dHistoryData::set(const cv::Mat & theDat)
{
	if (theDat.type() == CV_64FC3)
		theDat.copyTo(this->dat);
	else if (theDat.type() == CV_64FC2)
		theDat.convertTo(this->dat, CV_64FC3);
	else
	{
		cerr << "  Points3dHistoryData::set(): Cannot convert data type from "
			<< theDat.type() << " to CV_64FC3.\n";
		return -1;
	}
	return 0;
}

int Points3dHistoryData::set(const vector<cv::Mat>& theDat)
{
	// check theDat data type
	if (theDat[0].type() != CV_64FC3 && theDat[0].type() != CV_64FC2)
	{
		cerr << "  Points3dHistoryData::set(): Cannot convert data type from "
			<< theDat[0].type() << " to CV_64FC3.\n";
		return -1;
	}

	// find data dimension (nStep, nPoint)
	int nStep = (int)theDat.size();
	int nPoint = 0;
	for (int iStep = 0; iStep < nStep; iStep++)
		if (theDat[iStep].cols > nPoint) nPoint = theDat[iStep].cols;
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
	// copy data
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		for (int iPoint = 0; iPoint < theDat[iStep].cols; iPoint++)
		{
			if (theDat[iStep].type() == CV_64FC3)
				this->dat.at<cv::Point3d>(iStep, iPoint) = theDat[iStep].at<cv::Point3d>(0, iPoint);
			if (theDat[iStep].type() == CV_64FC2) {
				this->dat.at<cv::Point3d>(iStep, iPoint).x = (float)theDat[iStep].at<cv::Point3f>(0, iPoint).x;
				this->dat.at<cv::Point3d>(iStep, iPoint).y = (float)theDat[iStep].at<cv::Point3f>(0, iPoint).y;
				this->dat.at<cv::Point3d>(iStep, iPoint).z = (float)theDat[iStep].at<cv::Point3f>(0, iPoint).z;
			}
		}
	}
	return 0;
}

int Points3dHistoryData::set(const vector<vector<cv::Point3d> > & theDat)
{
	// find data dimension (nStep, nPoint)
	int nStep = (int)theDat.size();
	int nPoint = 0;
	for (int iStep = 0; iStep < nStep; iStep++)
		if ((int)theDat[iStep].size() > nPoint) nPoint = (int)theDat[iStep].size();
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
	// copy data
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		for (int iPoint = 0; iPoint < theDat[iStep].size(); iPoint++)
		{
			this->dat.at<cv::Point3d>(iStep, iPoint) = theDat[iStep][iPoint];
		}
	}
	return 0;
}

int Points3dHistoryData::set(const vector<vector<cv::Point3f> > & theDat)
{
	// find data dimension (nStep, nPoint)
	int nStep = (int)theDat.size();
	int nPoint = 0;
	for (int iStep = 0; iStep < nStep; iStep++)
		if ((int)theDat[iStep].size() > nPoint) nPoint = (int)theDat[iStep].size();
	// allocate data 
	this->dat = cv::Mat::zeros(nStep, nPoint, CV_64FC3);
	// copy data
	for (int iStep = 0; iStep < nStep; iStep++)
	{
		for (int iPoint = 0; iPoint < theDat[iStep].size(); iPoint++)
		{
			this->dat.at<cv::Point3d>(iStep, iPoint).x = (double)theDat[iStep][iPoint].x;
			this->dat.at<cv::Point3d>(iStep, iPoint).y = (double)theDat[iStep][iPoint].y;
			this->dat.at<cv::Point3d>(iStep, iPoint).y = (double)theDat[iStep][iPoint].z;
		}
	}
	return 0;
}

int Points3dHistoryData::appendLine(cv::Point3d p0, cv::Point3d p1, int nPointAdd)
{
	// reallocate data size
	int nStep = this->dat.rows;
	int nPointOld = this->dat.cols;
	int nPoint = this->dat.cols + nPointAdd;
	this->resize(nStep, nPoint);
	// set interpolated points
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = nPointOld; iPoint < nPoint; iPoint++) {
			cv::Point3d tmp = (1. - (iPoint - nPointOld) * 1. / nPointAdd) * p0 +
				(iPoint - nPointOld) * 1. / nPointAdd * p1;
			this->set(iStep, iPoint, tmp);
		}
	}
	return 0;
}

int Points3dHistoryData::appendQ4(cv::Point3d p0, cv::Point3d p1, cv::Point3d p2, cv::Point3d p3, int nPoint01, int nPoint12)
{
	// reallocate data size
	int nStep = this->dat.rows;
	int nPointOld = this->dat.cols;
	int nPoint = this->dat.cols + nPoint01 * nPoint12;
	this->resize(nStep, nPoint);
	// generate q4 points (with perspective effect)
	vector<cv::Point3d> p4(4);
	p4[0] = p0; p4[1] = p1; p4[2] = p2; p4[3] = p3;
	vector<cv::Point3d> q4 = interpQ43d(p4, nPoint01, nPoint12);
	// set new points
	for (int iStep = 0; iStep < nStep; iStep++) {
		for (int iPoint = nPointOld; iPoint < nPoint; iPoint++) {
			this->set(iStep, iPoint, q4[iPoint - nPointOld]);
		}
	}
	return 0;
}

void testPoints3dHistoryData()
{
	// 
	Points3dHistoryData a, b;
	a.resize(3, 5);
	a.set(2, 2, cv::Point3d(3., 4., 5.));
	a.writeToTxt("d:\\temp\\a.txt");
	b.readFromTxt("d:\\temp\\a.txt");
	b.writeToTxt("d:\\temp\\b.txt");

	a.set(1, 1, cv::Point3d(9., 5., 2.));
	a.writeToXml("d:\\temp\\a.xml");
	b.readFromXml("d:\\temp\\a.xml");
	b.writeToXml("d:\\temp\\b.xml");

	a.set(0, 1, cv::Point3d(7., 5., 3.));
	vector<unsigned char> bin = a.serialize();
	b.deserialize(bin);
	b.writeToXml("d:\\temp\\b.xml");

	a.appendLine(cv::Point3d(0, 0, 0), cv::Point3d(100, 1000, 0), 50);

	a.appendQ4(cv::Point3d(200, 400, 0), cv::Point3d(400, 200, 0),
		cv::Point3d(1000, 500, 1000), cv::Point3d(500, 1000, 1000), 20, 30);

	a.writeScriptMatAdvanced("d:\\temp\\a.m",
		true, 100);
}
