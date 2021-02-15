#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>

#include "pickAPoint.h"
#include "impro_util.h"
#include "ImagePointsPicker.h"

// #include "basic_def.h"
using namespace std; 

const float _nanf = nanf("");

ImagePointsPicker::ImagePointsPicker()
{
	this->nPoint = 0;
}


ImagePointsPicker::~ImagePointsPicker()
{
}

int ImagePointsPicker::setBackgroundImageByFilename(const std::string filename)
{
	this->imgfile = filename; 
	// try to read 
	int ok = refreshMarkedImage(); 
	if (ok != 0) return ok; 
	// output file name
	this->imgfile_marked = imgfile.substr(0, imgfile.size() - 4) + "_marked.JPG";
	this->txtfile = imgfile.substr(0, imgfile.size() - 4) + "_pickedPoints.txt";
	this->xmlfile = imgfile.substr(0, imgfile.size() - 4) + "_pickedPoints.xml";
	return 0;
}

// int ImagePointsPicker::setBackgroundImageByMat(const cv::Mat & _img)
// {
// 	this->img = _img.clone();
// 	this->imgfile = ""; 
// 	// try to read 
// 	int ok = refreshMarkedImage();
// 	if (ok != 0) return ok;
// 	// output file name
// 	this->imgfile_marked = "defaultImagePointsPicker_marked.JPG";
// 	this->txtfile = "defaultImagePointsPicker_pickedPoints.txt";
// 	this->xmlfile = "defaultImagePointsPicker_pickedPoints.xml";
// 	return 0;
// }

int ImagePointsPicker::pickPoints(int nPoint, int bKey, int cKey1, int cKey2)
{
	int nonSkippedPoint = 0; 
	this->nPoint = nPoint; 

	// Read background photo and refresh marked image
	this->refreshMarkedImage();

	// Pick points in a for-loop
	this->points = vector<cv::Point2f>(this->nPoint, cv::Point2f(_nanf, _nanf));
	for (int i = 0; i < this->nPoint; i++) {
		char strbuf[10000];
        snprintf(strbuf, 10000, "Pick Point %d of %d", i + 1, this->nPoint);
		std::string figtitle = strbuf;
		cv::Point2f point(_nanf, _nanf);
		// pick
		int key = pickAPoint(figtitle, this->img_marked, point, bKey, cKey1, cKey2);
		// check 
		if (key == 27) {
			continue;
		}
		// add to points array
		if (this->points.size() < i + 1)
			this->points.resize(i + 1);
		this->points[i] = point;
		nonSkippedPoint++; 

		// draw on image
		this->paintMarkerOnPoint(i); 
	}

	return nonSkippedPoint;
}

int ImagePointsPicker::pickTemplates(int nPoint, int bKey, int cKey1, int cKey2)
{
	int nonSkippedPoint = 0;
	this->nPoint = nPoint;

	// Read background photo and refresh marked image
	this->refreshMarkedImage();

	// Pick points in a for-loop
	this->points = vector<cv::Point2f>(this->nPoint, cv::Point2f(_nanf, _nanf));
	for (int i = 0; i < this->nPoint; i++) {
		char strbuf[10000];
        snprintf(strbuf, 10000, "Pick Point %d of %d", i + 1, this->nPoint);
		std::string figtitle = strbuf;
		cv::Point2f point(_nanf, _nanf);
		cv::Rect tmpltRect(0, 0, 0, 0); 
		// pick
		int key = pickATemplate(figtitle, this->img_marked, point, tmpltRect, bKey, cKey1, cKey2);
		// check 
		if (key == bKey) {
			continue;
		}
		// add to points array
		if (this->points.size() < i + 1)
			this->points.resize(i + 1);
		this->points[i] = point;
		// add to rois array
		if (this->tmpltRects.size() < i + 1)
			this->tmpltRects.resize(i + 1);
		this->tmpltRects[i] = tmpltRect;
		// Check if the point is within roi
		if (!(point.x >= tmpltRect.x - 0.5f && point.x <= tmpltRect.x - 0.5f + tmpltRect.width &&
			point.y >= tmpltRect.y - 0.5f && point.y <= tmpltRect.y - 0.5f + tmpltRect.height)) {
			cerr << "Point " << i + 1 << " (1-base) is not within its range (rect). "
				<< "Do you want to re-pick again?"
				<< "  ('y' or 'Y': Yes. I want to re-pick the template (point + rect).)\n"
				<< "  ('n' or else: No, I know what I am doing. Just go ahead.)\n";
			std::string repick; cin >> repick;
			if (repick.size() > 0 && (repick[0] == 'y' || repick[0] == 'Y')) {
				i--; continue;
			}
		}
		nonSkippedPoint++;
		// draw on image
		this->paintMarkerOnPoint(i);
	}

	return nonSkippedPoint;
}



int ImagePointsPicker::writeMarkersToImageFile(std::string fname, int draw_type, int mark_size)
{
	// output file name
	this->imgfile_marked = imgfile.substr(0, imgfile.size() - 4) + "_marked.JPG";

	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->imgfile_marked;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->imgfile_marked) + fname;

	// write markers to file
	this->refreshMarkedImage();
	for (int i = 0; i < this->points.size(); i++)
		this->paintMarkerOnPoint(i, draw_type, mark_size);
	cv::imwrite(fname, this->img_marked);
	return 0;
}

int ImagePointsPicker::writeTemplateMarkersToImageFile(std::string fname, int draw_type, int mark_size)
{
	// output file name
	this->imgfile_marked = imgfile.substr(0, imgfile.size() - 4) + "_TmpltMarked.JPG";

	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->imgfile_marked;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->imgfile_marked) + fname;

	// write markers to file
	this->refreshMarkedImage();
	for (int i = 0; i < this->points.size(); i++)
		this->paintTemplateMarkerOnPoint(i, draw_type, mark_size);
	cv::imwrite(fname, this->img_marked);
	return 0;
}

int ImagePointsPicker::pickQ4Templates(int nPoint12, int nPoint23, int bKey, int cKey1, int cKey2)
{
	// pick 4 points
	this->pickTemplates(4, bKey, cKey1, cKey2); 

	// interpolation points
	vector<cv::Point2f> p4 = this->points;
	vector<cv::Rect> p4tm = this->tmpltRects;
	vector<cv::Point2f> q4 = interpQ4(p4, nPoint12, nPoint23);
	vector<cv::Rect> q4tm(q4.size()); 

	// interpolate template size
	for (int i = 0; i < nPoint23; i++)
	{
		float fact1, fact2;
		fact2 = i * 1.f / (nPoint23 - 1); // from 0.0 to 1.0
		fact1 = 1.f - fact2; // from 1.0 to 0.0
		cv::Rect q4tmL, q4tmR; 
		q4tmL.x = (int)(fact1 * p4tm[0].x + fact2 * p4tm[3].x + .5); 
		q4tmL.y = (int)(fact1 * p4tm[0].y + fact2 * p4tm[3].y + .5);
		q4tmL.width = (int)(fact1 * p4tm[0].width + fact2 * p4tm[3].width + .5);
		q4tmL.height = (int)(fact1 * p4tm[0].height + fact2 * p4tm[3].height + .5);
		q4tmR.x = (int)(fact1 * p4tm[1].x + fact2 * p4tm[2].x + .5);
		q4tmR.y = (int)(fact1 * p4tm[1].y + fact2 * p4tm[2].y + .5);
		q4tmR.width = (int)(fact1 * p4tm[1].width + fact2 * p4tm[2].width + .5);
		q4tmR.height = (int)(fact1 * p4tm[1].height + fact2 * p4tm[2].height + .5);
		for (int j = 0; j < nPoint12; j++)
		{
			fact2 = j * 1.f / (nPoint12 - 1); // from 0.0 to 1.0
			fact1 = 1.f - fact2; // from 1.0 to 0.0
			//q4tm[j + i * nPoint12].x      = (int)(fact1 * q4tmL.x      + fact2 * q4tmR.x      + .5);
			//q4tm[j + i * nPoint12].y      = (int)(fact1 * q4tmL.y      + fact2 * q4tmR.y      + .5);
			q4tm[j + i * nPoint12].width  = (int)(fact1 * q4tmL.width  + fact2 * q4tmR.width  + .5);
			q4tm[j + i * nPoint12].height = (int)(fact1 * q4tmL.height + fact2 * q4tmR.height + .5);
	 		q4tm[j + i * nPoint12].x = (int)(q4[j + i * nPoint12].x - q4tm[j + i * nPoint12].width / 2. + .5);
	 		q4tm[j + i * nPoint12].y = (int)(q4[j + i * nPoint12].y - q4tm[j + i * nPoint12].height / 2. + .5);
			if (q4tm[j + i * nPoint12].x < 0) q4tm[j + i * nPoint12].x = 0;
			if (q4tm[j + i * nPoint12].y < 0) q4tm[j + i * nPoint12].y = 0; 
		}
	}

	// update points data 
	this->points = q4; 
	this->tmpltRects = q4tm; 
	this->nPoint = nPoint12 * nPoint23; 
	return 0;
}

int ImagePointsPicker::generateQ4Templates(int nPoint12, int nPoint23)
{
	// check if there are exactly four points
	if (this->points.size() != 4)
	{
		cerr << "ImagePointsPicker::generateQ4Templates() only works when number of points is 4.\n";
		cerr << "  But the number of points is " << this->points.size() << endl;
		return -1; 
	}

	// interpolation points
	vector<cv::Point2f> p4 = this->points;
	vector<cv::Point2f> q4 = interpQ4(p4, nPoint12, nPoint23);
	vector<cv::Rect> p4tm; 
	vector<cv::Rect> q4tm; 
	if (this->tmpltRects.size() == 4) {
		p4tm = this->tmpltRects;
		q4tm.resize(q4.size());
		// interpolate template size
		for (int i = 0; i < nPoint23; i++)
		{
			float fact1, fact2;
			fact2 = i * 1.f / (nPoint23 - 1); // from 0.0 to 1.0
			fact1 = 1.f - fact2; // from 1.0 to 0.0
			cv::Rect q4tmL, q4tmR;
			q4tmL.x = (int)(fact1 * p4tm[0].x + fact2 * p4tm[3].x + .5);
			q4tmL.y = (int)(fact1 * p4tm[0].y + fact2 * p4tm[3].y + .5);
			q4tmL.width = (int)(fact1 * p4tm[0].width + fact2 * p4tm[3].width + .5);
			q4tmL.height = (int)(fact1 * p4tm[0].height + fact2 * p4tm[3].height + .5);
			q4tmR.x = (int)(fact1 * p4tm[1].x + fact2 * p4tm[2].x + .5);
			q4tmR.y = (int)(fact1 * p4tm[1].y + fact2 * p4tm[2].y + .5);
			q4tmR.width = (int)(fact1 * p4tm[1].width + fact2 * p4tm[2].width + .5);
			q4tmR.height = (int)(fact1 * p4tm[1].height + fact2 * p4tm[2].height + .5);
			for (int j = 0; j < nPoint12; j++)
			{
				fact2 = j * 1.f / (nPoint12 - 1); // from 0.0 to 1.0
				fact1 = 1.f - fact2; // from 1.0 to 0.0
				//q4tm[j + i * nPoint12].x      = (int)(fact1 * q4tmL.x      + fact2 * q4tmR.x      + .5);
				//q4tm[j + i * nPoint12].y      = (int)(fact1 * q4tmL.y      + fact2 * q4tmR.y      + .5);
				q4tm[j + i * nPoint12].width = (int)(fact1 * q4tmL.width + fact2 * q4tmR.width + .5);
				q4tm[j + i * nPoint12].height = (int)(fact1 * q4tmL.height + fact2 * q4tmR.height + .5);
				q4tm[j + i * nPoint12].x = (int)(q4[j + i * nPoint12].x - q4tm[j + i * nPoint12].width / 2. + .5);
				q4tm[j + i * nPoint12].y = (int)(q4[j + i * nPoint12].y - q4tm[j + i * nPoint12].height / 2. + .5);
				if (q4tm[j + i * nPoint12].x < 0) q4tm[j + i * nPoint12].x = 0;
				if (q4tm[j + i * nPoint12].y < 0) q4tm[j + i * nPoint12].y = 0;
			}
		}
	} // end of if there are 4 templates

	// update points data 
	this->points = q4;
	this->nPoint = nPoint12 * nPoint23;
	if (this->tmpltRects.size() == 4) 
		this->tmpltRects = q4tm;
	return 0;
}


int ImagePointsPicker::num_points()
{
	return (int) this->points.size();
}

vector<cv::Point2f> ImagePointsPicker::pointsInPoint2fVector() const
{
	return points; 
}

cv::Mat ImagePointsPicker::pointsIn1xNCvMat() const
{
	cv::Mat pointsMat(1, this->nPoint, CV_32FC2);
	memcpy(pointsMat.data, this->points.data(), this->points.size() * sizeof(cv::Point2f)); 
	return pointsMat;
}

vector<cv::Rect> ImagePointsPicker::rectsInRectVector() const
{
	return tmpltRects;
}

int ImagePointsPicker::writePointsToTxtFile(std::string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->txtfile;
	else if(directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->txtfile) + fname; 

	if (this->points.size() > 0) {
		ofstream ofs(fname);
		for (int i = 0; i < this->points.size(); i++) {
			ofs << this->points[i].x << " " << this->points[i].y << endl;
		}
		ofs.close();
	}
	cout << "Saved points data at " << fname << endl;
	return 0;
}

int ImagePointsPicker::writeTemplatesToTxtFile(std::string fname)
{
	char buf[1000]; 
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = extFilenameRemoved(this->imgfile) + "_pickedTemplates.txt"; 
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->imgfile) + fname;

	if (this->points.size() > 0) {
		ofstream ofs(fname);
		for (int i = 0; i < this->points.size(); i++) {
			snprintf(buf, 1000, " %10.3f \t %10.3f \t %6d \t %6d \t %4d \t %4d \n",
				this->points[i].x, this->points[i].y,
				this->tmpltRects[i].x, this->tmpltRects[i].y,
				this->tmpltRects[i].width,
				this->tmpltRects[i].height); 
			ofs << buf; 
//			ofs << this->points[i].x << " " << this->points[i].y << " "
//				<< this->tmpltRects[i].x << " " << this->tmpltRects[i].y << " "
//				<< this->tmpltRects[i].width << " "
//				<< this->tmpltRects[i].height << endl;
		}
		ofs.close();
	}
	cout << "Saved points data at " << fname << endl;
	return 0;
}

int ImagePointsPicker::writePointsToXmlFile(std::string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->xmlfile;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->xmlfile) + fname;

	if (this->points.size() > 0) {
		FileStorage ofs(fname, cv::FileStorage::Mode::WRITE);
		ofs << "VecPoint2f" << this->points;
		ofs << "VecRect" << this->tmpltRects;
		ofs.release();
	}
	cout << "Saved points data at " << fname << endl;
	return 0;
}

int ImagePointsPicker::writeTemplatesToXmlFile(std::string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = extFilenameRemoved(this->imgfile) + "_pickedTemplates.xml";
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->imgfile) + fname;

	if (this->points.size() > 0) {
		vector<int> motionType(this->points.size(), 0); 
		vector<int> searchSizeX(this->points.size());
		vector<int> searchSizeY(this->points.size());
		for (int i = 0; i < this->points.size(); i++) {
			searchSizeX[i] = this->tmpltRects[i].width;
			searchSizeY[i] = this->tmpltRects[i].height;
		}
		FileStorage ofs(fname, cv::FileStorage::Mode::WRITE);
		ofs << "VecPoint2f" << this->points;
		ofs << "VecRect" << this->tmpltRects;
		ofs << "VecMotionType" << motionType;
		ofs << "VecSearchSizeX" << searchSizeX;
		ofs << "VecSearchSizeY" << searchSizeY;
		ofs.release();
	}
	cout << "Saved points data at " << fname << endl;
	return 0;
}

bool ImagePointsPicker::existMarkersImageFile()
{
	// image file name
	this->imgfile_marked = imgfile.substr(0, imgfile.size() - 4) + "_marked.JPG";
	// try to open file
	std::ifstream ifs(this->imgfile_marked, ios::binary); 
	if (ifs.is_open() == true) {
		ifs.close();
		return true;
	}
	return false;
}

bool ImagePointsPicker::existPointsTxtFile(string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->txtfile;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->txtfile) + fname;

	// try to open file
	std::ifstream ifs(fname);
	if (ifs.is_open() == true) {
		ifs.close();
		return true;
	}
	return false;
}

int ImagePointsPicker::existPointsXmlFile(string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->xmlfile;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->xmlfile) + fname;

	// try to open file
	std::ifstream ifs(fname);
	if (ifs.is_open() == true) {
		ifs.close();
		return true;
	}
	return false;
}

int ImagePointsPicker::readPointsFromTxtFile(std::string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->txtfile;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->txtfile) + fname;

	if (this->existPointsTxtFile(fname) == false)
		return -1;
	ifstream ifs(fname); 
	this->points.clear();
	while (true) {
		cv::Point2f p; 
		ifs >> p.x >> p.y;
		if (ifs.eof()) break;
		this->points.push_back(p); 
	}
	ifs.close(); 
	return 0;
}

int ImagePointsPicker::readPointsFromXmlFile(string fname)
{
	// if fname is given, use it. Or use default file name
	if (fname.length() <= 0)
		fname = this->xmlfile;
	else if (directoryOfFullPathFile(fname).length() == 0)
		fname = directoryOfFullPathFile(this->xmlfile) + fname;

	if (this->existPointsXmlFile(fname) == false)
		return -1;
	FileStorage ifs(fname, cv::FileStorage::Mode::READ); 
	ifs["VecPoint2f"] >> this->points;
	ifs["VecRect"] >> this->tmpltRects;
	ifs.release(); 
	return 0;
}

int ImagePointsPicker::refreshMarkedImage()
{
	// Read image
//	if (this->imgfile.length() > 0) {
		this->img = cv::imread(imgfile);
		if (img.empty()) {
			std::cout << "Cannot read image from file " << imgfile << endl;
			system("pause");
			return -1;
		}
		this->img.copyTo(this->img_marked);
//	}
	return 0;
}

int ImagePointsPicker::paintMarkerOnPoint(int iPoint, int draw_type, int mark_size)
{
	// Check
	if (iPoint >= this->points.size())
		return -1;
	if (this->img_marked.rows <= 0 || this->img_marked.cols <= 0)
		this->refreshMarkedImage();
	if (isnan(this->points[iPoint].x) || isnan(this->points[iPoint].y))
		return 0; 
	if (this->points[iPoint].x < 0 ||
		this->points[iPoint].x >= this->img_marked.cols ||
		this->points[iPoint].y < 0 ||
		this->points[iPoint].y >= this->img_marked.rows)
		return 0;

	// Mark a number on img_marked
	cv::Mat pimg_mrk_tmp;
	this->img_marked.copyTo(pimg_mrk_tmp);
	int shift = 6; // for drawing sub-pixel accuracy 
	if (draw_type == 1) {
		int radius = mark_size / 2 * (1 << shift);
		int thickness = mark_size / 6;
		cv::Point pi((int)(this->points[iPoint].x * (1 << shift) + 0.5f),
			(int)(this->points[iPoint].y * (1 << shift) + 0.5f));
		cv::circle(pimg_mrk_tmp, pi, radius, Scalar(0, 0, 255), thickness, LINE_8, 6);
	}
	else if (draw_type == 2) {
		int cross_size = mark_size;
		int thickness = mark_size / 6;
		cv::Point pi, pj;
		pi = cv::Point((int)(this->points[iPoint].x * (1 << shift) + 0.5f),
			(int)(this->points[iPoint].y * (1 << shift) + 0.5f));
		pj = cv::Point((int)((this->points[iPoint].x + cross_size / 2) * (1 << shift) + 0.5f),
			(int)(this->points[iPoint].y * (1 << shift) + 0.5f));
		cv::line(pimg_mrk_tmp, pi, pj, Scalar(0, 0, 255), thickness, LINE_8, 6);
		pj = cv::Point((int)((this->points[iPoint].x - cross_size / 2) * (1 << shift) + 0.5f),
			(int)(this->points[iPoint].y * (1 << shift) + 0.5f));
		cv::line(pimg_mrk_tmp, pi, pj, Scalar(0, 0, 255), thickness, LINE_8, 6);
		pj = cv::Point((int)(this->points[iPoint].x * (1 << shift) + 0.5f),
			(int)((this->points[iPoint].y + cross_size / 2) * (1 << shift) + 0.5f));
		cv::line(pimg_mrk_tmp, pi, pj, Scalar(0, 0, 255), thickness, LINE_8, 6);
		pj = cv::Point((int)(this->points[iPoint].x * (1 << shift) + 0.5f),
			(int)((this->points[iPoint].y - cross_size / 2) * (1 << shift) + 0.5f));
		cv::line(pimg_mrk_tmp, pi, pj, Scalar(0, 0, 255), thickness, LINE_8, 6);
	}
	// add text 
	char strbuf[100];
	int fontFace = FONT_HERSHEY_DUPLEX;
	double fontScale = mark_size / 8.0;
	cv::Scalar color = Scalar(0, 0, 255); // Blue, Green, Red
	int thickness = mark_size / 6;
	int lineType = 8;
	bool bottomLeftOri = false;
    snprintf(strbuf, 100, "%d", iPoint + 1);
	cv::putText(pimg_mrk_tmp, string(strbuf),
		cv::Point((int)(this->points[iPoint].x + 0.5f), (int)(this->points[iPoint].y + 0.5f)),
		fontFace, fontScale, color, thickness, lineType, bottomLeftOri);
	// do transparency
	this->img_marked = this->img_marked / 2 + pimg_mrk_tmp / 2;
	
	return 0;
}

int ImagePointsPicker::paintTemplateMarkerOnPoint(int iPoint, int draw_type, int mark_size)
{
	// Check
	if (iPoint >= this->points.size())
		return -1;
	if (this->img_marked.rows <= 0 || this->img_marked.cols <= 0)
		this->refreshMarkedImage();
	if (isnan(this->points[iPoint].x) || isnan(this->points[iPoint].y))
		return 0;
	if (this->points[iPoint].x < 0 ||
		this->points[iPoint].x >= this->img_marked.cols ||
		this->points[iPoint].y < 0 ||
		this->points[iPoint].y >= this->img_marked.rows)
		return 0;

	this->paintMarkerOnPoint(iPoint, draw_type, mark_size);

	// Mark a number on img_marked
	cv::Mat pimg_mrk_tmp;
	this->img_marked.copyTo(pimg_mrk_tmp);
	int thickness = mark_size / 6;
	cv::rectangle(pimg_mrk_tmp, this->tmpltRects[iPoint], Scalar(0, 0, 255), thickness, LINE_8);

	// do transparency
	this->img_marked = this->img_marked / 2 + pimg_mrk_tmp / 2;

	return 0;
}

cv::Size ImagePointsPicker::imgSize()
{
	return this->img.size();
}
