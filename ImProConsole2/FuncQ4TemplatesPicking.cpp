#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "ImagePointsPicker.h"
#include "impro_util.h"

using namespace std;
using namespace cv; 

// std::vector<cv::Point2f> interpQ4(const std::vector<cv::Point2f> & inPoints, int n12, int n23)

const String keys =
"{help h usage ? |      | print this message   }"
"{image img      |      | background image file }"
"{n12 npoint12   |      | number of points along line between (including) points 1 and 2 (1-2-3-4 clock-or-counter-clock wise)}"
"{n23 npoint12   |      | number of points along line between (including) points 2 and 3 (1-2-3-4 clock-or-counter-clock wise)}"
"{outTxtFile txt |      | output filename in txt (with .txt) } "
"{outXmlFile xml |      | output filename in xml (with .xml) } "
"{outMarkedFile marked |      | output filename of marked image (with .JPG) } "
;


int FuncQ4TemplatesPicking(int argc, char ** argv)
{
	// Variables declaration
	ImagePointsPicker pp;
	bool ui = false;
	int setImgOk = -1;
	int nPoint12 = 0;
	int nPoint23 = 0;
	string outTxtFile;
	string outXmlFile;
	string outMarkedFile;

	cv::CommandLineParser * pparser = NULL;
	if (argc >= 1) pparser = new cv::CommandLineParser(argc, argv, keys);

	if (argc == 0) {
		ui = true;
	}
	else if (argc == 1 || (*pparser).has("help")) {
		(*pparser).printMessage();
		cout << "\nUsage: \n"
			<< " [executable.exe] -img=theImage.JPG -n12=10 -n23=10 -xml=theImage_q4PickedTemplates.xml -marked=theImage_q4marked.JPG \n";
		cout << "Enter i for interaction. Otherwise to quit: ";
		string str_interaction;
		str_interaction = readStringFromCin();
		if (str_interaction[0] == 'i')
			ui = true;
		else
			return 0;
	}

	// Set background image
	if (argc >= 1 && (*pparser).has("image")) {
		setImgOk = pp.setBackgroundImageByFilename((*pparser).get<string>("image"));
	}
	else {
		string fname_img;
		cout << "Enter background image file ('g' for ui file dialog): ";
		fname_img = readStringFromCin();
		if (fname_img.length() == 1 && fname_img[0] == 'g')
			fname_img = uigetfile();
		setImgOk = pp.setBackgroundImageByFilename(fname_img);
		//		setImgOk = pp.setBackgroundImageByFilename("..\\test_images\\CalibGrid01.JPG");
	}
	if (setImgOk != 0) {
		cout << "Failed to set background image.\n";
		return -1;
	}

	// number of points to pick
	if (argc >= 1 && (*pparser).has("n12")) {
		nPoint12 = (*pparser).get<int>("n12");
	}
	else if (ui) {
		cout << "Number of points to generate along points 1 and 2: ";
		nPoint12 = readIntFromCin();
		if (nPoint12 <= 0) {
			cout << "Invalid number of points.\n";
			return -1;
		}
	}
	else {
		nPoint12 = 2;
	}
	if (argc >= 1 && (*pparser).has("n23")) {
		nPoint23 = (*pparser).get<int>("n23");
	}
	else if (ui) {
		cout << "Number of points to generate along points 2 and 3: ";
		nPoint23 = readIntFromCin();
		if (nPoint23 <= 0) {
			cout << "Invalid number of points.\n";
			return -1;
		}
	}
	else {
		nPoint23 = 2;
	}
	//	pp.pickPoints(nPoint);
//	pp.pickTemplates(4 /* four corners */, 27 /*ESC*/, 13 /*SPACE*/, 32 /*ENTER*/);
	pp.pickQ4Templates(nPoint12, nPoint23, 27 /*ESC*/, 13 /*SPACE*/, 32 /*ENTER*/);


	// Write to files (cv::FileStorage format)
	if (argc >= 1 && (*pparser).has("outTxtFile"))
		outTxtFile = (*pparser).get<string>("outTxtFile");
	else if (ui) {
		cout << "Enter output text file (.txt) ('g' for ui file dialog): " << endl;
		outTxtFile = readStringFromCin();
		if (outTxtFile.length() == 1 && outTxtFile[0] == 'g')
			outTxtFile = uiputfile();
	}
	else
		outTxtFile = string("");

	if (argc >= 1 && (*pparser).has("outXmlFile"))
		outXmlFile = (*pparser).get<string>("outXmlFile");
	else if (ui) {
		cout << "Enter output XML file (.xml) ('g' for ui file dialog): " << endl;
		outXmlFile = readStringFromCin();
		if (outXmlFile.length() == 1 && outXmlFile[0] == 'g')
			outXmlFile = uiputfile();
	}
	else
		outXmlFile = string("");

	if (argc >= 1 && (*pparser).has("outMarkedFile"))
		outMarkedFile = (*pparser).get<string>("outMarkedFile");
	else if (ui) {
		cout << "Enter output marked JPG file (.JPG) ('g' for ui file dialog): " << endl;
		outMarkedFile = readStringFromCin();
		if (outMarkedFile.length() == 1 && outMarkedFile[0] == 'g')
			outMarkedFile = uiputfile();
	}
	else
		outMarkedFile = string("");

	pp.writeTemplateMarkersToImageFile(outMarkedFile);
	pp.writeMarkersToImageFile(outMarkedFile + ".onlyPoints.JPG"); 
	pp.writeTemplatesToTxtFile(outTxtFile);
	pp.writeTemplatesToXmlFile(outXmlFile);

	return 0;	return 0;
}